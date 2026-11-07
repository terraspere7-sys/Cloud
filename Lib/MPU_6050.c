#include "Timer.h"
#include "SI2C_MPU.h"
#include "MPU_6050.h"
#include "USART.h"
#include "math.h"
#include "Servo.h"

#define MPU6050_ADDR 0xD0
#define SERVO_CENTER       90.0f
#define SERVO_MIN           0.0f
#define SERVO_MAX         180.0f

#define FILTER_ALPHA       0.3f
#define SERVO_SPEED        50.0f   // 每秒最多转动50度

static float pitch_filter = 0.0f;
static float roll_filter  = 0.0f;
//用来存储低通滤波后的角度
static float pitch_target = SERVO_CENTER;
static float roll_target  = SERVO_CENTER;
//用来存储舵机最终的目标角度
static float pitch_servo = SERVO_CENTER;
static float roll_servo  = SERVO_CENTER;
//用来存储输入给舵机的平滑角度
static uint8_t filter_initialized = 0;
static uint32_t last_tick = 0;

static float ax,ay,az;
static float gx,gy,gz;
static float gx_bias,gy_bias,gz_bias;
static float yaw,pitch,roll;



static uint8_t cal_state = MPU_CAL_IDLE;
static uint16_t cal_count = 0;
static uint32_t cal_next_tick = 0;

static float sum_gx = 0.0f;
static float sum_gy = 0.0f;
static float sum_gz = 0.0f;

static float sum_gx2 = 0.0f;
static float sum_gy2 = 0.0f;
static float sum_gz2 = 0.0f;
static float sum_acc_norm;


float yaw_g;
float pitch_g;
float roll_g;

void mpu_update(void);

void reg_write(uint8_t reg, uint8_t value)
{
    sendstart_mpu();

    sendbyte_mpu(MPU6050_ADDR);  // 发送写地址 0xD0
    sendbyte_mpu(reg);           // 发送寄存器地址
    sendbyte_mpu(value);         // 发送寄存器数据

    sendstop_mpu();
}

uint8_t reg_read(uint8_t reg)
{
    uint8_t value;

    sendstart_mpu();

    sendbyte_mpu(MPU6050_ADDR);      // 写地址 0xD0
    sendbyte_mpu(reg);               // 指定要读取的寄存器

    sendstart_mpu();                 // 重复起始
    sendbyte_mpu(MPU6050_ADDR | 1);  // 读地址 0xD1

    value = receivebyte_mpu(0);      // 读一个字节，最后发送 NACK

    sendstop_mpu();

    return value;
}
void mpu_update_cal(void){
	uint8_t ax_h = reg_read(0x3B);
	uint8_t ax_l = reg_read(0x3C);

	uint8_t ay_h = reg_read(0x3D);
	uint8_t ay_l = reg_read(0x3E);

	uint8_t az_h = reg_read(0x3F);
	uint8_t az_l = reg_read(0x40);

	int16_t ax_raw = (int16_t)(((uint16_t)ax_h << 8) | ax_l);
	int16_t ay_raw = (int16_t)(((uint16_t)ay_h << 8) | ay_l);
	int16_t az_raw = (int16_t)(((uint16_t)az_h << 8) | az_l);

	ax = (float)ax_raw / 16384.0f;
	ay = (float)ay_raw / 16384.0f;
	az = (float)az_raw / 16384.0f;
	
	int16_t temperture_raw = (int16_t)(reg_read(0x41) << 8) + reg_read(0x42);
	
	
	uint8_t gx_h = reg_read(0x43);
	uint8_t gx_l = reg_read(0x44);

	uint8_t gy_h = reg_read(0x45);
	uint8_t gy_l = reg_read(0x46);

	uint8_t gz_h = reg_read(0x47);
	uint8_t gz_l = reg_read(0x48);

	int16_t gx_raw = (int16_t)(((uint16_t)gx_h << 8) | gx_l);
	int16_t gy_raw = (int16_t)(((uint16_t)gy_h << 8) | gy_l);
	int16_t gz_raw = (int16_t)(((uint16_t)gz_h << 8) | gz_l);

	gx = (float)gx_raw / 65.5f;
	gy = (float)gy_raw / 65.5f;
	gz = (float)gz_raw / 65.5f;
}

void MPU_INIT(void){
	
	reg_write(0x6b,0x80);
	Delay(100);
	
	reg_write(0x6b,0x00);
	
	reg_write(0x1b,0x08);
	reg_write(0x1c,0x00);
}
	
uint8_t MPU_CalibrateStart(void)
{
    if (cal_state == MPU_CAL_RUNNING)
    {
        return 0;
    }

    cal_count = 0;

    sum_gx = 0.0f;
    sum_gy = 0.0f;
    sum_gz = 0.0f;

    sum_gx2 = 0.0f;
    sum_gy2 = 0.0f;
    sum_gz2 = 0.0f;

		sum_acc_norm = 0.0f;

    cal_next_tick = GetTick();
    cal_state = MPU_CAL_RUNNING;

    return 1;
}

void MPU_CalibrateTask(uint16_t sample){
	
	float acc_norm;
	
	if(cal_state != MPU_CAL_RUNNING){
		return;
	}
	
	if((int32_t)(GetTick() - cal_next_tick) < 0){
		return;
	}
	
	cal_next_tick = GetTick() + 5;
	
	mpu_update_cal();
	
	acc_norm = sqrtf(ax * ax + ay * ay + az * az);
	//计算三轴加速度向量和的模长，正常情况下应该接近1g，若出现明显不合理数据可以通过这个变量的计算值判断
	
    if (acc_norm < 0.5f || acc_norm > 1.5f)
    {
      cal_state = MPU_CAL_FAILED;
			USART1_SendString("ERR CAL");
      return;
    }
		
		sum_acc_norm += acc_norm;
		
		sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;


    sum_gx2 += gx * gx;
    sum_gy2 += gy * gy;
    sum_gz2 += gz * gz;
//将加速度计的读值的平方累加，用来后续计算方差来评估校准时是否出现明显抖动
		
    cal_count++;
		
		if (cal_count < sample){
			return;
		}
		
		{
			float mean_gx = sum_gx / sample;
      float mean_gy = sum_gy / sample;
      float mean_gz = sum_gz / sample;
			
			float var_gx = sum_gx2 /sample - mean_gx*mean_gx;
			float var_gy = sum_gy2 /sample - mean_gy*mean_gy;
			float var_gz = sum_gz2 /sample - mean_gz*mean_gz;
			//计算角速度的方差，判断波动程度
			float mean_acc = sum_acc_norm /sample;
			
			/* 防止浮点计算误差造成微小负数 */
      if (var_gx < 0.0f) var_gx = 0.0f;
      if (var_gy < 0.0f) var_gy = 0.0f;
      if (var_gz < 0.0f) var_gz = 0.0f;						
			
			if(fabsf(mean_gx) > 4 ||fabsf(mean_gy) > 7 ||fabsf(mean_gz) > 2){
				cal_state = MPU_CAL_FAILED;
				USART1_SendString("ERR CAL");
				return;
			}
			//初步判断校准结果，如果平均角速度的绝对值已经达到了阈值以上，认为校准结果不准确
			
			if (var_gx > 4.0f ||var_gy > 4.0f ||var_gz > 4.0f ||mean_acc < 0.5f ||mean_acc > 1.5f){
				cal_state = MPU_CAL_FAILED;
				USART1_SendString("ERR CAL");
				return;
			}
			//二次判断校准结果，如果波动程度过大，也认为校准结果不准确；如果计算出的三轴加速度的向量和的模长和1g相差过大，认为出现异常
			//补充一点，目前推断由于硬件精度原因，三轴加速度的向量和的模长大概会在0.6左右
			
			
			//经过两次检验后没有问题的数据保存到零偏值
			gx_bias = mean_gx;
      gy_bias = mean_gy;
      gz_bias = mean_gz;
			
			yaw_g =0.0f;
			pitch_g = 0.0f;
			roll_g = 0.0f;
			
			yaw = 0.0f;
			pitch = 0.0f;
			roll = 0.0f;
			
			USART1_SendString("OK");
			
			cal_state = MPU_CAL_SUCCESS;
		}
}
uint8_t MPU_CalibrateResult(void)
{
    return cal_state;
}

void mpu_update(void){
	if(cal_state == MPU_CAL_RUNNING){
			return;
	}
	//保证在校准的时候不会有其他的地方会调用mpu
	
	uint8_t ax_h = reg_read(0x3B);
	uint8_t ax_l = reg_read(0x3C);

	uint8_t ay_h = reg_read(0x3D);
	uint8_t ay_l = reg_read(0x3E);

	uint8_t az_h = reg_read(0x3F);
	uint8_t az_l = reg_read(0x40);

	int16_t ax_raw = (int16_t)(((uint16_t)ax_h << 8) | ax_l);
	int16_t ay_raw = (int16_t)(((uint16_t)ay_h << 8) | ay_l);
	int16_t az_raw = (int16_t)(((uint16_t)az_h << 8) | az_l);

	ax = (float)ax_raw / 16384.0f;
	ay = (float)ay_raw / 16384.0f;
	az = (float)az_raw / 16384.0f;
	
	int16_t temperture_raw = (int16_t)(reg_read(0x41) << 8) + reg_read(0x42);
	
	
	uint8_t gx_h = reg_read(0x43);
	uint8_t gx_l = reg_read(0x44);

	uint8_t gy_h = reg_read(0x45);
	uint8_t gy_l = reg_read(0x46);

	uint8_t gz_h = reg_read(0x47);
	uint8_t gz_l = reg_read(0x48);

	int16_t gx_raw = (int16_t)(((uint16_t)gx_h << 8) | gx_l);
	int16_t gy_raw = (int16_t)(((uint16_t)gy_h << 8) | gy_l);
	int16_t gz_raw = (int16_t)(((uint16_t)gz_h << 8) | gz_l);

gx = (float)gx_raw / 65.5f;
gy = (float)gy_raw / 65.5f;
gz = (float)gz_raw / 65.5f;
}

void Data_Process(void){
	
	mpu_update();
		
	yaw_g   = yaw_g   + (gz - gz_bias) * 0.015f;
	pitch_g = pitch_g + (gx - gx_bias) * 0.015f;
	roll_g  = roll_g  + (gy - gy_bias) * 0.015f;
	
	float pitch_a = atan2(ay,az) / 3.1415927f *180.0f;
	float roll_a = atan2(ax,az)/ 3.1415927f *180.0f;
	
	yaw = yaw_g *1.5;
	pitch = (0.95238 * pitch_g + (1 - 0.95238)* pitch_a)*1.5 ;
	roll =( 0.95238 * roll_g + (1 - 0.95238)* roll_a )*1.5;
	
	Delay(20);	
}
uint8_t MPU_ConnectCheck(void){
	if(reg_read(0x75) != 0x70){
		return 0;
	}
	else {
		return 1;
	}
}
float get_yaw(void){ return yaw;}
float get_pitch(void){ return pitch;}
float get_roll(void){return roll;}


/* 将角度限制在0~180度 */
static float Angle_Limit(float angle)
{
    if(angle < SERVO_MIN)
        angle = SERVO_MIN;

    if(angle > SERVO_MAX)
        angle = SERVO_MAX;

    return angle;
}

/* 平滑处理并更新舵机目标 */
void Gimbal_ControlTask(void)
{
    float pitch_now;
    float roll_now;
    float dt;
    float max_step;
    uint32_t now;

    now = GetTick();

    /* 第一次调用只记录时间 */
    if(last_tick == 0)
    {
        last_tick = now;
        return;
    }

    /* 实际经过的时间，单位为秒 */
    dt = (now - last_tick) / 1000.0f;

    /* 大间隔时避免一次更新跨越过大 */
    if(dt > 0.1f)
        dt = 0.1f;

    if(dt <= 0.0f)
        return;

    last_tick = now;

    /* 获取姿态解算结果 */
    pitch_now = get_pitch();
    roll_now  = get_roll();

    /* 第一次读取时初始化滤波值 */
    if(!filter_initialized)
    {
        pitch_filter = pitch_now;
        roll_filter = roll_now;
        filter_initialized = 1;
    }
    else
    {
        /* 一阶低通滤波 */
        pitch_filter += FILTER_ALPHA *
                        (pitch_now - pitch_filter);

        roll_filter += FILTER_ALPHA *
                       (roll_now - roll_filter);
    }

    /* 1:1角度映射，方向反了就将加号改为减号 */
    pitch_target = Angle_Limit(
                       SERVO_CENTER + pitch_filter);

    roll_target = Angle_Limit(
                      SERVO_CENTER + roll_filter);

    /* 限制本次舵机角度的最大变化量 */
    max_step = SERVO_SPEED * dt;

    if(pitch_target - pitch_servo > max_step)
        pitch_servo += max_step;
    else if(pitch_servo - pitch_target > max_step)
        pitch_servo -= max_step;
    else
        pitch_servo = pitch_target;

    if(roll_target - roll_servo > max_step)
        roll_servo += max_step;
    else if(roll_servo - roll_target > max_step)
        roll_servo -= max_step;
    else
        roll_servo = roll_target;

    Servo_SetAngle2 (pitch_servo);
		Servo_SetAngle1(roll_servo);
		
    
}
