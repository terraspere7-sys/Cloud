 #include "Command.h"
#include "USART.h"
#include "LED.h"
#include "OLED.h"
#include "Servo.h"
#include <string.h>
#include "MPU_6050.h"


#define CMD_BUF_SIZE 33

static char cmd_buffer[CMD_BUF_SIZE];
static uint8_t cmd_len = 0;
static uint8_t cmd_overlength = 0;

extern uint8_t mode;

/* 上一次指令的目标角度，初始化为中心位置 */
uint8_t yaw_target_angle = 90;
uint8_t pitch_target_angle = 90;

static void OLED_ShowCommandStatus(const char *status)
{
    strncpy(oled_cmd_status, status, sizeof(oled_cmd_status) - 1);
    oled_cmd_status[sizeof(oled_cmd_status) - 1] = '\0';

    if (oled_ok)
    {
        OLED_ClearLine(6);
        OLED_ShowString(6, 0, "CMD");
        OLED_ShowString(6, 36, oled_cmd_status);
    }
}

static void OLED_ShowAngle3(uint8_t page, uint8_t column, uint8_t angle)
{
	/* OLED的显示数字只支持一次显示一位，所以把三位数分开显示. */
    OLED_ShowChar(page, column, (char)('0' + (angle / 100U)));
    OLED_ShowChar(page, column + 6U, (char)('0' + ((angle / 10U) % 10U)));
    OLED_ShowChar(page, column + 12U, (char)('0' + (angle % 10U)));
}

void OLED_ShowAxisTargets(void)
{
    if (!oled_ok) return;

    OLED_ClearLine(1);
    OLED_ShowString(1, 0, "YAW");
    OLED_ShowAngle3(1, 36, yaw_target_angle);
    OLED_ClearLine(3);
    OLED_ShowString(3, 0, "PITCH");
    OLED_ShowAngle3(3, 36, pitch_target_angle);
}

static uint8_t ParseAngle(char *str, uint8_t *angle)
{
    uint16_t value = 0;

    if (*str == '\0') return 0;//如果第一位就是结束位那就结束判断

    while (*str != '\0')
    {
			if (*str < '0' || *str > '9') return 0;//如果参数不是0-9的数字，那么就直接返回0（参数形式错误）
        value = (uint16_t)(value * 10U + (uint16_t)(*str - '0'));//实现从字符到整型数据的转换
			if (value > 180U) return 0;//大于180认为是参数越界
        str++;
    }

    *angle = (uint8_t)value;//将解析出的参数存进变量
    return 1;
}

//cmd为读出的有效帧，key为目标指令（如YAW），angle用来存储有效的角度
static uint8_t ParseAxisCommand(char *cmd, const char *key, uint8_t *angle)
{
    uint8_t key_len = (uint8_t)strlen(key);//先计算一下目标指令的长度，用于后续指针移动
    char *p;

    if (strncmp(cmd, key, key_len) != 0) return 0;
	//用strncmp()函数来比价前len位和key存储的目标指令是不是一致
    if (cmd[key_len] != ' ') return 0;
	//检查目标指令后一位是不是格式规定的空格，排除YAWX这样子前三位正确的错误指令

    p = cmd + key_len;//另指针移动到格式目标指令末尾的空格
    while (*p == ' ') p++;//如果是空格就一直跳过直到遇到参数或者结束符
    if (*p == '\0') return 0;//空格跳过完了就读到了结束符说明没有参数，直接返回0
	//AI改写代码很喜欢加上局部作用域，貌似是一个好习惯，虽然这里不加局部作用域应该也是可以的
    {
			char *q = p;//另外定义一个指针q去检查参数后面是否有多余字符
			while (*q != '\0' && *q != ' ') q++;//如果不是结束符或者空格就一直一直检索下一位，保证可以越过当前参数
			if (*q != '\0') return 0; //参数后一位如果不是结束符，那么就认为存在冗余字符，判断为无效指令
    }

    return ParseAngle(p, angle);//这里p已经指到了参数位置的首位置
}

void Cmd_Process(void)
{
    if (rx_overflow)
    {
        __disable_irq();
        rx_read = rx_write;
        rx_overflow = 0;
			//舍弃尚未读取的数据来解决溢出情况
        __enable_irq();
			/*这里的“__disable_irq();”和“__enable_irq();”两行是AI建议我添加的代码
			目的是暂时关闭MPU对全局中断的响应，这样可以防止修改缓冲区变量时再次触发
			中断导致的数据上的冲突，是一个保障数据安全的设计*/
        cmd_len = 0;
        cmd_overlength = 0;
        USART1_SendString("ERR\r\n");
        OLED_ShowCommandStatus("ERR");
        return;
    }

    while (rx_read != rx_write)
    {
        uint8_t data = rx_buffer[rx_read];
        rx_read = (rx_read + 1) % RX_BUF_SIZE;

			if (data == '\r') continue;//忽略\R，直接不把它写进缓存区

        if (data == '\n')
        {
            if (cmd_len > 0 || cmd_overlength)
            {
                LED_Toggle(LED3);
                if (cmd_overlength)//过长报错
                {
                    USART1_SendString("ERR\r\n");
                    OLED_ShowCommandStatus("ERR");
                }
                else
                {
                    cmd_buffer[cmd_len] = '\0';
                    USART_ParseCommand(cmd_buffer);
                }
            }
            cmd_len = 0;
            cmd_overlength = 0;
        }
        else
        {
            if (cmd_len < CMD_BUF_SIZE - 1) cmd_buffer[cmd_len++] = (char)data;
            else cmd_overlength = 1;
        }
    }
}

void ToUpperString(char *str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
    {
        if (str[i] >= 'a' && str[i] <= 'z') str[i] = (char)(str[i] - 'a' + 'A');
        i++;
    }
}

void USART_ParseCommand(char *cmd)
{
    uint8_t angle;
    ToUpperString(cmd);

    if (strcmp(cmd, "AT") == 0)
    {
        USART1_SendString("OK\r\n");
        OLED_ShowCommandStatus("AT");
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        USART1_SendString("OK MODE=");
        USART1_SendByte((uint8_t)('0' + mode));
        USART1_SendString("\r\n");
        OLED_ShowCommandStatus("STATUS");
    }
		else if (strcmp(cmd, "CAL") == 0){
			if(MPU_ConnectCheck()){				
				OLED_ShowCommandStatus("CAL");
				MPU_CalibrateStart();
			}
			else{
				USART1_SendString("ERR CAL");
			}
		}
    else if (strncmp(cmd, "YAW", 3) == 0 && (cmd[3] == ' ' || cmd[3] == '\0'))
			//只有前三位是YAW且YAW后跟着的不是其他字符时才进入分支
    {
        if (!ParseAxisCommand(cmd, "YAW", &angle) || mode != 1)
        {
            USART1_SendString("ERR\r\n");
            OLED_ShowCommandStatus("ERR");
        }
        else
        {
            Servo_SetAngle1(angle);
            yaw_target_angle = angle;
            OLED_ShowAxisTargets();
            USART1_SendString("OK\r\n");
            OLED_ShowCommandStatus("YAW");
        }
    }
    else if (strncmp(cmd, "PITCH", 5) == 0 && (cmd[5] == ' ' || cmd[5] == '\0'))
    {
        if (!ParseAxisCommand(cmd, "PITCH", &angle) || mode != 1)
        {
            USART1_SendString("ERR\r\n");
            OLED_ShowCommandStatus("ERR");
        }
        else
        {
            Servo_SetAngle2(angle);
            pitch_target_angle = angle;
            OLED_ShowAxisTargets();
            USART1_SendString("OK\r\n");
            OLED_ShowCommandStatus("PITCH");
        }
    }
    else
    {
        USART1_SendString("ERR\r\n");
        OLED_ShowCommandStatus("ERR");
    }
}
