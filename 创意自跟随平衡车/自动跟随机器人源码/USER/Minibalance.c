/************************************************
名称：自动跟随机器人
作者：张巧龙
开发环境：MDK5
单片机型号：STM32
微信公众号【大鱼机器人】
微信回复关键字【电子开发工具】获取电子设计必备软件
微信回复关键字【电子设计资料】获取电子设计资料包
微信回复关键字【PS学习教程】  获取经典PS学习教程
微信回复关键字【更多资源获取】获取更多精彩资源哦~

知乎【张巧龙】https://www.zhihu.com/people/zhang-qiao-long/activities
新浪博客【张巧龙】:http://blog.sina.com.cn/u/3829139600

技术交流学习请加微信：best_xiaolong或者加QQ:746175735
备注【进群】拉你进交流群~
************************************************/

#include "sys.h"
#include "stm32f10x.h"
#include "timer.h"
#include "encode.h"
#include "mpu6050.h"
#include "pwm.h"
#include "usart.h"	
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "motor.h"
#include "adc.h"

float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;			//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;								//温度	  

int Encoder_Left,Encoder_Right; //电机左右编码器值 采用正交解码

int Moto1,Moto2; 								//电机左右编码器值 采用正交解码


u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=1; //蓝牙遥控相关的变量


unsigned char   dat1[3],dat2[3];												//超声波相关的变量
unsigned char   num1,num2;
int             distance1=0,distance2=0;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;									//PID算出的电机PWM相关的变量

int Voltage; 																						//adc测电压

u8 mode,key;

int main(void)
{  
	
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	NVIC_init();										//=====初始化中断优先级
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	MPU_Init();					    				//=====初始化MPU6050
	mpu_dmp_init();									//=====初始化MPU6050DMP模式  
	OLED_Init(); 										//=====初始化OLED	
	while(mpu_dmp_init())						//=====初始化正常 返回值为0 其他情况则不正常
	{
		OLED_ShowString(0,0,"MPU6050 ERROR");
		OLED_Refresh_Gram();	
		Moto1=0,Moto2=0;
	}
	OLED_ShowString(0,0,"MPU6050 OK");
	OLED_Refresh_Gram();
	TIM1_PWM_Init(7199,0);					//=====10Khz PWM
	Encoder_Init_TIM2();            //=====初始化编码器2
  Encoder_Init_TIM4();            //=====初始化编码器4
  KEY_Init_MY();                  //=====按键初始化
  uart1_init(9600);            		//=====初始化串口1
  uart2_init(115200);							//=====初始化串口2
	uart3_init(115200);         		//=====初始化串口3
	Motor_Init();										//=====初始化电机IO口
	Adc_Init();                     //=====adc初始化			
	Voltage=Get_battery_volt();     //=====获取电池电压	
	
	OLED_ShowString(00,40,"Volta");
	OLED_ShowString(58,40,".");
	OLED_ShowString(80,40,"V");
	OLED_ShowNumber(45,40,Voltage/100,2,12);
	OLED_ShowNumber(68,40,Voltage%100,2,12);
	if(Voltage%100<10) 	OLED_ShowNumber(62,40,0,2,12);
	
	while(1)///////////在该部分确定进入哪个模式!
	{
		key=KEY_Scan(0);
		if(key==1)			//模式1：自动跟随
		{
			mode=1;
			OLED_ShowString(0,20,"MODE:Auto Follow");	
			OLED_Refresh_Gram();			
			break;
		}else if(key==2)//模式2：蓝牙控制
		{
			mode=2;
			OLED_ShowString(0,20,"MODE: Bluetooth");
			OLED_Refresh_Gram();
			break;
		}
		delay_ms(5);			
	}	
	Motor_Stop();    								//=====初始化电机IO口
	TIM3_Int_Init(99,7199); 				//=====控制周期10ms
	Set_Pwm(0,0);
	while(1)
	{ 		
			if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
			{
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			}
	} 
}

