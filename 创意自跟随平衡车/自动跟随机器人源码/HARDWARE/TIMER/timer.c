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
#include "timer.h"
#include "encode.h" 
#include "motor.h"
#include "usart.h"	
#include "adc.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量

u8 jishi;

extern int Encoder_Left,Encoder_Right;
extern float pitch,roll,yaw; 		//欧拉角
extern short aacx,aacy,aacz;			//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern short temp;								//温度	
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern int distance1,distance2;
extern int Moto1,Moto2; 
extern int Voltage; 
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设							 
}

int TIM3_IRQHandler(void)   //TIM3中断
{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
			jishi++;
			if(jishi==500)
			{
				LED=!LED;
				jishi=0;
			}
			Encoder_Left =Read_Encoder(2);   											//===读取编码器的值
			Encoder_Right=Read_Encoder(4);   											//===读取编码器的值
			Balance_Pwm =balance(roll,gyrox); 								 		//===直立环PID控制	
			Velocity_Pwm=velocity(Encoder_Left,Encoder_Right); 		//===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
			Turn_Pwm    =turn(Encoder_Left,Encoder_Right,gyroz);  //===转向环PID控制     
			Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;           		//===计算左轮电机最终PWM
			Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;           		//===计算右轮电机最终PWM		
			Xianfu_Pwm();	
			if(roll>50||roll<-50) Moto1=0,Moto2=0;
			Set_Pwm(Moto1,Moto2);																								
		}
}