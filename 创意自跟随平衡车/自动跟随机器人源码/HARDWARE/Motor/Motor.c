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

#include "motor.h"

extern float xianshi1; 	


extern int Moto1,Moto2; 
extern int distance1,distance2;
extern float pitch,roll,yaw; 		//欧拉角
extern short aacx,aacy,aacz;			//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern short temp;								//温度	
extern u8 mode;
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; //蓝牙遥控相关的变量

void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
}

void Motor_Stop(void)
{
		AIN1=0;
		AIN2=0;
		BIN1=0;			
		BIN2=0;
		PWMA=0;	
		PWMB=0;	
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
    	if(moto1>0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
			PWMA=myabs(moto1);
		  if(moto2>0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
//		if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
//	  if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}

int balance(float Angle,float Gyro)
{  
   float Bias,kp=300,kd=1;
	 int balance;
	 Bias=Angle;       //===求出平衡的角度中值 和机械相关
	 balance=kp*Bias+Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
		static float Velocity,Encoder_Least,Encoder,Movement;
		static float Encoder_Integral,Target_Velocity;
		float kp=80,ki=0.4;
	
		float line_distance1,line_distance2;//超声波的距离
	
		float cha;
	
		float set_distance=600;//控制小车在600mm之内
	
		line_distance1=distance1-set_distance;
		line_distance2=distance2-set_distance;
		//=============自动跟随部分=======================//
		if(mode==1)	
		{
				if(cha==0) 	
				{
					if((line_distance1>0&&line_distance2>0)||(line_distance1<0&&line_distance2<0))
					{
						Movement=0.2*((line_distance1+line_distance2)/2);
					}		
				}
				if(distance1>2000||distance2>2000) Movement=0;
		}
		//=============遥控前进后退部分=======================//
		if(mode==2)
		{	
			if(1==Flag_Qian)	Movement=60;	             //===如果前进标志位置1 位移为负
			else if(1==Flag_Hou)	  Movement=-60;        //===如果后退标志位置1 位移为正
			else  Movement=0;	
		}
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
		if(roll>50||roll<-50) 	Encoder_Integral=0;												//===清除积分	
	  return Velocity;
}


int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	  static float Turn_Target,Encoder_temp,Turn_Convert=0.9,Turn_Count,Turn;
		float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;         
	  float Kp1=40,Kd1=0,ccc;  
	  float cha1;
		//=============自动跟随部分=======================//
		if(mode==1)
		{
			cha1=distance1-distance2;
			if(distance1>20&&distance1<2000&&distance2>20&&distance2<2000&&cha1<0) 	
			{
				ccc=1*cha1;	
			}
			else if(distance1>20&&distance1<2000&&distance2>20&&distance2<2000&&cha1>0) 	
			{	
				ccc=1*cha1;
			}
			else 
			{
				ccc=0;
			}		
			Turn_Target=ccc;
			Turn=-Turn_Target*Kp1;                 //===结合Z轴陀螺仪进行PD控制
			return Turn;
		}
		//=============蓝牙控制部分=======================//
		if(mode==2)
		{
			if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
			{
				if(++Turn_Count==1)
				Encoder_temp=myabs(encoder_left+encoder_right);
				Turn_Convert=50/Encoder_temp;
				if(Turn_Convert<0.6)Turn_Convert=0.6;
				if(Turn_Convert>3)Turn_Convert=3;
			}	
			else
			{
				Turn_Convert=0;
				Turn_Count=0;
				Encoder_temp=0;
			}			
			if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
			else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
			else Turn_Target=0;
		
			if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
			if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
			if(Flag_Qian==1||Flag_Hou==1)  Kd=1;        
			else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
			//=============转向PD控制器=======================//
			Turn=-Turn_Target*Kp-0*gyro*Kd;                 	//===结合Z轴陀螺仪进行PD控制
			return Turn;
	}
}

//int turn(int encoder_left,int encoder_right,float gyro)//转向控制
//{
//	  static float Turn_Target,Turn;
//	  float Kp=40,Kd=0,ccc;  
//	  float cha1;
//		cha1=distance1-distance2;
//		if(distance1>20&&distance1<1000&&distance2>20&&distance2<1000&&cha1<0) 	
//		{
//			ccc=1*cha1;	
//		}
//		else if(distance1>20&&distance1<1000&&distance2>20&&distance2<1000&&cha1>0) 	
//		{	
//			ccc=1*cha1;
//		}
//		else 
//		{
//			ccc=0;
//		}		
//		Turn_Target=ccc;
//		Turn=-Turn_Target*Kp;                 //===结合Z轴陀螺仪进行PD控制
//	  return Turn;
//}
