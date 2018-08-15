#include "key.h"

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

/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;               //使能PORTA时钟	   	 
	GPIOA->CRL&=0XFF0FFFFF;           
	GPIOA->CRL|=0X00800000;           //PA15 上拉输入
  GPIOA->ODR|=1<<15;                //PA15 上拉	
//	Ex_NVIC_Config(GPIO_A,5,FTIR);		//下降沿触发
//	MY_NVIC_Init(2,1,EXTI9_5_IRQn,2);  	//抢占2，子优先级1，组2
}

void KEY_Init_MY(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**************************************************************************
函数功能：外部中断读取按键状态
入口参数：无
返回  值：无 
**************************************************************************/
void EXTI9_5_IRQHandler(void)
{			
	delay_ms(10);   //消抖			 
   if(KEY==0)		//按键
	{
  	LED=~LED;
	}
 	EXTI->PR=1<<5; //清除LINE15上的中断标志位  
}
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	  if(KEY==0)  Forever_count++;   //长按标志位未置1
     else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//双击执行的指令
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//单击执行的指令
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
			static u8 flag_key=1;//按键按松开标志
			if(flag_key&&KEY==0)
			{
				delay_ms(10);	
				if(KEY==0)	
				{	
					flag_key=0;
					return 1;	// 按键按下
				}
			}
			else if(1==KEY)			flag_key=1;
			return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
			static u16 Long_Press_count,Long_Press;
	    if(Long_Press==0&&KEY==0)  Long_Press_count++;   //长按标志位未置1
      else                       Long_Press_count=0; 
		  if(Long_Press_count>200)		
			{
				Long_Press=1;	
				Long_Press_count=0;
				return 1;
			}				
			 if(Long_Press==1)     //长按标志位置1
			{
				  Long_Press=0;
			}
			return 0;
}

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY1==0||KEY2==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
	 if(KEY1==1)return 1;
		else if(KEY2==1)return 2;

	}else if(KEY1==0&&KEY2==0)key_up=1; 	    
 	return 0;// 无按键按下
}
