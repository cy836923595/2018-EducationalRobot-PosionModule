#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
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
#define Battery_Ch 6
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
#endif 















