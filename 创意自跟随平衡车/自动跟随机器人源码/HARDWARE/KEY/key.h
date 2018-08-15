#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define KEY PAin(5)
void KEY_Init(void);          //按键初始化
u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);

#define KEY1 PCin(13)
#define KEY2 PCin(14)

void KEY_Init_MY(void);
#endif 
