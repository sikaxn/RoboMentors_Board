/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/
 
#include "uart_device.h"
#include "sys.h"

#include "stdlib.h"
#include "string.h"

/* 解析后的遥控器数据 */
rc_type_t rc;
/* 接收到的遥控器原始数据 */
uint8_t   dbus_recv[DBUS_FRAME_SIZE];

/**
  * @brief     遥控器中断回调函数，在设置 UART 接收时注册
  */
void dbus_uart_callback(void)
{
  remote_data_handle(&rc, dbus_recv);
}

/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
  /* 下面是正常遥控器数据的处理 */
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  /* 防止遥控器零点有偏差 */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;

  /* 拨杆值获取 */
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  /* 遥控器异常值处理，函数直接返回 */
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_type_t));
    return ;
  }

  /* 鼠标移动速度获取 */
  rc->mouse.x = buff[6] | (buff[7] << 8);
  rc->mouse.y = buff[8] | (buff[9] << 8);
  
  /* 鼠标左右按键键值获取 */
  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  /* 键盘按键键值获取 */
  rc->kb.key_code = buff[14] | buff[15] << 8;
  
  /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
  rc->wheel = buff[16] | buff[17] << 8;
  rc->wheel -= 1024;
}
