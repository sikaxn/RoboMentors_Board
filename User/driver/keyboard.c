/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 



#include "keyboard.h"
#include "uart_device.h"
#include "gimbal_task.h"
#include "sys.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  800   //ms
/* key acceleration time */
#define KEY_ACC_TIME     1000  //ms

km_control_t km;

int16_t delta_spd = MAX_CHASSIS_VX_SPEED*1.0f/KEY_ACC_TIME*GIMBAL_PERIOD;

/**
  * @brief     鼠标按键状态机
  * @param[in] sta: 按键状态指针
  * @param[in] key: 按键键值
  */
static void key_fsm(kb_state_e *sta, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
        if (sta == &km.lk_sta)
          km.lk_cnt = 0;
        else
          km.rk_cnt = 0;
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
        if (sta == &km.lk_sta)
        {
          if (km.lk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else
        {
          if (km.rk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
    
    default:
    break;
      
  }
}

/**
  * @brief     PC 键盘鼠标数据处理函数
  */
void pc_kb_hook(void)
{
  if (rc.kb.bit.SHIFT)
  {
    km.move_mode = FAST_MODE;
    km.max_spd = 3500;
  }
  else if (rc.kb.bit.CTRL)
  {
    km.move_mode = SLOW_MODE;
    km.max_spd = 2500;
  }
  else
  {
    km.move_mode = NORMAL_MODE;
    km.max_spd = 3000;
  }

  //add ramp
  if (rc.kb.bit.W)
    km.vy += delta_spd;
  else if (rc.kb.bit.S)
    km.vy -= delta_spd;
  else
    km.vy = 0;

  if (rc.kb.bit.A)
    km.vx += -delta_spd;
  else if (rc.kb.bit.D)
    km.vx += delta_spd;
  else
    km.vx = 0;

  VAL_LIMIT(km.vx, -km.max_spd, km.max_spd);
  VAL_LIMIT(km.vy, -km.max_spd, km.max_spd);
  
  VAL_LIMIT(km.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
  VAL_LIMIT(km.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
  
  key_fsm(&km.lk_sta, rc.mouse.l);
  key_fsm(&km.rk_sta, rc.mouse.r);
}
