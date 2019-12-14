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
 


  
#include "detect_task.h"
#include "sys.h"
#include "cmsis_os.h"

#define LED_R_ON   write_led_io(LED_R, LED_ON);
#define LED_R_OFF  write_led_io(LED_R, LED_OFF);

#define LED_G_ON   write_led_io(LED_G, LED_ON);
#define LED_G_OFF  write_led_io(LED_G, LED_OFF);

glb_err_type_t glb_err;

static uint8_t  beep_ctrl;
static uint16_t err_count;

/* 以下为模块离线监测任务调用的内部函数，请勿改动 */


/**
  * @brief     初始化离线设备检测数据结构
  */
void global_err_detector_init(void)
{
  glb_err.err_now = NULL;

  glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist   = 0;
  glb_err.err_list[REMOTE_CTRL_OFFLINE].warn_pri    = 10;  //max priority
  glb_err.err_list[REMOTE_CTRL_OFFLINE].set_timeout = 100; //ms
  glb_err.err_list[REMOTE_CTRL_OFFLINE].delta_time  = 0;
  glb_err.err_list[REMOTE_CTRL_OFFLINE].last_time   = 0x00;
  glb_err.err_list[REMOTE_CTRL_OFFLINE].enable      = 1;

  glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist   = 0;
  glb_err.err_list[GIMBAL_PIT_OFFLINE].warn_pri    = 9;
  glb_err.err_list[GIMBAL_PIT_OFFLINE].set_timeout = 200;
  glb_err.err_list[GIMBAL_PIT_OFFLINE].delta_time  = 0;
  glb_err.err_list[GIMBAL_PIT_OFFLINE].last_time   = 0x00;
  glb_err.err_list[GIMBAL_PIT_OFFLINE].enable      = 1;

  glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist   = 0;
  glb_err.err_list[GIMBAL_YAW_OFFLINE].warn_pri    = 8;
  glb_err.err_list[GIMBAL_YAW_OFFLINE].set_timeout = 200;
  glb_err.err_list[GIMBAL_YAW_OFFLINE].delta_time  = 0;
  glb_err.err_list[GIMBAL_YAW_OFFLINE].last_time   = 0x00;
  glb_err.err_list[GIMBAL_YAW_OFFLINE].enable      = 1;

  glb_err.err_list[TRIGGER_MOTO_OFFLINE].err_exist   = 0;
  glb_err.err_list[TRIGGER_MOTO_OFFLINE].warn_pri    = 6;
  glb_err.err_list[TRIGGER_MOTO_OFFLINE].set_timeout = 200;
  glb_err.err_list[TRIGGER_MOTO_OFFLINE].delta_time  = 0;
  glb_err.err_list[TRIGGER_MOTO_OFFLINE].last_time   = 0x00;
  glb_err.err_list[TRIGGER_MOTO_OFFLINE].enable      = 1;

  for (int i = 0; i < 4; i++)
  {
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].err_exist   = 0;
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].warn_pri    = 2 + i; //2,3,4,5
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].set_timeout = 200;
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].delta_time  = 0;
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].last_time   = 0x00;
    glb_err.err_list[CHASSIS_M1_OFFLINE + i].enable      = 1;
  }

}

/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_detector_hook(int err_id)
{
  if (glb_err.err_list[err_id].enable)
    glb_err.err_list[err_id].last_time = HAL_GetTick();
}

/**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void detect_task(const void* argu)
{
  global_err_detector_init();
  osDelay(100);

  beep_ctrl = BEEP_OFF;
  
  while(1)
  {
    int max_priority = 0;
    int err_cnt      = 0;
    
    for (int id = 0; id < ERROR_LIST_LENGTH; id++)
    {
      glb_err.err_list[id].delta_time = HAL_GetTick() - glb_err.err_list[id].last_time;
      if (glb_err.err_list[id].enable && (glb_err.err_list[id].delta_time > glb_err.err_list[id].set_timeout))
      {
        glb_err.err_list[id].err_exist = 1; //this module is offline
        err_cnt++;
        if (glb_err.err_list[id].warn_pri > max_priority)
        {
          max_priority   = glb_err.err_list[id].warn_pri;
          glb_err.err_now = &(glb_err.err_list[id]);
          glb_err.err_id  = (err_id_e)id;
        }
      }
      else
      {
        glb_err.err_list[id].err_exist = 0;
      }
    }

    if (!err_cnt) //all scan no error, should clear err pointer!!!
      glb_err.err_now = NULL;

    if (glb_err.err_now != NULL)
    {
      LED_G_OFF;
      module_offline_callback();
    }
    else
    {
      LED_G_ON;
      beep_ctrl = BEEP_OFF;
    }
    
    //配置蜂鸣器开关
    set_beep_param(BEEP1_IO, BEEP_FREQ, beep_ctrl);
    
    osDelay(50);
  }
}


void module_offline_callback(void)
{
  err_count++;
  if (err_count > 50)
    err_count = 0;
  
  switch (glb_err.err_id)
  {
    case REMOTE_CTRL_OFFLINE:
    {
      if (err_count == 1)
      {
        LED_R_ON;
        beep_ctrl = BEEP_ON;
      }
      else
      {
        LED_R_OFF;
        beep_ctrl = BEEP_OFF;
      }
    }break;
    
    case GIMBAL_YAW_OFFLINE:
    {
      if (err_count == 1
          || err_count == 7)
      {
        LED_R_ON;
        beep_ctrl = BEEP_ON;
      }
      else
      {
        LED_R_OFF;
        beep_ctrl = BEEP_OFF;
      }
    }break;
    
    case GIMBAL_PIT_OFFLINE:
    {
      if (err_count == 1
          || err_count == 7
          || err_count == 13)
      {
        LED_R_ON;
        beep_ctrl = BEEP_ON;
      }
      else
      {
        LED_R_OFF;
        beep_ctrl = BEEP_OFF;
      }
    }break;
    
    case TRIGGER_MOTO_OFFLINE:
    {
      if (err_count == 1
          || err_count == 7
          || err_count == 13
          || err_count == 19)
      {
        LED_R_ON;
        beep_ctrl = BEEP_ON;
      }
      else
      {
        LED_R_OFF;
        beep_ctrl = BEEP_OFF;
      }
    }break;
    
    default:
    {
      LED_R_ON;
      beep_ctrl = BEEP_OFF;
    }break;
  }
}


