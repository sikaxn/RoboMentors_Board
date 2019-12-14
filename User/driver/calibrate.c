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
 


  
#include "rm_hal_lib.h"
#include "calibrate.h"
#include "can_device.h"

/* 全局校准数据结构体 */
global_cali_t glb_cali_data;

/**
  * @brief     保存校准数据到用户 flash 中
  */
static void save_cali_data(void)
{
  write_flash((uint8_t*)&glb_cali_data, sizeof(global_cali_t));
}
/**
  * @brief     从用户 flash 中读取校准数据
  */
void read_cali_data(void)
{
  read_flash((uint8_t*)&glb_cali_data, sizeof(global_cali_t));
}

/**
  * @brief     云台校准数据保存函数
  */
void gimbal_cali_hook(void)
{
  if (glb_cali_data.gimbal_cali_data.cali_cmd == 1)
  {
    glb_cali_data.gimbal_cali_data.pit_offset = moto_pit.ecd;
    glb_cali_data.gimbal_cali_data.yaw_offset = moto_yaw.ecd;
    glb_cali_data.gimbal_cali_data.calied_flag = CALIED_FLAG;
    glb_cali_data.gimbal_cali_data.cali_cmd  = 0;
    save_cali_data();
  }
}
