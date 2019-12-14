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
 



/************************************ 用户对底盘自定义控制 *************************************/
#include "chassis_task.h"
#include "gimbal_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "keyboard.h"
#include "pid.h"
#include "sys.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

/* 底盘电机期望转速(rpm) */
int16_t chassis_moto_speed_ref[4];
/* 底盘电机电流 */
int16_t chassis_moto_current[4];

/* 底盘控制信号获取 */
void chassis_control_information_get(void)
{
  //遥控器以及鼠标对底盘的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
  chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
  chassis.vy = rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
  chassis.vw = rc.ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc.mouse.x * CHASSIS_PC_MOVE_RATIO_R;
}


/* 底盘运动的速度分解，以及电机转速的闭环控制 */
void chassis_custom_control(void)
{
  //底盘速度分解，计算底盘电机转速
  chassis_moto_speed_calc(chassis.vx, chassis.vy, chassis.vw, chassis_moto_speed_ref);

  //开环计算底盘轮子电机电流
  //chassis_open_loop_calculate();
  
  //闭环计算底盘轮子电机电流
  chassis_close_loop_calculate();
  
  //将计算好的电流值发送给电调
  send_chassis_moto_current(chassis_moto_current);
}

/* 底盘的运动分解处理 */
/**
  * @param 输入参数1: vx左右平移速度(mm/s)，右为正方向
  *        输入参数2: vy前后平移速度(mm/s)，前为正方向
  *        输入参数3: vw底盘旋转速度(degree/s)，逆时针为正方向
  *        输入参数4: speed[] 4个电机转速(rpm)
  * @note  下面是电机轮子编号，左上角为0号
  * @map   1 %++++++% 0
               ++++
               ++++
           2 %++++++% 3
  */
void chassis_moto_speed_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2.0f - GIMBAL_OFFSET)/RADIAN_COEF;
  static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2.0f + GIMBAL_OFFSET)/RADIAN_COEF;
  static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);

  int16_t wheel_rpm[4];
  float max = 0;

  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
  VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
  
  wheel_rpm[0] = (+vx - vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
  wheel_rpm[1] = (+vx + vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
  wheel_rpm[2] = (-vx + vy + vw * rotate_ratio_b) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx - vy + vw * rotate_ratio_b) * wheel_rpm_ratio;

  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

void chassis_close_loop_calculate(void)
{
  for (int i = 0; i < 4; i++)
  {
    chassis_moto_current[i] = pid_calc(&pid_wheel_spd[i], moto_chassis[i].speed_rpm, chassis_moto_speed_ref[i]);
  }
}

void chassis_open_loop_calculate(void)
{
  for (int i = 0; i < 4; i++)
  {
    chassis_moto_current[i] = chassis_moto_speed_ref[i] * 2;
  }
}

/* 底盘扭腰处理 */
extern uint32_t twist_count;
void chassis_twist_handle(void)
{
  /* 扭腰周期时间 */
  static int16_t twist_period = TWIST_PERIOD/CHASSIS_PERIOD;
  /* 扭腰最大角度限制 */
  static int16_t twist_angle  = TWIST_ANGLE;
  /* 生成扭腰角度 */
  static float   chassis_angle_target;
  twist_count++;
  chassis_angle_target = twist_angle*sin(2*3.14/twist_period*twist_count);
  chassis.vw = pid_calc(&pid_chassis_angle, yaw_relative_angle, chassis_angle_target);
}
