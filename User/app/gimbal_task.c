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
 



#include "gimbal_task.h"
#include "chassis_task.h"
#include "detect_task.h"

#include "can_device.h"
#include "uart_device.h"

#include "calibrate.h"
#include "keyboard.h"
#include "pid.h"
#include "ramp.h"
#include "sys.h"

#include "cmsis_os.h"
#include "stdlib.h"
#include "math.h"

/* 以下是云台任务所使用变量的定义，请勿修改 */
/* gimbal global information */
gimbal_yaw_t gim;
imu_t        imu;

/* gimbal pid parameter */

float yaw_angle_fdb = 0;
float pit_angle_fdb = 0; 

/* read from flash */
int32_t   pit_center_offset = 0;
int32_t   yaw_center_offset = 0;

/* for debug */
uint32_t gimbal_time_last;
int gimbal_time_ms;
/* 执行云台功能任务的函数 */
void gimbal_task(const void* argu)
{
  //云台和拨弹电机参数初始化
  gimbal_init_param();
  
  //从flash中读取云台中点位置
  read_gimbal_offset(&pit_center_offset, &yaw_center_offset);
  
  //云台控制任务循环
  uint32_t gimbal_wake_time = osKernelSysTick();
  while (1)
  {
    gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
    gimbal_time_last = HAL_GetTick();
    
    //获取云台传感器信息
    get_gimbal_information();
    
    //根据控制命令切换云台状态
    get_gimbal_mode();
  
    switch (gim.ctrl_mode)
    {
      //云台初始化状态
      case GIMBAL_INIT:
      {
        gimbal_init_handle();
      }break;
      
      //云台无输入状态
      case GIMBAL_NO_ACTION:
      {
        gimbal_noaction_handle();
        shoot_task();
      }break;
      
      //云台闭环控制模式
      case GIMBAL_CLOSE_LOOP_ZGYRO:
      {
        gimbal_loop_handle();
        shoot_task();
      }break;
      
      default:
        break;
    }

    //云台控制，如果有模块离线，就切断云台电机输出
    if (gim.ctrl_mode != GIMBAL_RELAX                 //云台释放模式
        && !glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //遥控器离线
        && !glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw轴电机离线
        && !glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist) //pitch轴电机离线
    {
      gimbal_custom_control();
      
    }
    else
    {
      gim.ctrl_mode = GIMBAL_RELAX;
      pid_trigger.iout = 0;
      send_gimbal_moto_zero_current();
    }
    
    //云台任务周期控制 5ms
    osDelayUntil(&gimbal_wake_time, GIMBAL_PERIOD);
  }
}

/* 以下为云台任务调用的内部函数，请勿改动 */

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
  * @param[in] center_offset: read gimbal_cali_data from chip flash
  * @retval    relative angle, unit is degree.
  */
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}

static action_mode_e remote_is_action(void)
{
  if ((abs(rc.ch1) >= 10)
   || (abs(rc.ch2) >= 10)
   || (abs(rc.ch3) >= 10)
   || (abs(rc.ch4) >= 10)
   || (abs(rc.mouse.x) >= 5)
   || (abs(rc.mouse.y) >= 5))
  {
    return IS_ACTION;
  }
  else
  {
    return NO_ACTION;
  }
}

/* gimbal relative position param */
float     pit_relative_angle;
float     yaw_relative_angle;  //unit: degree
void get_gimbal_information(void)
{
  //获取 imu 数据
  get_imu_data(&imu);
  
  //云台相对角度获取
  yaw_relative_angle =  get_relative_pos(moto_yaw.ecd, yaw_center_offset)/22.75f;
  pit_relative_angle = -get_relative_pos(moto_pit.ecd, pit_center_offset)/22.75f;
  
  //云台模式获取
  gim.ac_mode = remote_is_action();
  gim.last_mode = gim.ctrl_mode;
  
  //读取遥控器的控制命令
  pc_kb_hook();
}

uint8_t last_cali_cmd;
extern TaskHandle_t task1_t;
void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset)
{
  /* 配置摩擦轮电调，同时电调消除警报 */
  start_pwm_output(PWM_IO1);
  start_pwm_output(PWM_IO2);
  
  if (glb_cali_data.gimbal_cali_data.calied_flag == CALIED_FLAG)
  {
    *pit_offset = glb_cali_data.gimbal_cali_data.pit_offset;
    *yaw_offset = glb_cali_data.gimbal_cali_data.yaw_offset;
    
    //一切正常，唤醒底盘任务
    osThreadResume(task1_t);
  }
  else
  {
    //云台没有校准，进入校准处理
    while (1)
    {
      gimbal_cali_hook();
      
      send_chassis_moto_zero_current();
      send_gimbal_moto_zero_current();
      
      osDelay(10);
    }
  }
}

gimbal_back_e gimbal_back_step;
static void gimbal_back_param(void)
{
  gimbal_back_step = BACK_PIT_STEP;
  gim.ecd_offset_angle = yaw_relative_angle;
  
  ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  
  pid_pit_speed.max_output = 2000;
  
}


float chassis_angle_ac = 5;
void get_gimbal_mode(void)
{
  switch (rc.sw2)
  {
    case (RC_UP):
    {
      if (gim.ac_mode == NO_ACTION)
      {
        if (gim.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO)
        {
          //if (fabs(chassis.vw) <= chassis_angle_ac)
          if (fabs(yaw_relative_angle) <= chassis_angle_ac)
          {
            //begin no action handle
            gim.ctrl_mode = GIMBAL_NO_ACTION;
            
            gim.no_action_flag = 1;
            gim.no_action_time = HAL_GetTick();
          }
        }
      }
      else  //IS_ACTION mode
      {
        if (gim.ctrl_mode == GIMBAL_NO_ACTION)
        {
          gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
          gim.no_action_flag = 0;
          
          yaw_angle_ref = 0;
          gim.yaw_offset_angle = imu.angle_z;
        }

      }
      
      if (chassis.mode == CHASSIS_TWIST)
      {
        if (gim.ctrl_mode == GIMBAL_NO_ACTION)
        {
          gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
          gim.no_action_flag = 0;
          
          yaw_angle_ref = 0;
          gim.yaw_offset_angle = imu.angle_z;
        }
      }

      if (gim.last_mode == GIMBAL_RELAX)
      {
        gim.ctrl_mode = GIMBAL_INIT;
        gimbal_back_param();
      }
      
      
      
    }
    break;
    default:
    {
      gim.ctrl_mode = GIMBAL_RELAX;
      gimbal_cali_hook();
    }
    break;
  }

}

void gimbal_loop_handle(void)
{
  static float chassis_angle_tmp;
  
  pit_angle_fdb = pit_relative_angle;
  yaw_angle_fdb = imu.angle_z - gim.yaw_offset_angle;
  
  chassis_angle_tmp = yaw_angle_fdb - yaw_relative_angle;
  if (chassis.mode != CHASSIS_FIXED_ROUTE)
  {
    //限制yaw轴的活动角度
    if ((yaw_relative_angle >= YAW_ANGLE_MIN) && (yaw_relative_angle <= YAW_ANGLE_MAX))
    {
      gimbal_yaw_control();
      VAL_LIMIT(yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
    }
    //限制pitch轴的活动角度
    if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX))
    {
      gimbal_pitch_control();
      VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    }
  }

}

void gimbal_init_handle(void)
{
  pit_angle_fdb = pit_relative_angle;
  yaw_angle_fdb = yaw_relative_angle;
  
  /* gimbal pitch back center */
  pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));

  switch (gimbal_back_step)
  {
    case BACK_PIT_STEP:
    {
      /* keep yaw unmove this time */
      yaw_angle_ref = gim.ecd_offset_angle;
      
      if(fabs(pit_angle_fdb) <= 2.0f)
        gimbal_back_step = YAW_BACK_STEP;
    }break;
    
    case YAW_BACK_STEP:
    {
      /* yaw back center after pitch arrive */
      yaw_angle_ref = yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));
      
      if (fabs(yaw_angle_fdb) <= 1.5f)
         gimbal_back_step = BACK_IS_OK;
    }break;
    
    case BACK_IS_OK:
    {
      /* yaw arrive and switch gimbal state */
      gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
      
      gim.yaw_offset_angle = imu.angle_z;
      pit_angle_ref = 0;
      yaw_angle_ref = 0;
      
      pid_pit_speed.max_output = 5000;
    }break;
  }
}


void gimbal_noaction_handle(void)
{
  if (gim.no_action_flag == 1)
  {
    if ((HAL_GetTick() - gim.no_action_time) < 1500)
    {
      gimbal_loop_handle();
    }
    else
    {
      gim.no_action_flag = 2;
      yaw_angle_ref = 0;
    }
  }
  
  if (gim.no_action_flag == 2)
  {
    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;
  }
}

/**
  * @brief     initialize gimbal pid parameter, such as pitch/yaw/trigger motor, 
  *            imu temperature
  * @attention before gimbal control loop in gimbal_task() function
  */
void gimbal_init_param(void)
{
  //电调 bug
  osDelay(3000);
  
  /* 云台pitch轴电机PID参数初始化 */
  pid_init(&pid_pit, 2000, 0,
                  30, 0, 0); //
  pid_init(&pid_pit_speed, 5000, 2000,
                  20, 0.1, 0);

  /* 云台yaw轴电机PID参数初始化 */
  pid_init(&pid_yaw, 2000, 0,
                  25, 0, 0); //
  pid_init(&pid_yaw_speed, 5000, 800,
                  20, 0, 0);

  /* 拨弹电机PID参数初始化 */
  pid_init(&pid_trigger, 4000, 2000,
                  0.15f, 0, 0);
  pid_init(&pid_trigger_speed, 8000, 4000,
                  1.5, 0.05, 0);

  /* 将云台的初始化状态设置为释放 */
  gim.ctrl_mode = GIMBAL_RELAX;
  
}

/* 卡弹处理 */
uint32_t stall_count = 0;
uint32_t stall_inv_count = 0;
uint8_t  stall_f = 0;
void block_bullet_handle(void)
{
  if (pid_trigger_speed.out <= -5000)  //卡弹电流
  {
    if (stall_f == 0)
      stall_count ++;
  }
  else
    stall_count = 0;
  
  if (stall_count >= 600) //卡弹时间3s
  {
    stall_f = 1;
    stall_count = 0;
    
  }
  
  if (stall_f == 1)
  {
    stall_inv_count++;
    
    if (stall_inv_count >= 100)  //反转时间0.5s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trigger_moto_speed_ref = 2000;
  }
}


/* 射击任务相关参数 */
uint8_t   shoot_cmd = 0;
uint32_t  continue_shoot_time;
uint8_t   continuous_shoot_cmd = 0;
uint8_t   fric_wheel_run = 0;
uint16_t  fric_wheel_speed = SHOT_FRIC_WHEEL_SPEED;
/* 上次的遥控数据数据 */
uint8_t   last_left_key;
uint8_t   last_right_key;
uint8_t   last_sw1;
uint8_t   last_sw2;
int16_t   last_wheel_value;
//遥控器开关摩擦轮
#define RC_FIRC_CTRL     ((last_sw1 != RC_UP) && (rc.sw1 == RC_UP))           //((last_wheel_value != -660) && (rc.wheel == -660))
//遥控器单发
#define RC_SINGLE_TRIG   ((last_sw1 != RC_DN) && (rc.sw1 == RC_DN))           // ((last_wheel_value != 660) && (rc.wheel == 660))
//遥控器连发
#define RC_CONTIN_TRIG   ((rc.sw1 == RC_DN) && (HAL_GetTick() - continue_shoot_time >= 1500))   //((rc.wheel == 660) && (HAL_GetTick() - continue_shoot_time >= 1000))
//遥控器退出连发模式
#define EXIT_CONTIN_TRIG (last_sw1 != RC_DN)                                 // (rc.wheel <= 600)
void shoot_task(void)
{
  /* 开关摩擦轮操作控制 */
  {
    if ( RC_FIRC_CTRL && rc.sw2 == RC_UP)
        fric_wheel_run = !fric_wheel_run;
    
    if (rc.kb.bit.Q && rc.sw2 == RC_UP)
      fric_wheel_run = 1;
    
    if ((rc.kb.bit.Q && rc.kb.bit.SHIFT) || rc.sw2 != RC_UP)
      fric_wheel_run = 0;

    if (glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)
      fric_wheel_run = 0;
  }

  
  /* 开关摩擦轮实现函数 */
  turn_on_off_friction_wheel();
  
  /* bullet single or continue trigger command control  */
  {
    if ( RC_SINGLE_TRIG                  //遥控器单发
      || (km.lk_sta == KEY_PRESS_ONCE) ) //鼠标单发
    {
      continue_shoot_time = HAL_GetTick();
      shoot_cmd   = 1;
      continuous_shoot_cmd = 0;
    }

    if ( RC_CONTIN_TRIG                  //遥控器连发
      || (km.lk_sta == KEY_PRESS_LONG) ) //鼠标连发
    {
      shoot_cmd   = 0;
      continuous_shoot_cmd = 1;
    }
    else
    {
      continuous_shoot_cmd = 0;
    }
    
    if ( EXIT_CONTIN_TRIG               //退出连发处理
      || ((km.lk_sta == KEY_RELEASE) && (last_left_key == KEY_PRESS_LONG)) )
    {
      trigger_moto_position_ref = moto_trigger.total_ecd;
    }
    
    if (fric_wheel_run == 0)
    {
      shoot_cmd   = 0;
      continuous_shoot_cmd = 0;
    }
  }


  /* 单发连发射击实现函数 */
  shoot_custom_control();

  last_sw1 = rc.sw1;
  last_sw2 = rc.sw2;
  last_left_key    = km.lk_sta;
  last_right_key   = km.rk_sta;
  last_wheel_value = rc.wheel;
}

