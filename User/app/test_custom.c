#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "pid.h"
#include "sys.h"

int16_t test_moto_speed = 0;
int16_t test_moto_current[1];

void test_moto_control(void)
{
  test_moto_speed = rc.ch2 / RC_MAX_VALUE * MAX_WHEEL_RPM;
  
  //test_moto_current = test_moto_speed;
  
  
  test_moto_current[0] = pid_calc(&pid_wheel_spd[0], moto_chassis[0].speed_rpm, test_moto_speed);

	set_test_motor_current(test_moto_current);
  
}

void test_moto_init(void)
{

 pid_init(&pid_pit, 7000, 0,
                  1, 0, 0); 
}
