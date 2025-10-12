#ifndef _ROS_CONTROL_H_
#define _ROS_CONTROL_H_

#include "config.h"

bool create_ros_entities();
bool destroy_ros_entities();
void subscription_callback(const void *msgin);
void service_callback(const void *request, void *response);
void populate_service_response(int servo, alfie_msgs__srv__ServoService_Response *res);
bool isWord(uint8_t a);
void generateLowStatus();
void error_loop();


#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      init = uxr_millis();             \
      X;                               \
    }                                  \
  } while (0)




  #endif // _ROS_CONTROL_H_