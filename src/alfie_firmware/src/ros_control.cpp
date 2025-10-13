#include "ros_control.h"
#include "config.h"
#include "driverboard.h"   
#include "motor_control.h"

extern DriverBoard b;
extern 




void subscription_callback(const void *msgin)
{
  const alfie_msgs__msg__DriverCmd *msg = (const alfie_msgs__msg__DriverCmd *)msgin;

  // Update watchdog timestamp
  b.last_drivercmd_time = millis();

  b.drivercmd_timeout = false;

  b.drivercmdbuf[0] = msg->driver_pwm[0];
  b.drivercmdbuf[1] = msg->driver_pwm[1];

  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.mBuf[i].memory.torqueSwitch = msg->servo_cmd[i].torqueswitch;
    b.mBuf[i].memory.acceleration = msg->servo_cmd[i].acceleration;
    b.mBuf[i].memory.targetLocation = msg->servo_cmd[i].targetlocation;
  }

}

void populate_service_response(int servo, alfie_msgs__srv__ServoService_Response *res)
{
  res->memorymap.firmwaremajor = b.mBuf[(servo - 1)].memory.firmwareMajor;
  res->memorymap.firmwaresub = b.mBuf[(servo - 1)].memory.firmwareSub;
  //res->memorymap.unassigned0 = b.mBuf[(servo - 1)].memory.unassigned0;
  res->memorymap.servomajor = b.mBuf[(servo - 1)].memory.servoMajor;
  res->memorymap.servosub = b.mBuf[(servo - 1)].memory.servoSub;
  res->memorymap.servoid = b.mBuf[(servo - 1)].memory.servoID;
  res->memorymap.baudrate = b.mBuf[(servo - 1)].memory.baudRate;
  res->memorymap.returndelay = b.mBuf[(servo - 1)].memory.returnDelay;
  res->memorymap.responsestatuslevel = b.mBuf[(servo - 1)].memory.responseStatusLevel;
  res->memorymap.minanglelimit = b.mBuf[(servo - 1)].memory.minAngleLimit;
  res->memorymap.maxanglelimit = b.mBuf[(servo - 1)].memory.maxAngleLimit;
  res->memorymap.maxtemplimit = b.mBuf[(servo - 1)].memory.maxTempLimit;
  res->memorymap.maxinputvoltage = b.mBuf[(servo - 1)].memory.maxInputVoltage;
  res->memorymap.mininputvoltage = b.mBuf[(servo - 1)].memory.minInputVoltage;
  res->memorymap.maxtorque = b.mBuf[(servo - 1)].memory.maxTorque;
  res->memorymap.phase = b.mBuf[(servo - 1)].memory.phase;
  res->memorymap.unloadingcondition = b.mBuf[(servo - 1)].memory.unloadingCondition;
  res->memorymap.ledalarmcondition = b.mBuf[(servo - 1)].memory.LEDAlarmCondition;
  res->memorymap.pcoefficient = b.mBuf[(servo - 1)].memory.Pcoefficient;
  res->memorymap.dcoefficient = b.mBuf[(servo - 1)].memory.Dcoefficient;
  res->memorymap.icoefficient = b.mBuf[(servo - 1)].memory.Icoefficient;
  res->memorymap.minstartupforce = b.mBuf[(servo - 1)].memory.minStartupForce;
  res->memorymap.clockwiseinsensitivearea = b.mBuf[(servo - 1)].memory.clockwiseInsensitiveArea;
  res->memorymap.counterclockwiseinsensitiveregion = b.mBuf[(servo - 1)].memory.counterclockwiseInsensitiveRegion;
  res->memorymap.protectioncurrent = b.mBuf[(servo - 1)].memory.protectionCurrent;
  res->memorymap.angularresolution = b.mBuf[(servo - 1)].memory.angularResolution;
  // TODO FIX positioncorrection
  res->memorymap.positioncorrection = b.mBuf[(servo - 1)].memory.positionCorrection;
  res->memorymap.operationmode = b.mBuf[(servo - 1)].memory.operationMode;
  res->memorymap.protectivetorque = b.mBuf[(servo - 1)].memory.protectiveTorque;
  res->memorymap.protectiontime = b.mBuf[(servo - 1)].memory.protectionTime;
  res->memorymap.overloadtorque = b.mBuf[(servo - 1)].memory.overloadTorque;
  res->memorymap.speedclosedlooppcoefficient = b.mBuf[(servo - 1)].memory.speedClosedLoopPcoefficient;
  res->memorymap.overcurrentprotectiontime = b.mBuf[(servo - 1)].memory.OvercurrentProtectionTime;
  res->memorymap.velocityclosedloopicoefficient = b.mBuf[(servo - 1)].memory.velocityClosedLoopIcoefficient;
  res->memorymap.torqueswitch = b.mBuf[(servo - 1)].memory.torqueSwitch;
  res->memorymap.acceleration = b.mBuf[(servo - 1)].memory.acceleration;
  res->memorymap.targetlocation = b.mBuf[(servo - 1)].memory.targetLocation;
  res->memorymap.runningtime = b.mBuf[(servo - 1)].memory.runningTime;
  res->memorymap.runningspeed = b.mBuf[(servo - 1)].memory.runningSpeed;
  res->memorymap.torquelimit = b.mBuf[(servo - 1)].memory.torqueLimit;
  //res->memorymap.unassigned1 = b.mBuf[(servo - 1)].memory.unassigned1;
  //res->memorymap.unassigned2 = b.mBuf[(servo - 1)].memory.unassigned2;
  //res->memorymap.unassigned3 = b.mBuf[(servo - 1)].memory.unassigned3;
  //res->memorymap.unassigned4 = b.mBuf[(servo - 1)].memory.unassigned4;
  //res->memorymap.unassigned5 = b.mBuf[(servo - 1)].memory.unassigned5;
  res->memorymap.lockmark = b.mBuf[(servo - 1)].memory.lockMark;
  res->memorymap.currentlocation = b.mBuf[(servo - 1)].memory.currentLocation;
  res->memorymap.currentspeed = b.mBuf[(servo - 1)].memory.currentSpeed;
  res->memorymap.currentload = b.mBuf[(servo - 1)].memory.currentLoad;
  res->memorymap.currentvoltage = b.mBuf[(servo - 1)].memory.currentVoltage;
  res->memorymap.currenttemperature = b.mBuf[(servo - 1)].memory.currentTemperature;
  res->memorymap.asyncwriteflag = b.mBuf[(servo - 1)].memory.asyncWriteFlag;
  res->memorymap.servostatus = b.mBuf[(servo - 1)].memory.servoStatus;
  res->memorymap.mobilesign = b.mBuf[(servo - 1)].memory.mobileSign;
  //res->memorymap.unassigned6 = b.mBuf[(servo - 1)].memory.unassigned6;
  //res->memorymap.unassigned7 = b.mBuf[(servo - 1)].memory.unassigned7;
  res->memorymap.currentcurrent = b.mBuf[(servo - 1)].memory.currentCurrent;

}

void service_callback(const void *request, void *response)
{
  const alfie_msgs__srv__ServoService_Request *req = (const alfie_msgs__srv__ServoService_Request *)request;
  alfie_msgs__srv__ServoService_Response *res = (alfie_msgs__srv__ServoService_Response *)response;

  int16_t value = 0;

  switch (req->operation)
  {
  case 'R': // Read
    populate_service_response(req->servo, res);
    break;

  case 'W': // Write
  if(isWord(req->address)){
      // if the address is 31 then set bit 11 as the negative bit and flip the sign of the short
      if (req->address == 31)
      {
        if (value < 0)
        {
          value = -(value - 0x800);
        }
      }
      // write the address and address + 1 as a 16 bit value
      b.mBuf[req->servo - 1].bytes[req->address] = req->value & 0xFF;
      b.mBuf[req->servo - 1].bytes[req->address + 1] = (req->value >> 8) & 0xFF;
    }
    else{
      b.mBuf[req->servo - 1].bytes[req->address] = req->value;
    }
    populate_service_response(req->servo, res);
    break;

  default:
    populate_service_response(req->servo, res);
    break;
  }
}



void generateLowStatus()
{

  b.driverState.driver_diagnostics.drivercmd_timeout = b.drivercmd_timeout;

  // Motor A state
  b.driverState.motor_state[0].pwm_cmd = b.drivercmdbuf[0];
  b.driverState.motor_state[0].pulse_count = b.A_wheel_pulse_count;
  b.driverState.motor_state[0].velocity = b.A_velocity;
  b.driverState.motor_state[0].acceleration = b.A_acceleration;
  b.driverState.motor_state[0].is_moving = b.A_is_moving;

  // Motor B state
  b.driverState.motor_state[1].pwm_cmd = b.drivercmdbuf[1];
  b.driverState.motor_state[1].pulse_count = b.B_wheel_pulse_count;
  b.driverState.motor_state[1].velocity = b.B_velocity;
  b.driverState.motor_state[1].acceleration = b.B_acceleration;
  b.driverState.motor_state[1].is_moving = b.B_is_moving;

  b.driverState.shoulder_limit_state = b.shoulder_limit_switch;

  b.driverState.driver_diagnostics.a_wheel_pulse = b.A_wheel_pulse_count;
  b.driverState.driver_diagnostics.b_wheel_pulse = b.B_wheel_pulse_count;

  b.driverState.driver_diagnostics.pollservostatusduration = b.pollservostatusduration;
  b.driverState.driver_diagnostics.updateservoidleduration = b.updateservoidleduration;
  b.driverState.driver_diagnostics.updateservoactiveduration = b.updateservoactiveduration;
  b.driverState.driver_diagnostics.imuupdateduration = b.imuupdateduration;

  b.driverState.board_temp = b.board_temp;
  b.driverState.imu.angular_velocity_x = b.stGyroRawData.X;
  b.driverState.imu.angular_velocity_y = b.stGyroRawData.Y;
  b.driverState.imu.angular_velocity_z = b.stGyroRawData.Z;
  b.driverState.imu.linear_acceleration_x = b.stAccelRawData.X;
  b.driverState.imu.linear_acceleration_y = b.stAccelRawData.Y;
  b.driverState.imu.linear_acceleration_z = b.stAccelRawData.Z;
  b.driverState.imu.orientation_x = b.IMU_Roll;
  b.driverState.imu.orientation_y = b.IMU_Pitch;
  b.driverState.imu.orientation_z = b.IMU_Yaw;
  b.driverState.imu.orientation_w = 0.0;
  b.driverState.magnetic_field.magnetic_field_x = b.stMagnRawData.X;
  b.driverState.magnetic_field.magnetic_field_y = b.stMagnRawData.Y;
  b.driverState.magnetic_field.magnetic_field_z = b.stMagnRawData.Z;



  for (int i = 0; i < NUMSERVOS; i++)
  {
    // Direct access - no memcpy needed!
    b.driverState.servo_state[i].acceleration = b.mBuf[i].memory.acceleration;
    b.driverState.servo_state[i].currentvoltage = b.mBuf[i].memory.currentVoltage;
    b.driverState.servo_state[i].currentcurrent = b.mBuf[i].memory.currentCurrent;
    b.driverState.servo_state[i].currentload = b.mBuf[i].memory.currentLoad;
    b.driverState.servo_state[i].currentlocation = b.mBuf[i].memory.currentLocation;
    b.driverState.servo_state[i].currenttemperature = b.mBuf[i].memory.currentTemperature;
    b.driverState.servo_state[i].mobilesign = b.mBuf[i].memory.mobileSign;
    b.driverState.servo_state[i].servostatus = b.mBuf[i].memory.servoStatus;
    b.driverState.servo_state[i].targetlocation = b.mBuf[i].memory.targetLocation;
    b.driverState.servo_state[i].torqueswitch = b.mBuf[i].memory.torqueSwitch;
  }

  //RCSOFTCHECK(rcl_publish(&b.publisher, &b.driverState, NULL));

}


bool create_ros_entities()
{
  // create init_
  b.allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&b.support, 0, NULL, &b.allocator));

  // create node
  RCCHECK(rclc_node_init_default(&b.node, NODENAME, "", &b.support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &b.publisher,
      &b.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, DriverState),
      STATEPUBLISHER));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &b.subscriber,
      &b.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, DriverCmd),
      CMDSUBSCRIBER));

  // create service
  RCCHECK(rclc_service_init_default(
      &b.service,
      &b.node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(alfie_msgs, srv, ServoService),
      SERVOSERVICE));

  // create executor
  RCCHECK(rclc_executor_init(&b.executor, &b.support.context, 2, &b.allocator));
  RCCHECK(rclc_executor_add_subscription(&b.executor, &b.subscriber, &b.driverCmd, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&b.executor, &b.service, &b.req, &b.res, service_callback));

  return true;
}

bool destroy_ros_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&b.support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t ret;
  ret = rcl_publisher_fini(&b.publisher, &b.node);
  ret = rcl_subscription_fini(&b.subscriber, &b.node);
  ret = rcl_service_fini(&b.service, &b.node);
  ret = rclc_executor_fini(&b.executor);
  ret = rclc_support_fini(&b.support);
  ret = rcl_node_fini(&b.node);

  return true;
}


/*
  these are the serial bus servo addresses
  that hold 16 bit values

  location 31 the sign bit is bit11
*/
bool isWord(uint8_t a) {
  if (a == 9 || a == 11 || a == 16 || a == 24 || a == 28 || a == 31 || a == 42 || a == 44 || a == 46 || a == 48) {
    return true;
  }
  return false;
}


// Error handle loop
void error_loop()
{
  while (1)
  {
    DriveA(16);
    DriveB(16);
    delay(200);
    DriveA(-16);
    DriveB(-16);
    delay(200);
  }
}
