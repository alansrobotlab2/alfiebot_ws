#include "config.h"
#include "driverboard.h"
#include "rosutils.h"

#if 0
// right arm and head
#define NODENAME "driver1"
#define STATEPUBLISHER "driverstate"
#define CMDSUBSCRIBER "drivercmd"
#define SERVOSERVICE "servoservice"
#define NUMSERVOS 10

#else
// left arm
#define NODENAME "driver0"
#define STATEPUBLISHER "driverstate"
#define CMDSUBSCRIBER "drivercmd"
#define SERVOSERVICE "servoservice"
#define NUMSERVOS 7

#endif

DriverBoard b;

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void generateLowStatus()
{
  b.driverState.mcu_temperature = 0;
  b.driverState.mcu_voltage = b.mBuf[0].memory.currentVoltage;
  b.driverState.servo_loop_ms = b.duration2;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    memcpy(b.rbuf.bytes, b.mBuf[i].bytes, sizeof(MemoryStruct));
    b.driverState.servo_state[i].acceleration = b.rbuf.memory.acceleration;
    b.driverState.servo_state[i].currentvoltage = b.rbuf.memory.currentVoltage;
    b.driverState.servo_state[i].currentcurrent = b.rbuf.memory.currentCurrent;
    b.driverState.servo_state[i].currentload = b.rbuf.memory.currentLoad;
    b.driverState.servo_state[i].currentlocation = b.rbuf.memory.currentLocation;
    b.driverState.servo_state[i].currenttemperature = b.rbuf.memory.currentTemperature;
    b.driverState.servo_state[i].mobilesign = b.rbuf.memory.mobileSign;
    b.driverState.servo_state[i].servostatus = b.rbuf.memory.servoStatus;
    b.driverState.servo_state[i].targetlocation = b.rbuf.memory.targetLocation;
    b.driverState.servo_state[i].torqueswitch = b.rbuf.memory.torqueSwitch;
  }

  RCSOFTCHECK(rcl_publish(&b.publisher, &b.driverState, NULL));
}

/*
  The only thing I could figure out in order to interrupt
  the sync comms and get everything reset.
  Otherwise the packets get out of sync and the 8ms loop goes to 120ms
  and all the data is wrong.
  It works.
*/
void flushSerialServoLine()
{
  for (int e = 0; e < NUMSERVOS; e++)
  {
    b.st.Ping(e + 1);
  }
}

void updateServoStatus()
{
  b.start_time1 = millis();
  b.start_time2 = micros();

  b.st.syncReadPacketTx(b.IDS, NUMSERVOS, 56, 15);

  int RxResult = 0;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    RxResult = b.st.syncReadPacketRx((i + 1), b.buf.bytes + 56);
    if (RxResult > 0)
    {
      memcpy(b.mBuf[i].bytes + 56, b.buf.bytes + 56, 15);
    }
    else
    {
      flushSerialServoLine();
      return;
    }
  }

  // update torque switch
  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.torquecommandbuf[i] = b.mBuf[i].memory.torqueSwitch;
  }
  // st.syncWrite(IDS, NUMSERVOS, 40, torquecommandbuf, 1);

  // selectively update position based on the torque switch == 1
  int index = 0;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    if (b.torquecommandbuf[i] == 1)
    {
      b.servocommandbuf[(3 * index) + 0] = b.mBuf[i].memory.acceleration;
      b.servocommandbuf[(3 * index) + 1] = b.mBuf[i].bytes[42];
      b.servocommandbuf[(3 * index) + 2] = b.mBuf[i].bytes[43];
      b.servoCMDIDS[index] = (i + 1);
      index++;
    }
  }

  if (index > 0)
  {
    // st.syncWrite(servoCMDIDS, index, 41, servocommandbuf, 3);
  }

  b.duration1 = static_cast<uint8_t>((millis() - b.start_time1));
  b.duration2 = static_cast<uint8_t>((micros() - b.start_time2) / 100);
}


void ServoLoop()
{

  switch (b.servoLoopState)
  {
  case b.RUNNING:
    updateServoStatus();
    generateLowStatus();
    break;
  case b.REQUEST_STOP:
    b.servoLoopState = b.STOPPED;
    break;
  case b.STOPPED:
    break;
  default:
    break;
  }
}


void subscription_callback(const void *msgin)
{
  const alfie_msgs__msg__DriverCmd *msg = (const alfie_msgs__msg__DriverCmd *)msgin;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.mBuf[i].memory.torqueSwitch = msg->servo_cmd[i].torqueswitch;
    b.mBuf[i].memory.acceleration = msg->servo_cmd[i].acceleration;
    b.mBuf[i].memory.targetLocation = msg->servo_cmd[i].targetlocation;
  }
}



/*
  the value from the servo is a 12 bit signed value in the range of -2048 to 2047
  so we convert to a formal 16 bit signed value
*/
int16_t convertcorrection_fromservo(int16_t servovalue)
{

  if(servovalue & 0x800)
  {
    return -(servovalue - 0x800);
  }
  else
  {
    return servovalue;
  }
}

/*
  the value from to the servo is a 12 bit signed value in the range of -2048 to 2047
  so we convert from a formal 16 bit signed value to a 12 bit signed value
*/
int16_t convertcorrection_toservo(int16_t controllervalue)
{
  if (controllervalue < 0)
  {
    return (0x800 + abs(controllervalue));
  }
  else
  {
    return controllervalue;
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

bool create_entities()
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
  RCCHECK(rclc_executor_init(&b.executor, &b.support.context, 1, &b.allocator));
  RCCHECK(rclc_executor_add_service(&b.executor, &b.service, &b.req, &b.res, service_callback));

  return true;
}

bool destroy_entities()
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

void vServoInterfaceTask(void *pvParameters)
{
  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.st.EnableTorque((i + 1), 0);
    b.st.Read((i + 1), 0, b.mBuf[i].bytes, sizeof(MemoryStruct)); // Pass the array to the Read function
    b.mBuf[i].memory.targetLocation = b.mBuf[i].memory.currentLocation;
    b.mBuf[i].memory.torqueSwitch = 0;
  }

  // b.mBuf[0].memory.torqueSwitch = 1;

  while (1)
  {
    // I never figured out why
    // but 9ms results in ~100hz loops
    EXECUTE_EVERY_N_MS(9, ServoLoop(););
    vTaskDelay(1 / portTICK_PERIOD_MS / 10);
  }
}

void vROSTask(void *pvParameters)
{
  while (1)
  {
    switch (b.agentState)
    {
    case b.WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, b.agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? b.AGENT_AVAILABLE : b.WAITING_AGENT;);
      break;
    case b.AGENT_AVAILABLE:
      b.agentState = (true == create_entities()) ? b.AGENT_CONNECTED : b.WAITING_AGENT;
      if (b.agentState == b.WAITING_AGENT)
      {
        destroy_entities();
      };
      break;
    case b.AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, b.agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? b.AGENT_CONNECTED : b.AGENT_DISCONNECTED;);
      if (b.agentState == b.AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&b.executor, RCL_MS_TO_NS(10));
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      break;
    case b.AGENT_DISCONNECTED:
      destroy_entities();
      b.agentState = b.WAITING_AGENT;
      break;
    default:
      break;
    }
  }
}

void setup()
{
  // Configure serial transport
  Serial.begin(b.BAUDRATE);

  Serial1.begin(
      1000000,
      SERIAL_8N1,
      S_RXD, S_TXD,
      false,
      20000,
      127);

  b.st.pSerial = &Serial1;

  set_microros_serial_transports(Serial);

  b.agentState = b.WAITING_AGENT;

  b.servoLoopState = b.RUNNING;

  xTaskCreatePinnedToCore(
      vServoInterfaceTask,
      "ServoInterfaceTask",   // A name just for humans
      10000,                  // Stack size in bytes
      NULL,                   // Parameter passed as input of the task
      1,                      // Priority of the task)
      &b.xTaskServoInterface, // Task handle to keep track of created task
      1);                     // Core where the task should run

  xTaskCreatePinnedToCore(
      vROSTask,
      "ROSTask",   // A name just for humans
      10000,       // Stack size in bytes
      NULL,        // Parameter passed as input of the task
      1,           // Priority of the task)
      &b.xTaskROS, // Task handle to keep track of created task
      0);          // Core where the task should run
}

void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}