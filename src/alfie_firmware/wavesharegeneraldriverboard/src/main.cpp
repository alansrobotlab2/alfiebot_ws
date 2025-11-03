#include "config.h"
#include "driverboard.h"
#include "ros_control.h"
#include "motor_control.h"
#include "imu_control.h"
#include "servo_control.h"
#include "shoulder_control.h"
#include "utils.h"

DriverBoard b;

void vServoInterfaceTask(void *pvParameters)
{

  int retval = 0;

  for (int i = 0; i < NUMSERVOS; i++)
  {
    b.st.EnableTorque((i + 1), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Read servo memory into mBuf[i].bytes (raw memory layout expected by protocol)
    retval = b.st.Read((i + 1), 0, b.mBuf[i].bytes, sizeof(MemoryStruct));
    b.mBuf[i].memory.unassigned0 = retval; // to indicate if read was successful or not
    b.mBuf[i].memory.targetLocation = b.mBuf[i].memory.currentLocation;
    b.mBuf[i].memory.torqueSwitch = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();

  b.servoLoopState = b.RUNNING;

  while (1)
  {
    // Wait for the next cycle (precise 100 Hz timing)
    xTaskDelayUntil(&xLastWakeTime, xFrequency);

    // we need to coordinate with service requests
    switch (b.servoLoopState)
    {
    case b.RUNNING:
      TIME_FUNCTION_MS(updateServoStatus(), b.pollservostatusduration);
      TIME_FUNCTION_MS(updateServoActive(), b.updateservoactiveduration);
      TIME_FUNCTION_MS(updateServoIdle(), b.updateservoidleduration);


      break;
    case b.REQUEST_STOP:
      b.servoLoopState = b.STOPPED;
      break;
    case b.STOPPED:
      continue; // skip the rest of the loop
      break;
    default:
      break;
    }
  }
}

void vHardwareInterfaceTask(void *pvParameters)
{

  int retval = 0;

  initMotors();
  imuInit();

  //  Set the interrupt and the corresponding callback function to call the B_wheel_pulse function when BEBCB changes from low to high (RISING).
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  // Set the interrupt and the corresponding callback function to call the A_wheel_pulse function when AEBCB changes from low to high (RISING).
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Wait for the next cycle (precise 100 Hz timing)
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    calculateMotorDynamics();
    generateLowStatus();
    TIME_FUNCTION_MS(getIMUData(), b.imuupdateduration);
    driveMotors();
  }
}

  void vROSTask(void *pvParameters)
  {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
      // Wait for the next cycle (precise 100 Hz timing)
      xTaskDelayUntil(&xLastWakeTime, xFrequency);

      switch (b.agentState)
      {
      case b.WAITING_AGENT:
        // Every 500ms, check if the micro-ROS agent is available and update agentState accordingly.
        EXECUTE_EVERY_N_MS(100, b.agentState = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? b.AGENT_AVAILABLE : b.WAITING_AGENT;);
        vTaskDelay(pdMS_TO_TICKS(50));
        break;
      case b.AGENT_AVAILABLE:
        b.agentState = (true == create_ros_entities()) ? b.AGENT_CONNECTED : b.WAITING_AGENT;
        if (b.agentState == b.WAITING_AGENT)
        {
          destroy_ros_entities();
          vTaskDelay(pdMS_TO_TICKS(50));
        };
        break;
      case b.AGENT_CONNECTED:
        // Check connection health every 200ms
        EXECUTE_EVERY_N_MS(200, b.agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? b.AGENT_CONNECTED : b.AGENT_DISCONNECTED;);
        RCSOFTCHECK(rcl_publish(&b.publisher, &b.driverState, NULL));

        // Process ROS callbacks (commands) - give it 1ms to process queued messages
        rclc_executor_spin_some(&b.executor, RCL_MS_TO_NS(1));

        // Watchdog: Check if GDBCmd timeout occurred
        if (b.last_drivercmd_time > 0 && (millis() - b.last_drivercmd_time) > WATCHDOG_TIMEOUT_MS)
        {
          b.drivercmd_timeout = true;
          disableAllServoTorques();
          disableAllMotors();
        }
        break;
      case b.AGENT_DISCONNECTED:
        destroy_ros_entities();
        vTaskDelay(pdMS_TO_TICKS(100));
        // ESP.restart();
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
    Serial.begin(BAUDRATE);

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
        25000,                  // Stack size in bytes
        NULL,                   // Parameter passed as input of the task
        1,                      // Priority of the task)
        &b.xTaskServoInterface, // Task handle to keep track of created task
        1);                     // Core where the task should run

    xTaskCreatePinnedToCore(
        vHardwareInterfaceTask,
        "HardwareInterfaceTask",   // A name just for humans
        25000,                     // Stack size in bytes
        NULL,                      // Parameter passed as input of the task
        1,                         // Priority of the task)
        &b.xTaskHardwareInterface, // Task handle to keep track of created task
        1);                        // Core where the task should run

    xTaskCreatePinnedToCore(
        vROSTask,
        "ROSTask",   // A name just for humans
        25000,       // Stack size in bytes
        NULL,        // Parameter passed as input of the task
        1,           // Priority of the task)
        &b.xTaskROS, // Task handle to keep track of created task
        0);          // Core where the task should run
  }

  void loop()
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // void setup1()
  // {
  //   // Configure serial transport
  //   Serial.begin(921600);

  //   Serial1.begin(
  //       1000000,
  //       SERIAL_8N1,
  //       S_RXD, S_TXD,
  //       false,
  //       20000,
  //       127);

  //   b.st.pSerial = &Serial1;

  //   vTaskDelay(pdMS_TO_TICKS(4000));
  // }

  // void loop1()
  // {

  //   int count = sizeof(MemoryStruct);
  //   int retval = 0;
  //   b.st.EnableTorque((1), 0);
  //   retval = b.st.Read(1, 0, b.mBuf[0].bytes, count);
  //   if (retval == 0)
  //   {
  //     Serial.print("Error: Read failed with code ");
  //     Serial.println(retval);
  //   }

  //   Serial.print("Read Firmware Major: ");
  //   Serial.println(b.mBuf[0].memory.firmwareMajor);
  //   Serial.println();
  //   Serial.println("Read bytes:");
  //   for (int i = 0; i < count; i++)
  //   {
  //     if (b.mBuf[0].bytes[i] < 0x10)
  //       Serial.print("0");
  //     Serial.print(b.mBuf[0].bytes[i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.println();

  //   vTaskDelay(pdMS_TO_TICKS(2000));
  // }