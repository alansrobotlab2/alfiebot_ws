#pragma once
#include <Preferences.h>
#include "IMU.h"

class DriverBoard
{
private:
    Preferences boardconfig;
    uint8_t boardid;  // Board ID loaded from EEPROM (0 or 1)

public:
    static constexpr uint8_t MAX_SERVOS = 10;

    DriverBoard()
        : micro_ros_init_successful(false),
          agentState(WAITING_AGENT),
          servoLoopState(RUNNING),
          start_time(0),
          duration(0),
          last_drivercmd_time(0),
          boardid(999)  // Default uninitialized value
    {
        for (uint8_t i = 0; i < MAX_SERVOS; ++i)
        {
            IDS[i] = i + 1;
            servoCMDIDS[i] = i + 1;
        }
    }

    // Load board ID from EEPROM using Preferences library
    void loadBoardId() {
        boardconfig.begin("boardconfig", true);  // Open in read-only mode
        boardid = boardconfig.getUInt("boardid", 255);  // Default 255 if not found
        boardconfig.end();
    }

    // Get board-specific configuration based on boardid
    const char* getNodeName() const {
        return (boardid == 0) ? "gdb0" : "gdb1";
    }

    const char* getStatePublisher() const {
        return (boardid == 0) ? "gdb0state" : "gdb1state";
    }

    const char* getCmdSubscriber() const {
        return (boardid == 0) ? "gdb0cmd" : "gdb1cmd";
    }

    const char* getServoService() const {
        return (boardid == 0) ? "gdb0servoservice" : "gdb1servoservice";
    }

    uint8_t getNumServos() const {
        return (boardid == 0) ? 10 : 7;
    }

    uint8_t getBoardId() const {
        return boardid;
    }

    alfie_msgs__msg__GDBState driverState;
    alfie_msgs__msg__GDBCmd driverCmd;

    rcl_publisher_t publisher;
    rcl_subscription_t subscriber;
    rcl_service_t service;

    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    alfie_msgs__srv__GDBServoService_Request req;
    alfie_msgs__srv__GDBServoService_Response res;

    SMS_STS st;

    TaskHandle_t xTaskServoInterface = NULL;
    TaskHandle_t xTaskHardwareInterface = NULL;
    TaskHandle_t xTaskROS = NULL;
    TaskHandle_t xTask100Hz = NULL;
    TaskHandle_t xTaskMotorEncoder = NULL;
    TaskHandle_t xTaskIMU = NULL;
    const UBaseType_t taskPriority = 0;

    bool micro_ros_init_successful;

    unsigned long start_time1 = 0;
    unsigned long start_time2 = 0;
    uint8_t duration1 = 0;
    uint8_t duration2 = 0;

    enum AgentStates
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } agentState;

    enum ServoLoopStates
    {
        RUNNING,
        REQUEST_STOP,
        STOPPED
    } servoLoopState;

    unsigned long start_time;
    uint8_t duration;

    // Watchdog variables
    volatile unsigned long last_drivercmd_time;

    volatile bool drivercmd_timeout = true;

    MemoryReplyBuf mBuf[MAX_SERVOS] = {};  // Zero-initialize the array

    uint8_t IDS[MAX_SERVOS];
    uint8_t servoCMDIDS[MAX_SERVOS];

    MemoryReplyBuf buf = {};   // Zero-initialize
    MemoryReplyBuf rbuf = {};  // Zero-initialize

    uint8_t torquecommandbuf[MAX_SERVOS] = {};    // Zero-initialize
    uint8_t servocommandbuf[SERVOCOMMANDPACKETSIZE * MAX_SERVOS] = {}; // Zero-initialize

    int16_t drivercmdbuf[2] = {};  // Zero-initialize

    // Velocity control targets (m/s) - commanded from ROS
    float A_target_velocity = 0.0f;
    float B_target_velocity = 0.0f;

    // PID control state - Motor A
    float A_velocity_error_integral = 0.0f;
    float A_velocity_error_previous = 0.0f;
    unsigned long A_pid_last_update_time = 0;

    // PID control state - Motor B
    float B_velocity_error_integral = 0.0f;
    float B_velocity_error_previous = 0.0f;
    unsigned long B_pid_last_update_time = 0;

    // IMU related variables
    // Set the values for the Roll, Pitch, and Yaw corners, as well as the Temp temperature.
    // Roll represents the roll angle of rotation around the X-axis, Pitch represents the pitch angle of rotation around the Y-axis, and Yaw represents the yaw angle of rotation around the Z-axis.
    int16_t IMU_Roll = 100;
    int16_t IMU_Pitch = 100;
    int16_t IMU_Yaw = 100;
    int16_t IMU_Temp = 100;

    EulerAngles stAngles;                    // Create structure variable stAngles to store the three angle data
    IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;  // For storing raw gyroscope data
    IMU_ST_SENSOR_DATA_FLOAT stAccelRawData; // Stores raw accelerometer data
    IMU_ST_SENSOR_DATA_FLOAT stMagnRawData;  // Storing magnetometer data in Tesla (T)

    volatile uint8_t board_temp = 0;

    volatile bool shoulder_limit_switch = false;

    // Used to calculate the number of level changes of one of the Hall sensors of the encoder during the "interval" time (in ms).
    // Since RISING is used later to initialize the interrupt, it is specifically the number of times the low level changes to a high level.
    volatile long B_wheel_pulse_count = 0;
    volatile long A_wheel_pulse_count = 0;

    // Motor dynamics tracking - Motor A
    long A_last_pulse_count = 0;
    float A_velocity = 0.0f;  // pulses per second
    float A_last_velocity = 0.0f;
    float A_acceleration = 0.0f;  // pulses per second^2
    bool A_is_moving = false;
    unsigned long A_last_update_time = 0;

    // Motor dynamics tracking - Motor B
    long B_last_pulse_count = 0;
    float B_velocity = 0.0f;  // pulses per second
    float B_last_velocity = 0.0f;
    float B_acceleration = 0.0f;  // pulses per second^2
    bool B_is_moving = false;
    unsigned long B_last_update_time = 0;

    float pollservostatusduration = 0.0f;
    float updateservoidleduration = 0.0f;
    float updateservoactiveduration = 0.0f;
    float updatemotorcommandduration = 0.0f;
    float imuupdateduration = 0.0f;
};

