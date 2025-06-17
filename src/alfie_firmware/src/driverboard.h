


class DriverBoard
{
public:
    static constexpr uint8_t MAX_SERVOS = 10;

    DriverBoard()
        : micro_ros_init_successful(false),
          agentState(WAITING_AGENT),
          servoLoopState(RUNNING),
          start_time(0),
          duration(0)
    {
        for (uint8_t i = 0; i < MAX_SERVOS; ++i)
        {
            IDS[i] = i + 1;
            servoCMDIDS[i] = i + 1;
        }
    }

    alfie_msgs__msg__DriverState driverState;

    rcl_publisher_t publisher;
    rcl_subscription_t subscriber;
    rcl_service_t service;

    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    alfie_msgs__srv__ServoService_Request req;
    alfie_msgs__srv__ServoService_Response res;

    SMS_STS st;

    static constexpr int BAUDRATE = 921600;

    TaskHandle_t xTaskServoInterface = NULL;
    TaskHandle_t xTaskROS = NULL;
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

    MemoryReplyBuf mBuf[MAX_SERVOS];

    uint8_t IDS[MAX_SERVOS];
    uint8_t servoCMDIDS[MAX_SERVOS];

    MemoryReplyBuf buf;
    MemoryReplyBuf rbuf;

    u8 torquecommandbuf[MAX_SERVOS];
    u8 servocommandbuf[3 * MAX_SERVOS];
};
