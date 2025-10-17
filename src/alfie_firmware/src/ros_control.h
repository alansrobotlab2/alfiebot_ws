#ifndef _ROS_CONTROL_H_
#define _ROS_CONTROL_H_

#include "config.h"

/**
 * @brief Creates and initializes all micro-ROS entities for the driver node
 * 
 * Establishes the complete micro-ROS infrastructure including:
 * - ROS 2 support system and allocator
 * - Node with configured name (NODENAME)
 * - Publisher for GDBState messages (best-effort QoS)
 * - Subscriber for GDBCmd messages (default QoS)
 * - Service for GDBServoService requests/responses
 * - Executor with 2 handles (subscriber + service)
 * 
 * The executor is configured to handle incoming subscription messages and
 * service requests through registered callbacks.
 * 
 * @return true if all entities created successfully
 * @return false if any entity creation fails (triggers error_loop via RCCHECK)
 * @note Should be called once after micro-ROS transport is established
 * @note Uses RCCHECK macro which calls error_loop() on failure
 * @see destroy_ros_entities() for cleanup
 * @see NODENAME, STATEPUBLISHER, CMDSUBSCRIBER, SERVOSERVICE for entity names
 */
bool create_ros_entities();


/**
 * @brief Destroys all micro-ROS entities and cleans up resources
 * 
 * Tears down the micro-ROS infrastructure in reverse order of creation:
 * - Sets destroy session timeout to 0 for immediate cleanup
 * - Finalizes publisher, subscriber, service
 * - Finalizes executor, support system, and node
 * 
 * This should be called before re-creating entities or shutting down the node.
 * 
 * @return true when cleanup completes (return value currently unused)
 * @note Called when reconnecting to ROS or during shutdown
 * @note Errors are captured but not currently handled
 * @see create_ros_entities() for entity creation
 */
bool destroy_ros_entities();


/**
 * @brief Callback for processing incoming GDBCmd messages from ROS subscription
 * 
 * Handles incoming command messages by:
 * 1. Updating watchdog timestamp to prevent timeout
 * 2. Clearing timeout flag
 * 3. Extracting motor PWM commands for both motors
 * 4. Extracting servo commands (torque, acceleration, position) for all servos
 * 5. Updating local command buffers and servo memory structures
 * 
 * This callback is invoked automatically by the executor when new GDBCmd
 * messages arrive on the subscribed topic.
 * 
 * @param msgin Pointer to incoming alfie_msgs__msg__GDBCmd message
 * @note Called by executor in response to new subscription data
 * @note Updates global DriverBoard members: last_drivercmd_time, drivercmd_timeout, drivercmdbuf, mBuf
 * @see create_ros_entities() for callback registration
 */
void subscription_callback(const void *msgin);


/**
 * @brief Callback for processing GDBServoService requests from ROS clients
 * 
 * Handles servo memory read/write service requests:
 * - Operation 'R' (Read): Populates response with servo memory map data
 * - Operation 'W' (Write): Updates servo memory at specified address, then returns current state
 * - Special handling for address 31 (position correction) with sign bit encoding
 * - Word (16-bit) vs byte (8-bit) write handling based on address
 * 
 * The service allows direct access to servo control registers for configuration
 * and monitoring purposes.
 * 
 * @param request Pointer to incoming alfie_msgs__srv__GDBServoService_Request
 * @param response Pointer to outgoing alfie_msgs__srv__GDBServoService_Response
 * @note Called by executor when service is invoked
 * @note Updates mBuf for write operations
 * @see populate_service_response() for response formatting
 * @see isWord() for address type checking
 */
void service_callback(const void *request, void *response);


/**
 * @brief Populates servo service response with complete memory map from specified servo
 * 
 * Extracts all servo memory registers from the local buffer (mBuf) for the
 * specified servo and copies them into the service response structure. This
 * includes configuration parameters, status data, and operational values
 * covering the complete servo memory map (approximately 47+ registers).
 * 
 * @param servo Servo ID (1-based index, e.g., 1 for first servo)
 * @param res Pointer to alfie_msgs__srv__GDBServoService_Response to populate
 * @note Servo parameter is 1-based but internally converted to 0-based array index
 * @note Reads from global mBuf array
 * @see service_callback() for usage context
 */
void populate_service_response(int servo, alfie_msgs__srv__GDBServoService_Response *res);


/**
 * @brief Checks if a servo memory address holds a 16-bit word value
 * 
 * Determines whether the given memory address contains a 2-byte (16-bit) value
 * versus a single byte. This is used to correctly parse and write multi-byte
 * servo parameters.
 * 
 * Word addresses include: 9, 11, 16, 24, 28, 31, 42, 44, 46, 48
 * Address 31 (position correction) uses special encoding with bit 11 as sign bit.
 * 
 * @param a Servo memory address to check
 * @return true if address holds a 16-bit word
 * @return false if address holds a single byte
 * @note Critical for correct service write operations
 * @see service_callback() for usage in write operations
 */
bool isWord(uint8_t a);


/**
 * @brief Generates and populates the GDBState message with current system status
 * 
 * Aggregates data from all subsystems into a comprehensive state message including:
 * - Motor state (PWM commands, encoder counts, velocity, acceleration, movement flags)
 * - Servo state (all servos: position, torque, temperature, voltage, current, etc.)
 * - IMU data (orientation, angular velocity, linear acceleration)
 * - Magnetometer data (magnetic field in Tesla)
 * - Shoulder limit switch state
 * - Performance diagnostics (polling durations)
 * - Board temperature
 * 
 * The populated message is stored in b.driverState ready for publishing.
 * Publishing is currently commented out but message is prepared for transmission.
 * 
 * @note Called periodically from main loop to prepare state updates
 * @note Does not publish - publishing should be done separately
 * @note Updates global b.driverState structure
 * @see GDBState message definition in alfie_msgs
 */
void generateLowStatus();


/**
 * @brief Error handling loop with visual motor feedback
 * 
 * Infinite loop that alternates motor PWM at low values to provide visual
 * indication of a critical error. Motors pulse forward and backward repeatedly
 * at ±16 PWM with 200ms intervals.
 * 
 * This function never returns and effectively halts normal operation.
 * Triggered by RCCHECK macro failures during ROS entity creation.
 * 
 * @note Never returns - blocks forever
 * @note Called automatically by RCCHECK macro on RCL errors
 * @warning Provides only basic error indication - no diagnostic information
 * @see RCCHECK macro for automatic error checking
 */
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