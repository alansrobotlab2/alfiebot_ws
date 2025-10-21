"""
MKS SERVO42C Usage Examples

Demonstrates various use cases for the MKSServo42C driver class.
"""

import time
import logging
from mksservo42c_refactored import MKSServo42C, Direction, MotorStatus, EnableStatus


def setup_logging():
    """Configure logging for debug output."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def example_basic_movement():
    """Example 1: Basic movement commands."""
    logger = setup_logging()
    logger.info("=== Basic Movement Example ===")
    
    # Initialize servo
    with MKSServo42C('/dev/ttyUSB0', 38400, logger=logger) as servo:
        # Enable motor
        servo.enable()
        time.sleep(0.2)
        
        # Move forward at moderate speed for 2 seconds
        servo.move(Direction.CW, 50)
        time.sleep(2.0)
        servo.stop()
        
        # Check position
        pulses = servo.read_pulses_received()
        logger.info(f"Position after move: {pulses} pulses")
        
        # Move back to start
        servo.move(Direction.CCW, 50)
        time.sleep(2.0)
        servo.stop()
        
        # Final position
        pulses = servo.read_pulses_received()
        logger.info(f"Final position: {pulses} pulses")


def example_position_control():
    """Example 2: Precise position-based movement."""
    logger = setup_logging()
    logger.info("=== Position Control Example ===")
    
    servo = MKSServo42C('/dev/ttyUSB0', 38400, logger=logger)
    
    try:
        # Set subdivision for smooth movement
        servo.set_subdivision(16)
        servo.enable()
        time.sleep(0.2)
        
        # Move one full rotation (3200 pulses for 1.8° motor, subdivision 16)
        logger.info("Moving one rotation...")
        servo.move_to(Direction.CW, 60, 3200)
        time.sleep(3.0)  # Wait for movement to complete
        
        # Check position
        pulses = servo.read_pulses_received()
        logger.info(f"Position: {pulses} pulses (should be ~3200)")
        
        # Move 90 degrees
        logger.info("Moving 90 degrees...")
        servo.move_to(Direction.CW, 60, 800)
        time.sleep(1.0)
        
        # Return to zero
        logger.info("Returning to start...")
        total_pulses = servo.read_pulses_received()
        servo.move_to(Direction.CCW, 60, abs(total_pulses))
        time.sleep(3.0)
        
    finally:
        servo.stop()
        servo.disable()
        servo.disconnect()


def example_homing_routine():
    """Example 3: Homing with limit switch."""
    logger = setup_logging()
    logger.info("=== Homing Routine Example ===")
    
    def read_limit_switch():
        """Mock limit switch - replace with actual GPIO read."""
        # This is a placeholder - implement actual hardware read
        import random
        return random.random() < 0.1  # Simulate 10% chance of trigger
    
    servo = MKSServo42C('/dev/ttyUSB0', 38400, logger=logger)
    
    try:
        servo.enable()
        time.sleep(0.2)
        
        # Check if already at limit
        if read_limit_switch():
            logger.info("Already at limit, moving away...")
            servo.move(Direction.CW, 30)
            time.sleep(0.5)
            servo.stop()
        
        # Move toward limit switch
        logger.info("Homing...")
        servo.move(Direction.CCW, 40)
        
        timeout = 30.0  # 30 second timeout
        start_time = time.time()
        
        while not read_limit_switch():
            if time.time() - start_time > timeout:
                logger.error("Homing timeout!")
                servo.stop()
                return False
            
            # Check for blocking
            if servo.is_blocked():
                logger.error("Motor blocked during homing!")
                servo.stop()
                return False
            
            time.sleep(0.01)
        
        servo.stop()
        
        # Set this as zero position
        zero_offset = servo.read_pulses_received()
        logger.info(f"Homing complete! Zero offset: {zero_offset}")
        
        return True
        
    finally:
        servo.stop()
        servo.disable()
        servo.disconnect()


def example_linear_actuator():
    """Example 4: Linear actuator control with position tracking."""
    logger = setup_logging()
    logger.info("=== Linear Actuator Example ===")
    
    # Configuration for GT2 belt, 20 teeth, subdivision 16
    PULLEY_TEETH = 20
    GT2_PITCH = 2.0  # mm
    SUBDIVISION = 16
    STEPS_PER_REV = 200 * SUBDIVISION  # 3200
    MM_PER_REV = GT2_PITCH * PULLEY_TEETH  # 40mm
    STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV  # 80 steps/mm
    MAX_HEIGHT_MM = 400  # mm
    
    servo = MKSServo42C('/dev/ttyUSB0', 38400, logger=logger)
    pulses_offset = 0  # Set during homing
    
    def get_height_mm():
        """Get current height in millimeters."""
        pulses = servo.read_pulses_received()
        return (pulses - pulses_offset) / STEPS_PER_MM
    
    def move_to_height_mm(target_mm, speed=60):
        """Move to absolute height in millimeters."""
        if not 0 <= target_mm <= MAX_HEIGHT_MM:
            logger.error(f"Height {target_mm}mm out of range (0-{MAX_HEIGHT_MM})")
            return False
        
        current_mm = get_height_mm()
        delta_mm = target_mm - current_mm
        
        if abs(delta_mm) < 0.5:  # Already at target
            return True
        
        direction = Direction.CW if delta_mm > 0 else Direction.CCW
        logger.info(f"Moving from {current_mm:.1f}mm to {target_mm}mm")
        
        # Use continuous movement with position monitoring
        servo.move(direction, speed)
        
        while True:
            current_mm = get_height_mm()
            remaining = abs(target_mm - current_mm)
            
            if remaining < 0.5:  # 0.5mm tolerance
                servo.stop()
                break
            
            time.sleep(0.01)
        
        final_mm = get_height_mm()
        logger.info(f"Reached {final_mm:.1f}mm")
        return True
    
    try:
        servo.set_subdivision(SUBDIVISION)
        servo.enable()
        time.sleep(0.2)
        
        # Assume homing was done and offset set
        pulses_offset = servo.read_pulses_received()
        
        # Move to different heights
        move_to_height_mm(100)  # Move to 100mm
        time.sleep(1.0)
        
        move_to_height_mm(300)  # Move to 300mm
        time.sleep(1.0)
        
        move_to_height_mm(150)  # Move to 150mm
        time.sleep(1.0)
        
        move_to_height_mm(0)    # Return to bottom
        
    finally:
        servo.stop()
        servo.disable()
        servo.disconnect()


def example_status_monitoring():
    """Example 5: Continuous status monitoring."""
    logger = setup_logging()
    logger.info("=== Status Monitoring Example ===")
    
    servo = MKSServo42C('/dev/ttyUSB0', 38400, logger=logger)
    
    try:
        servo.enable()
        
        # Start movement
        servo.move(Direction.CW, 50)
        
        # Monitor for 5 seconds
        for i in range(50):
            status = servo.get_status_dict()
            
            logger.info(
                f"Pulse: {status['pulses']:6d} | "
                f"Angle: {status['angle']:5d} | "
                f"Error: {status['error']:4d} | "
                f"Enabled: {status['enabled']} | "
                f"Blocked: {status['blocked']}"
            )
            
            # Check for problems
            if status['blocked']:
                logger.warning("Motor blocked! Stopping...")
                servo.stop()
                break
            
            time.sleep(0.1)
        
        servo.stop()
        
    finally:
        servo.disable()
        servo.disconnect()


def example_error_recovery():
    """Example 6: Error handling and recovery."""
    logger = setup_logging()
    logger.info("=== Error Recovery Example ===")
    
    servo = MKSServo42C('/dev/ttyUSB0', 38400, logger=logger)
    
    def recover_from_error():
        """Attempt to recover from error state."""
        logger.info("Attempting recovery...")
        
        try:
            # Stop any movement
            servo.stop()
            time.sleep(0.1)
            
            # Disable and re-enable
            servo.disable()
            time.sleep(1.0)
            servo.enable()
            time.sleep(0.5)
            
            # Verify
            if servo.is_enabled():
                logger.info("Recovery successful")
                return True
            else:
                logger.error("Recovery failed")
                return False
                
        except Exception as e:
            logger.error(f"Recovery exception: {e}")
            return False
    
    try:
        # Simulate normal operation
        servo.enable()
        servo.move(Direction.CW, 60)
        time.sleep(1.0)
        
        # Simulate blocking
        if servo.is_blocked():
            logger.warning("Motor blocked detected!")
            servo.stop()
            recover_from_error()
        
    finally:
        servo.stop()
        servo.disable()
        servo.disconnect()


if __name__ == "__main__":
    """Run examples - uncomment the one you want to test."""
    
    # example_basic_movement()
    # example_position_control()
    # example_homing_routine()
    # example_linear_actuator()
    # example_status_monitoring()
    # example_error_recovery()
    
    print("Uncomment an example function to run it!")
