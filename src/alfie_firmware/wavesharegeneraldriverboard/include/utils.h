#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * RateLimiter - Custom timing class that enforces a minimum period between loop iterations.
 * Unlike xTaskDelayUntil, this class does NOT "catch up" if iterations run long.
 * It guarantees a minimum delay between iterations, preventing burst behavior.
 * 
 * Usage:
 *   RateLimiter limiter(10);  // 10ms minimum period (100Hz max)
 *   while(1) {
 *     limiter.waitForNextCycle();
 *     // ... do work ...
 *   }
 */
class RateLimiter {
private:
    unsigned long periodMs;
    unsigned long lastIterationTime;
    
public:
    /**
     * Constructor
     * @param periodMs The minimum period in milliseconds between iterations
     */
    RateLimiter(unsigned long periodMs) : periodMs(periodMs), lastIterationTime(0) {}
    
    /**
     * Wait until the minimum period has elapsed since the last iteration.
     * If the period has already elapsed, returns immediately without delay.
     * This prevents "catch up" behavior - we never run faster than 1/periodMs Hz.
     */
    void waitForNextCycle() {
        unsigned long now = millis();
        
        // Calculate time elapsed since last iteration
        unsigned long elapsed = now - lastIterationTime;
        
        // If we haven't waited long enough, delay the remaining time
        if (elapsed < periodMs) {
            unsigned long remainingMs = periodMs - elapsed;
            // Use a spin-wait for sub-millisecond precision at the end
            // vTaskDelay has ~1ms granularity which causes timing drift
            if (remainingMs > 1) {
                vTaskDelay(pdMS_TO_TICKS(remainingMs - 1));
            }
            // Spin-wait for the remaining time for precise timing
            while ((millis() - lastIterationTime) < periodMs) {
                // Yield to other tasks but keep checking
                taskYIELD();
            }
        }
        
        // Mark the start of this iteration AFTER the wait completes
        // Use current time to prevent drift accumulation
        lastIterationTime = millis();
    }
    
    /**
     * Reset the timer (useful after state changes or initialization)
     */
    void reset() {
        lastIterationTime = millis();
    }
};

/**
 * Macro to measure function execution time in microseconds and store as float milliseconds
 * Optimal for measurements from microseconds up to ~1 second
 * 
 * Usage: TIME_FUNCTION_MS(someFunction(arg1, arg2), duration_var);
 * 
 * @param func_call The function call to time
 * @param duration_var Float variable to store the duration in milliseconds
 */
#define TIME_FUNCTION_MS(func_call, duration_var) \
    do { \
        unsigned long start_time = micros(); \
        func_call; \
        unsigned long end_time = micros(); \
        duration_var = (end_time - start_time) / 1000.0f; \
    } while(0)
