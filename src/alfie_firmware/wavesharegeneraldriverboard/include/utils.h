#pragma once

#include <Arduino.h>

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
