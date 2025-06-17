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
 