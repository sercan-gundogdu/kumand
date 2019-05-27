// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif

// ----------------------------------------------------------------------------
#include <stdint.h>

// ----------------------------------------------------------------------------
void pwrctl_init();

// ----------------------------------------------------------------------------
void pwrctl_sleep_flag_clear();

// ----------------------------------------------------------------------------
uint8_t pwrctl_sleep_flag_get();

// ----------------------------------------------------------------------------
void pwrctl_sleep_run();

#ifdef __cplusplus
  }
#endif