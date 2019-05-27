// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
#ifdef __cplusplus
  #include <ESP8266WiFi.h>
  extern "C" {
#endif

// ----------------------------------------------------------------------------
#include "gpio.h"
#include "user_interface.h"
#include "netctl.h"
#include "ioctl.h"

// ----------------------------------------------------------------------------
static volatile uint8_t m_sleep_flag = 0;

// ----------------------------------------------------------------------------
void pwrctl_sleep_flag_set(){
  m_sleep_flag = 1;
}

// ----------------------------------------------------------------------------
void pwrctl_sleep_flag_clear(){
  m_sleep_flag = 0;
}

// ----------------------------------------------------------------------------
uint8_t pwrctl_sleep_flag_get(){
  return m_sleep_flag;
}

// ----------------------------------------------------------------------------
void pwrctl_sleep_run(){
  m_sleep_flag = 1;
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  delay(250);
  wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
  gpio_pin_wakeup_enable(GPIO_ID_PIN(IO_PIN_SIG_WAKE), GPIO_PIN_INTR_LOLEVEL);
  // or LO/ANYLEVEL, no change GPIO_PIN_INTR_LOLEVEL
  wifi_fpm_open(); 
  wifi_fpm_do_sleep(0xFFFFFFF);
  delay(100);
}

#ifdef __cplusplus
  }
#endif