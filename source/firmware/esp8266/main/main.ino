// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
#ifdef __cplusplus
  #include <ESP8266WiFi.h>
  extern "C" {
#endif

// ----------------------------------------------------------------------------
#include "main.h"

ADC_MODE(ADC_VCC);

// ----------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  netctl_init();
  ioctl_init();
  ioctl_led_blink_pattern_1(IO_PIN_LED_GREEN, 3000);
  //wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  Serial.write((byte)true);        
}

// ----------------------------------------------------------------------------
void loop(){
  uint8_t l_sleep_flag = pwrctl_sleep_flag_get();
  if(l_sleep_flag == 1){
    netctl_wifi_reconnect();
    pwrctl_sleep_flag_clear();
  }
  cmdctl_loop();
}

#ifdef __cplusplus
  }
#endif