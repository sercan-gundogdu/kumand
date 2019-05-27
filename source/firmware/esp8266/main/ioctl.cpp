// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
#ifdef __cplusplus
  #include <ESP8266WiFi.h>
  extern "C" {
#endif

// ----------------------------------------------------------------------------
#include "ioctl.h"

// ----------------------------------------------------------------------------
void ioctl_pin_state_set(uint8_t l_io_pin_num);
void ioctl_pin_state_clear(uint8_t l_io_pin_num);
void ioctl_pin_mode_set(uint8_t l_io_pin_num, uint8_t l_io_pin_mode);

// ----------------------------------------------------------------------------
void ioctl_init(){
  ioctl_pin_mode_set(IO_PIN_LED_GREEN, OUTPUT);
  ioctl_pin_mode_set(IO_PIN_LED_RED, OUTPUT);

  ioctl_pin_state_clear(IO_PIN_LED_GREEN);
  ioctl_pin_state_clear(IO_PIN_LED_RED);
}

// ----------------------------------------------------------------------------
void ioctl_pin_state_set(uint8_t l_io_pin_num){
  digitalWrite(l_io_pin_num, HIGH);
}

// ----------------------------------------------------------------------------
void ioctl_pin_state_clear(uint8_t l_io_pin_num){
  digitalWrite(l_io_pin_num, LOW);
}

// ----------------------------------------------------------------------------
void ioctl_pin_mode_set(uint8_t l_io_pin_num, uint8_t l_io_pin_mode){
  pinMode(l_io_pin_num, l_io_pin_mode);
}

// ----------------------------------------------------------------------------
void ioctl_led_on(uint8_t l_io_pin_num){
  ioctl_pin_state_set(l_io_pin_num);
}

// ----------------------------------------------------------------------------
void ioctl_led_off(uint8_t l_io_pin_num){
  ioctl_pin_state_clear(l_io_pin_num);
}

// ----------------------------------------------------------------------------
void ioctl_led_blink_pattern_1(uint8_t l_io_pin_1_num, uint8_t l_io_delay_ms){
  ioctl_pin_state_set(l_io_pin_1_num);
  delay(l_io_delay_ms);
  ioctl_pin_state_clear(l_io_pin_1_num);
}

// ----------------------------------------------------------------------------
void ioctl_led_blink_pattern_2(
  uint8_t l_io_pin_1_num, uint8_t l_io_pin_2_num, uint8_t l_io_delay_ms
){
  ioctl_pin_state_set(l_io_pin_1_num);
  ioctl_pin_state_set(l_io_pin_2_num);
  delay(l_io_delay_ms);
  ioctl_pin_state_clear(l_io_pin_1_num);
  ioctl_pin_state_clear(l_io_pin_2_num);
}

#ifdef __cplusplus
  }
#endif