// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
#ifdef __cplusplus
  #include <ESP8266WiFi.h>
  extern "C" {
#endif

// ----------------------------------------------------------------------------
#include <string.h>
#include "cmdctl.h"
#include "ioctl.h"
#include "netctl.h"
#include "pwrctl.h"

// ----------------------------------------------------------------------------
#define CMD_GET_WLAN_STATUS 0x02
#define CMD_GET_PC_STATUS   0x03
#define CMD_SEND_KEY        0x04
#define CMD_SET_DEST_IP     0x05
#define CMD_RUN_WPS         0x0A
#define CMD_GET_DEST_IP     0x0B
#define CMD_RESET           0x64
#define CMD_SLEEP           0x65
#define CMD_GET_LOCAL_IP    0x66
#define CMD_GET_VBAT        0x67

// ----------------------------------------------------------------------------
void cmdctl_loop(){

  uint8_t l_command = Serial.read();

  switch (l_command){

    case CMD_GET_WLAN_STATUS:
    {
      uint8_t l_wifi_status;
      if(WiFi.status() == WL_CONNECTED)
      {
        l_wifi_status = 1;
      }
      else{
        l_wifi_status = 0;
      }

      Serial.write(l_wifi_status);

      if(l_wifi_status){
        ioctl_led_blink_pattern_1(IO_PIN_LED_GREEN, 500);
      }
      else{
        ioctl_led_blink_pattern_1(IO_PIN_LED_RED, 500);
      }
      break;
    }

    case CMD_GET_PC_STATUS:
    {
      Serial.write((byte)false);
      break;
    }

    case CMD_SEND_KEY:
    {
      Serial.write((byte)true);
      netctl_tcp_send();
      break;
    }
  
    case CMD_SET_DEST_IP:
    {
      Serial.write((byte)true);
      String l_host = Serial.readStringUntil(':');
      String l_port = Serial.readString();
      netctl_host_set(l_host);
      netctl_port_set(l_port);
      Serial.write((byte)true);
      break;
    }
  
    case CMD_RUN_WPS:
    {
      netctl_wifi_wps_run();
      break;
    }
  
    case CMD_GET_DEST_IP:
    {
      netctl_dest_address_get();
      break;
    }
  
    // case RESET:
    //  ESP.restart();
    //  break;
  
    case CMD_SLEEP:
    {
      pwrctl_sleep_run();
      break;
    }
  
    // case GET_LOCAL_IP:
    //  Serial.println(WiFi.localIP());
    //  Serial.println(WiFi.gatewayIP());
    //  Serial.println(WiFi.subnetMask());
    //  break;
  
    // case GET_VBAT:
    //  Serial.println((float)ESP.getVcc() / 1024); // getVcc()/1024.0
    //  break;
  
    // default:
    //   break;

    }
}

#ifdef __cplusplus
  }
#endif