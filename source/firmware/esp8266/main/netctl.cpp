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
#include "gpio.h"
#include "user_interface.h"
#include "netctl.h"
#include "ioctl.h"

// ----------------------------------------------------------------------------
static String m_host;
static String m_port;
static WiFiServer m_server(500);

// ----------------------------------------------------------------------------
void netctl_init(){
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  m_server.begin();
}

// ----------------------------------------------------------------------------
void netctl_host_set(String l_host){
  m_host = l_host;
}

// ----------------------------------------------------------------------------
void netctl_port_set(String l_port){
  m_port = l_port;
}

// ----------------------------------------------------------------------------
void netctl_wifi_reconnect(){
  wifi_set_opmode(WIFI_STA);
  wifi_station_connect();
}

// ----------------------------------------------------------------------------
void netctl_wifi_wps_run(){

  ioctl_led_blink_pattern_2(IO_PIN_LED_RED, IO_PIN_LED_GREEN, 250);

  WiFi.disconnect(true);
  WiFi.beginWPSConfig();
  WiFi.config(IPAddress(0,0,0,0), WiFi.gatewayIP(), WiFi.subnetMask());

  uint8_t l_connection_status;
  if(WiFi.SSID().length() > 0){
    l_connection_status = 1;
  }
  else{
    l_connection_status = 0;
  }

  if(l_connection_status){
    ioctl_led_blink_pattern_1(IO_PIN_LED_GREEN, 500);
  }
  else{
    ioctl_led_blink_pattern_1(IO_PIN_LED_RED, 500);
  }

  Serial.write((byte)l_connection_status);
}

// ----------------------------------------------------------------------------
void netctl_tcp_send(){

  WiFiClient l_client;
  l_client.setTimeout(50);

  String l_data = Serial.readStringUntil('\0');
  Serial.print((byte)true);

  if(l_client.connect(m_host, m_port.toInt())){
    ioctl_led_on(IO_PIN_LED_GREEN);
    l_client.print(l_data);
    String l_response = l_client.readStringUntil('\0');
    l_client.stop();
    ioctl_led_off(IO_PIN_LED_GREEN);
  }
  else{
    Serial.write((byte)false);
    ioctl_led_blink_pattern_1(IO_PIN_LED_RED, 50);
  }
}

// ----------------------------------------------------------------------------
void netctl_dest_address_get(){
  String l_dest_addr;

  WiFiClient l_client = m_server.available();;
  l_client.setTimeout(100);

  if(l_client.connected()){ 
    if (l_client.available()){
      l_dest_addr = l_client.readStringUntil('\0');
      l_client.print("IP OK");
    }
    ioctl_led_blink_pattern_1(IO_PIN_LED_GREEN, 50);
  }
  else{
    ioctl_led_blink_pattern_1(IO_PIN_LED_RED, 50);
  }

  l_client.stop();

  Serial.write((byte)0xFF);
  Serial.print(l_dest_addr);
  Serial.print('\0');
}


#ifdef __cplusplus
  }
#endif