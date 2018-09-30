#include <ESP8266WiFi.h>

extern "C" {
  #include "gpio.h"
}

extern "C" {
  #include "user_interface.h"
}

#define wake		14
#define green		12
#define red			13

void sendKey(void);
void getDestinationIP(void);
bool runWPS(void);
void GPIOInit(void);
void ledIndication(byte pin, uint ms);
void ledIndication(byte pin1, byte pin2, uint ms);
void goSleep(void);

String host;
String port;
byte command;
byte sleep = 0;

enum COMMANDS {	
	GET_WLAN_STATUS = 0x02, 
	GET_PC_STATUS, 
	SEND_KEY, 
	SET_DESTINATION_IP, 
	RUN_WPSPBC = 0x0A, 
	GET_DESTINATION_IP, 
	RESET = 0x64, 
	SLEEP,
	GET_LOCAL_IP, 
	GET_VBAT
};

ADC_MODE(ADC_VCC);
WiFiServer server(500);

void setup(){
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	WiFi.begin();
	server.begin();
	GPIOInit();
	ledIndication(green, 3000);
	//wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
	Serial.write((byte)true);        
}

void loop(){
	if (sleep){
		wifi_set_opmode(WIFI_STA);
		wifi_station_connect();
		sleep = 0;
	}
	command = Serial.read();
	switch (command){
    	case GET_WLAN_STATUS:
    		Serial.write((WiFi.status() != WL_CONNECTED) ? (byte)false : (byte)true);
    		(WiFi.status() != WL_CONNECTED) ? ledIndication(red, 500) : ledIndication(green, 500);
    		break;

    	case GET_PC_STATUS:
    		Serial.write((byte)false);
    		break;

    	case SEND_KEY:
    		Serial.write((byte)true);
    		sendKey();
    		break;

    	case SET_DESTINATION_IP:
    		Serial.write((byte)true);
    		host = Serial.readStringUntil(':');
			port = Serial.readString();
			Serial.write((byte)true);
			break;

		case RUN_WPSPBC:
			runWPSPBC();
    		break;

    	case GET_DESTINATION_IP:
			getDestinationIP();
    		break;

    	case RESET:
    		ESP.restart();
    		break;

    	case SLEEP:
    		goSleep();
    		break;

    	case GET_LOCAL_IP:
    		Serial.println(WiFi.localIP());
    		Serial.println(WiFi.gatewayIP());
    		Serial.println(WiFi.subnetMask());
    		break;

    	case GET_VBAT:
    		Serial.println((float)ESP.getVcc() / 1024); // getVcc()/1024.0
    		break;

    	default:
    		break;

		}
}

void sendKey(){
	WiFiClient client;
	client.setTimeout(50);
	String data = Serial.readStringUntil('\0');
	Serial.print((byte)true);
	if(client.connect(host, port.toInt())){
		digitalWrite(green, HIGH);
		client.print(data);
		String response = client.readStringUntil('\0');
		client.stop();
		digitalWrite(green, LOW);
		
	}
	else{
		Serial.write((byte)false);
		digitalWrite(red, HIGH);
		delay(50);
		digitalWrite(red, LOW);
	}
}

void getDestinationIP(){
	String ip;
	String clPort;
	WiFiClient client = server.available();;
	client.setTimeout(100);
	if(client.connected()){ 
		if (client.available()){
			ip = client.readStringUntil('\0');
			client.print("IP OK");
		}
		ledIndication(green, 50);
	}
	else{
		ledIndication(red, 50);
	}
	client.stop();
	Serial.write((byte)0xFF);
	Serial.print(ip);
	Serial.print('\0');
}

bool runWPSPBC(){
	ledIndication(red, green, 250);
	WiFi.disconnect(true);
	WiFi.beginWPSConfig();
	WiFi.config(IPAddress(0,0,0,0), WiFi.gatewayIP(), WiFi.subnetMask());
	bool status = (WiFi.SSID().length() > 0) ? (byte)true : (byte)false ;
	status ? ledIndication(green, 500) : ledIndication(red, 500);
	Serial.write((byte)status);
	return status;
}

void GPIOInit(){
	pinMode(green, OUTPUT);
	pinMode(red, OUTPUT);
	//pinMode(wake, INPUT);
	digitalWrite(green, LOW);
	digitalWrite(red, LOW);
}

void ledIndication(byte pin, uint ms){
	digitalWrite(pin, HIGH);
	delay(ms);
	digitalWrite(pin, LOW);
}

void ledIndication(byte pin1, byte pin2, uint ms){
	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, HIGH);
	delay(ms);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
}
void goSleep(){
	sleep = 1;
	wifi_station_disconnect();
  	wifi_set_opmode(NULL_MODE);
	delay(250);
	wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
	gpio_pin_wakeup_enable(GPIO_ID_PIN(wake), GPIO_PIN_INTR_LOLEVEL); // or LO/ANYLEVEL, no change GPIO_PIN_INTR_LOLEVEL
    wifi_fpm_open(); 
    wifi_fpm_do_sleep(0xFFFFFFF);
    delay(100);
}