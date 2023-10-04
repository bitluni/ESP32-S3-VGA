//This example shows how to use Adafruit MQTT library to subscribe to an mqtt topic providing realt time power data
//then displaying it with GfxWrapper to VGA
//check out livestream https://youtube.com/live/meuH0wQfqsE
//
//bitluni

#include "ESP32S3VGA.h"
#include <GfxWrapper.h>
#include <Fonts/FreeMonoBoldOblique24pt7b.h>
#include <Fonts/FreeSerif24pt7b.h>

//                   r,r,r,r,r,  g,g, g, g, g, g,   b, b, b, b, b,   h,v
const PinConfig pins(4,5,6,7,8,  9,10,11,12,13,14,  15,16,17,18,21,  1,2);

//VGA Device
VGA vga;
Mode mode = Mode::MODE_320x240x60;
GfxWrapper<VGA> gfx(vga, mode.hRes, mode.vRes);

#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char* WLAN_SSID = "YOUR_SSID";
const char* WLAN_PASS = "YOUR_PASSWORD";
const char* MQTT_BROKER = "YOUR_BROKER";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PWD = "";
const char* MQTT_TOPIC = "ha/power"; //the topic where to fetch the data

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PWD);
Adafruit_MQTT_Subscribe power = Adafruit_MQTT_Subscribe(&mqtt, MQTT_TOPIC);

//initial setup
void setup()
{
  	Serial.begin(115200);	
	WiFi.begin(WLAN_SSID, WLAN_PASS);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("WiFi connected");
	Serial.println("IP address: "); Serial.println(WiFi.localIP());
 	mqtt.subscribe(&power);	

	vga.bufferCount = 2;
	if(!vga.init(pins, mode, 16)) while(1) delay(1);

	vga.start();
}

void MQTT_connect() 
{
	int8_t ret;

	// Stop if already connected.
	if (mqtt.connected()) {
		return;
	}

	Serial.print("Connecting to MQTT... ");

	uint8_t retries = 3;
	while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
		Serial.println(mqtt.connectErrorString(ret));
		Serial.println("Retrying MQTT connection in 5 seconds...");
		mqtt.disconnect();
		delay(5000);  // wait 5 seconds
		retries--;
		if (retries == 0) {
			// basically die and wait for WDT to reset me
			while (1);
		}
	}
	Serial.println("MQTT Connected!");
}

//the loop is done every frame
void loop()
{
	MQTT_connect();
	static int x = 0;
	Adafruit_MQTT_Subscribe *subscription;
  	if(subscription = mqtt.readSubscription())	//just take what you get no wait
	{
		//nop
	}
	vga.clear(vga.rgb(0x80, 0, 0));
	//using adafruit gfx
	gfx.setFont(&FreeMonoBoldOblique24pt7b);
	gfx.setCursor(100, 100);
	gfx.print("Power");
	gfx.setFont(&FreeSerif24pt7b);
	gfx.setCursor(100, 150);
	gfx.print((char *)power.lastread + String("W"));
	vga.show();
}
