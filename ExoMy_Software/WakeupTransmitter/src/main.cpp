// Main functions for MyExoMy wakeup transmitter.

#include "Arduino.h"
#include <WiFi.h>


const char *WIFI_SSID = "wifiwifiwifi_guest";
const char *WIFI_PASSWD = "qwertyui";


void connectToWifi()
{
  //connect to WiFi
  Serial.printf("Connecting to %s ", WIFI_SSID);
  WiFi.begin((const char *)WIFI_SSID, (const char *)WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
}


void setup()
{
  Serial.begin(115200);

  Serial.println("Start MyExoMy wkaeup transmitter!");
}


void loop()
{

}
