// Main functions for MyExoMy wakeup transmitter.
// Note that for the below code to compile a patch has to be made in .pio\libdeps\esp32dev\esp32_https_server\src\HTTPConnection.hpp, see https://github.com/fhessel/esp32_https_server/issues/143
#include "Arduino.h"
#include <WiFi.h>
#include <FS.h>
#include <HTTPSServer.hpp>
#include <SSLCert.hpp>
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
#include <SPIFFS.h>

using namespace httpsserver;

const char *SSID = "wifiwifiwifi_guest";
const char *PASSWORD = "qwertyui";
const int PORTNUMBER = 55554;
const int WAKEUP_PIN = 2;
const int CONNECTION_STATUS = 4;

// The ESP32 server version 1.0.0 at https://github.com/fhessel/esp32_https_server/ does not seem to be able a SSL (https) connection robustly.
// It is very slow, crashes regurarly and seems to hang on the /off node.
// Therefore we use regular http at the moment.
SSLCert *cert;
// HTTPSServer *myServer;
HTTPServer *myServer;

void createCertificate()
{
  Serial.println("Creating certificate...");

  cert = new SSLCert();

  int createCertResult = createSelfSignedCert(
      *cert,
      KEYSIZE_2048,
      "CN=myesp.local,O=acme,C=US");

  if (createCertResult != 0)
  {
    Serial.printf("Error generating certificate");
    return;
  }

  Serial.println("Certificate created with success");
}

void serveFile(const char *path, HTTPResponse *res)
{
  File fileToServe = SPIFFS.open(path);
  if (!fileToServe)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (fileToServe.available())
  {
    res->write(fileToServe.read());
  };
  fileToServe.close();
}

void setup()
{

  Serial.begin(115200);
  Serial.println("Starting Wakeup Transmitter");
  pinMode(WAKEUP_PIN, OUTPUT);
  pinMode(CONNECTION_STATUS, OUTPUT);

  Serial.println("Initializing SPIFF...");
  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // createCertificate();

  // myServer = new HTTPSServer(cert, PORTNUMBER);
  myServer = new HTTPServer(PORTNUMBER);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("IP address: " + WiFi.localIP().toString());
  Serial.println("MAC address: " + String(WiFi.macAddress()));

  ResourceNode *nodeRootRoot = new ResourceNode("/", "GET", [](HTTPRequest *req, HTTPResponse *res)
                                                { serveFile("/index.html", res); });

  ResourceNode *nodeRootCss = new ResourceNode("/style.css", "GET", [](HTTPRequest *req, HTTPResponse *res)
                                               { serveFile("/style.css", res); });

  ResourceNode *nodeRootOn = new ResourceNode("/on", "GET", [](HTTPRequest *req, HTTPResponse *res)
                                              {
                                                Serial.println("ON pressed");
                                                serveFile("/index.html", res);
                                                digitalWrite(WAKEUP_PIN, HIGH);
                                                delay(1000);
                                                digitalWrite(WAKEUP_PIN, LOW); });

  ResourceNode *nodeRootOff = new ResourceNode("/off", "GET", [](HTTPRequest *req, HTTPResponse *res)
                                               { 
                                                Serial.println("OFF pressed");
                                                serveFile("/index.html", res); });

  myServer->registerNode(nodeRootRoot);
  myServer->registerNode(nodeRootCss);
  myServer->registerNode(nodeRootOn);
  myServer->registerNode(nodeRootOff);

  myServer->start();

  if (myServer->isRunning())
  {
    Serial.println("Server ready, listening to port " + String(PORTNUMBER));
  }
}

void loop()
{
  static int delayCount = 0;
  static int connectionStatusLed = LOW;

  myServer->loop();

  if (delayCount >= 100 && WiFi.status() == WL_CONNECTED)
  {
    connectionStatusLed = connectionStatusLed == LOW ? HIGH : LOW;
    digitalWrite(CONNECTION_STATUS, connectionStatusLed);
    delayCount = 0;
  }

  delayCount++;
  delay(10);
}