/*
   Program for MyExoMy power board.
   Meant for Atmega328_on_breadboard_8MHz.
*/
#include <Arduino.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

const int LdrPin = A0;
const int BatteryPin = A1;
const int SolarPin = A2;
const int InterruptPin = 2;            // PD2
const int HeadlightPin = 8;            // PB0
const int RelaisPin = 9;               // PB1
const int RGBBluePin = 5;              // PD5
const int RGBGreenPin = 6;             // PD6
const int RGBRedPin = 7;               // PD7
const int BatteryThresholdGreen = 680; // With resistors 68K and 10K this corresponds to 5.7V (analog reference is internal 1.1V).
const int BatteryThresholdBlue = 620;  // With resistors 68K and 10K this corresponds to 5.2V (analog reference is internal 1.1V).
// LDR thresholds. Lower and higher threshold to have hysteresis.
const int LdrThresholdHigher = 800;                        // Higher value means it must be darker to switch on the LEDs.
const int LdrThresholdLower = 600;                         // Lower value means it must be lighter to switch back off the LEDs.
const unsigned long MotionCheckIntervalMillis = 120000 / 2; // Because we use the internal clock of 8 MHz we have to divide the desired interval by two.
volatile bool motion = false;

int i2cCommand = 0;        // global variable for receiving command from I2C
int i2cParameters[10];     // global array for receiving parameters from I2C
int i2cParameterCount = 0; // global variable for keeping I2C parameter count
int i2cDataByteToSend = 0; // global variable to send back over I2C in receiveData() callback fucntion.

// callback for received data
void receiveData(int byteCount)
{
  int i2cData;
  while (Wire.available())
  {
    i2cData = Wire.read();
  }
  // All values below 128 are commands, else it is a parameter.
  if (i2cData < 128)
  {
    // New command so reset parameter count.
    i2cParameterCount = 0;
    i2cCommand = i2cData;
  }
  else
  {
    i2cParameters[i2cParameterCount++] = i2cData;
  }
}

// callback for sending data
void sendData()
{
  Wire.write(i2cDataByteToSend); // Send data back over I2C.
}

void setAllPinsToInput()
{
  // Set all pins to input with pullup resistor for minimum power consumption.
  // Pins that are used will receive a subsequent pinMode setting.
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
}

void enableAdc()
{
  bitSet(ADCSRA, ADEN);
}
void disableAdc()
{
  bitClear(ADCSRA, ADEN);
}

void disableAc()
{
  bitSet(ACSR, ACD);
}

void wakeup()
{
  motion = true;
}

void goToSleep()
{
  Serial.println("going to sleep!");
  delay(100); // Small delay to make sure the println is finished.
  digitalWrite(RelaisPin, LOW); // Switch off RPi and servo power.
  digitalWrite(HeadlightPin, LOW);
  digitalWrite(RGBGreenPin, LOW);
  digitalWrite(RGBBluePin, LOW);
  digitalWrite(RGBRedPin, LOW);
  disableAdc();        // This will save appr. 260 μA.
  power_all_disable(); // This does not seem to save additional power.
  sleep_bod_disable(); // BODS (Brown Out Detection Sleep) is active only 3 clock cycles, so sleep_cpu() must follow immediately. This will save appr. 20 μA.
  sleep_cpu();         // Power down! Power drops from appr. 8 mA to appr. 0.1 μA.
  power_all_enable();  // We will need this for the delay function (timer 0).
  enableAdc();         // We need this for the LDR measurement
  digitalWrite(RelaisPin, HIGH); // Switch on RPi and servo power.
}

void setup()
{
  Serial.begin(2 * 9600); // We need 9600 baud, but because we use the internal clock of 8 MHz we have to set it to 2x9600.
  Serial.println("Hello MyExoMy!");
  setAllPinsToInput(); // This does not seem to save additional power ( > 1 μA).
  disableAc();         // This does not seem to save additional power ( > 1 μA).
  wdt_disable();       // This does not seem to save additional power ( > 1 μA).
  pinMode(InterruptPin, INPUT_PULLUP);
  pinMode(RelaisPin, OUTPUT);
  pinMode(HeadlightPin, OUTPUT);
  pinMode(RGBBluePin, OUTPUT);
  pinMode(RGBGreenPin, OUTPUT);
  pinMode(RGBRedPin, OUTPUT);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode to power down.
  attachInterrupt(0, wakeup, FALLING); // trigger interrupt when INT0 goes from HIGH to LOW.
  sleep_enable();
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  digitalWrite(HeadlightPin, LOW); // Be sure to start with Headlights off.
  digitalWrite(RelaisPin, HIGH); // Switch on RPi and servo power.
}

void loop()
{
  static unsigned long previousMillis = 0;
  analogReference(INTERNAL);     // Analog reference to internal 1.1V (for ATmega328P). Used for measuring battery voltage, so reference must be independent of the supply voltage.
  int batteryVal = analogRead(BatteryPin);
  int solarVal = analogRead(SolarPin);
  analogReference(DEFAULT); // Analog reference to 5V. Used for measuring LDR voltage which is relative to the 5V supply.
  int ldrVal = analogRead(LdrPin);
  //Serial.println("Battery value: " + String(batteryVal));
  //Serial.println("LDR value: " + String(ldrVal));
  if (batteryVal > BatteryThresholdGreen) // Battery high voltage.
  {
    digitalWrite(RGBGreenPin, HIGH);
    digitalWrite(RGBBluePin, LOW);
    digitalWrite(RGBRedPin, LOW);
  }
  else if (batteryVal > BatteryThresholdBlue) // Battery medium voltage.
  {
    digitalWrite(RGBGreenPin, LOW);
    digitalWrite(RGBBluePin, HIGH);
    digitalWrite(RGBRedPin, LOW);
  }
  else // Battery low voltage.
  {
    digitalWrite(RGBGreenPin, LOW);
    digitalWrite(RGBBluePin, LOW);
    digitalWrite(RGBRedPin, HIGH);
  }
  if (ldrVal > LdrThresholdHigher) // If it is dark with hysteresis.
  {
    //digitalWrite(HeadlightPin, HIGH);
  }
  else if (ldrVal < LdrThresholdLower) // If it is light with hysteresis.
  {
    //digitalWrite(HeadlightPin, LOW);
  }

  //Serial.println("millis: " + String(millis()) + " motion: " + String(motion));
  if (motion)
  {
    previousMillis = millis();
    motion = false;
  }
  else if (millis() - previousMillis >= MotionCheckIntervalMillis)
  {
    goToSleep();
  }

  switch (i2cCommand)
  {
  case 1: // Light on.
    digitalWrite(HeadlightPin, HIGH);
    i2cCommand = 0;
    break;
  case 2: // Light off.
    digitalWrite(HeadlightPin, LOW);
    i2cCommand = 0;
    break;
  case 10: // Indicate that there is motion, to keep the MyExoMy awake.
    motion = true;
    i2cCommand = 0;
    break;
  case 100: // Command to indicate a read is going to follow.
    if (i2cParameterCount == 1)
    {
      if (i2cParameters[0] == 128)          // 128: read battery voltage.
      {
        i2cDataByteToSend = batteryVal / 4; // Map level [0..1023] to [0..255] so it fits in one byte.
      }
      else if (i2cParameters[0] == 129)     // 129: read solar panel voltage.
      {
        i2cDataByteToSend = solarVal / 4;   // Map level [0..1023] to [0..255] so it fits in one byte.
      }
      else if (i2cParameters[0] == 255)     // 255: go to sleep.
      {
        i2cDataByteToSend = 42;             // Send a magic number as acknowledge.
        // Delay to make sure acknowledge is sent back over I2C and Raspberry Pi is properly shut down.
        // Because we use the internal clock of 8 MHz we have to divide the desired interval by two.
        delay(30000UL / 2);
        goToSleep();
      }
      i2cCommand = 0;
    }
    break;
  default:
    break;
  }
  // No delay here as it will degrade I2C responsiveness.
}
