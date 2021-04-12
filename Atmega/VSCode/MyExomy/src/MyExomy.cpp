/*
   Program for MyExomy power board.
   Meant for Atmega328_on_breadboard_8MHz.
*/
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

const int LdrPin = A0;
const int BatteryPin = A1;
const int InterruptPin = 2; // PD2
const int HeadlightPin = 8; // PB0
const int RelaisPin = 9;    // PB1
const int RGBBluePin = 5;   // PD5
const int RGBGreenPin = 6;  // PD6
const int RGBRedPin = 7;    // PD7
const int BatteryThresholdGreen = 680; // With resistors 68K and 10K this corresponds to 5.7V (analog reference is internal 1.1V).
const int BatteryThresholdBlue = 620;  // With resistors 68K and 10K this corresponds to 5.2V (analog reference is internal 1.1V).
// LDR thresholds. Lower and higher threshold to have hysteresis.
const int LdrThresholdHigher = 800;             // Higher value means it must be darker to switch on the LEDs.
const int LdrThresholdLower = 600;              // Lower value means it must be lighter to switch back off the LEDs.
const unsigned long MotionCheckIntervalMillis = 10000 / 2; // Because we use the internal clock of 8 MHz we have to divide the desired interval by two.
volatile bool motion = false;

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

void setup()
{
  Serial.begin(2 * 9600); // We need 9600 baud, but because we use the internal clock of 8 MHz we have to set it to 2x9600.
  Serial.println("Hello MyExomy!");
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
}

void loop()
{
  static unsigned long previousMillis = 0;
  digitalWrite(RelaisPin, HIGH); // Switch on RPi and servo power.
  analogReference(INTERNAL); // Analog reference to internal 1.1V (for ATmega328P). Used for measuring battery voltage, so reference must be independent of the supply voltage.
  int batteryVal = analogRead(BatteryPin);
  analogReference(DEFAULT); // Analog reference to 5V. Used for measuring LDR voltage which is relative to the 5V supply.
  int ldrVal = analogRead(LdrPin);
  //Serial.println("LDR value: " + String(ldrVal));
  //Serial.println("Battery value: " + String(batteryVal));
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
    digitalWrite(HeadlightPin, HIGH);
  }
  else if (ldrVal < LdrThresholdLower) // If it is light with hysteresis.
  {
    digitalWrite(HeadlightPin, LOW);
  }

  //Serial.println("millis: " + String(millis()) + " motion: " + String(motion));
  if (motion)
  {
    previousMillis = millis();
    motion = false;
  }
  else if (millis() - previousMillis >= MotionCheckIntervalMillis)
  {
    Serial.println("going to sleep!");
    delay(100);
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
  }
}
