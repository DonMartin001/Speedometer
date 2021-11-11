/*
   Speedometer Prototype with LED, LCD &  clock/temp.

   Sketch Name - "SpeedometerLEDLCDClockGLED".

   Author - Don Martin
   Date - 6Nov18
   Updated - 24Mar20 - added RTC Clock with Temperature.
              4Apr20 - added PhotoResistors as optional sensors.
              8Apr20 - added Green LED to show what I'm doing.
              2Nov21 - give a bit more time to clear the speed trap.
              
   Pins Used:  A0 - left sensor (either IR or PhotoResistor).
               A1 - Right sensor (same sensor type as A0).
               D9 - Control of Green LED (positive feed).
               A4 - SDA.   Data Line.
               A5 - SCL.   Clock Line.
               5V - Voltage Out.
               GND - Ground Voltage (negative power in & negative power out).
               VIN - Input Positive (7-12V).                

*/
/***************************************************
  This is a library for our I2C LED Backpacks
  ----> http://www.adafruit.com/products/880
  ----> http://www.adafruit.com/products/879
  ----> http://www.adafruit.com/products/878

  Designed specifically to work with the Adafruit LED 7-Segment backpacks
  ----> http://www.adafruit.com/products/881

  These displays use I2C to communicate, 2 pins are required to
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
// Base Libraries.
#include <Wire.h>
#include <Adafruit_GFX.h>
// +Libraries for LED (needs "Wire.h")
#include "Adafruit_LEDBackpack.h"
// +Libraries for LCD
#include <LiquidCrystal_I2C.h>
// +Libraries for Real Time Clock
#include "RTClib.h"

//
// Setup Global Constants and Variables.
//
  boolean debugOn=false;           // true if debugging is on (writes data to the Serial Monitor).
                                   // Causes problems with speed calc when on.
  // Setup items for LED Speed display.
  Adafruit_7segment matrix = Adafruit_7segment();
  int brightSet = 5;              // Set the brightness of the display.
  // Setup items for LCD Speed display.
  const byte lcdAddr = 0x27;      // I2C address of 2 X 16 display.
  const byte lcdCols = 16;        // Number of columns on LCD display.
  const byte lcdRows = 2;         // Number of rows on LCD display.
  LiquidCrystal_I2C lcd(lcdAddr, lcdCols, lcdRows);
  // Definitions for the RTC (Real Time Clock)
  RTC_DS3231 rtc;
  // Global Programming Constants & Variables.
  boolean aLeft;                  // Save the Left sensor input.
  boolean bRight;                 // Save the Right sensor input.
  boolean oaLeft = false;         // Save the Left sensor input over multiple reads.
  boolean obRight = false;        // Save the Right sensor input over multiple reads.
  int oReadTimes = 5;             // The numer of times the sensor is read.
  unsigned long oReadDelay = 100; // Time between sensor reads.
  unsigned long startTime;        // Hold the time that the start sensor was triggered.
  unsigned long stopTime;         // Hold the time that the end sensor was triggered.
  unsigned long timeDiff;         // Determine the time between the sensors being tripped.
  unsigned long waitForStopTimer = 5000; // Length of time to wait for the Stop Timer to be tripped.
  unsigned long dispTimer;        // Time that characters were displayed for.
  unsigned long timerLength=10000;  // Time that the display should be active for (how long it displays the speed).
  boolean timerTripped = false;   // Used to indicate that the stop timer has been tripped.
  char timerDirection=' ';        // Direction the train is going ('L'eft or 'R'ight).
  float locoSpeed;                // Where the calculated speed goes.
  float sensorDistance = 143.0;   // Distance between sensors in mm this is for the Test Layout Front.  Actual is 145.0.
  float mmToMiles = 0.000000621371; // Convert milimeters to Miles.
  float msToHours = 0.00000027778; // Convert Miliseconds to Hours.
  float sensorToMiles = sensorDistance * mmToMiles; // Convert the Actual Distance to Miles.
  float toScale = 160;            // Multiply by this to get Scale MPH.
  int dispLength = 16;            // Number of characters to display on a row.
  char sensorType = 'I';          // Set to "I"nfrared or "P"hotocell.
  const int pC1 = A0;             // Pin for the first Sensor.
  const int pC2 = A1;             // Pin for the second Sensor.
  float cutOffSensor1 = 0;        // Cut off value for detection on Sensor 1.
  float cutOffSensor2 = 0;        // Cut off value for detection on Sensor 2.
  float cutOffPercent = 0.75;     // Percentage of Ambient Light used for cutoff.
  float irCutOffValue = 400;      // Default cutoff value for the IR Sensor.
  const int GLEDPin = 9;          // PWM Pin to control the Green LED.
  const int GLEDFlashRate = 1000; // How long to hold the pin on and off while flashing.
  const int GLEDIntensity = 100;  // Default Intensity of the LED (PWM value).
  const int GLEDIncrease = 05;    // Amount to increase the intensity of the LED.
  int GLEDCurValue = 0;           // Current Intensity Level.
  unsigned long startFlashTime = 0; // Holds the amount of time the LED is on or off.
  bool GLEDOn = false;              // The LED is either on or off.
  bool displayOff=true;             // Set to true when we turn off the Speed Display.
//
void setup() {
  Serial.begin(9600); delay(1000); // Open Serial Port and wait a second.
  if (debugOn == true) Serial.println("Starting");
  // Initialize LCD.
  lcd.begin();
  lcd.clear();
  // Assign LED I2C Address      
  matrix.begin(0x70);
  // Make sure we have a Real Time Clock.
  if (! rtc.begin()) Serial.println("Couldn't find RTC");
  // Set up the two pins to read the IR Detectors.
  pinMode(pC1, INPUT); // Receives signal from the left (A) sensor.
  pinMode(pC2, INPUT); // Receives signal from the right (B) sensor.
  /*
   * If we are using a PhotoResistor, figure out ambient light value and take
   * a percentage of that for the cutoff value.   For the IR sensors, use a set
   * cutoff value.
   */     
    analogWrite(GLEDPin,GLEDIntensity);    // Turn on the Green LED.
    //
    // Write a countdown timer to the LED and LCD.
    //
    matrix.setBrightness(brightSet);
    matrix.clear();
    matrix.writeDigitNum(0,0);
    matrix.writeDigitNum(1,0);
    lcd.clear();
    lcd.print("LCD Init: ");
    DateTime now = rtc.now();
    float readingTotal1 = 0;       // Total readings from Sensor 1.
    float noReadings1 = 0;         // Number of readings for Sensor 1.
    float readingTotal2 = 0;       // Total readings from SEnsor 2.
    float noReadings2 = 0;         // Number of readings for Sensor 2.
    for (int j = 11; j-- > 0;) {
      lcd.setCursor(10, 0);
      lcd.print(j);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(now.hour());
      lcd.print(":");
      if (now.minute()<10) lcd.print("0"); lcd.print(now.minute());
      lcd.print(":");
      if (now.second()<10) lcd.print("0"); lcd.print(now.second());
      lcd.print("  ");
      lcd.print(rtc.getTemperature(),1);
      lcd.print(" C");
      if (j>9) {
        matrix.writeDigitNum(3,1);
        matrix.writeDigitNum(4,0);
      }
      else {
        matrix.writeDigitNum(3,0);
        matrix.writeDigitNum(4,j);
      }
      matrix.writeDisplay();
      // Read each of the Sensors and add to total and count so we can figure out the average.
      readingTotal1 += analogRead(pC1);
      noReadings1++;
      readingTotal2 += analogRead(pC2);
      noReadings2++;
      delay(500);
    }
    dispTimer=millis();
    if (sensorType == 'P') {
         // If using PhotoResistors then calculate cutoff value.
         cutOffSensor1 = (int) (readingTotal1/noReadings1*cutOffPercent);
         cutOffSensor2 = (int) (readingTotal2/noReadings2*cutOffPercent); 
    } else {
          // If using IR detectors then use default value.
          cutOffSensor1 = irCutOffValue;
          cutOffSensor2 = irCutOffValue;  
    }
    if (debugOn) {
          Serial.print("Cutoff Value 1 = ");
          Serial.print(cutOffSensor1);
          Serial.print("   Cutoff Value 2 = ");
          Serial.println(cutOffSensor2);
          delay(2000);
    }
    analogWrite(GLEDPin,0);     // Turn off Green LED
    startFlashTime = millis();         // Used to determine when to turn the LED on or off.
}
//
// Now time for the main loop.
//
void loop() {
  //
  // See if it is time to turn the Green LED or off.
  // Each time we turn it on, we increase the brightness
  // until we reach the default seting, then we restart the
  // process.
  //
  if (displayOff) {
  if (millis()-startFlashTime > GLEDFlashRate) {
          if (GLEDOn) {
               analogWrite(GLEDPin, 0);      // Turn off Green LED.
               startFlashTime = millis();
               GLEDOn = false;
          } else {
               GLEDCurValue += GLEDIncrease;
               if (GLEDCurValue > GLEDIntensity) GLEDCurValue = 2;
               analogWrite(GLEDPin, GLEDCurValue);
               startFlashTime = millis();
               GLEDOn = true;                             
          }
    }
  }
  //
  // Get the Current Sensor Values.
  //
  readSensors();
  //
  // If either sensor has been tripped, this is the "start timer" so:
  // - Record the time.
  // - Call the routine that waits for the "stop timer" sensor.
  //
  if (aLeft == true) {
    timerDirection='L';
  }
  else if (bRight == true) {
    timerDirection='R';
  }
  else {
    timerDirection=' ';
  }
  if (timerDirection=='R' || timerDirection=='L') {
    startTime = millis();
    waitForStopSensor();
  }
  //
  // Execute the cannotwait function for stuff that has to be done every loop.
  //
  canNotWait();
  //
  // Once both sensors are clear and the display time has elapsed, blank the display.
  //
  stopTime = millis();
  timeDiff = stopTime - dispTimer;
  if (aLeft == false && bRight == false && timeDiff > timerLength) {
    dispTimer = millis();
    matrix.clear();
    matrix.writeDisplay();
    lcd.clear();
    lcd.noBacklight();  // Turn off LCD backlight.
    analogWrite(GLEDPin, 0);    // Turn off Green LED when we turn off Speed Display.
    startFlashTime=millis();
    displayOff=true;
  }
/*
 * Debug Code.
 */
 if (debugOn) {
  Serial.print("dispTimer = ");
  Serial.print(dispTimer);
  Serial.print(" Time Difference = ");
  stopTime = millis();
  timeDiff = stopTime - dispTimer;
  Serial.print(timeDiff);
  Serial.print("  aLeft = ");
  Serial.print(aLeft);
  Serial.print(" bRight = ");
  Serial.print(bRight);
  Serial.print(" timerDirection = ");
  Serial.println(timerDirection);
  delay(500);
 } 
}
/*
 * Read the two sensors and set variables based upon whether
 * the Sensor has been covered or not.
 */
void readSensors() {
  //
  // Save the Sensor Values.
  //
  int sensor1 = analogRead(pC1);
  int sensor2 = analogRead(pC2);
  if (sensor1 < cutOffSensor1) {
    aLeft = true;
  } else {
    aLeft = false;
  }
  if (sensor2 < cutOffSensor2) {
    bRight = true;
  } else {
    bRight = false;
  }
  if (debugOn) {Serial.print("Sensor 1 = "); Serial.print(sensor1); Serial.print("  Sensor 2 = ");Serial.println(sensor2);}
}
/*
 * Keep testing and wait for the "stop" sensor to be tripped:
 * - But, if it isn't tripped within a set period of time, reset
 *   everything.
 */
void waitForStopSensor(){
 analogWrite(GLEDPin,HIGH);      // Turn on the Green LED to Max to show we are "calculating).
 //
 // Sit in a nice tight little loop until the Stop Sensor is tripped
 // or the set amount of time has elapsed.
 //
 do {
  readSensors();
  stopTime = millis();
  timeDiff = stopTime-startTime;  
  if (timerDirection=='R' && aLeft==true || timerDirection=='L' && bRight==true)
    timerTripped = true;
  else
    timerTripped = false;
   canNotWait(); // Continue to execute time critical code.
  } while (timeDiff < waitForStopTimer && timerTripped == false);
 //
 // If Timer has been tripped, display the speed and wait until both sensors
 // are clear before exiting.
 //
 if (timerTripped == true) {
    analogWrite(GLEDPin,GLEDIntensity);   // Write default intensity to the Green LED till speed turns off.
    locoSpeed = calcSpeed(timeDiff);
    ledDisplay(locoSpeed);
    lcdDisplay(locoSpeed);
    if (debugOn) serialDisplay(locoSpeed);
    displayOff=false;
    dispTimer = millis();
    startTime=stopTime=timeDiff=0;
    //
    // Now wait until both sensors are clear.
    // Build in a slight delay (is this a bounce?).
    //
    do {
        //
        // Read Sensors "oReadTimes" times to "make sure".
        // If the sensor was activate at any time during the read loop then set the
        // sensor to active regardless of how many "non-active" states there were.
        //
      int iLoop=0;
      oaLeft = obRight = false;
      do {
         readSensors();
         if (aLeft == true) oaLeft = true;
         if (bRight == true) obRight = true;
         canNotWait(); // Continue to execute time critical code.
         delay(oReadDelay);
      } while (iLoop++ < oReadTimes);
    canNotWait(); // Continue to execute time critical code.
    aLeft=oaLeft;  // Set sensor to value from loop.
    bRight=obRight;  // Set sensor to value from loop.
    } while (aLeft == true || bRight == true);
  }
}
/*
 * canNotWait - This code is executed from within every wait loop.
 *              it is code (like moving characters on an LCD display)
 *              that needs to be executed even while waiting for
 *              other suff.
 */
void canNotWait() {
  // Put code in here for stuff that "can't wait".
}
/*
   Take the float number and display it on the LED.
*/
void ledDisplay(float numDisplay) {   
    matrix.println(locoSpeed);
    matrix.writeDisplay();
}
/*
   Take the float number and display it on the LCD.
*/
void lcdDisplay(float numDisplay) {    
  lcd.backlight();
  lcd.clear();
  lcd.print("Speed = ");
  lcd.setCursor(8, 0);
  lcd.print(numDisplay,1);
  //lcd.setCursor(15,0);
  if (timerDirection=='L')
    lcd.print(" =>");
  else
    lcd.print(" <=");
   lcd.setCursor(0, 1);
   DateTime now = rtc.now();
   lcd.print(now.hour());
   lcd.print(":");
   if (now.minute()<10) lcd.print("0"); lcd.print(now.minute());
   lcd.print(":");
   if (now.second()<10) lcd.print("0"); lcd.print(now.second());
   lcd.print("  ");
   lcd.print(rtc.getTemperature(),1);
   lcd.print(" C");  
}
/*
   Take the float number and display it to the Serial Device.
*/
void serialDisplay(float numDisplay) {
  if (debugOn) {
    Serial.print("SensorDistance=");
    Serial.print(sensorDistance);
    Serial.print("   Time=");
    Serial.print(timeDiff);
    Serial.print("   Scale speed =");
    Serial.println(numDisplay);
  }
}
/*
   Calculate the Speed.
*/
float calcSpeed(float timeDiff) {
  float convertedSpeed = ((sensorToMiles) / (timeDiff * msToHours) * toScale);
  if (convertedSpeed > 1000) convertedSpeed=0;    
  return (convertedSpeed);
}
 /* END OF CODE */
