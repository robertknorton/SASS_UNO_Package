/*
Author: Robert Norton
Project: Senior Design Project: Smart Automated Shower System (SASS)
Date Last Updated: 10/5/2018
*/

#include <Arduino.h>
// ToF sensor includes
#include "Adafruit_VL53L0X.h"
// Temp sensor includes
#include <OneWire.h>

// #include <Wire.h>

#include <FlowMeter.h>

#include <DallasTemperature.h>
// Encoder includes
// #include <Encoder.h>
// Servo motor includes
#include <Servo.h>

// SoftwareSerial includes for ToF
// #include <SoftwareSerial.h>

#include "serialTools.h"
#include "sass.h"



/*
--------------------------------------------------------------------------------
                         _____      _
                        / ____|    | |
                       | (___   ___| |_ _   _ _ __
                        \___ \ / _ \ __| | | | '_ \
                        ____) |  __/ |_| |_| | |_) |
                       |_____/ \___|\__|\__,_| .__/
                                             | |
                                             |_|
--------------------------------------------------------------------------------
*/
// Define Arduino Uno I/O pins
#define FLOW_SENSOR_PIN         2   // DIGITAL IN (Interrupt)
#define TEMP_SENSOR_PIN         3   // DIGITAL IN
#define FLOW_SERVO_PIN          5   // DIGITAL(PWM) OUT
#define TEMP_SERVO_PIN          6   // DIGITAL(PWM) OUT
#define SYSTRANS_RELAY_PIN      7   // DIGITAL OUT
#define BYPASS_RELAY_PIN        8   // DIGITAL OUT


unsigned long SensorTimer = millis();

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWireTemp(TEMP_SENSOR_PIN); // This configures pin 2 to be OneWire

// Pass our oneWire reference to Dallas Temperature for Water temp sensor
DallasTemperature tempSensor(&oneWireTemp);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Digiten flow sensor properties for calibration
FlowSensorProperties DigitenProp = {30.0f, 7.5f, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

// connect a flow meter to an interrupt pin
FlowMeter FlowSensor = FlowMeter(FLOW_SENSOR_PIN, DigitenProp);

// measurement period for flow sensor
unsigned long FlowTimer = millis();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Create a TOF sensor object
#define SHT_LOX1 12
#define LOX1_ADDRESS 0x31
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;

unsigned long ProxTimer = millis();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Creating Servo objects
Servo servoFlow;
Servo servoTemp;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// define an 'interrupt service handler' (ISR) for every interrupt pin you use
void MeterISR() {
  // let our flow meter count the pulses
  FlowSensor.count();
}

// This is a setup sequences to ensure proper boot of proximity sensor
void setID()
{
    // all reset
  digitalWrite(SHT_LOX1, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);

  // activating LOX1
  digitalWrite(SHT_LOX1, HIGH);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {

    // Setup serial communications with Raspberry Pi
    Serial.begin(115200);
    // Serial.println("Establishing serial communication with Raspberry Pi");
    while (!Serial){} // Wait for serial to begin
    // Serial.println("Serial communication established");

    // Servo motors Initialize
    servoFlow.attach(FLOW_SERVO_PIN);
    updateServoPosition(servoFlow, 0, 100, 0);
    servoTemp.attach(TEMP_SERVO_PIN);
    updateServoPosition(servoTemp, 0, 100, 0);

    // Pin Mode Assignments
    pinMode(FLOW_SERVO_PIN, OUTPUT);
    pinMode(TEMP_SERVO_PIN, OUTPUT);
    pinMode(SYSTRANS_RELAY_PIN, OUTPUT);
    pinMode(BYPASS_RELAY_PIN, OUTPUT);
    // pinMode(TEMP_SENSOR_PIN , INPUT);
    // pinMode(FLOW_SENSOR_PIN, INPUT);
    pinMode(SHT_LOX1, OUTPUT);

    // Setting up Proximity sensor
    digitalWrite(SHT_LOX1, LOW);
    // Serial.println("Resetting Prox Sensor(pins are low)");
    // Serial.println("Starting Prox initialization");
    setID();
    // Serial.println("Prox Initialization Complete");

    // Setting up Flow Sensor
    // enable a call to the 'interrupt service handler' (ISR) on every rising edge at the interrupt pin
    attachInterrupt(INT0, MeterISR, RISING);

    // sometimes initializing the gear generates some pulses that we should ignore
    FlowSensor.reset();

    // Setting up Water Temperature Sensor
    tempSensor.begin();

}

/*
--------------------------------------------------------------------------------
            __  __       _         _
           |  \/  |     (_)       | |
           | \  / | __ _ _ _ __   | |     ___   ___  _ __
           | |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \
           | |  | | (_| | | | | | | |___| (_) | (_) | |_) |
           |_|  |_|\__,_|_|_| |_| |______\___/ \___/| .__/
                                                    | |
                                                    |_|
--------------------------------------------------------------------------------
*/
// Input format from Raspberry Pi
// <STV[int],BPV[int],FS[float],TS[float]>

int Water_Temp_Measurement = 0;
int Water_Temp_Avg = 0;
int Water_Temp_Total = 0;
int Water_Flow_Rate_Measurement = 0;
int Water_Flow_Cul_Measurement = 0;
int Proximity_Measurement = 0;

int counter = 0; // this is used in calculating the average water temp used

bool SystemOn = false; // Use for determining if the shower is turn ON

void loop() {

    recvWithStartEndMarkers(); // This strips down the message by its end markers
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0

        // Save the current input settings before reading the next input message
        saveCurrentSettings();
        // Parse the new input message and update
        parseData();

        // Serial.println(STV);
        // Serial.println(BPV);
        // Serial.println(FS);
        // Serial.println(TS);

        // showParsedData();  // We don't want to print to serial and polute the bus
        if (STV != pre_STV) // if there is a new STV value do work
        {
          if (STV == 1) // Valve Open
          {
            digitalWrite(SYSTRANS_RELAY_PIN, HIGH);
            SystemOn = true; // this ACTIVATES sensor readings and publishing back to RPi
            Water_Temp_Avg = 0; // Reset avg water temp
            Water_Temp_Total = 0;
            counter = 0;
          }
          else if (STV == 0) // Valve Closed
          {
            digitalWrite(SYSTRANS_RELAY_PIN, LOW);
            pre_FS = 500;
            FS = 500;
            SystemOn = false; // this DEACTIVATES sensor readings and publishing back to RPi
          }
        }
        if (BPV != pre_BPV) // if there is a new BPV value do work
        {
          if (BPV == 1) // Valve Open
          {
            digitalWrite(BYPASS_RELAY_PIN, HIGH);
          }
          else if (BPV == 0) // Valve Closed
          {
            digitalWrite(BYPASS_RELAY_PIN, LOW);
          }
        }
        if (RST != pre_RST) // if there is a new RST value do work
        {
          if (RST == 1) // reset the flow sensor
          {
            // FlowSensor.reset(); // this resets the meter values
            FlowSensor.setTotalVolume(0.0); // reset total volume
            // FlowSensor.setTotalCorrection(0.0); // reset total correction
            // FlowSensor.setTotalDuration(0.0); // reset total duration
          }
          else if (RST == 0) // don't reset
          {
            // do nothing
          }
        }
        if ((FS != pre_FS) and (SystemOn == true))
        {
          if (0 <= FS and FS < 101) // Update servo angle command
          {
            updateServoPosition(servoFlow, int(FS), 100, 0);
          }
          
        }
        else if ((FS != 0) and (SystemOn == false)) // When system is off always have flow set to zero
        {
            updateServoPosition(servoFlow, int(0), 100, 0);
        }
        if ((TS != pre_TS)) // and (SystemOn == true)
        {
          if (0 <= TS and TS < 101) // Update servo angle command
          {
            updateServoPosition(servoTemp, int(TS), 100, 0); //updateServoPosition(servoTemp, int(TS), 0, 100);
          }
        }
        newData = false;
    }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // This is for periodiclly checking the proximity sensor
    if (millis() - ProxTimer > 500)
    {
        lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
        if(measure1.RangeStatus != 4)
        {     // if not out of range
            // Serial.println(measure1.RangeMilliMeter);
            Proximity_Measurement = measure1.RangeMilliMeter / 10; // smaller number to send
        }
        else
        {
            // Serial.println("Out of range");
        }
        ProxTimer = millis(); // reset timer value
    }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check every 1/2 second the sensors and then send out a message containing all sensor data to the Raspberry Pi
    if ((millis() - SensorTimer > 500)) // and (SystemOn == true
    {
        // Temperature Sensor Reading
        tempSensor.requestTemperatures();
        Water_Temp_Measurement = tempSensor.getTempFByIndex(0);
        Water_Temp_Total = Water_Temp_Total + Water_Temp_Measurement; // add new measurement to total
        counter = counter + 1; // increase number of measurements taken
        Water_Temp_Avg = Water_Temp_Total / counter; // calculate the new average temp

        // Flow Sensor reading
        if (millis() > FlowTimer)
        {

        unsigned long period = millis() - FlowTimer;
        FlowSensor.tick(period);
        // The reason the below measurements are multiplied by 100 is to truncate the decimals into int
        Water_Flow_Rate_Measurement = (FlowSensor.getCurrentFlowrate() / 3.79) * 100; // get measurment in (g/min*100)
        Water_Flow_Cul_Measurement = (FlowSensor.getTotalVolume() / 3.79) * 100; // get measurment in (g/min*100)

        // Serial.println("Currently " + String(Water_Flow_Rate_Measurement) + " g/min, " + String(Water_Flow_Cul_Measurement)+ " g total.");

        FlowTimer = millis(); // resets timer value
        }

        String newSensorOut = ('{'  + String(Water_Temp_Measurement) + ','
                                    + String(Water_Temp_Avg) + ','
                                    + String(Water_Flow_Rate_Measurement) + ','
                                    + String(Water_Flow_Cul_Measurement) + ','
                                    + String(Proximity_Measurement) + '}');
        Serial.println(newSensorOut);

        SensorTimer = millis(); // resets timer value
    }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
