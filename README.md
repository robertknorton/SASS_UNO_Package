# SASS_UNO_Package
Smart Automated Shower System (SASS) - Arduino Uno
This is a two microprocessor system that uses a Raspberry Pi 3B to handle the GUI, high-level logic, networking, and data storage.
While using a Arduino Uno for collecting sensor data, actuating solenoids, commanding servos, creating averages, and other low-level logic.
Both microcontrollers communicate via Serial and send structured messages that (raspi) send command signals to have actions performed or
(arduino) send sensor data and averages back to be displayed in the GUI and saved to a database for later.

This code is specificly for running on a Arduino Uno.
The Uno handles reading command messages from the Raspi 3B, does the commands, reads sensors, and then sends a message back containing sensor data.

    Arduino Libraries Used
        Arduino.h
        Adafruit_VL53L0X.h
        OneWire.h
        FlowMeter.h
        DallasTemperature.h
        Servo.h

Helpful links Arduino Serial Input Basics - https://forum.arduino.cc/index.php?topic=396450.0
