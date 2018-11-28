/*
Author: Robert Norton
Project: Senior Design Project: Smart Automated Shower System (SASS)
Date Last Updated: 11/27/2018
*/


void sweep(Servo myservo)
{
    int ustep = 0;
    for (ustep = 500; ustep <= 2500; ustep += 10)
    {
        // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.writeMicroseconds(ustep);
        // tell servo to go to position in variable 'pos'
        delay(15); // waits 15ms for the servo to reach the position
    }
    for (ustep = 2500; ustep >= 500; ustep -= 10)
    {
        // goes from 180 degrees to 0 degrees
        myservo.writeMicroseconds(ustep);
        // tell servo to go to position in variable 'pos'
        delay(15); // waits 15ms for the servo to reach the position
    }
}

int updateFlowControl(int pot)
{
    int flow_val = map(pot, 1, 690, 0, 100);
    //Serial.println("Flow Set Value: "); Serial.println(flow_val);
    return flow_val;
}

int updateTempControl(int pot)
{
    int temp_val = map(pot, 1, 690, 68, 110);
    //Serial.println("Temp Set Value: "); Serial.println(temp_val);
    return temp_val;
}

void updateServoPosition(Servo myservo, int newVal, int minInput, int maxInput)
{
    int setVal = map(newVal, minInput, maxInput, 500, 2500);
    // Serial.print("Flow Servo Set: "); Serial.println(setVal);
    myservo.writeMicroseconds(setVal);
}

int smooth(int data, float filterVal, float smoothedVal)
{
    if (filterVal > 1)
    {      // check to make sure param's are within range
        filterVal = .99;
    }
    else if (filterVal <= 0)
    {
        filterVal = 0;
    }
    smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
    return (int)smoothedVal;
}

float mmToft(int dist)
{
    return (dist*0.00328084);
}

void systemStatusPrint(int setWF, int setWT, int actualWF, int actualWT, VL53L0X_RangingMeasurementData_t proxSensor)
{
    Serial.print("Set WF: "); Serial.print(setWF); Serial.print(", ");
    Serial.print("Actual WF: "); Serial.print(actualWF); Serial.print(", ");
    Serial.print("Set WT: "); Serial.print(setWT); Serial.print(", ");
    Serial.print("Actual WT: "); Serial.print(actualWF); Serial.print(", ");

    if (proxSensor.RangeStatus != 4)
    {  // phase failures have incorrect data
        Serial.print("Proximity Dist (ft): ");
        Serial.println(mmToft(proxSensor.RangeMilliMeter));
    }
    else
    {
        Serial.println("Proximity Dist: out of range ");
    }
}

int checkEncoder(long* position, long newReading)
{
    if (newReading != *position)
    {
        Serial.print("New Reading: ");
        Serial.println(newReading);
        Serial.print("Cur Pos: ");
        Serial.println(*position);
        if (newReading > *position)
        {
            *position = newReading;
            Serial.println("Encoder = Left");
            return 1;  // left
        }
        else if (newReading < *position)
        {
            *position = newReading;
            Serial.println("Encoder = Right");
            return 2; // right
        }
    }
    else
    {
        return 0;
    }
}

// void updateServo(Servo myservo, char type, int val)
// {
//     int setVal = 0;
//     switch (type) {
//         case 't':
//             setVal = map(val, 0, 110, 500, 2500);
//             Serial.print(setVal);
//             myservo.writeMicroseconds(setVal);
//             break;
//         case 'f':
//             setVal = map(val, 0, 100, 500, 2500);
//             Serial.print(setVal);
//             myservo.writeMicroseconds(map(Set_Flow_Value, 0, 100, 500, 2500));
//             Serial.print(myservo.readMicroseconds());
//             break;
//     }
// }
