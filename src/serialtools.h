#include <Arduino.h>

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// EXAMPLE: variables to hold the parsed data
// char messageFromPC[numChars] = {0};
// int integerFromPC = 0;
// float floatFromPC = 0.0;

// Parsed Variables
int STV = 0; // system transfer valve, 0 is closed, 1 is open
int BPV = 0; // bypass valve, 0 is closed, 1 is open
int RST = 0; // reset indicator; mostly for reseting flow sensor
int FS = 0; // flow servo, accepts a int between 0 and 100
int TS = 0; // temp servo, accepts a int between 0 and 100

// Previous varables used for check if their value has changed
int pre_STV = 0; // system transfer valve, 0 is closed, 1 is open
int pre_BPV = 0; // bypass valve, 0 is closed, 1 is open
int pre_RST = 0; // reset indicator; mostly for reseting flow sensor
int pre_FS = 0; // flow servo, accepts a int between 0 and 100
int pre_TS = 0; // temp servo, accepts a int between 0 and 100

boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void saveCurrentSettings()
{
    pre_STV = STV;
    pre_BPV = BPV;
    pre_RST = RST;
    pre_FS = FS;
    pre_TS = TS;
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    // strtokIndx = strtok(tempChars,",");      // get the first part - the string
    // strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
    STV = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    BPV = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    RST = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    FS = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    TS = atof(strtokIndx);     // convert this part to a float

    // Serial.println(pre_STV);
    // Serial.println(pre_BPV);
    // Serial.println(pre_FS);
    // Serial.println(pre_TS);

}

void showParsedData() {
    // Serial.print("Message ");
    // Serial.println(messageFromPC);
    // Serial.print("Integer ");
    // Serial.println(integerFromPC);
    // Serial.print("Float ");
    // Serial.println(floatFromPC);
}
