#ifndef BLE_H
#define BLE_H

#include <stdint.h>


#define DIR_POSITIVE 1
#define DIR_NEGATIVE 0

// encoding for direction
#define DIR_X_CODE 


// in total 8 = 2 * 4 bytes for each direction

// only using first 2 bytes for torque encoding, the rest is reserved, total 128 bits

typedef struct 
{
    double mt;     // main motor torque
    double mp;     // pendulum position
} Command_t;

typedef struct BLEData // received BLE data structure
{
    Command_t uCommand; // user command data
    char rawData[128]; // raw data received from BLE python
} BLEData; // all data received from BLE

typedef struct MotorStatus // send back motor status structure
{
    double pg;      // pendulum angle in degrees
    double pdg;     // pendulum angular velocity
    double mp;       // motor position
    double mdp;     // motor velocity
    char rawData[128]; // raw data received from BLE
} MotorStatus;



// public functions

volatile BLEData *getBLEData(); // get current command data
void runBLE(); // run BLE process, send data to MCU in this function, data received from central is processed in callback
void receiveBLEData(char *rawData, int length); // receive raw data from central and decode


#endif /* BLE_H */
