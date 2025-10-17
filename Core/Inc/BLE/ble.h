#ifndef BLE_H
#define BLE_H

#include <stdint.h>


#define DIR_POSITIVE 1
#define DIR_NEGATIVE 0

// encoding for direction
#define DIR_X_CODE 0
#define DIR_Y_CODE 1
#define DIR_Z_CODE 2
#define DIR_W_CODE 3

#define DIR_CODE_LENGTH 3 // in bits for direction encoding, leave 1 bit for sign

#define TORQUE_CODE_LENGTH 13 // in bits for torque encoding, unit A

// first 3 digit of torque is exponent (2), next 10 digit is mantissa (10)
#define TORQUE_EXP_LENGTH 3
#define TORQUE_MANT_LENGTH 10

#define TORQUE_FACTOR 0.1f // torque unit is 0.1A

// in total 8 = 2 * 4 bytes for each direction

// only using first 2 bytes for torque encoding, the rest is reserved, total 128 bits

typedef struct
{
    uint8_t pos;
    uint8_t vel;
    uint8_t torque;
    char rawData[128]; // orginal data
    uint8_t size;      // actual data size
} MainMotorData_t;

typedef struct
{
    char rawData[128]; // orginal data
    uint8_t size;      // actual data size
} BalancerMotorData_t;

typedef struct
{
    char rawData[128]; // orginal data
    uint8_t size;      // actual data size
} UCommandData_t;

typedef struct 
{
    int8_t dirX;   // -100 ~ 100
    int8_t dirY;   // -100 ~ 100
    int8_t dirZ;   // -100 ~ 100
    int8_t dirW;   // -100 ~ 100
} Command_t;

typedef struct BLEData
{
    MainMotorData_t mainMotor;
    BalancerMotorData_t balancerMotor;
    UCommandData_t uCommand;
} BLEData; // all data received from BLE




// public functions
Command_t getDirectionCommand();





uint16_t numEncode(int torque, double factor); // use this to encode numbers to 2 bytes for transmission, for no 
int numDecode(int torque, double factor); // use this to decode 2 bytes back to numbers
void updateMotorData(char *rawData, int length);
void updateBalancerData(char *rawData, int length);
void loadCommand(char *data, int length);
volatile UCommandData_t *getCommand();


#endif /* BLE_H */
