#ifndef BLE_H
#define BLE_H

#include <stdint.h>

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

typedef struct BLEData
{
    MainMotorData_t mainMotor;
    BalancerMotorData_t balancerMotor;
    UCommandData_t uCommand;
} BLEData;

void updateMotorData(char *rawData, int length);
void updateBalancerData(char *rawData, int length);
void loadCommand(char *data, int length);
volatile UCommandData_t *getCommand();

#endif /* BLE_H */
