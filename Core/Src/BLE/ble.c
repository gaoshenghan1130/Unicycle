#include "../../Inc/BLE/ble.h"
#include "../../../STM32_WPAN/App/custom_stm.h"

volatile BLEData bleData = {
    .mainMotor = {
        .pos = 0,
        .vel = 0,
        .torque = 0,
        .rawData = {0}},
    .balancerMotor = {.rawData = {0}},
    .uCommand = {.rawData = {0}}};

volatile UCommandData_t *getCommand()
{
    return &bleData.uCommand;
}

void updateMotorData(char *rawData, int length)
{
    for (int i = 0; i < length && i < 128; i++)
    {
        bleData.mainMotor.rawData[i] = rawData[i];
    }
    bleData.mainMotor.size = length;
    Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_MM, (uint8_t *)bleData.mainMotor.rawData, length);
}

void updateBalancerData(char *rawData, int length)
{
    for (int i = 0; i < length && i < 128; i++)
    {
        bleData.balancerMotor.rawData[i] = rawData[i];
    }
    bleData.balancerMotor.size = length;
    Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_BM, (uint8_t *)bleData.balancerMotor.rawData, length);
}

void loadCommand(char *data, int length)
{
    for (int i = 0; i < length && i < 128; i++)
    {
        bleData.uCommand.rawData[i] = data[i];
    }
    bleData.uCommand.size = length;
}
