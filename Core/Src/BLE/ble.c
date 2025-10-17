#include "../../Inc/BLE/ble.h"
#include "../../../STM32_WPAN/App/custom_stm.h"
#include <stdint.h>


uint16_t numEncode(int torque, double factor)
{
    torque = (int)(torque / factor); // change to unit 0.1A
    if (torque > 4095 || torque < -4095)
        return 0xFFFF; // out of range

    uint16_t absTorque = (torque < 0) ? -torque : torque;
    uint8_t exponent = 0;

    // find exponent
    while (absTorque > 1023) // 10-bit mantissa
    {
        absTorque >>= 1;
        exponent++;
    }

    // encode to 16-bit
    uint16_t encoded = 0;
    if (torque < 0)
        encoded |= (1 << 13);      // sign bit: 13 th
    encoded |= ((exponent & 0x07) << TORQUE_MANT_LENGTH); // exp: bit12~bit10
    encoded |= (absTorque & 0x3FF);                       // mantissa: bit9~bit0

    return encoded;
}

int numDecode(int torque, double factor)
{
    if (torque == 0xFFFF)
        return 0; // error code

    int sign = (torque & (1 << 13)) ? -1 : 1; // sign bit: 13 th
    int exponent = (torque >> TORQUE_MANT_LENGTH) & 0x07; // exp: bit12~bit10
    int mantissa = torque & 0x3FF;                        // mantissa: bit9~bit0

    int decoded = mantissa << exponent; // reconstruct the number
    decoded *= sign;
    decoded = (int)(decoded * factor); // change back to A

    return decoded;
}

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

Command_t getDirectionCommand()
{
    Command_t cmd = {0, 0, 0, 0};
    for (int i=0; i < 16 && i < bleData.uCommand.size; i+=2) {
        uint16_t val = (uint8_t)bleData.uCommand.rawData[i] | ((uint8_t)bleData.uCommand.rawData[i+1] << 8);
        int decoded = numDecode(val, 1.0); // decode with factor 1.0
        int dir = val >> 13 & 0x03; // get direction
        switch (dir) {
            case 0:
                cmd.dirX = (decoded < -100) ? -100 : (decoded > 100) ? 100 : decoded;
                break;
            case 1:
                cmd.dirY = (decoded < -100) ? -100 : (decoded > 100) ? 100 : decoded;
                break;
            case 2:
                cmd.dirZ = (decoded < -100) ? -100 : (decoded > 100) ? 100 : decoded;
                break;
            case 3:
                cmd.dirW = (decoded < -100) ? -100 : (decoded > 100) ? 100 : decoded;
                break;
            default:
                break;
        }
    }
    return cmd;
}
