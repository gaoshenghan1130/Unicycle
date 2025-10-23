#include "../../Inc/BLE/ble.h"
#include "../../../STM32_WPAN/App/custom_stm.h"
#include "ble_std.h"
#include <stdint.h>
#include <stdio.h>
#include "custom_app.h"

volatile BLEData bleData;         // global BLE data
volatile MotorStatus motorStatus; // global motor status to send back

void encodeDouble2RawData(double value, volatile char *rawData,
                          int startIndex) {
  union {
    double d;
    uint8_t bytes[8];
  } converter;
  converter.d = value;

  for (int i = 0; i < 8; i++) {
    rawData[startIndex + i] = converter.bytes[i];
  }
}

void decodeRawData2Double(volatile char *rawData, int startIndex,
                          double *value) {
  union {
    double d;
    uint8_t bytes[8];
  } converter;

  for (int i = 0; i < 8; i++) {
    converter.bytes[i] = rawData[startIndex + i];
  }

  *value = converter.d;
}

void loadBLEData() // encode data into BLEData struct
{
  for (int i = 0; i < 128; i += 8) {
    double byte = 0;

    decodeRawData2Double(bleData.rawData, i, &byte);
    switch (i) {
    case 0:
      bleData.uCommand.mt = byte;
      break;
    case 8:
      bleData.uCommand.mp = byte;
      break;
    default:
      break;
    }

    if (i == 8)
      break; // only encode 2 doubles for command
  }
}

void receiveBLEData(char *rawData, int length) {
  for (int i = 0; i < length && i < 128; i++) {
    bleData.rawData[i] = rawData[i];
  }
  printf("received data from central\r\n");
  loadBLEData();
}

void encodeMotorStatusToRawData() {
  // encode data into rawData
  encodeDouble2RawData(motorStatus.pg, motorStatus.rawData, 0);
  encodeDouble2RawData(motorStatus.pdg, motorStatus.rawData, 8);
  encodeDouble2RawData(motorStatus.mp, motorStatus.rawData, 16);
  encodeDouble2RawData(motorStatus.mdp, motorStatus.rawData, 24);
}

void runBLE() {
  encodeMotorStatusToRawData();
  Custom_STM_App_Update_Char_Variable_Length(
      CUSTOM_STM_MM, (uint8_t *)motorStatus.rawData, 128);
  // print current command
  printf("BLE Command: mt = %.2f, mp = %.2f\r\n", bleData.uCommand.mt,
         bleData.uCommand.mp);
}
volatile BLEData *getBLEData() { return &bleData; }
