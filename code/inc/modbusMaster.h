/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU
protocol).
@defgroup setup ModbusMaster Object Instantiation/Initialization
@defgroup buffer ModbusMaster Buffer Management
@defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
@defgroup register Modbus Function Codes for Holding/Input Registers
@defgroup constant Modbus Function Codes, Exception Codes
*/
/*
  ModbusMaster.h - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).
  Library:: ModbusMaster
  Copyright:: 2009-2016 Doc Walker
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
      http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef __MODBUS_MASTER_H_
#define __MODBUS_MASTER_H_

#include "eonOS.h"

// Uart
#define MBSER_WRITE uart1_write
#define MBSER_AVAILABLE uart1_available
#define MBSER_READ uart1_read

// Response codes
#define MB_SUCCESS 0x00
#define MB_ILLEGAL_FUNC 0x01
#define MB_ILLEGAL_DATA_ADDR 0x02
#define MB_ILLEGAL_DATA_VAL 0x03
#define MB_SLAVE_DEV_FAILURE 0x04
#define MB_INVALID_SLAVE_ID 0xE0
#define MB_INVALID_FUNCTION 0xE1
#define MB_RESPONSE_TIMEOUT 0xE2
#define MB_INVALID_CRC 0xE3

// Functions
void modbus_init(uint8_t slave);

uint8_t modbus_setTransmitBuffer(uint8_t index, uint16_t value);
void modbus_clearTransmitBuffer(void);

uint16_t modbus_getResponseBuffer(uint8_t index);
void modbus_clearResponseBuffer(void);

uint8_t modbus_readCoils(uint16_t readAddress, uint16_t bitQty);
uint8_t modbus_readDiscreteInputs(uint16_t readAddress, uint16_t bitQty);
uint8_t modbus_readHoldingRegisters(uint16_t readAddress, uint16_t readQty);
uint8_t modbus_readInputRegisters(uint16_t readAddress, uint8_t readQty);
uint8_t modbus_writeSingleCoil(uint16_t writeAddress, uint8_t state);
uint8_t modbus_writeSingleRegister(uint16_t writeAddress, uint16_t writeValue);
uint8_t modbus_writeMultipleCoils(uint16_t writeAddress, uint16_t bitQty);
uint8_t modbus_writeMultipleRegisters(uint16_t writeAddress, uint16_t writeQty);
uint8_t modbus_maskWriteRegister(uint16_t writeAddress, uint16_t andMask,
                                 uint16_t orMask);
uint8_t modbus_readWriteMultipleRegisters(uint16_t readAddress,
                                          uint16_t readQty,
                                          uint16_t writeAddress,
                                          uint16_t writeQty);

#endif