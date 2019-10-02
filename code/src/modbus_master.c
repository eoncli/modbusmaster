/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU
protocol).
*/
/*
  ModbusMaster.cpp - Arduino library for communicating with Modbus slaves
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

#include "modbusMaster.h"

/********************************************************************
 * Constants
 ********************************************************************/
#define MB_MAX_BUFFER_SIZE 64
#define MB_RESPONSE_TIMEOUT_VAL 2000 ///< Modbus timeout [milliseconds]

/********************************************************************
 * Function Codes Registers
 ********************************************************************/

// Modbus function codes for bit access
#define MBFC_READ_COILS 0x01           ///< Modbus FC01 Read Coils
#define MBFC_READ_DISCRETE_INPUTS 0x02 ///< Modbus FC02 Read Discrete Inputs
#define MBFC_WRITE_SINGLE_COIL 0x05    ///< Modbus FC05 Write Single Coil
#define MBFC_WRITE_MULTIPLE_COILS 0x0F ///< Modbus FC15 Write Single Coil

// Modbus function codes for 16 bit access
#define MBFC_READ_HOLDING_REGISTERS 0x03 ///< Modbus FC03 Read Holding Registers
#define MBFC_READ_INPUT_REGISTERS 0x04   ///< Modbus FC04 Read Input Registers
#define MBFC_WRITE_SINGLE_REGISTER 0x06  ///< Modbus FC06 Write Single Register
#define MBFC_WRITE_MULTIPLE_REGISTERS                                          \
  0x10                                ///< Modbus FC16 Write Multiple Registers
#define MBFC_MASK_WRITE_REGISTER 0x16 ///< Modbus FC22 Mask Write Register
#define MBFC_READ_WRITE_MULTIPLE_REGISTERS                                     \
  0x17 ///< Modbus FC23 Read Write Multiple Registers

/********************************************************************
 * Private variables
 ********************************************************************/

static uint8_t _slave = 0;
static uint16_t _write_address = 0;
static uint16_t _write_qty = 0;
static uint16_t _read_address = 0;
static uint16_t _read_qty = 0;
static uint16_t _transmit_buffer[MB_MAX_BUFFER_SIZE];
static uint8_t _transmit_buffer_index = 0;
static uint16_t _transmit_buffer_length = 0;
static uint16_t _response_buffer[MB_MAX_BUFFER_SIZE];
static uint8_t _response_buffer_index = 0;
static uint8_t _response_buffer_length = 0;

/********************************************************************
 * Private Functions Declaration
 ********************************************************************/

static uint8_t modbus_transaction(uint8_t mb_function_code);

/********************************************************************
 * Funciones públicas
 ********************************************************************/

void modbus_init(uint8_t slave) {
  _slave = slave;
  _transmit_buffer_index = 0;
  _transmit_buffer_length = 0;
}

uint8_t modbus_setTransmitBuffer(uint8_t index, uint16_t value) {
  if (index < MB_MAX_BUFFER_SIZE) {
    _transmit_buffer[index] = value;
    return MB_SUCCESS;
  } else {
    return MB_ILLEGAL_DATA_ADDR;
  }
}

void modbus_clearTransmitBuffer(void) {
  uint8_t i = 0;
  for (i = 0; i < MB_MAX_BUFFER_SIZE; i++) {
    _transmit_buffer[i] = 0;
  }
}

uint16_t modbus_getResponseBuffer(uint8_t index) {
  if (index < MB_MAX_BUFFER_SIZE) {
    return _response_buffer[index];
  } else {
    return 0xFFFF;
  }
}

void modbus_clearResponseBuffer(void) {
  uint8_t i;
  for (i = 0; i < MB_MAX_BUFFER_SIZE; i++) {
    _response_buffer[i] = 0;
  }
}

uint8_t modbus_readCoils(uint16_t readAddress, uint16_t bitQty) {
  _read_address = readAddress;
  _read_qty = bitQty;
  return modbus_transaction(MBFC_READ_COILS);
}

uint8_t modbus_readDiscreteInputs(uint16_t readAddress, uint16_t bitQty) {
  _read_address = readAddress;
  _read_qty = bitQty;
  return modbus_transaction(MBFC_READ_DISCRETE_INPUTS);
}

uint8_t modbus_readHoldingRegisters(uint16_t readAddress, uint16_t readQty) {
  _read_address = readAddress;
  _read_qty = readQty;
  return modbus_transaction(MBFC_READ_HOLDING_REGISTERS);
}

uint8_t modbus_readInputRegisters(uint16_t readAddress, uint8_t readQty) {
  _read_address = readAddress;
  _read_qty = readQty;
  return modbus_transaction(MBFC_READ_INPUT_REGISTERS);
}

uint8_t modbus_writeSingleCoil(uint16_t writeAddress, uint8_t state) {
  _write_address = writeAddress;
  _write_qty = (state ? 0xFF00 : 0x0000);
  return modbus_transaction(MBFC_WRITE_SINGLE_COIL);
}

uint8_t modbus_writeSingleRegister(uint16_t writeAddress, uint16_t writeValue) {
  _write_address = writeAddress;
  _write_qty = 0;
  _transmit_buffer[0] = writeValue;
  return modbus_transaction(MBFC_WRITE_SINGLE_REGISTER);
}

uint8_t modbus_writeMultipleCoils(uint16_t writeAddress, uint16_t bitQty) {
  _write_address = writeAddress;
  _write_qty = bitQty;
  return modbus_transaction(MBFC_WRITE_MULTIPLE_COILS);
}

uint8_t modbus_writeMultipleRegisters(uint16_t writeAddress,
                                      uint16_t writeQty) {
  _write_address = writeAddress;
  _write_qty = writeQty;
  return modbus_transaction(MBFC_WRITE_MULTIPLE_REGISTERS);
}

uint8_t modbus_maskWriteRegister(uint16_t writeAddress, uint16_t andMask,
                                 uint16_t orMask) {
  _write_address = writeAddress;
  _transmit_buffer[0] = andMask;
  _transmit_buffer[1] = orMask;
  return modbus_transaction(MBFC_MASK_WRITE_REGISTER);
}

uint8_t modbus_readWriteMultipleRegisters(uint16_t readAddress,
                                          uint16_t readQty,
                                          uint16_t writeAddress,
                                          uint16_t writeQty) {
  _read_address = readAddress;
  _read_qty = readQty;
  _write_address = writeAddress;
  _write_qty = writeQty;
  return modbus_transaction(MBFC_READ_WRITE_MULTIPLE_REGISTERS);
}

/********************************************************************
 * Funciones privadas
 ********************************************************************/
static uint8_t modbus_transaction(uint8_t mb_function_code) {
  uint8_t modbusADU[256];
  uint8_t modbusADUSize = 0;
  uint8_t i, qty;
  uint16_t crc;
  uint32_t startTime;
  uint8_t bytesLeft = 8;
  uint8_t status = MB_SUCCESS;

  modbusADU[modbusADUSize++] = _slave;
  modbusADU[modbusADUSize++] = mb_function_code;

  switch (mb_function_code) {
    case MBFC_READ_COILS:
    case MBFC_READ_DISCRETE_INPUTS:
    case MBFC_READ_INPUT_REGISTERS:
    case MBFC_READ_HOLDING_REGISTERS:
    case MBFC_READ_WRITE_MULTIPLE_REGISTERS:
      modbusADU[modbusADUSize++] = highByte(_read_address);
      modbusADU[modbusADUSize++] = lowByte(_read_address);
      modbusADU[modbusADUSize++] = highByte(_read_qty);
      modbusADU[modbusADUSize++] = lowByte(_read_qty);
      break;
  }

  switch (mb_function_code) {
    case MBFC_WRITE_SINGLE_COIL:
    case MBFC_MASK_WRITE_REGISTER:
    case MBFC_WRITE_MULTIPLE_COILS:
    case MBFC_WRITE_SINGLE_REGISTER:
    case MBFC_WRITE_MULTIPLE_REGISTERS:
    case MBFC_READ_WRITE_MULTIPLE_REGISTERS:
      modbusADU[modbusADUSize++] = highByte(_write_address);
      modbusADU[modbusADUSize++] = lowByte(_write_address);
      break;
  }

  switch (mb_function_code) {
    case MBFC_WRITE_SINGLE_COIL:
      modbusADU[modbusADUSize++] = highByte(_write_qty);
      modbusADU[modbusADUSize++] = lowByte(_write_qty);
      break;

    case MBFC_WRITE_SINGLE_REGISTER:
      modbusADU[modbusADUSize++] = highByte(_transmit_buffer[0]);
      modbusADU[modbusADUSize++] = lowByte(_transmit_buffer[0]);
      break;

    case MBFC_WRITE_MULTIPLE_COILS:
      modbusADU[modbusADUSize++] = highByte(_write_qty);
      modbusADU[modbusADUSize++] = lowByte(_write_qty);
      qty = (_write_qty % 8) ? ((_write_qty >> 3) + 1) : (_write_qty >> 3);
      modbusADU[modbusADUSize++] = qty;
      for (i = 0; i < qty; i++) {
        switch (i % 2) {
          case 0: // i is even
            modbusADU[modbusADUSize++] = lowByte(_transmit_buffer[i >> 1]);
            break;

          case 1: // i is odd
            modbusADU[modbusADUSize++] = highByte(_transmit_buffer[i >> 1]);
            break;
        }
      }
      break;

    case MBFC_WRITE_MULTIPLE_REGISTERS:
    case MBFC_READ_WRITE_MULTIPLE_REGISTERS:
      modbusADU[modbusADUSize++] = highByte(_write_qty);
      modbusADU[modbusADUSize++] = lowByte(_write_qty);
      modbusADU[modbusADUSize++] = lowByte(_write_qty << 1);

      for (i = 0; i < lowByte(_write_qty); i++) {
        modbusADU[modbusADUSize++] = highByte(_transmit_buffer[i]);
        modbusADU[modbusADUSize++] = lowByte(_transmit_buffer[i]);
      }
      break;

    case MBFC_MASK_WRITE_REGISTER:
      modbusADU[modbusADUSize++] = highByte(_transmit_buffer[0]);
      modbusADU[modbusADUSize++] = lowByte(_transmit_buffer[0]);
      modbusADU[modbusADUSize++] = highByte(_transmit_buffer[1]);
      modbusADU[modbusADUSize++] = lowByte(_transmit_buffer[1]);
      break;
  }

  // append CRC
  crc = 0xFFFF;
  for (i = 0; i < modbusADUSize; i++) {
    crc = crc16_update(crc, modbusADU[i]);
  }
  modbusADU[modbusADUSize++] = lowByte(crc);
  modbusADU[modbusADUSize++] = highByte(crc);
  modbusADU[modbusADUSize] = 0;

  // flush receive buffer before transmitting request
  while (MBSER_AVAILABLE()) {
    MBSER_READ();
  }

  for (i = 0; i < modbusADUSize; i++) {
    MBSER_WRITE(modbusADU[i]);
  }

  modbusADUSize = 0;
  // uart_flush();    // flush transmit buffer

  // loop until we run out of time or bytes, or an error occurs
  startTime = millis();
  while (bytesLeft && !status) {
    if (MBSER_AVAILABLE()) {
      modbusADU[modbusADUSize++] = MBSER_READ();
      bytesLeft--;
    }

    // evaluate slave ID, function code once enough bytes have been read
    if (modbusADUSize == 5) {
      // verify response is for correct Modbus slave
      if (modbusADU[0] != _slave) {
        status = MB_INVALID_SLAVE_ID;
        break;
      }

      // verify response is for correct Modbus function code (mask exception bit
      // 7)
      if ((modbusADU[1] & 0x7F) != mb_function_code) {
        status = MB_INVALID_FUNCTION;
        break;
      }

      // check whether Modbus exception occurred; return Modbus Exception Code
      if (bitRead(modbusADU[1], 7)) {
        status = modbusADU[2];
        break;
      }

      // evaluate returned Modbus function code
      switch (modbusADU[1]) {
        case MBFC_READ_COILS:
        case MBFC_READ_DISCRETE_INPUTS:
        case MBFC_READ_INPUT_REGISTERS:
        case MBFC_READ_HOLDING_REGISTERS:
        case MBFC_READ_WRITE_MULTIPLE_REGISTERS:
          bytesLeft = modbusADU[2];
          break;

        case MBFC_WRITE_SINGLE_COIL:
        case MBFC_WRITE_MULTIPLE_COILS:
        case MBFC_WRITE_SINGLE_REGISTER:
        case MBFC_WRITE_MULTIPLE_REGISTERS: bytesLeft = 3; break;

        case MBFC_MASK_WRITE_REGISTER: bytesLeft = 5; break;
      }
    }

    if ((millis() - startTime) > MB_RESPONSE_TIMEOUT_VAL) {
      status = MB_RESPONSE_TIMEOUT;
    }
  }

  // verify response is large enough to inspect further
  if (!status && modbusADUSize >= 5) {
    // calculate CRC
    crc = 0xFFFF;
    for (i = 0; i < (modbusADUSize - 2); i++) {
      crc = crc16_update(crc, modbusADU[i]);
    }

    // verify CRC
    if (!status && (lowByte(crc) != modbusADU[modbusADUSize - 2] ||
                    highByte(crc) != modbusADU[modbusADUSize - 1])) {
      status = MB_INVALID_CRC;
    }
  }

  // disassemble ADU into words
  if (!status) {
    // evaluate returned Modbus function code
    switch (modbusADU[1]) {
      case MBFC_READ_COILS:
      case MBFC_READ_DISCRETE_INPUTS:
        // load bytes into word; response bytes are ordered L, H, L, H, ...
        for (i = 0; i < (modbusADU[2] >> 1); i++) {
          if (i < MB_MAX_BUFFER_SIZE) {
            _response_buffer[i] =
                makeHalfWord(modbusADU[2 * i + 4], modbusADU[2 * i + 3]);
          }

          _response_buffer_length = i;
        }

        // in the event of an odd number of bytes, load last byte into
        // zero-padded word
        if (modbusADU[2] % 2) {
          if (i < MB_MAX_BUFFER_SIZE) {
            _response_buffer[i] = makeHalfWord(0, modbusADU[2 * i + 3]);
          }

          _response_buffer_length = i + 1;
        }
        break;

      case MBFC_READ_INPUT_REGISTERS:
      case MBFC_READ_HOLDING_REGISTERS:
      case MBFC_READ_WRITE_MULTIPLE_REGISTERS:
        // load bytes into word; response bytes are ordered H, L, H, L, ...
        for (i = 0; i < (modbusADU[2] >> 1); i++) {
          if (i < MB_MAX_BUFFER_SIZE) {
            _response_buffer[i] =
                makeHalfWord(modbusADU[2 * i + 3], modbusADU[2 * i + 4]);
          }

          _response_buffer_length = i;
        }
        break;
    }
  }

  _transmit_buffer_index = 0;
  _transmit_buffer_length = 0;
  _response_buffer_index = 0;
  return status;
}
