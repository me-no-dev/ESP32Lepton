/*******************************************************************************
 **
 **    File NAME: FLIR_I2C.c
 **
 **      AUTHOR:  Hart Thomson
 **
 **      CREATED: 7/10/2015
 **
 **      DESCRIPTION: Lepton Device-Specific Driver for various Master I2C Devices
 **
 **      HISTORY:  7/19/2015 HT - Initial Draft
 **
 ** Copyright 2010, 2011, 2012, 2013, 2014, 2015 FLIR Systems - Commercial Vision Systems
 **
 **  All rights reserved.
 **
 **  Redistribution and use in source and binary forms, with or without
 **  modification, are permitted provided that the following conditions are met:
 **
 **  Redistributions of source code must retain the above copyright notice, this
 **  list of conditions and the following disclaimer.
 **
 **  Redistributions in binary form must reproduce the above copyright notice,
 **  this list of conditions and the following disclaimer in the documentation
 **  and/or other materials provided with the distribution.
 **
 **  Neither the name of the Indigo Systems Corporation nor the names of its
 **  contributors may be used to endorse or promote products derived from this
 **  software without specific prior written permission.
 **
 **  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 **  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 **  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 **  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 **  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 **  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 **  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 **  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 **  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 **  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 **  THE POSSIBILITY OF SUCH DAMAGE.
 **
 *******************************************************************************/

#include "LEPTON_Types.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_MACROS.h"
#include "FLIR_I2C.h"
#include "LEPTON_I2C_Reg.h"
#include <stdio.h>
#include <stdlib.h>

#define ADDRESS_SIZE_BYTES  2
#define VALUE_SIZE_BYTES    2
#define I2C_BUFFER_SIZE (ADDRESS_SIZE_BYTES + LEP_I2C_DATA_BUFFER_0_LENGTH)
LEP_UINT8 tx[I2C_BUFFER_SIZE];
LEP_UINT8 rx[I2C_BUFFER_SIZE];

LEP_CMD_PACKET_T cmdPacket;
LEP_RESPONSE_PACKET_T responsePacket;

extern LEP_RESULT ESP_CCI_Init(uint16_t portID, uint16_t baudRateInkHz, int8_t pinSDA, int8_t pinSCL);
extern LEP_RESULT ESP_CCI_Close(uint16_t portID);
extern void ESP_CCI_Write(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * txdata, uint32_t * bytesActuallyWritten);
extern void ESP_CCI_Read(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * rxdata, uint32_t * bytesActuallyRead);

/******************************************************************************/
/**
 * Performs I2C Master Initialization
 * 
 * @param portID     LEP_UINT16  User specified port ID tag.  Can be used to
 *                   select between multiple cameras
 * 
 * @param BaudRate   Clock speed in kHz. Typically this is 400.
 *                   The Device Specific Driver will try to match the desired
 *                   speed.  This parameter is updated to the actual speed the
 *                   driver can use.
 * 
 * @return LEP_RESULT  0 if all goes well, errno otherwise
 */
LEP_RESULT DEV_I2C_MasterInit(LEP_UINT16 portID, LEP_UINT16 *BaudRate, LEP_INT8 pinSDA, LEP_INT8 pinSCL)
{
    LEP_RESULT result = LEP_OK;
    result = ESP_CCI_Init(portID, *BaudRate, pinSDA, pinSCL);
    return(result);
}

/**
 * Closes the I2C driver connection.
 * 
 * @return LEP_RESULT  0 if all goes well, errno otherwise.
 */
LEP_RESULT DEV_I2C_MasterClose(LEP_UINT16 portID)
{
    LEP_RESULT result = LEP_OK;
    result = ESP_CCI_Close(portID);
    return(result);
}

LEP_RESULT DEV_I2C_MasterReadData(
    LEP_UINT16  portID,               // User-defined port ID
    LEP_UINT8   deviceAddress,        // Lepton Camera I2C Device Address
    LEP_UINT16  regAddress,           // Lepton Register Address
    LEP_UINT16 *readDataPtr,          // Read DATA buffer pointer
    LEP_UINT16  wordsToRead,          // Number of 16-bit words to Read
    LEP_UINT16 *numWordsRead,         // Number of 16-bit words actually Read
    LEP_UINT16 *status                // Transaction Status
)
{
    LEP_RESULT result = LEP_OK;

    LEP_UINT32 bytesToWrite = ADDRESS_SIZE_BYTES;
    LEP_UINT32 bytesToRead = wordsToRead << 1;
    LEP_UINT32 bytesActuallyWritten = 0;
    LEP_UINT32 bytesActuallyRead = 0;
    LEP_UINT32 wordsActuallyRead = 0;
    LEP_UINT8* txdata = &tx[0];
    LEP_UINT8* rxdata = &rx[0];
    LEP_UINT16 *dataPtr;
    LEP_UINT16 *writePtr;

    *(LEP_UINT16*)txdata = REVERSE_ENDIENESS_UINT16(regAddress);


    //Write the address, which is 2 bytes
    ESP_CCI_Write(portID, deviceAddress, ADDRESS_SIZE_BYTES, (LEP_UINT8*)txdata, (LEP_UINT32*)&bytesActuallyWritten);
    if(bytesActuallyWritten != bytesToWrite) {
        result = LEP_ERROR;
    }

    //Read back the data at the address written above
    ESP_CCI_Read(portID, deviceAddress, (LEP_UINT32)bytesToRead, (LEP_UINT8*)rxdata, (LEP_UINT32*)&bytesActuallyRead);
    if(bytesActuallyRead != bytesToRead) {
        result = LEP_ERROR;
    }

    wordsActuallyRead = (LEP_UINT16)(bytesActuallyRead >> 1);
    *numWordsRead = wordsActuallyRead;

    if(result == LEP_OK) {
        dataPtr = (LEP_UINT16*)&rxdata[0];
        writePtr = readDataPtr;
        while(wordsActuallyRead--) {
            *writePtr++ = REVERSE_ENDIENESS_UINT16(*dataPtr);
            dataPtr++;
        }
    }

    return(result);
}

LEP_RESULT DEV_I2C_MasterWriteData(
    LEP_UINT16  portID,              // User-defined port ID
    LEP_UINT8   deviceAddress,       // Lepton Camera I2C Device Address
    LEP_UINT16  regAddress,          // Lepton Register Address
    LEP_UINT16 *writeDataPtr,        // Write DATA buffer pointer
    LEP_UINT16  wordsToWrite,        // Number of 16-bit words to Write
    LEP_UINT16 *numWordsWritten,     // Number of 16-bit words actually written
    LEP_UINT16 *status)              // Transaction Status
{
    LEP_RESULT result = LEP_OK;

    LEP_INT32 bytesOfDataToWrite = (wordsToWrite << 1);
    LEP_INT32 bytesToWrite = bytesOfDataToWrite + ADDRESS_SIZE_BYTES;
    LEP_INT32 bytesActuallyWritten = 0;
    LEP_UINT8* txdata = &tx[0];
    LEP_UINT16 *dataPtr;
    LEP_UINT16 *txPtr;

    *(LEP_UINT16*)txdata = REVERSE_ENDIENESS_UINT16(regAddress);
    dataPtr = (LEP_UINT16*)&writeDataPtr[0];
    txPtr = (LEP_UINT16*)&txdata[ADDRESS_SIZE_BYTES];
    while(wordsToWrite--){
        *txPtr++ = (LEP_UINT16)REVERSE_ENDIENESS_UINT16(*dataPtr);
        dataPtr++;
    }

    ESP_CCI_Write(portID, deviceAddress, bytesToWrite, (LEP_UINT8*)txdata, (LEP_UINT32*)&bytesActuallyWritten);
    if(bytesActuallyWritten != bytesToWrite) {
        result = LEP_ERROR;
    }

    *numWordsWritten = (bytesActuallyWritten >> 1);

    return(result);
}

LEP_RESULT DEV_I2C_MasterReadRegister(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 regAddress,
    LEP_UINT16 *regValue,     // Number of 16-bit words actually written
    LEP_UINT16 *status
)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 wordsActuallyRead;
    result = DEV_I2C_MasterReadData(portID, deviceAddress, regAddress, regValue, 1 /*1 word*/, &wordsActuallyRead, status);
    return(result);
}

LEP_RESULT DEV_I2C_MasterWriteRegister(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 regAddress,
    LEP_UINT16 regValue,     // Number of 16-bit words actually written
    LEP_UINT16 *status
)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 wordsActuallyWritten;
    result = DEV_I2C_MasterWriteData(portID, deviceAddress, regAddress, &regValue, 1, &wordsActuallyWritten, status);
    return(result);
}
