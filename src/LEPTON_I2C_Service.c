/*******************************************************************************
 *
 *    FILE: LEPTON_I2C_Service.c
 *
 *    DESCRIPTION: I2C Driver Service Interface
 *
 *    AUTHOR: Dart
 *
 *    CREATED: 4/10/2012
 *
 *    HISTORY: 4/10/2012 DWD Initial Draft
 *
 *    This service layer supports the Lepton protocol above the
 *    I2C device.  Lepton supported I2C drivers will attach in
 *    each of these supported functions.
 *
 **      Copyright 2011,2012,2013,2014 FLIR Systems - Commercial
 **      Vision Systems.  All rights reserved.
 **
 **      Proprietary - PROPRIETARY - FLIR Systems Inc..
 **
 **      This document is controlled to FLIR Technology Level 2.
 **      The information contained in this document pertains to a
 **      dual use product Controlled for export by the Export
 **      Administration Regulations (EAR). Diversion contrary to
 **      US law is prohibited.  US Department of Commerce
 **      authorization is not required prior to export or
 **      transfer to foreign persons or parties unless otherwise
 **      prohibited.
 **
 **      Redistribution and use in source and binary forms, with
 **      or without modification, are permitted provided that the
 **      following conditions are met:
 **
 **      Redistributions of source code must retain the above
 **      copyright notice, this list of conditions and the
 **      following disclaimer.
 **
 **      Redistributions in binary form must reproduce the above
 **      copyright notice, this list of conditions and the
 **      following disclaimer in the documentation and/or other
 **      materials provided with the distribution.
 **
 **      Neither the name of the FLIR Systems Corporation nor the
 **      names of its contributors may be used to endorse or
 **      promote products derived from this software without
 **      specific prior written permission.
 **
 **      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 **      CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 **      WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 **      PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 **      COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
 **      DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 **      CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 **      PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 **      USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 **      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 **      CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 **      NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 **      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 **      OF SUCH DAMAGE.
 **
 *******************************************************************************/

#include "LEPTON_Types.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_I2C_Service.h"
#include "FLIR_I2C.h"

#define LEP_I2C_BUSY_BIT_MASK                   0x0001   /* Bit 0 is the Busy Bit */

/* Driver Open
 */
LEP_RESULT LEP_I2C_MasterOpen(LEP_UINT16 portID, LEP_UINT16 *portBaudRate, LEP_INT8 pinSDA, LEP_INT8 pinSCL)
{
    LEP_RESULT result;

    /* Call the I2C Device-Specific Driver to open device as a
     ** Master
     */
    result = DEV_I2C_MasterInit( portID, portBaudRate, pinSDA, pinSCL );

    return(result);
}

/* Driver Close
 */
LEP_RESULT LEP_I2C_MasterClose(LEP_CAMERA_PORT_DESC_T_PTR portDescriptorPtr)
{
    LEP_RESULT result = LEP_OK;

    /* Do any device-specific calls to implement a close operation
     */
    result = DEV_I2C_MasterClose(portDescriptorPtr->portID);
    return(result);
}

/**
 * Driver Read
 *    Use Lepton I2C protocol for READ starting from current
 *      location
 * 
 * @param portID        User-defined parameter to identify one of multiple ports
 * 
 * @param deviceAddress This is the Lepton TWI/CCI (I2C) device address.
 * 
 * @param subAddress    Specifies the Lepton Register Address to write to
 * 
 * @param dataPtr       Pointer to the DATA buffer that is filled by this command
 * 
 * @param dataLength    Number of 16-bit words to read.
 * 
 * @return 
 */
LEP_RESULT LEP_I2C_MasterReadData(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 subAddress,
    LEP_UINT16 *dataPtr,
    LEP_UINT16 dataLength)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 transactionStatus;
    LEP_UINT16 numWordsRead;

    result = DEV_I2C_MasterReadData(portID,
        deviceAddress,
        subAddress,
        dataPtr,
        dataLength,
        &numWordsRead,
        &transactionStatus
    );

    return(result);
}

/**
 * Driver Write
 * 
 * @param portID     User-defined parameter to identify one of multiple ports
 * 
 * @param subAddress Specifies the Lepton Register Address to write to
 * 
 * @param dataPtr    Pointer to a DATA buffer to source the transfers
 * 
 * @param dataLength Specifies the number of 16-bit words to write from the bufffer
 * 
 * @return LEP_RESULT   LEP_OK if all goes well; otherwise a Lepton error code.
 */
LEP_RESULT LEP_I2C_MasterWriteData(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 subAddress,
    LEP_UINT16 *dataPtr,
    LEP_UINT16 dataLength)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 transactionStatus;
    LEP_UINT16 numWordsWritten;

    result = DEV_I2C_MasterWriteData(portID,
        deviceAddress,
        subAddress,
        dataPtr,
        dataLength,
        &numWordsWritten,
        &transactionStatus
    );
    return(result);
}


LEP_RESULT LEP_I2C_MasterReadRegister(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 regAddress,
    LEP_UINT16 *regValue)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 transactionStatus;

    result = DEV_I2C_MasterReadRegister(portID,
        deviceAddress,
        regAddress,
        regValue,
        &transactionStatus
    );
    return(result);
}


LEP_RESULT LEP_I2C_MasterWriteRegister(
    LEP_UINT16 portID,
    LEP_UINT8  deviceAddress,
    LEP_UINT16 regAddress,
    LEP_UINT16 regValue)
{
    LEP_RESULT result = LEP_OK;
    LEP_UINT16 transactionStatus;

    result = DEV_I2C_MasterWriteRegister(portID,
        deviceAddress,
        regAddress,
        regValue,
        &transactionStatus
    );
    return(result);
}
