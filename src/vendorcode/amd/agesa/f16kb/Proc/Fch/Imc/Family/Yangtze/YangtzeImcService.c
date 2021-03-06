/* $NoKeywords:$ */
/**
 * @file
 *
 * Config Fch Imc controller
 *
 * Init Imc Controller features.
 *
 * @xrefitem bom "File Content Label" "Release Content"
 * @e project:     AGESA
 * @e sub-project: FCH
 * @e \$Revision: 87213 $   @e \$Date: 2013-01-30 15:37:54 -0600 (Wed, 30 Jan 2013) $
 *
 */
/*
*****************************************************************************
*
 * Copyright (c) 2008 - 2013, Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ADVANCED MICRO DEVICES, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************
*/
#include "FchPlatform.h"
#include "Filecode.h"
#define FILECODE PROC_FCH_IMC_FAMILY_YANGTZE_YANGTZEIMCSERVICE_FILECODE

//
// Declaration of local functions
//


/**
 * SoftwareToggleImcStrapping - Software Toggle IMC Firmware Strapping.
 *
 *
 * @param[in] FchDataPtr Fch configuration structure pointer.
 *
 */
VOID
SoftwareToggleImcStrapping (
  IN  VOID     *FchDataPtr
  )
{
  UINT8    ValueByte;
  AMD_CONFIG_PARAMS      *StdHeader;
  FCH_DATA_BLOCK         *LocalCfgPtr;

  StdHeader = ((FCH_DATA_BLOCK *) FchDataPtr)->StdHeader;
  LocalCfgPtr = (FCH_DATA_BLOCK *) FchDataPtr;

  if ( LocalCfgPtr->Imc.ImcEnableOverWrite == 1 ) {
    RwMem (ACPI_MMIO_BASE + PMIO_BASE + FCH_PMIOA_REGD6 + 1, AccessWidth8, 0x7F, BIT7);

    ValueByte = 0x0;
    while (ValueByte == 0) {
      FchStall (10000, StdHeader);
      ReadPci ((LPC_BUS_DEV_FUN << 16) + FCH_LPC_REG40, AccessWidth8, &ValueByte, StdHeader);
      ValueByte &= 0x80;
    };
    RwMem (ACPI_MMIO_BASE + PMIO_BASE + FCH_PMIOA_REGC4, AccessWidth8, 0xF7, 0x08);

    EnterEcConfig (StdHeader);
    RwEc8 (FCH_EC_REG07, 0x00, 0x09, StdHeader);                      ///switch to device 9 (Mailbox)
    RwEc8 (FCH_EC_REG60, 0x00, (MailBoxPort >> 8), StdHeader);        ///set MSB of Mailbox port
    RwEc8 (FCH_EC_REG61, 0x00, (MailBoxPort & 0xFF), StdHeader);      ///set LSB of Mailbox port
    RwEc8 (FCH_EC_REG30, 0x00, 0x01, StdHeader);                      ///;Enable Mailbox Registers Interface, bit0=1

    RwMem (ACPI_MMIO_BASE + SMI_BASE + FCH_SMI_REGB3, AccessWidth8, (UINT32)~BIT6, BIT6);
    ExitEcConfig (StdHeader);

    ImcSleep (FchDataPtr);
  } else {
    RwMem (ACPI_MMIO_BASE + PMIO_BASE + FCH_PMIOA_REGD6 + 1, AccessWidth8, 0x7F, 0);
  }
}

