/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#ifndef __SPI_DXE_H__
#define __SPI_DXE_H__

#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Uefi/UefiBaseType.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Protocol/Spi.h>

#define SPI_MASTER_SIGNATURE                    SIGNATURE_32 ('M', 'S', 'P', 'I')
#define SPI_MASTER_FROM_SPI_MASTER_PROTOCOL(a)  CR (a, SPI_MASTER, SpiMasterProtocol, SPI_MASTER_SIGNATURE)

#define SPI_CMD_READ                    0
#define SPI_CMD_WRITE                   1
#define SPI_READ                        2
#define SPI_WRITE                       3

#define SZ_8M                           0x00800000
#define NSEC_PER_SEC			              1000000000L
#define DIV_ROUND_UP(n,d)               (((n) + (d) - 1) / (d))
#define SPI_READ_CAPTURE_MAX_DELAY	    16

typedef struct {
  SPI_MASTER_PROTOCOL SpiMasterProtocol;
  UINTN                   Signature;
  EFI_HANDLE              Handle;
  EFI_LOCK                Lock;
} SPI_MASTER;

#endif //__SPI_DXE_H__
