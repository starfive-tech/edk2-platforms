/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#ifndef __SPI_FLASH_DXE_H__
#define __SPI_FLASH_DXE_H__

#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Uefi/UefiBaseType.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Protocol/Spi.h>
#include <Protocol/SpiFlash.h>

#define SPI_FLASH_SIGNATURE             SIGNATURE_32 ('F', 'S', 'P', 'I')

#define CMD_READ_ID                     0x9f
#define READ_STATUS_REG_CMD             0x0b
#define CMD_WRITE_ENABLE                0x06
#define CMD_READ_STATUS                 0x05
#define CMD_FLAG_STATUS                 0x70
#define CMD_WRITE_STATUS_REG            0x01
#define CMD_READ_ARRAY_FAST             0x0b
#define CMD_PAGE_PROGRAM                0x02
#define CMD_BANK_WRITE                  0xc5
#define CMD_BANKADDR_BRWR               0x17
#define CMD_ERASE_4K                    0x20
#define CMD_ERASE_32K                   0x52
#define CMD_ERASE_64K                   0xd8
#define CMD_4B_ADDR_ENABLE              0xb7
#define CMD_READ_DATA                   0x03

#define STATUS_REG_POLL_WIP_MSK         BIT(0)
#define STATUS_REG_POLL_PEC_MSK         BIT(7)

#define SPI_FLASH_16MB_BOUND            0x1000000

typedef struct {
  SPI_FLASH_PROTOCOL  SpiFlashProtocol;
  UINTN                   Signature;
  EFI_HANDLE              Handle;
} SPI_FLASH_INSTANCE;

EFI_STATUS
EFIAPI
EfiSpiFlashInit (
  IN SPI_FLASH_PROTOCOL *This,
  IN SPI_DEVICE *Slave
  );

#endif //__SPI_FLASH_DXE_H__
