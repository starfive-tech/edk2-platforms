/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#ifndef __SPI_FLASH_PROTOCOL_H__
#define __SPI_FLASH_PROTOCOL_H__

#include <Protocol/FirmwareManagement.h>
#include <Protocol/Spi.h>

extern EFI_GUID gMarvellSpiFlashProtocolGuid;

typedef struct _SPI_FLASH_PROTOCOL SPI_FLASH_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *SPI_FLASH_INIT) (
  IN SPI_FLASH_PROTOCOL *This,
  IN SPI_DEVICE *SpiDev
  );

typedef
EFI_STATUS
(EFIAPI *SPI_FLASH_READ_ID) (
  IN SPI_DEVICE *SpiDev,
  IN BOOLEAN UseInRuntime
  );

typedef
EFI_STATUS
(EFIAPI *SPI_FLASH_READ) (
  IN SPI_DEVICE *SpiDev,
  IN UINT32 Address,
  IN UINTN DataByteCount,
  IN VOID *Buffer
  );

typedef
EFI_STATUS
(EFIAPI *SPI_FLASH_WRITE) (
  IN SPI_DEVICE *SpiDev,
  IN UINT32 Address,
  IN UINTN DataByteCount,
  IN VOID *Buffer
  );

typedef
EFI_STATUS
(EFIAPI *SPI_FLASH_ERASE) (
  IN SPI_DEVICE *SpiDev,
  IN UINTN Address,
  IN UINTN DataByteCount
  );

struct _SPI_FLASH_PROTOCOL {
  SPI_FLASH_INIT    Init;
  SPI_FLASH_READ_ID ReadId;
  SPI_FLASH_READ    Read;
  SPI_FLASH_WRITE   Write;
  SPI_FLASH_ERASE   Erase;
};

#endif // __SPI_FLASH_PROTOCOL_H__
