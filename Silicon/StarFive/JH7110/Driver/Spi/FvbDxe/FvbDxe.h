/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#ifndef __FVB_DXE_H__
#define __FVB_DXE_H__

#include <Protocol/BlockIo.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include <Protocol/Spi.h>
#include <Protocol/SpiFlash.h>

#define GET_DATA_OFFSET(BaseAddr, Lba, LbaSize) ((BaseAddr) + (UINTN)((Lba) * (LbaSize)))

#define FVB_FLASH_SIGNATURE                       SIGNATURE_32('S', 'n', 'o', 'r')
#define INSTANCE_FROM_FVB_THIS(a)                 CR(a, FVB_DEVICE, FvbProtocol, FVB_FLASH_SIGNATURE)

//
// Define two helper macro to extract the Capability field or Status field in FVB
// bit fields.
//
#define EFI_FVB2_CAPABILITIES (EFI_FVB2_READ_DISABLED_CAP | \
                               EFI_FVB2_READ_ENABLED_CAP | \
                               EFI_FVB2_WRITE_DISABLED_CAP | \
                               EFI_FVB2_WRITE_ENABLED_CAP | \
                               EFI_FVB2_LOCK_CAP)

#define EFI_FVB2_STATUS       (EFI_FVB2_READ_STATUS | \
                               EFI_FVB2_WRITE_STATUS | \
                               EFI_FVB2_LOCK_STATUS)

typedef struct {
  VENDOR_DEVICE_PATH                  Vendor;
  EFI_DEVICE_PATH_PROTOCOL            End;
} FVB_DEVICE_PATH;

typedef struct {
  SPI_DEVICE                          SpiDevice;

  SPI_FLASH_PROTOCOL                  *SpiFlashProtocol;
  SPI_MASTER_PROTOCOL                 *SpiMasterProtocol;

  EFI_HANDLE                          Handle;

  UINT32                              Signature;

  BOOLEAN                             IsMemoryMapped;
  UINTN                               RegionBaseAddress;
  UINTN                               Size;
  UINTN                               FvbOffset;
  UINTN                               FvbSize;
  EFI_LBA                             StartLba;

  EFI_BLOCK_IO_MEDIA                  Media;
  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol;

  FVB_DEVICE_PATH                     DevicePath;
} FVB_DEVICE;

EFI_STATUS
EFIAPI
FvbGetAttributes(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    OUT       EFI_FVB_ATTRIBUTES_2*                    Attributes
);

EFI_STATUS
EFIAPI
FvbSetAttributes(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    IN OUT    EFI_FVB_ATTRIBUTES_2*                    Attributes
);

EFI_STATUS
EFIAPI
FvbGetPhysicalAddress(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    OUT       EFI_PHYSICAL_ADDRESS*                    Address
);

EFI_STATUS
EFIAPI
FvbGetBlockSize(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    IN        EFI_LBA                                  Lba,
    OUT       UINTN*                                   BlockSize,
    OUT       UINTN*                                   NumberOfBlocks
);

EFI_STATUS
EFIAPI
FvbRead(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    IN        EFI_LBA                                  Lba,
    IN        UINTN                                    Offset,
    IN OUT    UINTN*                                   NumBytes,
    IN OUT    UINT8*                                   Buffer
);

EFI_STATUS
EFIAPI
FvbWrite(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    IN        EFI_LBA                                  Lba,
    IN        UINTN                                    Offset,
    IN OUT    UINTN*                                   NumBytes,
    IN        UINT8*                                   Buffer
);

EFI_STATUS
EFIAPI
FvbEraseBlocks(
    IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL*     This,
    ...
);

#endif //__FVB_DXE_H__
