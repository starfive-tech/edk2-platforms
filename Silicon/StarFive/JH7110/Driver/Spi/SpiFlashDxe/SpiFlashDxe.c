/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#include "SpiFlashDxe.h"

STATIC EFI_EVENT    mSpiFlashVirtualAddrChangeEvent;
SPI_MASTER_PROTOCOL *SpiMasterProtocol;
SPI_FLASH_INSTANCE  *mSpiFlashInstance;

STATIC
EFI_STATUS
SpiFlashWriteEnableCmd (
  IN  SPI_DEVICE   *Slave
  )
{
  EFI_STATUS Status;

  SPI_MEM_OPS Ops = SPINOR_WR_EN_DIS_OP(TRUE);

  // Send write enable command
  Status = SpiMasterProtocol->Transfer (Slave, &Ops);

  return Status;
}

STATIC
EFI_STATUS
SpiFlashWriteDisableCmd (
  IN  SPI_DEVICE   *Slave
  )
{
  EFI_STATUS Status;

  SPI_MEM_OPS Ops = SPINOR_WR_EN_DIS_OP(FALSE);

  // Send write disable command
  Status = SpiMasterProtocol->Transfer (Slave, &Ops);

  return Status;
}

STATIC
EFI_STATUS
SpiFlashWriteCommon (
  IN SPI_DEVICE *Slave,
  IN UINT32 WriteAddr,
  IN UINT32 Length,
  IN UINT8* Buffer,
  IN UINT32 BufferLength
  )
{
  EFI_STATUS Status;
  UINT32 Counter = 0xFFFFF;
  UINT8 PollBitMsk = STATUS_REG_POLL_WIP_MSK;
  UINT8 CheckStatus = 0x0;

  Status = SpiFlashWriteEnableCmd(Slave);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
    return Status;
  }

  SPI_MEM_OPS OpsPgProg = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_PAGE_PROGRAM, 0),
                      SPI_MEM_OP_ADDR(3, WriteAddr, 0),
                      SPI_MEM_OP_NO_DUMMY,
                      SPI_MEM_OP_DATA_OUT(BufferLength, (VOID*)Buffer, 0));

  // Page program command - write to flash
  Status = SpiMasterProtocol->Transfer(Slave, &OpsPgProg);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Poll status register
  UINT8 Len = 2;
  UINT8 List[2] = {0};
  UINT8 *State = List;
  SPI_MEM_OPS OpsRdSts = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_READ_STATUS, 1),
                      SPI_MEM_OP_NO_ADDR,
                      SPI_MEM_OP_NO_DUMMY,
                      SPI_MEM_OP_DATA_IN(Len, (VOID *)State, 1));

  Status = SpiMasterProtocol->Transfer(Slave, &OpsRdSts);
  if (EFI_ERROR (Status)) {
    return Status;
  }



  do {
    Status = SpiMasterProtocol->Transfer(Slave, &OpsRdSts);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Spi terror while reading status\n"));
      return Status;
    }
    Counter--;
    if ((*State & PollBitMsk) == CheckStatus) {
      break;
    }
  } while (Counter > 0);
  if (Counter == 0) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Timeout while writing to spi flash\n"));
    return EFI_TIMEOUT;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashErase (
  IN SPI_DEVICE *Slave,
  IN UINTN Offset,
  IN UINTN Length
  )
{
  EFI_STATUS Status;
  UINT32 EraseAddr;
  UINTN EraseSize;
  UINT8 EraseCmd;
  UINT32 Counter = 0xFFFFF;
  UINT8 PollBitMsk = STATUS_REG_POLL_WIP_MSK;
  UINT8 CheckStatus = 0x0;


  // Get erase flash size
  if (Slave->Info->Flags & NOR_FLASH_ERASE_4K) {
    EraseCmd = CMD_ERASE_4K;
    EraseSize = SIZE_4KB;
  } else if (Slave->Info->Flags & NOR_FLASH_ERASE_32K) {
    EraseCmd = CMD_ERASE_32K;
    EraseSize = SIZE_32KB;
  } else {
    EraseCmd = CMD_ERASE_64K;
    EraseSize = Slave->Info->SectorSize;
  }

  // Verify input parameters
  if (Offset % EraseSize || Length % EraseSize) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Either erase offset or length "
      "is not multiple of erase size\n"));
    return EFI_DEVICE_ERROR;
  }

  while (Length) {
    EraseAddr = Offset;

    Status = SpiFlashWriteEnableCmd(Slave);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
      return Status;
    }
  
    // Send erase command
    SPI_MEM_OPS OpsErase = SPI_MEM_OP(SPI_MEM_OP_CMD(EraseCmd, 1),
                        SPI_MEM_OP_ADDR(3, EraseAddr, 1),
                        SPI_MEM_OP_NO_DUMMY,
                        SPI_MEM_OP_NO_DATA);

    Status = SpiMasterProtocol->Transfer(Slave, &OpsErase);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Spi erase fail\n"));
      return Status;
    }

    // Polling for status register
    UINT8 Len = 2;
    UINT8 List[2] = {0};
    UINT8 *State = List;
    SPI_MEM_OPS OpsRdSts = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_READ_STATUS, 1),
                            SPI_MEM_OP_NO_ADDR,
                            SPI_MEM_OP_NO_DUMMY,
                            SPI_MEM_OP_DATA_IN(Len, (VOID *)State, 1));

    do {
      Status = SpiMasterProtocol->Transfer(Slave, &OpsRdSts);
      if (EFI_ERROR (Status)) {
        DEBUG((DEBUG_ERROR, "SpiFlash: Spi error while reading status\n"));
        return Status;
      }
      Counter--;
      if ((*State & PollBitMsk) == CheckStatus) {
        break;
      }
    } while (Counter > 0);

    if (Counter == 0) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Timeout while writing to spi flash\n"));
      return EFI_TIMEOUT;
    }

    Offset += EraseSize;
    Length -= EraseSize;
  }

  Status = SpiFlashWriteDisableCmd(Slave);
  if (EFI_ERROR (Status)) {
   DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_disable\n"));
   return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashRead (
  IN SPI_DEVICE *Slave,
  IN UINT32     Offset,
  IN UINTN      Length,
  IN VOID       *Buf
  )
{
  EFI_STATUS Status;
  UINT32 ReadAddr, ReadLength, RemainLength;
  UINTN BankSel = 0;

  while (Length) {
    ReadAddr = Offset;

    RemainLength = (SPI_FLASH_16MB_BOUND * (BankSel + 1)) - Offset;
    ReadLength = (Length < RemainLength) ? Length : RemainLength;

    // Send read command
    SPI_MEM_OPS OpsRead = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_READ_DATA, 1),
                            SPI_MEM_OP_ADDR(3, Offset, 1),
                            SPI_MEM_OP_NO_DUMMY,
                            SPI_MEM_OP_DATA_IN(ReadLength, Buf, 1));

    Status = SpiMasterProtocol->Transfer(Slave, &OpsRead);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Spi error while reading data\n"));
      return Status;
    }

    Offset += ReadLength;
    Length -= ReadLength;
    Buf = (VOID*)((UINTN)Buf + ReadLength);
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashWrite (
  IN SPI_DEVICE *Slave,
  IN UINT32     Offset,
  IN UINTN      Length,
  IN VOID       *Buf
  )
{
  EFI_STATUS Status;
  UINTN ByteAddr, ChunkLength, ActualIndex, PageSize;
  UINT32 WriteAddr;

  PageSize = Slave->Info->PageSize;

  for (ActualIndex = 0; ActualIndex < Length; ActualIndex += ChunkLength) {
    WriteAddr = Offset;

    ByteAddr = Offset % PageSize;
    ChunkLength = MIN(Length - ActualIndex, (UINT64) (PageSize - ByteAddr));

    Status = SpiFlashWriteCommon (
      Slave, WriteAddr, Slave->AddrSize + 1,
      (VOID*)((UINTN)Buf + ActualIndex), ChunkLength
      );

    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while programming write address\n"));
      return Status;
    }

    Offset += ChunkLength;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiFlashReadId (
  IN     SPI_DEVICE *SpiDev,
  IN     BOOLEAN     UseInRuntime
  )
{
  EFI_STATUS Status;
  UINT8 Id[NOR_FLASH_MAX_ID_LEN];

  SPI_MEM_OPS Ops = SPINOR_READID_OP(0, Id, 3);
  Status = SpiMasterProtocol->Transfer(SpiDev, &Ops);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "ReadId: Spi error while reading id\n"));
    return Status;
  }

  Status = NorFlashGetInfo (Id, &SpiDev->Info, UseInRuntime);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Unrecognized JEDEC Id bytes: 0x%02x%02x%02x\n",
      __FUNCTION__,
      Id[0],
      Id[1],
      Id[2]));
    return Status;
  }

  NorFlashPrintInfo (SpiDev->Info);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiFlashInit (
  IN SPI_FLASH_PROTOCOL *This,
  IN SPI_DEVICE *Slave
  )
{
  EFI_STATUS Status;
  UINT8 StatusRegister;

  Slave->AddrSize = (Slave->Info->Flags & NOR_FLASH_4B_ADDR) ? 4 : 3;

  if (Slave->AddrSize == 4) {
    Status = SpiFlashWriteEnableCmd (Slave);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
      return Status;
    }

    // Enable 4byte addressing
    SPI_MEM_OPS Ops4BAddEn = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_4B_ADDR_ENABLE, 1),
            SPI_MEM_OP_NO_ADDR,
            SPI_MEM_OP_NO_DUMMY,
            SPI_MEM_OP_NO_DATA);
    Status = SpiMasterProtocol->Transfer (Slave, &Ops4BAddEn);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting 4B address\n"));
      return Status;
    }
  }

  Status = SpiFlashWriteEnableCmd (Slave);
  if (EFI_ERROR (Status)) {
  DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
    return Status;
  }

  // Initialize flash status register
  StatusRegister = 0x0;
  SPI_MEM_OPS OpsWrSts = SPI_MEM_OP(SPI_MEM_OP_CMD(CMD_WRITE_STATUS_REG, 1),
          SPI_MEM_OP_NO_ADDR,
          SPI_MEM_OP_NO_DUMMY,
          SPI_MEM_OP_DATA_OUT(1, (VOID *)&StatusRegister, 1));
  Status = SpiMasterProtocol->Transfer (Slave, &OpsWrSts);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Error while writing status register\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashInitProtocol (
  IN SPI_FLASH_PROTOCOL *SpiFlashProtocol
  )
{
  SpiFlashProtocol->Init   = SpiFlashInit;
  SpiFlashProtocol->ReadId = SpiFlashReadId;
  SpiFlashProtocol->Read   = SpiFlashRead;
  SpiFlashProtocol->Write  = SpiFlashWrite;
  SpiFlashProtocol->Erase  = SpiFlashErase;

  return EFI_SUCCESS;
}

STATIC
VOID
EFIAPI
SpiFlashVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  EfiConvertPointer (0x0, (VOID**)&SpiMasterProtocol->Transfer);
  EfiConvertPointer (0x0, (VOID**)&SpiMasterProtocol);
}

EFI_STATUS
EFIAPI
SpiFlashEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS  Status;

  Status = gBS->LocateProtocol (
    &gJH7110SpiMasterProtocolGuid,
    NULL,
    (VOID **)&SpiMasterProtocol
  );
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot locate SPI Master protocol\n"));
    return EFI_DEVICE_ERROR;
  }

  mSpiFlashInstance = AllocateRuntimeZeroPool (sizeof (SPI_FLASH_INSTANCE));
  if (mSpiFlashInstance == NULL) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot allocate memory\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  SpiFlashInitProtocol (&mSpiFlashInstance->SpiFlashProtocol);

  mSpiFlashInstance->Signature = SPI_FLASH_SIGNATURE;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &(mSpiFlashInstance->Handle),
                  &gJH7110SpiFlashProtocolGuid,
                  &(mSpiFlashInstance->SpiFlashProtocol),
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot install SPI flash protocol\n"));
    goto ErrorInstallProto;
  }

  // Register for the virtual address change event
  Status = gBS->CreateEventEx (EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  SpiFlashVirtualNotifyEvent,
                  NULL,
                  &gEfiEventVirtualAddressChangeGuid,
                  &mSpiFlashVirtualAddrChangeEvent);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Failed to register VA change event\n", __FUNCTION__));
    goto ErrorCreateEvent;
  }

  return EFI_SUCCESS;

ErrorCreateEvent:
  gBS->UninstallMultipleProtocolInterfaces (&mSpiFlashInstance->Handle,
    &gJH7110SpiFlashProtocolGuid,
    NULL);

ErrorInstallProto:
  FreePool (mSpiFlashInstance);

  return EFI_SUCCESS;
}
