/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#include <Library/PcdLib.h>

#include "SpiDxe.h"
#include "SpiApbDxe.h"

SPI_MASTER *mSpiMasterInstance;

STATIC
VOID
SpiWriteSpeed (
  IN SPI_DEVICE *Slave,
  IN UINT32 Hz
  )
{
  // Configure baudrate
  SpiApbConfigBaudrateDiv(Slave->RegBase,
               Slave->RefClkHz, Hz);

  // Configure delay timing
  SpiApbDelay(Slave->RegBase, Slave->RefClkHz, Hz,
             Slave->TshslNs, Slave->Tsd2dNs,
             Slave->TchshNs, Slave->TslchNs);
}

STATIC
EFI_STATUS
SpiReadId (
  IN SPI_DEVICE *Slave,
  UINT8 Len,
  UINT8 *IdCode
  )
{
  SPI_MEM_OPS Ops = SPINOR_READID_OP(0, IdCode, Len);
  return SpiApbCommandRead(Slave, &Ops);
}

STATIC
EFI_STATUS
SpiCalibration (
  IN SPI_DEVICE *Slave,
  IN UINT32 Hz
  )
{
  UINTN IdCode = 0, Temp = 0;
  INTN i, RangeLow = -1, RangeHigh = -1;
  EFI_STATUS Status;

  // Start calibration with slowest clock speed at 1 MHz
  SpiWriteSpeed(Slave, 1000000);

  // Set the read data capture delay register to 0
  SpiApbReadDataCapture(Slave->RegBase, 1, 0);

  SpiApbControllerEnable(Slave->RegBase);

  // Get flash ID value as reference
  Status = SpiReadId(Slave, 3, (UINT8 *)&IdCode);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Spi: Calibration failed (read id)\n"));
    return EFI_ABORTED;
  }

  // Use the input speed
  SpiWriteSpeed(Slave, Hz);

  // Find high and low range
  for (i = 0; i < SPI_READ_CAPTURE_MAX_DELAY; i++) {

    SpiApbControllerDisable(Slave->RegBase);

    // Change the read data capture delay register
    SpiApbReadDataCapture(Slave->RegBase, 1, i);

    SpiApbControllerEnable(Slave->RegBase);

    // Read flash ID for comparison later
    Status = SpiReadId(Slave, 3, (UINT8 *)&Temp);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Spi: Calibration failed (read)\n"));
      return EFI_ABORTED;
    }

    // Verify low range
    if (RangeLow == -1 && Temp == IdCode) {
      RangeLow = i;
      continue;
    }
 
    // Verify high range
    if (RangeLow != -1 && Temp != IdCode) {
      RangeHigh = i - 1;
      break;
    }
    RangeHigh = i;
  }

  if (RangeLow == -1) {
    DEBUG ((DEBUG_ERROR, "Spi: Calibration failed (low range)\n"));
    return EFI_ABORTED;
  }

  SpiApbControllerDisable(Slave->RegBase);

  //
  // Set the final value for read data capture delay register based
  // on the calibrated value
  //
  SpiApbReadDataCapture(Slave->RegBase, 1, (RangeHigh + RangeLow) / 2);
  DEBUG ((DEBUG_ERROR, "Spi: Read data capture delay calibrated to %d (%d - %d)\n",
          (RangeHigh + RangeLow) / 2, RangeLow, RangeHigh));

  Slave->CaliHz = Hz;

  SpiApbControllerEnable(Slave->RegBase);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SpiSetSpeed (
  IN SPI_DEVICE *Slave,
  IN UINT32 Hz
  )
{
  EFI_STATUS  Status;

  if (Hz > Slave->MaxHz) {
    Hz = Slave->MaxHz;
  }

  SpiApbControllerDisable(Slave->RegBase);

  //
  // If ReadDelay value provided then dont need
  // to do calibration
  //
  if (Slave->ReadDelay != 0xFFFF) {
    SpiWriteSpeed(Slave, Hz);
    SpiApbReadDataCapture(Slave->RegBase, 1,
              Slave->ReadDelay);
  } else if (Slave->PrevHz != Hz || Slave->CaliHz != Hz) {
    Status = SpiCalibration(Slave, Hz);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    // Prevent re-calibration same speed is requested
    Slave->PrevHz = Hz;
  }

  SpiApbControllerEnable(Slave->RegBase);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SpiSetMode (
  IN SPI_DEVICE *Slave,
  IN UINT8 Mode
  )
{
  UINTN SpiRegBase, RegMode;

  SpiRegBase = Slave->RegBase;

  SpiApbControllerDisable(SpiRegBase);

  // Set SPI Mode
  switch(Mode) {
    case SPI_MODE0:
      RegMode = SPI_MODE_0;
      break;
    case SPI_MODE1:
      RegMode = SPI_MODE_1;
      break;
    case SPI_MODE2:
      RegMode = SPI_MODE_2;
      break;
    case SPI_MODE3:
      RegMode = SPI_MODE_3;
      break;
    default:
      return EFI_INVALID_PARAMETER;
      break;
  }
  SpiApbSetClkMode(SpiRegBase, RegMode);

  // Enable Direct Access Controller
  if (Slave->DacMode) {
    SpiApbDacModeEnable(SpiRegBase);
  }

  SpiApbControllerEnable(SpiRegBase);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SpiSetupTransfer (
  IN SPI_MASTER_PROTOCOL *This,
  IN SPI_DEVICE *Slave
  )
{
  UINT32 MaxHz, SpiPrevFreq, SpiCaliFreq;
  UINTN SpiRegBase;
  EFI_STATUS Status;

  SpiRegBase  = Slave->RegBase;
  MaxHz       = Slave->MaxHz;
  SpiPrevFreq = Slave->PrevHz;
  SpiCaliFreq = Slave->CaliHz;

  Status = SpiSetSpeed(Slave, Slave->MaxHz);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SpiSetMode(Slave, Slave->Mode);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;

}

EFI_STATUS
EFIAPI
SpiTransfer (
  IN SPI_DEVICE *Slave,
  SPI_MEM_OPS *Ops
  )
{
  UINT32 Mode;
  UINTN RegBase = Slave->RegBase;
  EFI_STATUS Status;

  SpiApbChipSelect(RegBase, 0, Slave->IsDecodedCs);

  if (Ops->Data.Dir == SPI_MEM_DATA_IN && Ops->Data.Buf.In) {
    if (!Ops->Addr.NBytes)
      Mode = SPI_CMD_READ;
    else
      Mode = SPI_READ;
  } else {
    if (!Ops->Addr.NBytes || !Ops->Data.Buf.Out)
      Mode = SPI_CMD_WRITE;
    else
      Mode = SPI_WRITE;
  }

  switch (Mode) {
  case SPI_CMD_READ:
    Status = SpiApbCommandReadSetup(Slave, Ops);
    if (!EFI_ERROR (Status)) {
      Status = SpiApbCommandRead(Slave, Ops);
        if (EFI_ERROR (Status)) {
          return Status;
        }
      }
    break;
  case SPI_CMD_WRITE:
    Status = SpiApbCommandWriteSetup(Slave, Ops);
    if (!EFI_ERROR (Status)) {
      Status = SpiApbCommandWrite(Slave, Ops);
      if (EFI_ERROR (Status)) {
        return Status;
      }
    }
    break;
  case SPI_READ:
    Status = SpiApbReadSetup(Slave, Ops);
    if (!EFI_ERROR (Status)) {
        Status = SpiApbReadExecute(Slave, Ops);
        if (EFI_ERROR (Status)) {
          return Status;
        }
      }
    break;
  case SPI_WRITE:
    Status = SpiApbWriteSetup(Slave, Ops);
      if (!EFI_ERROR (Status)) {
        Status = SpiApbWriteExecute(Slave, Ops);
        if (EFI_ERROR (Status)) {
          return Status;
        }
      }
    break;
  }

  return EFI_SUCCESS;
}

SPI_DEVICE *
EFIAPI
SpiSetupSlave (
  IN SPI_MASTER_PROTOCOL *This,
  IN SPI_DEVICE *Slave,
  IN SPI_MODE Mode
  )
{
  EFI_STATUS Status;

  if (!Slave) {
    Slave = AllocateZeroPool (sizeof(SPI_DEVICE));
    if (Slave == NULL) {
      DEBUG((DEBUG_ERROR, "Cannot allocate memory\n"));
      return NULL;
    }
    Slave->Mode = Mode;
  }

  if (!Slave->Info) {
    Slave->Info = AllocateZeroPool (sizeof(NOR_FLASH_INFO));
    if (Slave->Info == NULL) {
      DEBUG((DEBUG_ERROR, "Cannot allocate memory\n"));
      return NULL;
    }
  }

  Slave->RegBase   = PcdGet32(PcdSpiFlashRegBase);
  Slave->AhbBase   = (VOID *)(UINTN)PcdGet64(PcdSpiFlashAhbBase);
  Slave->AhbSize   = PcdGet32(PcdSpiFlashAhbSize);
  Slave->FifoDepth = PcdGet16(PcdSpiFlashFifoDepth);
  Slave->FifoWidth = PcdGet8(PcdSpiFlashFifoWidth);
  Slave->TrigAdd   = PcdGet32(PcdSpiFlashTrigAdd);
  Slave->ReadDelay = PcdGet16(PcdSpiFlashReadDelay);
  Slave->MaxHz     = PcdGet32(PcdSpiFlashMaxHz);
  Slave->RefClkHz  = PcdGet32(PcdSpiFlashRefClkHz);
  Slave->TshslNs   = PcdGet32(PcdSpiFlashTshslNs);
  Slave->Tsd2dNs   = PcdGet32(PcdSpiFlashTsd2dNs);
  Slave->TchshNs   = PcdGet32(PcdSpiFlashTchshNs);
  Slave->TslchNs   = PcdGet32(PcdSpiFlashTslchNs);

  // Enable DAC mode if MMIO window is more than 8M
  if (Slave->AhbSize >= SZ_8M) {
    Slave->DacMode = TRUE;
  }

  Slave->WRDelay = 50 * DIV_ROUND_UP(NSEC_PER_SEC, Slave->RefClkHz);

  Status = SpiSetupTransfer (This, Slave);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "Fail to setup SPI slave\n"));
    FreePool (Slave);
    return NULL;
  }

  return Slave;
}

EFI_STATUS
EFIAPI
SpiFreeSlave (
  IN SPI_DEVICE *Slave
  )
{
  FreePool (Slave);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SpiMasterInitProtocol (
  IN SPI_MASTER_PROTOCOL *SpiMasterProtocol
  )
{
  SpiMasterProtocol->SetupDevice = SpiSetupSlave;
  SpiMasterProtocol->FreeDevice  = SpiFreeSlave;
  SpiMasterProtocol->Transfer    = SpiTransfer;
  SpiMasterProtocol->SetSpeed = SpiSetSpeed;
  SpiMasterProtocol->SetMode = SpiSetMode;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS  Status;

  mSpiMasterInstance = AllocateRuntimeZeroPool (sizeof (SPI_MASTER));
  if (mSpiMasterInstance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  EfiInitializeLock (&mSpiMasterInstance->Lock, TPL_NOTIFY);

  SpiMasterInitProtocol (&mSpiMasterInstance->SpiMasterProtocol);

  mSpiMasterInstance->Signature = SPI_MASTER_SIGNATURE;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &(mSpiMasterInstance->Handle),
                  &gJH7110SpiMasterProtocolGuid,
                  &(mSpiMasterInstance->SpiMasterProtocol),
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    FreePool (mSpiMasterInstance);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}
