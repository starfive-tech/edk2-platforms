/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#include "SpiApbDxe.h"

VOID
SpiApbControllerEnable (
  IN UINT32 RegBase
  )
{
  UINT32 Reg;
  
  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);
  Reg |= SPI_REG_CONFIG_ENABLE;
  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);
}

VOID 
SpiApbControllerDisable (
  IN UINT32 RegBase
  )
{
  UINT32 Reg;

  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);
  Reg &= ~SPI_REG_CONFIG_ENABLE;
  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);
}

VOID 
SpiApbDacModeEnable (
  IN UINT32 RegBase
  )
{
  UINT32 Reg;

  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);
  Reg |= SPI_REG_CONFIG_DIRECT;
  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);
}

STATIC
UINT32 
SpiApbCalcDummy (
  IN CONST SPI_MEM_OPS *Ops,
  IN BOOLEAN Dtr
  )
{
  UINT32 DummyClk;

  if (!Ops->Dummy.NBytes || !Ops->Dummy.BusWidth) {
    return 0;
  }

  DummyClk = Ops->Dummy.NBytes * (8 / Ops->Dummy.BusWidth);
  if (Dtr) {
    DummyClk /= 2;
  }

  return DummyClk;
}

STATIC
UINT32 
SpiApbCalcReadReg (
  IN SPI_DEVICE *Slave
  )
{
  UINT32 ReadReg = 0;

  ReadReg |= Slave->InstWidth << SPI_REG_RD_INSTR_TYPE_INSTR_LSB;
  ReadReg |= Slave->AddrWidth << SPI_REG_RD_INSTR_TYPE_ADDR_LSB;
  ReadReg |= Slave->DataWidth << SPI_REG_RD_INSTR_TYPE_DATA_LSB;

  return ReadReg;
}

STATIC
INTN 
SpiApbBuswidthToInstType (
  IN UINT8 BusWidth
  )
{
  switch (BusWidth) {
  case 0:
  case 1:
    return SPI_INST_TYPE_SINGLE;
  case 2:
    return SPI_INST_TYPE_DUAL;
  case 4:
    return SPI_INST_TYPE_QUAD;
  case 8:
    return SPI_INST_TYPE_OCTAL;
  default:
    return -1;
  }
}

STATIC
EFI_STATUS
SpiApbSetProtocol (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  INTN Ret;

  Slave->Dtr = Ops->Data.Dtr && Ops->Cmd.Dtr && Ops->Addr.Dtr;

  Ret = SpiApbBuswidthToInstType(Ops->Cmd.BusWidth);
  if (Ret < 0) {
    return EFI_UNSUPPORTED;
  }
  Slave->InstWidth = Ret;

  Ret = SpiApbBuswidthToInstType(Ops->Addr.BusWidth);
  if (Ret < 0) {
    return EFI_UNSUPPORTED;
  }
  Slave->AddrWidth = Ret;

  Ret = SpiApbBuswidthToInstType(Ops->Data.BusWidth);
  if (Ret < 0) {
    return EFI_UNSUPPORTED;
  }
  Slave->DataWidth = Ret;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS 
SpiApbWaitIdle (
  IN UINT32 RegBase
  )
{
  UINT32 Count = 0;
  UINT32 TimeoutMs = 5000000;

  do {
      Count = (SPI_REG_IS_IDLE(RegBase)) ? Count+1 : 0;
    /*
     * Make sure the QSPI controller is in really idle
     * for n period of time before proceed
     */
    if (Count >= SPI_POLL_IDLE_RETRY) {
      return EFI_SUCCESS;
    }
    gBS->Stall(1);
  } while (TimeoutMs);

  return EFI_TIMEOUT;
}

VOID 
SpiApbReadDataCapture (
  IN UINT32 RegBase,
  IN UINT32 ByPass, 
  IN UINT32 Delay
  )
{
  UINT32 Reg;
  SpiApbControllerDisable(RegBase);

  Reg = MmioRead32(RegBase + SPI_REG_RD_DATA_CAPTURE);

  if (ByPass) {
    Reg |= SPI_REG_RD_DATA_CAPTURE_BYPASS;
  } else {
    Reg &= ~SPI_REG_RD_DATA_CAPTURE_BYPASS;
  }

  Reg &= ~(SPI_REG_RD_DATA_CAPTURE_DELAY_MASK
    << SPI_REG_RD_DATA_CAPTURE_DELAY_LSB);

  Reg |= (Delay & SPI_REG_RD_DATA_CAPTURE_DELAY_MASK)
    << SPI_REG_RD_DATA_CAPTURE_DELAY_LSB;

  MmioWrite32(RegBase + SPI_REG_RD_DATA_CAPTURE, Reg);

  SpiApbControllerEnable(RegBase);
}

VOID 
SpiApbConfigBaudrateDiv (
  IN UINT32 RegBase,
  IN UINT32 RefClkHz, 
  IN UINT32 SclkHz
  )
{
  UINT32 Reg;
  UINT32 Div;

  SpiApbControllerDisable(RegBase);
  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);
  Reg &= ~(SPI_REG_CONFIG_BAUD_MASK << SPI_REG_CONFIG_BAUD_LSB);

  Div = DIV_ROUND_UP(RefClkHz, SclkHz * 2) - 1;

  if (Div > SPI_REG_CONFIG_BAUD_MASK) {
    Div = SPI_REG_CONFIG_BAUD_MASK;
  }

  DEBUG ((DEBUG_ERROR, "%s: RefClk %dHz sclk %dHz Div 0x%x, actual %dHz\n", __func__,
         RefClkHz, SclkHz, Div, RefClkHz / (2 * (Div + 1))));

  Reg |= (Div << SPI_REG_CONFIG_BAUD_LSB);
  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);

  SpiApbControllerEnable(RegBase);
}

VOID 
SpiApbSetClkMode (
  IN UINT32 RegBase, 
  IN UINTN Mode
)
{
  UINT32 Reg;

  SpiApbControllerDisable(RegBase);
  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);
  Reg &= ~(SPI_REG_CONFIG_CLK_POL | SPI_REG_CONFIG_CLK_PHA);

  if (Mode & SPI_CPOL) {
    Reg |= SPI_REG_CONFIG_CLK_POL;
  }
  if (Mode & SPI_CPHA) {
    Reg |= SPI_REG_CONFIG_CLK_PHA;
  }

  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);

  SpiApbControllerEnable(RegBase);
}

VOID 
SpiApbChipSelect (
  IN UINT32 RegBase,
  IN UINT32 ChipSel, 
  IN UINT32 DecoderEnable
  )
{
  UINT32 Reg;

  SpiApbControllerDisable(RegBase);

  Reg = MmioRead32(RegBase + SPI_REG_CONFIG);

  if (DecoderEnable) {
    Reg |= SPI_REG_CONFIG_DECODE;
  } else {
    Reg &= ~SPI_REG_CONFIG_DECODE;
    /* Convert CS if without decoder.
     * CS0 to 4b'1110
     * CS1 to 4b'1101
     * CS2 to 4b'1011
     * CS3 to 4b'0111
     */
    ChipSel = 0xF & ~(1 << ChipSel);
  }

  Reg &= ~(SPI_REG_CONFIG_CHIPSELECT_MASK
      << SPI_REG_CONFIG_CHIPSELECT_LSB);
  Reg |= (ChipSel & SPI_REG_CONFIG_CHIPSELECT_MASK)
      << SPI_REG_CONFIG_CHIPSELECT_LSB;
  MmioWrite32(RegBase + SPI_REG_CONFIG, Reg);

  SpiApbControllerEnable(RegBase);
}

VOID 
SpiApbDelay (
  IN UINT32 RegBase,
  IN UINT32 RefClk,
  IN UINT32 SclkHz,
  IN UINT32 TshslNs, 
  IN UINT32 Tsd2dNs,
  IN UINT32 TchshNs, 
  IN UINT32 TslchNs
  )
{
  UINT32 RefClkNs;
  UINT32 SclkNs;
  UINT32 Tshsl, Tchsh, Tslch, Tsd2d;
  UINT32 Reg;

  SpiApbControllerDisable(RegBase);

  // Convert to ns.
  RefClkNs = DIV_ROUND_UP(1000000000, RefClk);

  // Convert to ns.
  SclkNs = DIV_ROUND_UP(1000000000, SclkHz);

  // The controller adds additional delay to that programmed in the Reg
  if (TshslNs >= SclkNs + RefClkNs) {
    TshslNs -= SclkNs + RefClkNs;
  }
  if (TchshNs >= SclkNs + 3 * RefClkNs) {
    TchshNs -= SclkNs + 3 * RefClkNs;
  }
  Tshsl = DIV_ROUND_UP(TshslNs, RefClkNs);
  Tchsh = DIV_ROUND_UP(TchshNs, RefClkNs);
  Tslch = DIV_ROUND_UP(TslchNs, RefClkNs);
  Tsd2d = DIV_ROUND_UP(Tsd2dNs, RefClkNs);

  Reg = ((Tshsl & SPI_REG_DELAY_TSHSL_MASK)
      << SPI_REG_DELAY_TSHSL_LSB);
  Reg |= ((Tchsh & SPI_REG_DELAY_TCHSH_MASK)
      << SPI_REG_DELAY_TCHSH_LSB);
  Reg |= ((Tslch & SPI_REG_DELAY_TSLCH_MASK)
      << SPI_REG_DELAY_TSLCH_LSB);
  Reg |= ((Tsd2d & SPI_REG_DELAY_TSD2D_MASK)
      << SPI_REG_DELAY_TSD2D_LSB);
  MmioWrite32(RegBase + SPI_REG_DELAY, Reg);

  SpiApbControllerEnable(RegBase);
}

STATIC
EFI_STATUS 
SpiApbExecFlashCmd (
  IN UINT32 RegBase,
  IN UINT32 Reg
  )
{
  EFI_STATUS Status;
  UINT32 Retry = SPI_REG_RETRY;

  // Write the CMDCTRL without start execution.
  MmioWrite32(RegBase + SPI_REG_CMDCTRL, Reg);
  // Start execute
  Reg |= SPI_REG_CMDCTRL_EXECUTE;
  MmioWrite32(RegBase + SPI_REG_CMDCTRL, Reg);

  while (Retry--) {
    Reg = MmioRead32(RegBase + SPI_REG_CMDCTRL);
    if ((Reg & SPI_REG_CMDCTRL_INPROGRESS) == 0) {
      break;
    }
    gBS->Stall(1);
  }

  if (!Retry) {
    DEBUG ((DEBUG_ERROR, "QSPI: flash command execution Timeout\n"));
    return EFI_TIMEOUT;
  }

  // Polling QSPI idle status.
  Status = SpiApbWaitIdle(RegBase);
  if (EFI_ERROR (Status)) {
    return EFI_TIMEOUT;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS 
SpiApbSetupOpCodeExt (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops,
  IN UINT32 Shift
  )
{
  UINT32 Reg;
  UINT8 Ext;

  if (Ops->Cmd.NBytes != 2) {
    return EFI_UNSUPPORTED;
  }

  // Opcode extension is the LSB.
  Ext = Ops->Cmd.OpCode & 0xFF;

  Reg = MmioRead32(Slave->RegBase + SPI_REG_OP_EXT_LOWER);
  Reg &= ~(0xFF << Shift);
  Reg |= Ext << Shift;
  MmioWrite32(Slave->RegBase + SPI_REG_OP_EXT_LOWER, Reg);

  return EFI_SUCCESS;
}

STATIC 
EFI_STATUS
SpiApbEnableDtr (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops,
  IN UINT32 Shift,
  IN BOOLEAN Enable
  )
{
  UINT32 Reg;
  EFI_STATUS Status;

  Reg = MmioRead32(Slave->RegBase + SPI_REG_CONFIG);

  if (Enable) {
    Reg |= SPI_REG_CONFIG_DTR_PROTO;
    Reg |= SPI_REG_CONFIG_DUAL_OPCODE;

    // Set up command OpCode extension.
    Status = SpiApbSetupOpCodeExt(Slave, Ops, Shift);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  } else {
    Reg &= ~SPI_REG_CONFIG_DTR_PROTO;
    Reg &= ~SPI_REG_CONFIG_DUAL_OPCODE;
  }

  MmioWrite32(Slave->RegBase + SPI_REG_CONFIG, Reg);

  return EFI_SUCCESS;
}

EFI_STATUS 
SpiApbCommandReadSetup (
  SPI_DEVICE *Slave,
  CONST SPI_MEM_OPS *Ops
  )
{
  EFI_STATUS Status;
  UINT32 Reg;

  Status = SpiApbSetProtocol(Slave, Ops);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SpiApbEnableDtr(Slave, Ops, SPI_REG_OP_EXT_STIG_LSB,
              Slave->Dtr);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Reg = SpiApbCalcReadReg(Slave);
  MmioWrite32(Slave->RegBase + SPI_REG_RD_INSTR, Reg);

  return EFI_SUCCESS;
}

// For command RDID, RDSR.
EFI_STATUS 
SpiApbCommandRead (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT32 RegBase = Slave->RegBase;
  UINT32 Reg;
  UINT32 ReadLen;
  EFI_STATUS Status;
  UINT32 RxLen = Ops->Data.NBytes;
  VOID *RxBuf = Ops->Data.Buf.In;
  UINT32 DummyClk;
  UINT8 OpCode;

  if (RxLen > SPI_STIG_DATA_LEN_MAX || !RxBuf) {
    DEBUG ((DEBUG_ERROR, "QSPI: Invalid input arguments RxLen %d\n", RxLen));
    return EFI_INVALID_PARAMETER;
  }

  OpCode = (Slave->Dtr) ? (Ops->Cmd.OpCode >> 8) : Ops->Cmd.OpCode; 

  Reg = OpCode << SPI_REG_CMDCTRL_OPCODE_LSB;

  // Set up Dummy cycles.
  DummyClk = SpiApbCalcDummy(Ops, Slave->Dtr);
  if (DummyClk > SPI_DUMMY_CLKS_MAX) {
    return EFI_INVALID_PARAMETER;
  }

  if (DummyClk) {
    Reg |= (DummyClk & SPI_REG_CMDCTRL_DUMMY_MASK)
         << SPI_REG_CMDCTRL_DUMMY_LSB;
  }

  Reg |= (0x1 << SPI_REG_CMDCTRL_RD_EN_LSB);

  // 0 means 1 byte.
  Reg |= (((RxLen - 1) & SPI_REG_CMDCTRL_RD_BYTES_MASK)
    << SPI_REG_CMDCTRL_RD_BYTES_LSB);
  Status = SpiApbExecFlashCmd(RegBase, Reg);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Reg = MmioRead32(RegBase + SPI_REG_CMDREADDATALOWER);

  // Put the read value into rx_buf
  ReadLen = (RxLen > 4) ? 4 : RxLen;
  CopyMem(RxBuf, &Reg, ReadLen);
  RxBuf += ReadLen;

  if (RxLen > 4) {
    Reg = MmioRead32(RegBase + SPI_REG_CMDREADDATAUPPER);

    ReadLen = RxLen - ReadLen;
    CopyMem(RxBuf, &Reg, ReadLen);
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiApbCommandWriteSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  EFI_STATUS Status;
  UINT32 Reg;

  Status = SpiApbSetProtocol(Slave, Ops);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SpiApbEnableDtr(Slave, Ops, SPI_REG_OP_EXT_STIG_LSB,
              Slave->Dtr);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Reg = SpiApbCalcReadReg(Slave);
  MmioWrite32(Slave->RegBase + SPI_REG_RD_INSTR, Reg);

  return EFI_SUCCESS;
}

// For commands: WRSR, WREN, WRDI, CHIP_ERASE, BE, etc.
EFI_STATUS
SpiApbCommandWrite (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT32 Reg = 0;
  UINT32 WriteData;
  UINT32 WriteLen;
  UINT32 TxLen = Ops->Data.NBytes;
  CONST VOID *TxBuf = Ops->Data.Buf.Out;
  UINT32 RegBase = Slave->RegBase;
  UINT32 Addr;
  UINT8 OpCode;
  EFI_STATUS Status;

  // Reorder address to SPI bus order if only transferring address
  if (!TxLen) {
    Addr = cpu_to_be32(Ops->Addr.Val);
    if (Ops->Addr.NBytes == 3) {
      Addr >>= 8;
    }
    TxBuf = &Addr;
    TxLen = Ops->Addr.NBytes;
  }

  if (TxLen > SPI_STIG_DATA_LEN_MAX) {
    DEBUG ((DEBUG_ERROR, "QSPI: Invalid input arguments TxLen %d\n", TxLen));
    return EFI_INVALID_PARAMETER;
  }

  if (Slave->Dtr) {
    OpCode = Ops->Cmd.OpCode >> 8;
  } else {
    OpCode = Ops->Cmd.OpCode;
  }

  Reg |= OpCode << SPI_REG_CMDCTRL_OPCODE_LSB;

  if (TxLen) {
    // writing Data = yes
    Reg |= (0x1 << SPI_REG_CMDCTRL_WR_EN_LSB);
    Reg |= ((TxLen - 1) & SPI_REG_CMDCTRL_WR_BYTES_MASK)
      << SPI_REG_CMDCTRL_WR_BYTES_LSB;

    WriteLen = TxLen > 4 ? 4 : TxLen;
    CopyMem(&WriteData, TxBuf, WriteLen);
    MmioWrite32(RegBase + SPI_REG_CMDWRITEDATALOWER, WriteData);

    if (TxLen > 4) {
      TxBuf += WriteLen;
      WriteLen = TxLen - WriteLen;
      CopyMem(&WriteData, TxBuf, WriteLen);
      MmioWrite32(RegBase + SPI_REG_CMDWRITEDATAUPPER, WriteData);
    }
  }

  // Execute the command
  Status = SpiApbExecFlashCmd(RegBase, Reg);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
UINT32
SpiApbGetReadSramLevel (
  IN SPI_DEVICE *Slave
  )
{
  UINT32 Reg = MmioRead32(Slave->RegBase + SPI_REG_SDRAMLEVEL);
  Reg >>= SPI_REG_SDRAMLEVEL_RD_LSB;
  return Reg & SPI_REG_SDRAMLEVEL_RD_MASK;
}

// Opcode + Address (3/4 bytes) + Dummy bytes (0-4 bytes)
EFI_STATUS
SpiApbReadSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT32 Reg;
  UINT32 ReadReg;
  UINT32 DummyClk;
  UINT32 DummyBytes = Ops->Dummy.NBytes;
  EFI_STATUS Status;
  UINT8 OpCode;

  Status = SpiApbSetProtocol(Slave, Ops);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SpiApbEnableDtr(Slave, Ops, SPI_REG_OP_EXT_READ_LSB,
              Slave->Dtr);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Setup the indirect trigger address
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTTRIGGER, Slave->TrigAdd);

  // Configure the OpCode
  if (Slave->Dtr) {
    OpCode = Ops->Cmd.OpCode >> 8;
  } else {
    OpCode = Ops->Cmd.OpCode;
  }

  ReadReg = OpCode << SPI_REG_RD_INSTR_OPCODE_LSB;
  ReadReg |= SpiApbCalcReadReg(Slave);

  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTRDSTARTADDR, Ops->Addr.Val);

  if (DummyBytes) {
    // Convert to clock cycles.
    DummyClk = SpiApbCalcDummy(Ops, Slave->Dtr);

    if (DummyClk > SPI_DUMMY_CLKS_MAX) {
      return EFI_INVALID_PARAMETER;
    }

    if (DummyClk) {
      ReadReg |= (DummyClk & SPI_REG_RD_INSTR_DUMMY_MASK)
        << SPI_REG_RD_INSTR_DUMMY_LSB;
    }
  }


  MmioWrite32(Slave->RegBase + SPI_REG_RD_INSTR, ReadReg);

  // Set device size
  Reg = MmioRead32(Slave->RegBase + SPI_REG_SIZE);
  Reg &= ~SPI_REG_SIZE_ADDRESS_MASK;
  Reg |= (Ops->Addr.NBytes - 1);
  MmioWrite32(Slave->RegBase + SPI_REG_SIZE, Reg);

  return EFI_SUCCESS;
}

STATIC 
INTN 
SpiApbWaitForData (
  SPI_DEVICE *Slave
  )
{
  UINT32 Timeout = 10000;
  UINT32 Reg;

  while (Timeout--) {
    Reg = SpiApbGetReadSramLevel(Slave);
    if (Reg) {
      return Reg;
  }
    gBS->Stall(1);
  }

  return -1;
}

STATIC
VOID
SpiApbReadByte (
  IN VOID *Addr, 
  IN VOID *Data, 
  IN INTN ByteLen
  )
{
  UINT8 *ptr;
  UINT8 *ptr2;

  ptr = (UINT8 *)Addr;
  ptr2 = (UINT8 *)Data;

  while (ByteLen) {
    *ptr2 = *ptr;
    ptr2++;
    ByteLen--;
  }
}

STATIC
VOID 
SpiApbReadLong (
  VOID *Addr, 
  VOID *Data,
  INTN LongLen
  )
{
  UINT32 *ptr;
  UINT32 *ptr2;

  ptr = (UINT32 *)Addr;
  ptr2 = (UINT32 *)Data;

  while (LongLen) {
    *ptr2 = *ptr;
    ptr2++;
    LongLen--;
  }
}

STATIC
EFI_STATUS
SpiApbWaitForBitLe32 (
  IN INT32 Reg,
  IN CONST UINT32 Mask,
  IN CONST BOOLEAN Set,
  IN CONST UINT32 TimeoutMs
  )
{
  UINT32 Val;
  UINTN Start = TimeoutMs*1000;

  while (1) {
    Val = MmioRead32(Reg);

    if (!Set) {
      Val = ~Val;
    }

    if ((Val & Mask) == Mask) {
      return EFI_SUCCESS;
    }

    if (Start == 0) {
      break;
    } else {
      Start--;
    }

    gBS->Stall(1);
  }

  DEBUG ((DEBUG_ERROR, "Timeout (Reg=%lx Mask=%x wait_set=%d)\n", Reg, Mask, Set));

  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
SpiApbIndirectReadExecute (
  IN SPI_DEVICE *Slave,
  IN UINTN NRx, 
  IN UINT8 *RxBuf
  )
{
  UINTN Remaining = NRx;
  UINTN bytes_to_read = 0;
  EFI_STATUS Status;
  INTN ret;

  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTRDBYTES, NRx);

  // Start the indirect read transfer
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTRD, SPI_REG_INDIRECTRD_START);

  while (Remaining > 0) {
    ret = SpiApbWaitForData(Slave);
    if (ret < 0) {
      DEBUG ((DEBUG_ERROR, "Indirect write timed out (%d)\n", ret));
      goto FailRead;
    }

    bytes_to_read = ret;
    while (bytes_to_read != 0) {
      bytes_to_read *= Slave->FifoWidth;
      bytes_to_read = bytes_to_read > Remaining ?
          Remaining : bytes_to_read;

      if (((UINTN)RxBuf % 4) || (bytes_to_read % 4)) {
        SpiApbReadByte(Slave->AhbBase, RxBuf, bytes_to_read);
      } else {
        SpiApbReadLong(Slave->AhbBase, RxBuf, bytes_to_read >> 2);
    }
    RxBuf += bytes_to_read;
      Remaining -= bytes_to_read;
      bytes_to_read = SpiApbGetReadSramLevel(Slave);
    }
  }

  // Check indirect done status
  Status = SpiApbWaitForBitLe32(Slave->RegBase + SPI_REG_INDIRECTRD,
        SPI_REG_INDIRECTRD_DONE, 1, 10);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Indirect read completion error (%d)\n", ret));
    goto FailRead;
  }

  // Clear indirect completion status
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTRD, SPI_REG_INDIRECTRD_DONE);

  return EFI_SUCCESS;

FailRead:
  // Cancel the indirect read
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTRD, SPI_REG_INDIRECTRD_CANCEL);
  return EFI_ABORTED;
}

EFI_STATUS
SpiApbReadExecute (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT64 from = Ops->Addr.Val;
  VOID *Buf = Ops->Data.Buf.In;
  UINTN len = Ops->Data.NBytes;
  EFI_STATUS Status;

  if (Slave->DacMode && (from + len < Slave->AhbSize)) {
    if (len < 256) { 
      CopyMem(Buf, (&Slave->AhbSize + from), len);
    }

  Status = SpiApbWaitIdle(Slave->RegBase);
    if (EFI_ERROR (Status)) {
      return EFI_TIMEOUT;
    }
    return EFI_SUCCESS;
  }

  Status = SpiApbIndirectReadExecute(Slave, len, Buf);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

// Opcode + Address (3/4 bytes)
EFI_STATUS
SpiApbWriteSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT32 Reg;
  EFI_STATUS Status;
  UINT8 OpCode;

  Status = SpiApbSetProtocol(Slave, Ops);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SpiApbEnableDtr(Slave, Ops, SPI_REG_OP_EXT_WRITE_LSB,
              Slave->Dtr);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Setup the indirect trigger address
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTTRIGGER, Slave->TrigAdd);

  // Configure the OpCode
  OpCode = (Slave->Dtr) ? (Ops->Cmd.OpCode >> 8) : Ops->Cmd.OpCode;

  Reg = OpCode << SPI_REG_WR_INSTR_OPCODE_LSB;
  Reg |= Slave->DataWidth << SPI_REG_WR_INSTR_TYPE_DATA_LSB;
  Reg |= Slave->AddrWidth << SPI_REG_WR_INSTR_TYPE_ADDR_LSB;
  MmioWrite32(Slave->RegBase + SPI_REG_WR_INSTR, Reg);

  Reg = SpiApbCalcReadReg(Slave);
  MmioWrite32(Slave->RegBase + SPI_REG_RD_INSTR, Reg);

  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTWRSTARTADDR, Ops->Addr.Val);

  if (Slave->Dtr) {
    Reg = MmioRead32(Slave->RegBase + SPI_REG_WR_COMPLETION_CTRL);
    Reg |= SPI_REG_WR_DISABLE_AUTO_POLL;
    MmioWrite32(Slave->RegBase + SPI_REG_WR_COMPLETION_CTRL, Reg);
  }

  Reg = MmioRead32(Slave->RegBase + SPI_REG_SIZE);
  Reg &= ~SPI_REG_SIZE_ADDRESS_MASK;
  Reg |= (Ops->Addr.NBytes - 1);
  MmioWrite32(Slave->RegBase + SPI_REG_SIZE, Reg);

  return EFI_SUCCESS;
}

STATIC
VOID 
SpiApbDelayNanoSec (
  IN UINTN nsec
  )
{
  UINT32 Timeout = DIV_ROUND_UP(nsec, 1000);

  do {
    Timeout--;
    gBS->Stall(1);
  } while(Timeout);
}

STATIC
VOID 
SpiApbWriteLong (
  IN VOID *Addr, 
  IN CONST VOID *Data,
  IN INTN LongLen
  )
{
  UINT32 *ptr;
  UINT32 *ptr2;

  ptr = (UINT32 *)Addr;
  ptr2 = (UINT32 *)Data;

  while (LongLen) {
    *ptr = *ptr2;
    ptr2++;
    LongLen--;
  }
}

STATIC 
VOID 
SpiApbWriteByte (
  IN VOID *Addr, 
  IN CONST VOID *Data,
  IN INTN ByteLen
  )
{
  UINT8 *ptr;
  UINT8 *ptr2;

  ptr = (UINT8 *)Addr;
  ptr2 = (UINT8 *)Data;

  while (ByteLen) {
    *ptr = *ptr2;
    ptr2++;
    ByteLen--;
  }
}

STATIC
EFI_STATUS
SpiApbIndirectWriteExecute (
  IN SPI_DEVICE *Slave,
  IN UINT32 NumTx, 
  IN CONST UINT8 *TxBuf
  )
{
  UINT32 PageSize = Slave->Info->PageSize;
  UINT32 Remaining = NumTx;
  CONST UINT8 *TxBufPtr = TxBuf;
  UINT32 WriteBytes;
  EFI_STATUS Status;

  // Configure the indirect read transfer bytes
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTWRBYTES, NumTx);

  // Start the indirect write transfer
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTWR, SPI_REG_INDIRECTWR_START);

  // Ddelay is required for QSPI module to synchronized internally
  SpiApbDelayNanoSec(Slave->WRDelay);

  while (Remaining > 0) {
    WriteBytes = Remaining > PageSize ? PageSize : Remaining;
    SpiApbWriteLong(Slave->AhbBase, TxBufPtr, WriteBytes >> 2);
    if (WriteBytes % 4) {
      SpiApbWriteByte(Slave->AhbBase,
        TxBufPtr + rounddown(WriteBytes, 4),
        WriteBytes % 4);
    }
    Status = SpiApbWaitForBitLe32(Slave->RegBase + SPI_REG_SDRAMLEVEL,
          SPI_REG_SDRAMLEVEL_WR_MASK <<
          SPI_REG_SDRAMLEVEL_WR_LSB, 0, 10);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Indirect write timed out (%d)\n", Status));
      goto FailWrite;
    }
    TxBufPtr += WriteBytes;
    Remaining -= WriteBytes;
  }

  // Check indirect done status
  Status = SpiApbWaitForBitLe32(Slave->RegBase + SPI_REG_INDIRECTWR,
        SPI_REG_INDIRECTWR_DONE, 1, 10);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Indirect write completion error (%d)\n", Status));
    goto FailWrite;
  }

  // Clear indirect completion status
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTWR, SPI_REG_INDIRECTWR_DONE);
  return EFI_SUCCESS;

FailWrite:
  // Cancel the indirect write
  MmioWrite32(Slave->RegBase + SPI_REG_INDIRECTWR, SPI_REG_INDIRECTWR_CANCEL);
  return Status;
}

EFI_STATUS
SpiApbWriteExecute (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  )
{
  UINT32 To = Ops->Addr.Val;
  CONST VOID *Buf = Ops->Data.Buf.Out;
  UINT32 Len = Ops->Data.NBytes;
  EFI_STATUS Status;

  if (!Slave->Dtr && Slave->DacMode && (To + Len < Slave->AhbSize)) {
    CopyMem((Slave->AhbBase + To), Buf, Len);
    Status = SpiApbWaitIdle(Slave->RegBase);
    if (EFI_ERROR (Status)) {
      return EFI_TIMEOUT;
    }
    return EFI_SUCCESS;
  }

  Status = SpiApbIndirectWriteExecute(Slave, Len, Buf);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}
