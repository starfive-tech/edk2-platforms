/** @file
 *
 *  Copyright (c) 2023, Yuin Yee, Chew (John) <yuinyee.chew@starfivetech.com>.
 *
 *  SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 **/

#ifndef __SPI_MASTER_PROTOCOL_H__
#define __SPI_MASTER_PROTOCOL_H__

#include <Library/NorFlashInfoLib.h>

#ifndef BIT
#define BIT(nr) (1 << (nr))
#endif

#define SPI_CPHA  BIT(0)  // CLock Phase
#define SPI_CPOL  BIT(1)  // Clock Polarity
#define SPI_MODE_0  (0|0)
#define SPI_MODE_1  (0|SPI_CPHA)
#define SPI_MODE_2  (SPI_CPOL|0)
#define SPI_MODE_3  (SPI_CPOL|SPI_CPHA)

extern EFI_GUID gMarvellSpiMasterProtocolGuid;

typedef struct _SPI_MASTER_PROTOCOL SPI_MASTER_PROTOCOL;

#define SPI_MEM_OP_CMD(__OpCode, __BusWidth)      \
  {              \
    .BusWidth = __BusWidth,        \
    .OpCode = __OpCode,        \
    .NBytes = 1,          \
  }

#define SPI_MEM_OP_ADDR(__NBytes, __Val, __BusWidth)    \
  {              \
    .NBytes = __NBytes,        \
    .Val = __Val,          \
    .BusWidth = __BusWidth,        \
  }

#define SPI_MEM_OP_NO_ADDR  { }

#define SPI_MEM_OP_DUMMY(__NBytes, __BusWidth)      \
  {              \
    .NBytes = __NBytes,        \
    .BusWidth = __BusWidth,        \
  }

#define SPI_MEM_OP_NO_DUMMY  { }

#define SPI_MEM_OP_DATA_IN(__NBytes, __Buf, __BusWidth)    \
  {              \
    .Dir = SPI_MEM_DATA_IN,        \
    .NBytes = __NBytes,        \
    .Buf.In = __Buf,        \
    .BusWidth = __BusWidth,        \
  }

#define SPI_MEM_OP_DATA_OUT(__NBytes, __Buf, __BusWidth)  \
  {              \
    .Dir = SPI_MEM_DATA_OUT,      \
    .NBytes = __NBytes,        \
    .Buf.Out = __Buf,        \
    .BusWidth = __BusWidth,        \
  }

#define SPI_MEM_OP_NO_DATA  { }

#define SPI_MEM_OP(__Cmd, __Addr, __Dummy, __Data)    \
  {              \
    .Cmd = __Cmd,          \
    .Addr = __Addr,          \
    .Dummy = __Dummy,        \
    .Data = __Data,          \
  }

/**
 * Standard SPI NOR flash operations
 */
#define SPINOR_RESET_OP            \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0xff, 1),        \
       SPI_MEM_OP_NO_ADDR,          \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_NO_DATA)

#define SPINOR_WR_EN_DIS_OP(Enable)          \
  SPI_MEM_OP(SPI_MEM_OP_CMD((Enable) ? 0x06 : 0x04, 1),    \
       SPI_MEM_OP_NO_ADDR,          \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_NO_DATA)

#define SPINOR_READID_OP(NDummy, Buf, Len)        \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x9f, 1),        \
       SPI_MEM_OP_NO_ADDR,          \
       SPI_MEM_OP_DUMMY(NDummy, 1),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 1))

#define SPINOR_SET_FEATURE_OP(reg, ValPtr)        \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x1f, 1),        \
       SPI_MEM_OP_ADDR(1, reg, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_DATA_OUT(1, ValPtr, 1))

#define SPINOR_GET_FEATURE_OP(reg, ValPtr)        \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x0f, 1),        \
       SPI_MEM_OP_ADDR(1, reg, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_DATA_IN(1, ValPtr, 1))

#define SPINOR_BLK_ERASE_OP(Addr)          \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0xd8, 1),        \
       SPI_MEM_OP_ADDR(3, Addr, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_NO_DATA)

#define SPINOR_PAGE_READ_OP(Addr)          \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x13, 1),        \
       SPI_MEM_OP_ADDR(3, Addr, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_NO_DATA)

#define SPINOR_PAGE_READ_FROM_CACHE_OP(Fast, Addr, NDummy, Buf, Len)  \
  SPI_MEM_OP(SPI_MEM_OP_CMD(Fast ? 0x0b : 0x03, 1),    \
       SPI_MEM_OP_ADDR(2, Addr, 1),        \
       SPI_MEM_OP_DUMMY(NDummy, 1),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 1))

#define SPINOR_PAGE_READ_FROM_CACHE_X2_OP(Addr, NDummy, Buf, Len)  \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x3b, 1),        \
       SPI_MEM_OP_ADDR(2, Addr, 1),        \
       SPI_MEM_OP_DUMMY(NDummy, 1),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 2))

#define SPINOR_PAGE_READ_FROM_CACHE_X4_OP(Addr, NDummy, Buf, Len)  \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x6b, 1),        \
       SPI_MEM_OP_ADDR(2, Addr, 1),        \
       SPI_MEM_OP_DUMMY(NDummy, 1),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 4))

#define SPINOR_PAGE_READ_FROM_CACHE_DUALIO_OP(Addr, NDummy, Buf, Len)  \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0xbb, 1),        \
       SPI_MEM_OP_ADDR(2, Addr, 2),        \
       SPI_MEM_OP_DUMMY(NDummy, 2),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 2))

#define SPINOR_PAGE_READ_FROM_CACHE_QUADIO_OP(Addr, NDummy, Buf, Len)  \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0xeb, 1),        \
       SPI_MEM_OP_ADDR(2, Addr, 4),        \
       SPI_MEM_OP_DUMMY(NDummy, 4),        \
       SPI_MEM_OP_DATA_IN(Len, Buf, 4))

#define SPINOR_PROG_EXEC_OP(Addr)          \
  SPI_MEM_OP(SPI_MEM_OP_CMD(0x10, 1),        \
       SPI_MEM_OP_ADDR(3, Addr, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_NO_DATA)

#define SPINOR_PROG_LOAD(Reset, Addr, Buf, Len)      \
  SPI_MEM_OP(SPI_MEM_OP_CMD(Reset ? 0x02 : 0x84, 1),    \
       SPI_MEM_OP_ADDR(2, Addr, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_DATA_OUT(Len, Buf, 1))

#define SPINOR_PROG_LOAD_X4(Reset, Addr, Buf, Len)      \
  SPI_MEM_OP(SPI_MEM_OP_CMD(Reset ? 0x32 : 0x34, 1),    \
       SPI_MEM_OP_ADDR(2, Addr, 1),        \
       SPI_MEM_OP_NO_DUMMY,          \
       SPI_MEM_OP_DATA_OUT(Len, Buf, 4))

enum spi_mem_data_dir {
  SPI_MEM_NO_DATA,
  SPI_MEM_DATA_IN,
  SPI_MEM_DATA_OUT,
};

typedef struct {
  struct {
    UINT8 NBytes;
    UINT8 BusWidth;
    UINT8 Dtr : 1;
    UINT16 OpCode;
  } Cmd;

  struct {
    UINT8 NBytes;
    UINT8 BusWidth;
    UINT8 Dtr : 1;
    UINT64 Val;
  } Addr;

  struct {
    UINT8 NBytes;
    UINT8 BusWidth;
    UINT8 Dtr : 1;
  } Dummy;

  struct {
    UINT8 BusWidth;
    UINT8 Dtr : 1;
    enum spi_mem_data_dir Dir;
    UINT32 NBytes;
    union {
      VOID *In;
      CONST VOID *Out;
    } Buf;
  } Data;
}SPI_MEM_OPS;

typedef enum {
  SPI_MODE0, // CPOL = 0 & CPHA = 0
  SPI_MODE1, // CPOL = 0 & CPHA = 1
  SPI_MODE2, // CPOL = 1 & CPHA = 0
  SPI_MODE3  // CPOL = 1 & CPHA = 1
} SPI_MODE;

typedef struct {
  SPI_MODE Mode;
  UINT32 AddrSize;
  NOR_FLASH_INFO *Info;

  UINT32  RegBase;
  UINT32  MaxHz;
  UINT32  PrevHz;
  UINT32  CaliHz;
  INTN    ReadDelay;
  UINT32  RefClkHz;
  UINT8   CaliCs;
  UINT8   IsDecodedCs;

  VOID*   AhbBase;
  UINT32  AhbSize;
  UINT16  FifoDepth;
  UINT8   FifoWidth;
  UINT8   TrigAdd;
  BOOLEAN DacMode;
  UINT32  WRDelay;

  /* Flash parameters */
  UINT8  TshslNs;
  UINT8  Tsd2dNs;
  UINT8  TchshNs;
  UINT8  TslchNs;

  /* Transaction protocol parameters. */
  UINT8  InstWidth;
  UINT8  AddrWidth;
  UINT8  DataWidth;
  UINT8  Dtr;

} SPI_DEVICE;

typedef
EFI_STATUS
(EFIAPI *MV_SPI_INIT) (
  IN SPI_MASTER_PROTOCOL *This
  );

typedef
EFI_STATUS
(EFIAPI *MV_SPI_TRANSFER) (
  IN SPI_DEVICE *Slave,
  IN SPI_MEM_OPS *op
  );

typedef
SPI_DEVICE *
(EFIAPI *MV_SPI_SETUP_DEVICE) (
  IN SPI_MASTER_PROTOCOL *This,
  IN SPI_DEVICE *Slave,
  IN SPI_MODE Mode
  );

typedef
EFI_STATUS
(EFIAPI *MV_SPI_FREE_DEVICE) (
  IN SPI_DEVICE *SpiDev
  );

typedef
EFI_STATUS
(EFIAPI *MV_SPI_SET_SPEED) (
  IN SPI_DEVICE *Slave,
  IN UINT32 Hz
  );

typedef
EFI_STATUS
(EFIAPI *MV_SPI_SET_MODE) (
  IN SPI_DEVICE *Slave,
  IN UINT8 Mode
  );

struct _SPI_MASTER_PROTOCOL {
  MV_SPI_INIT         Init;
  MV_SPI_TRANSFER     Transfer;
  MV_SPI_SETUP_DEVICE SetupDevice;
  MV_SPI_FREE_DEVICE  FreeDevice;
  MV_SPI_SET_SPEED   SetSpeed;
  MV_SPI_SET_MODE    SetMode;
};

#endif // __SPI_MASTER_PROTOCOL_H__
