/** @file

  Copyright (c) 
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef __SPI_APB_DXE_H__
#define __SPI_APB_DXE_H__

#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseMemoryLib.h>

#include <Protocol/Spi.h>

// Configs
#define SPI_REG_POLL_US         1
#define SPI_REG_RETRY           10000
#define SPI_POLL_IDLE_RETRY     3
#define SPI_STIG_DATA_LEN_MAX   8
#define SPI_DUMMY_CLKS_PER_BYTE 8
#define SPI_DUMMY_CLKS_MAX      31

// Transfer Mode
#define SPI_INST_TYPE_SINGLE    0
#define SPI_INST_TYPE_DUAL      1
#define SPI_INST_TYPE_QUAD      2
#define SPI_INST_TYPE_OCTAL     3

//
// QSPI controller's config and status register (offset from QSPI_BASE)
//
#define SPI_REG_CONFIG                     0x00
#define SPI_REG_CONFIG_ENABLE              BIT(0)
#define SPI_REG_CONFIG_CLK_POL             BIT(1)
#define SPI_REG_CONFIG_CLK_PHA             BIT(2)
#define SPI_REG_CONFIG_DIRECT              BIT(7)
#define SPI_REG_CONFIG_DECODE              BIT(9)
#define SPI_REG_CONFIG_XIP_IMM             BIT(18)
#define SPI_REG_CONFIG_CHIPSELECT_LSB      10
#define SPI_REG_CONFIG_BAUD_LSB            19
#define SPI_REG_CONFIG_DTR_PROTO           BIT(24)
#define SPI_REG_CONFIG_DUAL_OPCODE         BIT(30)
#define SPI_REG_CONFIG_IDLE_LSB            31
#define SPI_REG_CONFIG_CHIPSELECT_MASK     0xF
#define SPI_REG_CONFIG_BAUD_MASK           0xF

#define SPI_REG_RD_INSTR                   0x04
#define SPI_REG_RD_INSTR_OPCODE_LSB        0
#define SPI_REG_RD_INSTR_TYPE_INSTR_LSB    8
#define SPI_REG_RD_INSTR_TYPE_ADDR_LSB     12
#define SPI_REG_RD_INSTR_TYPE_DATA_LSB     16
#define SPI_REG_RD_INSTR_MODE_EN_LSB       20
#define SPI_REG_RD_INSTR_DUMMY_LSB         24
#define SPI_REG_RD_INSTR_TYPE_INSTR_MASK   0x3
#define SPI_REG_RD_INSTR_TYPE_ADDR_MASK    0x3
#define SPI_REG_RD_INSTR_TYPE_DATA_MASK    0x3
#define SPI_REG_RD_INSTR_DUMMY_MASK        0x1F

#define SPI_REG_WR_INSTR                   0x08
#define SPI_REG_WR_INSTR_OPCODE_LSB        0
#define SPI_REG_WR_INSTR_TYPE_ADDR_LSB     12
#define SPI_REG_WR_INSTR_TYPE_DATA_LSB     16

#define SPI_REG_DELAY                      0x0C
#define SPI_REG_DELAY_TSLCH_LSB            0
#define SPI_REG_DELAY_TCHSH_LSB            8
#define SPI_REG_DELAY_TSD2D_LSB            16
#define SPI_REG_DELAY_TSHSL_LSB            24
#define SPI_REG_DELAY_TSLCH_MASK           0xFF
#define SPI_REG_DELAY_TCHSH_MASK           0xFF
#define SPI_REG_DELAY_TSD2D_MASK           0xFF
#define SPI_REG_DELAY_TSHSL_MASK           0xFF

#define SPI_REG_RD_DATA_CAPTURE            0x10
#define SPI_REG_RD_DATA_CAPTURE_BYPASS     BIT(0)
#define SPI_REG_RD_DATA_CAPTURE_DELAY_LSB  1
#define SPI_REG_RD_DATA_CAPTURE_DELAY_MASK 0xF

#define SPI_REG_SIZE                       0x14
#define SPI_REG_SIZE_ADDRESS_LSB           0
#define SPI_REG_SIZE_PAGE_LSB              4
#define SPI_REG_SIZE_BLOCK_LSB             16
#define SPI_REG_SIZE_ADDRESS_MASK          0xF
#define SPI_REG_SIZE_PAGE_MASK             0xFFF
#define SPI_REG_SIZE_BLOCK_MASK            0x3F

#define SPI_REG_SRAMPARTITION              0x18
#define SPI_REG_INDIRECTTRIGGER            0x1C

#define SPI_REG_REMAP                      0x24
#define SPI_REG_MODE_BIT                   0x28

#define SPI_REG_SDRAMLEVEL                 0x2C
#define SPI_REG_SDRAMLEVEL_RD_LSB          0
#define SPI_REG_SDRAMLEVEL_WR_LSB          16
#define SPI_REG_SDRAMLEVEL_RD_MASK         0xFFFF
#define SPI_REG_SDRAMLEVEL_WR_MASK         0xFFFF

#define SPI_REG_WR_COMPLETION_CTRL         0x38
#define SPI_REG_WR_DISABLE_AUTO_POLL       BIT(14)

#define SPI_REG_IRQSTATUS                  0x40
#define SPI_REG_IRQMASK                    0x44

#define SPI_REG_INDIRECTRD                 0x60
#define SPI_REG_INDIRECTRD_START           BIT(0)
#define SPI_REG_INDIRECTRD_CANCEL          BIT(1)
#define SPI_REG_INDIRECTRD_INPROGRESS      BIT(2)
#define SPI_REG_INDIRECTRD_DONE            BIT(5)

#define SPI_REG_INDIRECTRDWATERMARK        0x64
#define SPI_REG_INDIRECTRDSTARTADDR        0x68
#define SPI_REG_INDIRECTRDBYTES            0x6C

#define SPI_REG_CMDCTRL                    0x90
#define SPI_REG_CMDCTRL_EXECUTE            BIT(0)
#define SPI_REG_CMDCTRL_INPROGRESS         BIT(1)
#define SPI_REG_CMDCTRL_DUMMY_LSB          7
#define SPI_REG_CMDCTRL_WR_BYTES_LSB       12
#define SPI_REG_CMDCTRL_WR_EN_LSB          15
#define SPI_REG_CMDCTRL_ADD_BYTES_LSB      16
#define SPI_REG_CMDCTRL_ADDR_EN_LSB        19
#define SPI_REG_CMDCTRL_RD_BYTES_LSB       20
#define SPI_REG_CMDCTRL_RD_EN_LSB          23
#define SPI_REG_CMDCTRL_OPCODE_LSB         24
#define SPI_REG_CMDCTRL_DUMMY_MASK         0x1F
#define SPI_REG_CMDCTRL_WR_BYTES_MASK      0x7
#define SPI_REG_CMDCTRL_ADD_BYTES_MASK     0x3
#define SPI_REG_CMDCTRL_RD_BYTES_MASK      0x7
#define SPI_REG_CMDCTRL_OPCODE_MASK        0xFF

#define SPI_REG_INDIRECTWR                 0x70
#define SPI_REG_INDIRECTWR_START           BIT(0)
#define SPI_REG_INDIRECTWR_CANCEL          BIT(1)
#define SPI_REG_INDIRECTWR_INPROGRESS      BIT(2)
#define SPI_REG_INDIRECTWR_DONE            BIT(5)

#define SPI_REG_INDIRECTWRWATERMARK        0x74
#define SPI_REG_INDIRECTWRSTARTADDR        0x78
#define SPI_REG_INDIRECTWRBYTES            0x7C

#define SPI_REG_CMDADDRESS                 0x94
#define SPI_REG_CMDREADDATALOWER           0xA0
#define SPI_REG_CMDREADDATAUPPER           0xA4
#define SPI_REG_CMDWRITEDATALOWER          0xA8
#define SPI_REG_CMDWRITEDATAUPPER          0xAC

#define SPI_REG_OP_EXT_LOWER               0xE0
#define SPI_REG_OP_EXT_READ_LSB            24
#define SPI_REG_OP_EXT_WRITE_LSB           16
#define SPI_REG_OP_EXT_STIG_LSB            0


#define SPI_REG_IS_IDLE(base)            \
  ((MmioRead32(base + SPI_REG_CONFIG) >> \
    SPI_REG_CONFIG_IDLE_LSB) & 0x1)

#define SPI_GET_RD_SRAM_LEVEL(RegBase)            \
  (((MmioRead32(RegBase + SPI_REG_SDRAMLEVEL)) >> \
  SPI_REG_SDRAMLEVEL_RD_LSB) & SPI_REG_SDRAMLEVEL_RD_MASK)

#define SPI_GET_WR_SRAM_LEVEL(RegBase)            \
  (((MmioRead32(RegBase + SPI_REG_SDRAMLEVEL)) >> \
  SPI_REG_SDRAMLEVEL_WR_LSB) & SPI_REG_SDRAMLEVEL_WR_MASK)


#define rounddown(x, y) ( \
{                         \
  typeof(x) __x = (x);    \
  __x - (__x % (y));      \
}                         \
)

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define uswap_32(x) \
  ((((x) & 0xff000000) >> 24) | \
   (((x) & 0x00ff0000) >>  8) | \
   (((x) & 0x0000ff00) <<  8) | \
   (((x) & 0x000000ff) << 24))
# define cpu_to_be32(x)    uswap_32(x)


VOID 
SpiApbControllerEnable (
  IN UINT32 RegBaseAddr
  );

VOID 
SpiApbControllerDisable (
  IN UINT32 RegBaseAddr
  );

VOID 
SpiApbDacModeEnable (
  IN UINT32 RegBase
  );

VOID 
SpiApbChipSelect (
  IN UINT32 RegBase,
  IN UINT32 ChipSel, 
  IN UINT32 DecoderEnable
  );

VOID 
SpiApbSetClkMode (
  IN UINT32 RegBase, 
  IN UINTN Mode
  );

VOID 
SpiApbConfigBaudrateDiv (
  IN UINT32 RegBase,
  IN UINT32 RefClkHz, 
  IN UINT32 SclkHz
  );

VOID 
SpiApbDelay (
  IN UINT32 RegBase,
  IN UINT32 RefCLk, 
  IN UINT32 SclkHz,
  IN UINT32 TshslNs, 
  IN UINT32 Tsd2dNs,
  IN UINT32 TchshNs, 
  IN UINT32 TslchNs
  );

VOID 
SpiApbReadDataCapture (
  IN UINT32 RegBase,
  IN UINT32 ByPass, 
  IN UINT32 Delay
  );

EFI_STATUS
SpiApbCommandReadSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbCommandRead (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbCommandWriteSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbCommandWrite (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbReadSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );
  
EFI_STATUS
SpiApbReadExecute (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbWriteSetup (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

EFI_STATUS
SpiApbWriteExecute (
  IN SPI_DEVICE *Slave,
  IN CONST SPI_MEM_OPS *Ops
  );

#endif //__SPI_APB_DXE_H__
