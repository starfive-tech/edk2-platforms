/** @file
  UART Serial Port library functions

  Copyright (c) 2019, Hewlett Packard Enterprise Development LP. All rights reserved.<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Base.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/SerialPortLib.h>
#include <Include/StarFiveUart.h>

//---------------------------------------------
// UART Register Offsets
//---------------------------------------------

#define UART_RBR_OFFSET		0	/* In:  Recieve Buffer Register */
#define UART_THR_OFFSET		0	/* Out: Transmitter Holding Register */
#define UART_DLL_OFFSET		0	/* Out: Divisor Latch Low */
#define UART_IER_OFFSET		1	/* I/O: Interrupt Enable Register */
#define UART_DLM_OFFSET		1	/* Out: Divisor Latch High */
#define UART_FCR_OFFSET		2	/* Out: FIFO Control Register */
#define UART_IIR_OFFSET		2	/* I/O: Interrupt Identification Register */
#define UART_LCR_OFFSET		3	/* Out: Line Control Register */
#define UART_MCR_OFFSET		4	/* Out: Modem Control Register */
#define UART_LSR_OFFSET		5	/* In:  Line Status Register */
#define UART_MSR_OFFSET		6	/* In:  Modem Status Register */
#define UART_SCR_OFFSET		7	/* I/O: Scratch Register */
#define UART_MDR1_OFFSET	8	/* I/O:  Mode Register */

#define UART_LSR_FIFOE		0x80	/* Fifo error */
#define UART_LSR_TEMT		0x40	/* Transmitter empty */
#define UART_LSR_THRE		0x20	/* Transmit-hold-register empty */
#define UART_LSR_BI		0x10	/* Break interrupt indicator */
#define UART_LSR_FE		0x08	/* Frame error indicator */
#define UART_LSR_PE		0x04	/* Parity error indicator */
#define UART_LSR_OE		0x02	/* Overrun error indicator */
#define UART_LSR_DR		0x01	/* Receiver data ready */
#define UART_LSR_BRK_ERROR_BITS	0x1E	/* BI, FE, PE, OE bits */

//---------------------------------------------
// UART Settings
//---------------------------------------------

#define UART_BAUDRATE  115200
#define SYS_CLK        FixedPcdGet32(PcdU5PlatformSystemClock)

BOOLEAN Initiated = TRUE;

/**
  Get value from serial port register.

  @param  RegIndex   Register index

  @retval Vale returned from from serial port.

**/

#if 0

#endif
//static UINT32 uart8250_reg_width = 4;
//static UINT32 uart8250_reg_shift = 2;
UINT32 GetReg (
  IN UINT32 RegIndex
  )
{
  STATIC volatile UINT32 * const uart = (UINT32 *)FixedPcdGet32(PcdU5UartBase);

  return readl ((volatile void *)(&uart[RegIndex]));
}

/**
  Set serial port register.

  @param RegIndex   Register index
  @param Value      Value write to Register

**/
VOID SetReg (
  IN UINT32 RegIndex,
  IN UINT32 Value
  )
{
  STATIC volatile UINT32 * const uart = (UINT32 *)FixedPcdGet32(PcdU5UartBase);

  writel (Value, (volatile void *)(&uart[RegIndex]));
}

/**
  Character output to serial port.

  @param Ch        The character to serial port.

**/
VOID StarfiveUartPutChar (
  IN UINT8 Ch
  )
{
  while ((GetReg (UART_LSR_OFFSET) & UART_LSR_THRE) == 0);

  SetReg (UART_THR_OFFSET, Ch);
}

/**
  Get character from serial port.

  @retval character        The character from serial port.

**/
UINT32 StarfiveUartGetChar (VOID)
{
  UINT32 Ret;

  Ret = GetReg (UART_LSR_OFFSET);
  if (Ret & UART_LSR_DR) {
    return GetReg(UART_RBR_OFFSET);
  }
  return -1;
}
/**
  Find minimum divisor divides in_freq to max_target_hz;
  Based on uart driver n SiFive FSBL.

  f_baud = f_in / (div + 1) => div = (f_in / f_baud) - 1
  The nearest integer solution requires rounding up as to not exceed max_target_hz.
  div  = ceil(f_in / f_baud) - 1
   = floor((f_in - 1 + f_baud) / f_baud) - 1
  This should not overflow as long as (f_in - 1 + f_baud) does not exceed
  2^32 - 1, which is unlikely since we represent frequencies in kHz.

  @param Freq         The given clock to UART.
  @param MaxTargetHZ  Target baudrate.

**/
#if 0
UINT32
UartMinClkDivisor (
  IN UINT64 Freq,
  IN UINT64 MaxTargetHZ
  )
{
    UINT64 Quotient;

    Quotient = (Freq + MaxTargetHZ - 1) / (MaxTargetHZ);
    if (Quotient == 0) {
        return 0;
    } else {
        return Quotient - 1;
    }
}
#endif
/**
  Initialize the serial device hardware.

  If no initialization is required, then return RETURN_SUCCESS.
  If the serial device was successfuly initialized, then return RETURN_SUCCESS.
  If the serial device could not be initialized, then return RETURN_DEVICE_ERROR.

  @retval RETURN_SUCCESS        The serial device was initialized.
  @retval RETURN_DEVICE_ERROR   The serail device could not be initialized.

**/
EFI_STATUS
EFIAPI
SerialPortInitialize (
  VOID
  )
{
#if 0
  
  UINT32 CurrentDivisor;

  Divisor = UartMinClkDivisor (SYS_CLK / 2, UART_BAUDRATE);
  if (Divisor == 0) {
    return EFI_INVALID_PARAMETER;
  }
  CurrentDivisor = GetReg(UART_REG_DIV);
  if (Divisor != CurrentDivisor) {
    uart8250_init(FixedPcdGet32(PcdU5UartBase),
                             SYS_CLK/2, /* todo */
                             UART_BAUDRATE,
                             0, /* shift */
                             4);
  }
#endif
#if 0
  UINT32 bdiv;

  bdiv = (SYS_CLK + 8 * UART_BAUDRATE) / (16 * UART_BAUDRATE);
  
  /* Disable all interrupts */
  SetReg(UART_IER_OFFSET, 0x00);
  /* Enable DLAB */
  SetReg(UART_LCR_OFFSET, 0x80);
  
  if (bdiv) {
	  /* Set divisor low byte */
	  SetReg(UART_DLL_OFFSET, bdiv & 0xff);
	  /* Set divisor high byte */
	  SetReg(UART_DLM_OFFSET, (bdiv >> 8) & 0xff);
  }
  
  /* 8 bits, no parity, one stop bit */
  SetReg(UART_LCR_OFFSET, 0x03);
  /* Enable FIFO */
  SetReg(UART_FCR_OFFSET, 0x01);
  /* No modem control DTR RTS */
  SetReg(UART_MCR_OFFSET, 0x00);
  /* Clear line status */
  GetReg(UART_LSR_OFFSET);
  /* Read receive buffer */
  GetReg(UART_RBR_OFFSET);
  /* Set scratchpad */
  SetReg(UART_SCR_OFFSET, 0x00);
#endif
  return EFI_SUCCESS;
}

/**
  Write data from buffer to serial device.

  Writes NumberOfBytes data bytes from Buffer to the serial device.
  The number of bytes actually written to the serial device is returned.
  If the return value is less than NumberOfBytes, then the write operation failed.

  If Buffer is NULL, then ASSERT().

  If NumberOfBytes is zero, then return 0.

  @param  Buffer           Pointer to the data buffer to be written.
  @param  NumberOfBytes    Number of bytes to written to the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes written to the serial device.
                           If this value is less than NumberOfBytes, then the write operation failed.

**/
UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  )
{
  UINTN Index;

  if (Buffer == NULL || Initiated == FALSE) {
    return 0;
  }

  for(Index = 0; Index < NumberOfBytes; Index ++) {
    StarfiveUartPutChar (Buffer [Index]);
  }

  return Index;
}

/**
  Reads data from a serial device into a buffer.

  @param  Buffer           Pointer to the data buffer to store the data read from the serial device.
  @param  NumberOfBytes    Number of bytes to read from the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes read from the serial device.
                           If this value is less than NumberOfBytes, then the read operation failed.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
  )
{
  UINTN Index;

  if (NULL == Buffer || Initiated == FALSE) {
    return 0;
  }

  for (Index = 0; Index < NumberOfBytes; Index ++) {
    Buffer [Index] = (UINT8)StarfiveUartGetChar ();
  }

  return Index;
}

/**
  Polls a serial device to see if there is any data waiting to be read.

  Polls aserial device to see if there is any data waiting to be read.
  If there is data waiting to be read from the serial device, then TRUE is returned.
  If there is no data waiting to be read from the serial device, then FALSE is returned.

  @retval TRUE             Data is waiting to be read from the serial device.
  @retval FALSE            There is no data waiting to be read from the serial device.

**/
BOOLEAN
EFIAPI
SerialPortPoll (
  VOID
  )
{
  UINT32 Ret;
  Ret = GetReg (UART_LSR_OFFSET);
  if ((Ret & UART_LSR_DR) == UART_LSR_DR)
     return TRUE;
#if 0
  STATIC volatile UINT32 * const uart = (UINT32 *)FixedPcdGet32(PcdU5UartBase);
  UINT32 IP;

  if (Initiated == FALSE) {
    return FALSE;
  }
  IP = MmioRead32 ((UINTN)(uart + UART_REG_IP));
  if(IP & UART_IP_RXWM) {
    return TRUE;
  } else {
    return FALSE;
  }
#endif
  return FALSE;
}

/**
  Sets the control bits on a serial device.

  @param Control                Sets the bits of Control that are settable.

  @retval RETURN_SUCCESS        The new control bits were set on the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortSetControl (
  IN UINT32 Control
  )
{
  if (Initiated == FALSE) {
    return EFI_NOT_READY;
  }
  return RETURN_SUCCESS;
}

/**
  Retrieve the status of the control bits on a serial device.

  @param Control                A pointer to return the current control signals from the serial device.

  @retval RETURN_SUCCESS        The control bits were read from the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortGetControl (
  OUT UINT32 *Control
  )
{
  if (Initiated == FALSE) {
    return EFI_NOT_READY;
  }
  *Control = 0;
  return RETURN_SUCCESS;
}

/**
  Sets the baud rate, receive FIFO depth, transmit/receice time out, parity,
  data bits, and stop bits on a serial device.

  @param BaudRate           The requested baud rate. A BaudRate value of 0 will use the
                            device's default interface speed.
                            On output, the value actually set.
  @param ReveiveFifoDepth   The requested depth of the FIFO on the receive side of the
                            serial interface. A ReceiveFifoDepth value of 0 will use
                            the device's default FIFO depth.
                            On output, the value actually set.
  @param Timeout            The requested time out for a single character in microseconds.
                            This timeout applies to both the transmit and receive side of the
                            interface. A Timeout value of 0 will use the device's default time
                            out value.
                            On output, the value actually set.
  @param Parity             The type of parity to use on this serial device. A Parity value of
                            DefaultParity will use the device's default parity value.
                            On output, the value actually set.
  @param DataBits           The number of data bits to use on the serial device. A DataBits
                            vaule of 0 will use the device's default data bit setting.
                            On output, the value actually set.
  @param StopBits           The number of stop bits to use on this serial device. A StopBits
                            value of DefaultStopBits will use the device's default number of
                            stop bits.
                            On output, the value actually set.

  @retval RETURN_SUCCESS            The new attributes were set on the serial device.
  @retval RETURN_UNSUPPORTED        The serial device does not support this operation.
  @retval RETURN_INVALID_PARAMETER  One or more of the attributes has an unsupported value.
  @retval RETURN_DEVICE_ERROR       The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortSetAttributes (
  IN OUT UINT64             *BaudRate,
  IN OUT UINT32             *ReceiveFifoDepth,
  IN OUT UINT32             *Timeout,
  IN OUT EFI_PARITY_TYPE    *Parity,
  IN OUT UINT8              *DataBits,
  IN OUT EFI_STOP_BITS_TYPE *StopBits
  )
{
  if (Initiated == FALSE) {
    return EFI_NOT_READY;
  }
  return RETURN_SUCCESS;
}
