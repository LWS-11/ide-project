# Remote Acquisition Receiver - Five Time Bases

This project is the display-side STM32F746G-DISCO firmware:

- receive the final board-to-board CH1/CH2 stream over UART7
  (`PF7/PF6`, 1 Mbit/s), while still accepting PC test frames on USART1;
- validate version, length, channel flag, and CRC-16;
- discard damaged frames and resynchronize from the next `A5 5A` marker;
- display CH1 and CH2 in different colors over the 0-3 V range;
- select 10 ms/div, 5 ms/div, 1 ms/div, 500 us/div, or 100 us/div
  with the touch screen (`10MS`, `5MS`, `1MS`, `500US`, `100US`);
- detect the FT5336 touch controller over I2C3 (`PH7`/`PH8`, address `0x70`);
- retry an unacknowledged time-base command until a matching data frame arrives;
- notify the acquisition board of a time-base change over UART7;
- keep USART1 as a PC debug/monitor port and mirror UART7 ACK/control traffic
  to it;
- show channel voltages, RX status, sequence number, selected time base, and
  touch-controller state;
- use two RGB565 frame buffers in the external SDRAM to avoid tearing.

The 216 MHz clock tree comes from the user's original `design.ioc`: HSE 25 MHz,
PLL M=25, N=432, P=2, AHB=216 MHz, APB1=54 MHz, APB2=108 MHz.

## Serial interfaces

- USART1, 115200 baud, 8 data bits, no parity, 1 stop bit.
- RX: `PB7` (`VCP_RX` on STM32F746G-DISCO).
- TX: `PA9` (`VCP_TX`).
- USART1 is the PC/ST-LINK Virtual COM Port, not the board-to-board link.
- UART7, 1000000 baud, 8 data bits, no parity, 1 stop bit.
- UART7 TX: `PF7` (Arduino `A4`).
- UART7 RX: `PF6` (Arduino `A5`).
- Both UARTs use interrupt-driven one-byte receive into independent ring
  buffers.
- Touch controller: FT5336, I2C3, 7-bit address `0x70`.

On reset the receiver emits:

```text
R2 READY,VER=1,TOUCH=81
```

`81` is decimal for the FT5336 chip ID `0x51`. `TOUCH=ERR` means the touch
controller was not detected.

## Channel frame format

All multi-byte fields are little-endian.

| Offset | Size | Meaning |
| ---: | ---: | --- |
| 0 | 2 | sync `A5 5A` |
| 2 | 1 | protocol version, currently `01` |
| 3 | 1 | frame type: CH1=`10`, CH2=`20`, CH1+CH2=`30` |
| 4 | 2 | sequence number |
| 6 | 2 | sample count, 1 to 500 |
| 8 | 2 | sample interval in microseconds |
| 10 | 1 | channel flags: CH1=`01`, CH2=`02`, both=`03`, digital=`04` |
| 11 | 1 | reserved, currently `0x00` |
| 12 | variable | dual extended frame: N interleaved `CH1 mV, CH2 mV, D1..D3` groups |
| end | 2 | CRC-16/CCITT-FALSE over all bytes before the CRC |

The five time bases use 50 samples per division and 500 samples per channel:
10 ms/div uses 200 us, 5 ms/div uses 100 us, 1 ms/div uses 20 us, 500 us/div
uses 10 us, and 100 us/div uses 2 us. The three digital channels are sampled
on the same timer event and share the analog time axis.

CRC parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR. The CRC field itself is little-endian.

## Receiver ACK

Every complete frame produces one text line on UART7. The receiver also mirrors
the line to USART1 so a PC can monitor the board-to-board traffic:

```text
ACK,<status>,<sequence>,<points>,<interval_us>,<ok>,<crc_errors>,<drops>\r\n
```

| Field | Meaning |
| --- | --- |
| `status` | `OK`, `CRC`, or `FORMAT` |
| `sequence` | sequence number copied from the data frame |
| `points` | points per channel |
| `interval_us` | sample interval copied from the data frame |
| `ok` | cumulative accepted-frame count |
| `crc_errors` | cumulative CRC-error count |
| `drops` | cumulative UART ring-buffer drop count |

Example:

```text
ACK,OK,37,500,100,38,2,0
```

## Time-base control from receiver to acquisition board

The receiver sends this 9-byte packet when a touch-screen time-base button or
the user button is pressed. It is sent on UART7 and mirrored to USART1:

| Offset | Size | Meaning |
| ---: | ---: | --- |
| 0 | 2 | sync `C5 5C` |
| 2 | 1 | protocol version `01` |
| 3 | 1 | command `01` (time base) |
| 4 | 1 | time-base index: `0=10ms`, `1=5ms`, `2=1ms`, `3=500us`, `4=100us` |
| 5 | 2 | requested sample interval in microseconds |
| 7 | 2 | CRC-16/CCITT-FALSE over bytes 0-6 |

The user button pauses and resumes waveform updates. Touch buttons select a
time base directly.

For PC automation, `TB=0` through `TB=4` text commands enter the same selection
function as the touch buttons. The receiver then emits the binary
`C5 5C` packet above. This is used by the connection test to verify the
receiver-to-acquisition control path without human timing.

## PC upper-computer test

Connect the ST-LINK Virtual COM Port, then run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\pc_host.ps1 `
  -PortName COM3 -DurationSeconds 12 -FrameDelayMs 100
```

The test checks:

1. normal CH1+CH2 frame and ACK;
2. CRC-corrupted frame rejection;
3. 200/100/20/10/2 us intervals for the five scan-time settings;
4. `C5 5C` time-base control packet and host-side interval switching;
5. continuous bidirectional traffic. Touch any of the five scan-time buttons
   while this phase runs to observe live control packets.

The upper-computer integration notes and recorded test results are in
`上位机.md`.

## Build roles

The same CubeIDE project can build both boards. Select the role in
`Core/Inc/app_role.h`:

```c
#define APP_ROLE APP_ROLE_RECEIVER
```

For the acquisition/sending board:

```c
#define APP_ROLE APP_ROLE_ACQUISITION
```

The acquisition role in `Core/Src/app_acquisition.c` sends two synthetic
500-point channels at 20 frames/s so the complete UART7 path can be tested.
Replace `build_frame()` with the teammate's real ADC sample buffers when the
practical acquisition code is integrated.

For direct TTL testing, connect only:

```text
acquisition A4/PF7 (TX) -> receiver A5/PF6 (RX)
acquisition A5/PF6 (RX) <- receiver A4/PF7 (TX)
acquisition GND         -> receiver GND
```

For 5-metre RS-485, use 3.3V transceivers or automatic-direction RS-485
modules. Manual-direction modules with DE/RE connected to `PE4` can be enabled
in `Core/Inc/board_link.h` by setting
`BOARD_LINK_RS485_DIRECTION_CONTROL` to `1`.

## Build

Open the folder in STM32CubeIDE and build the `Debug` configuration. The new
shared link code is in `Core/Src/board_link.c`. The generated receiver image is:

`Debug/ReceiverPart1.elf`

The prebuilt acquisition demonstration image is:

`Debug/AcquisitionPart1.elf`
