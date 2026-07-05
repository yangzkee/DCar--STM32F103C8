# Mac Makefile Build

This folder can be built without Keil on macOS using `arm-none-eabi-gcc`.

## Install Tools

```sh
brew install arm-none-eabi-gcc stlink openocd
```

This Mac already has `arm-none-eabi-gcc`, `st-flash`, `openocd`, and `make`.

## Build

```sh
cd DFCom_Example
make
```

Outputs:

- `build/dfcom_example.elf`
- `build/dfcom_example.bin`
- `build/dfcom_example.hex`
- `build/dfcom_example.map`

## Flash With ST-Link

Connect ST-Link SWD to the STM32 board:

- SWDIO
- SWCLK
- GND
- 3V3 sense if your debugger requires it

Then run:

```sh
make flash
```

The default flash command is:

```sh
st-flash --reset write build/dfcom_example.bin 0x08000000
```

OpenOCD alternative:

```sh
make flash-openocd
```

For CMSIS-DAP:

```sh
make OPENOCD_INTERFACE=interface/cmsis-dap.cfg flash-openocd
```

## Runtime Serial Connections

- `USART1 PA9/PA10 @ 460800` connects to the DcarON vehicle UART.
- `USART2 PA2/PA3 @ 115200` connects to USB-TTL for terminal output.

Open a serial terminal on the USB-TTL port at `115200 8N1`.

## MCU Target

The Keil project in this zip is configured as `STM32F103C8`, so this Makefile defaults to:

- `STM32F10X_MD`
- 64KB Flash
- 20KB RAM
- `GCC/STM32F103C8Tx_FLASH.ld`

If you actually use an STM32F103ZET6 board, create or switch to a linker script with the ZET6 memory size and change the startup/vector coverage if you enable extra high-density peripherals. The current example only uses common F103 peripherals: USART1, USART2, DMA1, TIM2, GPIOA, RCC, AFIO.
