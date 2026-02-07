## Design Philosophy
- **Pure bare-metal implementation** (no HAL / no LL)
- **CMSIS-style organisation** (Inc/Src separation)
- **Readable and well-structured code**
- **One driver per peripheral**
- Interrupt handling designed to be **explicit and user-controlled**

The drivers are written to be easily integrated into custom bare-metal projects while remaining transparent and easy to debug.

---

## Driver Overview

### GPIO
- Input / Output configuration
- Alternate function support
- Pull-up / Pull-down configuration
- **External interrupt (EXTI) support fully implemented**
- NVIC configuration handled within the driver where applicable

### SPI
- Master mode support
- Configurable clock polarity and phase
- Polling-based data transfer

### I2C
- Master mode communication
- Start/Stop condition handling
- ACK/NACK management

### USART (UART)
- Transmit and receive support
- Configurable baud rate and frame format
- Polling-based operation

---

## Interrupt Handling Strategy
- **GPIO interrupts** (EXTI + NVIC) are **fully implemented** inside the driver
- **SPI, I2C, and USART** drivers are designed to support interrupts, but:
  - Interrupt enablement
  - ISR linking
  - Application-level handling  
  are intentionally left to the user for flexibility

This approach keeps the drivers lightweight while allowing advanced users to integrate custom interrupt-driven workflows.

---

## Test Codes
The repository includes **dedicated test programs** for:
- GPIO functionality and interrupt verification
- SPI communication tests
- I2C transaction validation
- USART transmit/receive testing

These test codes are intended to:
- Validate driver correctness
- Demonstrate expected usage
- Serve as a starting point for application integration

---

## Toolchain & Usage
The drivers are toolchain-agnostic and can be used with:
- `arm-none-eabi-gcc`
- STM32CubeIDE (HAL disabled)
- Makefile or CMake-based bare-metal setups

Basic integration steps:
1. Add `Inc/` to compiler include paths
2. Compile required source files from `Src/`
3. Ensure CMSIS core headers are available
4. Enable peripheral clocks and interrupts as required by the application

---

## References
- **STM32F1 Reference Manual (RM0008)**
- ARM Cortex-M3 Technical Reference Manual
- STM32F1 Series Datasheets

---

## Disclaimer
This project is developed for **educational, experimental, and learning purposes**.  
It is not intended to replace production-grade vendor libraries but to **provide clarity on peripheral behaviour at the register level**.

---

## Author
Developed by **Navdeep**  
Electronics & Communication Engineering  
Interests: Embedded Systems, Low-Level Firmware, MCU Internals, Automotive & Communication Systems
