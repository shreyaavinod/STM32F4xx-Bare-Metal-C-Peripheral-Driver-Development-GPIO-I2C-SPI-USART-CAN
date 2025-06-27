# STM32F4xx Peripheral Driver Development (Bare-Metal C)

This project is a custom driver development framework for STM32F4xx microcontrollers using **bare-metal C**. It is built entirely from scratch without relying on STM32 HAL or LL libraries. The goal is to gain low-level control and deep understanding of how hardware peripherals work by programming directly with memory-mapped registers.

Currently, the project implements **GPIO drivers** and will be extended to include **I2C**, **SPI**, **USART**, and **CAN** drivers.

---

## 🧩 Features

- ✅ Full **GPIO Driver** implementation:
  - Configure pin mode (Input, Output, Alt Func, Analog)
  - Read from individual pin or entire port
  - Write to individual pin or entire port
  - Toggle output pins
  - Enable/disable peripheral clocks
  - Reset GPIO ports

- 🚧 Upcoming Drivers:
  - I2C
  - SPI
  - USART
  - CAN

- 🧪 Written in **pure C**, following modular driver structure

---

## 🔧 Tools Used

- STM32CubeIDE  
- STM32F446RE (Nucleo-64 development board)  
- Reference Manual & Datasheet for STM32F446xx  

---

## 💡 Technical Skills Involved
|-----------------------|------------------|
|  Embedded Systems    | Bare-metal programming, register-level control |
|  Microcontroller     | STM32F4 architecture, memory-mapped peripherals |
|  Bitwise Operations  | Masking, shifting, toggling bits in registers |


---

## 📁 Project Structure

STM32F4xx_DRIVER_DEV/
├── drivers/
│ ├── inc/
│ │ ├── stm32f446xx.h # Register definitions
│ │ └── stm32f446xx_gpio_driver.h # GPIO driver API
│ └── src/
│ └── stm32f446xx_gpio_driver.c # GPIO driver logic
├── Startup/
│ └── startup_stm32f446retx.s # Startup file
├── STM32F446RETX_FLASH.ld # Linker script
├── STM32F446RETX_RAM.ld # RAM mapping
├── .gitignore
├── .project, .cproject (optional)
└── README.md
