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
## Technical Skills

🛠 Embedded Systems    | Bare-metal programming, register-level control  
⚙️ Microcontroller     | STM32F4 architecture, memory-mapped peripherals  
🧠 Bitwise Operations  | Masking, shifting, toggling bits in registers  





