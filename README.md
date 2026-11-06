# stm32-encoder
STM32-based encoder interface for quadrature encoder measurement and rotational velocity estimation.

## Project Overview

Development of an embedded firmware solution for quadrature encoder signal acquisition using STM32 microcontroller with HAL. The system is designed to measure the rotational position of the encoder. Encoder values are used for timer-based velocity estimation. The estimation strategy (estimation from period / estimation from frequency) is adjusted to the current velocity estimation.

## Key Features

- Quadrature encoder pulse acquisition with hardware timer
- Position calculation up to signed 32 bit positions
- Rotational velocity estimation
- Smooth transition between estimation strategies

## Hardware

| Component | Specification |
|---|---|
| Microcontroller | tested with STM32f411ceu6 (Blackpill Dev Board) |
| Encoder | Optical or magnetic encoders with A/B channels|

## Software

- Programming language: C
- IDE: STM32CubeIDE/CubeMX
- Framework: STM32 HAL

## Documentation

See the 'docs/' directory for technical specifications

---
