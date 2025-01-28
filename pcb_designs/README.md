# RedLeader PCB

This repository contains the design files for the **RedLeader PCB**, a custom PCB designed for a differential mobile robot. It is ready for manufacturing and assembly.

## Description

This PCB is designed for a differential mobile robot with four DC motors and two encoders. It facilitates the connection of a 30-pin ESP32 with a bus from the Pololu DUAL_VNH5019 driver and connects the two encoders via JST connectors. Additionally, it provides 5V and GND outputs for peripherals, as well as outputs for encoders 3 and 4 and motors 3 and 4, allowing the unused pins of the ESP32 to be utilized, making it a multifunctional board for various robots.

## Files

- **Gerber Files**: Located in `pcb_designs/GerberFiles/`, these are necessary for PCB fabrication.
- **Drill Files**: Located in `pcb_designs/DrillFiles/`, these are necessary for PCB drilling.
- **Schematic**: `pcb_designs/RedLeaderSchematic.pdf` provides the connection diagram for components.
- **Design Details**: See below for PCB specifications.

## Specifications

- **Layers**: 2
- **Thickness**: 1.57 mm
- **Dimensions**: 53.32 x 73.33 mm
- **Format**: Gerber RS-274X
- **Designed with**: Autodesk Fusion Electronics (v9.7.0)

## Components

- **ESP32_30PIN**: ESP32 module with 30 pins.
- **JST-XH-4PIN**: XH Connector Top Entry - 4 positions, used for connecting the encoders.
- **PINHD-1X13**: 13-position pin header, used for the bus from the Pololu driver.
- **PINHD-1X6**: 6-position pin header, allows for multipurpose use by exposing more pins from the ESP32.
- **PINHD-2X5**: 2x5-position pin header, used for peripherals.
- **TERM-BLK-2P**: 2-position terminal block, used for connecting 12V power.

## Usage

1. **Manufacturing**: Upload the Gerber files in `pcb_designs/GerberFiles/` and the Drill files in `pcb_designs/DrillFiles/` to a PCB manufacturer (e.g., JLCPCB).
2. **Assembly**: Refer to `pcb_designs/RedLeaderSchematic.pdf` for guidance on component placement.
