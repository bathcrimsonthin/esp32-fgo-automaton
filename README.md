# ESP32 FGO Automaton
An ESP32-based system that automates FGO grinding.

## Overview
ESP32 connects to smartphone via BLE and operates using switch control.
INMP441 detects sound effects to make sure that the operation was done.

## Motivation
I wanted to save time and effort spent on FGO grinding.

## Features
- Switch control via BLE
- Pick up sounds with INMP441
- Identify sounds using MFCC

## Hardware
| Component | Quantity |
|:---:|:---:|
| ESP32-WROOM-32E | 1 |
| INMP441 | 1 |
| 5V Power Supply | 1 |
