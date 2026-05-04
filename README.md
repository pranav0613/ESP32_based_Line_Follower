# ESP32_based_Line_Follower
Embedded C++ firmware for an autonomous line-following robot utilizing PID control and an 8-channel QTR sensor array.

## 📖 Project Overview
This repository contains the embedded C++ firmware for an autonomous Line Follower Robot (LFR). The system relies on an 8-channel QTR reflectance sensor array to read track data and utilizes a custom-tuned PID (Proportional-Integral-Derivative) control algorithm to dynamically adjust differential motor speeds, ensuring smooth and rapid track navigation.

## 🛠️ Hardware Stack
Microcontroller: ESP32

Sensors: Pololu QTR-8RC Reflectance Sensor Array

Actuators: N20 DC Gear Motors driven via a dual H-bridge motor driver (In this case, TB6612FNG is used) PWM speed control

## 💻 Firmware Architecture & PID Implementation
The core logic (LFR.ino) actively calculates positional error and adjusts motor PWM outputs in real-time.

PID Tuning Parameters
The control loop was empirically tuned for smooth continuous motion rather than jittery binary corrections:

Proportional (Kp): 0.07 - Corrects the immediate cross-track error.

Integral (Ki): 0.0008 - Accumulates past errors to eliminate steady-state offset.

Derivative (Kd): 0.7 - Predicts future error to dampen oscillation and overshoot.

Key Features in Code
Dynamic Calibration: A setup routine that sweeps the QTR sensors over the line to establish dynamic environmental threshold values for black/white contrasts.

Constraint Limits: Motor outputs are computationally constrained to maximum speed thresholds (maxspeed = 80) to prevent current-draw brownouts and track derailment.

## 🤖 Physical Prototype
The fully assembled hardware prototype featuring the chassis, sensor placement, and embedded electronics.

## 🚀 Key Learning Outcomes
Translated control theory into functional embedded C++ code.

Tuned closed-loop feedback systems using PID algorithms.

Handled physical hardware integration, bridging software logic with PWM-driven mechanical actuators.
