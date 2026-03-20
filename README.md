# B.HIVE — Autonomous Mobile Robot (AMR)

Part of the **B.HIVE Autonomous Luggage Concierge** system, a capstone project at the Singapore University of Technology and Design (SUTD).

This repository contains the ROS 2 software stack for the AMR — the mobile base responsible for navigating to and from luggage loading zones autonomously.

## Overview

The AMR is built on a ROS 2 distributed node architecture running on a Jetson Orin Nano. It handles real-time sensor data publishing and subscribing, motor control via a ZLAC motor driver, and robot bringup for coordinated operation within the broader B.HIVE ecosystem.

## Repository Structure

| Package | Description |
|---|---|
| `robot_bringup` | Launch files and system startup configuration |
| `robot_description` | URDF/robot model definitions |
| `grove_sensors` | ROS 2 node for Grove sensor integration |
| `zlac_driver` | Motor driver interface for ZLAC servo controllers |

## Tech Stack

- **Framework:** ROS 2
- **Language:** Python
- **Hardware:** NVIDIA Jetson Orin Nano
- **Build:** CMake / colcon

## Related

- [B.HIVE Load & Weigh](https://github.com/friyk/2026S23CapstoneBHIVELOADANDWEIGH) — Embedded firmware for the luggage loading and weighing subsystem
