# Real-Time Fluid Simulation LED Pendant

A wearable LED pendant that visualizes real-time fluid motion on a compact circular display.

This project combines a physics-inspired fluid simulation, motion sensing, custom LED matrix control, and a 3D-printed enclosure. The goal was to create a small interactive pendant where “liquid” reacts naturally to gravity, tilt, and movement.

## Overview

The pendant simulates fluid-like behavior using a simplified FLIP-based approach adapted for real-time execution. Particles represent the visible liquid, while a grid is used to stabilize the simulation and preserve incompressibility.

The visual output is displayed on a custom circular LED matrix. The matrix is controlled using Charlieplexing, which allows many LEDs to be driven with a limited number of microcontroller pins — an important constraint for a compact wearable device.

An accelerometer provides real-time orientation data, making the simulated fluid respond to user motion.

## Key Features

- Real-time fluid simulation inspired by the FLIP method
- Motion-responsive behavior using accelerometer data
- Circular LED matrix visualization
- Charlieplexed LED control for efficient pin usage
- STM32-based embedded implementation
- 3D-modeled and 3D-printed pendant enclosure
- Modular hardware prototype with separate display and control electronics

## Project Structure

```text
.
├── eulier_faster.py
├── pendant_sim.html
├── fluid_simulation_on_stm_nucleo_L432.zip
├── pendant.blend
├── pendant1.stl
└── README.md
```

# Files

- `eulier_faster.py` — early Python implementation of the fluid simulation
- `pendant_sim.html` — browser-based JavaScript simulation prototype
- `fluid_simulation_on_stm_nucleo_L432.zip` — STM32 implementation for running the simulation on hardware
- `pendant.blend` — Blender model of the pendant case
- `pendant1.stl` — 3D-printable pendant enclosure model

# How It Works

The system consists of three main parts:

## 1. Fluid Simulation

The simulation is based on a simplified incompressible fluid model. It uses particles to preserve visually rich motion and a grid to maintain stable behavior.

Each simulation step includes:

- Moving particles according to velocity and gravity
- Transferring particle velocities to a grid
- Applying pressure projection to reduce compression
- Updating particle velocities from the corrected grid
- Applying density correction to avoid clustering or gaps

This creates smooth, liquid-like motion that can run in real time on embedded hardware.

## 2. Motion Input

An accelerometer detects the pendant’s orientation and movement. Its data is used as the gravity direction in the simulation, so the virtual liquid reacts when the pendant is tilted or rotated.

## 3. LED Matrix Display

The fluid density is mapped to LED brightness. Particles influence nearby LEDs, creating a glowing visual pattern that resembles moving liquid.

The display uses Charlieplexing, allowing a large number of LEDs to be controlled with fewer MCU pins. Only one LED is activated at a time, but rapid scanning creates the perception of a continuously lit display.

# Hardware

The prototype includes:

- STM32 microcontroller
- Accelerometer
- Custom circular LED matrix
- Charlieplexed LED driver network
- Li-ion battery power system
- 3D-printed pendant enclosure
- Transparent dome for the LED display

The final hardware approach uses a modular architecture: a custom LED matrix board combined with an STM32 development platform. This made debugging, programming, and integration more reliable.

# Enclosure

The pendant case was designed as a compact 3D-printed structure with:

- a front lid for holding the transparent dome,
- a middle spacer/body for positioning the electronics,
- a back cover with access for charging.

The enclosure keeps the LED matrix aligned under the transparent dome while protecting the electronics inside.

# Result

The project demonstrates a working prototype of an interactive LED pendant with real-time fluid-like behavior. The final version successfully combines simulation, motion sensing, LED visualization, and physical hardware integration.

Although the prototype is not as compact as the original target, it proves the core idea: a wearable device can display responsive fluid simulation using embedded hardware and a custom LED matrix.

# Authors

- Marharyta Paduchak
- Daryna Shevchuk
- Olena Dovbenchuk

Department of Computer Science  
Ukrainian Catholic University  
Lviv, Ukraine

Mentor: Oleksiy Hoyev
