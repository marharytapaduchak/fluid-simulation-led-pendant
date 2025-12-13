# Real-time fluid simulation

A compact wearable LED pendant that visualizes real-time fluid motion.

The project combines a simplified FLIP-based fluid simulation running on an STM32 microcontroller with a custom circular LED matrix driven via Charlieplexing. An accelerometer provides real-time orientation data, allowing the simulated fluid to respond naturally to gravity and motion.

Includes simulation code, hardware design files, and a 3D-printed pendant prototype.

- `eulier_faster.py` - the first step: implementation of the eulier simulation on Python
- `pendant.blend` and `pendant.stl` - case for the pendant
- `pendant_sim.html` - fluid simulation on Java Script
- `fluid_simulation_on_stm_nucleo_L432.zip` - zip file with algorithm on the matrix
