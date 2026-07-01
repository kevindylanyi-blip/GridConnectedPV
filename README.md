# Three-Phase Grid-Connected Solar Photovoltaic System

A MATLAB/Simulink model of a **35 kW rooftop grid-connected solar PV system** that injects power into a three-phase 415 V, 50 Hz grid with **unity power factor (UPF)**, implemented without an intermediate DC-DC converter (boostless transformerless topology).

This example supports design decisions about the number of panels, connection topology, MPPT algorithm selection, inverter topology, and controller tuning — all parameterized from a solar panel manufacturer datasheet.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-blue)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-R2023b-orange)](https://www.mathworks.com/products/simulink.html)
[![Simscape Electrical](https://img.shields.io/badge/Simscape%20Electrical-Required-green)](https://www.mathworks.com/products/simscape-electrical.html)
[![License](https://img.shields.io/badge/License-MathWorks%20Example-lightgrey)](LICENSE)

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Model Components](#model-components)
  - [Solar Plant](#solar-plant)
  - [Power Circuit & Inverter](#power-circuit--inverter)
  - [Controller (dq-Frame)](#controller-dq-frame)
  - [MPPT Algorithms](#mppt-algorithms)
  - [PLL & Grid Protection](#pll--grid-protection)
  - [Modulation & Gating](#modulation--gating)
- [Variant Configuration](#variant-configuration)
- [Linearization & Bode Plot](#linearization--bode-plot)
- [Key Parameters](#key-parameters)
- [Requirements](#requirements)
- [References](#references)

---

## Overview

This Simulink model simulates a complete grid-tied solar PV system, from the PV array through the inverter to the grid connection point. The system is designed to:

- **Size the PV array** — Determine the number of series panels per string and parallel strings needed to meet a target power rating while ensuring the DC bus voltage always exceeds the peak grid voltage.
- **Compare MPPT techniques** — Switch between Incremental Conductance and Perturbation & Observation algorithms via a workspace variable.
- **Evaluate inverter topologies** — Choose between average-model, two-level, and three-level inverter variants.
- **Design and tune controllers** — PI-based dq-frame current and voltage controllers with analytical tuning formulas.
- **Analyze stability** — Generate open-loop Bode plots of the inner current loop to assess gain and phase margins.
- **Test protection schemes** — Grid over/under voltage, over/under frequency, DC bus voltage, over-current, and short-circuit protection with latching trip logic.

### System Specifications

| Parameter | Value |
|---|---|
| **Target Plant Power** | 35 kW |
| **Grid Voltage (L-L)** | 415 V RMS |
| **Grid Frequency** | 50 Hz |
| **Solar Panel Power** | ~225 W per panel |
| **Switching Frequency** | 20 kHz |
| **Operating Temperature** | 40 °C |
| **Power Factor** | Unity (Iqref = 0) |
| **DC-DC Converter** | None (boostless) |

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         GridConnectedPV.slx                           │
├─────────────┬──────────┬────────────┬────────────┬───────────────────┤
│ Solar Plant │  Power   │  Filter &  │    Grid    │  Circuit Breaker  │
│  (PV Array) │ Circuit  │ AC Meas.   │  (415V,    │   (Three-Phase)   │
│   with Cpv  │ (Inverter│  (RLC +    │   50Hz)    │                   │
│             │  + DC Bus│  Sensors)  │            │                   │
│             │ Caps)    │            │            │                   │
└──────┬──────┴────┬─────┴─────┬──────┴─────┬──────┴───────────────────┘
       │           │           │            │
       │  Vpv, Ipv │  mABC     │  Vabc,Iabc │  θ, Vsd, Freq
       ▼           ▲           │            │
┌──────────┐ ┌─────┴────────┐  │   ┌───────┴──────────┐
│   MPPT   │ │  Modulation  │  │   │  PLL & Protection│
│ (P&O or  │ │   Signal     │  │   │  (θ, f, V meas,  │
│  IncCond)│ │ (dq→αβ→mABC) │  │   │   trip logic)    │
└────┬─────┘ └──────▲───────┘  │   └──────────────────┘
     │               │          │
     │ Vmpp          │ Vd*, Vq* │
     └───┬───────────┘          │
         ▼                      │
┌──────────────────────────────┐│
│        Controller (dq)       ││
│  • Voltage PI (outer loop)   ││
│  • d-axis Current PI         │◄── Vabc, Iabc (dq transformed)
│  • q-axis Current PI (UPF)   ││
│  • dq Decoupling (ωL)        ││
└──────────────────────────────┘
```

The control strategy uses cascaded PI loops in the synchronous (dq) reference frame:
1. **Outer voltage loop** — Regulates DC bus voltage to the MPPT reference (Vmpp), produces d-axis current reference (Idref).
2. **Inner current loops** — Regulate d-axis and q-axis currents independently. q-axis reference is set to zero for unity power factor.
3. **dq Decoupling** — Cross-coupling terms (ωL·Id, ωL·Iq) compensate for the inductive coupling between d and q axes.

---

## Key Features

### MPPT Algorithms
Two Maximum Power Point Tracking techniques are implemented as variant subsystems:

| Algorithm | Variable Setting | Description |
|---|---|---|
| **Perturbation & Observation (P&O)** | `MPPT = 0` | Perturbs voltage and observes power change; uses logic gates (AND, NOR, OR) to decide direction |
| **Incremental Conductance (IncCond)** | `MPPT = 1` | Compares dI/dV with −I/V at the MPP; uses if-else control logic for step decisions |

- MPPT update period: **5 ms**
- Voltage step size: **1% of Vmpp**
- Search range: 0.8×Vmpp to 1.15×Vmpp

### Inverter Topologies
Three inverter models are available as variant subsystems:

| Inverter Type | Variable Setting | Use Case |
|---|---|---|
| **Average Model** | `powerCircuit = 0` | Fast simulation, linearization / stability analysis |
| **Two-Level** | `powerCircuit = 1` | Standard switching model with IGBTs |
| **Three-Level (NPC)** | `powerCircuit = 2` | Higher efficiency, lower THD switching model |

### Protection System
Five protection functions are OR-gated into a latched trip signal:

- **Grid Voltage (GV)** — Under/over voltage (±20%/±10% of peak)
- **Grid Frequency (GF)** — 49.2–50.3 Hz window
- **DC Bus Voltage (DCV)** — Min/max thresholds
- **Grid Over-Current (GOI)** — 120% of rated peak current
- **Short-Circuit Current (GSC)** — 200% of rated peak current

Trip is latched via an SR flip-flop. Protection can be disabled with `protection.enableTripSignal = 0`.

---

## Project Structure

```
GridConnectedPVExample/
├── GridConnectedPV.slx           # Main Simulink model
├── GridConnectedPV.slxc          # Simulink cache file
├── GridConnectedPVData.mlx       # Parameter initialization (MATLAB Live Script)
├── GridConnectedPVExample.m      # Main example runner script
├── GridConnectedPVPlotBode.m     # Linearization & Bode plot generation
├── MPPTCTRL.m                    # Shortcut: set MPPT=0 (P&O) and run
├── MPPTCHANGE.m                  # Shortcut: open MPPT subsystem
├── slprj/                        # Simulink project cache
└── README.md                     # This file
```

| File | Purpose |
|---|---|
| `GridConnectedPVData.mlx` | Computes all system parameters from solar panel datasheet: number of panels, controller gains, protection thresholds. **Must be run before simulation.** |
| `GridConnectedPVExample.m` | Opens the model and scope, loads parameters, runs simulation, then optionally generates Bode plot |
| `GridConnectedPVPlotBode.m` | Sets up the model for linearization, extracts state-space matrices with `linmod`, computes and plots the frequency response of the inner current loop |
| `MPPTCTRL.m` | Convenience script to set `MPPT=0` and simulate |
| `MPPTCHANGE.m` | Convenience script to open the MPPT variant subsystem |

---

## Quick Start

### Prerequisites

- MATLAB R2023b or later
- Simulink
- Simscape Electrical™
- (Optional) Simulink Control Design™ — for interactive linearization

### Running the Simulation

1. **Open MATLAB** and navigate to the project directory.

2. **Run the main example script:**

   ```matlab
   GridConnectedPVExample
   ```

   This will:
   - Open the Simulink model and scope
   - Load all parameters from `GridConnectedPVData`
   - Run the simulation (default: 1.4 s)
   - Display PV voltage, current, power, and grid voltage/current waveforms
   - Generate an open-loop Bode plot for stability analysis

3. **To switch MPPT algorithms**, set the workspace variable before running:

   ```matlab
   MPPT = 0;  % Perturbation & Observation
   MPPT = 1;  % Incremental Conductance
   GridConnectedPVData;
   sim('GridConnectedPV');
   ```

4. **To switch inverter topologies**:

   ```matlab
   powerCircuit = 0;  % Average model
   powerCircuit = 1;  % Two-level inverter
   powerCircuit = 2;  % Three-level inverter
   GridConnectedPVData;
   sim('GridConnectedPV');
   ```

5. **To generate Bode plot only**:

   ```matlab
   GridConnectedPVPlotBode
   ```

---

## Model Components

### Solar Plant

The solar plant models parallel-connected strings of solar panels using the **Solar Cell** block from Simscape Electrical. To reduce simulation complexity:

- Uniform irradiance and temperature are assumed across all panels.
- Controlled current/voltage sources aggregate multiple panels into a single equivalent source.
- Parasitic capacitance from the solar panels is modeled with two lumped capacitors, which is critical for accurate common-mode behavior in transformerless topologies.

The number of series panels per string and parallel strings is calculated based on:
- Grid voltage and allowable fluctuation (±10%)
- Voltage drop across the line inductor
- Temperature-dependent open-circuit voltage
- Target plant power rating

### Power Circuit & Inverter

Contains:
- **DC bus capacitors** — Sized based on energy storage requirements during grid disturbances
- **Parasitic capacitance** — Lumped model of PV panel parasitic capacitance to ground
- **DC voltage and current sensors**
- **Variant inverter subsystem** — Selectable between average-model, two-level, and three-level topologies

### Controller (dq-Frame)

Cascaded PI control in the synchronous reference frame:

1. **Voltage Controller (Outer Loop)**
   - PI with anti-windup integrator
   - Kp = `controller.voltageGain`
   - Ki = `1 / controller.voltagePole`
   - Saturation: `1.3 × peak AC current × 1.5`

2. **Current Controllers (Inner Loop, d and q axes)**
   - Identical PI structure for both axes
   - Kp = `controller.currentGain`
   - Ki = `1 / controller.currentPole`
   - Pole placement: PI zero cancels the plant pole (line time constant)
   - Gain: tuned for a target bandwidth considering inverter delay and sensor filtering
   - dq decoupling with ωL feedforward terms

3. **Unity Power Factor**: Iqref = 0 ensures reactive power is zero.

### MPPT Algorithms

Both MPPT methods use the PV array voltage (Vpv) and current (Ipv) measurements:

- **P&O**: Periodically perturbs Vpv by a small step (±ΔV) and compares the resulting power. If power increases, perturbation continues in the same direction; otherwise it reverses.
- **Incremental Conductance**: Uses the fact that dP/dV = 0 at the MPP, which implies dI/dV = −I/V. Compares instantaneous conductance (I/V) with incremental conductance (ΔI/ΔV) to determine the direction to move.

### PLL & Grid Protection

- **Three-Phase PLL**: Synchronizes the controller dq reference frame to the grid voltage. PI-tuned with Kp = 0.05, Ki = 1. Outputs grid angle (θ), frequency, and Vsd (3/2 × phase voltage magnitude).
- **Protection System**: Monitors five fault conditions. Any fault sets an SR flip-flop that opens the three-phase circuit breaker. Once tripped, the breaker remains open (latched) until the simulation is reset.

### Modulation & Gating

- dq-to-αβ transformation
- Normalization by DC bus voltage (Vdc/2)
- Saturation to [−1, 1] to prevent over-modulation
- Output: three-phase modulation signals (mABC) to the inverter

---

## Variant Configuration

The model uses Simulink **Variant Subsystems** controlled by workspace variables:

| Variant | Workspace Variable | Options |
|---|---|---|
| **MPPT Algorithm** | `MPPT` | `0` = P&O, `1` = Incremental Conductance |
| **Inverter Model** | `powerCircuit` | `0` = Average, `1` = Two-Level, `2` = Three-Level |
| **Control Loop** | `closedLoop` | `0` = Open-loop (linearization), `1` = Closed-loop (normal) |
| **Protection** | `protection.enableTripSignal` | `0` = Disabled, `1` = Enabled |

---

## Linearization & Bode Plot

The script `GridConnectedPVPlotBode.m` performs:

1. Sets up the model for linearization:
   - MPPT = 1 (incremental conductance)
   - powerCircuit = 0 (average inverter model)
   - Protection disabled
   - Simulation time reduced to 20/frequency

2. Runs a closed-loop simulation to find the steady-state operating point at t ≈ 200 ms

3. Breaks the feedback loop (`closedLoop = 0`)

4. Uses `linmod` to extract state-space matrices **(A, B, C, D)** from the Simscape model

5. Computes the frequency response over **1 Hz to 100 kHz** (1500 points)

6. Plots:
   - **Magnitude (dB)** vs frequency
   - **Phase (degrees)** vs frequency

7. Restores original variant settings (saves and restores MPPT variant, inverter variant, protection flag, and simulation time)

> **Tip**: If you have Simulink Control Design™, use **APPS → Control System → Model Linearizer** for an interactive workflow with operating point management.

---

## Key Parameters

### Solar Panel (STC Datasheet)
| Parameter | Value |
|---|---|
| Short-circuit current (Isc) | 8.18 A |
| Open-circuit voltage (Voc) | 36.7 V |
| Max power voltage (Vmp) | 29.9 V |
| Max power current (Imp) | 7.53 A |
| Cells per panel | 60 |
| Diode ideality factor | 1.5 |
| Isc temperature coefficient | −0.04%/°C |
| Voc temperature coefficient | −0.32%/°C |

### Controller Tuning
| Parameter | Formula / Basis |
|---|---|
| Current PI zero | `controller.currentPole` = Line time constant (pole-zero cancellation) |
| Current PI gain | Tuned for target bandwidth with inverter delay + sensor filter |
| Voltage PI gain | `C_dc / (0.5 × √(ω_c × ω_v) × ω_LP)` |
| Voltage PI pole | `√(ω_c × ω_v)² × ω_LP` |
| Sensor sampling | 1 MHz (50 × switching frequency) |
| Inverter time constant | 25 μs (1 / 2×fsw) |

### Protection Thresholds
| Parameter | Value |
|---|---|
| Grid voltage min | 0.8 × Vpeak |
| Grid voltage max | 1.1 × Vpeak |
| Grid frequency min | 49.2 Hz |
| Grid frequency max | 50.3 Hz |
| Overload current | 1.2 × Ipeak_max |
| Short-circuit current | 2.0 × Ipeak_max |

---

## Requirements

| Software | Version |
|---|---|
| MATLAB | R2023b or later |
| Simulink | R2023b or later |
| Simscape Electrical | Required (Solar Cell block, power electronics) |
| Simulink Control Design | Optional (recommended for interactive linearization) |

---

## References

- **Solar Cell (Simscape Electrical)**: [MathWorks Documentation](https://www.mathworks.com/help/sps/powersys/ref/solarcell.html)
- **Three-Phase Grid-Connected Solar PV System**: [MathWorks Example](https://www.mathworks.com/help/sps/ug/three-phase-grid-connected-solar-photovoltaic-system.html)
- **Average-Value Inverter**: Suitable for control design and linearization studies
- **Transformerless PV Systems**: Leakage current mitigation through parasitic capacitance modeling

---

*Copyright 2019–2023 The MathWorks, Inc.*
