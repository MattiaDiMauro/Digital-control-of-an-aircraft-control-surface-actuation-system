# Digital Control of an Aircraft Control Surface Actuation System

**Authors:** Michele Crippa, Mattia Di Mauro  
**Course:** Digital Control Technology for Aeronautics (DCTA)  
**Institution:** Politecnico di Milano — MSc Aeronautical Engineering  
**Year:** 2025–2026

---

## Overview

This project develops a **digital cascade control system** for an aircraft control surface
positioning mechanism, modeled as a flexible electromechanical transmission driven by a DC motor.

The controlled variable is the angular position of the output shaft θ₁ ∈ [−60°, +60°],
actuated via motor armature voltage Vₐ. The system is modeled in state-space form with
four states: motor angle, motor angular velocity, load angle, and load angular velocity.

---

## Control Architecture

A **cascade (inner–outer) control structure** is adopted:

- **Inner loop** — velocity controller (PID): regulates θ̇₁, provides disturbance rejection
  and enforces the ±45°/s speed constraint via saturation + anti-windup
- **Outer loop** — position controller (P): regulates θ₁ using the inner loop as a fast
  inner subsystem

Loop bandwidths: ωc,inner = 6.3 rad/s, ωc,outer = 1.3 rad/s  
Sampling time: Ts = 0.02 s (20 ms)

---

## Design Requirements — All Met ✅

| Requirement | Target | Result |
|---|---|---|
| Steady-state position error | < 0.3° | 0° (integral action) |
| Overshoot | < 10% | Verified |
| Output angular velocity | ≤ ±45°/s | Enforced via saturation |
| Load torque disturbance rejection | Full rejection | Verified |
| Actuator voltage | ≤ ±12 V | Enforced via saturation |

---

## Digital Implementation

Controllers discretized via **Tustin (bilinear)** transformation; plant via **ZOH**.

**Signal chain modeled in Simulink:**
- Tachometer (10 mV/°/s) + op-amp conditioning → 9-bit ADC (inner loop)
- Optical encoder (2048 counts/rev) → 11-bit ADC (outer loop)  
- Second-order anti-aliasing filter (ωAA ≈ 52.4 rad/s)
- DAC + Power Amplifier (9-bit, ±12 V output range)
- Computational delay: Ts/2

---

## Repository Contents

| File | Description |
|---|---|
| `MatlabCode_Crippa_DiMauro.m` | MATLAB script: plant analysis, controller design, discretization |
| `Simulink_Crippa_DiMauro.slx` | Simulink hybrid simulation model |
| `DCTA_Report.pdf` | Full project report |

---

## Tools

MATLAB R2024b · Simulink · Control System Toolbox
