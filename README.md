# Interleaved Two-Phase DC-DC Boost Converter Design and Analysis

This Jupyter notebook implements an **advanced interleaved two-phase DC-DC boost converter design**. The code performs a comprehensive multi-domain analysis, including electrical sizing, power loss estimation, thermal assessment, and cost evaluation, considering realistic component characteristics and temperature dependencies.

### Key Features:

1. **Design Specifications and Operating Point Analysis**
   - Configurable input voltage range (`Vin_min`, `Vin_nom`, `Vin_max`), output voltage (`Vout`), output power (`Pout`), switching frequency (`fs`), ripple limits, and efficiency target.
   - Calculates duty cycle, average and RMS inductor currents, and input/output capacitor sizing for continuous conduction mode (CCM).

2. **Component Database and Realistic Modeling**
   - Includes MOSFET, diode, inductor, and capacitor parameters with temperature-dependent characteristics, series resistances, junction capacitances, thermal resistances, and cost factors.
   - Enables accurate thermal and electrical simulations accounting for interleaving benefits.

3. **Power Loss Calculation**
   - Calculates conduction, switching, and reverse recovery losses for MOSFETs and diodes.
   - Considers realistic gate drive current, hard switching, soft recovery effects, and temperature-dependent loss scaling.
   - Estimates inductor core losses using an enhanced Steinmetz approach and capacitor ESR losses.

4. **Thermal Analysis**
   - Iterative junction temperature calculation for diodes and MOSFETs.
   - Calculates thermal margins and thermal stress factors.
   - Supports forced and natural convection assumptions for realistic thermal modeling.
   - Includes inductor temperature rise and temperature-corrected ESR evaluation.

5. **Efficiency and Cost Evaluation**
   - Computes overall system efficiency from component losses.
   - Performs cost analysis, including volume discounts and miscellaneous PCB/assembly costs.
   - Provides per-watt cost evaluation for design optimization.

6. **Design Verification and Optimization**
   - Checks thermal margins, efficiency, and component sizing against specified targets.
   - Provides suggestions for improving thermal management, efficiency, or CCM operation.
   - Flags designs requiring further optimization.

7. **Advanced Visualization**
   - Generates detailed diagnostic plots including:
     - Power loss breakdown
     - Efficiency versus switching frequency
     - Thermal profiles with margins
     - Component stress and sizing comparison
     - Cost distribution
     - Load regulation behavior
   - Summarizes key design metrics in a dashboard-like display for easy review.


# Multi-Objective Optimization of a 24V → 48V Boost Converter

## 1. Overview

The converter is designed to deliver **100 W of output power** at 48V (≈2.08 A), using a switching duty cycle derived from the input and output voltage ratio:

\[
D = 1 - \frac{V_{in}}{V_{out}}
\]

The optimization framework uses a **genetic algorithm (NSGA-II)** to explore a Pareto-optimal trade-off between conflicting objectives.

## 2. Objectives

The multi-objective optimization targets:

1. **Efficiency** – Minimizing total power losses in MOSFETs, inductors, and capacitors.  
2. **Temperature** – Reducing junction temperature by selecting components with low thermal resistance and power loss.  
3. **Volume** – Minimizing total physical volume of inductors, capacitors, MOSFETs, and magnetic cores.  
4. **Cost** – Minimizing total component cost including MOSFETs, passive components, cores, and wiring.

## 3. Component Databases

The system uses realistic component databases including:

- **MOSFETs**: On-resistance, maximum voltage, gate charge, thermal resistance, cost.  
- **Inductors**: Inductance, DC resistance, saturation current, RMS current, volume, cost.  
- **Capacitors**: Capacitance, ESR, ripple current rating, voltage rating, volume, cost.  
- **Magnetic cores**: Inductance factor (AL), core loss density, volume, cost.  
- **Wire diameters**: To calculate copper losses and thermal constraints.

## 4. System Parameters

- Input voltage: 24V  
- Output voltage: 48V  
- Output power: 100 W  
- Ambient temperature: 25°C  
- Number of phases: 2  

## 5. Optimization Setup

- **Design variables**: Inductor, input capacitor, output capacitor, switching frequency, MOSFET, wire diameter, core selection.  
- **Constraints**: Maximum junction temperature, current stress limits, minimum efficiency requirement.  
- **Methodology**: NSGA-II algorithm with a population of 2000 individuals over 50 generations.

## 6. Analysis

- The Pareto front provides trade-offs between efficiency, thermal performance, volume, and cost.  
- Detailed loss calculations include:
  - Conduction and switching losses of MOSFETs  
  - Copper losses in inductors and wiring  
  - ESR losses in capacitors  
- Solutions can be ranked using weighted sums or designer-preferred priorities.

## 7. Visualization

- 3D Pareto front: Efficiency vs Temperature vs Volume (Color = Cost)  
- 2D plots: Efficiency vs Temperature, Volume vs Cost, Switching frequency distribution  

This structured approach allows **engineers to make informed design decisions** based on realistic component characteristics and system requirements.



# Advanced Interleaved DC-DC Boost Converter Design Suite


## Key Features

- **Modular Function-Based Architecture:** Each module (specification, component design, loss analysis, control design, sensitivity analysis, visualization, and reporting) is implemented as a separate, reusable function to facilitate systematic design workflows.
- **Advanced Component Modeling:** Inductor and capacitor design accounts for current ripple, ESR effects, core losses, saturation limits, and thermal constraints. Semiconductor selection includes conduction and switching losses with thermal derating.
- **Comprehensive Loss Analysis:** Detailed estimation of conduction, switching, core, dielectric, and gate drive losses, providing realistic efficiency prediction under various operating conditions.
- **Control System Design:** Small-signal modeling with PID/PI compensator tuning, frequency response analysis, phase and gain margin calculations, and closed-loop performance assessment.
- **Sensitivity and Monte Carlo Analysis:** Evaluates robustness against component tolerances and parameter variations, enabling reliability-centered design and statistical performance assessment.
- **Visualization Suite:** Automatically generates multi-panel plots including component summaries, loss breakdowns, efficiency curves, Bode plots, waveforms, thermal profiles, sensitivity trends, and Monte Carlo histograms.
- **Automated Reporting:** Generates a detailed text report summarizing specifications, component designs, losses, control performance, and design robustness.

---

## Usage Workflow

1. **Define Specifications:** Input voltage range, output voltage, maximum output power, switching frequency, interleaving phases, performance targets, and environmental conditions.
2. **Component Design:** Compute inductor, capacitor, semiconductor, and magnetic design parameters with safety margins and derating factors.
3. **Loss Analysis:** Calculate MOSFET, diode, core, capacitor, and gate drive losses to estimate total losses and efficiency.
4. **Control Design:** Develop small-signal models, design PID/PI controllers, and assess closed-loop performance metrics (bandwidth, settling time, overshoot).
5. **Sensitivity Analysis:** Perform parameter variation and Monte Carlo simulations to evaluate design robustness against tolerances.
6. **Visualization:** Generate comprehensive plots for all key design and performance metrics.
7. **Reporting:** Export a detailed design report summarizing specifications, calculations, and analysis results.

---

## Scientific and Practical Relevance

The suite enables power electronics engineers and researchers to:

- Design interleaved boost converters with high precision and reliability.
- Optimize component selection and control strategy for efficiency and thermal performance.
- Quantify the impact of parameter variations and tolerances on converter behavior.
- Generate documentation-ready plots and reports for professional or academic purposes.

For additional details on the methodology and applied modeling techniques, please refer to my LinkedIn post: [Mathematical Modeling and Simulation of PV Systems](https://www.linkedin.com/posts/ouassim-laouar_mathematical-modeling-and-simulation-of-pv-activity-7360639178742611969-m4Kx?utm_source=social_share_send&utm_medium=member_desktop_web&rcm=ACoAAD68_YkBN31).



This notebook is intended for academic research, engineering design, and performance optimization in modern DC-DC power conversion systems.
