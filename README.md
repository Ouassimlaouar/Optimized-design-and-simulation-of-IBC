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


