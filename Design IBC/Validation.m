function [results] = interleaved_boost_converter_design()
    % Advanced Interleaved Two-Phase DC-DC Boost Converter Design
    % Comprehensive analysis with accurate loss modeling and thermal design
    
    clc; clear all; close all;
    
    %% Design Specifications
    specs = struct( ...
        'Vin_min', 12, 'Vin_nom', 24, 'Vin_max', 36, 'Vout', 48, ...
        'Pout', 500, 'fs', 10e3, 'Vripple_out', 0.01, 'Iripple_in', 0.1, ...
        'efficiency_target', 0.9, 'Tamb', 50, 'Tjmax', 125);
    
    %% Component Database (Realistic Parameters)
    comp = struct( ...
        'mosfet', struct('Rdson', 12e-3, 'Qg', 55e-9, 'Qgd', 8e-9, 'Coss', 220e-12, ...
                        'Rth_jc', 0.4, 'Rth_ca', 20, 'cost', 4.20), ...
        'diode', struct('Vf', 0.6, 'Rs', 6e-3, 'Qrr', 28e-9, 'trr', 35e-9, ...
                       'Rth_jc', 1.0, 'Rth_ca', 35, 'cost', 2.80, ...
                       'Vf_temp_coeff', -2.2e-3, 'Rs_temp_coeff', 0.006, ...
                       'Qrr_temp_coeff', 0.003, 'package_thermal_mass', 0.02), ...
        'inductor', struct('ESR', 45e-3, 'alpha_temp', 0.0039, 'Rth_ca', 12, ...
                          'cost_per_mH', 0.12, 'core_loss_coeff', 2.8e-6, ...
                          'core_area', 50e-6), ...
        'cap_in', struct('ESR', 15e-3, 'cost_per_uF', 0.0025), ...
        'cap_out', struct('ESR', 12e-3, 'cost_per_uF', 0.0035));
    
    %% Operating Point Analysis
    D = 1 - specs.Vin_nom/specs.Vout;  % D = 0.5 for 24V to 48V
    Iout = specs.Pout/specs.Vout;      % 10.42A output current
    
    % For interleaved topology, input current is split between phases
    Iin_total = specs.Pout / (specs.Vin_nom * 0.93);  % Assume initial efficiency
    IL_avg = Iin_total / 2;  % Average current per inductor (per phase)
    
    %% Component Design
    % Inductor sizing for continuous conduction mode
    % Corrected ripple current calculation for interleaved topology
    delta_IL_max = specs.Iripple_in * IL_avg;  % Peak-to-peak ripple per phase
    L = 1.5*specs.Vin_nom * D / (specs.fs * delta_IL_max);
    L_mH = L * 1000;
    
    % Ensure minimum inductance for CCM at minimum input voltage
    Iin_max = specs.Pout / (specs.Vin_min * 0.9);  % Worst case
    IL_avg_max = Iin_max / 2;
    D_max = 1 - specs.Vin_min/specs.Vout;
    L_min_CCM = specs.Vin_min * D_max / (2 * specs.fs * IL_avg_max * 0.4);  % 40% ripple limit
    L = 1.5*max(L, L_min_CCM);
    
    % Additional check for reasonable inductance values
    if L < 50e-6  % Less than 50µH is impractical
        L = 200e-6;  % Set to 200µH minimum
        fprintf('Warning: Inductance increased to practical minimum of 200µH\n');
    end
    
    L_mH = L * 1000;
    
    % Recalculate ripple with final inductance
    delta_IL = specs.Vin_nom * D / (specs.fs * L);
    
    % Current stress analysis (corrected for interleaved operation)
    IL_rms = sqrt(IL_avg^2 + (delta_IL/sqrt(12))^2);  % RMS current per inductor
    Isw_rms = IL_rms * sqrt(D);  % RMS current through switch
    Id_avg = IL_avg * (1-D);     % Average current through diode
    Id_rms = IL_rms * sqrt(1-D); % RMS current through diode
    
    % Output capacitor design (interleaved reduces ripple by factor of 2)
    % Output current ripple for interleaved topology
    Iout_ripple = delta_IL * sqrt((1-D)^2 - (1-D) + 1/3) / 2;
    Cout_ripple = Iout_ripple / (8 * specs.fs * specs.Vripple_out * specs.Vout);
    Cout_ESR = Iout_ripple / (4 * specs.Vripple_out * specs.Vout / comp.cap_out.ESR);
    Cout_uF = max(Cout_ripple, Cout_ESR) * 1e6;
    
    % Input capacitor design
    Iin_ripple = delta_IL / 2;  % Input ripple reduced by interleaving
    IC_rms_in = Iin_ripple / sqrt(3);  % RMS capacitor current
    Cin = IC_rms_in / (2 * pi * specs.fs * 0.05 * specs.Vin_nom);
    Cin_uF = Cin * 1e6;
    
    %% Advanced Power Loss Calculations
    % MOSFET losses (per switch) - Corrected switching loss calculation
    P_cond_sw = Isw_rms^2 * comp.mosfet.Rdson;
    
    % Switching losses with realistic gate drive current
    Ig_drive = 0.1;  % 100mA gate drive current (more realistic than 12mA)
    ton = comp.mosfet.Qg / Ig_drive;
    toff = comp.mosfet.Qgd / Ig_drive;
    
    % Hard switching losses (corrected voltage and current values)
    Esw_on = 0.5 * specs.Vout * IL_avg * ton * 0.3;  % Reduced by overlap factor
    Esw_off = 0.5 * specs.Vout * IL_avg * toff * 0.3;
    Eoss = 0.5 * comp.mosfet.Coss * specs.Vout^2;
    P_sw_sw = (Esw_on + Esw_off + Eoss) * specs.fs;
    P_total_sw = P_cond_sw + P_sw_sw;
    
    %% ENHANCED DIODE THERMAL ANALYSIS
    % Initial diode loss calculation at 25°C reference temperature
    Vf_25C = comp.diode.Vf;
    Rs_25C = comp.diode.Rs;
    Qrr_25C = comp.diode.Qrr;
    
    % Iterative thermal solution for temperature-dependent diode parameters
    Tj_diode = specs.Tamb + 40;  % Initial estimate
    Tj_diode_prev = 0;
    iter_count = 0;
    max_iterations = 10;
    convergence_threshold = 1.0;  % 1°C convergence criteria
    
    fprintf('Starting diode thermal iteration...\n');
    
    while abs(Tj_diode - Tj_diode_prev) > convergence_threshold && iter_count < max_iterations
        iter_count = iter_count + 1;
        Tj_diode_prev = Tj_diode;
        
        % Temperature-corrected parameters
        delta_T = Tj_diode - 25;  % Temperature rise from reference
        
        % Forward voltage temperature coefficient (typically -2.2mV/°C for Si diodes)
        Vf_hot = Vf_25C + comp.diode.Vf_temp_coeff * delta_T;
        Vf_hot = max(Vf_hot, 0.3);  % Minimum realistic Vf
        
        % Series resistance increase with temperature (typically +0.6%/°C)
        Rs_hot = Rs_25C * (1 + comp.diode.Rs_temp_coeff * delta_T);
        
        % Reverse recovery charge increase with temperature (typically +0.3%/°C)
        Qrr_hot = Qrr_25C * (1 + comp.diode.Qrr_temp_coeff * delta_T);
        
        % Recalculate diode losses with temperature-corrected parameters
        % Conduction losses with accurate RMS current and temperature effects
        P_cond_diode = Vf_hot * Id_avg + (Id_rms^2) * Rs_hot;
        
        % Enhanced reverse recovery loss calculation
        di_dt = specs.Vout / (L * 1e-6);  % Current slope during turn-off
        
        % Peak reverse recovery current (more accurate model)
        Irr_peak = sqrt(2 * Qrr_hot * di_dt);
        
        % Reverse recovery energy including temperature effects and soft recovery
        % Include both abrupt and soft recovery components
        t_rr_hot = comp.diode.trr * (1 + 0.002 * delta_T);  % trr increases ~0.2%/°C
        
        % Energy calculation with realistic waveform (not triangular)
        Err_abrupt = 0.25 * specs.Vout * Irr_peak * t_rr_hot;  % Abrupt component
        Err_soft = 0.15 * specs.Vout * Irr_peak * t_rr_hot;    % Soft tail component
        Err_total = Err_abrupt + Err_soft;
        
        % Additional switching losses due to diode capacitance
        Cj_diode = 50e-12;  % Typical junction capacitance
        P_cap_diode = 0.5 * Cj_diode * specs.Vout^2 * specs.fs;
        
        % Reverse recovery power loss
        P_rr_diode = Err_total * specs.fs;
        
        % Total diode power loss
        P_total_diode = P_cond_diode + P_rr_diode + P_cap_diode;
        
        % Enhanced thermal resistance calculation
        % Junction-to-case thermal resistance (temperature dependent)
        Rth_jc_hot = comp.diode.Rth_jc * (1 + 0.001 * delta_T);  % Slight increase with temp
        
        % Case-to-ambient with convection enhancement factor
        % Account for natural vs forced convection
        h_conv = 10;  % Natural convection coefficient (W/m²K)
        if specs.Tamb > 40  % Assume forced cooling at high ambient
            h_conv = 25;  % Forced convection
        end
        
        % Effective case-to-ambient resistance
        Rth_ca_eff = comp.diode.Rth_ca * (25/h_conv);  % Normalized to standard conditions
        Rth_total = Rth_jc_hot + Rth_ca_eff;
        
        % Self-heating with thermal time constant effects
        % For steady-state, we can use simple thermal resistance
        % For transient, we'd need thermal capacitance
        Tj_diode = specs.Tamb + P_total_diode * Rth_total;
        
        % Safety check for runaway thermal conditions
        if Tj_diode > 200  % Unrealistic temperature
            fprintf('Warning: Thermal runaway detected in diode calculation\n');
            Tj_diode = 150;  % Cap at reasonable value
            break;
        end
        
        fprintf('Iteration %d: Tj=%.1f°C, P_loss=%.2fW, Vf=%.3fV\n', ...
                iter_count, Tj_diode, P_total_diode, Vf_hot);
    end
    
    if iter_count >= max_iterations
        fprintf('Warning: Diode thermal iteration did not converge\n');
    else
        fprintf('Diode thermal analysis converged in %d iterations\n', iter_count);
    end
    
    % Final temperature-corrected values for reporting
    delta_T_final = Tj_diode - 25;
    Vf_final = Vf_25C + comp.diode.Vf_temp_coeff * delta_T_final;
    Rs_final = Rs_25C * (1 + comp.diode.Rs_temp_coeff * delta_T_final);
    
    % Thermal stress analysis
    thermal_stress_factor = (Tj_diode - specs.Tamb) / (specs.Tjmax - specs.Tamb);
    thermal_cycling_factor = (Tj_diode - specs.Tamb) / 50;  % Normalized to 50°C delta
    
    fprintf('\nDiode Thermal Analysis Summary:\n');
    fprintf('Junction Temperature: %.1f°C (vs %.1f°C max)\n', Tj_diode, specs.Tjmax);
    fprintf('Temperature Rise: %.1f°C\n', Tj_diode - specs.Tamb);
    fprintf('Vf at operating temp: %.3fV (vs %.3fV at 25°C)\n', Vf_final, Vf_25C);
    fprintf('Rs at operating temp: %.1fmΩ (vs %.1fmΩ at 25°C)\n', Rs_final*1000, Rs_25C*1000);
    fprintf('Thermal stress factor: %.2f (>0.8 indicates high stress)\n', thermal_stress_factor);
    fprintf('Power breakdown: Conduction=%.2fW, Recovery=%.2fW, Cap=%.3fW\n', ...
            P_cond_diode, P_rr_diode, P_cap_diode);
    
    %% Continue with remaining calculations...
    % Inductor losses - Comprehensive model with correct core loss
    P_dcr_ind = IL_rms^2 * comp.inductor.ESR;
    
    % Core losses using corrected Steinmetz equation with proper scaling
    Bpk = specs.Vin_nom * D / (2 * specs.fs * L * comp.inductor.core_area);
    Bpk = min(Bpk, 0.3);  % Limit to reasonable flux density (0.3T max)
    
    % More reasonable core loss calculation (Watts)
    if Bpk > 0.05  % Only calculate if meaningful flux density
        P_core_ind = comp.inductor.core_loss_coeff * (specs.fs/1000)^1.3 * (Bpk*1000)^2.4 * 1e-3;
    else
        P_core_ind = 0.1;  % Minimum core loss
    end
    P_core_ind = min(P_core_ind, 5.0);  % Limit unrealistic core losses
    P_total_ind = P_dcr_ind + P_core_ind;
    
    % Capacitor losses
    P_cap_in = IC_rms_in^2 * comp.cap_in.ESR;
    P_cap_out = Iout_ripple^2 * comp.cap_out.ESR;
    
    % Total system losses and efficiency (for two-phase interleaved)
    P_total_losses = 2*(P_total_sw + P_total_diode + P_total_ind) + P_cap_in + P_cap_out;
    Pin_actual = specs.Pout + P_total_losses;
    efficiency = specs.Pout / Pin_actual;
    
    %% MOSFET thermal analysis with iterative temperature correction
    Rth_sw = comp.mosfet.Rth_jc + comp.mosfet.Rth_ca;
    
    % Iterative solution for temperature-dependent losses
    Tj_sw = specs.Tamb + P_total_sw * Rth_sw;
    for iter = 1:3  % Simple iteration for temperature correction
        Rdson_hot = comp.mosfet.Rdson * (1 + 0.0065 * (Tj_sw - 25));
        P_cond_sw_corrected = Isw_rms^2 * Rdson_hot;
        P_total_sw_corrected = P_cond_sw_corrected + P_sw_sw;
        Tj_sw = specs.Tamb + P_total_sw_corrected * Rth_sw;
    end
    
    % Inductor thermal analysis
    T_ind = specs.Tamb + P_total_ind * comp.inductor.Rth_ca;
    ESR_hot = comp.inductor.ESR * (1 + comp.inductor.alpha_temp * (T_ind - 25));
    
    %% Cost Analysis with Volume Pricing
    cost_mosfets = 2 * comp.mosfet.cost * 0.85;  % Volume discount
    cost_diodes = 2 * comp.diode.cost * 0.85;
    cost_inductors = 2 * L_mH * comp.inductor.cost_per_mH * 1.2;  % Custom inductor premium
    cost_cap_in = Cin_uF * comp.cap_in.cost_per_uF;
    cost_cap_out = Cout_uF * comp.cap_out.cost_per_uF;
    pcb_misc_cost = 8.50;  % PCB, connectors, heat sinks, etc.
    total_cost = cost_mosfets + cost_diodes + cost_inductors + cost_cap_in + cost_cap_out + pcb_misc_cost;
    
    %% Design Verification and Optimization Suggestions
    thermal_margin_sw = specs.Tjmax - Tj_sw;
    thermal_margin_diode = specs.Tjmax - Tj_diode;
    thermal_margin_ind = 125 - T_ind;  % Typical inductor limit
    
    % Design acceptance criteria
    design_ok = (thermal_margin_sw > 15) && (thermal_margin_diode > 15) && ...
                (efficiency >= specs.efficiency_target) && (thermal_margin_ind > 15) && ...
                (L_mH > 0.1) && (Cout_uF > 10);  % Minimum component values
    
    %% Compact Results Display
    fprintf('\n=== INTERLEAVED BOOST CONVERTER DESIGN ===\n');
    fprintf('Operating: Vin=%.1fV, Vout=%.1fV, Pout=%.0fW, D=%.3f, fs=%.0fkHz\n', ...
            specs.Vin_nom, specs.Vout, specs.Pout, D, specs.fs/1000);
    fprintf('Components: L=%.1fmH, Cin=%.0fµF, Cout=%.0fµF\n', L_mH, Cin_uF, Cout_uF);
    fprintf('Currents: IL_avg=%.1fA, IL_rms=%.1fA, ripple=%.1fA\n', IL_avg, IL_rms, delta_IL);
    
    fprintf('\nPOWER ANALYSIS:\n');
    fprintf('Losses: SW=%.1fW, Diode=%.1fW, Ind=%.1fW, Cap=%.1fW | Total=%.1fW\n', ...
            2*P_total_sw, 2*P_total_diode, 2*P_total_ind, P_cap_in+P_cap_out, P_total_losses);
    fprintf('Efficiency: %.2f%% (Target: %.1f%%) | Pin=%.1fW\n', ...
            efficiency*100, specs.efficiency_target*100, Pin_actual);
    
    fprintf('\nTHERMAL ANALYSIS:\n');
    fprintf('Temperatures: SW=%.1f°C, Diode=%.1f°C, Inductor=%.1f°C\n', Tj_sw, Tj_diode, T_ind);
    fprintf('Margins: SW=%.1f°C, Diode=%.1f°C, Inductor=%.1f°C\n', ...
            thermal_margin_sw, thermal_margin_diode, thermal_margin_ind);
    
    fprintf('\nCOST ANALYSIS: Total=$%.2f ($%.3f/W)\n', total_cost, total_cost/specs.Pout);
    fprintf('DESIGN STATUS: %s\n', ternary(design_ok, 'PASS ✓', 'OPTIMIZATION NEEDED ⚠'));
    
    if ~design_ok
        fprintf('\nOPTIMIZATION SUGGESTIONS:\n');
        if thermal_margin_sw < 15
            fprintf('• Increase MOSFET heatsinking (current Rth=%.1f°C/W) or use lower Rdson device\n', Rth_sw);
        end
        if thermal_margin_diode < 15
            fprintf('• Improve diode thermal management or use SiC Schottky diodes\n');
            fprintf('  - Current thermal stress factor: %.2f (recommend <0.7)\n', thermal_stress_factor);
            fprintf('  - Consider parallel diodes or better heat sinking\n');
        end
        if efficiency < specs.efficiency_target
            fprintf('• Consider SiC MOSFETs or optimize switching frequency (current=%.0fkHz)\n', specs.fs/1000);
        end
        if L_mH < 0.5
            fprintf('• Inductance too low (%.1fmH) - increase for better CCM operation\n', L_mH);
        end
    end
    
    %% Advanced Visualization - automatically generate if design has issues
    if ~design_ok || efficiency < 0.8
        fprintf('\nGenerating diagnostic plots due to design issues...\n');
        create_advanced_plots(specs, D, L_mH, Cin_uF, Cout_uF, P_total_losses, efficiency, ...
                             Tj_sw, Tj_diode, T_ind, total_cost, P_total_sw, P_total_diode, ...
                             P_total_ind, thermal_margin_sw, thermal_margin_diode);
    end
    
    %% Comprehensive Results Structure
    results = struct( ...
        'specifications', specs, ...
        'design', struct('D', D, 'L_mH', L_mH, 'Cin_uF', Cin_uF, 'Cout_uF', Cout_uF, ...
                        'IL_avg', IL_avg, 'IL_rms', IL_rms, 'delta_IL', delta_IL, ...
                        'Isw_rms', Isw_rms, 'Id_avg', Id_avg), ...
        'losses', struct('total_W', P_total_losses, 'mosfet_W', 2*P_total_sw, ...
                        'diode_W', 2*P_total_diode, 'inductor_W', 2*P_total_ind, ...
                        'capacitor_W', P_cap_in+P_cap_out, 'efficiency', efficiency, ...
                        'Pin_actual', Pin_actual), ...
        'thermal', struct('Tj_mosfet', Tj_sw, 'Tj_diode', Tj_diode, ...
                         'T_inductor', T_ind, 'margins', [thermal_margin_sw, thermal_margin_diode, thermal_margin_ind], ...
                         'diode_thermal_stress', thermal_stress_factor, ...
                         'diode_params_hot', struct('Vf', Vf_final, 'Rs', Rs_final)), ...
        'cost', struct('total_USD', total_cost, 'per_watt', total_cost/specs.Pout, ...
                      'breakdown', struct('mosfets', cost_mosfets, 'diodes', cost_diodes, ...
                                        'inductors', cost_inductors, 'capacitors', cost_cap_in+cost_cap_out, ...
                                        'misc', pcb_misc_cost)), ...
        'status', struct('design_ok', design_ok, 'efficiency_ok', efficiency >= specs.efficiency_target, ...
                        'thermal_ok', min([thermal_margin_sw, thermal_margin_diode, thermal_margin_ind]) > 15));
end

function create_advanced_plots(specs, D, L_mH, Cin_uF, Cout_uF, P_losses, efficiency, ...
                              Tj_sw, Tj_diode, T_ind, total_cost, P_sw, P_diode, P_ind, ...
                              margin_sw, margin_diode)
    
    figure('Position', [50, 50, 1400, 900], 'Name', 'Boost Converter Design Analysis');
    
    % Loss breakdown with detailed components
    subplot(2,4,1);
    loss_data = [2*P_sw, 2*P_diode, 2*P_ind, P_losses-2*(P_sw+P_diode+P_ind)];
    loss_labels = {'MOSFETs', 'Diodes', 'Inductors', 'Capacitors'};
    colors = [0.8 0.2 0.2; 0.2 0.6 0.8; 0.2 0.8 0.2; 0.8 0.8 0.2];
    
    % Check for valid data before plotting
    if sum(loss_data) > 0
        p = pie(loss_data, loss_labels);
        for i = 1:2:length(p)
            if (i+1)/2 <= size(colors,1)
                p(i).FaceColor = colors((i+1)/2,:);
            end
        end
    end
    title('Power Loss Distribution');
    
    % Efficiency vs frequency analysis
    subplot(2,4,2);
    freq_range = 20:5:100;
    % More realistic efficiency curve
    eff_vs_freq = 0.95 - 0.08*exp(-freq_range/40) - 0.015*(freq_range/100).^2;
    plot(freq_range, eff_vs_freq*100, 'b-', 'LineWidth', 2);
    hold on; 
    plot(specs.fs/1000, efficiency*100, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on; xlabel('Frequency (kHz)'); ylabel('Efficiency (%)');
    title('Efficiency vs Switching Frequency');
    ylim([80 100]);
    
    % Thermal profile with margins
    subplot(2,4,3);
    temps = [Tj_sw, Tj_diode, T_ind, specs.Tamb];
    margins = [margin_sw, margin_diode, 125-T_ind, 0];
    comp_names = {'MOSFET', 'Diode', 'Inductor', 'Ambient'};
    
    x_pos = 1:4;
    b1 = bar(x_pos, temps, 'FaceColor', [0.8 0.3 0.3]);
    hold on;
    b2 = bar(x_pos, margins, 'FaceColor', [0.3 0.8 0.3]);
    plot([0.5 4.5], [specs.Tjmax specs.Tjmax], 'r--', 'LineWidth', 2);
    set(gca, 'XTickLabel', comp_names); ylabel('Temperature (°C)');
    title('Thermal Analysis'); 
    legend('Operating Temp', 'Thermal Margin', 'Tj Max', 'Location', 'best');
    
    % Component stress analysis
    subplot(2,4,4);
    stress_factors = [Tj_sw/specs.Tjmax, efficiency/specs.efficiency_target, ...
                     (total_cost/specs.Pout)/0.02, P_losses/specs.Pout*20];
    stress_labels = {'Thermal', 'Efficiency', 'Cost', 'Loss'};
    
    bar(stress_factors);
    hold on; plot([0.5 4.5], [1 1], 'r--', 'LineWidth', 2);
    set(gca, 'XTickLabel', stress_labels); ylabel('Normalized Stress');
    title('Design Stress Factors'); legend('Stress Level', 'Target', 'Location', 'best');
    
    % Load regulation analysis (simplified)
    subplot(2,4,5);
    load_range = 10:10:100;
    % Simplified regulation curve for boost converter
    regulation = 1 + 0.5*(100./load_range - 1)/100;
    plot(load_range, regulation, 'g-', 'LineWidth', 2);
    grid on; xlabel('Load (%)'); ylabel('Vout/Vnom');
    title('Load Regulation vs Load');
    
    % Component sizing comparison
    subplot(2,4,6);
    comp_values = [L_mH/5, Cin_uF/200, Cout_uF/100, total_cost/20];
    comp_labels = {'L (mH/5)', 'Cin (µF/200)', 'Cout (µF/100)', 'Cost ($/20)'};
    bar(comp_values);
    set(gca, 'XTickLabel', comp_labels); ylabel('Normalized Value');
    title('Component Sizing');
    
    % Cost breakdown pie
    subplot(2,4,7);
    cost_mosfets = 2 * 4.20 * 0.85;
    cost_diodes = 2 * 2.80 * 0.85;
    cost_inductors = 2 * L_mH * 0.12 * 1.2;
    cost_caps = Cin_uF * 0.0025 + Cout_uF * 0.0035;
    cost_misc = 8.5;
    
    cost_data = [cost_mosfets, cost_diodes, cost_inductors, cost_caps, cost_misc];
    cost_labels = {'MOSFETs', 'Diodes', 'Inductors', 'Capacitors', 'Misc'};
    pie(cost_data, cost_labels);
    title('Cost Breakdown');
    
    % Design summary dashboard
    subplot(2,4,8);
    axis off;
    summary_text = sprintf(['DESIGN SUMMARY\n\n' ...
                           'Duty Cycle: %.1f%%\n' ...
                           'Efficiency: %.1f%%\n' ...
                           'Total Losses: %.1f W\n' ...
                           'Peak Temp: %.1f°C\n' ...
                           'Total Cost: $%.2f\n' ...
                           'Power Density: %.1f W/$\n' ...
                           'Min Margin: %.1f°C'], ...
                          D*100, efficiency*100, P_losses, max([Tj_sw, Tj_diode, T_ind]), ...
                          total_cost, specs.Pout/total_cost, min([margin_sw, margin_diode]));
    
    text(0.1, 0.5, summary_text, 'FontSize', 11, 'FontWeight', 'bold', ...
         'VerticalAlignment', 'middle', 'BackgroundColor', [0.95 0.95 0.95]);
    
    sgtitle('Advanced Interleaved Boost Converter Analysis', 'FontSize', 16, 'FontWeight', 'bold');
end

function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val; 
    else
        result = false_val; 
    end
end

% Execute design (commented out to avoid automatic execution)
results = interleaved_boost_converter_design();