%% Enhanced N-Phase Interleaved Boost Converter Simulation with Parasitics
% Advanced academic MATLAB simulation for master's thesis
% Includes comprehensive parasitic modeling with enable/disable functionality
% Models ESR, ESL, MOSFET/diode resistances, and real-world non-idealities
% Generalized for N phases with automatic phase shift calculation

clear; clc; close all;

%% Configuration Parameters
N_PHASES = 10;               % Number of interleaved phases (can be changed)
ENABLE_PARASITICS = true;   % Set to false to disable all parasitics for ideal simulation

%% Circuit Parameters
L = 1000e-6;        % Inductance per phase [H]
C = 470e-6;         % Output capacitance [F]
Vin = 12;           % Input voltage [V]
R = 10;             % Load resistance [Ω]
fsw = 10e3;         % Switching frequency [Hz]
D = 0.7;            % Duty cycle
Tsw = 1/fsw;        % Switching period [s]

%% Parasitic Parameters (only applied if ENABLE_PARASITICS = true)
% Inductor parasitics (with slight mismatches for realism)
RL_base = 50e-3;            % Base inductor ESR [Ω]
RL_mismatch = 0.4;         % 5% mismatch factor
RL = RL_base * (1 + RL_mismatch * (rand(N_PHASES,1) - 0.5));  % Per-phase ESR

% Capacitor parasitics
RC = 15e-3;         % Capacitor ESR [Ω]
LC = 10e-9;         % Capacitor ESL [H]

% MOSFET parasitics (with slight mismatches)
RDS_on_base = 8e-3;         % Base MOSFET on-resistance [Ω]
RDS_on_mismatch = 0.2;     % 5% mismatch factor
RDS_on = RDS_on_base * (1 + RDS_on_mismatch * (rand(N_PHASES,1) - 0.5));
Coss = 200e-12;     % MOSFET output capacitance [F]

% Diode parasitics (with slight mismatches)
Vf_base = 0.7;              % Base diode forward voltage [V]
Vf_mismatch = 0.1;         % 2% mismatch factor
Vf = Vf_base * (1 + Vf_mismatch * (rand(N_PHASES,1) - 0.5));
Rd_base = 10e-3;            % Base diode resistance [Ω]
Rd_mismatch = 0.2;         % 5% mismatch factor
Rd = Rd_base * (1 + Rd_mismatch * (rand(N_PHASES,1) - 0.5));

% PCB/wiring parasitics
Rw = 5e-3;          % Wiring resistance [Ω]
Lw = 2e-9;          % Wiring inductance [H]

% Package and thermal effects
T_amb = 25;         % Ambient temperature [°C]
Rth_ja = 40;        % Thermal resistance junction-to-ambient [°C/W]

%% Apply or disable parasitics based on flag
if ~ENABLE_PARASITICS
    RL = zeros(N_PHASES,1); RC = 0; LC = 0;
    RDS_on = zeros(N_PHASES,1); Coss = 0;
    Vf = zeros(N_PHASES,1); Rd = zeros(N_PHASES,1);
    Rw = 0; Lw = 0;
end

%% Calculate Phase Shifts
phase_shift = (0:N_PHASES-1)' * (360/N_PHASES);  % Degrees
phase_shift_rad = phase_shift * pi/180;           % Radians

%% Simulation Parameters
t_sim = 100e-3;     % Simulation time [s]
dt_max = Tsw/500;   % Maximum time step for better accuracy with parasitics

%% Initial Conditions
% Extended state vector: [iL1, iL2, ..., iLN, vC, iLC, diL1_dt_prev, ..., diLN_dt_prev]
Vo_ideal = Vin/(1-D);
iL_steady = Vo_ideal/(R*(1-D)^2);
iL_per_phase = iL_steady/N_PHASES;

% Initialize state vector
x0 = zeros(N_PHASES + 2 + N_PHASES, 1);
x0(1:N_PHASES) = iL_per_phase;              % Individual phase currents
x0(N_PHASES + 1) = Vo_ideal;                % Output capacitor voltage
x0(N_PHASES + 2) = 0;                       % Capacitor ESL current
x0(N_PHASES + 3:end) = 0;                   % Previous derivatives

%% Create parameter structure for passing to ODE function
params.N_PHASES = N_PHASES;
params.L = L; params.C = C; params.Vin = Vin; params.R = R;
params.fsw = fsw; params.D = D; params.ENABLE_PARASITICS = ENABLE_PARASITICS;
params.RL = RL; params.RC = RC; params.LC = LC;
params.RDS_on = RDS_on; params.Coss = Coss;
params.Vf = Vf; params.Rd = Rd;
params.Rw = Rw; params.Lw = Lw; 
params.T_amb = T_amb; params.Rth_ja = Rth_ja;
params.phase_shift_rad = phase_shift_rad;

%% ODE Solver Options
options = odeset('MaxStep', dt_max, 'RelTol', 1e-7, 'AbsTol', 1e-10, ...
                'Stats', 'on', 'OutputFcn', @progress_monitor);

%% Solve ODE System
fprintf('Starting %d-phase interleaved boost converter simulation...\n', N_PHASES);
fprintf('Parasitics: %s\n', ternary(ENABLE_PARASITICS, 'ENABLED', 'DISABLED'));
tic;
[t, x] = ode45(@(t,x) enhanced_n_phase_boost_dynamics(t, x, params), [0 t_sim], x0, options);
sim_time = toc;
fprintf('Simulation completed in %.2f seconds.\n', sim_time);

%% Extract State Variables
iL_phases = x(:,1:N_PHASES);                % Individual phase currents [A]
vC = x(:,N_PHASES + 1);                     % Output capacitor voltage [V]
iLC = x(:,N_PHASES + 2);                    % Capacitor ESL current [A]
iL_total = sum(iL_phases, 2);               % Total input current [A]

%% Calculate Power Losses (if parasitics enabled)
if ENABLE_PARASITICS
    [P_losses, efficiency] = calculate_n_phase_power_losses(t, x, params);
else
    P_losses = struct(); efficiency = 100;
end

%% Generate Enhanced PWM Signals with Dead Time
t_pwm = linspace(0, min(t_sim, 1e-3), 10000);  % High resolution for PWM
dead_time = 100e-9;  % 100ns dead time
[pwm_signals, gate_signals] = generate_n_phase_pwm(t_pwm, params, dead_time);

%% Advanced Plotting with Comprehensive Analysis
create_n_phase_plots(t, iL_phases, vC, iL_total, iLC, t_pwm, pwm_signals, gate_signals, params, P_losses, efficiency);

%% Detailed Performance Analysis
perform_n_phase_analysis(t, iL_phases, vC, iL_total, params, P_losses, efficiency);

%% Frequency Domain Analysis
perform_n_phase_frequency_analysis(t, iL_total, vC, params);

%% Enhanced N-Phase System Dynamics Function
function dxdt = enhanced_n_phase_boost_dynamics(t, x, params)
    % Extract parameters
    N_PHASES = params.N_PHASES;
    L = params.L; C = params.C; Vin = params.Vin; R = params.R;
    fsw = params.fsw; D = params.D; ENABLE_PARASITICS = params.ENABLE_PARASITICS;
    
    % Extract state variables
    iL_phases = x(1:N_PHASES);              % Phase currents
    vC = x(N_PHASES + 1);                   % Capacitor voltage
    iLC = x(N_PHASES + 2);                  % ESL current
    diL_dt_prev = x(N_PHASES + 3:end);      % Previous derivatives
    
    % Generate PWM signals with dead time
    dead_time = 100e-9;
    [pwm_signals, ~] = generate_n_phase_pwm(t, params, dead_time);
    
    % Initialize derivatives
    diL_dt = zeros(N_PHASES, 1);
    
    if ENABLE_PARASITICS
        % Calculate temperature-dependent resistances for each phase
        RDS_on_temp = zeros(N_PHASES, 1);
        for i = 1:N_PHASES
            T_junction = params.T_amb + params.Rth_ja * abs(iL_phases(i)^2 * params.RDS_on(i));
            temp_coeff = 0.004;  % 0.4%/°C temperature coefficient
            RDS_on_temp(i) = params.RDS_on(i) * (1 + temp_coeff * (T_junction - 25));
        end
        
        % Calculate dynamics for each phase
        for i = 1:N_PHASES
            if pwm_signals(i) == 1  % MOSFET ON
                V_drop = iL_phases(i) * (params.RL(i) + RDS_on_temp(i) + params.Rw);
                diL_dt(i) = (Vin - V_drop) / (L + params.Lw);
            else  % MOSFET OFF, diode conducting
                V_drop = iL_phases(i) * (params.RL(i) + params.Rd(i) + params.Rw) + params.Vf(i);
                diL_dt(i) = (Vin - vC - V_drop) / (L + params.Lw);
            end
        end
        
        % Output circuit with ESR and ESL
        iD_total = sum((1 - pwm_signals) .* iL_phases);  % Total diode current
        iR = vC / R;                                     % Load current
        
        % ESL dynamics
        diLC_dt = (vC - iLC * params.RC - (vC + iLC * params.RC)) / params.LC;
        
        % Capacitor voltage dynamics with ESR
        dvC_dt = (iD_total - iR - iLC) / C;
        
    else
        % Ideal dynamics (no parasitics)
        for i = 1:N_PHASES
            if pwm_signals(i) == 1
                diL_dt(i) = Vin / L;
            else
                diL_dt(i) = (Vin - vC) / L;
            end
        end
        
        % Ideal output capacitor dynamics
        iD_total = sum((1 - pwm_signals) .* iL_phases);
        iC = iD_total - vC/R;
        dvC_dt = iC / C;
        
        diLC_dt = 0;
    end
    
    dxdt = [diL_dt; dvC_dt; diLC_dt; diL_dt];
end

%% Enhanced N-Phase PWM Generation with Dead Time
function [pwm_signals, gate_signals] = generate_n_phase_pwm(t, params, dead_time)
    N_PHASES = params.N_PHASES;
    fsw = params.fsw; D = params.D;
    phase_shift_rad = params.phase_shift_rad;
    Tsw = 1/fsw;
    
    if length(t) == 1
        % Single time point
        pwm_signals = zeros(N_PHASES, 1);
        gate_signals = zeros(N_PHASES, 1);
        
        for i = 1:N_PHASES
            % Calculate phase-shifted time
            t_phase = t + phase_shift_rad(i) * Tsw / (2*pi);
            t_mod = mod(t_phase, Tsw);
            
            pwm_ideal = double(t_mod < D * Tsw);
            
            % Apply dead time
            gate_signal = pwm_ideal && (t_mod > dead_time) && (t_mod < D * Tsw - dead_time);
            
            pwm_signals(i) = double(gate_signal);
            gate_signals(i) = gate_signal;
        end
    else
        % Vector input
        pwm_signals = zeros(length(t), N_PHASES);
        gate_signals = zeros(length(t), N_PHASES);
        
        for j = 1:length(t)
            [pwm_temp, gate_temp] = generate_n_phase_pwm(t(j), params, dead_time);
            pwm_signals(j, :) = pwm_temp';
            gate_signals(j, :) = gate_temp';
        end
    end
end

%% N-Phase Power Loss Calculation Function
function [P_losses, efficiency] = calculate_n_phase_power_losses(t, x, params)
    N_PHASES = params.N_PHASES;
    iL_phases = x(:,1:N_PHASES);
    vC = x(:,N_PHASES + 1);
    
    % Calculate RMS currents for loss calculation
    iL_rms = zeros(N_PHASES, 1);
    for i = 1:N_PHASES
        iL_rms(i) = rms(iL_phases(end-1000:end, i));
    end
    vC_avg = mean(vC(end-1000:end));
    iL_total_rms = rms(sum(iL_phases(end-1000:end, :), 2));
    
    % Initialize loss arrays
    P_L = zeros(N_PHASES, 1);       % Inductor losses
    P_MOSFET = zeros(N_PHASES, 1);  % MOSFET conduction losses
    P_diode = zeros(N_PHASES, 1);   % Diode losses
    P_sw = zeros(N_PHASES, 1);      % Switching losses
    
    % Calculate losses for each phase
    for i = 1:N_PHASES
        % Conduction losses
        P_L(i) = iL_rms(i)^2 * params.RL(i);
        P_MOSFET(i) = iL_rms(i)^2 * params.RDS_on(i) * params.D;
        P_diode(i) = iL_rms(i) * params.Vf(i) * (1-params.D) + ...
                     iL_rms(i)^2 * params.Rd(i) * (1-params.D);
        
        % Switching losses (simplified)
        P_sw(i) = 0.5 * params.Coss * vC_avg^2 * params.fsw;
    end
    
    % Capacitor ESR losses
    P_C = iL_total_rms^2 * params.RC;
    
    % Store individual losses
    P_losses.P_L = P_L;
    P_losses.P_MOSFET = P_MOSFET;
    P_losses.P_diode = P_diode;
    P_losses.P_sw = P_sw;
    P_losses.P_C = P_C;
    
    % Total losses
    P_losses.P_L_total = sum(P_L);
    P_losses.P_MOSFET_total = sum(P_MOSFET);
    P_losses.P_diode_total = sum(P_diode);
    P_losses.P_sw_total = sum(P_sw);
    P_losses.P_total = P_losses.P_L_total + P_losses.P_MOSFET_total + ...
                       P_losses.P_diode_total + P_losses.P_sw_total + P_C;
    
    % Efficiency calculation
    P_out = vC_avg^2 / params.R;
    P_in = P_out + P_losses.P_total;
    efficiency = (P_out / P_in) * 100;
end

%% Comprehensive N-Phase Plotting Function
function create_n_phase_plots(t, iL_phases, vC, iL_total, iLC, t_pwm, pwm_signals, gate_signals, params, P_losses, efficiency)
    N_PHASES = params.N_PHASES;
    
    % Create main figure with subplots
    fig1 = figure('Position', [50, 50, 1600, 1200]);
    
    % Plot 1: Individual Phase Currents
    subplot(4,3,1);
    colors = lines(N_PHASES);
    for i = 1:N_PHASES
        plot(t*1000, iL_phases(:,i), 'Color', colors(i,:), 'LineWidth', 1.5); hold on;
    end
    xlabel('Time [ms]'); ylabel('Phase Current [A]');
    title('Individual Phase Inductor Currents');
    legend_labels = arrayfun(@(x) sprintf('Phase %d', x), 1:N_PHASES, 'UniformOutput', false);
    legend(legend_labels, 'Location', 'best');
    grid on; xlim([0, 5]);
    
    % Plot 2: Total Input Current
    subplot(4,3,2);
    plot(t*1000, iL_total, 'k-', 'LineWidth', 1.5);
    xlabel('Time [ms]'); ylabel('Total Current [A]');
    title('Total Input Current');
    grid on; xlim([0, 5]);
    
    % Plot 3: Output Voltage
    subplot(4,3,3);
    plot(t*1000, vC, 'g-', 'LineWidth', 1.5);
    xlabel('Time [ms]'); ylabel('Output Voltage [V]');
    title('Output Capacitor Voltage');
    grid on; xlim([0, 20]);
    
    % Plot 4: PWM Signals
    subplot(4,3,4);
    t_detail = t_pwm(t_pwm <= 300e-6);
    pwm_detail = pwm_signals(t_pwm <= 300e-6, :);
    
    for i = 1:min(N_PHASES, 4)  % Show max 4 phases for clarity
        plot(t_detail*1e6, pwm_detail(:,i) + (i-1)*1.2, 'Color', colors(i,:), 'LineWidth', 2); hold on;
    end
    xlabel('Time [μs]'); ylabel('PWM Signals');
    title(sprintf('PWM Signals (First %d Phases)', min(N_PHASES, 4)));
    ylim([-0.5, N_PHASES*1.2]);
    grid on;
    
    % Plot 5: Current Ripple Analysis
    subplot(4,3,5);
    idx_steady = round(0.8*length(t)):length(t);
    for i = 1:N_PHASES
        plot(t(idx_steady)*1000, iL_phases(idx_steady,i), 'Color', colors(i,:), 'LineWidth', 1); hold on;
    end
    plot(t(idx_steady)*1000, iL_total(idx_steady), 'k-', 'LineWidth', 2);
    xlabel('Time [ms]'); ylabel('Current [A]');
    title('Steady-State Current Ripple');
    grid on;
    
    % Plot 6: Phase Current Balance
    subplot(4,3,6);
    iL_avg = mean(iL_phases(idx_steady, :), 1);
    iL_total_avg = mean(iL_total(idx_steady));
    phase_imbalance = abs(iL_avg - iL_total_avg/N_PHASES) / (iL_total_avg/N_PHASES) * 100;
    
    bar(1:N_PHASES, phase_imbalance);
    xlabel('Phase Number'); ylabel('Current Imbalance [%]');
    title('Phase Current Balance');
    grid on;
    
    % Plot 7: Power Loss Breakdown (if parasitics enabled)
    subplot(4,3,7);
    if params.ENABLE_PARASITICS && ~isempty(P_losses)
        loss_categories = {'Inductors', 'MOSFETs', 'Diodes', 'Switching', 'Capacitor'};
        loss_values = [P_losses.P_L_total, P_losses.P_MOSFET_total, ...
                      P_losses.P_diode_total, P_losses.P_sw_total, P_losses.P_C];
        pie(loss_values, loss_categories);
        title(sprintf('Power Loss Distribution\nTotal: %.2f W', P_losses.P_total));
    else
        text(0.5, 0.5, 'Power Losses\n(Parasitics Disabled)', ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        title('Power Loss Distribution');
    end
    
    % Plot 8: Per-Phase Loss Distribution
    subplot(4,3,8);
    if params.ENABLE_PARASITICS && ~isempty(P_losses)
        P_per_phase = P_losses.P_L + P_losses.P_MOSFET + P_losses.P_diode + P_losses.P_sw;
        bar(1:N_PHASES, P_per_phase);
        xlabel('Phase Number'); ylabel('Power Loss [W]');
        title('Per-Phase Power Losses');
        grid on;
    else
        text(0.5, 0.5, 'Per-Phase Losses\n(Parasitics Disabled)', ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        title('Per-Phase Power Losses');
    end
    
    % Plot 9: Efficiency
    subplot(4,3,9);
    if params.ENABLE_PARASITICS
        text(0.1, 0.7, sprintf('Overall Efficiency'), 'FontSize', 14, 'FontWeight', 'bold');
        text(0.1, 0.5, sprintf('%.2f%%', efficiency), 'FontSize', 24, 'Color', 'blue');
        text(0.1, 0.3, sprintf('Total Losses: %.2f W', P_losses.P_total), 'FontSize', 12);
    else
        text(0.1, 0.5, '100%', 'FontSize', 24, 'Color', 'green');
        text(0.1, 0.3, '(Ideal Case)', 'FontSize', 12);
    end
    title('System Efficiency');
    xlim([0, 1]); ylim([0, 1]); axis off;
    
    % Plot 10: Current Ripple Comparison
    subplot(4,3,10);
    single_phase_ripple = max(iL_phases(idx_steady, 1)) - min(iL_phases(idx_steady, 1));
    total_ripple = max(iL_total(idx_steady)) - min(iL_total(idx_steady));
    ripple_reduction = single_phase_ripple / total_ripple;
    
    bar([1, 2], [single_phase_ripple, total_ripple]);
    set(gca, 'XTickLabel', {'Single Phase', 'N-Phase Total'});
    ylabel('Current Ripple [A]');
    title(sprintf('Ripple Reduction: %.1fx', ripple_reduction));
    grid on;
    
    % Plot 11: Harmonic Content
    subplot(4,3,11);
    effective_freq = N_PHASES * params.fsw;
    text(0.1, 0.8, 'Effective Switching Frequency:', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.6, sprintf('%.1f kHz', effective_freq/1000), 'FontSize', 16, 'Color', 'red');
    text(0.1, 0.4, sprintf('(%dx improvement)', N_PHASES), 'FontSize', 12);
    text(0.1, 0.2, sprintf('Individual fsw: %.1f kHz', params.fsw/1000), 'FontSize', 10);
    xlim([0, 1]); ylim([0, 1]); axis off;
    title('Frequency Multiplication');
    
    % Plot 12: System Summary
    subplot(4,3,12);
    text(0.05, 0.9, sprintf('%d-Phase Interleaved Boost', N_PHASES), 'FontSize', 12, 'FontWeight', 'bold');
    text(0.05, 0.7, sprintf('Input: %.1f V', params.Vin), 'FontSize', 6);
    text(0.05, 0.6, sprintf('Output: %.1f V', mean(vC(idx_steady))), 'FontSize', 6);
    text(0.05, 0.5, sprintf('Duty Cycle: %.1f%%', params.D*100), 'FontSize', 6);
    text(0.05, 0.4, sprintf('Switching Freq: %.1f kHz', params.fsw/1000), 'FontSize', 6);
    text(0.05, 0.3, sprintf('Phase Shift: %.1f°', 360/N_PHASES), 'FontSize', 6);
    text(0.05, 0.2, sprintf('Load: %.1f Ω', params.R), 'FontSize', 6);
    text(0.05, 0.1, sprintf('Parasitics: %s', ternary(params.ENABLE_PARASITICS, 'ON', 'OFF')), 'FontSize', 6);
    xlim([0, 1]); ylim([0, 1]); axis off;
    title('System Parameters');
    
    sgtitle(sprintf('%d-Phase Enhanced Interleaved Boost Converter Analysis\nParasitics: %s', ...
            N_PHASES, ternary(params.ENABLE_PARASITICS, 'ENABLED', 'DISABLED')));
end
%% Detailed N-Phase Performance Analysis Function
function perform_n_phase_analysis(t, iL_phases, vC, iL_total, params, P_losses, efficiency)
    N_PHASES = params.N_PHASES;
    
    fprintf('\n=== ENHANCED %d-Phase Interleaved Boost Converter Analysis ===\n', N_PHASES);
    fprintf('Parasitics: %s\n', ternary(params.ENABLE_PARASITICS, 'ENABLED', 'DISABLED'));
    fprintf('Input Voltage: %.1f V\n', params.Vin);
    fprintf('Switching Frequency: %.1f kHz\n', params.fsw/1000);
    fprintf('Duty Cycle: %.1f%%\n', params.D*100);
    fprintf('Phase Shift: %.1f° per phase\n', 360/N_PHASES);
    
    % Steady-state analysis (last 20% of simulation)
    idx_ss = round(0.8*length(t)):length(t);
    
    fprintf('\n--- Output Performance ---\n');
    vC_avg = mean(vC(idx_ss));
    vC_ripple = max(vC(idx_ss)) - min(vC(idx_ss));
    fprintf('Average Output Voltage: %.3f V\n', vC_avg);
    fprintf('Theoretical Output Voltage: %.3f V\n', params.Vin/(1-params.D));
    fprintf('Voltage Error: %.2f%%\n', abs(vC_avg - params.Vin/(1-params.D))/(params.Vin/(1-params.D))*100);
    fprintf('Output Voltage Ripple (pp): %.3f V (%.2f%%)\n', vC_ripple, vC_ripple/vC_avg*100);
    
    fprintf('\n--- Current Analysis ---\n');
    iL_phase_avg = mean(iL_phases(idx_ss, :), 1);
    iL_total_avg = mean(iL_total(idx_ss));
    iL_phase_ripples = max(iL_phases(idx_ss, :), [], 1) - min(iL_phases(idx_ss, :), [], 1);
    iL_total_ripple = max(iL_total(idx_ss)) - min(iL_total(idx_ss));
    
    fprintf('Total Average Current: %.4f A\n', iL_total_avg);
    fprintf('Average Current per Phase: %.4f A\n', iL_total_avg/N_PHASES);
    
    fprintf('\n--- Phase Current Balance ---\n');
    for i = 1:N_PHASES
        sharing_error = abs(iL_phase_avg(i) - iL_total_avg/N_PHASES) / (iL_total_avg/N_PHASES) * 100;
        fprintf('Phase %d: %.4f A (%.2f%% sharing error)\n', i, iL_phase_avg(i), sharing_error);
    end
    
    fprintf('\n--- Ripple Analysis ---\n');
    avg_phase_ripple = mean(iL_phase_ripples);
    fprintf('Average Phase Current Ripple: %.4f A (%.1f%%)\n', avg_phase_ripple, avg_phase_ripple/mean(iL_phase_avg)*100);
    fprintf('Total Current Ripple: %.4f A (%.1f%%)\n', iL_total_ripple, iL_total_ripple/iL_total_avg*100);
    fprintf('Ripple Reduction Factor: %.1fx\n', avg_phase_ripple/iL_total_ripple);
    
    if params.ENABLE_PARASITICS && ~isempty(P_losses)
        fprintf('\n--- Power and Efficiency Analysis ---\n');
        P_out = vC_avg^2 / params.R;
        P_in = P_out + P_losses.P_total;
        
        fprintf('Output Power: %.3f W\n', P_out);
        fprintf('Input Power: %.3f W\n', P_in);
        fprintf('Total Power Losses: %.3f W\n', P_losses.P_total);
        fprintf('Overall Efficiency: %.2f%%\n', efficiency);
        
        fprintf('\n--- Detailed Loss Breakdown ---\n');
        fprintf('Inductor Losses: %.3f W (%.1f%% of total)\n', P_losses.P_L_total, P_losses.P_L_total/P_losses.P_total*100);
        fprintf('MOSFET Conduction Losses: %.3f W (%.1f%% of total)\n', P_losses.P_MOSFET_total, P_losses.P_MOSFET_total/P_losses.P_total*100);
        fprintf('Diode Losses: %.3f W (%.1f%% of total)\n', P_losses.P_diode_total, P_losses.P_diode_total/P_losses.P_total*100);
        fprintf('Switching Losses: %.3f W (%.1f%% of total)\n', P_losses.P_sw_total, P_losses.P_sw_total/P_losses.P_total*100);
        fprintf('Capacitor ESR Losses: %.3f W (%.1f%% of total)\n', P_losses.P_C, P_losses.P_C/P_losses.P_total*100);
        
        fprintf('\n--- Per-Phase Loss Distribution ---\n');
        for i = 1:N_PHASES
            P_phase_total = P_losses.P_L(i) + P_losses.P_MOSFET(i) + P_losses.P_diode(i) + P_losses.P_sw(i);
            fprintf('Phase %d: %.3f W\n', i, P_phase_total);
        end
    else
        fprintf('\n--- Ideal Performance (No Parasitics) ---\n');
        P_out = vC_avg^2 / params.R;
        fprintf('Output Power: %.3f W\n', P_out);
        fprintf('Efficiency: 100%% (ideal)\n');
    end
    
    fprintf('\n--- Frequency Domain Benefits ---\n');
    effective_freq = N_PHASES * params.fsw;
    fprintf('Individual Phase Switching Frequency: %.1f kHz\n', params.fsw/1000);
    fprintf('Effective Output Frequency: %.1f kHz\n', effective_freq/1000);
    fprintf('Frequency Multiplication Factor: %dx\n', N_PHASES);
    
    fprintf('\n--- Component Stress Analysis ---\n');
    fprintf('Peak Phase Current: %.3f A\n', max(max(iL_phases(idx_ss, :))));
    fprintf('Peak Output Voltage: %.3f V\n', max(vC(idx_ss)));
    fprintf('RMS Current per Phase: %.3f A\n', rms(mean(iL_phases(idx_ss, :), 2)));
    
    fprintf('\n=== Analysis Complete ===\n\n');
end

%% N-Phase Frequency Domain Analysis Function
function perform_n_phase_frequency_analysis(t, iL_total, vC, params)
    N_PHASES = params.N_PHASES;
    
    % Extract steady-state data for FFT
    idx_ss = round(0.8*length(t)):length(t);
    t_ss = t(idx_ss);
    iL_ss = iL_total(idx_ss);
    vC_ss = vC(idx_ss);
    
    % Ensure even number of samples for FFT
    if mod(length(t_ss), 2) == 1
        t_ss = t_ss(1:end-1);
        iL_ss = iL_ss(1:end-1);
        vC_ss = vC_ss(1:end-1);
    end
    
    dt = mean(diff(t_ss));
    fs = 1/dt;
    N = length(t_ss);
    
    % Frequency vector
    f = (0:N/2-1) * fs/N;
    
    % FFT of input current
    iL_fft = fft(iL_ss - mean(iL_ss));
    iL_mag = 2*abs(iL_fft(1:N/2))/N;
    
    % FFT of output voltage
    vC_fft = fft(vC_ss - mean(vC_ss));
    vC_mag = 2*abs(vC_fft(1:N/2))/N;
    
    % Create frequency analysis figure
    figure('Position', [100, 100, 1400, 800]);
    
    % Input current spectrum
    subplot(2,2,1);
    semilogx(f/1000, 20*log10(iL_mag + eps), 'b-', 'LineWidth', 1.5);
    xlabel('Frequency [kHz]'); ylabel('Magnitude [dB]');
    title('Input Current Spectrum');
    grid on;
    xlim([0.1, fs/2000]);
    
    % Mark switching frequency harmonics
    hold on;
    for i = 1:5
        harm_freq = i * params.fsw;
        if harm_freq < fs/2
            xline(harm_freq/1000, 'r--', sprintf('f_{sw}×%d', i), 'LabelHorizontalAlignment', 'center');
        end
    end
    
    % Mark N-phase switching frequency harmonics
    for i = 1:3
        harm_freq = i * N_PHASES * params.fsw;
        if harm_freq < fs/2
            xline(harm_freq/1000, 'g--', sprintf('%d×f_{sw}×%d', N_PHASES, i), 'LabelHorizontalAlignment', 'center');
        end
    end
    
    % Output voltage spectrum
    subplot(2,2,2);
    semilogx(f/1000, 20*log10(vC_mag + eps), 'r-', 'LineWidth', 1.5);
    xlabel('Frequency [kHz]'); ylabel('Magnitude [dB]');
    title('Output Voltage Spectrum');
    grid on;
    xlim([0.1, fs/2000]);
    
    % Mark switching frequency harmonics
    hold on;
    for i = 1:5
        harm_freq = i * params.fsw;
        if harm_freq < fs/2
            xline(harm_freq/1000, 'r--', sprintf('f_{sw}×%d', i), 'LabelHorizontalAlignment', 'center');
        end
    end
    
    % Current ripple vs frequency
    subplot(2,2,3);
    ripple_freqs = [params.fsw, N_PHASES*params.fsw];
    ripple_labels = {'Single Phase', sprintf('%d-Phase', N_PHASES)};
    
    % Find ripple at switching frequencies
    [~, idx1] = min(abs(f - params.fsw));
    [~, idx2] = min(abs(f - N_PHASES*params.fsw));
    
    ripple_mags = [iL_mag(idx1), iL_mag(idx2)];
    
    bar(ripple_freqs/1000, ripple_mags);
    set(gca, 'XTickLabel', ripple_labels);
    xlabel('Configuration'); ylabel('Ripple Magnitude [A]');
    title('Current Ripple Comparison');
    grid on;
    
    % THD calculation
    subplot(2,2,4);
    fundamental_freq = params.fsw * N_PHASES;
    [~, fund_idx] = min(abs(f - fundamental_freq));
    
    % Calculate THD for first 10 harmonics
    fundamental_mag = iL_mag(fund_idx);
    harmonic_mags = zeros(1, 10);
    
    for i = 2:11
        harm_freq = i * fundamental_freq;
        [~, harm_idx] = min(abs(f - harm_freq));
        if harm_idx <= length(iL_mag)
            harmonic_mags(i-1) = iL_mag(harm_idx);
        end
    end
    
    THD = sqrt(sum(harmonic_mags.^2)) / fundamental_mag * 100;
    
    bar(2:11, harmonic_mags);
    xlabel('Harmonic Number'); ylabel('Magnitude [A]');
    title(sprintf('Harmonic Content (THD = %.2f%%)', THD));
    grid on;
    
    sgtitle(sprintf('%d-Phase Interleaved Boost Converter - Frequency Domain Analysis', N_PHASES));
    
    % Print frequency analysis results
    fprintf('\n=== FREQUENCY DOMAIN ANALYSIS ===\n');
    fprintf('Sampling Frequency: %.1f kHz\n', fs/1000);
    fprintf('Fundamental Switching Frequency: %.1f kHz\n', params.fsw/1000);
    fprintf('Effective Switching Frequency: %.1f kHz\n', fundamental_freq/1000);
    fprintf('Current Ripple at fsw: %.4f A\n', ripple_mags(1));
    fprintf('Current Ripple at N×fsw: %.4f A\n', ripple_mags(2));
    fprintf('Ripple Reduction: %.1fx\n', ripple_mags(1)/ripple_mags(2));
    fprintf('Total Harmonic Distortion: %.2f%%\n', THD);
    fprintf('===============================\n\n');
end

%% Progress Monitor Function
function status = progress_monitor(t, y, flag)
    persistent last_update;
    status = 0;
    
    if isempty(last_update)
        last_update = 0;
    end
    
    if strcmp(flag, 'init')
        fprintf('Simulation started...\n');
        last_update = 0;
    elseif strcmp(flag, 'done')
        fprintf('Simulation completed!\n');
    elseif isempty(flag) && (t(end) - last_update) > 0.01  % Update every 10ms
        progress = t(end) / 0.1 * 100;  % Assuming 100ms simulation
        fprintf('Progress: %.1f%% (t = %.1f ms)\n', min(progress, 100), t(end)*1000);
        last_update = t(end);
    end
end

%% Utility Ternary Function
function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end