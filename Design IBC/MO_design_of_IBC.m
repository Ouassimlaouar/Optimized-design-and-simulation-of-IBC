clc; clear;
%% ===================================================================
%%                    POWER CONVERTER MULTI-OBJECTIVE OPTIMIZATION
%% ===================================================================
% Optimizes a 24V->48V boost converter for minimal losses, temperature, and size
% Uses NSGA-II genetic algorithm for Pareto-optimal solutions

%% === COMPONENT DATABASES ===
% MOSFETs: {Name, R_DS(on) [Ω], V_DS_max [V], Qg [C], Rth_ja [°C/W], Cost [$]}
MOSFETs = {
'IRF540N', 0.044, 100, 71e-9, 62, 1.50;
'IRLB8743', 0.003, 55, 6e-9, 35, 2.20;
'IPB017N10N5', 0.0017, 100, 17e-9, 37, 3.50;
'STP55NF06L', 0.018, 60, 36e-9, 62, 1.80;
'IRLB3034', 0.0017, 55, 40e-9, 30, 2.80;
'IRLZ44N', 0.022, 55, 67e-9, 62, 1.20;
'CSD19536KCS', 0.002, 55, 17e-9, 45, 4.20;
'BSC056N06NS3G', 0.0036, 60, 12e-9, 50, 2.60;
'FDP7030BL', 0.0055, 55, 9e-9, 40, 1.90;
'NTMFS5C628NL', 0.0018, 55, 15e-9, 40, 3.80;
'IRFZ44N', 0.028, 55, 67e-9, 62, 1.10;
'IRF3205', 0.008, 55, 110e-9, 62, 1.60;
'STP80NF55', 0.0055, 55, 50e-9, 40, 2.40;
'IPP110N20N3G', 0.0015, 200, 30e-9, 30, 5.20;
'IRF3710', 0.009, 100, 70e-9, 40, 2.10;
'IRLB3036', 0.0017, 55, 20e-9, 35, 2.90;
'FDS6910', 0.015, 55, 40e-9, 62, 1.40;
'IRLZ34N', 0.035, 55, 80e-9, 70, 0.90;
'IRF520', 0.27, 100, 120e-9, 70, 0.80;
'BSC027N08NS5', 0.0027, 80, 25e-9, 40, 3.20;
};

% Power Inductors: {Name, L [H], DCR [Ω], Isat [A], Irms [A], Volume [cm³], Cost [$]}
inductors = {
'WE-PD2 1uH', 1e-6, 0.008, 15, 18, 0.8, 2.50;
'WE-PD2 2.2uH', 2.2e-6, 0.012, 14, 16, 0.9, 2.60;
'WE-PD2 4.7uH', 4.7e-6, 0.018, 12, 14, 1.0, 2.80;
'WE-PD2 10uH', 10e-6, 0.025, 12, 13, 1.2, 3.00;
'WE-PD2 22uH', 22e-6, 0.035, 10, 11, 1.5, 3.20;
'WE-PD2 47uH', 47e-6, 0.055, 8, 9, 2.0, 3.60;
'WE-PD2 100uH', 100e-6, 0.085, 6, 7, 2.8, 4.20;
'Coilcraft XAL7070-222ME', 220e-6, 0.12, 5, 6, 3.5, 5.80;
'Coilcraft XAL7070-471ME', 470e-6, 0.18, 4, 5, 4.2, 6.50;
'Bourns SRR1260-4R7M', 4.7e-6, 0.015, 13, 15, 0.8, 2.40;
'Bourns SRR1260-100M', 100e-6, 0.075, 7, 8, 2.5, 4.00;
'Bourns SRR1260-221M', 220e-6, 0.11, 5.5, 6.5, 3.0, 5.20;
'Bourns SRR1260-471M', 470e-6, 0.16, 4.5, 5.5, 3.8, 6.00;
'Vishay IHLP2525-1R0M', 1e-6, 0.006, 18, 20, 0.6, 2.20;
'Vishay IHLP2525-4R7M', 4.7e-6, 0.015, 14, 16, 0.8, 2.40;
'Vishay IHLP2525-100M', 100e-6, 0.08, 6.5, 7.5, 2.2, 3.80;
'TDK SPM6530T-2R2M', 2.2e-6, 0.01, 16, 18, 0.7, 2.30;
'TDK SPM6530T-100M', 100e-6, 0.07, 7, 8, 2.0, 3.60;
'Murata LQH32CN2R2M', 2.2e-6, 0.012, 15, 17, 0.75, 2.35;
'Murata LQH32CN100K', 100e-6, 0.09, 6, 7, 2.1, 3.70;
};

% Capacitors: {Name, C [F], ESR [Ω], Ripple_I [A], V_max [V], Volume [cm³], Cost [$]}
capacitors = {
'Panasonic 10uF 63V', 10e-6, 0.08, 2.0, 63, 0.15, 0.45;
'Panasonic 22uF 63V', 22e-6, 0.06, 2.5, 63, 0.25, 0.65;
'Panasonic 47uF 63V', 47e-6, 0.05, 3.0, 63, 0.4, 0.85;
'NCC 100uF 63V', 100e-6, 0.05, 3.5, 63, 0.5, 1.20;
'NCC 220uF 63V', 220e-6, 0.04, 4.0, 63, 1.0, 1.80;
'NCC 470uF 63V', 470e-6, 0.035, 5.0, 63, 2.5, 2.50;
'NCC 1000uF 63V', 1000e-6, 0.03, 6.0, 63, 5.0, 3.80;
'Rubycon 2200uF 63V', 2200e-6, 0.025, 8.0, 63, 10.0, 6.20;
'Panasonic 330uF 63V', 330e-6, 0.04, 4.5, 63, 1.5, 2.20;
'NCC 4700uF 63V', 4700e-6, 0.02, 10.0, 63, 12.0, 8.50;
'Rubycon 680uF 63V', 680e-6, 0.03, 5.5, 63, 3.5, 3.20;
'Nichicon 150uF 63V', 150e-6, 0.035, 3.8, 63, 1.2, 1.60;
'Panasonic 220uF 100V', 220e-6, 0.03, 4.2, 100, 2.0, 2.80;
'AVX 10uF 100V-X7R', 10e-6, 0.005, 2.0, 100, 0.1, 1.20;
'Kemet 22uF 63V-X7R', 22e-6, 0.01, 2.5, 63, 0.15, 0.95;
'Murata 47uF 63V-X7R', 47e-6, 0.008, 3.0, 63, 0.2, 1.40;
'TDK 100uF 63V-X7R', 100e-6, 0.006, 3.5, 63, 0.3, 2.20;
'AVX 220uF 63V-X7R', 220e-6, 0.004, 4.0, 63, 0.5, 3.80;
'Kemet 470uF 63V-X7R', 470e-6, 0.003, 5.0, 63, 0.8, 6.50;
'Murata 1000uF 63V-X7R', 1000e-6, 0.002, 6.0, 63, 1.2, 12.00;
};

% Wire diameters [m] (AWG equivalents)
wireDiam = [0.2e-3, 0.25e-3, 0.3e-3, 0.35e-3, 0.4e-3, 0.5e-3, 0.6e-3, 0.7e-3, 0.8e-3, 1.0e-3, 1.2e-3, 1.5e-3, 2.0e-3, 2.5e-3, 3.0e-3, 3.5e-3, 4.0e-3, 4.5e-3, 5.0e-3, 6.0e-3];

% Magnetic cores: {Name, A_L [H/t²], k_loss [W/cm³], alpha, Volume [cm³], Cost [$]}
cores = {
'EPCOS E25/13/7', 300e-9, 0.04, 1.5, 6.5, 3.20;
'EPCOS E30/15/7', 200e-9, 0.05, 1.6, 8.0, 4.50;
'EPCOS E40/20/10', 150e-9, 0.06, 1.7, 15.0, 8.20;
'EPCOS E50/22/14', 100e-9, 0.07, 1.8, 28.0, 15.60;
'Fair-Rite 5943003801', 250e-9, 0.045, 1.5, 7.0, 4.80;
'Fair-Rite 5943003802', 180e-9, 0.05, 1.55, 9.0, 6.20;
'EPCOS E20/10/6', 500e-9, 0.035, 1.4, 3.0, 2.10;
'EPCOS E16/8/5', 700e-9, 0.03, 1.3, 1.5, 1.50;
'EPCOS E12/6/4', 1000e-9, 0.025, 1.2, 0.7, 1.20;
'Micrometals T37-6', 500e-9, 0.035, 1.45, 3.5, 2.80;
'Micrometals T50-2', 200e-9, 0.04, 1.5, 10.0, 5.20;
'Micrometals T68-6', 150e-9, 0.05, 1.6, 15.0, 8.60;
'Micrometals T80-2', 100e-9, 0.06, 1.7, 20.0, 12.40;
'Micrometals T106-2', 50e-9, 0.08, 1.8, 30.0, 18.20;
'Fair-Rite FT50-43', 120e-9, 0.07, 1.75, 18.0, 9.80;
'Fair-Rite FT37-43', 220e-9, 0.05, 1.5, 8.0, 6.40;
'Fair-Rite FT82-61', 80e-9, 0.065, 1.7, 22.0, 14.60;
'Fair-Rite FT140-43', 40e-9, 0.09, 1.85, 40.0, 28.40;
'Fair-Rite FT240-43', 20e-9, 0.12, 2.0, 80.0, 52.00;
'Fair-Rite FT50-61', 150e-9, 0.055, 1.6, 12.0, 7.80;
};

%% === SYSTEM PARAMETERS ===
Vin = 24;           % Input voltage [V]
Vout = 48;          % Output voltage [V]
Pout = 100;         % Output power [W] (48V * 4.8A)
Iout = Pout/Vout;   % Output current [A]
D = 1 - Vin/Vout;   % Duty cycle
Tamb = 25;          % Ambient temperature [°C]
nPhases = 2;        % Single phase converter

%% === OPTIMIZATION SETUP ===
% Variables: [L_idx, Cin_idx, Cout_idx, fsw, MOSFET_idx, wire_idx, core_idx]
nvars = 7;
lb = [1, 1, 1, 10e3, 1, 1, 1];
ub = [size(inductors,1), size(capacitors,1), size(capacitors,1), 50e3, size(MOSFETs,1), length(wireDiam), size(cores,1)];

% Enhanced GA options for better convergence
opts = optimoptions('gamultiobj', ...
    'PopulationSize', 2000, ...
    'MaxGenerations', 50, ...
    'MaxStallGenerations', 200, ...
    'FunctionTolerance', 1e-6, ...
    'DistanceMeasureFcn', {@distancecrowding,'phenotype'}, ...
    'SelectionFcn', {@selectiontournament,2}, ...
    'CrossoverFraction', 0.8, ...
    'MutationFcn', {@mutationadaptfeasible}, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotpareto, @gaplotspread}, ...
    'UseParallel', false);

% Objective function
objFun = @(x) converterObjectives(x, inductors, capacitors, MOSFETs, wireDiam, cores, Vin, Vout, Pout, Tamb);

%% === RUN OPTIMIZATION ===
fprintf('Starting multi-objective optimization...\n');
fprintf('System: %.0fV -> %.0fV, %.0fW (%.1fA)\n', Vin, Vout, Pout, Iout);
fprintf('Variables: L, Cin, Cout, fsw, MOSFET, wire, core\n');
fprintf('Objectives: Efficiency, Temperature, Volume, Cost\n\n');

tic;
[x_opt, fval] = gamultiobj(objFun, nvars, [], [], [], [], lb, ub, opts);
optimization_time = toc;

fprintf('\nOptimization completed in %.1f seconds\n', optimization_time);
fprintf('Found %d Pareto-optimal solutions\n\n', size(x_opt,1));

%% === ANALYZE AND DISPLAY RESULTS ===
num_solutions = min(5, size(x_opt,1)); % Show top 5 solutions

% Sort solutions by total score (weighted sum of normalized objectives)
weights = [0.4, 0.3, 0.2, 0.1]; % Efficiency, Temperature, Volume, Cost
fval_norm = (fval - min(fval)) ./ (max(fval) - min(fval) + eps);
scores = fval_norm * weights';
[~, sort_idx] = sort(scores);

fprintf('=== TOP %d PARETO-OPTIMAL SOLUTIONS ===\n', num_solutions);
fprintf('Objectives: [Efficiency(%%), Temperature(°C), Volume(cm³), Cost($)]\n\n');

for i = 1:num_solutions
    idx = sort_idx(i);
    x = x_opt(idx,:);
    f = fval(idx,:);
    
    % Extract design variables
    L_idx = round(x(1));
    Cin_idx = round(x(2));
    Cout_idx = round(x(3));
    fsw = x(4);
    M_idx = round(x(5));
    w_idx = round(x(6));
    core_idx = round(x(7));
    
    % Get component data
    L_data = inductors(L_idx, :);
    Cin_data = capacitors(Cin_idx, :);
    Cout_data = capacitors(Cout_idx, :);
    M_data = MOSFETs(M_idx, :);
    wire_d = wireDiam(w_idx);
    core_data = cores(core_idx, :);
    
    fprintf('--- SOLUTION %d (Score: %.3f) ---\n', i, scores(idx));
    fprintf('Objectives: [%.2f%%, %.1f°C, %.1fcm³, $%.2f]\n', f(1), f(2), f(3), f(4));
    fprintf('Switching frequency: %.1f kHz\n', fsw/1000);
    fprintf('Inductor: %s (%.1fµH, %.1fmΩ, %.1fA)\n', L_data{1}, L_data{2}*1e6, L_data{3}*1000, L_data{4});
    fprintf('Input Cap: %s (%.0fµF, %.1fmΩ)\n', Cin_data{1}, Cin_data{2}*1e6, Cin_data{3}*1000);
    fprintf('Output Cap: %s (%.0fµF, %.1fmΩ)\n', Cout_data{1}, Cout_data{2}*1e6, Cout_data{3}*1000);
    fprintf('MOSFET: %s (%.1fmΩ, %.0fV, $%.2f)\n', M_data{1}, M_data{2}*1000, M_data{3}, M_data{6});
    fprintf('Wire: %.1fmm diameter\n', wire_d*1000);
    fprintf('Core: %s (%.1fcm³, $%.2f)\n', core_data{1}, core_data{5}, core_data{6});
    
    % Calculate detailed losses
    [detailed_losses, ~] = calculateDetailedLosses(x, inductors, capacitors, MOSFETs, wireDiam, cores, Vin, Vout, Pout, Tamb);
    fprintf('Detailed losses: Cond=%.2fW, SW=%.2fW, L=%.2fW, Cap=%.2fW\n', ...
        detailed_losses.P_conduction, detailed_losses.P_switching, ...
        detailed_losses.P_inductor, detailed_losses.P_capacitor);
    fprintf('Total losses: %.2fW, Efficiency: %.2f%%\n\n', ...
        detailed_losses.P_total, detailed_losses.efficiency);
end

%% === PLOT RESULTS ===
if size(fval,1) > 1
    figure('Name', 'Pareto Front Analysis', 'Position', [100 100 1200 800]);
    
    % 3D Pareto front
    subplot(2,2,1);
    scatter3(100-fval(:,1), fval(:,2), fval(:,3), 60, fval(:,4), 'filled');
    xlabel('Efficiency (%)'); ylabel('Temperature (°C)'); zlabel('Volume (cm³)');
    title('3D Pareto Front (Color = Cost)'); grid on; colorbar;
    
    % Efficiency vs Temperature
    subplot(2,2,2);
    scatter(100-fval(:,1), fval(:,2), 60, fval(:,4), 'filled');
    xlabel('Efficiency (%)'); ylabel('Temperature (°C)');
    title('Efficiency vs Temperature (color=cost)'); grid on; colorbar;
    
    % Volume vs Cost
    subplot(2,2,3);
    scatter(fval(:,3), fval(:,4), 60, 100-fval(:,1), 'filled');
    xlabel('Volume (cm³)'); ylabel('Cost ($)');
    title('Volume vs Cost (Color = Efficiency)'); grid on; colorbar;
    
    % Switching frequency distribution
    subplot(2,2,4);
    histogram(x_opt(:,4)/1000, 20);
    xlabel('Switching Frequency (kHz)'); ylabel('Count');
    title('Switching Frequency Distribution'); grid on;
end

%% === OBJECTIVE FUNCTION ===
function objectives = converterObjectives(x, inductors, capacitors, MOSFETs, wireDiam, cores, Vin, Vout, Pout, Tamb)
    % Extract variables with bounds checking
    L_idx = max(1, min(round(x(1)), size(inductors,1)));
    Cin_idx = max(1, min(round(x(2)), size(capacitors,1)));
    Cout_idx = max(1, min(round(x(3)), size(capacitors,1)));
    fsw = max(20e3, min(x(4), 200e3));
    M_idx = max(1, min(round(x(5)), size(MOSFETs,1)));
    w_idx = max(1, min(round(x(6)), length(wireDiam)));
    core_idx = max(1, min(round(x(7)), size(cores,1)));
    
    % Calculate losses and performance
    [losses, volume] = calculateDetailedLosses([L_idx, Cin_idx, Cout_idx, fsw, M_idx, w_idx, core_idx], ...
        inductors, capacitors, MOSFETs, wireDiam, cores, Vin, Vout, Pout, Tamb);
    
    % Calculate cost
    cost = inductors{L_idx,7} + capacitors{Cin_idx,7} + capacitors{Cout_idx,7} + ...
           MOSFETs{M_idx,6} + cores{core_idx,6} + 2.0; % +$2 for wire, PCB, etc.
    
    % Objectives to minimize (converted appropriately)
    efficiency = losses.efficiency;
    temperature = losses.temperature;
    
    % Convert efficiency to minimization problem (100 - efficiency)
    objectives = [100 - efficiency, temperature, volume, cost];
    
    % Add penalties for constraint violations
    if losses.current_stress > 0.9 % Current > 90% of rating
        objectives = objectives + 1000;
    end
    if temperature > 125 % Junction temperature limit
        objectives = objectives + 1000;
    end
    if efficiency < 70 % Minimum efficiency requirement
        objectives = objectives + 1000;
    end
end

%% === DETAILED LOSS CALCULATION FUNCTION ===
function [losses, volume] = calculateDetailedLosses(x, inductors, capacitors, MOSFETs, wireDiam, cores, Vin, Vout, Pout, Tamb)
    % Extract design parameters
    L_idx = round(x(1)); Cin_idx = round(x(2)); Cout_idx = round(x(3));
    fsw = x(4); M_idx = round(x(5)); w_idx = round(x(6)); core_idx = round(x(7));
    
    % Component parameters
    L = inductors{L_idx, 2}; DCR_L = inductors{L_idx, 3}; Isat = inductors{L_idx, 4};
    Cin = capacitors{Cin_idx, 2}; ESR_Cin = capacitors{Cin_idx, 3};
    Cout = capacitors{Cout_idx, 2}; ESR_Cout = capacitors{Cout_idx, 3};
    Rdson = MOSFETs{M_idx, 2}; Qg = MOSFETs{M_idx, 4}; Rth_ja = MOSFETs{M_idx, 5};
    wire_d = wireDiam(w_idx);
    AL = cores{core_idx, 2}; k_loss = cores{core_idx, 3};
    
    % System calculations
    Iout = Pout / Vout;
    D = 1 - Vin / Vout;
    
    % Current calculations
    IL_avg = Iout / (1 - D);
    dIL = (Vin * D) / (L * fsw);
    IL_rms = sqrt((IL_avg/2)^2 + ((dIL/2)^2)/12);
    IL_peak = IL_avg/2 + dIL/4;
    
    % Capacitor ripple currents
    IC_rms = dIL / (2 * sqrt(3)); % Output cap ripple current
    ICin_rms = IL_rms; % Input cap sees full inductor current
    
    % Loss calculations
    P_conduction = Rdson * IL_rms^2 * D; % MOSFET conduction losses
    P_switching = 0.5 * Vin * IL_avg * Qg * fsw; % Switching losses
    P_inductor = DCR_L * IL_rms^2; % Inductor copper losses
    P_Cout = ESR_Cout * IC_rms^2; % Output capacitor losses
    P_Cin = ESR_Cin * ICin_rms^2; % Input capacitor losses
    
    % Total losses and efficiency
    P_total = P_conduction + P_switching + P_inductor + P_Cout + P_Cin;
    efficiency = (Pout / (Pout + P_total)) * 100;
    
    % Temperature calculation
    temperature = Tamb + (P_conduction + P_switching) * Rth_ja;
    
    % Volume calculation
    volume = inductors{L_idx,6} + capacitors{Cin_idx,6} + capacitors{Cout_idx,6} + ...
             cores{core_idx,5} + 2.0; % +2cm³ for MOSFET and misc components
    
    % Current stress factor
    current_stress = IL_peak / Isat;
    
    % Return structure
    losses = struct('P_conduction', P_conduction, 'P_switching', P_switching, ...
                   'P_inductor', P_inductor, 'P_capacitor', P_Cout + P_Cin, ...
                   'P_total', P_total, 'efficiency', efficiency, ...
                   'temperature', temperature, 'current_stress', current_stress);
end