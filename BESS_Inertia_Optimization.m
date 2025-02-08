% --- Parameters ---
f_nominal = 50;           % Nominal frequency (Hz)
P_mechanical = 1.0;       % Mechanical power (pu)
P_electrical = 0.9;       % Electrical power (pu)
delta_t = 0.01;           % Time step (s)
time = 0:delta_t:10;      % Time vector

% --- BESS Parameters ---
SOC_initial = 0.8;        % Initial State of Charge (SOC, pu)
BESS_capacity = 10;       % BESS capacity (pu)
BESS_efficiency = 0.95;   % BESS efficiency

% --- Simulated Frequency Disturbance ---
frequency = f_nominal - 0.1 * sin(2 * pi * 0.5 * time); % Frequency disturbance

% --- RoCoF Calculation ---
RoCoF = [0, diff(frequency) / delta_t]; % Calculate Rate of Change of Frequency
time_RoCoF = time; % Ensure consistent length with time vector

% --- BESS Response ---
P_BESS = -BESS_capacity * RoCoF * BESS_efficiency; % BESS power support
SOC = SOC_initial + cumsum(P_BESS) * delta_t / BESS_capacity; % SOC dynamics

% --- Polynomial Approximation for Smoothing RoCoF ---
poly_order = 3;                         % Polynomial degree
poly_coeffs = polyfit(time_RoCoF, RoCoF, poly_order); % Fit polynomial
smooth_RoCoF = polyval(poly_coeffs, time_RoCoF);      % Evaluate smoothed RoCoF

% --- Swing Equation for Inertia Estimation with BESS ---
P_imbalance = P_mechanical - P_electrical + P_BESS; % Adjust power imbalance
H_estimated = P_imbalance ./ (smooth_RoCoF + eps); % Estimated inertia constant

% --- Optimization: Particle Swarm Optimization (PSO) ---
objective_function = @(H) mean((P_imbalance - H .* smooth_RoCoF).^2); % Minimize error
options = optimoptions('particleswarm', 'Display', 'iter', 'SwarmSize', 50);
[H_optimized, ~] = particleswarm(objective_function, 1, 0.1, 10, options);

% --- Visualization ---
figure;

% Frequency response
subplot(4, 1, 1);
plot(time, frequency, 'b', 'LineWidth', 1.5);
title('Frequency Response');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
grid on;

% RoCoF (original and smoothed)
subplot(4, 1, 2);
plot(time_RoCoF, RoCoF, 'r--', 'LineWidth', 1);
hold on;
plot(time_RoCoF, smooth_RoCoF, 'k', 'LineWidth', 1.5);
title('Rate of Change of Frequency (RoCoF)');
xlabel('Time (s)');
ylabel('RoCoF (Hz/s)');
legend('Original RoCoF', 'Smoothed RoCoF');
grid on;

% BESS State of Charge
subplot(4, 1, 3);
plot(time, SOC, 'g', 'LineWidth', 1.5);
title('BESS State of Charge');
xlabel('Time (s)');
ylabel('SOC (pu)');
grid on;

% Estimated and Optimized Inertia
subplot(4, 1, 4);
plot(time_RoCoF, H_estimated, 'm', 'LineWidth', 1.5);
hold on;
yline(H_optimized, 'k--', 'LineWidth', 1.5);
title('Estimated and Optimized Inertia');
xlabel('Time (s)');
ylabel('Inertia (pu·s)');
legend('Estimated Inertia', 'Optimized Inertia');
grid on;
