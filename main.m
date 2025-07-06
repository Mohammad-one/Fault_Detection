clc; clear; close all;

%% System Configuration
sys = struct();
sys.M = @M_function;        
sys.Vm = @Vm_function;      
sys.G = @G_function;        
sys.beta = @(T, t) double(t >= T);  % fault enable func tion
%sys.M_tilde = @M_tilde_fn;
   
%% Simulation Parameters
sim = struct();
sim.T_final = 10;
sim.dt = 0.05;
sim.t = 0:sim.dt:sim.T_final;
sim.fault_time = 5; 

%% Robot Initial State
state = struct();
state.q = [0; 0];           
state.q_dot = [0; 0];       

%% Desired Trajectory Parameters (from paper)
A = 19.45 * pi/180;   % Amplitude in radians (~0.2 rad)
f = 1/(2*pi);         % Frequency in Hz (~0.159 Hz)
omega = 2 * pi * f;   % Angular frequency in rad/s
trajectory = @(t) sinusoidal_trajectory(t, 11.45 * pi/180, 1/(2*pi));

%% Controller Gains (as per paper)
Kp = 100 * eye(2);    % 2x2 diagonal with 100
Kd = 20 * eye(2);     % 2x2 diagonal with 20

%% Data Logging Initialization
data.q_d_hist = zeros(2, length(sim.t));
data.q_dot_d_hist = zeros(2, length(sim.t));
data.q_ddot_d_hist = zeros(2, length(sim.t));
data.q_hist = zeros(2, length(sim.t));
data.q_dot_hist = zeros(2, length(sim.t));
data.tau_hist = zeros(2, length(sim.t));

%% Main Simulation Loop (with fault term)
for i = 1:length(sim.t)
    t_curr = sim.t(i);
    
    % Desired trajectory and derivatives
    %{
    q_d = A * [sin(omega * t_curr); sin(omega * t_curr)];
    q_dot_d = A * omega * [cos(omega * t_curr); cos(omega * t_curr)];
    q_ddot_d = -A * omega^2 * [sin(omega * t_curr); sin(omega * t_curr)];
    %}
    [q_d, q_dot_d, q_ddot_d] = trajectory(t_curr);

    
    % Store desired trajectory
    data.q_d_hist(:, i) = q_d;
    data.q_dot_d_hist(:, i) = q_dot_d;
    data.q_ddot_d_hist(:, i) = q_ddot_d;
    
    % Compute control torque with current states and desired trajectory
    tau = control_law(state.q, state.q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kd, sys.M, sys.Vm, sys.G);

    % Define faulty inertia matrix function handle for current time
    M_tilde_q = M_tilde_fn(state.q, t_curr, sim.fault_time);

    % Compute fault term based on faulty inertia matrix
    psi = fault_term(state.q, state.q_dot, tau, M_tilde_q, sys.M, sys.Vm, sys.G);
    q_ddot = sys.M(state.q) \ (tau - sys.Vm(state.q, state.q_dot) * state.q_dot - sys.G(state.q)) + psi;
    
    % Compute joint accelerations including fault effect
    %q_ddot = sys.M(state.q) \ (tau - sys.Vm(state.q, state.q_dot) * state.q_dot - sys.G(state.q));
    
    % Store actual states and inputs
    data.q_hist(:, i) = state.q;
    data.q_dot_hist(:, i) = state.q_dot;
    data.tau_hist(:, i) = tau;
    
    % Integrate states using Euler method
    state.q_dot = state.q_dot + q_ddot * sim.dt;
    state.q = state.q + state.q_dot * sim.dt;
end


%% Visualization
% 1. Desired vs Actual Joint Angles (Sinusoidal)
figure('Name', 'Joint Angle Tracking', 'Position', [100 100 800 600]);
subplot(2,1,1);
plot(sim.t, data.q_d_hist(1,:), 'r--', 'LineWidth', 2); 
hold on;
plot(sim.t, data.q_hist(1,:), 'b-', 'LineWidth', 1.5);
xline(sim.fault_time, 'k--', 'Fault Injected', 'LabelVerticalAlignment', 'top');
xlabel('Time (s)'); ylabel('Position (rad)');
title('Joint 1: Sinusoidal Tracking (11.45°, 0.159Hz)');
legend('Desired q₁', 'Actual q₁', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(sim.t, data.q_d_hist(2,:), 'r--', 'LineWidth', 2);
hold on;
plot(sim.t, data.q_hist(2,:), 'b-', 'LineWidth', 1.5);
xline(sim.fault_time, 'k--', 'LabelVerticalAlignment', 'top');
xlabel('Time (s)'); ylabel('Position (rad)');
title('Joint 2: Sinusoidal Tracking (11.45°, 0.159Hz)');
legend('Desired q₂', 'Actual q₂', 'Location', 'best');
grid on;

% 2. Tracking Error Plot
figure('Name', 'Tracking Errors', 'Position', [200 200 800 400]);
q1_error = data.q_d_hist(1,:) - data.q_hist(1,:);
q2_error = data.q_d_hist(2,:) - data.q_hist(2,:);
plot(sim.t, q1_error, 'r-', 'LineWidth', 1.5);
hold on;
plot(sim.t, q2_error, 'b-', 'LineWidth', 1.5);
xline(sim.fault_time, 'k--', 'Fault Injected', 'LabelVerticalAlignment', 'top');
xlabel('Time (s)'); ylabel('Tracking Error (rad)');
title('Joint Angle Tracking Errors');
legend('q₁ error', 'q₂ error', 'Location', 'best');
grid on;

% 3. System Performance Plots
figure('Name', 'System Performance', 'Position', [300 300 800 600]);
subplot(2,1,1);
plot(sim.t, data.q_hist(1,:), 'r-', 'LineWidth', 1.5);
hold on;
plot(sim.t, data.q_hist(2,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position (rad)');
legend('q₁', 'q₂'); title('Joint Positions');
grid on;

subplot(2,1,2);
plot(sim.t, data.tau_hist(1,:), 'r-', 'LineWidth', 1.5);
hold on;
plot(sim.t, data.tau_hist(2,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('τ₁', 'τ₂'); title('Control Inputs');
grid on;

disp('Simulation completed successfully.');

%% Trajectory Functions
% Sinusoidal trajectory function
function [q_d, q_dot_d, q_ddot_d] = sinusoidal_trajectory(t, A, f)
    omega = 2 * pi * f;
    q_d = A * [sin(omega * t); sin(omega * t)];
    q_dot_d = A * omega * [cos(omega * t); cos(omega * t)];
    q_ddot_d = -A * omega^2 * [sin(omega * t); sin(omega * t)];
end