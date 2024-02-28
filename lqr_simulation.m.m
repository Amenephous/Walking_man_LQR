% Clear all variables, close figures, clear command window, close files, close Simulink models
clear all;
close all;
clc;
fclose all;
bdclose all;

% Initialize desired state
desired_state = 0;

% Cost function parameters
Q = [100, 0; 0, 10];
R = 0.6;

% Time step and dynamics parameters
del_t = 0.5;
w = 10;
A = [1, del_t; w * del_t, 1];
B = [0; -w * del_t];

% Number of iterations
horizon_len = floor(12 / del_t);

% Initialize arrays to store state derivatives
der_state_array = zeros(horizon_len, 2);

% Generate desired state trajectory
for i = 1:horizon_len
    r = randi([4 6], 1);
    v = r / 10;
    desired_state = desired_state + v * del_t;
    der_state_array(i, :) = [desired_state; 0]; 
end

% Backward pass to compute control gains
K = struct;
k = struct;
P_next = Q;
p_next = -Q * [10; 0];

for n = horizon_len:-1:1
    qn = -Q * der_state_array(n, :)';
    Kn = -inv(R + B' * P_next * B) * B' * P_next * A;
    Pn = Q + A' * P_next * A + A' * P_next * B * Kn;
    kn = -inv(R + B' * P_next * B) * B' * p_next;
    pn = qn + A' * p_next + A' * P_next * B * kn;
    
    K(n).value = Kn;
    k(n).value = kn;
    
    P_next = Pn;
    p_next = pn;
end

% Call the walking man simulation function
WalkingManSimulation(K, k, del_t);
