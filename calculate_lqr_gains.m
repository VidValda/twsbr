%% calculate_lqr_gains.m

clear; clc;

% --- 1. Model Parameters ---
d = 0.59;
l = 0.14;
r = 0.2;
mb = 41;
mw = 2;
J = 0.04;
K_param = 0.02;
I1 = 1;
I2 = 1;
I3 = 1;
calpha = 0.01;
g = 9.81;

fprintf('--- 1. Model Parameters ---\n');
fprintf('mb = %g kg, l = %g m, r = %g m\n\n', mb, l, r);

fprintf('--- 2. Linearizing dynamics around theta = 0 ---\n');

a11 = mb + 2*mw + 2*J/r^2;
a12 = mb*l;
a21 = a12;
a22 = I2 + mb*l^2;
a33 = I3 + 2*K_param + mw*d^2/2 + J*d^2/(2*r^2);

M0 = [a11, a12, 0;
      a21, a22, 0;
      0, 0, a33];

d11 = 2*calpha/r^2;
d12 = -2*calpha/r;
d21 = d12;
d22 = 2*calpha;
d33 = calpha*d^2/(2*r^2);

D_matrix = [d11, d12, 0;
            d21, d22, 0;
            0, 0, d33];

b11 = 1/r;
b12 = b11;
b21 = -1;
b22 = b21;
b31 = -d / (2*r);
b32 = -b31;

B_input = [b11, b12;
           b21, b22;
           b31, b32];

G_lin = [0, 0, 0;
         0, -mb*l*g, 0;
         0, 0, 0];
         
M0_inv = inv(M0);

A = [zeros(3,3),   eye(3);
     -M0_inv * G_lin, -M0_inv * D_matrix];

B_ss = [zeros(3,2);
        M0_inv * B_input];

fprintf('\nState Matrix A (6x6):\n');
disp(A);
fprintf('\nInput Matrix B (6x2):\n');
disp(B_ss);

fprintf('\n--- 3. Designing LQR Controller ---\n');

Q = diag([
    1,     % x
    100,   % theta
    0.1,   % psi
    1,     % x_dot
    10,    % theta_dot
    0.1    % psi_dot
]);

R = diag([0.1, 0.1]);

fprintf('\nState Weights Q (diagonal):\n');
disp(diag(Q)');
fprintf('\nControl Weights R (diagonal):\n');
disp(diag(R)');

fprintf('\nSolving LQR with built-in lqr()...\n');
[K_lqr, S, poles_closed] = lqr(A, B_ss, Q, R);

if isempty(K_lqr)
    fprintf('LQR calculation failed. Exiting.\n');
    return;
end

fprintf('\n--- 4. LQR Gain Matrix K (2x6) ---\n');
disp(K_lqr);

fprintf('\nClosed-loop poles:\n');
disp(poles_closed);

fprintf('\n--- 5. Copy this into the Modelica model ---\n');
fprintf('parameter Real K[2,6] = {\n');
[rows, cols] = size(K_lqr);
for i = 1:rows
    fprintf('  {');
    for j = 1:cols
        fprintf('%.8f', K_lqr(i,j));
        if j < cols
            fprintf(', ');
        end
    end
    fprintf('}');
    if i < rows
        fprintf(',\n');
    else
        fprintf('\n');
    end
end
fprintf('};\n');
fprintf('----------------------------------------------\n');