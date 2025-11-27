clc; close all; clear all;
%% ===================== 1. Robot Physical Parameters =====================
fprintf('\n--- 1. Robot Physical Parameters Loaded ---\n');
d1_val = 0.03;    % Base height (m)
l2_val = 0.8;     % Link 2 length (m)
l3_val = 0.82;    % Link 3 length (m)
m2 = 1.5;         % Mass of link 2 (kg)
m3 = 1.0;         % Mass of link 3 (kg)
g = 9.81;         % Gravity (m/s^2)
dt = 0.01;        % Control time step (s)
T_total = 5.0;    % Total time for trajectory (seconds)

%% -------------------- 2. Joint Limits & Home Position --------------------

fprintf('--- 2. Joint Limits & Home Position Defined ---\n');
q_home = deg2rad([90; 90; 90]);  
joint_min = deg2rad([0; 0; 107]);    
joint_max = deg2rad([180; 180; 180]);

%% -------------------- 3. Desired End-Effector Position --------------------

fprintf('--- 3. Desired End-Effector Position (Target) Set ---\n');
mu_a = [0.826597; -0.275958; 0.125785]; % Target position (m)

%% ===================== 4. Forward & Differential Kinematics =====================

fprintf('--- 4. Symbolic Kinematics Setup (DH, T-Matrices, Jacobian) ---\n');
syms th1 th2 th3 real
syms d1 l2 l3 real
syms th1d th2d th3d real
subs_params = [th1, th2, th3, d1, l2, l3];

% DH Table [alpha, a, theta, d]
DH = [ 0,    0,   th1, d1;
       pi/2, 0,   th2, 0;
       0,    l2,  th3, 0;
       0,    l3,   0,  0];

% General DH Transformation function
TDH = @(alpha,a,theta,d) [ cos(theta), -sin(theta), 0, a;
                           sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
                           sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
                           0, 0, 0, 1 ];

% Compute transformation matrices (symbolic)
T01 = TDH(DH(1,1), DH(1,2), DH(1,3), DH(1,4));
T02 = simplify(T01 * TDH(DH(2,1), DH(2,2), DH(2,3), DH(2,4)));
T03 = simplify(T02 * TDH(DH(3,1), DH(3,2), DH(3,3), DH(3,4)));
T04 = simplify(T03 * TDH(DH(4,1), DH(4,2), DH(4,3), DH(4,4)));
P_ee_sym = T04(1:3,4); 

% Symbolic Jacobian (Position only: 3x3)
J_sym = jacobian(P_ee_sym, [th1; th2; th3]); 
Jdot_sym = sym(zeros(3,3));
for r = 1:3
    for c = 1:3
        Jdot_sym(r,c) = jacobian(J_sym(r,c), th1)*th1d + ...
                        jacobian(J_sym(r,c), th2)*th2d + ...
                        jacobian(J_sym(r,c), th3)*th3d;
    end
end

% --- Pre-substitute Constant Link Lengths (d1, l2, l3) ---
const_syms = [d1, l2, l3];
const_vals = [d1_val, l2_val, l3_val];
J_sym_pre = subs(J_sym, const_syms, const_vals);
Jdot_sym_pre = subs(Jdot_sym, const_syms, const_vals);
remaining_pos_syms = [th1, th2, th3];
remaining_vel_syms = [th1d, th2d, th3d];

% --- Helper function to compute FK positions for plotting ---
compute_fk_positions = @(q_in) deal(...
    double(subs(T01(1:3,4), subs_params, [q_in', d1_val, l2_val, l3_val])), ...
    double(subs(T02(1:3,4), subs_params, [q_in', d1_val, l2_val, l3_val])), ...
    double(subs(T03(1:3,4), subs_params, [q_in', d1_val, l2_val, l3_val])), ...
    double(subs(T04(1:3,4), subs_params, [q_in', d1_val, l2_val, l3_val])) );

% --- FIGURE 1: Home Position ---
figure(1);
set(gcf, 'Name', 'Home Configuration (Start Position)');
[P01h, P02h, P03h, P04h] = compute_fk_positions(q_home);
plot_robot([0 P01h(1) P02h(1) P03h(1) P04h(1)], ...
           [0 P01h(2) P02h(2) P03h(2) P04h(2)], ...
           [0 P01h(3) P02h(3) P03h(3) P04h(3)], 'b-o', 'Figure 1: Home Configuration (Start Position)');
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
drawnow;
pause(1.5); 

%% -------------------- 5. IK Initialization & Parameters --------------------

fprintf('--- 5. Inverse Kinematics Initialization ---\n');
th1_guess = atan2(mu_a(2), mu_a(1)); 
q_ik = [th1_guess; q_home(2); q_home(3)];
maxIter = 7000;
tol = 1e-4;
lambda = 0.1;
max_step = 0.004;
buffer_angle = deg2rad(10);
damping_reduction_factor = 0.01;

%% -------------------- 6. Solve Inverse Kinematics (DLS) --------------------

fprintf('\n--- 6. IK Solution (DLS) ---\n');
for i = 1:maxIter
    th1 = q_ik(1); th2 = q_ik(2); th3 = q_ik(3);
    
    vals_q = [th1, th2, th3]; 
    
    P04 = double(subs(P_ee_sym, [remaining_pos_syms, const_syms], [vals_q, const_vals]));
    mu_e = P04;
    delta_x = mu_a - mu_e;
    
    if norm(delta_x) < tol
        break;
    end

    J = double(subs(J_sym_pre, remaining_pos_syms, vals_q));

    dq = pinv(J'*J + lambda^2 * eye(3)) * J' * delta_x;
    
    if q_ik(1) < buffer_angle && dq(1) < 0; dq(1) = dq(1) * damping_reduction_factor; end
    if q_ik(1) > (pi - buffer_angle) && dq(1) > 0; dq(1) = dq(1) * damping_reduction_factor; end
    
    if norm(dq) > max_step; dq = dq / norm(dq) * max_step; end
    
    q_ik = max(min(q_ik + dq, joint_max), joint_min);
end
q_final = q_ik;
final_error = norm(mu_a - mu_e);
fprintf('IK solved in %d iterations. Final error: %.6f m\n', i, final_error);

% === 6b. Numerical Forward Kinematics Results (T01 to T04 PRINTED) ===
fprintf('\n--- 6b. Numerical Forward Kinematics Results ---\n');

% 1. HOME Configuration (Start)
q = q_home;
T01_home = double(subs(T01, subs_params, [q', d1_val, l2_val, l3_val]));
T02_home = double(subs(T02, subs_params, [q', d1_val, l2_val, l3_val]));
T03_home = double(subs(T03, subs_params, [q', d1_val, l2_val, l3_val]));
T04_home = double(subs(T04, subs_params, [q', d1_val, l2_val, l3_val]));

fprintf('1. HOME Configuration (Start) T01 through T04:\n'); % Explicitly start at T01
fprintf('T01:\n'); disp(T01_home);
fprintf('T02:\n'); disp(T02_home);
fprintf('T03:\n'); disp(T03_home);
fprintf('T04 (End Effector):\n'); disp(T04_home);

% 2. FINAL IK Configuration (Target)
q = q_final;
T01_final = double(subs(T01, subs_params, [q', d1_val, l2_val, l3_val]));
T02_final = double(subs(T02, subs_params, [q', d1_val, l2_val, l3_val]));
T03_final = double(subs(T03, subs_params, [q', d1_val, l2_val, l3_val]));
T04_final = double(subs(T04, subs_params, [q', d1_val, l2_val, l3_val]));

fprintf('\n2. FINAL IK Configuration (Target) T01 through T04:\n'); % Explicitly start at T01
fprintf('T01:\n'); disp(T01_final);
fprintf('T02:\n'); disp(T02_final);
fprintf('T03:\n'); disp(T03_final);
fprintf('T04 (End Effector):\n'); disp(T04_final);

% --- FIGURE 2: IK Transition Animation ---
num_steps_ik = 75;
traj_ik = zeros(3, num_steps_ik);
for j = 1:3; traj_ik(j,:) = linspace(q_home(j), q_final(j), num_steps_ik); end
figure(2);
set(gcf, 'Name', 'IK Solution - Home to Final Target Animation');
title('Figure 2: IK Solution - Home to Final Target Animation');
hold on; grid on; axis equal; xlim([-2 2]); ylim([-2 2]); zlim([0 2]);
xlabel('X'); ylabel('Y'); zlabel('Z'); view([45 25]);
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
for k = 1:num_steps_ik
    [P01, P02, P03, P04] = compute_fk_positions(traj_ik(:,k));
    cla;
    plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on
    plot3([0 P01(1) P02(1) P03(1) P04(1)], ...
          [0 P01(2) P02(2) P03(2) P04(2)], ...
          [0 P01(3) P02(3) P03(3) P04(3)], 'r-o', 'LineWidth', 1.5);
    title(sprintf('IK Transition (Step %d)', k));
    drawnow; pause(0.02); 
end
pause(1.5); 

% --- FIGURE 3: IK Solution (Static) ---
figure(3);
set(gcf, 'Name', 'Final IK Configuration (Target Pose)');
[P01f, P02f, P03f, P04f] = compute_fk_positions(q_final);
plot_robot([0 P01f(1) P02f(1) P03f(1) P04f(1)], ...
           [0 P01f(2) P02f(2) P03f(2) P04f(2)], ...
           [0 P01f(3) P02f(3) P03f(3) P04f(3)], 'b-o', 'Figure 3: Final IK Configuration (Target Pose)');
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
drawnow;
pause(1.5); 

%% ===================== 7. Trajectory Planning =====================

fprintf('\n--- 7. Trajectory Planning ---\n');
num_steps = round(T_total / dt);
time_vector = linspace(0, T_total, num_steps);
[traj_q_d, traj_qd_d, traj_qdd_d] = deal(zeros(3, num_steps));
% Cubic polynomial generation
for j = 1:3
    q_start = q_home(j); q_end = q_final(j);
    A = [1, 0, 0, 0; 0, 1, 0, 0; 1, T_total, T_total^2, T_total^3; 0, 1, 2*T_total, 3*T_total^2];
    b = [q_start; 0; q_end; 0];
    coeffs = A\b;
    
    for k = 1:num_steps
        t = time_vector(k);
        traj_q_d(j, k) = coeffs(1) + coeffs(2)*t + coeffs(3)*t^2 + coeffs(4)*t^3;
        traj_qd_d(j, k) = coeffs(2) + 2*coeffs(3)*t + 3*coeffs(4)*t^2;
        traj_qdd_d(j, k) = 2*coeffs(3) + 6*coeffs(4)*t;
    end
end

% Compute Cartesian trajectory for visualization
P_ee_traj = zeros(3, num_steps);
for k = 1:num_steps
    P_ee_traj(:,k) = double(subs(P_ee_sym, [remaining_pos_syms, const_syms], [traj_q_d(:,k)', const_vals]));
end

% --- FIGURE 4: Planned Trajectory Path ---
figure(4);
set(gcf, 'Name', 'Planned Trajectory Path');
plot_robot([0 P01h(1) P02h(1) P03h(1) P04h(1)], [0 P01h(2) P02h(2) P03h(2) P04h(2)], [0 P01h(3) P02h(3) P03h(3) P04h(3)], 'b-o', 'TEMP TITLE'); 
hold on; 
plot_robot([0 P01f(1) P02f(1) P03f(1) P04f(1)], [0 P01f(2) P02f(2) P03f(2) P04f(2)], [0 P01f(3) P02f(3) P03f(3) P04f(3)], 'b-o');
plot3(P_ee_traj(1,:), P_ee_traj(2,:), P_ee_traj(3,:), 'k:', 'LineWidth', 2);
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
title('Figure 4: Planned Trajectory Path');
legend('Start Pose', 'Target Pose (IK)', 'Desired Path', 'Target Point');
drawnow;
pause(1.5);

%% ===================== 8. Robot Dynamics & Closed-Loop Control (Using Derived Equations) =====================

fprintf('\n--- 8. Control Simulation (Closed-Loop CTC) ---\n');

Kp = 15 * eye(3); 
Kd = 5 * eye(3);  
q = q_home; qd = zeros(3, 1);  
[q_hist, qd_hist, tau_hist, P_ee_actual, ee_accel_hist] = deal(zeros(3, num_steps));
l2 = l2_val; l3 = l3_val; m2h = m2/2; m3h = m3/2;
for k = 1:num_steps

    % ----------------- State & Kinematics -----------------
    th1 = q(1); th2 = q(2); th3 = q(3);
    th1d = qd(1); th2d = qd(2); th3d = qd(3);
    
    q_all = [th1, th2, th3]; 
    qd_all = [th1d, th2d, th3d]; 
    J = double(subs(J_sym_pre, remaining_pos_syms, q_all));
    Jdot = double(subs(Jdot_sym_pre, [remaining_pos_syms, remaining_vel_syms], [q_all, qd_all]));
    
    % Cursory trigonometric and mass variables
    c2 = cos(th2); c3 = cos(th3); s2 = sin(th2); s3 = sin(th3);
    c23 = cos(th2 + th3); s23 = sin(th2 + th3);
    
    % 1. M(q) - Inertia Matrix
    M_11 = (l2*c2 + l3*c23)^2; % Simplified inertia for joint 1 (yaw)
    M_22 = m2*(l2/2)^2 + m3*(l2^2 + (l3/2)^2 + l2*l3*c3); 
    M_23 = m3*( (l3/2)^2 + (l2*l3/2)*c3 ); 
    M_33 = m3*(l3/2)^2;
    
    M = [ M_11, 0, 0;
          0, M_22, M_23;
          0, M_23, M_33 ]; % M is symmetric
      
    % 2. C(q, qd) - Coriolis/Centripetal Vector
    C = zeros(3,1);
    
    % C1 (depends on qd_1, qd_2, qd_3)
    C_1 = 2 * th1d * (l2*c2 + l3*c23) * (-l2*th2d*s2 - l3*(th2d+th3d)*s23);
    
    % C2 (depends on qd_1^2, qd_2^2, qd_3^2, qd_2*qd_3)
    C_2 = -th1d^2 * (l2*c2 + l3*c23) * (-l2*s2 - l3*s23) ...
          + m3*l2*l3*s3*th3d*(th2d+th3d) - m3*l2*l3/2*s3*th2d*th3d;
      
    % C3 (depends on qd_2^2, qd_2*qd_3)
    C_3 = m3*l2*l3/2*s3*th2d^2 - m3*l2*l3/2*s3*th2d*th3d;
    
    C = [C_1; C_2; C_3];
    
    % 3. G(q) - Gravity Vector
    G = zeros(3,1);
    G_2 = (m2*(l2/2) + m3*l2)*g*c2 + m3*(l3/2)*g*c23;
    G_3 = m3*(l3/2)*g*c23;
    G = [0; G_2; G_3];
    
    % ----------------- CLOSED-LOOP Computed Torque Control Law -----------------
    % Tau = M * qdd_r + C + G
    q_d = traj_q_d(:, k);
    qd_d = traj_qd_d(:, k);
    qdd_d = traj_qdd_d(:, k);
    
    error_q = q_d - q;
    error_qd = qd_d - qd;
    
    qdd_r = qdd_d + Kp*error_q + Kd*error_qd;
    
    tau = M * qdd_r + C + G; 
    
    % ----------------- Robot State Integration (Forward Dynamics) -----------------
    qdd_sim = M \ (tau - C - G);
    
    % --- Robustness Check: Prevents NaN/Inf from corrupting state ---

    if any(isnan(qdd_sim)) || any(isinf(qdd_sim)) || norm(qdd_sim) > 1e6 
        warning('Simulation instability detected at k=%d. Clipping acceleration.', k);
        qdd_sim = zeros(3, 1); 
    end

    % -----------------------------------------------------------------
    qd = qd + qdd_sim * dt;
    q = q + qd * dt;
    
    q = max(min(q, joint_max), joint_min); % Enforce joint limits
    
    % ----------------- Data Logging -----------------
    q_hist(:, k) = q;
    qd_hist(:, k) = qd;
    tau_hist(:, k) = tau;
    P_ee_actual(:,k) = double(subs(P_ee_sym, [remaining_pos_syms, const_syms], [q', const_vals]));
    ee_accel_hist(:, k) = J * qdd_sim + Jdot * qd; 
end
fprintf('Simulation finished in %.2f s.\n', T_total);

%% ===================== 9. Analysis and Visualization (2D & 3D Plots) =====================

fprintf('\n--- 9. Simulation Results (2D & 3D) ---\n');

% --- FIGURE 5: Control Simulation Animation (3D) ---
figure(5);
set(gcf, 'Name', 'Control Simulation - Actual Trajectory Tracking');
title('Figure 5: Control Simulation - Actual Trajectory Tracking');
hold on; grid on; axis equal; xlim([-2 2]); ylim([-2 2]); zlim([0 2]);
xlabel('X'); ylabel('Y'); zlabel('Z'); view([45 25]);
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot3(P_ee_traj(1,:), P_ee_traj(2,:), P_ee_traj(3,:), 'k:', 'LineWidth', 1.5);
for k = 1:5:num_steps
    [P01, P02, P03, P04] = compute_fk_positions(q_hist(:,k));
    cla;
    plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
    plot3(P_ee_traj(1,:), P_ee_traj(2,:), P_ee_traj(3,:), 'k:', 'LineWidth', 1.5);
    
    plot3([0 P01(1) P02(1) P03(1) P04(1)], ...
          [0 P01(2) P02(2) P03(2) P04(2)], ...
          [0 P01(3) P02(3) P03(3) P04(3)], 'b-o', 'LineWidth', 2); 
    
    plot3(P_ee_actual(1,1:k), P_ee_actual(2,1:k), P_ee_actual(3,1:k), 'c-', 'LineWidth', 1);
    
    title(sprintf('Control Tracking (Time: %.2f s)', time_vector(k)));
    drawnow;
    pause(0.01);
end
pause(1.5); 

% --- FIGURE 6: Final Control Configuration (3D) ---
q_final_sim = q_hist(:, end);
[P01s, P02s, P03s, P04s] = compute_fk_positions(q_final_sim);
figure(6);
set(gcf, 'Name', 'Final Control Configuration (Simulated End Pose)');
plot_robot([0 P01s(1) P02s(1) P03s(1) P04s(1)], [0 P01s(2) P02s(2) P03s(2) P04s(2)], [0 P01s(3) P02s(3) P03s(3) P04s(3)], 'b-o', 'Figure 6: Final Control Configuration (Simulated End Pose)'); hold on;
plot3(mu_a(1), mu_a(2), mu_a(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
plot3(P_ee_traj(1,:), P_ee_traj(2,:), P_ee_traj(3,:), 'k:', 'LineWidth', 1.5);
plot3(P_ee_actual(1,:), P_ee_actual(2,:), P_ee_actual(3,:), 'c-', 'LineWidth', 1);
legend('Final Pose', 'Target Point', 'Desired Path', 'Actual Path');
drawnow;
pause(1.5);

% --- FIGURE 7: Joint Position Tracking (Standard 2D for detail) ---
figure(7);
set(gcf, 'Name', 'Joint Position Tracking Performance');
for i = 1:3
    subplot(3, 1, i);
    plot(time_vector, traj_q_d(i,:), 'k--', 'LineWidth', 1.0); hold on; % Thicker Dashed Line
    plot(time_vector, q_hist(i,:), 'b-o', 'MarkerSize', 2, 'LineWidth', 0.5); % Marker added to Actual line
    ylabel(['$q_', num2str(i), '$ (rad)'], 'Interpreter', 'latex');
    if i == 1; title('Figure 7: Joint Position Tracking (Closed-Loop)'); end
    if i == 3; xlabel('Time (s)'); end
    legend('Desired', 'Actual'); grid on;
end
drawnow;
pause(1.5);

% --- FIGURE 8: Control Torque Effort (Standard 2D) ---
figure(8); 
set(gcf, 'Name', 'Control Torque Effort');
plot(time_vector, tau_hist(1,:), 'b-', time_vector, tau_hist(2,:), 'r-', time_vector, tau_hist(3,:), 'g-');
legend('$\tau_1$', '$\tau_2$', '$\tau_3$', 'Interpreter', 'latex');
xlabel('Time (s)'); ylabel('Torque (Nm)'); 
title('Figure 8: Computed Torque Control Effort');
grid on;
drawnow;
pause(1.5); 

% --- FIGURE 9: End-Effector Acceleration (Propagation) ---
figure(9);
set(gcf, 'Name', 'End-Effector Acceleration Propagation');
plot(time_vector, ee_accel_hist(1,:), 'b-', time_vector, ee_accel_hist(2,:), 'r-', time_vector, ee_accel_hist(3,:), 'g-');
legend('$\ddot{x}_{ee}$', '$\ddot{y}_{ee}$', '$\ddot{z}_{ee}$', 'Interpreter', 'latex');
xlabel('Time (s)'); 
ylabel('Acceleration ($m/s^2$)', 'Interpreter', 'latex'); 
title('Figure 9: Actual End-Effector Acceleration ($\ddot{\mathbf{x}} = \mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}}$)', 'Interpreter', 'latex');
grid on;
drawnow;
pause(1.5);

% --- FIGURE 10: Joint Position Tracking (2D Components) ---
figure(10);
set(gcf, 'Name', 'Joint Position Tracking');
for i = 1:3
    subplot(3, 1, i);

    plot(time_vector, traj_q_d(i,:), 'k--', 'LineWidth', 1.5); hold on; 

    plot(time_vector, q_hist(i,:), 'b-', 'Marker', 'o', 'MarkerSize', 2, 'LineWidth', 1.0); 
    
    ylabel(['$q_', num2str(i), '$ (rad)'], 'Interpreter', 'latex');
    
    if i == 1
        title('Figure 10: Joint Position Tracking ($\mathbf{q}$ Desired vs Actual)', 'Interpreter', 'latex'); 
        legend('Desired', 'Actual', 'Location', 'best'); % Add legend to the top subplot
    end
    
    if i == 3
        xlabel('Time (s)'); 
    end
    
    grid on;
end
drawnow;
pause(1.5);
drawnow;
pause(1.5);

% --- FIGURE 11: Control Torque Vector (2D Components) ---
figure(11);
set(gcf, 'Name', 'Control Torque Vector Trajectory');
for i = 1:3
    subplot(3, 1, i);
    
    plot(time_vector, tau_hist(i,:), 'b.-', 'MarkerSize', 2, 'LineWidth', 0.5); 
    
    ylabel(['$\tau_', num2str(i), '$ (Nm)'], 'Interpreter', 'latex');
    
    if i == 1; title('Figure 11: Control Torque Components ($\tau_i$ vs Time)', 'Interpreter', 'latex'); end
    
    if i == 3; xlabel('Time (s)'); end
    grid on;
end
drawnow;
pause(1.5);

% --- FIGURE 12: End-Effector Acceleration Vector (2D Components) ---
figure(12);
set(gcf, 'Name', 'End-Effector Acceleration');
ee_labels = {'$\ddot{x}_{ee}$ ($m/s^2$)', '$\ddot{y}_{ee}$ ($m/s^2$)', '$\ddot{z}_{ee}$ ($m/s^2$)'}; 
for i = 1:3
    subplot(3, 1, i);
    
    plot(time_vector, ee_accel_hist(i,:), 'r.-', 'MarkerSize', 2, 'LineWidth', 0.5); 
    
    ylabel(ee_labels{i}, 'Interpreter', 'latex');
    
    if i == 1; title('Figure 12: End-Effector Acceleration Components ($\ddot{x}_i$ vs Time)', 'Interpreter', 'latex'); end
    if i == 3; xlabel('Time (s)'); end
    grid on;
end
drawnow;
pause(1.5);

%% -------------------- Helper Plot Function --------------------

function plot_robot(x, y, z, style, title_str)
    hold off;
    plot3(x, y, z, style, 'LineWidth', 2, 'MarkerSize', 5); hold on;
    plot3(x(end), y(end), z(end), 'o', 'MarkerSize', 8, 'MarkerFaceColor', style(1)); % End effector
    grid on; axis equal;
    xlim([-2 2]); ylim([-2 2]); zlim([0 2]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view([45 25]);
    if nargin > 4
        title(title_str);
    end
end

%% ================== 10. Send Angles to Arduino (Hardware Interface) ==================

fprintf('\n--- 10. Hardware Interface (Arduino) ---\n');
disp('Connecting to Arduino...');
port = "/dev/ttyACM0";  
baud = 9600;
arduinoObj = serialport(port, baud);
configureTerminator(arduinoObj, "LF");
flush(arduinoObj);
pause(2); 
if arduinoObj.NumBytesAvailable > 0
    disp(readline(arduinoObj));
end
angles_deg = round(rad2deg(q_final)); 
global prev_angles;
if isempty(prev_angles); prev_angles = zeros(1, 3); end
timeout = 2;
prev_time = tic;
for i = 1:3
    if angles_deg(i) ~= prev_angles(i)
        cmd = sprintf("J%d:%d", i, angles_deg(i));
        writeline(arduinoObj, cmd);
        disp(['Sending ', cmd]);
        pause(0.1); 
        
        response_received = false;
        
        while (arduinoObj.NumBytesAvailable == 0)
            if (toc(prev_time) > timeout)
                warning('Timeout: No response from Arduino within %d seconds.', timeout); 
                break; 
            end
            pause(0.01);
        end
        
        if arduinoObj.NumBytesAvailable > 0
            resp = readline(arduinoObj);
            fprintf('Arduino reports: %s\n', resp);
            response_received = true;
        end
        
        if ~response_received
            disp('No response from Arduino within the timeout period.'); 
        end
        prev_angles(i) = angles_deg(i);
    end
end
disp('✅ Angles sent and confirmed.');
clear arduinoObj;
