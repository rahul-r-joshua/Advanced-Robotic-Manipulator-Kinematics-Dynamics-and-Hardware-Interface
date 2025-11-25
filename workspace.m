clc; clear; close all;

%% Robot Physical Parameters
d1 = 0.03;     
l2 = 0.8;      
l3 = 0.82;     

%% Joint Limits
points_per_joint = 40;
q1_range = deg2rad(linspace(0, 180, points_per_joint)); 
q2_range = deg2rad(linspace(0, 180, points_per_joint)); 
q3_range = deg2rad(linspace(107, 180, points_per_joint)); 

%% Workspace Calculation (Brute-Force Sampling)
[X, Y, Z] = deal([]);
total_points = length(q1_range) * length(q2_range) * length(q3_range);
disp(['Computing workspace with ', num2str(total_points), ' points...']);

for q1 = q1_range
    for q2 = q2_range
        for q3 = q3_range
            
            r_xy = l2*cos(q2) + l3*cos(q2+q3);
            x = cos(q1) * r_xy;
            y = sin(q1) * r_xy;
            z = d1 + l2*sin(q2) + l3*sin(q2+q3);

            X(end+1) = x;
            Y(end+1) = y;
            Z(end+1) = z;
        end
    end
end

%% Plot Workspace
figure('Name','3-DOF Robot Workspace','Position',[200 100 800 600]);
scatter3(X, Y, Z, 6, Z, 'filled'); 
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Reachable Workspace of 3-DOF Robot');
axis equal; grid on; view(45, 25);
colorbar; colormap(jet);
xlim([-1.8 1.8]); ylim([-1.8 1.8]); zlim([0 1.8]);

%% Compute Reach Limits
fprintf('\n--- Workspace Limits ---\n');
fprintf('X: min = %.3f m, max = %.3f m\n', min(X), max(X));
fprintf('Y: min = %.3f m, max = %.3f m\n', min(Y), max(Y));
fprintf('Z: min = %.3f m, max = %.3f m\n', min(Z), max(Z));

radius_from_base = sqrt(X.^2 + Y.^2 + (Z - d1).^2); 
fprintf('\nMax Reach (Radius): %.3f m\n', max(radius_from_base));
fprintf('Min Reach (Radius): %.3f m\n', min(radius_from_base)); 

%% End-Effector FK Equations
fprintf('\n--- End-Effector Position (FK) Equations ---\n');
fprintf('r_{xy} = l2*cos(q2) + l3*cos(q2 + q3)\n');
fprintf('x = cos(q1) * r_{xy}\n');
fprintf('y = sin(q1) * r_{xy}\n');
fprintf('z = d1 + l2*sin(q2) + l3*sin(q2 + q3)\n');
