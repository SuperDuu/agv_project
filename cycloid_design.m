% =========================================================================
% Modified Double-Disc Cycloidal Gear Profile Generator
% Optimized for FDM 3D Printing (PET-CF) & Backlash Elimination
% DuBotics Project - Gear Design Analysis
% =========================================================================

clear; clc; close all;

%% 1. Design Constraints & Input Parameters (Locked for Motor 5010)
N = 17;           % Number of cycloid lobes (tỷ số truyền 1:20 với 21 chốt vỏ)
R_p = 39.0;       % Base pin ring radius (mm)
R_r = 2.0;        % Base housing pin radius (mm) (phi 4mm steel pin)
e = 0.50;         % Eccentricity (mm) - Optimized to ensure positive curvature radius
num_points = 1000;% Number of points for smooth spline in SolidWorks

% Tolerance Compensation (Bù trừ co ngót PET-CF & Preload)
delta_R_p = -0.04; % XY shrinkage compensation (mm) for PET-Carbon Fiber
delta_R_r = -0.02; % Active backlash clearance (mm) to prevent binding

% Modified Geometry Calculation
R_p_mod = R_p + delta_R_p;
R_r_mod = R_r + delta_R_r;

fprintf('--- Cycloid Gear Design Parameters ---\n');
fprintf('Lobes (N): %d | Pins: %d\n', N, N+1);
fprintf('Modified Pin Ring Radius (Rp_mod): %.3f mm (Delta: %.2f mm)\n', R_p_mod, delta_R_p);
fprintf('Modified Pin Radius (Rr_mod): %.3f mm (Delta: %.2f mm)\n', R_r_mod, delta_R_r);
fprintf('Eccentricity (e): %.3f mm\n', e);
fprintf('--------------------------------------\n');

% Limit Check to avoid self-intersection (cusps) in cycloid profile
if e * (N + 1) >= R_p_mod
    error('Design Limit Violated: e * (N + 1) = %.2f >= Rp_mod = %.2f. Profile will self-intersect!', e * (N + 1), R_p_mod);
end

%% 2. Profile Generation (Local Coordinate System Centered at 0,0)
% We use 1001 points and close the curve by making the last point identical to the first.
phi = linspace(0, 2*pi, num_points + 1); 

% --- DISC 1 CALCULATION (Local Frame) ---
% Improved cycloid equation with modified values
psi = atan(sin(N*phi) ./ (R_p_mod / (e * (N + 1)) - cos(N*phi)));

% Parametric coordinates for Disc 1 (Centered at 0,0 for CAD import)
x1 = R_p_mod * cos(phi) - R_r_mod * cos(phi + psi) + e * cos((N + 1) * phi);
y1 = R_p_mod * sin(phi) - R_r_mod * sin(phi + psi) + e * sin((N + 1) * phi);
z1 = zeros(size(phi));

% --- DISC 2 CALCULATION (Rotational Phase Shift in Local Frame) ---
% Rotate the entire profile of Disc 1 by pi/N radians (180 deg / N)
theta_rot = pi / N;
x2 = x1 * cos(theta_rot) - y1 * sin(theta_rot);
y2 = x1 * sin(theta_rot) + y1 * cos(theta_rot);
z2 = zeros(size(phi));

%% 3. Export Data for SolidWorks
% Format: X \t Y \t Z (Tab-separated)
% Export Disc 1 coordinates centered at (0,0)

disc1_data = [x1', y1', z1'];
file_out = 'DuBotics_Cycloid_1_20.txt';
fid = fopen(file_out, 'w');
for i = 1:length(phi)
    fprintf(fid, '%.6f\t%.6f\t%.6f\r\n', disc1_data(i,1), disc1_data(i,2), disc1_data(i,3));
end
fclose(fid);

fprintf('Exported coordinates successfully:\n');
fprintf(' -> Disc 1: %s\n', fullfile(pwd, file_out));

%% 4. Graphical Verification Plot (Assembled State View)
figure('Color', [1 1 1], 'Position', [100 100 800 800]);
hold on; grid on; box on;

% In the assembly:
% - Disc 1 center translated to [-e, 0]
% - Disc 2 center translated to [e, 0]
x1_assembled = x1 - e;
y1_assembled = y1;
x2_assembled = x2 + e;
y2_assembled = y2;

% Plot Disc 1 (Blue) and Disc 2 (Magenta)
plot(x1_assembled, y1_assembled, 'b-', 'LineWidth', 2, 'DisplayName', 'Disc 1 (Center at [-e, 0])');
plot(x2_assembled, y2_assembled, 'm-', 'LineWidth', 2, 'DisplayName', 'Disc 2 (Center at [e, 0], Rotated \pi/N)');

% Plot Pin Ring (Housing Pins) - exactly N+1 = 21 pins
pin_angles = (0:N) * (2*pi / (N + 1));
pin_centers_x = R_p_mod * cos(pin_angles);
pin_centers_y = R_p_mod * sin(pin_angles);

% Draw housing pins as red circles
for i = 1:length(pin_angles)
    rectangle('Position', [pin_centers_x(i)-R_r_mod, pin_centers_y(i)-R_r_mod, 2*R_r_mod, 2*R_r_mod], ...
              'Curvature', [1 1], 'EdgeColor', [0.8 0.2 0.2], 'LineStyle', '-', ...
              'LineWidth', 1.2, 'HandleVisibility', 'off');
end
% Add dummy plot for legend
plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Housing Pins (\phi 4mm)');

% Plot eccentric centers
plot(-e, 0, 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Disc 1 Center');
plot(e, 0, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Disc 2 Center');
plot(0, 0, 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Housing Center');

title('Modified Double-Disc Cycloidal Profile (Assembled State)');
xlabel('X Coordinate (mm)');
ylabel('Y Coordinate (mm)');
axis equal;
xlim([-(R_p_mod + 2*R_r_mod + 1), R_p_mod + 2*R_r_mod + 1]);
ylim([-(R_p_mod + 2*R_r_mod + 1), R_p_mod + 2*R_r_mod + 1]);
legend('Location', 'northeastoutside');

% Save verification plot
saveas(gcf, 'cycloid_double_disc_plot.png');
fprintf('Saved verification plot to: %s\n', fullfile(pwd, 'cycloid_double_disc_plot.png'));
