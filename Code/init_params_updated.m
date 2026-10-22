% init_params.m  (your file + the new obstacle project)
m  = 0.025;                      % kg
Jx = 4.856e-3; Jy = 4.856e-3; Jz = 8.801e-3;
J  = [Jx Jy Jz];
g  = 9.81;

% Position controller gains (eq. 20)
useTunedPositionGains = true;    % true: critically damped (Kp 16, Kd 8); false: paper values
if useTunedPositionGains
    Kp_xi = diag([16 16 16]);  Kd_xi = diag([8 8 8]);  Ki_xi = diag([2 2 2]);
else
    Kp_xi = diag([60 60 60]);  Kd_xi = diag([6 6 6]);  Ki_xi = diag([0.1 0.1 0.1]);
end

% Attitude controller gains (eq. 27)  (unchanged)
Kp_q = diag([2000 2000 2000]);
Kd_q = diag([10 10 10]);
Ki_q = diag([10 10 5]);

q3r_des = 0;   % desired yaw-related quaternion component (fixed to 0 as in paper)

% Initial conditions (same start point as the new B-spline reference)
xi0    = [1.8; 2.27; 0.6];
xidot0 = [0;0;0];
q0_init = [1;0;0;0];   % identity quaternion
omega0  = [0;0;0];

% ---- obstacle project: put these folders on the path ------------------------
proj = fileparts(mfilename('fullpath'));
for d = {proj, fullfile(proj,'simulink'), fullfile(proj,'existing_model')}
    if isfolder(d{1}), addpath(d{1}); end          % only folders that exist
end
% obstacles, noise, B-spline degree, sensor range ... are edited in default_config.m
