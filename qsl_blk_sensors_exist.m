function [xi_hat, xi_dot_hat, gps] = qsl_blk_sensors_exist(t, xi, q, Tprev)
% NEW BLOCK "Sensors_KF" (between plant and PositionController).
% GPS (20 Hz, noisy) corrects an IMU-driven prediction: Kalman "blending".
% Accelerometer reading of the previous step = thrust/m along the body z axis
% (+ noise) because the plant has no drag/wind. Tprev comes through a Unit Delay.
W = qsl_world();  cfg = W.cfg;
x = [xi(:); zeros(3,1); q(:); zeros(3,1)];
fprev = [0; 0; Tprev/cfg.m] + cfg.accStd*randn(3,1);
[xi_hat, xi_dot_hat, ~, ~, gps] = qsl_blk_sensors(t, x, fprev);
end
