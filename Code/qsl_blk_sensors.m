function [ph, vh, qm, wm, gps] = qsl_blk_sensors(t, x, fprev)
% BLOCK 2  Onboard sensors + own-state Kalman filter ("blending").
%   IMU  : every step, PREDICTS position/velocity from the accelerometer
%   GPS  : 20 Hz, CORRECTS it:   x = x_pred + K (z - H x_pred)
%   AHRS + gyro give the measured quaternion qm and angular rate wm.
% x = true plant state [p v q w] (13), fprev = accelerometer reading of the
% previous step (3).
persistent xh Ph qmPrev started
W = qsl_world();  cfg = W.cfg;  g = cfg.g;  dt = cfg.dt;
k = round(t/dt) + 1;
if isempty(started) || k == 1
    xh = [W.p0; 0;0;0];
    Ph = diag([1e-4 1e-4 1e-4 1e-2 1e-2 1e-2]);
    qmPrev = [1;0;0;0];  started = true;
else                                           % PREDICT with last step's IMU
    a_imu = qsl_lib('qm_to_R', qmPrev)*fprev(:) + [0;0;-g];
    Fk = [eye(3) dt*eye(3); zeros(3) eye(3)];
    xh = Fk*xh + [0.5*dt^2*a_imu; dt*a_imu];
    Gk = [0.5*dt^2*eye(3); dt*eye(3)];
    Ph = Fk*Ph*Fk' + Gk*(cfg.accStd^2*eye(3))*Gk';
end
p = x(1:3);  q = x(7:10);  w = x(11:13);
qm = q + cfg.attStd*randn(4,1);  qm = qm/norm(qm);
if qm(1) < 0, qm = -qm; end
wm = w + cfg.gyroStd*randn(3,1);
qmPrev = qm;
gps = nan(3,1);
if mod(k-1, cfg.sensorEvery) == 0              % CORRECT with GPS
    z = p + cfg.gpsStd*randn(3,1);
    gps = z;
    H = [eye(3) zeros(3)];
    S = H*Ph*H' + cfg.gpsStd^2*eye(3);
    K = Ph*H'/S;
    xh = xh + K*(z - H*xh);
    Ph = (eye(6) - K*H)*Ph;
end
ph = xh(1:3);  vh = xh(4:6);
end
