function cfg = default_config()
% DEFAULT_CONFIG  All tunable settings for the quaternion quadcopter +
% obstacle-avoidance project in ONE place. Edit here, not in the engine.

cfg.seed = 7;                     % random seed (noise is repeatable)

%% ---------------- vehicle (same numbers as your init_params.m) ----------
cfg.m   = 0.025;                  % [kg]
cfg.J   = [4.856e-3 4.856e-3 8.801e-3];   % [kg m^2]  (paper values)
cfg.g   = 9.81;
cfg.CD  = 0.8;  cfg.rho = 1.225;  cfg.A = 0.01425;   % drag, paper eq. (30)
cfg.Tmax = 3*cfg.m*cfg.g;         % thrust limit (3:1 thrust-to-weight)
cfg.droneRadius = 0.20;           % collision radius of the whole drone [m]

%% ---------------- timing -------------------------------------------------
cfg.dt = 0.005;                   % base step 200 Hz (attitude loop rate)
cfg.Ttraj = 55;                   % time to fly the reference [s]
cfg.tHold = 4;                    % extra time hovering at destination [s]
cfg.posLoopEvery = 4;             % position loop = 50 Hz  (slower outer loop)
cfg.sensorEvery  = 10;            % GPS + obstacle sensor = 20 Hz
cfg.planEvery    = 20;            % avoidance planner = 10 Hz

%% ---------------- mission + reference path -------------------------------
cfg.waypoints = [ ...
    1.80  1.95  2.10  2.30  2.50  2.70  2.90;
    2.27  2.40  2.55  2.70  2.82  2.92  3.00;
    0.60  0.70  0.82  0.95  1.05  1.12  1.20];
cfg.missionScale = 10;            % 1 = your original 1.1 m path. 10 leaves clear
                                  % gaps between obstacles (start stays at P0)

% noisy raw path -> least-squares B-spline (this is where noise is added)
cfg.refSamples    = 150;          % raw points sampled along the path
cfg.refNoiseStd   = 0.04;         % [m] random noise on every raw point
cfg.bsplineDegree = 5;            % B-spline degree (3 = cubic, 5 = quintic)
cfg.nCtrl         = 10;           % number of control points (>= degree+1)

%% ---------------- controller gains ---------------------------------------
% (paper gains Kp=60,Kd=6 and Kq=2000,Kd=10 are under-damped; these are
%  critically damped: Kd = 2*sqrt(Kp). Outer loop ~4 rad/s, inner ~30 rad/s)
cfg.Kp  = [16 16 16];   cfg.Kd  = [8 8 8];     cfg.Ki  = [2 2 2];
cfg.Kpq = [900 900 900]; cfg.Kdq = [60 60 60]; cfg.Kiq = [20 20 10];
cfg.q3r = 0;                      % reference for q3 (yaw-related), as paper
cfg.cmdFilterWn = 40;             % [rad/s] smooths reference quaternion

%% ---------------- disturbances + sensor noise ----------------------------
cfg.windStd   = 0.004;            % [N] random wind force (OU process)
cfg.gust      = [14 1.0 0.02 -0.015 0];  % [t0 duration Fx Fy Fz] one gust
cfg.gpsStd    = 0.02;             % [m]    GPS / Vicon position noise
cfg.accStd    = 0.05;             % [m/s^2] accelerometer noise
cfg.gyroStd   = 0.003;            % [rad/s] gyro noise
cfg.attStd    = 0.0005;           % quaternion (AHRS) noise

%% ---------------- obstacle SENSOR (LiDAR + camera, fused) -----------------
cfg.sensorRange = 2.5;            % [m] sees an obstacle only inside this
cfg.sensorStd   = 0.02;           % [m] noise on measured obstacle centre

%% ---------------- avoidance planner --------------------------------------
cfg.enableAvoidance = true;       % false = baseline (shows the collisions)
cfg.margin      = 0.15;           % [m] extra clearance beyond droneRadius
cfg.deltaWn     = 2.2;            % [rad/s] how fast the detour is flown
cfg.planHorizon = 2.5;            % [s] look-ahead
cfg.planStep    = 0.2;            % [s] look-ahead sample spacing
cfg.nDirs       = 64;             % candidate directions (all of 3D)
cfg.nMags       = 28;             % candidate detour sizes
cfg.maxOffset   = 1.6;            % [m] largest detour allowed
cfg.wVert       = 1.5;            % >1 : climbing/descending costs more
cfg.wPrev       = 0.6;            % <1 : returns to path when it is safe
cfg.slack       = 0.04;           % [m] a NEW decision needs this much spare clearance
cfg.wSwitch     = 0.10;           % fixed cost of changing decision (anti-flicker)
cfg.zMin        = 0.35;           % [m] never fly lower than this
cfg.zMax        = 11.0;            % [m] never fly higher than this

%% ---------------- world: buildings + moving obstacles ---------------------
P0 = cfg.waypoints(:,1);
Wp = P0 + cfg.missionScale*(cfg.waypoints - P0);        % scaled waypoints
P6 = Wp(:,end);
sg = sqrt(sum(diff(Wp,1,2).^2,1));  cs = [0 cumsum(sg)];  cs = cs/cs(end);
% point at fraction f of the path LENGTH (same polyline the B-spline is fitted to)
pp = @(f) [interp1(cs,Wp(1,:),f); interp1(cs,Wp(2,:),f); interp1(cs,Wp(3,:),f)];

% buildings: [cx cy size_x size_y z_bottom z_top]
a = pp(0.08);  b = pp(0.40);  c = pp(0.62);
zb = b(3);                                % path height at building B
zc = c(3);                                % path height at building C
cfg.buildings = [ ...
    a(1) a(2) 0.60 0.60 0.00 12.0;        % A: very tall tower ON the path -> must go AROUND
    b(1) b(2) 1.00 1.00 0.00 zb+0.05;     % B: wide low block, top at path height -> go OVER
    c(1) c(2) 1.20 1.20 zc-0.18 zc+1.6];  % C: floating sky-bridge, bottom just under path -> go UNDER

% moving obstacles (spheres): [fraction_along_path vx vy vz radius]
% placed so each one would hit the nominal path at that point if nothing changed
% (the fraction is converted to a time using the same time-law as the reference)
sm = @(x) 10*x.^3 - 15*x.^4 + 6*x.^5;
tof = @(f) cfg.Ttraj*bisect_inv(sm, f);
cfg.dyn = [ ...
    tof(0.24)  -0.12  0.18  0.00  0.18;   % D1: crosses sideways (between A and B)
    tof(0.82)   0.00  0.00 -0.20  0.18];  % D2: drops from above (after C)
end

function x = bisect_inv(fun, y)
lo = 0; hi = 1;
for it = 1:60
    mid = (lo+hi)/2;
    if fun(mid) < y, lo = mid; else, hi = mid; end
end
x = (lo+hi)/2;
end
