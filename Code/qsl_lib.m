function varargout = qsl_lib(fn, varargin)
% QSL_LIB  Shared maths of the project (same code as run_quad_sim.m) so that
% every Simulink block can call it:  [..] = qsl_lib('ref_eval', ref, t)
if nargout == 0
    feval(fn, varargin{:});
else
    [varargout{1:nargout}] = feval(fn, varargin{:});
end
end

function seed_rng(s)
if exist('OCTAVE_VERSION','builtin')
    randn('state',s); rand('state',s);
else
    rng(s);
end
end

function R = quat2rot(q)
q0=q(1); q1=q(2); q2=q(3); q3=q(4);
R = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3);
     2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2)];       % paper eq. (2)
end

function R = qm_to_R(q)
R = quat2rot(q/norm(q));
end

function Q = Qmat(qb)
% omega = Q(qbar) * qbar_dot   (paper eq. (5), with q0 = sqrt(1-|qbar|^2))
q0 = sqrt(max(1 - qb'*qb, 0.04));
M = [ q0   qb(3) -qb(2);
     -qb(3) q0    qb(1);
      qb(2) -qb(1) q0 ];
Q = 2*(M + (qb*qb')/q0);
end

function xd = plant_deriv(x, Tc, tau, par)
v = x(4:6); q = x(7:10); w = x(11:13);
R = quat2rot(q);
Fd = -par.k*norm(v)*v;                                   % drag, eq. (30)
vd = [0;0;-par.g] + (Tc/par.m)*R(:,3) + (Fd + par.F)/par.m;      % eq. (9)
qd = 0.5*[-q(2:4)'*w; q(1)*w + cross(q(2:4),w)];                % eq. (3)
wd = (tau - cross(w, par.J.*w))./par.J;                          % eq. (10)
xd = [v; vd; qd; wd];
end

function [T, qbar_r] = flatness_map(a, q3r, m, g)
% Paper eq. (16)-(17) / Appendix A: desired acceleration -> thrust + q_ref
N = max(norm([a(1); a(2); a(3)+g]), 0.3*g);
T = m*N;
inner = max(1 + (a(3)+g)/N - 2*q3r^2, 0);
q0 = sqrt(inner/2);
den = max(N + a(3) + g, 1e-6);
q1 = (a(1)*q3r - q0*a(2))/den;
q2 = (a(2)*q3r + q0*a(1))/den;
qbar_r = [q1; q2; q3r];
end

%% --------------------------------------------------------- reference path
function ref = make_reference(cfg)
P = cfg.waypoints;  P0 = P(:,1);
P = P0 + cfg.missionScale*(P - P0);
seg = sqrt(sum(diff(P,1,2).^2,1));  cs = [0 cumsum(seg)];
s = linspace(0, cs(end), cfg.refSamples);
D = zeros(cfg.refSamples,3);
for i = 1:3, D(:,i) = interp1(cs, P(i,:), s)'; end
noise = cfg.refNoiseStd*randn(size(D));
noise([1 end],:) = 0;                                % start / goal stay exact
Dn = D + noise;                                      % NOISY raw reference path

p = cfg.bsplineDegree;  n = cfg.nCtrl;
d = [0; cumsum(sqrt(sum(diff(Dn).^2,2)))];  u = d/d(end);       % chord-length params
nint = n - p - 1;                                                % interior knots
t = [zeros(1,p+1), (1:nint)/(nint+1), ones(1,p+1)];              % clamped uniform
Nm = zeros(numel(u), n);
for i = 1:numel(u), Nm(i,:) = bs_basis_all(t, p, u(i)); end
Rhs = Dn(2:end-1,:) - Nm(2:end-1,1)*Dn(1,:) - Nm(2:end-1,n)*Dn(end,:);
Cint = Nm(2:end-1,2:n-1) \ Rhs;                                  % least squares
C0 = [Dn(1,:); Cint; Dn(end,:)];

[C1,t1,p1] = bs_deriv_ctrl(C0, t,  p);
[C2,t2,p2] = bs_deriv_ctrl(C1, t1, p1);
ref.C0 = C0; ref.C1 = C1; ref.C2 = C2;
ref.t0 = t;  ref.t1 = t1; ref.t2 = t2;
ref.p0 = p;  ref.p1 = p1; ref.p2 = p2;
ref.T  = cfg.Ttraj;
ref.raw = Dn;  ref.ideal = D;  ref.waypoints = P;
end

function [pos, vel, acc] = ref_eval(ref, t)
T = ref.T;  tau = min(max(t/T, 0), 1);
u   = 10*tau^3 - 15*tau^4 + 6*tau^5;                 % smootherstep time law:
ud  = (30*tau^2 - 60*tau^3 + 30*tau^4)/T;            % zero speed + zero accel
udd = (60*tau - 180*tau^2 + 120*tau^3)/T^2;          % at start and at goal
c0 = (bs_basis_all(ref.t0, ref.p0, u)*ref.C0)';
c1 = (bs_basis_all(ref.t1, ref.p1, u)*ref.C1)';
c2 = (bs_basis_all(ref.t2, ref.p2, u)*ref.C2)';
pos = c0;
vel = c1*ud;
acc = c2*ud^2 + c1*udd;
end

function N = bs_basis_all(t, p, u)
% All B-spline basis functions of degree p at parameter u (Piegl & Tiller A2.2)
n = numel(t) - p - 1;
N = zeros(1,n);
if u >= t(n+1)
    k = n;
else
    k = find(t <= u, 1, 'last');
    k = min(max(k, p+1), n);
end
left = zeros(1,p+1); right = zeros(1,p+1); Nl = zeros(1,p+1); Nl(1) = 1;
for j = 1:p
    left(j+1)  = u - t(k+1-j);
    right(j+1) = t(k+j) - u;
    saved = 0;
    for r = 0:j-1
        temp = Nl(r+1)/(right(r+2) + left(j-r+1));
        Nl(r+1) = saved + right(r+2)*temp;
        saved = left(j-r+1)*temp;
    end
    Nl(j+1) = saved;
end
N(k-p:k) = Nl;
end

function [Cd, td, pd] = bs_deriv_ctrl(C, t, p)
% control points of the derivative curve
n = size(C,1);
Cd = zeros(n-1, size(C,2));
for i = 1:n-1
    Cd(i,:) = p*(C(i+1,:) - C(i,:))/(t(i+p+1) - t(i+1));
end
td = t(2:end-1);  pd = p - 1;
end

%% ------------------------------------------------------------- obstacles
function obs = build_obstacles(cfg, ref)
obs = struct('type',{},'p0',{},'vel',{},'size',{});
for i = 1:size(cfg.buildings,1)
    b = cfg.buildings(i,:);                 % [cx cy sx sy z0 z1]
    obs(end+1).type = 2;                    %#ok<AGROW>
    obs(end).p0   = [b(1); b(2); (b(5)+b(6))/2];
    obs(end).vel  = zeros(3,1);
    obs(end).size = [b(3)/2; b(4)/2; (b(6)-b(5))/2];        % half-extents
end
for i = 1:size(cfg.dyn,1)
    d = cfg.dyn(i,:);                       % [t_hit vx vy vz r]
    [ph,~,~] = ref_eval(ref, d(1));
    vel = d(2:4)';
    obs(end+1).type = 1;                    %#ok<AGROW>
    obs(end).p0   = ph - vel*d(1);          % would hit the path at t_hit
    obs(end).vel  = vel;
    obs(end).size = [d(5); 0; 0];           % radius
end
end

function c = obs_clearance(P, type, ctr, sz)
% distance from points P (K x 3) to the obstacle SURFACE (negative = inside)
if type == 1
    c = sqrt(sum((P - ctr).^2, 2)) - sz(1);
else
    q = abs(P - ctr) - sz;                  % broadcast: K x 3
    outside = sqrt(sum(max(q,0).^2, 2));
    inside  = min(max(q(:,1), max(q(:,2), q(:,3))), 0);
    c = outside + inside;
end
end

%% ---------------------------------------------------------- the planner
function pl = make_planner(cfg)
wn = cfg.deltaWn;  h = cfg.planStep;  K = round(cfg.planHorizon/h);
A = [0 1 0; 0 0 1; -wn^3 -3*wn^2 -3*wn];  B = [0; 0; wn^3];
Phi = expm(A*h);  Gam = (A\(Phi - eye(3)))*B;
Ainit = zeros(K,3);  b = zeros(K,1);  Pk = eye(3);  acc = zeros(3,1);
for k = 1:K
    acc = Phi*acc + Gam;  Pk = Phi*Pk;
    Ainit(k,:) = Pk(1,:);  b(k) = acc(1);
end
M = cfg.nDirs;  i = (0:M-1)';                % evenly spread directions on a sphere
zc = 1 - 2*(i+0.5)/M;  rr = sqrt(1 - zc.^2);  th = pi*(3-sqrt(5))*i;
dirs = [rr.*cos(th), rr.*sin(th), zc];
mags = linspace(cfg.maxOffset/cfg.nMags, cfg.maxOffset, cfg.nMags);
Dc = zeros(1,3);                             % first candidate = "no detour"
for im = 1:numel(mags), Dc = [Dc; mags(im)*dirs]; end %#ok<AGROW>
pl.K = K;  pl.Ainit = Ainit;  pl.b = b;  pl.Dc = Dc;
pl.tau = (1:K)'*h;
end

function [dstar, feasible, bestClr] = plan_offset(pl, cfg, refpts, ds, list, dprev, rho)
% Choose the detour vector d* (any direction in 3-D) with the lowest cost
% such that the predicted path keeps 'rho' clearance from every obstacle.
%
% Anti-flicker (Schmitt trigger):
%   * the PREVIOUS decision is re-evaluated exactly and may stay while its
%     clearance is >= 0
%   * any NEW decision must have extra slack (cfg.slack) and pays a fixed
%     switching cost (cfg.wSwitch), so noise cannot flip it back and forth
%   * "no detour" (return to the path) is free to choose once it is safe
Dc = [pl.Dc(1,:); dprev'; pl.Dc(2:end,:)];      % row1 = none, row2 = previous
Nc = size(Dc,1);  K = pl.K;
D0 = pl.Ainit*ds';                              % K x 3 free response of detour filter
minClr = inf(Nc,1);
for k = 1:K
    Pk = refpts(k,:) + D0(k,:) + pl.b(k)*Dc;                 % Nc x 3 predicted path
    for j = 1:numel(list)
        c = list(j).c + list(j).v*pl.tau(k);
        cl = obs_clearance(Pk, list(j).type, c, list(j).size) - rho;
        minClr = min(minClr, cl);
    end
    minClr = min(minClr, Pk(:,3) - cfg.zMin);                % floor
    minClr = min(minClr, cfg.zMax - Pk(:,3));                % ceiling
end
W = [1 1 cfg.wVert];
cost = sum((Dc.^2).*W, 2) + cfg.wPrev*sum((Dc - dprev').^2, 2) + 1e-6*(1:Nc)';
cost(3:end) = cost(3:end) + cfg.wSwitch;                     % changing decision costs extra
cost(2) = sum((dprev.^2).*W');                               % keeping it is free
allowed = minClr >= cfg.slack;
allowed(2) = minClr(2) >= 0;                                 % previous: only needs to be safe
if ~any(allowed), allowed = minClr >= 0; end                 % relax if nothing has slack
if any(allowed)
    cost(~allowed) = inf;
    [~, idx] = min(cost);
    feasible = true;
else
    [~, idx] = max(minClr);                                  % emergency: best effort
    feasible = false;
end
dstar = Dc(idx,:)';
bestClr = minClr(idx);
end

function s = label_decision(d, vref)
h = vref; h(3) = 0;
if norm(h) < 1e-6, h = [1;0;0]; end
h = h/norm(h);
left = cross([0;0;1], h);
lat = dot(d, left);  up = d(3);  thr = 0.15;
parts = {};
if lat >  thr, parts{end+1} = 'LEFT';  end %#ok<AGROW>
if lat < -thr, parts{end+1} = 'RIGHT'; end %#ok<AGROW>
if up  >  thr, parts{end+1} = 'UP';    end %#ok<AGROW>
if up  < -thr, parts{end+1} = 'DOWN';  end %#ok<AGROW>
if isempty(parts)
    s = 'STRAIGHT';
else
    s = parts{1};
    for i = 2:numel(parts), s = [s '+' parts{i}]; end %#ok<AGROW>
end
end

function R = qrot_public(q)
R = quat2rot(q);
end

function obs = obs_make(cfg, ref)
obs = build_obstacles(cfg, ref);
end
