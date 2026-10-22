function [tau, qf] = qsl_blk_att(qm, wm, qr)
% BLOCK 6  Attitude loop (200 Hz): 2nd-order command filter (gives qr', qr'')
% + computed-torque control (paper eq. 26-27) -> body torques tau.
persistent qf_s qfd qfdd intEq t_count
W = qsl_world();  cfg = W.cfg;  dt = cfg.dt;  Jv = cfg.J(:);
if isempty(qf_s) || isempty(t_count)
    qf_s = zeros(3,1); qfd = zeros(3,1); qfdd = zeros(3,1); intEq = zeros(3,1); t_count = 0;
end
wf = cfg.cmdFilterWn;
qfdd = wf^2*(qr(:) - qf_s) - 2*wf*qfd;
qfd  = qfd + dt*qfdd;
qf_s = qf_s + dt*qfd;
qm = qm(:); wm = wm(:);
qb  = qm(2:4);
qdm = 0.5*[-qb'*wm; qm(1)*wm + cross(qb,wm)];
qbd = qdm(2:4);
eq  = qf_s - qb;  eqd = qfd - qbd;
intEq = min(max(intEq + eq*dt, -0.2), 0.2);
tp  = qfdd + cfg.Kpq(:).*eq + cfg.Kdq(:).*eqd + cfg.Kiq(:).*intEq;
Qm  = qsl_lib('Qmat', qb);
epsd = 1e-6;
Qdot_qd = (qsl_lib('Qmat', qb + epsd*qbd) - Qm)/epsd*qbd;
wdot_des = Qm*tp + Qdot_qd;
tau = Jv.*wdot_des + cross(wm, Jv.*wm);
qf = qf_s;
end
