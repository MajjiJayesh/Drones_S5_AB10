function [Tc, qr] = qsl_blk_pos(t, pc, vc, ac, ph, vh)
% BLOCK 5  Position loop (50 Hz): feedback linearisation (paper eq. 20) +
% differential flatness (eq. 16-17) -> total thrust Tc and reference
% quaternion vector qr = [q1r q2r q3r].
persistent intE Tc_h qr_h
W = qsl_world();  cfg = W.cfg;  dt = cfg.dt;  g = cfg.g;  m = cfg.m;
k = round(t/dt) + 1;
if isempty(intE) || k == 1
    intE = zeros(3,1);  Tc_h = m*g;  qr_h = zeros(3,1);
end
if mod(k-1, cfg.posLoopEvery) == 0
    dtp = cfg.posLoopEvery*dt;
    e = pc(:) - ph(:);  ed = vc(:) - vh(:);
    intE = min(max(intE + e*dtp, -0.5), 0.5);
    astar = ac(:) + cfg.Kp(:).*e + cfg.Kd(:).*ed + cfg.Ki(:).*intE;
    axy = norm(astar(1:2));
    if axy > 4, astar(1:2) = astar(1:2)*4/axy; end
    astar(3) = max(astar(3), -0.7*g);
    [Treq, qr_h] = qsl_lib('flatness_map', astar, cfg.q3r, m, g);
    Tc_h = min(max(Treq, 0), cfg.Tmax);
end
Tc = Tc_h;  qr = qr_h;
end
