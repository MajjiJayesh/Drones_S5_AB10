function [delta, deltad, deltadd, dstar] = qsl_blk_avoid(t, Tr)
% BLOCK 4  3-D avoidance planner (10 Hz) + 3rd-order detour filter (every step).
% The planner searches detour vectors d* over ~64 directions (left, right,
% up, down, diagonals) x 28 sizes and picks the cheapest that keeps the
% predicted path clear. The filter makes the detour smooth. Output = how far
% the reference is shifted (delta) and its first two derivatives.
persistent ds dstar_p
W = qsl_world();  cfg = W.cfg;  dt = cfg.dt;  k = round(t/dt) + 1;
if isempty(ds) || k == 1
    ds = zeros(3,3);  dstar_p = zeros(3,1);
end
if mod(k-1, cfg.planEvery) == 0
    if cfg.enableAvoidance
        list = struct('type',{},'c',{},'v',{},'size',{});
        for j = 1:W.nObs
            if Tr(1,j) > 0.5 && (t - Tr(2,j)) < 1.5
                list(end+1) = struct('type',W.obs(j).type, ...
                    'c',Tr(6:8,j)','v',Tr(9:11,j)','size',Tr(3:5,j)'); %#ok<AGROW>
            end
        end
        refpts = zeros(W.pl.K,3);
        for kk = 1:W.pl.K
            rp = qsl_lib('ref_eval', W.ref, t + kk*cfg.planStep);
            refpts(kk,:) = rp';
        end
        dstar_p = qsl_lib('plan_offset', W.pl, cfg, refpts, ds, list, dstar_p, W.rho);
    else
        dstar_p = zeros(3,1);
    end
end
wn = cfg.deltaWn;
for i = 1:3
    d0 = ds(i,1); d1 = ds(i,2); d2 = ds(i,3);
    ds(i,3) = d2 + dt*(wn^3*(dstar_p(i)-d0) - 3*wn^2*d1 - 3*wn*d2);
    ds(i,2) = d1 + dt*d2;
    ds(i,1) = d0 + dt*d1;
end
delta = ds(:,1);  deltad = ds(:,2);  deltadd = ds(:,3);  dstar = dstar_p;
end
