function W = qsl_world(action)
% QSL_WORLD  Builds (once) and caches everything the blocks share: settings,
% noisy B-spline reference, obstacles, planner tables.
%   W = qsl_world();          % get the world (built on first call)
%   qsl_world('reset');       % forget it (done at every simulation start)
persistent Wc
if nargin > 0 && strcmp(action,'reset'), Wc = []; if nargout == 0, return; end, end
if isempty(Wc)
    cfg = default_config();
    qsl_lib('seed_rng', cfg.seed);
    ref = qsl_lib('make_reference', cfg);
    obs = qsl_lib('build_obstacles', cfg, ref);
    Wc.cfg = cfg;  Wc.ref = ref;  Wc.obs = obs;  Wc.nObs = numel(obs);
    Wc.pl  = qsl_lib('make_planner', cfg);
    Wc.rho = cfg.droneRadius + cfg.margin;
    p0 = qsl_lib('ref_eval', ref, 0);
    Wc.p0 = p0;
    Wc.x0 = [p0; 0;0;0; 1;0;0;0; 0;0;0];
    Wc.pDest = qsl_lib('ref_eval', ref, cfg.Ttraj);
end
W = Wc;
end
