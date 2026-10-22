function res = qsl_make_res(LOG)
% QSL_MAKE_RES  Turns the logged matrix of the Simulink run into the same
% 'res' structure that plot_results / summarize_run use.
%   res = qsl_make_res(qsl_log);        % qsl_log = To Workspace variable
W = qsl_world();  cfg = W.cfg;  obs = W.obs;  nO = max(W.nObs,1);  N = size(LOG,1);
idx = cumsum([1 1 13 3 3 3 1 3 3 3 3 3 3 11*nO]);
cols = @(a) LOG(:, idx(a):idx(a+1)-1);
L.t = cols(1);  X = cols(2);
L.p = X(:,1:3);  L.v = X(:,4:6);  L.q = X(:,7:10);  L.w = X(:,11:13);
L.pr = cols(3);  L.pc = cols(4);  L.delta = cols(5);  L.T = cols(6);  L.tau = cols(7);
L.ph = cols(8);  L.vh = cols(9);  qf = cols(10);  L.wind = cols(11);  L.gps = cols(12);
L.qref = [sqrt(max(1 - sum(qf.^2,2),0)), qf];
Trf = cols(13);
L.obsPos = zeros(N,3,nO);  L.clr = zeros(N,nO);  L.det = false(N,nO);  L.trkPos = nan(N,3,nO);
for j = 1:W.nObs
    P = obs(j).p0' + L.t*obs(j).vel';
    L.obsPos(:,:,j) = P;
    for k = 1:N
        L.clr(k,j) = qsl_lib('obs_clearance', L.p(k,:), obs(j).type, P(k,:), obs(j).size') - cfg.droneRadius;
    end
    L.det(:,j) = (L.clr(:,j) + cfg.droneRadius) <= cfg.sensorRange;
    base = (j-1)*11;
    ok = Trf(:,base+1) > 0.5;
    tp = Trf(:,base+6:base+8);  tp(~ok,:) = nan;
    L.trkPos(:,:,j) = tp;
end
L.decision = cell(N,1);  ev = {};  last = '';
for k = 1:N
    [~,vr] = qsl_lib('ref_eval', W.ref, L.t(k));
    if mod(k-1, cfg.planEvery) == 0 || isempty(last)
        s = qsl_lib('label_decision', L.delta(k,:)', vr);
        if ~strcmp(s,last), ev{end+1} = sprintf('t=%5.1f s : %s', L.t(k), s); end %#ok<AGROW>
        last = s;
    end
    L.decision{k} = last;
end
res.cfg = cfg;  res.ref = W.ref;  res.obs = obs;  res.L = L;  res.t = L.t;  res.events = ev;
res.pDest = W.pDest;  res.nObs = W.nObs;  res.rho = W.rho;
res.minClr = min(L.clr(:,1:nO),[],1);
res.rmse_cmd = sqrt(mean(sum((L.p - L.pc).^2,2)));
res.rmse_nom = sqrt(mean(sum((L.p - L.pr).^2,2)));
okg = ~isnan(L.gps(:,1));
res.rmse_gps = sqrt(mean(sum((L.gps(okg,:) - L.p(okg,:)).^2,2)));
res.rmse_kf  = sqrt(mean(sum((L.ph - L.p).^2,2)));
res.finalErr = norm(L.p(end,:)' - res.pDest);
res.maxDelta = max(sqrt(sum(L.delta.^2,2)));
res.collided = any(res.minClr < 0);
end
