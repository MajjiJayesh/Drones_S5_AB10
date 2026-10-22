function qsl_post()
% Runs automatically when the Simulink simulation stops (model StopFcn):
% builds 'res' from the logged signals, prints the numbers, shows all plots.
% Your To Workspace block stores the data inside the simulation output
% (shown as out.qsl_log), so look there first; a plain variable qsl_log also works.
LOG = [];
if evalin('base','exist(''out'',''var'')')
    o = evalin('base','out');
    try, LOG = o.qsl_log; catch, LOG = []; end
end
if isempty(LOG) && evalin('base','exist(''qsl_log'',''var'')')
    LOG = evalin('base','qsl_log');
end
if isempty(LOG)
    warning('qsl_post:nolog', ['No logged data (out.qsl_log). The simulation stopped early - ' ...
            'fix the FIRST error in the Diagnostic Viewer; the plots appear after a complete run.']);
    return
end
LOG = squeeze(LOG);
W = qsl_world();  cfg = W.cfg;  want = 42 + 11*max(W.nObs,1);
if size(LOG,1) == want && size(LOG,2) ~= want, LOG = LOG.'; end   % 97 x N  ->  N x 97
if size(LOG,2) ~= want
    error('qsl_post:width', ['qsl_log has %d columns but %d are expected (42 + 11 x %d obstacles). ' ...
          'Check that all 13 LogPacker inputs are connected and that Fext0 is zeros(3,1).'], ...
          size(LOG,2), want, max(W.nObs,1));
end
if LOG(end,1) < 0.98*(cfg.Ttraj + cfg.tHold)
    warning('qsl_post:short', ['The log ends at t = %.1f s but a full run is %.0f s. Set the model Stop time to %.0f ' ...
            'and run again (this may also be an old log from a previous run).'], LOG(end,1), cfg.Ttraj+cfg.tHold, cfg.Ttraj+cfg.tHold);
    return
end
res = qsl_make_res(LOG);
assignin('base','res',res);
assignin('base','travelled',[res.L.t, res.L.p]);          % every point the drone travelled
summarize_run(res);
plot_results(res, 'results_simulink');
end
