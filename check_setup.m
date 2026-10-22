function check_setup()
% CHECK_SETUP  Run this in the Command Window. It tells you which project
% files are missing, which are Live Scripts (.mlx) that must be replaced by
% plain .m files, and which names exist twice.
req = {'default_config','init_params_updated','qsl_lib','qsl_world','qsl_make_res','qsl_post', ...
       'qsl_scene','qsl_blk_ref','qsl_blk_sensors','qsl_blk_obstacles','qsl_blk_avoid','qsl_blk_pos', ...
       'qsl_blk_att','qsl_blk_anim','qsl_blk_guidance','qsl_blk_sensors_exist','qsl_blk_anim_exist', ...
       'summarize_run','plot_results'};
bad = 0;
for i = 1:numel(req)
    all_ = which(req{i}, '-all');
    if ischar(all_), all_ = {all_}; end
    if isempty(all_)
        fprintf('MISSING     %s.m   -> save it into your project folder\n', req{i});  bad = bad + 1;
    else
        isml = cellfun(@(p) numel(p) > 4 && strcmpi(p(end-3:end), '.mlx'), all_);
        if any(isml)
            fprintf('LIVE SCRIPT %s  -> delete the .mlx (keep only the plain .m)\n', req{i});  bad = bad + 1;
        elseif numel(all_) > 1
            fprintf('DUPLICATE   %s  (%d copies on the path):\n', req{i}, numel(all_));
            fprintf('              %s\n', all_{:});  bad = bad + 1;
        else
            fprintf('ok          %s\n', all_{1});
        end
    end
end
if bad == 0, fprintf('\nAll %d files found as plain .m files. You can press Run.\n', numel(req));
else, fprintf('\n%d problem(s) above. Fix them, then run check_setup again.\n', bad); end
end
