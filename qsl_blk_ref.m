function [pr, vr, ar] = qsl_blk_ref(t)
% BLOCK 1  Reference trajectory: noisy path samples -> B-spline (built once in
% qsl_world), evaluated at time t with a smootherstep time law.
W = qsl_world();
[pr, vr, ar] = qsl_lib('ref_eval', W.ref, t);
end
