function done = qsl_blk_anim(t, x, delta, Tr)
% BLOCK 8  Animation (runs every 0.05 s inside the simulation).
done = 0;
W = qsl_world();
k = round(t/W.cfg.dt) + 1;
if mod(k-1,10) == 0 || t >= W.cfg.Ttraj + W.cfg.tHold - W.cfg.dt   % 20 frames per second
    qsl_scene(t, x, delta, Tr);
end
end
