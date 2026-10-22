function done = qsl_blk_anim_exist(t, xi, q, delta, Tr)
% NEW BLOCK "Animation": live 3-D scene (buildings, moving obstacles, sensor,
% drone, every visited point). Uses the true plant position and attitude.
done = qsl_blk_anim(t, [xi(:); zeros(3,1); q(:); zeros(3,1)], delta, Tr);
end
