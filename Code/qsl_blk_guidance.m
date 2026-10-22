function [xi_c, xi_c_dot, xi_c_ddot, delta, Tr] = qsl_blk_guidance(t, xi, xi_r, xi_r_dot, xi_r_ddot)
% NEW BLOCK "ObstacleAvoidance" (sits between RefTrajectory and PositionController).
% Sensor + Kalman tracks of the obstacles -> 3-D avoidance planner -> smooth
% detour 'delta'. The PositionController then follows  xi_c = xi_r + delta,
% so when nothing is in the way delta = 0 and the drone flies the B-spline.
Tr = qsl_blk_obstacles(t, [xi(:); zeros(10,1)]);
[delta, deltad, deltadd, ~] = qsl_blk_avoid(t, Tr);
xi_c      = xi_r(:)      + delta;
xi_c_dot  = xi_r_dot(:)  + deltad;
xi_c_ddot = xi_r_ddot(:) + deltadd;
end
