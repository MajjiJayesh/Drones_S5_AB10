function Tr = qsl_blk_obstacles(t, x)
% BLOCK 3  Obstacle sensor (LiDAR/camera style, range cfg.sensorRange, noisy
% centre measurement) + one Kalman filter per obstacle (constant velocity;
% buildings are static). Output Tr (11 x nObs), one column per obstacle:
%   row 1 valid | row 2 last time seen | rows 3-5 size | rows 6-11 [pos; vel]
persistent valid X P seen sz Tout
W = qsl_world();  cfg = W.cfg;  obs = W.obs;  n = max(W.nObs,1);
dt = cfg.dt;  dts = cfg.sensorEvery*dt;  k = round(t/dt) + 1;
if isempty(valid) || k == 1
    valid = false(n,1);  X = zeros(6,n);  P = zeros(6,6,n);
    seen = -inf(n,1);  sz = zeros(3,n);  Tout = zeros(11,n);
    Tout(2,:) = -1e9;
end
if mod(k-1, cfg.sensorEvery) == 0
    p = x(1:3);
    Fo = [eye(3) dts*eye(3); zeros(3) eye(3)];  Ho = [eye(3) zeros(3)];
    for j = 1:W.nObs
        tru = obs(j).p0 + obs(j).vel*t;
        surf = qsl_lib('obs_clearance', p', obs(j).type, tru', obs(j).size');
        sees = surf <= cfg.sensorRange;                       % inside sensor range?
        isStatic = (obs(j).type == 2);
        qv = 0.05; if isStatic, qv = 0; end
        Qo = diag([1e-4 1e-4 1e-4 qv*dts qv*dts qv*dts]);
        if valid(j)                                           % predict
            X(:,j) = Fo*X(:,j);
            if isStatic, X(4:6,j) = 0; end
            P(:,:,j) = Fo*P(:,:,j)*Fo' + Qo;
        end
        if sees                                               % measure
            zc = tru + cfg.sensorStd*randn(3,1);
            if ~valid(j)
                X(:,j) = [zc; 0;0;0];
                vv = 0.5^2; if isStatic, vv = 1e-6; end
                P(:,:,j) = diag([cfg.sensorStd^2*[1 1 1], vv*[1 1 1]]);
                sz(:,j) = obs(j).size + 0.01*abs(randn(3,1));
                valid(j) = true;
            else
                Ro = cfg.sensorStd^2*eye(3);
                So = Ho*P(:,:,j)*Ho' + Ro;
                Ko = P(:,:,j)*Ho'/So;
                X(:,j) = X(:,j) + Ko*(zc - Ho*X(:,j));
                P(:,:,j) = (eye(6) - Ko*Ho)*P(:,:,j);
            end
            seen(j) = t;
        end
    end
    for j = 1:W.nObs
        Tout(:,j) = [double(valid(j)); max(seen(j),-1e9); sz(:,j); X(:,j)];
    end
end
Tr = Tout;
end
