function plot_results(res, saveDir)
% PLOT_RESULTS  All the plots shown after the drone reaches the destination.
%
%   plot_results(res)             % just show them
%   plot_results(res, 'results')  % also save every figure as PNG + CSV of points
%
% Figures:  1 reference generation (noise -> B-spline)   2 x,y,z tracking
%           3 3-D tracking + visited points              4 thrust + torques
%           5 angular velocity                           6 quaternion + Euler angles
%           7 avoidance (detour, clearance)              8 Kalman filter blending

if nargin < 2, saveDir = ''; end
cfg = res.cfg;  L = res.L;  t = L.t;  ref = res.ref;
vis = 'on'; if ~isempty(getenv('QUAD_HEADLESS')), vis = 'off'; end
active = sqrt(sum(L.delta.^2,2)) > 0.15;         % avoidance manoeuvre in progress
names = obstacle_labels(cfg);
C = lines(8);

%% ---- 1. reference generation: noisy samples -> B-spline ------------------
f1 = figure('Name','1 Reference path generation','Color','w','Visible',vis);
uu = linspace(0,1,400);  cur = zeros(400,3);
for i = 1:400
    cur(i,:) = (bs_basis_all_pub(ref.t0, ref.p0, uu(i))*ref.C0);
end
plot3(ref.raw(:,1),ref.raw(:,2),ref.raw(:,3),'.','Color',[0.6 0.6 0.6],'MarkerSize',8); hold on;
plot3(ref.ideal(:,1),ref.ideal(:,2),ref.ideal(:,3),'k:','LineWidth',1);
plot3(cur(:,1),cur(:,2),cur(:,3),'-','Color',C(1,:),'LineWidth',2.5);
plot3(ref.C0(:,1),ref.C0(:,2),ref.C0(:,3),'s--','Color',C(2,:),'MarkerFaceColor',C(2,:));
plot3(ref.waypoints(1,:),ref.waypoints(2,:),ref.waypoints(3,:),'p','Color','k', ...
      'MarkerFaceColor',[1 0.8 0],'MarkerSize',11);
grid on; axis equal; view(35,22); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Reference: %d noisy samples (\\sigma=%.2f m)  ->  degree-%d B-spline, %d control points', ...
      cfg.refSamples, cfg.refNoiseStd, cfg.bsplineDegree, cfg.nCtrl));
legend('noisy raw samples','ideal polyline','B-spline (smooth, used as reference)', ...
       'control points','waypoints P0-P6','Location','best');

%% ---- 2. x, y, z tracking ------------------------------------------------------
f2 = figure('Name','2 Position tracking x,y,z','Color','w','Visible',vis,'Position',[100 60 900 760]);
lab = {'x (m)','y (m)','z (m)'};
for i = 1:3
    subplot(3,1,i); hold on; shade(t, active);
    plot(t, L.pr(:,i), '--', 'Color',[0.45 0.45 0.45], 'LineWidth',1.5);
    plot(t, L.pc(:,i), '-',  'Color',[0.3 0.6 0.95],  'LineWidth',1.2);
    plot(t, L.p(:,i),  '-',  'Color',[0.85 0.25 0.1], 'LineWidth',1.6);
    ylabel(lab{i}); grid on; xlim([t(1) t(end)]);
    if i == 1
        title('Position tracking (shaded = obstacle avoidance in progress)');
        legend('avoidance active','nominal reference','commanded (reference + detour)','actual','Location','best');
    end
end
xlabel('time (s)');

%% ---- 3. 3-D tracking with obstacles and every visited point ------------------
f3 = figure('Name','3 Quadcopter 3-D tracking','Color','w','Visible',vis,'Position',[120 80 1000 780]);
hold on; grid on; axis equal; view(38,24);
draw_world(gca, res);
hn = plot3(L.pr(:,1),L.pr(:,2),L.pr(:,3),'--','Color',[0.4 0.4 0.4],'LineWidth',1.8);
ha = plot3(L.p(:,1),L.p(:,2),L.p(:,3),'-','Color',[0.85 0.25 0.1],'LineWidth',2);
sk = 1:max(1,round(0.05/cfg.dt)):numel(t);
hp = plot3(L.p(sk,1),L.p(sk,2),L.p(sk,3),'.','Color',[0.1 0.4 0.9],'MarkerSize',9);
h0 = plot3(L.p(1,1),L.p(1,2),L.p(1,3),'o','MarkerSize',10,'MarkerFaceColor',[0.1 0.8 0.1],'Color','k');
h1 = plot3(res.pDest(1),res.pDest(2),res.pDest(3),'p','MarkerSize',14,'MarkerFaceColor','r','Color','k');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3-D tracking: every point the drone travelled through (dots)');
legend([hn ha hp h0 h1],{'nominal reference','actual path','visited points (every 0.05 s)','start','destination'}, ...
       'Location','northeast');

%% ---- 4. thrust and control torques -------------------------------------------
f4 = figure('Name','4 Total thrust and control torque','Color','w','Visible',vis,'Position',[140 60 900 780]);
subplot(4,1,1); hold on; shade(t,active);
plot(t, L.T, 'LineWidth',1.4,'Color',C(1,:)); plot(t, cfg.m*cfg.g*ones(size(t)),'k:');
ylabel('thrust T (N)'); grid on; xlim([t(1) t(end)]);
title('Total thrust (dotted = weight mg) and control torques');
tl = {'\tau_x (N m)','\tau_y (N m)','\tau_z (N m)'};
for i = 1:3
    subplot(4,1,i+1); hold on; shade(t,active);
    plot(t, L.tau(:,i), 'LineWidth',1.2,'Color',C(i+1,:)); ylabel(tl{i}); grid on; xlim([t(1) t(end)]);
end
xlabel('time (s)');

%% ---- 5. angular velocity --------------------------------------------------------
f5 = figure('Name','5 Angular velocity','Color','w','Visible',vis,'Position',[160 60 900 700]);
wl = {'\omega_x (rad/s)','\omega_y (rad/s)','\omega_z (rad/s)'};
for i = 1:3
    subplot(3,1,i); hold on; shade(t,active);
    plot(t, L.w(:,i), 'LineWidth',1.3,'Color',C(i,:)); ylabel(wl{i}); grid on; xlim([t(1) t(end)]);
end
subplot(3,1,1); title('Angular velocity of the body'); xlabel('');  subplot(3,1,3); xlabel('time (s)');

%% ---- 6. quaternion tracking + Euler angles -----------------------------------
f6 = figure('Name','6 Attitude: quaternion and Euler angles','Color','w','Visible',vis,'Position',[180 40 1000 820]);
ql = {'q_0','q_1','q_2','q_3'};
for i = 1:4
    subplot(4,2,2*i-1); hold on; shade(t,active);
    plot(t, L.qref(:,i), '--','Color',[0.4 0.4 0.4],'LineWidth',1.3);
    plot(t, L.q(:,i), '-','Color',C(i,:),'LineWidth',1.3);
    ylabel(ql{i}); grid on; xlim([t(1) t(end)]);
    if i == 1, title('Quaternion: reference vs actual'); end
end
xlabel('time (s)');
q = L.q;
roll  = atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)), 1-2*(q(:,2).^2+q(:,3).^2));       % paper eq. (11)
pitch = asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,2).*q(:,4)))));
yaw   = atan2(2*(q(:,1).*q(:,4)+q(:,2).*q(:,3)), 1-2*(q(:,3).^2+q(:,4).^2));
eu = [roll pitch yaw]*180/pi;  el = {'roll \phi (deg)','pitch \theta (deg)','yaw \psi (deg)'};
for i = 1:3
    subplot(4,2,2*i); hold on; shade(t,active);
    plot(t, eu(:,i),'LineWidth',1.3,'Color',C(i+4,:)); ylabel(el{i}); grid on; xlim([t(1) t(end)]);
    if i == 1, title('Euler angles (from quaternion)'); end
end
xlabel('time (s)');

%% ---- 7. avoidance: detour, clearance, decision --------------------------------
f7 = figure('Name','7 Obstacle avoidance','Color','w','Visible',vis,'Position',[200 60 950 800]);
subplot(3,1,1); hold on;
plot(t, L.delta(:,1),'LineWidth',1.4); plot(t, L.delta(:,2),'LineWidth',1.4); plot(t, L.delta(:,3),'LineWidth',1.4);
plot(t, sqrt(sum(L.delta.^2,2)),'k--','LineWidth',1.2);
grid on; xlim([t(1) t(end)]); ylabel('detour (m)');
legend('\delta_x','\delta_y','\delta_z','|\delta|','Location','best');
title('Detour added to the reference (returns to 0 = back on original path)');
subplot(3,1,2); hold on;
for j = 1:res.nObs, plot(t, L.clr(:,j), 'LineWidth',1.3,'Color',C(mod(j-1,8)+1,:)); end
plot(t, zeros(size(t)), 'r-', 'LineWidth',1.5);
grid on; xlim([t(1) t(end)]); ylabel('clearance (m)'); ylim([-0.1 min(6, max(L.clr(:))+0.2)]);
legend([names {'collision (0)'}],'Location','northeast');
title('Distance from the drone body to every obstacle surface (must stay above 0)');
subplot(3,1,3); hold on;
sp = sqrt(sum(L.v.^2,2));  plot(t, sp, 'LineWidth',1.4,'Color',C(1,:));
dd = zeros(size(t)); for j = 1:res.nObs, dd = dd + L.det(:,j); end
stairs(t, dd*max(sp)/max(1,res.nObs), 'Color',C(2,:),'LineWidth',1.2);
grid on; xlim([t(1) t(end)]); ylabel('speed (m/s)'); xlabel('time (s)');
legend('speed','# obstacles inside sensor range (scaled)','Location','best');

%% ---- 8. Kalman filter: blending prediction and measurement -------------------
f8 = figure('Name','8 Kalman filter blending','Color','w','Visible',vis,'Position',[220 60 950 640]);
ok = ~isnan(L.gps(:,1));
subplot(2,1,1); hold on;
plot(t(ok), sqrt(sum((L.gps(ok,:)-L.p(ok,:)).^2,2)), '.', 'Color',[0.8 0.4 0.2]);
plot(t, sqrt(sum((L.ph-L.p).^2,2)), '-', 'Color',[0.1 0.4 0.8], 'LineWidth',1.3);
grid on; xlim([t(1) t(end)]); ylabel('position error (m)');
legend(sprintf('raw GPS measurement  (RMS %.3f m)',res.rmse_gps), ...
       sprintf('Kalman estimate = prediction blended with GPS  (RMS %.3f m)',res.rmse_kf),'Location','best');
title('Own-position estimate: IMU prediction + GPS correction');
subplot(2,1,2); hold on;
for j = 1:res.nObs
    e = sqrt(sum((squeeze(L.trkPos(:,:,j)) - L.obsPos(:,:,j)).^2,2));
    plot(t, e, 'LineWidth',1.3, 'Color',C(mod(j-1,8)+1,:));
end
grid on; xlim([t(1) t(end)]); ylabel('obstacle position error (m)'); xlabel('time (s)');
legend(names,'Location','best'); title('Obstacle tracks: Kalman-filtered estimate vs truth (only once detected)');

%% ---- save ---------------------------------------------------------------------------
if ~isempty(saveDir)
    if ~exist(saveDir,'dir'), mkdir(saveDir); end
    figs = {f1,f2,f3,f4,f5,f6,f7,f8};
    nm = {'1_reference_bspline','2_xyz_tracking','3_trajectory_3D','4_thrust_torque', ...
          '5_angular_velocity','6_attitude','7_avoidance','8_kalman'};
    for i = 1:numel(figs)
        print(figs{i}, fullfile(saveDir, [nm{i} '.png']), '-dpng', '-r110');
    end
    % every point the drone travelled through (time, x, y, z, speed)
    pts = [t, L.p, sqrt(sum(L.v.^2,2))];
    fn = fullfile(saveDir,'travelled_points.csv');
    fid = fopen(fn,'w'); fprintf(fid,'time_s,x_m,y_m,z_m,speed_mps\n'); fclose(fid);
    dlmwrite(fn, pts(1:round(0.05/cfg.dt):end,:), '-append', 'delimiter', ',', 'precision', '%.4f');
    fprintf('Saved 8 figures and travelled_points.csv in %s\n', saveDir);
end
end

%% ------------------------------------------------------------------ helpers
function shade(t, mask)
% light grey bands where an avoidance manoeuvre is active
yl = get(gca,'YLim');  d = diff([0; mask(:); 0]);  s = find(d == 1);  e = find(d == -1) - 1;
for i = 1:numel(s)
    patch([t(s(i)) t(e(i)) t(e(i)) t(s(i))], [yl(1) yl(1) yl(2) yl(2)], [0.85 0.88 0.95], ...
          'EdgeColor','none','FaceAlpha',0.55,'HandleVisibility', ifelse(i==1,'on','off'));
end
end

function out = ifelse(c,a,b)
if c, out = a; else, out = b; end
end

function names = obstacle_labels(cfg)
names = {};
bl = {'Building A','Building B','Building C','Building D','Building E'};
for i = 1:size(cfg.buildings,1), names{end+1} = bl{min(i,5)}; end %#ok<AGROW>
for i = 1:size(cfg.dyn,1), names{end+1} = sprintf('Moving D%d',i); end %#ok<AGROW>
end

function draw_world(ax, res)
% static buildings as boxes (used by the 3-D result plot)
cfg = res.cfg;  F = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8];
for i = 1:size(cfg.buildings,1)
    b = cfg.buildings(i,:);  hx = b(3)/2; hy = b(4)/2;
    V = [b(1)-hx b(2)-hy b(5); b(1)+hx b(2)-hy b(5); b(1)+hx b(2)+hy b(5); b(1)-hx b(2)+hy b(5); ...
         b(1)-hx b(2)-hy b(6); b(1)+hx b(2)-hy b(6); b(1)+hx b(2)+hy b(6); b(1)-hx b(2)+hy b(6)];
    patch('Parent',ax,'Vertices',V,'Faces',F,'FaceColor',[0.62 0.66 0.74],'FaceAlpha',0.45, ...
          'EdgeColor',[0.3 0.3 0.35],'HandleVisibility','off');
end
[sx,sy,sz] = sphere(14);
for j = 1:res.nObs
    if res.obs(j).type == 1
        r = res.obs(j).size(1);  P = squeeze(res.L.obsPos(:,:,j));
        plot3(ax,P(:,1),P(:,2),P(:,3),':','Color',[0.9 0.4 0.2],'HandleVisibility','off');
        k = round(size(P,1)*0.55);  c = P(k,:);
        surf(ax, sx*r+c(1), sy*r+c(2), sz*r+c(3), 'FaceColor',[0.95 0.45 0.25], ...
             'EdgeColor','none','FaceAlpha',0.7,'HandleVisibility','off');
    end
end
end

function N = bs_basis_all_pub(t, p, u)
% (same Cox-de Boor routine as in run_quad_sim.m, used only to draw the curve)
n = numel(t) - p - 1;  N = zeros(1,n);
if u >= t(n+1), k = n; else, k = min(max(find(t <= u, 1, 'last'), p+1), n); end
left = zeros(1,p+1); right = zeros(1,p+1); Nl = zeros(1,p+1); Nl(1) = 1;
for j = 1:p
    left(j+1) = u - t(k+1-j);  right(j+1) = t(k+j) - u;  saved = 0;
    for r = 0:j-1
        temp = Nl(r+1)/(right(r+2) + left(j-r+1));
        Nl(r+1) = saved + right(r+2)*temp;  saved = left(j-r+1)*temp;
    end
    Nl(j+1) = saved;
end
N(k-p:k) = Nl;
end
