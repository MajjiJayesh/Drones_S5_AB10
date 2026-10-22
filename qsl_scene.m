function qsl_scene(t, x, delta, Tr)
% QSL_SCENE  Live 3-D animation that runs INSIDE the Simulink simulation
% (called by the Animation block every 0.05 s). Draws buildings, moving
% obstacles, sensor range, detection lines, the drone, the reference path and
% EVERY point travelled (kept after arrival). Window has a Pause/Resume
% button (it pauses/continues the Simulink run) and a side list of points.
persistent S
W = qsl_world();  cfg = W.cfg;
if isempty(S) || t <= 0 || ~isfield(S,'fig') || ~ishandle(S.fig)
    S = build_scene(W);
end
p = x(1:3)';
S.n = S.n + 1;  S.P(S.n,:) = p;  S.T(S.n) = t;
set(S.hTrail,'XData',S.P(1:S.n,1),'YData',S.P(1:S.n,2),'ZData',S.P(1:S.n,3));
set(S.hDots,'XData',S.P(1:S.n,1),'YData',S.P(1:S.n,2),'ZData',S.P(1:S.n,3));
if mod(S.n-1,5) == 0 || t >= cfg.Ttraj + cfg.tHold - 0.06        % side list ~4 Hz
    ii = 1:5:S.n;
    set(S.hList,'String',arrayfun(@(a) sprintf('%5.2f %6.2f %6.2f %6.2f',S.T(a),S.P(a,1),S.P(a,2),S.P(a,3)),ii,'UniformOutput',false), ...
        'Value',numel(ii),'ListboxTop',max(1,numel(ii)-10));
end
for s = 1:numel(S.sphId)
    j = S.sphId(s);  r = W.obs(j).size(1);  c = W.obs(j).p0 + W.obs(j).vel*t;
    set(S.hSph(s),'XData',S.sx*r+c(1),'YData',S.sy*r+c(2),'ZData',S.sz*r+c(3));
end
ring = S.ring;
set(S.hRing(1),'XData',p(1)+ring(1,:),'YData',p(2)+ring(2,:),'ZData',p(3)+ring(3,:));
set(S.hRing(2),'XData',p(1)+ring(1,:),'YData',p(2)+ring(3,:),'ZData',p(3)+ring(2,:));
set(S.hRing(3),'XData',p(1)+ring(3,:),'YData',p(2)+ring(1,:),'ZData',p(3)+ring(2,:));
minc = inf;
for j = 1:W.nObs
    tru = W.obs(j).p0 + W.obs(j).vel*t;
    cl = qsl_lib('obs_clearance', p, W.obs(j).type, tru', W.obs(j).size') - cfg.droneRadius;
    minc = min(minc, cl);
    if Tr(1,j) > 0.5 && (cl + cfg.droneRadius) <= cfg.sensorRange
        c = Tr(6:8,j)';
        set(S.hDet(j),'XData',[p(1) c(1)],'YData',[p(2) c(2)],'ZData',[p(3) c(3)]);
    else
        set(S.hDet(j),'XData',nan,'YData',nan,'ZData',nan);
    end
end
R = qsl_lib('quat2rot', x(7:10));  a = 0.20*S.vs;
e = {p'+R*[a;a;0]*0.707, p'+R*[-a;a;0]*0.707, p'+R*[-a;-a;0]*0.707, p'+R*[a;-a;0]*0.707};
set(S.hArm(1),'XData',[e{1}(1) e{3}(1)],'YData',[e{1}(2) e{3}(2)],'ZData',[e{1}(3) e{3}(3)]);
set(S.hArm(2),'XData',[e{2}(1) e{4}(1)],'YData',[e{2}(2) e{4}(2)],'ZData',[e{2}(3) e{4}(3)]);
ph = linspace(0,2*pi,18);
for m = 1:4
    d = e{m} + R*[0.45*a*cos(ph); 0.45*a*sin(ph); zeros(size(ph))];
    set(S.hRot(m),'XData',d(1,:),'YData',d(2,:),'ZData',d(3,:));
end
nose = p' + R*[1.1*a;0;0];
set(S.hNose,'XData',[p(1) nose(1)],'YData',[p(2) nose(2)],'ZData',[p(3) nose(3)]);
dn = norm(delta);
if dn < 0.15, lab = 'ON REFERENCE PATH'; else, lab = qsl_lib('label_decision', delta(:), [1;0;0]); end
msg = sprintf('t = %5.1f s\nposition  %5.2f %5.2f %5.2f\ndetour    %4.2f m\nmin clear %4.2f m\nstate: %s', t,p(1),p(2),p(3),dn,minc,lab);
if t >= cfg.Ttraj + cfg.tHold - 0.06, msg = [msg sprintf('\nDESTINATION REACHED')]; end
set(S.hHud,'String',msg);
if exist('OCTAVE_VERSION','builtin'), drawnow; else, drawnow limitrate; end
end

function S = build_scene(W)
cfg = W.cfg;  vis = 'on'; if ~isempty(getenv('QUAD_HEADLESS')), vis = 'off'; end
S.fig = figure('Name','Quadcopter obstacle avoidance (Simulink)','Color','w','Visible',vis, ...
               'Position',[60 40 1250 740],'NumberTitle','off');
ax = axes('Parent',S.fig,'Position',[0.05 0.12 0.68 0.82]);  hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,38,24); xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)'); set(ax,'Box','on');
F = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8];
cols = [0.55 0.60 0.70; 0.70 0.62 0.55; 0.60 0.68 0.62];
for i = 1:size(cfg.buildings,1)
    b = cfg.buildings(i,:);  hx = b(3)/2;  hy = b(4)/2;
    V = [b(1)-hx b(2)-hy b(5); b(1)+hx b(2)-hy b(5); b(1)+hx b(2)+hy b(5); b(1)-hx b(2)+hy b(5); ...
         b(1)-hx b(2)-hy b(6); b(1)+hx b(2)-hy b(6); b(1)+hx b(2)+hy b(6); b(1)-hx b(2)+hy b(6)];
    patch('Parent',ax,'Vertices',V,'Faces',F,'FaceColor',cols(mod(i-1,3)+1,:),'FaceAlpha',0.75,'EdgeColor',[0.2 0.2 0.25]);
    text(b(1),b(2),b(6)+0.3,char('A'+i-1),'Parent',ax,'FontWeight','bold','HorizontalAlignment','center');
end
tt = linspace(0,cfg.Ttraj,300);  PR = zeros(300,3);
for i = 1:300, PR(i,:) = qsl_lib('ref_eval', W.ref, tt(i))'; end
m = 1.5;  x0 = min(PR(:,1))-m; x1 = max(PR(:,1))+m; y0 = min(PR(:,2))-m; y1 = max(PR(:,2))+m;
patch('Parent',ax,'Vertices',[x0 y0 0; x1 y0 0; x1 y1 0; x0 y1 0],'Faces',[1 2 3; 1 3 4],'FaceColor',[0.85 0.92 0.85],'FaceAlpha',0.6,'EdgeColor','none');
xlim(ax,[x0 x1]); ylim(ax,[y0 y1]); zlim(ax,[0 min(max([PR(:,3); cfg.buildings(:,6)])+0.5,13)]);
plot3(ax,PR(:,1),PR(:,2),PR(:,3),'--','Color',[0.35 0.35 0.35],'LineWidth',1.4);
plot3(ax,W.p0(1),W.p0(2),W.p0(3),'o','MarkerSize',10,'MarkerFaceColor',[0.1 0.8 0.1],'Color','k');
plot3(ax,W.pDest(1),W.pDest(2),W.pDest(3),'p','MarkerSize',16,'MarkerFaceColor','r','Color','k');
wp = W.ref.waypoints;  plot3(ax,wp(1,:),wp(2,:),wp(3,:),'k.','MarkerSize',14);
for i = 1:size(wp,2), text(wp(1,i),wp(2,i),wp(3,i)+0.35,sprintf('P%d',i-1),'Parent',ax,'FontSize',9); end
[S.sx,S.sy,S.sz] = sphere(16);  S.hSph = [];  S.sphId = [];
for j = 1:W.nObs
    if W.obs(j).type == 1
        r = W.obs(j).size(1);  c = W.obs(j).p0;
        S.hSph(end+1) = surf(ax,S.sx*r+c(1),S.sy*r+c(2),S.sz*r+c(3),'FaceColor',[0.95 0.4 0.15],'EdgeColor','none','FaceAlpha',0.85);
        S.sphId(end+1) = j;
    end
end
th = linspace(0,2*pi,60);  S.ring = [cos(th); sin(th); zeros(size(th))]*cfg.sensorRange;
S.hRing = zeros(1,3);  for k = 1:3, S.hRing(k) = plot3(ax,nan,nan,nan,'-','Color',[0.2 0.7 0.9],'LineWidth',0.8); end
S.hDet = zeros(1,W.nObs);  for j = 1:W.nObs, S.hDet(j) = plot3(ax,nan,nan,nan,'-','Color',[0.9 0.1 0.1],'LineWidth',1.6); end
S.hTrail = plot3(ax,nan,nan,nan,'-','Color',[0.1 0.4 0.9],'LineWidth',1.6);
S.hDots  = plot3(ax,nan,nan,nan,'.','Color',[0 0.2 0.8],'MarkerSize',9);
S.hArm = zeros(1,2); for k = 1:2, S.hArm(k) = plot3(ax,nan,nan,nan,'-','Color',[0.1 0.1 0.1],'LineWidth',3); end
S.hRot = zeros(1,4); for k = 1:4, S.hRot(k) = plot3(ax,nan,nan,nan,'-','Color',[0.85 0.1 0.1],'LineWidth',2); end
S.hNose = plot3(ax,nan,nan,nan,'-','Color',[0 0.6 0],'LineWidth',3);
S.hHud = text(0.02,0.98,'','Units','normalized','Parent',ax,'VerticalAlignment','top','FontName','Courier','FontSize',10,'BackgroundColor',[1 1 1]);
S.vs = 1.6;
uicontrol(S.fig,'Style','text','Units','normalized','Position',[0.75 0.93 0.23 0.04],'String','Visited points  (t, x, y, z)', ...
          'BackgroundColor','w','FontWeight','bold','HorizontalAlignment','left');
S.hList = uicontrol(S.fig,'Style','listbox','Units','normalized','Position',[0.75 0.12 0.23 0.81],'String',{'(none yet)'},'FontName','Courier','FontSize',9);
mdl = '';  try, mdl = bdroot; catch, end                                  %#ok<NASGU>
uicontrol(S.fig,'Style','pushbutton','Units','normalized','Position',[0.05 0.025 0.14 0.06],'String','Pause / Resume','FontSize',10,'Callback',@(h,e) pause_cb());
S.P = nan(ceil((cfg.Ttraj+cfg.tHold)/0.05)+5,3);  S.T = zeros(size(S.P,1),1);  S.n = 0;
end

function pause_cb()
% pauses / continues whichever Simulink model is running
mdls = find_system('SearchDepth',0,'type','block_diagram');
for i = 1:numel(mdls)
    try
        st = get_param(mdls{i},'SimulationStatus');
        if strcmp(st,'paused'), set_param(mdls{i},'SimulationCommand','continue'); return
        elseif strcmp(st,'running'), set_param(mdls{i},'SimulationCommand','pause'); return
        end
    catch
    end
end
end
