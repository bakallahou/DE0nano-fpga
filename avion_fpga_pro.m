function avion_fpga_pro
%AVION_FPGA_PRO  Simulateur de vol 3D temps reel pour chaine FPGA/ESP32 -> MATLAB.
%
%   Lancer :  avion_fpga_pro
%
%   FONCTIONS
%     - Modele 3D monomoteur : fuselage profile mat, ailes NACA effilees avec
%       dihedre, empennage, derive bleue, verriere transparente, helice animee
%       en temps reel (independante du framerate), train tricycle, feux de
%       navigation et feu anticollision.
%     - Eclairage studio (cle / remplissage / contre-jour), ombre portee
%       dynamique, sol a grille defilante dans le sens du cap.
%     - Horizon artificiel (attitude indicator) incruste.
%     - Filtrage des donnees accelerometre : Kalman adaptatif (2 etats par
%       axe), complementaire adaptatif, ou brut.
%     - Port serie moderne (serialport) avec callback asynchrone
%       (configureCallback) : aucune boucle bloquante.
%
%   FORMAT D'ENTREE (UART/USB)  :  "ax,ay,az\n"   en g, ou LSB brut (256 LSB/g).
%
%   REPERE CORPS : X avant, Y gauche, Z haut (repere direct).
%   CONVENTIONS  : Roll+ = aile droite basse, Pitch+ = nez en haut,
%                  Yaw+ = nez a droite (cap).
%     roll  = atan2(ay, az)
%     pitch = atan2(-ax, sqrt(ay^2 + az^2))
%
%   CLAVIER : fleches = roll/pitch (mode manuel), A/E = yaw, D = demo, R = reset.
%
%   PREREQUIS : MATLAB R2019b ou plus recent (serialport). Aucune toolbox.

%% ======================================================================
%  1. CONFIGURATION
%  ======================================================================
CFG.port      = 'COM7';     % port par defaut
CFG.baud      = 115200;     % debit fixe
CFG.fps       = 30;         % cadence d'affichage
CFG.rxTimeout = 1.5;        % [s] alerte si plus de donnees
CFG.speed     = 3.0;        % [unites/s] vitesse de defilement du sol
CFG.turnV     = 35;         % [m/s] vitesse fictive pour le virage coordonne
CFG.zGround   = -3.4;       % altitude du sol dans la scene
CFG.kalmanR   = 2.0;        % [deg^2] variance de mesure de base
CFG.kalmanQ   = 250;        % [deg^2/s^3] bruit de processus
CFG.propRps   = 3.5;        % [tr/s] vitesse visuelle de l'helice

C.bg    = [0.04 0.05 0.08];
C.panel = [0.08 0.09 0.13];
C.field = [0.14 0.16 0.21];
C.fg    = [0.90 0.93 0.97];
C.dim   = [0.55 0.60 0.68];
C.acc   = [0.25 0.78 1.00];
C.ok    = [0.35 0.90 0.50];
C.warn  = [1.00 0.70 0.25];
C.err   = [1.00 0.40 0.40];

zG = CFG.zGround;
d2r = pi/180;

%% ======================================================================
%  2. ETAT PARTAGE
%  ======================================================================
roll = 0; pitch = 0; yaw = 0;          % angles courants [deg]
posX = 0; posY = 0;                    % position monde (defilement du sol)
mRoll = 0; mPitch = 0; mYaw = 0;       % consignes manuelles (curseurs)
demoOn = true; autoYaw = false; introDone = false;
filtMode = 1;                          % 1 Kalman, 2 complementaire, 3 brut

sp = [];                               % objet serialport
isConnected = false; lostLink = false; lostMsg = '';
hasRx = false; lastRx = 0; rxRate = 0; accNorm = 1; badLines = 0;
rollEst = 0; pitchEst = 0;
kR = kalmanInit(); kP = kalmanInit();

t0 = tic; lastT = 0; frameCnt = 0;
shBase = {}; shH = gobjects(0);
lastErr = '';

%% ======================================================================
%  3. FENETRE ET SCENE 3D
%  ======================================================================
delete(findall(0,'Type','figure','Tag','AVION_FPGA_PRO'));
scr = get(0,'ScreenSize');
fig = figure('Name','AVION FPGA PRO - Simulateur de vol 3D','NumberTitle','off', ...
    'Tag','AVION_FPGA_PRO','Color',C.bg,'MenuBar','none','ToolBar','none', ...
    'Position',[scr(1)+0.06*scr(3), scr(2)+0.08*scr(4), 0.86*scr(3), 0.84*scr(4)], ...
    'CloseRequestFcn',@onClose,'WindowKeyPressFcn',@onKey);

ax = axes('Parent',fig,'Units','normalized','Position',[0 0 0.72 1],'Color',C.bg);
hold(ax,'on');
set(ax,'DataAspectRatio',[1 1 1],'XLim',[-6.5 6.5],'YLim',[-6.5 6.5],'ZLim',[-6.5 6.5], ...
    'Projection','perspective','Clipping','off');
axis(ax,'off'); axis(ax,'vis3d');
view(ax,[160 20]);

% Eclairage studio : cle chaude, remplissage froid, contre-jour
light('Parent',ax,'Style','infinite','Position',[ 1.0 -0.8  1.6],'Color',[1.00 0.96 0.90]);
light('Parent',ax,'Style','infinite','Position',[-0.8  1.0  0.4],'Color',[0.30 0.40 0.60]);
light('Parent',ax,'Style','infinite','Position',[-1.0 -0.1  0.7],'Color',[0.45 0.55 0.85]);

% Proprietes communes des surfaces (rendu mat)
com = {'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',0.35, ...
       'DiffuseStrength',0.85,'SpecularStrength',0.25,'SpecularExponent',12,'Clipping','off'};

% --- Sol : disque degrade + grilles defilantes (3 couches = fondu radial) ---
[RR,TH] = meshgrid(linspace(0,14,28), linspace(0,2*pi,72));
fade = exp(-(RR/8).^2);
cCtr = [0.10 0.14 0.22];
CD = zeros([size(RR) 3]);
for ic = 1:3
    CD(:,:,ic) = C.bg(ic) + fade*(cCtr(ic)-C.bg(ic));
end
surface(RR.*cos(TH),RR.*sin(TH),zG*ones(size(RR)),CD,'Parent',ax, ...
    'FaceColor','interp','EdgeColor','none','FaceLighting','none','Clipping','off');
gridR = [11 8 5];
gl = gobjects(1,3);
for ig = 1:3
    gl(ig) = line('Parent',ax,'XData',nan,'YData',nan,'ZData',nan, ...
        'Color',[0.35 0.58 0.90 0.20],'LineWidth',1,'Clipping','off');
end

%% ======================================================================
%  4. MODELE AVION
%  ======================================================================
hBody = hgtransform('Parent',ax);
hProp = hgtransform('Parent',hBody);

cBody  = [0.90 0.91 0.93];
cWing  = [0.84 0.86 0.90];
cAcc   = [0.10 0.30 0.70];
cGlass = [0.30 0.58 0.88];
cGray  = [0.78 0.80 0.84];
cTire  = [0.45 0.47 0.52];
cDark  = [0.12 0.13 0.16];
cLine  = [0.18 0.20 0.26];

% --- Fuselage (surface de revolution, profil spline) ---
xk = [-6 -5 -3.5 -1.5 0 1.5 3 4 4.4 4.55];
rk = [0.07 0.13 0.30 0.60 0.78 0.78 0.70 0.62 0.54 0.47];
xs = linspace(-6,4.55,80);
rs = interp1(xk,rk,xs,'pchip');
[X,Y,Z] = revolve(xs,rs,0,linspace(0,2*pi,49));
surface(X,Y,Z,'Parent',hBody,'FaceColor',cBody,com{:});
registerShadow(X,Y,Z);

% Bande de livree laterale (bleue, effilee)
xb = linspace(-4.2,3.6,50); rb = interp1(xk,rk,xb,'pchip')*1.006;
[X,Y,Z] = revolve(xb,rb,0,linspace(-0.16,0.16,6));
surface(X,Y,Z,'Parent',hBody,'FaceColor',cAcc,com{:});
[X,Y,Z] = revolve(xb,rb,0,pi+linspace(-0.16,0.16,6));
surface(X,Y,Z,'Parent',hBody,'FaceColor',cAcc,com{:});

% Panneau anti-reflet sur le capot
xg = linspace(2.2,4.2,20); rg = interp1(xk,rk,xg,'pchip')*1.005;
[X,Y,Z] = revolve(xg,rg,0,pi/2+linspace(-0.55,0.55,14));
surface(X,Y,Z,'Parent',hBody,'FaceColor',cDark,com{:});

% Cone d'helice
xsn = linspace(4.55,5.35,26);
rsn = 0.47*sqrt(max(0,1-((xsn-4.55)/0.80).^2));
[X,Y,Z] = revolve(xsn,rsn,0,linspace(0,2*pi,49));
surface(X,Y,Z,'Parent',hBody,'FaceColor',cAcc,com{:});
registerShadow(X,Y,Z);

% --- Voilure : aile basse a dihedre 7 deg, ailerons + volets traces ---
% (mirror=false -> cote +Y = gauche ; mirror=true -> cote -Y = droite)
tipLeft  = addWing( 1.2,-0.35,5.0,2.4,1.3,0.7,7,0.14,false,cWing,[9 13; 2 6]);
tipRight = addWing( 1.2,-0.35,5.0,2.4,1.3,0.7,7,0.14,true ,cWing,[9 13; 2 6]);
% --- Empennage horizontal (gouvernes de profondeur) ---
addWing(-4.4, 0.10,2.1,1.3,0.7,0.5,5,0.09,false,cWing,[2 13]);
addWing(-4.4, 0.10,2.1,1.3,0.7,0.5,5,0.09,true ,cWing,[2 13]);
% --- Derive bleue (gouverne de direction) ---
tipFin = addWing(-3.6, 0.25,2.2,1.9,0.9,1.1,90,0.09,false,cAcc,[2 13]);

% Feux de navigation (rouge a gauche, vert a droite) et anticollision
addGlow(tipLeft  + [0  0.10 0.02],0.09,[1.0 0.12 0.12]);
addGlow(tipRight + [0 -0.10 0.02],0.09,[0.10 1.0 0.25]);
hBeacon = addGlow(tipFin + [0 0 0.06],0.08,[1.0 1.0 1.0]);

% Verriere du cockpit (transparente)
[ex,ey,ez] = ellipsoid(0.9,0,0.55,1.3,0.58,0.42,28);
surface(ex,ey,ez,'Parent',hBody,'FaceColor',cGlass,'FaceAlpha',0.55, ...
    'EdgeColor','none','FaceLighting','gouraud','SpecularStrength',0.95, ...
    'SpecularExponent',45,'AmbientStrength',0.25,'Clipping','off');
% Cadre de verriere
for rr = [8 15 22]
    line('Parent',hBody,'XData',ex(rr,:),'YData',ey(rr,:),'ZData',ez(rr,:), ...
        'Color',cLine,'LineWidth',1.2,'Clipping','off');
end

% Antenne et echappements
addCyl([-1.5 0 0.74],[-1.75 0 1.20],0.018,cGray);
for sy = [1 -1]
    addCyl([3.40 sy*0.50 -0.55],[2.95 sy*0.62 -0.88],0.07,cDark);
end

% Train tricycle gris clair
addCyl([3.00 0 -0.50],[3.10 0 -1.55],0.05,cGray);          % jambe avant
addWheel(3.10,0,-1.55,0.25,0.15,cTire,cGray);
for sy = [1 -1]
    addCyl([0.80 sy*0.75 -0.32],[0.65 sy*1.30 -1.45],0.055,cGray);   % jambe principale
    addCyl([0.45 sy*0.60 -0.32],[0.62 sy*1.25 -1.20],0.030,cGray);   % contre-fiche
    addWheel(0.65,sy*1.30,-1.55,0.30,0.17,cTire,cGray);
end

% --- Helice : 2 pales vrillees + disque de flou ---
rbl = linspace(0.20,1.75,16)';
chd = interp1([0.2 0.5 0.9 1.4 1.75],[0.20 0.26 0.30 0.22 0.10],rbl,'pchip');
twi = interp1([0.2 1.75],[0.55 0.14],rbl);
Sb  = chd*[-0.5 0.5];
Xb  = sin(twi).*Sb;  Yb = cos(twi).*Sb;  Zb = rbl*ones(1,2);
cProp = [0.75 0.77 0.82];
surface(Xb, Yb, Zb,'Parent',hProp,'FaceColor',cProp,com{:},'BackFaceLighting','lit');
surface(Xb,-Yb,-Zb,'Parent',hProp,'FaceColor',cProp,com{:},'BackFaceLighting','lit');
[Rd,Td] = meshgrid([0 1.78],linspace(0,2*pi,49));
surface(zeros(size(Rd)),Rd.*cos(Td),Rd.*sin(Td),'Parent',hProp,'FaceColor',[0.80 0.85 0.95], ...
    'FaceAlpha',0.07,'EdgeColor','none','FaceLighting','none','Clipping','off');

% Axes corps (reperes discrets)
line('Parent',hBody,'XData',[0 2.2],'YData',[0 0],'ZData',[0 0],'Color',[0.95 0.35 0.35],'LineWidth',1.2,'Clipping','off');
line('Parent',hBody,'XData',[0 0],'YData',[0 2.2],'ZData',[0 0],'Color',[0.35 0.95 0.45],'LineWidth',1.2,'Clipping','off');
line('Parent',hBody,'XData',[0 0],'YData',[0 0],'ZData',[0 2.2],'Color',[0.40 0.60 1.00],'LineWidth',1.2,'Clipping','off');

%% ======================================================================
%  5. HORIZON ARTIFICIEL (incruste en bas a gauche)
%  ======================================================================
axA = axes('Parent',fig,'Units','normalized','Position',[0.02 0.04 0.19 0.33],'Color','none');
hold(axA,'on');
set(axA,'DataAspectRatio',[1 1 1],'XLim',[-1.1 1.1],'YLim',[-1.1 1.1]);
axis(axA,'off'); view(axA,2);

hH  = hgtransform('Parent',axA);
hRS = hgtransform('Parent',axA);
patch('Parent',hH,'Vertices',[-3 0 0; 3 0 0; 3 3 0; -3 3 0],'Faces',[1 2 3 4], ...
    'FaceColor',[0.22 0.52 0.88],'EdgeColor','none');
patch('Parent',hH,'Vertices',[-3 -3 0; 3 -3 0; 3 0 0; -3 0 0],'Faces',[1 2 3 4], ...
    'FaceColor',[0.45 0.30 0.16],'EdgeColor','none');
line('Parent',hH,'XData',[-3 3],'YData',[0 0],'ZData',[0.1 0.1],'Color','w','LineWidth',1.5);
for pdeg = [-30 -20 -10 10 20 30]
    wl = 0.18 + 0.12*(abs(pdeg)==20);
    line('Parent',hH,'XData',[-wl wl],'YData',pdeg*0.03*[1 1],'ZData',[0.1 0.1],'Color','w','LineWidth',1);
end
% Masque circulaire
[Rm,Tm] = meshgrid([1.0 1.8],linspace(0,2*pi,73));
surface(Rm.*cos(Tm),Rm.*sin(Tm),ones(size(Rm)),'Parent',axA,'FaceColor',C.bg, ...
    'EdgeColor','none','FaceLighting','none');
tt = linspace(0,2*pi,100);
line('Parent',axA,'XData',cos(tt),'YData',sin(tt),'ZData',1.5*ones(size(tt)),'Color',C.dim,'LineWidth',1.5);
for ia = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
    r2 = 1.07 + 0.04*(mod(ia,30)==0);
    line('Parent',hRS,'XData',[1.0 r2]*sind(ia),'YData',[1.0 r2]*cosd(ia),'ZData',[2 2],'Color','w','LineWidth',1.2);
end
patch('Parent',axA,'Vertices',[0 0.99 3; -0.06 0.87 3; 0.06 0.87 3],'Faces',[1 2 3], ...
    'FaceColor',[1 0.85 0.1],'EdgeColor','none');
line('Parent',axA,'XData',[-0.55 -0.20 -0.20],'YData',[0 0 -0.07],'ZData',[3 3 3],'Color',[1 0.85 0.1],'LineWidth',3);
line('Parent',axA,'XData',[ 0.55  0.20  0.20],'YData',[0 0 -0.07],'ZData',[3 3 3],'Color',[1 0.85 0.1],'LineWidth',3);
line('Parent',axA,'XData',0,'YData',0,'ZData',3,'Marker','o','MarkerSize',5,'MarkerFaceColor',[1 0.85 0.1],'Color',[1 0.85 0.1]);

%% ======================================================================
%  6. INTERFACE UTILISATEUR
%  ======================================================================
uicontrol(fig,'Style','text','Units','normalized','Position',[0.015 0.925 0.50 0.055], ...
    'String','AVION FPGA PRO','FontSize',20,'FontWeight','bold','ForegroundColor',C.acc, ...
    'BackgroundColor',C.bg,'HorizontalAlignment','left');
txtMode = uicontrol(fig,'Style','text','Units','normalized','Position',[0.015 0.89 0.50 0.035], ...
    'String','','FontSize',11,'ForegroundColor',C.dim,'BackgroundColor',C.bg,'HorizontalAlignment','left');

P = uipanel('Parent',fig,'Units','normalized','Position',[0.73 0.02 0.255 0.96], ...
    'BackgroundColor',C.panel,'ForegroundColor',C.acc,'Title',' TELEMETRIE DE VOL ', ...
    'FontWeight','bold','FontSize',11,'BorderType','line','HighlightColor',[0.2 0.25 0.35]);

% Afficheurs numeriques
lblNames = {'ROLL','PITCH','YAW'}; vals = gobjects(1,3);
for ik = 1:3
    xk0 = 0.04 + (ik-1)*0.31;
    mkText([xk0 0.915 0.29 0.03],lblNames{ik},9,C.dim,'center','bold');
    vals(ik) = mkText([xk0 0.855 0.29 0.055],'+0.0',18,C.acc,'center','bold');
end
lblRoll = vals(1); lblPitch = vals(2); lblYaw = vals(3);
set(vals,'FontName','Consolas');

% Curseurs
mkText([0.06 0.805 0.88 0.03],'Roll (manuel)',9,C.fg,'left','normal');
sldR = mkSlider([0.06 0.775 0.88 0.028],-90,90);
mkText([0.06 0.735 0.88 0.03],'Pitch (manuel)',9,C.fg,'left','normal');
sldP = mkSlider([0.06 0.705 0.88 0.028],-90,90);
mkText([0.06 0.665 0.88 0.03],'Yaw (manuel)',9,C.fg,'left','normal');
sldY = mkSlider([0.06 0.635 0.88 0.028],-180,180);

% Boutons
uicontrol(P,'Style','pushbutton','Units','normalized','Position',[0.06 0.560 0.42 0.05], ...
    'String','REINITIALISER','FontWeight','bold','Callback',@onReset);
btnDemo = uicontrol(P,'Style','togglebutton','Units','normalized','Position',[0.52 0.560 0.42 0.05], ...
    'String','MODE DEMO','FontWeight','bold','Value',1,'Callback',@onDemo);
chkYaw = uicontrol(P,'Style','checkbox','Units','normalized','Position',[0.06 0.505 0.88 0.04], ...
    'String','Yaw auto (virage coordonne)','BackgroundColor',C.panel,'ForegroundColor',C.fg, ...
    'Callback',@onAutoYaw); %#ok<NASGU>

mkText([0.06 0.450 0.28 0.035],'Filtre :',9,C.fg,'left','normal');
uicontrol(P,'Style','popupmenu','Units','normalized','Position',[0.34 0.455 0.60 0.035], ...
    'String',{'Kalman adaptatif','Complementaire adaptatif','Brut (sans filtre)'}, ...
    'BackgroundColor',C.field,'ForegroundColor',C.fg,'Callback',@onFilter);
mkText([0.06 0.400 0.28 0.035],'Vue :',9,C.fg,'left','normal');
uicontrol(P,'Style','popupmenu','Units','normalized','Position',[0.34 0.405 0.60 0.035], ...
    'String',{'3D (iso)','Dessus','Cote','Face'},'BackgroundColor',C.field, ...
    'ForegroundColor',C.fg,'Callback',@onView);

mkText([0.06 0.335 0.88 0.035],'-------- LIAISON SERIE --------',9,C.acc,'center','bold');
mkText([0.06 0.290 0.28 0.035],'Port :',9,C.fg,'left','normal');
portList = listPorts(CFG.port);
popPort = uicontrol(P,'Style','popupmenu','Units','normalized','Position',[0.34 0.295 0.60 0.035], ...
    'String',portList,'Value',1,'BackgroundColor',C.field,'ForegroundColor',C.fg);
mkText([0.06 0.245 0.88 0.035],sprintf('Debit : %d bauds (fixe)',CFG.baud),9,C.dim,'left','normal');
btnConn = uicontrol(P,'Style','togglebutton','Units','normalized','Position',[0.06 0.180 0.88 0.055], ...
    'String','CONNECTER','FontWeight','bold','Callback',@onConnect);
txtStat = mkText([0.06 0.115 0.88 0.05],'',9,C.ok,'left','normal');
txtRate = mkText([0.06 0.075 0.88 0.035],'',9,C.dim,'left','normal');
mkText([0.06 0.012 0.88 0.055],'Fleches: roll/pitch | A,E: yaw | D: demo | R: reset',8,C.dim,'left','normal');

applyUiState();

%% ======================================================================
%  7. BOUCLE D'AFFICHAGE (timer) - la serie est traitee par callback
%  ======================================================================
tm = timer('ExecutionMode','fixedSpacing','Period',round(1000/CFG.fps)/1000, ...
    'BusyMode','drop','TimerFcn',@(~,~)tick());
drawnow;
try, camva(ax,camva(ax)*0.80); catch, end
start(tm);

%% ======================================================================
%  FONCTIONS IMBRIQUEES : scene
%  ======================================================================
    function registerShadow(Xs,Ys,Zs)
        shBase{end+1} = cat(3,Xs,Ys,Zs);
        shH(end+1) = surface(Xs,Ys,repmat(zG+0.01,size(Xs)),'Parent',ax, ...
            'FaceColor',[0.02 0.03 0.05],'EdgeColor','none','FaceLighting','none','Clipping','off');
    end

    function updateShadows(M)
        slx = 0.25; sly = 0.15;       % inclinaison de la lumiere
        for iq = 1:numel(shH)
            B = shBase{iq}; [bm,bn,~] = size(B);
            Q = [reshape(B,[],3), ones(bm*bn,1)]*M';
            dz = Q(:,3) - zG;
            set(shH(iq),'XData',reshape(Q(:,1)+slx*dz,bm,bn),'YData',reshape(Q(:,2)+sly*dz,bm,bn));
        end
    end

    function tip = addWing(x0,z0,span,cr,ct,sweep,dih,tc,mirror,col,rows)
        [Xw,Yw,Zw,xc] = wingSurf(x0,0,z0,span,cr,ct,sweep,dih,tc,mirror);
        surface(Xw,Yw,Zw,'Parent',hBody,'FaceColor',col,com{:});
        registerShadow(Xw,Yw,Zw);
        nx = numel(xc); iH = find(xc>=0.72,1);
        sgn = 1 - 2*double(mirror);
        oy = -sind(dih)*0.012*sgn;  oz = cosd(dih)*0.012;
        for kr = 1:size(rows,1)
            r1 = rows(kr,1); r2 = rows(kr,2);
            for side = [1 -1]
                if side == 1, ch = iH; cc = iH:nx;
                else,         ch = 2*nx-iH; cc = ch:-1:nx;
                end
                line('Parent',hBody,'XData',Xw(r1:r2,ch),'YData',Yw(r1:r2,ch)+side*oy, ...
                    'ZData',Zw(r1:r2,ch)+side*oz,'Color',cLine,'LineWidth',1,'Clipping','off');
                for rw = [r1 r2]
                    line('Parent',hBody,'XData',Xw(rw,cc),'YData',Yw(rw,cc)+side*oy, ...
                        'ZData',Zw(rw,cc)+side*oz,'Color',cLine,'LineWidth',1,'Clipping','off');
                end
            end
        end
        tip = [mean(Xw(end,:)) mean(Yw(end,:)) mean(Zw(end,:))];
    end

    function h = addGlow(cen,rad,col)
        [gx,gy,gz] = ellipsoid(cen(1),cen(2),cen(3),rad,rad,rad,10);
        h = surface(gx,gy,gz,'Parent',hBody,'FaceColor',col,'EdgeColor','none', ...
            'FaceLighting','none','Clipping','off');
    end

    function addCyl(p1,p2,rad,col)
        [Xc,Yc,Zc] = cylSurf(p1,p2,rad,24);
        surface(Xc,Yc,Zc,'Parent',hBody,'FaceColor',col,com{:});
        for ie = 1:2
            pe = (ie==1)*p1 + (ie==2)*p2;
            [Xd,Yd,Zd] = discSurf(pe,p2-p1,rad,24);
            surface(Xd,Yd,Zd,'Parent',hBody,'FaceColor',col,com{:});
        end
    end

    function addWheel(x,y,z,rad,w,colT,colH)
        addCyl([x y-w/2 z],[x y+w/2 z],rad,colT);
        addCyl([x y-w/2-0.006 z],[x y+w/2+0.006 z],rad*0.55,colH);
    end

%% ======================================================================
%  FONCTIONS IMBRIQUEES : interface
%  ======================================================================
    function h = mkText(pos,str,fs,col,al,wt)
        h = uicontrol(P,'Style','text','Units','normalized','Position',pos,'String',str, ...
            'FontSize',fs,'ForegroundColor',col,'BackgroundColor',C.panel, ...
            'HorizontalAlignment',al,'FontWeight',wt);
    end

    function h = mkSlider(pos,lo,hi)
        h = uicontrol(P,'Style','slider','Units','normalized','Position',pos, ...
            'Min',lo,'Max',hi,'Value',0,'Callback',@onSlider);
        try, addlistener(h,'ContinuousValueChange',@onSlider); catch, end
    end

    function onSlider(~,~)
        mRoll = get(sldR,'Value'); mPitch = get(sldP,'Value'); mYaw = get(sldY,'Value');
    end

    function onReset(~,~)
        mRoll = 0; mPitch = 0; mYaw = 0; yaw = 0; posX = 0; posY = 0;
        if ~isConnected
            roll = 0; pitch = 0; demoOn = false; set(btnDemo,'Value',0);
        end
        set([sldR sldP sldY],'Value',0);
        hasRx = false;
        applyUiState();
    end

    function onDemo(src,~)
        if isConnected, set(src,'Value',0); return; end
        demoOn = get(src,'Value') == 1;
        if ~demoOn
            mRoll = roll; mPitch = pitch; mYaw = yaw;
            set(sldR,'Value',clampv(roll,-90,90)); set(sldP,'Value',clampv(pitch,-90,90));
            set(sldY,'Value',clampv(yaw,-180,180));
        end
        applyUiState();
    end

    function onAutoYaw(src,~)
        autoYaw = get(src,'Value') == 1;
        if ~autoYaw, mYaw = yaw; set(sldY,'Value',clampv(yaw,-180,180)); end
        applyUiState();
    end

    function onFilter(src,~)
        filtMode = get(src,'Value'); hasRx = false;
    end

    function onView(src,~)
        introDone = true;
        switch get(src,'Value')
            case 1, view(ax,[55 18]);
            case 2, view(ax,[0 90]);
            case 3, view(ax,[0 0]);
            case 4, view(ax,[90 0]);
        end
    end

    function applyUiState()
        manualAtt = ~isConnected && ~demoOn;
        set([sldR sldP],'Enable',onoff(manualAtt));
        set(sldY,'Enable',onoff(~demoOn && ~autoYaw));
        set(btnDemo,'Enable',onoff(~isConnected));
        set(popPort,'Enable',onoff(~isConnected));
    end

    function onKey(~,ev)
        manualAtt = ~isConnected && ~demoOn;
        switch lower(ev.Key)
            case 'r', onReset();
            case 'd'
                if ~isConnected
                    set(btnDemo,'Value',1-get(btnDemo,'Value')); onDemo(btnDemo);
                end
            case 'leftarrow',  if manualAtt, mRoll  = clampv(mRoll-3,-90,90);  set(sldR,'Value',mRoll);  end
            case 'rightarrow', if manualAtt, mRoll  = clampv(mRoll+3,-90,90);  set(sldR,'Value',mRoll);  end
            case 'uparrow',    if manualAtt, mPitch = clampv(mPitch+3,-90,90); set(sldP,'Value',mPitch); end
            case 'downarrow',  if manualAtt, mPitch = clampv(mPitch-3,-90,90); set(sldP,'Value',mPitch); end
            case 'a', if ~demoOn && ~autoYaw, mYaw = wrap180(mYaw-5); set(sldY,'Value',mYaw); end
            case 'e', if ~demoOn && ~autoYaw, mYaw = wrap180(mYaw+5); set(sldY,'Value',mYaw); end
        end
    end

%% ======================================================================
%  FONCTIONS IMBRIQUEES : liaison serie (asynchrone)
%  ======================================================================
    function onConnect(src,~)
        if get(src,'Value') == 1
            connectPort();
        else
            disconnectPort('Deconnecte');
        end
    end

    function connectPort()
        list = get(popPort,'String'); port = list{get(popPort,'Value')};
        set(txtStat,'String',['Connexion a ' port ' ...'],'ForegroundColor',C.warn); drawnow;
        try
            sp = serialport(port,CFG.baud,'Timeout',1);
            configureTerminator(sp,'LF');
            flush(sp);
            isConnected = true; lostLink = false; hasRx = false; badLines = 0;
            lastRx = toc(t0); rxRate = 0;
            demoOn = false; set(btnDemo,'Value',0);
            mYaw = yaw;
            set(btnConn,'String','DECONNECTER');
            configureCallback(sp,'terminator',@onSerial);   % lecture asynchrone
            applyUiState();
        catch ME
            sp = []; isConnected = false;
            set(btnConn,'Value',0,'String','CONNECTER');
            set(txtStat,'String','Echec de connexion','ForegroundColor',C.err);
            errordlg(ME.message,'Liaison serie');
        end
    end

    function disconnectPort(msg)
        try, if ~isempty(sp), configureCallback(sp,'off'); end, catch, end
        sp = [];                         % libere le port
        isConnected = false; lostLink = false;
        set(btnConn,'Value',0,'String','CONNECTER');
        set(txtStat,'String',msg,'ForegroundColor',C.warn);
        applyUiState();
    end

    function onSerial(src,~)
        % Appele par MATLAB a chaque ligne complete : traitement court, sans boucle.
        try
            ln = readline(src);
            if ismissing(ln), return; end
            v = sscanf(char(ln),'%f,%f,%f');
            if numel(v) < 3, badLines = badLines + 1; return; end
            acc = v(1:3);
            if max(abs(acc)) > 20, acc = acc/256; end       % LSB brut -> g
            nrm = norm(acc);
            if nrm < 0.2, badLines = badLines + 1; return; end
            tn = toc(t0);
            if hasRx, dtm = min(max(tn-lastRx,0.001),0.25); else, dtm = 0.02; end
            lastRx = tn; accNorm = nrm;
            rxRate = rxRate + 0.1*(1/dtm - rxRate);

            zr = atan2d(acc(2),acc(3));
            zp = atan2d(-acc(1),hypot(acc(2),acc(3)));
            dev = abs(nrm-1);                                % ecart a 1 g : mesure peu fiable

            if ~hasRx
                kR = kalmanInit(zr); kP = kalmanInit(zp);
                rollEst = zr; pitchEst = zp; hasRx = true;
                return;
            end
            switch filtMode
                case 1   % Kalman adaptatif (R augmente quand |a| s'ecarte de 1 g)
                    Rm = CFG.kalmanR*(1+(dev/0.05)^2);
                    kR = kalmanStep(kR,zr,dtm,Rm,CFG.kalmanQ);
                    kP = kalmanStep(kP,zp,dtm,Rm,CFG.kalmanQ);
                    rollEst = kR.x(1); pitchEst = kP.x(1);
                case 2   % complementaire / passe-bas adaptatif
                    gate = 1/(1+(dev/0.10)^2);
                    ar = (0.05 + 0.60*min(1,abs(wrap180(zr-rollEst))/20))*gate;
                    ap = (0.05 + 0.60*min(1,abs(wrap180(zp-pitchEst))/20))*gate;
                    rollEst  = wrap180(rollEst  + ar*wrap180(zr-rollEst));
                    pitchEst = wrap180(pitchEst + ap*wrap180(zp-pitchEst));
                otherwise
                    rollEst = zr; pitchEst = zp;
            end
        catch ME
            lostLink = true; lostMsg = ME.message;
        end
    end

%% ======================================================================
%  FONCTIONS IMBRIQUEES : boucle de rendu
%  ======================================================================
    function tick()
        if ~ishandle(fig)
            try, stop(tm); delete(tm); catch, end
            return;
        end
        try
            tNow = toc(t0); dt = min(max(tNow-lastT,0.001),0.2); lastT = tNow;
            frameCnt = frameCnt + 1;

            % --- Supervision de la liaison ---
            if isConnected
                if lostLink
                    disconnectPort('Liaison perdue');
                else
                    try, sp.NumBytesAvailable; catch, disconnectPort('Port indisponible'); end
                end
            end

            % --- Source des angles ---
            if isConnected
                roll = rollEst; pitch = pitchEst;
                set(sldR,'Value',clampv(roll,-90,90)); set(sldP,'Value',clampv(pitch,-90,90));
            elseif demoOn
                roll  = 38*sin(2*pi*tNow/14) + 3*sin(2*pi*1.3*tNow);
                pitch = 7*sin(2*pi*tNow/7+0.8) + 2.5*sin(2*pi*0.9*tNow);
                set(sldR,'Value',clampv(roll,-90,90)); set(sldP,'Value',clampv(pitch,-90,90));
            else
                roll = mRoll; pitch = mPitch;
            end
            if demoOn || autoYaw
                yaw = wrap180(yaw + 57.29578*9.81*tand(clampv(roll,-80,80))/CFG.turnV*dt);
                set(sldY,'Value',clampv(yaw,-180,180));
            else
                yaw = mYaw;
            end

            % --- Position monde (defilement du sol le long du cap) ---
            posX = mod(posX + CFG.speed*dt*cosd(yaw),2);
            posY = mod(posY - CFG.speed*dt*sind(yaw),2);

            % --- Matrices de transformation ---
            M = makehgtform('zrotate',-yaw*d2r,'yrotate',-pitch*d2r,'xrotate',roll*d2r);
            set(hBody,'Matrix',M);
            set(hProp,'Matrix',makehgtform('translate',[4.62 0 0],'xrotate', ...
                mod(2*pi*CFG.propRps*tNow,2*pi)));
            updateShadows(M);
            updateGround();
            set(hBeacon,'Visible',onoff(mod(tNow,1.2) < 0.12));

            % --- Horizon artificiel ---
            set(hH, 'Matrix',makehgtform('zrotate',roll*d2r,'translate',[0 -clampv(pitch,-60,60)*0.03 0]));
            set(hRS,'Matrix',makehgtform('zrotate',roll*d2r));

            % --- Camera d'introduction ---
            if ~introDone
                u = min(tNow/6,1); az = 55 + 105*(1-easeOut(u));
                view(ax,[az 18]);
                if u >= 1, introDone = true; end
            end

            % --- Afficheurs ---
            set(lblRoll, 'String',sprintf('%+6.1f%c',roll,176));
            set(lblPitch,'String',sprintf('%+6.1f%c',pitch,176));
            set(lblYaw,  'String',sprintf('%+6.1f%c',yaw,176));
            if mod(frameCnt,6) == 0, updateStatus(tNow); end
            drawnow limitrate;
        catch ME
            lastErr = ME.message;
            set(txtStat,'String',['Erreur: ' lastErr],'ForegroundColor',C.err);
        end
    end

    function updateGround()
        zl = zG + 0.02;
        for ig2 = 1:3
            Rg = gridR(ig2);
            kk = -ceil(Rg/2)-1 : ceil(Rg/2)+1;
            yr = 2*kk - posY; yr = yr(abs(yr) < Rg); xe = sqrt(Rg^2 - yr.^2);
            xr = 2*kk - posX; xr = xr(abs(xr) < Rg); ye = sqrt(Rg^2 - xr.^2);
            XA = [-xe; xe; nan(size(xe))];  YA = [yr; yr; nan(size(yr))];
            XB = [xr; xr; nan(size(xr))];   YB = [-ye; ye; nan(size(ye))];
            XD = [XA(:); XB(:)];  YD = [YA(:); YB(:)];
            set(gl(ig2),'XData',XD,'YData',YD,'ZData',zl*ones(size(XD)));
        end
    end

    function updateStatus(tNow)
        if isConnected
            if tNow - lastRx > CFG.rxTimeout
                set(txtMode,'String','FPGA / ESP32  |  EN ATTENTE DE DONNEES');
                set(txtStat,'String','Connecte - aucune donnee recue','ForegroundColor',C.warn);
            else
                set(txtMode,'String','FPGA / ESP32  |  TEMPS REEL');
                set(txtStat,'String','Connecte - flux actif','ForegroundColor',C.ok);
            end
            set(txtRate,'String',sprintf('%.0f Hz | |a| = %.2f g | rejetes : %d',rxRate,accNorm,badLines));
        else
            if demoOn, set(txtMode,'String','SIMULATION  |  MODE DEMO');
            else,      set(txtMode,'String','SIMULATION  |  PILOTAGE MANUEL');
            end
            set(txtRate,'String','');
            cur = get(txtStat,'String');
            if isempty(strfind(cur,'perdue')) && isempty(strfind(cur,'indisponible')) ...
                    && isempty(strfind(cur,'Echec')) && isempty(strfind(cur,'Erreur'))
                set(txtStat,'String','Hors ligne','ForegroundColor',C.dim);
            end
        end
    end

    function onClose(~,~)
        try, stop(tm); delete(tm); catch, end
        try, configureCallback(sp,'off'); catch, end
        sp = [];
        delete(fig);
    end
end

%% ======================================================================
%  FONCTIONS LOCALES
%  ======================================================================
function [X,Y,Z] = revolve(xs,rs,zc,th)
% Surface de revolution autour de l'axe X (secteur angulaire th).
X = xs(:)*ones(1,numel(th));
Y = rs(:)*cos(th(:)');
Z = zc + rs(:)*sin(th(:)');
end

function [X,Y,Z,xc] = wingSurf(x0,y0,z0,span,cr,ct,sweep,dih,tc,mirror)
% Voilure a profil NACA symetrique : effilement, fleche, dihedre (dih=90 -> derive).
nx = 25; beta = linspace(0,pi,nx); xc = (1-cos(beta))/2;
t  = 5*tc*(0.2969*sqrt(xc)-0.1260*xc-0.3516*xc.^2+0.2843*xc.^3-0.1036*xc.^4);
xl = [xc fliplr(xc(1:end-1))];
zl = [t -fliplr(t(1:end-1))];
ns = 14; s = linspace(0,1,ns)';
c   = cr + (ct-cr)*s;
xle = x0 - sweep*s;
sp  = span*s;
X = xle - c*xl;
Y = y0 + sp*cosd(dih) - (c*zl)*sind(dih);
Z = z0 + sp*sind(dih) + (c*zl)*cosd(dih);
if mirror, Y = -Y; end
end

function [X,Y,Z] = cylSurf(p1,p2,r,n)
% Cylindre de rayon r entre deux points quelconques.
p1 = p1(:); p2 = p2(:); a = p2-p1; a = a/norm(a);
if abs(a(1)) < 0.9, h = [1;0;0]; else, h = [0;1;0]; end
u = cross(a,h); u = u/norm(u); w = cross(a,u);
th = linspace(0,2*pi,n); ring = u*cos(th) + w*sin(th);
X = [p1(1)+r*ring(1,:); p2(1)+r*ring(1,:)];
Y = [p1(2)+r*ring(2,:); p2(2)+r*ring(2,:)];
Z = [p1(3)+r*ring(3,:); p2(3)+r*ring(3,:)];
end

function [X,Y,Z] = discSurf(c,a,r,n)
% Disque (bouchon) de rayon r, normal a, centre c.
c = c(:); a = a(:)/norm(a);
if abs(a(1)) < 0.9, h = [1;0;0]; else, h = [0;1;0]; end
u = cross(a,h); u = u/norm(u); w = cross(a,u);
th = linspace(0,2*pi,n); ring = u*cos(th) + w*sin(th);
X = [c(1)*ones(1,n); c(1)+r*ring(1,:)];
Y = [c(2)*ones(1,n); c(2)+r*ring(2,:)];
Z = [c(3)*ones(1,n); c(3)+r*ring(3,:)];
end

function K = kalmanInit(z0)
% Etat [angle; vitesse angulaire], covariance initiale large.
if nargin < 1, z0 = 0; end
K.x = [z0; 0];
K.P = diag([10 400]);
end

function K = kalmanStep(K,z,dt,R,q)
% Filtre de Kalman a vitesse constante, mesure = angle issu de l'accelerometre.
F = [1 dt; 0 1];
Q = q*[dt^3/3 dt^2/2; dt^2/2 dt];
x = F*K.x;  P = F*K.P*F' + Q;
y = wrap180(z - x(1));              % innovation (gere le repliement +-180)
S = P(1,1) + R;
G = P(:,1)/S;
x = x + G*y;
P = (eye(2) - G*[1 0])*P;
x(1) = wrap180(x(1));
K.x = x; K.P = P;
end

function a = wrap180(a)
a = mod(a+180,360) - 180;
end

function v = clampv(v,lo,hi)
v = min(max(v,lo),hi);
end

function s = onoff(tf)
if tf, s = 'on'; else, s = 'off'; end
end

function u = easeOut(u)
u = 1 - (1-u).^3;
end

function lst = listPorts(def)
% COM1..COM20 + ports detectes ; le port par defaut est place en premier.
lst = {};
try
    p = serialportlist("all");
    if ~isempty(p), lst = cellstr(p(:))'; end
catch
end
lst = unique([lst, arrayfun(@(k) sprintf('COM%d',k),1:20,'UniformOutput',false)],'stable');
lst = [{def}, setdiff(lst,{def},'stable')];
end
