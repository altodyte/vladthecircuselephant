function configBB8
% BB8 % open simulink project
% BB8_sim % open simulink project

%% psi input model (https://en.wikipedia.org/wiki/Smoothstep)
stepMax = 10; % final value of step [rad]
stepStart = 1; % start time of step [s]
stepDur = 4; % duration of step [s]
% stepCoefs = stepMax*[-2 3 0 0]; % coefficients of smoothstep
stepCoefs = stepMax*[6 -15 10 0 0 0]; % coefficients of smootherstep

%% conversion factors
in2m = convlength(1, 'in', 'm');

%% plant constants
% global
s = tf('s');
g = 9.8;

% ball M
R = in2m*14; % radius of ball [m]
M = 1.12; % mass of ball [kg]
IM = 2/3*M*R^2;

% robot lambda
r = in2m*2; % radius of robot wheel [m]
lambda = 1.5; % mass of robot [kg]
Ilambda = 2*1.918e-4; % moment of inertia of robot wheels [kg-m^2]

% derived values
C = -(r + M*r/(2*lambda) + IM*r/(2*R^2*lambda))/g;
tL = sqrt((R + 2*pi*(R+r) + M*R/(2*lambda) + IM/(2*R*lambda))/g);
K = 2*pi*Ilambda/r - lambda*r;
L = lambda*R + 2*pi*lambda*(R+r);
N = M*r/2 + IM*r/(2*R^2) + 2*pi*Ilambda/r;
P = M*R/2 + IM/(2*R);
Jpsi = (r*N*L+r*P*K)/(2*pi*L+2*pi*P); % effective moment of inertia about motor [kg-m^2]

%% motor constants
% parameters
Kt = 2*0.61; % motor torque constant, taking into account 2 motors [...]
Ke = 0.61; % motor emf constant [...]
Ra = 2.5; % series resistance of motor [ohm]
Bpsi = 2*3e-6; % effective viscous damping about the motor [N-m-s/rad]

% derived values
Km = Kt/(Ra*Bpsi + Kt*Ke);
tm = Ra*Jpsi/(Ra*Bpsi + Kt*Ke);

%% compensator parameters
% position minor loop and double lag compensator
Kp = 0.5; % motor position feedback loop gain
Kk = -1500; % K gain
tkp = 4; % K pole time constant
tkz = 1; % K zero time constant

% position minor loop and single lag compensator
% Kp = 0.75; % motor position feedback loop gain
% Kk = -400; % K gain
% tkp = 2; % K pole time constant
% tkz = 1/4; % K zero time constant

% only lag compensator
% Kk = -1000; % K gain
% tkp = 2; % K pole time constant
% tkz = 1/4; % K zero time constant

% velocity minor loop
Kv = 0; % motor velocity feedback loop gain

% only integral compensator
% Kk = -400; % gain
% tkz = 1/4; % zero time constant

% only proportional compensator
% Kk = -10000;

%% compensator derived values
% position minor loop around motor
Kmv = Km/(1 + Kv*Km);
tmv = tm/(1 + Kv*Km);
Kmp = 1/Kp;
te = (sqrt(1+4*Kp*Kmv*tmv)+1)/(2*Kp*Kmv);
tmp = (sqrt(1+4*Kp*Kmv*tmv)-1)/(2*Kp*Kmv);

% velocity minor loop around motor
% Kmv = Km/(1 + Kv*Km);
% tmv = tm/(1 + Kv*Km);
 
%% compensator transfer functions
% K = Kk; % proportional
% K = Kk*(tkz*s+1)/s; % integral
% K = Kk*(tkz*s+1)/(tkp*s+1); % single lag
K = Kk*((tkz*s+1)/(tkp*s+1))^2; % double lag

%% plant transfer function
G = C*s^2/(tL*s+1)/(tL*s-1);

%% motor transfer functions
% motor armature voltage to velocity (proportional and integral)
% Ma = Km/(tm*s+1);

% pre-velocity voltage to position TF with velocity feedback
% Mv = Kmv/s/(tmv*s+1); % algebraically-found
% Mv = minreal(M/(1+Kv*M)/s); % symbolically-found TF

% pre-position voltage to position TF with position feedback
Mp = Kmp/(te*s-1)/(tmp*s+1); % algebraically-found
% Mp = minreal(Mv/(1-Kp*Mv)); % symbolically-found TF

figure('OuterPosition',[800+1 40+1 800 900-80]); % left, bottom, width, height
%% proportional compensator analysis
% rlocus(-Ma/s*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Proportional Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% rlocus(-Ma/s*G);
% axis([-5 5 -5 5]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Proportional Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% pzmap(Ma/s,G);
% legend('M','G');
% margin(K*Ma/s*G);
% nyquist(K*Ma/s*G);
% axis equal
% step((K*Ma/s)/(1+K*Ma/s*G)/10, 0:0.001:2);
% title('Step Response; K = -10000');

%% integral compensator analysis
% rlocus(K*Ma/s*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Integral Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% rlocus(K*Ma/s*G, logspace(-2, 2, 20000));
% axis([-10 4 -7 7]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Integral Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% pzmap(K,Ma/s,G);
% legend('K','Ma','G');
% margin(K*Ma/s*G);
% nyquist(K*Ma/s*G);
% axis equal
% step(minreal(K*Ma/s/(1+K*Ma/s*G)/10), 100);
% h = title('Step Response; $K = \frac{-400(0.25s+1)}{s}$');
% set(h, 'interpreter', 'latex');

%% integral compensator and velocity minor loop analysis
% rlocus(K*Mv*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Velocity Minor Loop and Integral Compensator Root Locus');
% set(h, 'interpreter', 'latex');

%% lag compensator
% rlocus(K*Mv*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% rlocus(K*Mv*G, logspace(-3, 2, 20000));
% axis([-10 4 -7 7]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% pzmap(K, Mv, G);
% axis([-10 4 -7 7]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Lag Compensator Pole-Zero Map');
% set(h, 'interpreter', 'latex');
% legend('K', 'Mv', 'G', 'location', 'best');
% nyquist(K*Mv*G);
% axis equal
% margin(K*Mv*G);

%% lag compensator and position minor loop analysis
% rlocus(K*Mp*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Position Minor Loop and Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% rlocus(K*Mp*G, logspace(-3, 2, 10000));
% axis([-10 4 -7 7]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Position Minor Loop and Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% step(minreal(K*Mp/(1+K*Mp*G)/10), 20);
% h = title('Step Response; $K = -400\frac{0.25s+1}{2s+1}$; $K_p = 0.75$');
% set(h, 'interpreter', 'latex');
% nyquist(K*Mp*G);
% axis equal
% margin(K*Mp*G);

%% double lag compensator and position minor loop analysis
% rlocus(K*Mp*G);
% axis([-450 50 -250 250]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Position Minor Loop and Double Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% rlocus(K*Mp*G);
% axis([-10 4 -7 7]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Position Minor Loop and Double Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% pzmap(minreal(K*Mp/(1+K*Mp*G)/10));
% axis([-5 3 -4 4]);
% hpos = get(gcf, 'Position');
% set(gcf, 'Position', [hpos([1 2 4]) hpos(4)]);
% h = title('Position Minor Loop and Double Lag Compensator Root Locus');
% set(h, 'interpreter', 'latex');
% step(minreal(K*Mp/(1+K*Mp*G)/10), 40);
% h = title('Step Response; $K = -1500\frac{s+1}{4s+1}^2$; $K_p = 0.5$');
% set(h, 'interpreter', 'latex');
% nyquist(K*Mp*G);
% axis equal
% margin(K*Mp*G);

% pzmap(Ma, Mp, G);
% legend('M', 'Mp', 'G');
% figure;
% subplot(2,2,1);
% rlocus(K*Mp*G);
% subplot(2,2,2);
% pzmap(K,Mp,G);
% legend('K','Mp','G');
% subplot(2,2,3);
% margin(K*Mp*G);
% subplot(2,2,4);
% nyquist(K*Mp*G);

% sysd = c2d(K, 0.010)
% [Num, Den, ~] = tfdata(sysd);
% vpa(Num{:}, 10)
% vpa(Den{:}, 10)

%% setting block properties from parameters
hws = get_param('BB8_sim', 'modelworkspace');
hws.assignin('Kt', Kt);
hws.assignin('Ke', Ke);
hws.assignin('Ra', Ra);
hws.assignin('IM', IM);
hws.assignin('Il', Ilambda);
hws.assignin('M', M);
hws.assignin('R', R);
hws.assignin('g', g);
hws.assignin('l', lambda);
hws.assignin('r', r);
hws.assignin('stepMax', stepMax);
hws.assignin('stepStart', stepStart);
hws.assignin('stepDur', stepDur);
hws.assignin('stepCoefs', stepCoefs);
% hws.whos

% setGain('Kv', Kv);
% setGain('Kp', Kp);
hws.assignin('Kv', Kv);
hws.assignin('Kp', Kp);

[z, p, k] = zpkdata(K);
hws.assignin('Kgain', k);
hws.assignin('Kpoles', p{:});
hws.assignin('Kzeros', z{:});
% setVal('K', 'Gain', k);
% setVal('K', 'Poles', p{:}');
% setVal('K', 'Zeros', z{:}');

% [z, p, k] = zpkdata(Ma);
% setVal('M', 'Gain', k);
% setVal('M', 'Poles', p{:}');
% setVal('M', 'Zeros', z{:}');

% [z, p, k] = zpkdata(G);
% setVal('G', 'Gain', k);
% setVal('G', 'Poles', p{:}');
% setVal('G', 'Zeros', z{:}');

    function setVal(block, prop, val)
        set_param(['BB8/' block], prop,  ['[' num2str(val, 5) ']']);
    end

    function setGain(block, gain)
        set_param(['BB8/' block], 'Gain', num2str(gain, 5));
    end
end