function configBB8
BB8 % open simulink project
BB8_sim % open simulink project

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
% position minor loop around motor and lag compensator(s)
Kv = 0; % motor velocity feedback loop gain
Kp = 0.5; % motor position feedback loop gain
% double lag
Kk = -4000; % K gain
tkp = 4; % K pole time constant
tkz = 1; % K zero time constant
% single lag
% Kk = -8000; % K gain
% tkp = 8; % K pole time constant
% tkz = 1; % K zero time constant

% only integral compensator
% Kk = 200; % gain
% tkz = 1/4; % zero time constant

% only velocity minor loop around motor
% Kv = 0; % motor velocity feedback loop gain

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
% motor armature voltage to velocity
Ma = Km/(tm*s+1);

% pre-velocity voltage to position TF with velocity feedback
Mv = Kmv/s/(tmv*s+1); % algebraically-found
% Mv = minreal(M/(1+Kv*M)/s); % symbolically-found TF

% pre-position voltage to position TF with position feedback
Mp = Kmp/(te*s-1)/(tmp*s+1); % algebraically-found
% Mp = minreal(Mv/(1-Kp*Mv)); % symbolically-found TF

%% proportional compensator analysis
% rlocus(-M/s*G);
% figure;
% pzmap(M/s,G);
% legend('M','G');

%% integral compensator analysis
% rlocus(-K*M/s*G);
% figure;
% pzmap(K,M/s,G);
% legend('K','M','G');
% figure;
% margin(-K*M/s*G);
% figure;
% step(-K*M/s*G/(1-K*M/s*G));
% figure;
% nyquist(-K*M/s*G);

%% integral compensator and velocity minor loop analysis
% rlocus(-K*Mv*G);
% figure;
% pzmap(K,Mv,G);
% legend('K','Mv','G');
% figure;
% nyquist(-K*Mv*G);

%% lag compensator, velocity minor loop, and position minor loop analysis
% pzmap((K*Mp)/(1+K*Mp*G));
% margin(-2000*Mp*G);
% [Gm, Pm, Wgm, Wpm] = margin(K*Mp*G);
% Pm
% step(minreal(K*Mp*G/(1+K*Mp*G)));
% step(K*Mp*G/(1+K*Mp*G),20);

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

% sysd = c2d(K, 0.002)
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