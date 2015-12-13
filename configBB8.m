function configBB8
BB8 % open simulink project
BB8_sim % open simulink project

%% conversion factors
in2m = convlength(1, 'in', 'm');

%% plant constants
% global
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
Bpsi = 3e-6; % effective viscous damping about the motor [N-m-s/rad]

% derived values
Km = Kt/(Ra*Bpsi + Kt*Ke);
tm = Ra*Jpsi/(Ra*Bpsi + Kt*Ke);

%% compensator parameters
% position minor loop around motor
Kv = 0; % motor velocity feedback loop gain
Kp = 1; % motor position feedback loop gain
Kk = -5000; % K gain
tkp = 10; % K pole time constant
tkz = 2/3; % K zero time constant

% integral compensator
Kk1 = 200; % K gain
tkz1 = 1/4; % K zero time constant

% velocity minor loop around motor
Kv1 = 0; % motor velocity feedback loop gain

%% compensator derived values
% position minor loop around motor
Kmv = Km/(1 + Kv*Km);
tmv = tm/(1 + Kv*Km);
Kmp = 1/Kp;
te = (sqrt(1+4*Kp*Kmv*tmv)+1)/(2*Kp*Kmv);
tmp = (sqrt(1+4*Kp*Kmv*tmv)-1)/(2*Kp*Kmv);

% velocity minor loop around motor
Kmv1 = Km/(1 + Kv1*Km);
tmv1 = tm/(1 + Kv1*Km);

%% compensator analysis
s = tf('s');
K = Kk*(tkz*s+1)/(tkp*s+1);
Ma = Km/(tm*s+1);
G = C*s^2/(tL*s+1)/(tL*s-1);
Mv = Kmv/s/(tmv*s+1); % algebraically-found velocity feedback motor TF
% Mv2 = minreal(M/(1+Kv*M)/s);
Mp = Kmp/(te*s-1)/(tmp*s+1); % algebraically-found position feedback motor TF
% Mp2 = minreal(Mv2/(1-Kp*Mv2));

%% proportional compensator
% rlocus(-M/s*G);
% figure;
% pzmap(M/s,G);
% legend('M','G');

%% integral compensator
K1 = Kk1*(tkz1*s+1)/s;
% rlocus(-K1*M/s*G);
% figure;
% pzmap(K1,M/s,G);
% legend('K1','M','G');
% figure;
% margin(-K1*M/s*G);
% figure;
% step(-K1*M/s*G/(1-K1*M/s*G));
% figure;
% nyquist(-K1*M/s*G);

%% velocity minor loop around motor
Mv1 = Kmv1/s/(tmv1*s+1); % algebraically-found velocity feedback motor TF
% rlocus(-K1*Mv1*G);
% figure;
% pzmap(K1,Mv1,G);
% legend('K1','Mv1','G');
% figure;
% nyquist(-K1*Mv1*G);

%% position minor loop around motor
% rlocus(K*Mp*G);
% figure;
% pzmap(K,Mp,G);
% legend('K','Mp','G');
% figure;
% margin(K*Mp*G);
% [Gm, Pm, Wgm, Wpm] = margin(K*Mp*G);
% Pm
% step(minreal(K*Mp*G/(1+K*Mp*G)));
% step(K*Mp*G/(1+K*Mp*G),20);
% figure;
% nyquist(K*Mp*G);
% figure;

% sysd = c2d(K, 0.002);
% [Num, Den, ~] = tfdata(sysd);
% vpa(Num{:}, 6)
% vpa(Den{:}, 6)

%% setting block properties from parameters
% http://www.mathworks.com/help/simulink/ug/using-model-workspaces.html
% http://www.mathworks.com/help/simulink/slref/simulink.modelworkspace.html#f29-123886
hws = get_param('BB8_sim', 'modelworkspace');
hws.assignin('Kt', Kt);
hws.assignin('Ke', Ke);
hws.assignin('Ra_inv', 1/Ra);
hws.assignin('IM', IM);
hws.assignin('Il', Ilambda);
hws.assignin('M', M);
hws.assignin('R', R);
hws.assignin('g', g);
hws.assignin('l', lambda);
hws.assignin('r', r);
% hws.whos
% set_param('BB8_sim/Motor_elec/Kt', 'Gain', num2str(Kt, 5));
% set_param('BB8_sim/Motor_elec/Ke', 'Gain', num2str(Ke, 5));
% set_param('BB8_sim/Motor_elec/Ra_inv', 'Gain', num2str(1/Ra, 5));
% set_param('BB8_sim/Plant/IM', 'Value', num2str(IM, 5));
% set_param('BB8_sim/Plant/Il', 'Value', num2str(Ilambda, 5));
% set_param('BB8_sim/Plant/M', 'Value', num2str(M, 5));
% set_param('BB8_sim/Plant/R', 'Value', num2str(R, 5));
% set_param('BB8_sim/Plant/g', 'Value', num2str(g, 5));
% set_param('BB8_sim/Plant/l', 'Value', num2str(lambda, 5));
% set_param('BB8_sim/Plant/r', 'Value', num2str(r, 5));

[z, p, k] = zpkdata(K);
setVal('K', 'Gain', k);
setVal('K', 'Poles', p{:}');
setVal('K', 'Zeros', z{:}');
% setVal('K', 'Gain', Kk/(tkp*tkz));
% setVal('K', 'Poles', -1/tkp);
% setVal('K', 'Zeros', -1/tkz);

[z, p, k] = zpkdata(Ma);
setVal('M', 'Gain', k);
setVal('M', 'Poles', p{:}');
setVal('M', 'Zeros', z{:}');
% setVal('M', 'Gain', Km/tm);
% setVal('M', 'Poles', -1/tm);
% setVal('M', 'Zeros', []);
setGain('Kv', Kv);
setGain('Kp', Kp);

[z, p, k] = zpkdata(G);
setVal('G', 'Gain', k);
setVal('G', 'Poles', p{:}');
setVal('G', 'Zeros', z{:}');
% setVal('G', 'Gain', C/tL^2);
% setVal('G', 'Poles', [-1/tL 1/tL]);
% setVal('G', 'Zeros', [0 0]);

    function setVal(block, prop, val)
        set_param(['BB8/' block], prop,  ['[' num2str(val, 5) ']']);
    end

    function setGain(block, gain)
        set_param(['BB8/' block], 'Gain', num2str(gain, 5));
    end
end