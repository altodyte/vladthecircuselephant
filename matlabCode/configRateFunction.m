%% initialize symbolic variables
% constants
syms t g T % time, gravity, torque [s, m/s^2, N-m]
syms l M % robot mass, ball mass [kg]
syms Il IM % robot wheel of inertia, ball moment of inertia [kg-m^2]
syms r R % robot radius, ball radius [m]
syms fl % friction force on robot from ball [N]
syms nl % normal force on robot from ball [N]
syms fM % friction force on ball from ground [N]

% state variables
phi = sym('phi(t)');
psiV = sym('psiV(t)');

% geometric relationship
x = r/2/pi*psiV + R/2/pi*phi;

%% lambda l equations of motion
% accelerations
rl = [x; R] + (R+r)*[sin(phi); cos(phi)];
vl = simplify(diff(rl, t), 10);
al = simplify(diff(vl, t), 10);
alphal = diff(psiV, t, 2);

% sums of forces and torques
sumfl = [0; -l*g] + fl*[cos(phi); -sin(phi)] + nl*[sin(phi); cos(phi)];
sumtl = fl*r + T;

% F=ma
eomsl = [sumfl == l*al; sumtl == Il*alphal];

%% ball M equations of motion
% accelerations (ihat)
rM = x;
vM = simplify(diff(rM, t), 10);
aM = simplify(diff(vM, t), 10);
alphaM = -aM/R;

% sums of forces (ihat) and torques
sumfM = fM - fl*cos(phi) - nl*sin(phi);
sumtM = R*(fM + fl);

% F=ma
eomsM = [sumfM == M*aM; sumtM == IM*alphaM];

%% rewrite equations of motion into system of equations
stateSyms = [phi psiV];
stateVars = {'phi' 'psi'};
dstateSyms = diff(stateSyms, t);
dstateVars = {'dphi', 'dpis'};
ddstateSyms = diff(dstateSyms, t);
ddstateVars = {'ddphi' 'ddpsi'};
% stateSyms = {'diff(phi, t, t)', 'diff(psi, t, t)', ...
%     'diff(phi, t)', 'diff(psi, t)', ...
%     phi, psiV};
% stateVars = {'ddphi', 'ddpsi', 'dphi', 'dpsi', 'phi', 'psi'};
eoms = [eomsl; eomsM];
eoms_expr = subs(eoms, ddstateSyms, ddstateVars);
eoms_expr = subs(eoms_expr, dstateSyms, dstateVars);
eoms_expr = subs(eoms_expr, stateSyms, stateVars);
vars = {'ddphi'; 'ddpsi'; fl; nl; fM};
[A, b] = equationsToMatrix(eoms_expr, vars);
matlabFunction(A, b, 'File', 'Eoms.m');