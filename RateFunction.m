function [dphi, ddphi, ddpsi] = RateFunction(T, phi, dphi)

IM = 0;
Il = 0;
M = 0;
R = 0;
g = 0;
l = 0;
r = 0;
[A, b] = Eoms(IM, Il, M, R, T, dphi, g, l, phi, r);

vars = A\b;
ddphi = vars(1);
ddpsi = vars(2);
% fl = vars(3);
% nl = vars(4);
% fM = vars(5);

end