function [A,b] = Eoms(IM,Il,M,R,T,dphi,g,l,phi,r)
%EOMS
%    [A,B] = EOMS(IM,IL,M,R,T,DPHI,G,L,PHI,R)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    07-Dec-2015 15:09:08

t2 = 1.0./pi;
t3 = cos(phi);
t4 = sin(phi);
t5 = R+r;
A = reshape([-l.*(R.*t2.*(1.0./2.0)+t3.*t5),l.*t4.*t5,0.0,M.*R.*t2.*(-1.0./2.0),IM.*t2.*(1.0./2.0),l.*r.*t2.*(-1.0./2.0),0.0,-Il,M.*r.*t2.*(-1.0./2.0),(IM.*r.*t2.*(1.0./2.0))./R,t3,-t4,r,-t3,R,t4,t3,0.0,-t4,0.0,0.0,0.0,0.0,1.0,R],[5,5]);
if nargout > 1
    t6 = dphi.^2;
    b = [-l.*t4.*t5.*t6;g.*l-l.*t3.*t5.*t6;-T;0.0;0.0];
end
