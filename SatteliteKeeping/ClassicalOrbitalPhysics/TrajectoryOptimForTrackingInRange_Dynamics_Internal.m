
function [dx] = TrajectoryOptimForTrackingInRange_Dynamics_Internal(x,u,p,t,data)
% Template for specifying the dynamics for internal model 
%
% Syntax:  
%          [dx] = myProblem_Dynamics_Internal(x,u,p,t,vdat)	(Dynamics Only)
%          [dx,g_eq] = myProblem_Dynamics_Internal(x,u,p,t,vdat)   (Dynamics and Eqaulity Path Constraints)
%          [dx,g_neq] = myProblem_Dynamics_Internal(x,u,p,t,vdat)   (Dynamics and Inqaulity Path Constraints)
%          [dx,g_eq,g_neq] = myProblem_Dynamics_Internal(x,u,p,t,vdat)   (Dynamics, Equality and Ineqaulity Path Constraints)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    data - structured variable containing the values of additional data used inside
%          the function%      
% Output:
%    dx - time derivative of x
%    g_eq - constraint function for equality constraints
%    g_neq - constraint function for inequality constraints
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------

%Stored data
mu = data.mu;

%Define states
% z = [a, h, k,p, q, \lambda]^T

a = x(:,1);
h = x(:,2);
k = x(:,3);
p = x(:,4);
q = x(:,5);
lambda = x(:,6);


% a - semi major, i - incidence, e - eccentricity,  ω the argument of perigee, Ω the RAAN and L is the true longitude. 

e = sqrt(h.^2 + k.^2);
i = 2.*atan2(sqrt(p.^2 + q.^2),1);
O = atan2(p,q);
w = atan2(h,k) - atan2(p,q);
M = lambda - atan2(h,k);
G = sqrt(1 - h.^2 - k.^2);
beta = 1./(1+G);
n = sqrt(mu).*a.^(-3/2);

%Define inputs
F = u(:,1);

cf = cos(F);
sf = sin(F);

%unit vecs (????) SORT THIS SHIT    
f_cap = 0;
g_cap = 0;
w_cap = 0;

%Define diffrential equation elements
r = a.*(1 - k.*cf - h.*sf);

X1 = a.* ((1 - h.^2 .* beta).*cf + h.*k.*beta.*sf - k);
Y1 = a.* (h.*k.*beta.*cf + (1 - k.^2.*beta).*sf - h);
dX1 = a.^2 .*n.* r.^(-1) .* (h.*k.*beta.*cf - (1 - h.^2 .* beta).*sf);
dY1 = a.^2 .*n.* r.^(-1) .* ((1 - k.^2 .* beta).*cf - h.*k.*beta.*sf);
dX1dh = a.*(-(h.*cf - k.*sf).*(beta + (h.^2 .* beta.^3)./(1-beta)) - (a./r) .* cf.*(h.*beta - sf));
dX1dk = -a.*((h.*cf - k.*sf).*(h.^2 .* k .* beta.^3)./(1-beta) + 1 + (a./r) .* sf.*(sf - h.*beta));
dY1dh = a.*((h.*cf - k.*sf).*(h.*k.*beta.^3./(1-beta)) - 1 + (a./r).*cf.*(k.*beta - cf));
dY1dk = a.*((h.*cf - k.*sf).*(beta + (k.^2 .* beta.^3 ./ (1 - beta))) + (a./r).*sf.*(cf - k.*beta));

%Define ODE 
dadr = 2.*a.^(-1).*n.^(-2).*(dX1.*f_cap + dY1.*g_cap);
dhdr = G.*n.^(-1).*a.^(-2).*((dX1dk - h.*beta.*dX1./n).*f_cap + (dY1dk - h.*beta.*dY1./n).*g_cap);
dkdr = -G.*n.^(-1).*a.^(-2).*((dX1dh + k.*beta.*dX1./n).*f_cap + (dY1dh + k.*beta.*dY1./n).*g_cap);
dpdr = (K.*Y1.*n.^(-1).*a.^(-2).*G.^(-1)./2).*w_cap;
dqdr = (K.*X1.*n.^(-1).*a.^(-2).*G.^(-1)./2).*w_cap;
dlambdadr = n.^(-1).*a.^(-2).*(-2.*X1 + G.*(h.*beta.*dX1dh + k.*beta.*dX1dk)).*f_cap...
            + n.^(-1).*a.^(-2).*(-2.*Y1 + G.*(h.*beta.*dY1dh + k.*beta.*dY1dk)).*g_cap...
            + n.^(-1).*a.^(-2).*G^(-1).*(q.*Y1 - p.*X1).*w_cap;
dx(:,1) = dadr;
dx(:,2) = dhdr;
dx(:,3) = dkdr;
dx(:,4) = dpdr;
dx(:,5) = dqdr;
dx(:,6) = dlambdadr;

% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 
% 


%------------- END OF CODE --------------