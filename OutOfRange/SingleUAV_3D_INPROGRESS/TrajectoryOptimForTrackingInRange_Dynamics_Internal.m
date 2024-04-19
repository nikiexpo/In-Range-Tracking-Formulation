
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
% x = [x,y,z,Vair,Vvert,theta,E]
%Stored data
m = data.m;
g = data.g;
c1 = data.c1;
c2 = data.c2;
c3 = data.c3;
c4 = data.c4;
c5= data.c5;
c6= data.c6; 
dia = data.dia;
maxT = data.maxT;
k1 = data.k1;
k2  = data.k2; 
RPM_max = data.RPM_max;
% xt = ppval(data.XT,t);
%xt = 5.*sin(2.*pi.*t./200);
%Define states
posx = x(:,1);
posy = x(:,2);
posz = x(:,3);
Vair = x(:,4);
Vvert = x(:,5);
theta = x(:,6);
E = x(:,7);

%Define inputs
gamma = u(:,1);
theta_tilde = u(:,2);
alpha = u(:,3);

%un
% N = interp1(data.gammaLookup(:,1), data.gammaLookup(:,2), gamma);
J=  65.89 .* (Vair.*sin(alpha).*cos(theta_tilde-theta)) ./(gamma.*(RPM_max-100)+100);
eff=(-0.625.*(J.^2)+1.25.*J+0.255);
eff(eff<0.78)=0.78;
% L = c5.*(Vair.*cos(alpha))^2;
% D = c4.*Vair.^2;
thr=interp1(data.gammaLookup(:,1),data.gammaLookup(:,2),gamma,'spline');
thr(thr<0) = 0;
T = maxT .* eff .* thr;
Pi = k1.*T.*(Vvert./2 + sqrt((Vvert./2).^2 + T./k2^2));
Pp = c2.*T.^(3/2) + c3.*(Vair.*cos(alpha)).^2 .* T.^(1/2);
Ppar = c4.*Vair.^3;



%Define ODE right-hand side
dx(:,1) = Vair.*cos(theta);
dx(:,2) = Vair.*sin(theta);
dx(:,3) = Vvert;
dx(:,4) = (T.*cos(theta_tilde - theta).*sin(alpha) - c4.* Vair.^2)./m;
dx(:,5) = (T.*cos(alpha) + c5.*(Vair .* cos(alpha)).^2)./m -g;
dx(:,6) = (T.*sin(theta_tilde - theta).*sin(alpha))./(m.*Vair); %vair cannot be zero
dx(:,7) = - (Pi + Ppar + Pp);

% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 
% 
% xt = ppval(data.XT, t); % Target x position
% yt = ppval(data.YT, t); % Target y position
% g_eq = (posx-xt).^2 + (posy-yt).^2 -(posz.* 40/130).^2;
% g_eq = (posx-xt).^2 + (posy-yt).^2 -(40).^2;
%------------- END OF CODE --------------