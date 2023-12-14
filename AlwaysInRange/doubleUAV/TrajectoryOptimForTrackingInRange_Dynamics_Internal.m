
function [dx, g_neq] = TrajectoryOptimForTrackingInRange_Dynamics_Internal(x,u,p,t,data)
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
m = data.m;
xt = ppval(data.XT,t);
% xt = 5.*sin(2.*pi.*t./200)+5;
%Define states
x1 = x(:,1);
x2 = x(:,2);
v1 = x(:,3);
v2 = x(:,4);
E1 =x(:,5);
E2 = x(:,6);

%Define inputs
u1 = u(:,1);
u2 = u(:,2);


%Define ODE right-hand side
dx(:,1) = v1;
dx(:,2) = v2;
dx(:,3) = u1./m;
dx(:,4) = u2./m;
% dx(:,5) = -(1-exp(-(x1 - data.xb).^2)).*0.1 - (0.283*u1).^2 - (0.566*v1).^2;
% dx(:,6) = -(1-exp(-(x2 - data.xb).^2)).*0.1 - (0.283*u2).^2 - (0.566*v2).^2;
% dx(:,5) = -tanh(10.*(x1 - data.xb)).*0.1 - (0.283*u1).^2 - (0.566*v1).^2;
% dx(:,6) = -tanh(10.*(x2 - data.xb)).*0.1 - (0.283*u2).^2 - (0.566*v2).^2;
% dx(:,5) = -(1+tanh(10.*(x1 - data.xb))).*0.1 - (0.283*u1).^2 - (0.566*v1).^2;
% dx(:,6) = -(1+tanh(10.*(x2 - data.xb))).*0.1 - (0.283*u2).^2 - (0.566*v2).^2;
dx(:,5) = -0.1 - (0.283*u1).^2 - (0.566*v1).^2;
dx(:,6) = -0.1 - (0.283*u2).^2 - (0.566*v2).^2;
% dx(:,5) = -0.1 - (m*9.81/2).*v1.^2;
% dx(:,6) = -0.1 - (m*9.81/2).*v2.^2;
% dx(:,5) = -0.1;
% dx(:,6) = -0.1;
% dx(:,5) = -0.05.*(1+tanh(10.*(x1+4.5))) - (0.2.*sqrt(2).*u1).^2 - (0.4.*sqrt(2).*v1).^2;
% dx(:,6) = -0.05.*(1+tanh(10*(x2+4.5))) - (0.2.*sqrt(2).*u2).^2 - (0.4.*sqrt(2).*v2).^2;
% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 
% 
%g_neq(:,1)= 0.5.*((x1 - xt).^2 + (x2 - xt).^2 - abs((x1 - xt).^2 - (x2 - xt).^2)) - data.delta.^2;

f1 = (x1 - xt).^2;
f2 = (x2 - xt).^2;
y = u(:,3);
soft_min = @(x,y) -log(exp(-x) + exp(-y));
% soft_max = @(x,y) max(x,y)+log(1+exp(min(x,y) - max(x,y)));
% g_neq(:,1) = f1 + (f1 - f2).*y - data.delta.^2;
% g_neq(:,1) = min(f1,f2) - data.delta.^2 - s.^2;
g_neq(:,1) = soft_min(f1,f2) - data.delta.^2;
% g_neq(:,1) = -soft_max(-f1,-f2) - data.delta.^2;
%------------- END OF CODE --------------