
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
m = data.m;
% xt = ppval(data.XT,t);
%xt = 5.*sin(2.*pi.*t./200);
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

% without rest periods 
dx(:,5) = -0.085 - (0.283*u1).^2 - 0.566*(v1).^2;
dx(:,6) = -0.085 - (0.283*u2).^2 - 0.566*(v2).^2;

% with rest periods
% % dx(:,5) = -(0.7+tanh(-1.*(x1 - data.xb1))).*0.05 - (0.283*u1).^2 - 0.566*(v1).^2; % for charging location above the target
% dx(:,5) = -(0.7+tanh(1.*(x1 - data.xb1))).*0.05 - (0.283*u1).^2 - 0.566*(v1).^2; % for charging location below the target
% dx(:,6) = -(0.7+tanh(1.*(x2 - data.xb2))).*0.05 - (0.283*u2).^2 - 0.566*(v2).^2;


%------------- END OF CODE --------------