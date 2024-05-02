
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
% G0 = data.G0;
% H0 = data.H0;
m1 = data.m1;
m2 = data.m2;
m3 = data.m3;
m4 = data.m4;
dist = data.disturb;

%Define states
x1 = x(:,1) + dist(t);
x2 = x(:,2);

%Define inputs
u1 = u(:,1);


%Define ODE right-hand side
dx(:,1) = -m1.*x1 - m2.*x2;
dx(:,2) = -m3.*x2 + m4.*x1 + u1;

% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 
% 


%------------- END OF CODE --------------