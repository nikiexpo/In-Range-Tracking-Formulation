
function [dx,g_eq, g_neq] = TrajectoryOptimForTrackingInRange_Dynamics_Internal(x,u,p,t,data)
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
at = data.at;
delta = data.delta;
ref = data.ref; % SHOULD BE raduies of CIRCLE

% states [rx,ry,rz,vx,vy,vz]
rx = x(:,1);
ry = x(:,2);
rz = x(:,3);
vx = x(:,4);
vy = x(:,5);
vz = x(:,6);
E = x(:,7);

% controls [ax,ay,az]
ax = u(:,1);
ay = u(:,2);
az = u(:,3);

r = sqrt(rx.^2 + ry.^2 + rz.^2);
%ode
dx(:,1) = vx;
dx(:,2) = vy;
dx(:,3) = vz;
dx(:,4) = -mu./r.^3 .* rx + ax; 
dx(:,5) = -mu./r.^3 .* ry + ay; 
dx(:,6) = -mu./r.^3 .* rz + az; 
dx(:,7) = -at;

%const -- force thrust vectoring by eq const on the total acceleration
g_eq = sqrt(ax.^2 + ay.^2 + az.^2) - at; % = 0 ? 

g_neq = (ref - r).^2 - delta.^2; 
%------------- END OF CODE --------------