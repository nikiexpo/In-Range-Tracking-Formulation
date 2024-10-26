
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

%Define states
c1 = x(:,1); 
c2 = x(:,2);
c3 = x(:,3); 

%Define inputs
u1 = u(:,1); 
u2 = u(:,2); 

%Define Data
F = data.F;
Fu = data.Fu;
V = data.V;
eta = data.eta;
Ccu = data.Ccu;
beta = data.beta;
gamma = data.gamma;
alpha = data.alpha;
lambda = data.lambda;
k1 = data.k1;
k2 = data.k2;
k3 = data.k3;
Cfe2 = data.Cfe2;
Cfe3 = data.Cfe3;
pH = data.pH;

Co2 = log(lambda.*u1 + 1);
c0 = [Cfe2, Cfe3, 10^-pH];
%Define reaction rates


v1 = k1 .* (1+eta .* Ccu).*(Co2).^beta .* (c3).^gamma;
v2 = k2 .* c2;
v3 = k3 .* u2 .* c3;


A1 = F./V;
    
A2 = -(F+Fu)./V;

phi = [-4.*v1, 4.*v1 - v2, -4.*v1+3.*v2-2.*v3];

cdot(:,1) = A1.*c0(1) + A2.*c1 + phi(:,1);
cdot(:,2) = A1.*c0(2) + A2.*c2 + phi(:,2);
cdot(:,3) = A1.*c0(3) + A2.*c3 + phi(:,3);


dx = cdot;

%------------- END OF CODE --------------