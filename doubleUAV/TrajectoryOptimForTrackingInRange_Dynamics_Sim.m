
function [dx] = myProblem_Dynamics_Sim(x,u,p,t,data)
% Template for specifying the dynamics for simulation 
%
% Syntax:  
%          [dx] = myProblem_Dynamics_Sim(x,u,p,t,vdat)
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

%Define states
x1 = x(:,1);
v1 = x(:,2);
E =x(:,3);

%Define inputs
u1 = u(:,1);


%Define ODE right-hand side
dx(:,1) = v1;
dx(:,2) = u1./m;
dx(:,3) = -0.1 - (0.283*u1).^2 - (0.566*v1).^2;

% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 

g_neq(:,1)= (x1 - sin(2.*pi.*t./50));

%------------- END OF CODE --------------