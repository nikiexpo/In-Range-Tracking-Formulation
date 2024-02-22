
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

%Define states
c11 = x(:,1); 
c21 = x(:,2);
c31 = x(:,3); 
c12 = x(:,4); 
c22 = x(:,5);
c32 = x(:,6); 
c13 = x(:,7); 
c23 = x(:,8);
c33 = x(:,9); 
c14 = x(:,10); 
c24 = x(:,11);
c34 = x(:,12); 

%Define inputs
u11 = u(:,1); 
u21 = u(:,2); 
u12 = u(:,3); 
u22 = u(:,4); 
u13 = u(:,5); 
u23 = u(:,6); 
u14 = u(:,7); 
u24 = u(:,8); 

%Define Data
F = data.F;
Fu = data.Fu;
V = data.V;

%Define reaction rates
for i=1:4

    if i == 1
        v1(i) = k1(i) .* (1+eta(i) .* Ccu(i)).*(Co2(i)).^beta .* (c3(i)).^gamma;
    else
        v1(i) = k1(i) .* (1+eta(i) .* Ccu(i)).*(c1(i)).^alpha.*(Co2(i)).^beta .* (c3(i)).^gamma;
    end

    v2(i) = k2(i) .* c2(i);
    v3(i) = k3(i) .* u2(i) .* c3(i);
end

for i = 1:4
    
    if i == 1
        A1 = [F./V, 0, 0;
                0, F./V, 0;
                0, 0, F./V];
    else
    
        A1 = [(F+Fu)./V, 0 ,0;
                0, (F+Fu)./V, 0;
                0, 0, (F+Fu)./V];
    end
    
    A2 = [-(F+Fu)./V, 0 ,0;
            0, -(F+Fu)./V, 0;
            0, 0, -(F+Fu)./V];

    phi(i) = [-4.*v1(i), 4.*v1(i) - v2(i), -4.*v1(i)+3.*v2(i)-2.*v3(i)]';

    cdot(:,i) = A1*c0(:,i) %CHANGE EVERYTHING TO NON MATRIX FORM FOR EASIER COMPUTING
end


%Define ODE right-hand side
dx(:,1) = v1;
dx(:,2) = u1./m;
dx(:,3) = -0.1 - (0.283*u1).^2 - (0.566*v1).^2;

% %Define Path constraints
% g_eq(:,1)=g_eq1(x1,...,u1,...p,t);
% g_eq(:,2)=g_eq2(x1,...,u1,...p,t);
% ...
% 
% 
g_neq(:,1)= (x1 - xt).^2 - data.delta.^2;

%------------- END OF CODE --------------