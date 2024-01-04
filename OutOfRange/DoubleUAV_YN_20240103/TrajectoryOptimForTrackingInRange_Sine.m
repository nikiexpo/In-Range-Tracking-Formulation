function [problem,guess] = TrajectoryOptimForTrackingInRange_Sine
%myProblem - Template file for optimal control problem definition
%
%Syntax:  [problem,guess] = myProblem
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
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
% Plant model name, provide in the format of function handle
InternalDynamics=@TrajectoryOptimForTrackingInRange_Dynamics_Internal; 
SimDynamics=@TrajectoryOptimForTrackingInRange_Dynamics_Sim;

% Analytic derivative files (optional), provide in the format of function handle
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% (optional) customized call back function
%problem.callback=@callback_myProblem;

% Settings file
problem.settings=@settings_TrajectoryOptimForTrackingInRange;

%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max = 0;
guess.t0 = 0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=1500;     
problem.time.tf_max=1500; 
guess.tf=1500;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system.
problem.states.x0=[18 -6 0 0 80 80];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=problem.states.x0;
problem.states.x0u=problem.states.x0;

% State bounds. xl=< x <=xu
problem.states.xl=[-100 problem.states.x0(2) -5 -5 10 10]; 
problem.states.xu=[problem.states.x0(1) 100 5 5 100 100];

% State rate bounds. xrl=< x_dot <=xru
% problem.states.xrl=[x1dot_lowerbound ... xndot_lowerbound]; 
% problem.states.xru=[x1dot_upperbound ... xndot_upperbound]; 

% State error bounds
problem.states.xErrorTol_local=[0.1 0.1 0.1 0.1 0.1 0.1]; 
problem.states.xErrorTol_integral=[0.1 0.1 0.1 0.1 0.1 0.1]; 

% State constraint error bounds
problem.states.xConstraintTol=[0.1 0.1 0.5 0.5 0.1 0.1];
% problem.states.xrConstraintTol=[eps_x1dot_bounds ... eps_xndot_bounds];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[problem.states.x0(1) problem.states.x0(2) -5 -5 10 10]; 
problem.states.xfu=[problem.states.x0(1) problem.states.x0(2) 5 5 100 100];

% Guess the state trajectories with [x0 ... xf]
% guess.time=[t0 ... tf];
% guess.states(:,1)=[problem.states.x0(1) problem.states.x0(1)];
% guess.states(:,2)=[problem.states.x0(2) problem.states.x0(2)];
guess.states(:,1)=[10 10];
guess.states(:,2)=[2 2];
guess.states(:,3)=[0 0];
guess.states(:,4)=[0 0];
guess.states(:,5)=[100 50];
guess.states(:,6)=[100 50];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=[-15 -15];
problem.inputs.uu=[15 15];

% Bounds on the first control action
problem.inputs.u0l=[-15 -15];
problem.inputs.u0u=[15 15];

% Input rate bounds
problem.inputs.url=[]; 
problem.inputs.uru=[]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1, 0.1];
problem.inputs.urConstraintTol=[];

% Guess the input sequences with [u0 ... uf]
guess.inputs(:,1)=[0 0];
guess.inputs(:,2)=[0 0];

% Path constraint function 
problem.constraints.ng_eq=0; % number of quality constraints in format of g(x,u,p,t) == 0
problem.constraints.gTol_eq=[]; % equality cosntraint error bounds
% 

problem.constraints.gl=[]; % Lower ounds for inequality constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gu=[]; % Upper ounds for inequality constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gTol_neq=[]; % inequality constraint error bounds

% OPTIONAL: define the time duration each constraint will be active, for
% example (for ECH enabled in setings)
% problem.constraints.gActiveTime{1}=[guess.tf/2 guess.tf];
% problem.constraints.gActiveTime{2}=[];
% ...
% problem.constraints.gActiveTime{5}=[];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];
problem.constraints.bTol=[]; 

% store the necessary problem parameters used in the functions
problem.data.m=10;
problem.data.delta = 3.5;
problem.data.xb1 = problem.states.x0(1)-2;
problem.data.xb2 = problem.states.x0(2)+2;
% optional setting for automatic regularization
problem.data.penalty.values=[1, 10, 40];
problem.data.penalty.i=1; %starting weight

% pt = repmat([0 -5 -5 10 10 -7 -7 0],1,10);
% tt = linspace(0,3000,length(pt));

tt = linspace(0,1500,150);
pt=5.*sin(2.*pi.*tt./200)+6;

% x_t = pchip(tt,pt);
x_t = griddedInterpolant(tt,pt,'pchip','nearest');
problem.data.XT = x_t;


% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,data)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------
k_L=data.penalty.values(data.penalty.i);
k_R=1e05+(1.6e06-1e05)*(k_L-data.penalty.values(1))./(data.penalty.values(end)-data.penalty.values(1));

soft_max = @(x,y,k) log(exp(k.*x) + exp(k.*y)) ./ k;
%Define states and setpoints
x1 = x(:, 1); % Chaser position
x2 = x(:,2);
x_t = data.XT(t); % Target position

JL1=tanh(k_L.*((x1-x_t)-data.delta))+tanh(k_L.*(-(x1-x_t)-data.delta));
JL2=tanh(k_L.*((x2-x_t)-data.delta))+tanh(k_L.*(-(x2-x_t)-data.delta));
% JR1=1/k_R.*(x1-x_t).^2;
% JR2=1/k_R.*(x2-x_t).^2;
JR1=1/k_R.*soft_max(((x1-x_t).^2-data.delta.^2)./data.delta.^2,0,1);
JR2=1/k_R.*soft_max(((x2-x_t).^2-data.delta.^2)./data.delta.^2,0,1);
J1=(JL1+JR1)*10;
J2=(JL2+JR2)*10;
% stageCost = min( (J1), (J2));
stageCost = soft_max(-soft_max( -J1, -J2,1),-5,1);

% f = -soft_max( -(x1-x_t).^2, -(x2 - x_t).^2, 1) - data.delta.^2;
% %k = data.penalty.values(data.penalty.i);
% stageCost = soft_max(f,0,3);
%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------
boundaryCost=(-xf(5)-xf(6))*tf/3000;
% boundaryCost=0;
%------------- END OF CODE --------------

function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,data,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
% Leave it here
varargin=varargin{1};
%------------- BEGIN CODE --------------
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if strcmp(options.transcription,'hpLGR') && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end

%------------- END OF CODE --------------

