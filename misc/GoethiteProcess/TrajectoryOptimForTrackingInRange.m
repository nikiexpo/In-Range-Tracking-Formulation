function [problem,guess] = TrajectoryOptimForTrackingInRange
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
problem.time.tf_min=300/(150+49) * 60;     
problem.time.tf_max=300/(150+49) * 60; 
guess.tf=300/(150+49) * 60;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system.
problem.states.x0=[0.1964 0.0195 10^-2.1];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[0.1964 0.0195 10^-2.1];
problem.states.x0u=[0.1964 0.0195 10^-2.1];

% State bounds. xl=< x <=xu
problem.states.xl=[0.1074, 0, 10^-3.6]; 
problem.states.xu=[0.2149, 0.1, 10^-2];

% State rate bounds. xrl=< x_dot <=xru
% problem.states.xrl=[x1dot_lowerbound ... xndot_lowerbound]; 
% problem.states.xru=[x1dot_upperbound ... xndot_upperbound]; 

% State error bounds
problem.states.xErrorTol_local=[0.1 0.1 0.1]; 
problem.states.xErrorTol_integral=[0.1 0.1 0.1]; 

% State constraint error bounds
problem.states.xConstraintTol=[0.1 0.1 0.1];
% problem.states.xrConstraintTol=[eps_x1dot_bounds ... eps_xndot_bounds];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[6/55.845, 0, 10^-3.5]; 
problem.states.xfu=[8/55.845, 2/55.845, 10^-2.7];

% Guess the state trajectories with [x0 ... xf]
% guess.time=[t0 ... tf];
guess.states(:,1)=[0.1964 6/55.845];

guess.states(:,2)=[0.0195 0];

guess.states(:,3)=[10^-2.1 10^-3.5];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=[1 1];
problem.inputs.uu=[50 50];

% Bounds on the first control action
problem.inputs.u0l=[1 1];
problem.inputs.u0u=[50 50];

% Input rate bounds
problem.inputs.url=[]; 
problem.inputs.uru=[]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1 0.1];
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
problem.data.V = 300; %m3
problem.data.Fu = 49; %m3/h 
problem.data.F = 150; %m3/h 
problem.data.Cfe3 = 0.0195;
problem.data.Cfe2 = 0.1964;
problem.data.Ccu = 0.03; %g/L
problem.data.pH = 2.1;
problem.data.k1 = 1.0908e-4;
problem.data.k2 = 1.5196;
problem.data.k3 = 3.864;
problem.data.alpha = 2;
problem.data.beta = 1;
problem.data.gamma = 1;
problem.data.lambda = -0.36;
problem.data.eta = 1; %Guessed

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

%Define states and setpoints

DCfe2 = 7.5 / 55.845;
stageCost = 20.*u(:,1) + 44.*u(:,2) + 100.*(DCfe2 - x(:,1)).^2;
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


DCfe3 = 0.3 / 55.845;


boundaryCost= 100.*(DCfe3 - xf(2)).^2;

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

