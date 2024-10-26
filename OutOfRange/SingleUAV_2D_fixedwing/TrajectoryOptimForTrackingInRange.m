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
problem.time.tf_min=300;     
problem.time.tf_max=300; 
guess.tf=300;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system.
problem.states.x0=[0 0 0.01 0 203796];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[0 0 0.01 -pi 203796]; 
problem.states.x0u=[0 0 0.01  pi 203796]; 

% State bounds. xl=< x <=xu
problem.states.xl=  [-1100  -1100  0.01 -pi 0];
problem.states.xu=  [1100 1100   18  pi 203796];

% State rate bounds. xrl=< x_dot <=xru
% problem.states.xrl=[x1dot_lowerbound ... xndot_lowerbound]; 
% problem.states.xru=[x1dot_upperbound ... xndot_upperbound]; 

% State error bounds
problem.states.xErrorTol_local=[1 1  1e-1  deg2rad(5) 100]; 
problem.states.xErrorTol_integral=[1 1  1e-1  deg2rad(5) 100]; 

% State constraint error bounds
problem.states.xConstraintTol=[1 1  1e-1  deg2rad(5) 100];
% problem.states.xrConstraintTol=[eps_x1dot_bounds ... eps_xndot_bounds];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[-1100  -1100    0.01  -pi 0]; 
problem.states.xfu=[1100 1100   18   pi 203796];

% Guess the state trajectories with [x0 ... xf]
% guess.time=[t0 ... tf];
guess.states(:,1)=[0 0];
guess.states(:,2)=[0 0];
guess.states(:,3)=[12 12];
guess.states(:,4)=[deg2rad(45) deg2rad(45)];
guess.states(:,5)=[203796 0.4*203796];

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=[0 -pi/2 deg2rad(-35)]; % throttle (%), thrust orientation (theta tilde), alpha (AoA)
problem.inputs.uu=[100  pi/2  deg2rad(35)];

% Bounds on the first control action
problem.inputs.u0l=[0 -pi/2 deg2rad(-35)];
problem.inputs.u0u=[100  pi/2  deg2rad(35)]; 

% Input rate bounds
problem.inputs.url=[]; 
problem.inputs.uru=[]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1 deg2rad(5) deg2rad(3)];
problem.inputs.urConstraintTol=[];

% Guess the input sequences with [u0 ... uf]
guess.inputs(:,1)=[1 1];
guess.inputs(:,2)=[deg2rad(45) deg2rad(45) ];
guess.inputs(:,3)=[deg2rad(35) deg2rad(35) ];

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
problem.data.m=1.282;
problem.data.g = 9.81;
problem.data.delta = 2.5; % -- change the range here
problem.data.k1 = 0.8554;
problem.data.k2 = 0.3051;
problem.data.c1 = 2.8037;
problem.data.c2 = 0.3177;
problem.data.c3 =0;
problem.data.c4 = 0.0296;
problem.data.c5 = 0.0279;
problem.data.c6 = 0;
problem.data.dia = 10/12; %ft
problem.data.gammaLookup = [0 0; 30, 187/706; 45, 282/706; 65, 438/706; 75, 542/706; 100, 706/706];
problem.data.effLookup = [0.6, 0.79; 0.8, 0.85; 1, 0.88; 1.2, 0.86; 1.4, 0.77];
problem.data.maxT = 4 * 0.706 * 9.81*(10545/8498); %N
problem.data.RPM_max = 10545;

% optional setting for automatic regularization
% problem.data.penalty.values=[1, 2, 3];
% problem.data.penalty.i=1; %starting weight

% pt = repmat([0 20 20 -5 -5 20 20 0],1,10);
% tt = linspace(0,3000,length(pt));
% 
% x_t = pchip(tt,pt);
% 
% problem.data.XT = x_t;

% target trajectory -- From Bazili 
pt_x = repmat([0 17.04 66.99 146.45 250 370.59 500 1000-370.59 750 1000-146.45 1000-66.99 1000-17.04 1000],1,1);
pt_y = repmat([0 129.41 250 353.55 433.04 482.96 500 1000-482.96 1000-433.04 1000-353.55 750 1000-129.41 1000],1,1);
pt_z = repmat([100], 1, length(pt_y));
tt = linspace(0,problem.time.tf_max,length(pt_x));

x_t =  griddedInterpolant(tt,pt_x,'pchip','nearest');
y_t =  griddedInterpolant(tt,pt_y,'pchip','nearest');
z_t =  griddedInterpolant(tt,pt_z,'pchip','nearest');

problem.data.XT = x_t;
problem.data.YT = y_t;
problem.data.ZT = z_t;


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
soft_max = @(x,y,k) log(exp(k.*x) + exp(k.*y)) ./ k;
%Define states and setpoints
xpos = x(:, 1); % Chaser position
ypos = x(:, 2); 

xt = data.XT(t);
yt = data.YT(t); % Target y position

% x_t = 5.*sin(2.*pi.*t./200)+9;

% k = data.penalty.values(data.penalty.i);
% stageCost = soft_max( soft_max(fx, fy, 1/50) ,0, 1/50);
% stageCost = fx + fy;
% d = 1;
stageCost =(xpos-xt).^2 + (ypos-yt).^2;
% stageCost = max(f,0);
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

boundaryCost=-10.*xf(5);

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

