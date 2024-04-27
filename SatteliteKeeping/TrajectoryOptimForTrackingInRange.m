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
problem.time.tf_min=1;     
problem.time.tf_max=3000; 
guess.tf=300;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];


a0 = 42000;
e0 = 0;
i0 = 28.5;
O0 = 30;
w0 = 10;
M0 = 0;

ft = 3.5 * 10^(-7);

af = 42767.073;
ef = 1.64459 * 10^(-4);
i_f = 28.343;
Of = 29.999;
wf = 247.299;
Mf = 120.905; 

h0 = e0.*sind(w0 + O0);
k0 = e0.*cosd(w0 + O0);
p0 = tand(i0/2).*sind(O0);
q0 = tand(i0/2)*cosd(O0);
lambda0 = M0 + w0 + O0;

hf = ef.*sind(wf + Of);
kf = ef.*cosd(wf + Of);
pf = tand(i_f/2).*sind(Of);
qf = tand(i_f/2)*cosd(Of);
lambdaf = Mf + wf + Of;

init = [a0, h0, k0, p0, q0, lambda0];
fin = [af, hf, kf, pf, qf, lambdaf];
% Initial conditions for system

problem.states.x0=init;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=init;
problem.states.x0u=init;

% State bounds. xl=< x <=xu
problem.states.xl=[-inf -inf -inf -inf -inf -inf]; 
problem.states.xu=[inf inf inf inf inf inf];


% State rate bounds. xrl=< x_dot <=xru
% problem.states.xrl=[x1dot_lowerbound ... xndot_lowerbound]; 
% problem.states.xru=[x1dot_upperbound ... xndot_upperbound]; 

% State error bounds
problem.states.xErrorTol_local=[0.1 0.1 0.1 0.1 0.1 0.1]; 
problem.states.xErrorTol_integral=[0.1 0.1 0.1 0.1 0.1 0.1]; 

% State constraint error bounds
problem.states.xConstraintTol=[0.1 0.5 0.1 0.1 0.1 0.1];
% problem.states.xrConstraintTol=[eps_x1dot_bounds ... eps_xndot_bounds];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=fin ; 
problem.states.xfu=fin ;

% Guess the state trajectories with [x0 ... xf]
% guess.time=[t0 ... tf];
guess.states(:,:)=[init' fin'];


% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=[-15];
problem.inputs.uu=[15];

% Bounds on the first control action
problem.inputs.u0l=[-15];
problem.inputs.u0u=[15];

% Input rate bounds
problem.inputs.url=[]; 
problem.inputs.uru=[]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1];
problem.inputs.urConstraintTol=[];

% Guess the input sequences with [u0 ... uf]
guess.inputs(:,1)=[0 0];

% Path constraint function 
problem.constraints.ng_eq=0; % number of quality constraints in format of g(x,u,p,t) == 0
problem.constraints.gTol_eq=[]; % equality cosntraint error bounds
% 

problem.constraints.gl=[-inf]; % Lower ounds for inequality constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gu=[0]; % Upper ounds for inequality constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gTol_neq=[1]; % inequality constraint error bounds

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
problem.data.delta = 2.5;
% optional setting for automatic regularization
% problem.data.penalty.values=[weight_1, weight_2, ... weight_n];
% problem.data.penalty.i=1; %starting weight

pt = repmat([0 20 20 -5 -5 20 20 0],1,10);
tt = linspace(0,3000,length(pt));

x_t = pchip(tt,pt);

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

%Define states and setpoints
a = x(:, 1); 
h = x(:, 2);
k = x(:, 3);
p = x(:, 4);
q = x(:, 5);
lambda = x(:,6);

at = data.at;
ht = data.ht;
kt = data.kt;
pt = data.pt;
qt = data.qt;
lambdat = data.lambdat;



%x_t = 5.*sin(2.*pi.*t./50);

%stageCost = 200.*(x-x_t).^2;
stageCost = ((a - at)./at).^2 + (h-ht).^2 + (k-kt).^2 + (p-pt).^2 + (q-qt).^2 ...
            + ((lamda - lambdat)./(2.*pi)).^2;
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

boundaryCost=tf;

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

