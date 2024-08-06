% MAIN - Main script to solve the Optimal Control Problem
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk


%--------------------------------------------------------
clear all;close all;format compact;
load('SingleUAV_InRange_Profile1.mat');
Sol_setpoint=load('SingleUAV_Setpoint_Profile1.mat');
Sol_soft=load('SingleUAV_SoftConstraint_Profile1.mat');

% 
% % Obtainting the interpolated solution from polynomials
tt=transpose(linspace(solution.t0,solution.tf,1000));
x1=speval(solution,'X',1,tt);
% x2=speval(solution,'X',2,tt);
x5=speval(solution,'X',3,tt);
% x6=speval(solution,'X',6,tt);
% ...
u1=speval(solution,'U',1,tt);
% u2=speval(solution,'U',2,tt);
% ...

figure
% subplot(2,1,1)
hold on
ylim([-13 25])
y2 = problem.data.XT(tt) + problem.data.delta;
y1 = problem.data.XT(tt) - problem.data.delta;
% y2 = 5.*sin(2.*pi.*solution.T./200)+6 + problem.data.delta;
% y1 = 5.*sin(2.*pi.*solution.T./200)+6 - problem.data.delta;
p5=patch([tt' fliplr(tt')], [y1' min(ylim).*ones(size(y1'))], [.6 .6 .6], 'FaceAlpha', .3 );        % Below Lower Curve
patch([tt' fliplr(tt')], [y2' max(ylim).*ones(size(y2'))], [.6 .6 .6], 'FaceAlpha', .3)        % Above Upper Curve

p1=plot(Sol_setpoint.tt, Sol_setpoint.x1, 'LineWidth',2,'Color',[0 0.4470 0.7410]);
p2=plot(tt, x1, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
p3=plot(Sol_soft.tt, Sol_soft.x1,'--','LineWidth',2,'Color',[0.8500 0.3250 0.0980]);

% ylim([problem.states.x0(2) problem.states.x0(1)])

hold on
% plot(tt, x2, LineWidth=2)
p4=plot(tt, problem.data.XT(tt), 'k--');
% plot(solution.T, 5.*sin(2.*pi.*solution.T./200)+9 + problem.data.delta)
% plot(solution.T, 5.*sin(2.*pi.*solution.T./200)+9 - problem.data.delta)



hold off 
xlim([0 tt(end)])
legend([p1,p2,p3,p4,p5],["Set-point Tracking", "In-range Tracking (n.a.i.r.)", "In-range Tracking (a.i.r.)", "Reference", "Out-of-range Region"],'Location','northoutside','NumColumns',2,'Interpreter','latex','FontSize',11)
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Position [m]", FontSize=12,FontWeight="bold",Interpreter="latex")


% subplot(2,1,2)
% hold on
% grid on
% plot(Sol_setpoint.tt, Sol_setpoint.x5, 'LineWidth',2,'Color',[0 0.4470 0.7410])
% plot(tt, x5, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880])
% plot([tt(1); tt(end)],[10, 10],'r-' )
% xlim([0 tt(end)])
% xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
% ylabel("Energy Level [\%]", FontSize=12,FontWeight="bold",Interpreter="latex")



(Sol_soft.solution.tf-Sol_setpoint.solution.tf)/Sol_setpoint.solution.tf

solution.tf-Sol_soft.solution.tf

(solution.tf-(245.84-202.84+296.96-253.96+546-503+597-553+735-717)-Sol_setpoint.solution.tf)/Sol_setpoint.solution.tf
