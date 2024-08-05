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
load('Shape1.mat');

tt=transpose(linspace(solution.t0,solution.tf,1000));
x1=speval(solution,'X',1,tt);
x2=speval(solution,'X',2,tt);
x5=speval(solution,'X',5,tt);
x6=speval(solution,'X',6,tt);
u1=speval(solution,'U',1,tt);
u2=speval(solution,'U',2,tt);

figure
subplot(1,2,1)
hold on
ylim([-18 20])
y2 = problem.data.XT(tt) + problem.data.delta;
y1 = problem.data.XT(tt) - problem.data.delta;
p4=patch([tt' fliplr(tt')], [y1' min(ylim).*ones(size(y1'))], [0.6 0.6 0.6], 'FaceAlpha', .3 );        % Below Lower Curve
patch([tt' fliplr(tt')], [y2' max(ylim).*ones(size(y2'))], [0.6 0.6 0.6], 'FaceAlpha', .3)        % Above Upper Curve

p1=plot(tt, x1, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560]);
p2=plot(tt, x2, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
p3=plot(tt, problem.data.XT(tt), 'k--');
hold off 
xlim([0 tt(end)])
legend([p1,p2,p3,p4],["Tracker 1", "Tracker 2", "Target", "Out-of-range Region"],'Location','northoutside','NumColumns',2,'Interpreter','latex','FontSize',11)
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Position [m]", FontSize=12,FontWeight="bold",Interpreter="latex")

subplot(1,2,2)
hold on
grid on
plot(tt, x5, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
plot(tt, x6, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
plot([tt(1); tt(end)],[10, 10],'r-' )
xlim([0 tt(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Energy Level [\%]", FontSize=12,FontWeight="bold",Interpreter="latex")

load('Shape1_Charging_500.mat');

tt=transpose(linspace(solution.t0,solution.tf,1000));
x1=speval(solution,'X',1,tt);
x2=speval(solution,'X',2,tt);
x5=speval(solution,'X',5,tt);
x6=speval(solution,'X',6,tt);
u1=speval(solution,'U',1,tt);
u2=speval(solution,'U',2,tt);

figure
subplot(1,2,1)
hold on
ylim([-18 20])
y2 = problem.data.XT(tt) + problem.data.delta;
y1 = problem.data.XT(tt) - problem.data.delta;
p4=patch([tt' fliplr(tt')], [y1' min(ylim).*ones(size(y1'))], [0.6 0.6 0.6], 'FaceAlpha', .3 );        % Below Lower Curve
patch([tt' fliplr(tt')], [y2' max(ylim).*ones(size(y2'))], [0.6 0.6 0.6], 'FaceAlpha', .3)        % Above Upper Curve

p1=plot(tt, x1, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560]);
p2=plot(tt, x2, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
p3=plot(tt, problem.data.XT(tt), 'k--');
hold off 
xlim([0 tt(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Position [m]", FontSize=12,FontWeight="bold",Interpreter="latex")

subplot(1,2,2)
hold on
grid on
plot(tt, x5, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
plot(tt, x6, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
plot([tt(1); tt(end)],[10, 10],'r-' )
xlim([0 tt(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Energy Level [\%]", FontSize=12,FontWeight="bold",Interpreter="latex")

load('Shape1_Charging_1500.mat');

tt=transpose(linspace(solution.t0,solution.tf,1000));
x1=speval(solution,'X',1,tt);
x2=speval(solution,'X',2,tt);
x5=speval(solution,'X',5,tt);
x6=speval(solution,'X',6,tt);
u1=speval(solution,'U',1,tt);
u2=speval(solution,'U',2,tt);

figure
subplot(1,2,1)
hold on
ylim([-18 20])
y2 = problem.data.XT(tt) + problem.data.delta;
y1 = problem.data.XT(tt) - problem.data.delta;
p4=patch([tt' fliplr(tt')], [y1' min(ylim).*ones(size(y1'))], [0.6 0.6 0.6], 'FaceAlpha', .3 );        % Below Lower Curve
patch([tt' fliplr(tt')], [y2' max(ylim).*ones(size(y2'))], [0.6 0.6 0.6], 'FaceAlpha', .3)        % Above Upper Curve

p1=plot(tt, x1, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560]);
p2=plot(tt, x2, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
p3=plot(tt, problem.data.XT(tt), 'k--');
hold off 
xlim([0 tt(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Position [m]", FontSize=12,FontWeight="bold",Interpreter="latex")

subplot(1,2,2)
hold on
grid on
plot(tt, x5, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
plot(tt, x6, 'LineWidth',2,'Color',[0.4660 0.6740 0.1880]);
plot([tt(1); tt(end)],[10, 10],'r-' )
xlim([0 tt(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Energy Level [\%]", FontSize=12,FontWeight="bold",Interpreter="latex")


load('SingleUAVSetpoint.mat');

tt=transpose(linspace(solution.t0,solution.tf,1000));
tt1=transpose(linspace(solution.t0,500,1000));
x1=speval(solution,'X',1,tt);
x5=speval(solution,'X',3,tt);
u1=speval(solution,'U',1,tt);

figure
subplot(1,2,1)
hold on
ylim([-18 20])
y2 = problem.data.XT(tt1) + problem.data.delta;
y1 = problem.data.XT(tt1) - problem.data.delta;
p4=patch([tt1' fliplr(tt1')], [y1' min(ylim).*ones(size(y1'))], [0.6 0.6 0.6], 'FaceAlpha', .3 );        % Below Lower Curve
patch([tt1' fliplr(tt1')], [y2' max(ylim).*ones(size(y2'))], [0.6 0.6 0.6], 'FaceAlpha', .3)        % Above Upper Curve

p1=plot(tt, x1, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560]);
p3=plot(tt1, problem.data.XT(tt1), 'k--');
hold off 
xlim([0 tt1(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Position [m]", FontSize=12,FontWeight="bold",Interpreter="latex")

subplot(1,2,2)
hold on
grid on
plot(tt, x5, 'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
plot([tt(1); tt1(end)],[10, 10],'r-' )
xlim([0 tt1(end)])
xlabel("Time [s]", FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel("Energy Level [\%]", FontSize=12,FontWeight="bold",Interpreter="latex")


