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

[problem,guess]=TrajectoryOptimForTrackingInRange;          % Fetch the problem definition
options= problem.settings(100);                  % Get options and solver settings (h method)
% options= problem.settings(Nps,Npd);            % Get options and solver settings (hp method)
% options= problem.settings([-1,...,1],[Npd_1,...Npd_N]);            % Get options and solver settings (hp method)
[solution,MRHistory]=solveMyProblem( problem,guess,options); % Format for NLP solver, solve and post-process the NLP


%% Optional functions

% Built-in figure generation (check settings for the type of plots to be generated)
% genSolutionPlots(options, solution); 
% 
% % Simulate the obtained solution (open-loop) with ODE integration 
% % [ tv, xv, uv ] = simulateSolution( problem, solution);
% % [ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113');
% [ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113', 'ode113', 0.01 );
% 
% % Obtainting the interpolated solution from polynomials
% tt=linspace(solution.t0,solution.tf,1000);
% x1=speval(solution,'X',1,tt);
% x2=speval(solution,'X',2,tt);
% ...
% u1=speval(solution,'U',1,tt);
% u2=speval(solution,'U',2,tt);
% ...

figure
plot(solution.T, sqrt(solution.X(:,1).^2 + solution.X(:,2).^2 + solution.X(:,3).^2), LineWidth=3)
hold on
ylim([6720, 6860])
plot(solution.T, problem.data.ref.*ones(size(solution.T)), 'r--')
x = solution.T;
y2 = (problem.data.ref+problem.data.delta).*ones(size(solution.T));
y1 = (problem.data.ref-problem.data.delta).*ones(size(solution.T));
patch([x' fliplr(x')], [y1' min(ylim).*ones(size(y1'))], 'r', 'FaceAlpha', .3 )        % Below Lower Curve
patch([x' fliplr(x')], [y2' max(ylim).*ones(size(y2'))], 'r', 'FaceAlpha', .3)        % Above Upper Curv
% plot(solution.T, (problem.data.ref+problem.data.delta).*ones(size(solution.T)), 'r--')
% plot(solution.T, (problem.data.ref-problem.data.delta).*ones(size(solution.T)), 'r--')
hold off
grid on
legend(["Tracker", "Reference", "Out of Bounds" ])
xlabel("Time $(s)$", FontSize=12, Interpreter="latex")
ylabel("Position $(km)$", FontSize=12,Interpreter="latex")

figure
plot(solution.T, [solution.U(:,1), solution.U(:,2), solution.U(:,3)], LineWidth=3)
grid on
legend(["Acceleration in X axis","Acceleration in Y axis", "Acceleration in Z axis"])
xlabel("Time $(s)$", FontSize=12, Interpreter="latex")
ylabel("Acceleration $(\frac{km}{x^2})$", FontSize=12,Interpreter="latex")

figure
plot(solution.T, [solution.X(:,1), solution.X(:,2), solution.X(:,3)], LineWidth=3)
grid on
legend(["Position in X axis", "Position in Y axis", "Position in Z axis" ])
xlabel("Time $(s)$", FontSize=12, Interpreter="latex")
ylabel("Position $(km)$", FontSize=12,Interpreter="latex")

% axis([0 solution.T(end)    -10 10])
% hold on
% plot(solution.T, 5.*sin(2.*pi.*solution.T./200), 'r--')
% % plot(solution.T, 5.*sin(2.*pi.*solution.T./200) + problem.data.delta)
% % plot(solution.T, 5.*sin(2.*pi.*solution.T./200) - problem.data.delta)
% x = solution.T;
% y2 = 5.*sin(2.*pi.*solution.T./200) + problem.data.delta;
% y1 = 5.*sin(2.*pi.*solution.T./200) - problem.data.delta;
% patch([x' fliplr(x')], [y1' min(ylim).*ones(size(y1'))], 'r', 'FaceAlpha', .3 )        % Below Lower Curve
% patch([x' fliplr(x')], [y2' max(ylim).*ones(size(y2'))], 'r', 'FaceAlpha', .3)        % Above Upper Curve
% hold off 
% legend(["Tracker", "Target" , "Out of Bounds"])
% xlabel("Time (s)", FontSize=12, Interpreter="latex")
% ylabel("Position (m)", FontSize=12,Interpreter="latex")
