clear; close all

%% Setting up variables
dt = 1.5;
% kL = 10;
alpha = 1;
J = @(x,kL) tanh(kL .* (x - dt)) + tanh(kL .* (-x - dt));
f = linspace(-5,5,1000); % setting this as (x - x_t) 
f2 = linspace(-5,5,10000); % setting this as (x - x_t) 
soft_max = @(x,y,k) log(exp(k.*x) + exp(k.*y)) .* (1/k); %soft max function
k = [1, 5,40, 100, 150];
k2 = [1, 10, 100, 1000];
rho = [1,5,10]
xr = 1.5;

%% New comp
figure(15)
xlim([-5,5])

p5= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on
p6 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');

cost = f.^2; %set point
p1 = plot(f+xr,cost, LineWidth=2, Color=[0 0.4470 0.7410]);
cost = J(f2, 1e09); % Lr
p2 = plot(f2+xr,cost, LineWidth=2,Color=[0.4660 0.6740 0.1880]);
out = soft_max((f.^2 - dt.^2).*alpha, 0,50); % soft const
p3 = plot(f+xr,out, LineWidth=2, Color=[ 0.8500 0.3250 0.0980]); 
p4 = xline(1.5, 'r--',"Reference",LineWidth=2); %ref


idx1 = find(f>=0,1);
idx2 = find(f<=3,1,"last");
hard = zeros(size(f(idx1:idx2)));

p7 = plot(f(idx1:idx2), hard, LineWidth=2, Color=[0.5,0,0.5]);
p8 = plot([0;0], [0; max(ylim)], LineWidth=2, Color=[0.5,0,0.5]);
p9 = plot([3;3], [0; max(ylim)], LineWidth=2, Color=[0.5,0,0.5]);
hold off
grid on
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Lagrange Cost',FontSize=12,FontWeight="bold",Interpreter="latex")
xlim([-2 5])
ylim([-2.5 10])
legend([p1, p2, p3,p8, p4, p5] ,["Setpoint tracking","In-range tracking (n.a.i.r)", "In-range tracking (a.i.r) with soft constraints", "In-range tracking (a.i.r) with hard constraints", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)


%% Tracking cost comparison
figure(1)
xlim([-5,5])

p5= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on
p6 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');

cost = f.^2; %set point
p1 = plot(f+xr,cost, LineWidth=2, Color=[0 0.4470 0.7410]);
cost = J(f2, 1e09); % Lr
p2 = plot(f2+xr,cost, LineWidth=2,Color=[0.4660 0.6740 0.1880]);
out = soft_max((f.^2 - dt.^2).*alpha, 0,50); % soft const
p3 = plot(f+xr,out, LineWidth=2, Color=[ 0.8500 0.3250 0.0980]); 
p4 = xline(1.5, 'r--',LineWidth=2); %ref

hold off
grid on
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Lagrange Cost',FontSize=12,FontWeight="bold",Interpreter="latex")
xlim([-2 5])
ylim([-2.5 10])
legend([p1, p2, p3, p4, p5] ,["Setpoint tracking","In-range tracking (n.a.i.r)", "In-range tracking (a.i.r) with soft constraints", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)


%% Unregularized cost: varying K1
figure(2)
f = linspace(-10,10, 1000);
xlim([-10,10])
p4= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on 
p5 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');

cost = J(f, k(1));
p1 = plot(f+xr,cost, LineWidth=2,Color=[0 0.4470 0.7410]);
cost = J(f, k(2));
p2 = plot(f+xr,cost, LineWidth=2);
pr = xline(1.5, 'r--',LineWidth=2);
cost = J(f, k(4));
p3 = plot(f+xr,cost, LineWidth=2,Color=[0.5 0 0.5]);
hold off
grid on
xlim([-2, 5])
ylim([-2, 1])
legend([p1, p2, p3,pr, p4] ,["$k_1 = 1$","$k_1 = 5$", "$k_1 = 100$", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('$L_S$',FontSize=12,FontWeight="bold",Interpreter="latex")

%% Regularized cost: Varying K1 (K2 = 1000)
figure(3)
f = linspace(-10,10, 1000);
xlim([-10,10])
p4= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on 
p5 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');

cost = J(f, k(1));
cost = cost + f.^2.*(1/k2(4).^3);

p1 = plot(f+xr,cost, LineWidth=2,Color=[0 0.4470 0.7410]);
cost = J(f, k(2));
cost = cost + f.^2.*(1/k2(4).^3);
p2 = plot(f+xr,cost, LineWidth=2);
pr = xline(1.5, 'r--',LineWidth=2);
cost = J(f, k(4));
cost = cost + f.^2.*(1/k2(4));
p3 = plot(f+xr,cost, LineWidth=2,Color=[0.5 0 0.5]);
hold off
grid on
xlim([-2, 5])
ylim([-2, 1])
legend([p1, p2, p3,pr, p4] ,["$k_1 = 1$","$k_1 = 5$", "$k_1 = 100$", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('$L_S$',FontSize=12,FontWeight="bold",Interpreter="latex")

%% Regularized cost: varying K2 (K1 = 100)
figure(4)
f = linspace(-10,10, 1000);
xlim([-10,10])
p4= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on 
p5 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,10).*(1/k2(1));
p1 = plot(f+xr,cost, LineWidth=2,Color=[0 0.4470 0.7410]);
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,10).*(1/k2(2));
p2 = plot(f+xr,cost, LineWidth=2);
pr = xline(1.5, 'r--',LineWidth=2);
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,10).*(1/k2(4));
p3 = plot(f+xr,cost, LineWidth=2,Color=[0.5 0 0.5]);
hold off
grid on
xlim([-2, 5])
ylim([-2, 10])
legend([p1, p2, p3,pr, p4] ,["$k_2 = 1$","$k_2 = 100$", "$k_2 = 1000$", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('$L_S+L_R$',FontSize=12,FontWeight="bold",Interpreter="latex")

%% Regularized cost: varying K1 and K2
figure(5)
f = linspace(-10,10, 1000);
xlim([-10,10])
p4= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on 
p5 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
cost = J(f, k(1));
cost = cost + soft_max(f.^2-dt.^2,0,rho(1)).*(1/k2(1));
p1 = plot(f+xr,cost, LineWidth=2,Color=[0 0.4470 0.7410]);
cost = J(f, k(2));
cost = cost + soft_max(f.^2-dt.^2,0,rho(2)).*(1/k2(2));
p2 = plot(f+xr,cost, LineWidth=2);
pr = xline(1.5, 'r--',LineWidth=2);
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,rho(3)).*(1/k2(4));
p3 = plot(f+xr,cost, LineWidth=2,Color=[0.5 0 0.5]);
hold off
grid on
xlim([-2, 5])
ylim([-2, 10])
legend([p1, p2, p3,pr, p4] ,["$k_1 = 1, k_2 = 1, \rho=1$","$k_1 = 5, k_2 = 50, \rho=3$", "$k_1 = 20, k_2 = 100, \rho=5$", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('$L_S+L_R$',FontSize=12,FontWeight="bold",Interpreter="latex")


