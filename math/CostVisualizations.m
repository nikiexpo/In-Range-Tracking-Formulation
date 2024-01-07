clear; 

%% Setting up variables
dt = 1.5;
% kL = 10;
alpha = 1;
J = @(x,kL) tanh(kL .* (x - dt)) + tanh(kL .* (-x - dt));
f = linspace(-5,5,1000); % setting this as (x - x_t) 
soft_max = @(x,y,k) log(exp(k.*x) + exp(k.*y)) .* (1/k); %soft max function
k = [1, 5,40, 100, 150];
k2 = [1, 10, 100, 1000];
xr = 1.5;

%% Tracking cost comparison
figure(1)
xlim([-5,5])

p5= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on
p6 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');

cost = f.^2; %set point
p1 = plot(f+xr,cost, LineWidth=2, Color=[0 0.4470 0.7410]);
cost = J(f, k(5)); % Lr
p2 = plot(f+xr,cost, LineWidth=2,Color=[0.4660 0.6740 0.1880]);
out = soft_max((f.^2 - dt.^2).*alpha, 0,4); % soft const
p3 = plot(f+xr,out, LineWidth=2, Color=[ 0.8500 0.3250 0.0980]); 
p4 = xline(1.5, 'r--',LineWidth=2); %ref

hold off
grid on
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Cost Value',FontSize=12,FontWeight="bold",Interpreter="latex")
xlim([-2 5])
ylim([-2.5 10])
legend([p1, p2, p3, p4, p5] ,["Setpoint cost","In-range cost", "Soft constraint cost", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)


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
legend([p1, p2, p3,pr, p4] ,["k = 1","k = 5", "k = 100", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Cost Value',FontSize=12,FontWeight="bold",Interpreter="latex")

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
legend([p1, p2, p3,pr, p4] ,["k1 = 1","k1 = 5", "k1 = 100", "Reference", "Out of range"], 'Location','northoutside', 'NumColumns', 2,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Cost Value',FontSize=12,FontWeight="bold",Interpreter="latex")

%% Regularized cost: varying K2 (K1 = 100)
figure(4)
f = linspace(-10,10, 1000);
xlim([-10,10])
p4= patch([-dt.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on 
p5 = patch([+dt.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))]+xr, [linspace(-5, f(1).^2, length(f)), fliplr(linspace(-1, f(1).^2, length(f)))], [0, 0, 0] , 'FaceAlpha', 0.15, 'EdgeColor', 'none');
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,3).*(1/k2(1));
p1 = plot(f+xr,cost, LineWidth=2,Color=[0 0.4470 0.7410]);
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,3).*(1/k2(2));
p2 = plot(f+xr,cost, LineWidth=2);
pr = xline(1.5, 'r--',LineWidth=2);
cost = J(f, k(4));
cost = cost + soft_max(f.^2-dt.^2,0,3).*(1/k2(4));
p3 = plot(f+xr,cost, LineWidth=2,Color=[0.5 0 0.5]);
hold off
grid on
xlim([-2, 5])
ylim([-2, 1])
legend([p1, p2, p3,pr, p4] ,["k2 = 1","k2 = 100", "k2 = 1000"], 'Location','northoutside', 'NumColumns', 3,  'Interpreter','latex','FontSize',11)
xlabel("Position [m]",FontSize=12,FontWeight="bold",Interpreter="latex")
ylabel('Cost Value',FontSize=12,FontWeight="bold",Interpreter="latex")

