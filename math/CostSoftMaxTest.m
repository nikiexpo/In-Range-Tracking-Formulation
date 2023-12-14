% Out-of range cost function for tracking in range 
soft_max = @(x,y,k) log(exp(k.*x) + exp(k.*y)); %soft max function

f = linspace(-20,20); % x - x_t 
delta = 10; % range
alpha = 1; % some scalar value should scaling be necessary in cost
k = 1; % scalar value should scaling be necessary (underfull, overfull)
out = soft_max((f.^2 - delta.^2).*alpha, 0,1);
out2 = f.^2  

[DX] = gradient(out,1);

figure
plot(f,out, LineWidth=2)
hold on
plot(f,out2,LineWidth=2,Color=[1 0 0])
% plot(-delta.*ones(size(f)), linspace(0, f(1).^2 - delta.^2, length(f)), 'r--')
% plot(delta.*ones(size(f)), linspace(0, f(1).^2 - delta.^2, length(f)), 'r--')
% patch([delta.*ones(size(f)), fliplr(-delta.*ones(size(f)))], [linspace(0, f(1).^2 - delta.^2, length(f)), fliplr(linspace(0, f(1).^2 - delta.^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.25)
patch([-delta.*ones(size(f)), fliplr(min(xlim).*ones(size(f)))], [linspace(0, f(1).^2, length(f)), fliplr(linspace(0, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.25)
patch([+delta.*ones(size(f)), fliplr(max(xlim).*ones(size(f)))], [linspace(0, f(1).^2, length(f)), fliplr(linspace(0, f(1).^2, length(f)))], [1, 0, 0] , 'FaceAlpha', 0.25)
hold off
legend(["Cost in Tracking-in-range", "Cost in Setpoint tracking", "Out-Of-Range"])
grid on
xlabel("Deviation from reference", FontWeight="bold", FontSize=14)
ylabel("Cost Value", FontWeight="bold", FontSize=14)
