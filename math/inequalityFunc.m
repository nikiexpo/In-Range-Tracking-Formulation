% Analysis of the inequality constraint using Betts Minimum value operator
% function

% pt = repmat([0 20 20 -5 -5 20 20 0],1,10);
% tt = linspace(0,3000,length(pt));
% x_t = pchip(tt,pt);

x1 = linspace(-20,20);
x2 = linspace(-20,20);
% xt =  ppval(data.XT,t);
y = linspace(-1,0);
[X1,X2,Y] = ndgrid(x1,x2,y);

%method: Minimum value operator - Betts
delta = 10;
g_neq = @(x,y,z) x.^2 + (x.^2 - y.^2).*z - delta.^2;
F = g_neq(X1,X2,Y);



%method: Matlab min
g_neq2 = @(x,y) min(x.^2,y.^2) - delta.^2;
[X12, X22] = meshgrid(x1,x2);
F2 = g_neq2(X12, X22);
[U,V,W] = surfnorm(F2);
figure
quiver3(X12,X22,F2,U,V,W, 'r')
hold on
surf(X12,X22,F2)
colorbar
hold off
figure
quiver3(X12,X22,F2,U,V,W, 'r')

figure
surf(X12,X22,F2)
colorbar



%method; Soft max for min so (-max(-x,-y))
soft_min = @(x,y) -log(exp(-x.^2) + exp(-y.^2)) - delta.^2;
Fsm = soft_min(X12,X22);
figure
surf(X12,X22,Fsm)
colorbar

figure
hold on
for i = 1:100
    surf(X1(:,:,i),X2(:,:,i),Y(:,:,i),F(:,:,i)) ;
end
hold off
view(45,45)
colorbar