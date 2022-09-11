% 
% 
% 
% 
% x2 = [1 2];
% y2 = [4 4];
% p = plot(x2,y2);
% xlim([0 120])
% ylim([2.5 4])
% xlabel('Iteration')
% ylabel('Approximation for \pi')
% 
% p.XDataSource = 'x2';
% p.YDataSource = 'y2';
% 
% denom = 1;
% k = -1;
% for t = 3:100
%     denom = denom + 2;
%     x2(t) = t;
%     y2(t) = 4*(y2(t-1)/4 + k/denom);
%     refreshdata
%     drawnow
%     k = -k;
% end


X = [1 2 3 4];
Y = [11 5 6 9];
Z = [6 2 9 5];

p3 = plot3(X,Y,Z);
grid on
p3.XDataSource = 'X';
p3.YDataSource = 'Y';
p3.ZDataSource = 'Z';


for i = 1:5
    Z(3) = i;
    X(2) = 2*i;
    refreshdata
    drawnow
    pause(1);
end







