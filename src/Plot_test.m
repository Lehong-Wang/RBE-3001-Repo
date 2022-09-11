

% h = animatedline('MaximumNumPoints',100);
% axis([0,4*pi,-1,1])
% 
% x = linspace(0,4*pi,1000);
% y = sin(x);
% for k = 1:length(x)
%     addpoints(h,x(k),y(k));
%     drawnow
% end



% x = [1 2];
% y = [1 2];
% h = animatedline(x,y,'Color','r','LineWidth',3);


% t = 0:pi/50:10*pi;
% st = sin(t);
% ct = cos(t);
% plot3(st,ct,t)

% 
% P0 = rand(7,3) ;
% P1 = rand(7,3) ;
% X = [P0(:,1) P1(:,1)] ;
% Y = [P0(:,2) P1(:,2)] ;
% Z = [P0(:,3) P1(:,3)] ;
% X = [1,2,3]
% plot3(X',Y',Z')
% hold on
% plot3(X',Y',Z','.')

% X = [1,2,3];
% Y = [2,4,6];
% Z = [3,2,1];
% 
% plot3(X,Y,Z.*sin(X))
% hold on
% plot3(X,Y,Z.*sin(X),'.')
% hold off
% pause(1);
% 

X = [1,2,6];
Y = [2,4,6];
Z = [3,2,1];

plot3(X,Y,Z)
pause(1);
hold on
plot3(X,Y,Z.*sin(X),'.')
hold off



% clear all;clc
% x=[0:.01:16]
% y=sin(3*x)
% figure(1);hold all
% Dx=50;y1=-1.2;y2=1.2;
% for n=1:1:numel(x)
%       plot(x,y);axis([x(n) x(n+Dx) y1 y2]);drawnow
% end
% 
% i = 0;
% 
% % hold all
%     
% while i<2
%     X = [1,2,1];
%     Y = [2,4,7];
%     Z = [3,6,1];
%     disp(i);
%     i = i + 1;
% %     hold on
%     plot3(X,Y,Z)
% %     plot3(X,Y,Z,'.')
% %     hold off
% %     drawnow
%     pause(1);
% end

% hold on
% plot3(1, 1, 1)


% x=0;y=0;
% fcn=@(u,v,z)x.^2+y.^2+u.^2+v.^2-z + 0./(abs(u)<=1) +0./(abs(v)<=1);
% ax=[-2 2 -2 2 0 5];
% fimplicit3(fcn,ax)
% axis(ax); xlabel 'U', ylabel 'V', zlabel 'Z'
% pause(2);

h = 1;
j = 1;

for i = (1:10)
    X = [0,0,sin(i)+2,5];
    Y = [0,0,cos(i)+2,5];
    Z = [0,5,sin(i)+2,sin(i)+2];
    try
        delete(h);
        delete(j);
    catch exception
        disp(exception);
    end
    hold on

    h = plot3(X, Y, Z)
    j = plot3(X, Y, Z, '.')
%     hold off
%     disp(X,Y,Z)
    axis([0 10 0 10 0 10]);
%     drawnow
    pause(2);

%     if i ~= 5
%         delete(h)
%         delete(i)
%     else
%         i = 1;
%     end


end




