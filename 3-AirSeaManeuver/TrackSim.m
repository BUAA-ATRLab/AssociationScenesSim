clear all;
close all;
%% // 设置参数 //
para = setParameter();
[xPlane,yPlane,zPlane,VxPlane,VyPlane,VzPlane,~,~,~] = S_motion(para);
[xBoat,yBoat,zBoat,VxBoat,VyBoat,VzBoat] = CT_motion(para);
save('./data/truth.mat')

%% // 绘图 //
figure;
plot(xPlane/1e3,yPlane/1e3,'r',xBoat/1e3,yBoat/1e3,'b','LineWidth',3);
xlabel('X(North)/km');ylabel('Y(East)/km');
legend('plane','boat');
% axis equal;
grid on;

figure;
plot3(xPlane/1e3,yPlane/1e3,zPlane/1e3,'LineWidth',3);hold on;
plot3(xBoat/1e3,yBoat/1e3,zBoat/1e3,'LineWidth',3);hold on;
view(124,24);
xlabel('X(North)/km');ylabel('Y(East)/km');zlabel('Z(Down)/km');
legend('plane','boat');
grid on;
axis equal;

%% //生成量测
generateMeasurements(para);
