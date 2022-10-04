clear all;
close all;
%% // 设置参数 //
para = setParameter();
[xPlane1,yPlane1,zPlane1,VxPlane1,VyPlane1,VzPlane1,~,~,~] = S_motion(para);
xPlane2 = xPlane1 - para.d_plane;
yPlane2 = yPlane1 + para.d_plane;
zPlane2 = zPlane1;
xPlane3 = xPlane1 - para.d_plane;
yPlane3 = yPlane1 - para.d_plane;
zPlane3 = zPlane1;
xPlane4 = xPlane1 - 2*para.d_plane;
yPlane4 = yPlane1 + 2*para.d_plane;
zPlane4 = zPlane1;
xPlane5 = xPlane1 - 2*para.d_plane;
yPlane5 = yPlane1 - 2*para.d_plane;
zPlane5 = zPlane1;

[xBoat1,yBoat1,zBoat1,VxBoat1,VyBoat1,VzBoat1] = CT_motion(para);
xBoat2 = xBoat1 - para.d_boat;
yBoat2 = yBoat1 + para.d_boat;
zBoat2 = zBoat1;
xBoat3 = xBoat1 - para.d_boat;
yBoat3 = yBoat1 - para.d_boat;
zBoat3 = zBoat1;
save('./data/truth.mat')

%% // 绘图 //
figure;
plot(xPlane1/1e3,yPlane1/1e3,'r',xBoat1/1e3,yBoat1/1e3,'b','LineWidth',3);hold on;
plot(xPlane2/1e3,yPlane2/1e3,'r',xBoat2/1e3,yBoat2/1e3,'b','LineWidth',3);hold on;
plot(xPlane3/1e3,yPlane3/1e3,'r',xBoat3/1e3,yBoat3/1e3,'b','LineWidth',3);hold on;
plot(xPlane4/1e3,yPlane4/1e3,'r','LineWidth',3);hold on;
plot(xPlane5/1e3,yPlane5/1e3,'r','LineWidth',3);hold on;
xlabel('X(North)/km');ylabel('Y(East)/km');
legend('plane','boat');
axis equal;
grid on;

figure;
h1=plot3(xPlane1/1e3,yPlane1/1e3,zPlane1/1e3,'r-','LineWidth',3);hold on;
h2=plot3(xBoat1/1e3,yBoat1/1e3,zBoat1/1e3,'b','LineWidth',3);hold on;
plot3(xPlane2/1e3,yPlane2/1e3,zPlane2/1e3,'r','LineWidth',3);hold on;
plot3(xBoat2/1e3,yBoat2/1e3,zBoat2/1e3,'b','LineWidth',3);hold on;
plot3(xPlane3/1e3,yPlane3/1e3,zPlane3/1e3,'r','LineWidth',3);hold on;
plot3(xBoat3/1e3,yBoat3/1e3,zBoat3/1e3,'b','LineWidth',3);hold on;
plot3(xPlane4/1e3,yPlane4/1e3,zPlane4/1e3,'r','LineWidth',3);hold on;
plot3(xPlane5/1e3,yPlane5/1e3,zPlane5/1e3,'r','LineWidth',3);hold on;
view(124,24);
xlabel('X(North)/km');ylabel('Y(East)/km');zlabel('Z(Down)/km');
legend([h1,h2],{'plane','boat'});
grid on;
axis equal;


%% //生成量测
generateMeasurements(para);