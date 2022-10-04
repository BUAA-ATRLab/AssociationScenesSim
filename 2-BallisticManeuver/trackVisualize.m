function trackVisualize(settings)
close all;
numTrajectory=settings.iMisNum;
numSumTarget = 2;
dt = settings.delta_t;

%% //读取数据 //
filename = ['.\Final\truth',num2str(numTrajectory),'.mat'];
load(filename);
filename = ['.\mid\Missiles_Track\PointNumber',num2str(numTrajectory),'.txt'];
tmp = textread(filename);
ReentryTime = tmp(1)+tmp(2);

%% // 绘图 //
% 1. 经纬度
figure;
for i = 1:numSumTarget
    Bdeg = dataBLH{i}(ReentryTime:200:end,1);
    Ldeg = dataBLH{i}(ReentryTime:200:end,2);
    
    Ldeg = Ldeg + 180;
    Ldeg = Ldeg';
    Bdeg = Bdeg';
    
    Ldeg_diff = diff(Ldeg);
    [~,Ln] = find(Ldeg_diff < -160);
    if length(Ln) >= 1
        Ldeg(1,Ln(1)+1:end) = Ldeg(1,Ln(1)+1:end) +180;
    end
    
    if Ldeg(1,1) > 180
        Ldeg = Ldeg - 180;
    end
    
    if i == 1
        h1 = plot(Ldeg,Bdeg,'g+-');  % 绿色
    else
        h2 = plot(Ldeg,Bdeg,'mx--'); % 黑色
    end
    hold on; 
end
Radar_Ldeg = settings.dLongRadar;
Radar_Bdeg = settings.dLatiRadar;
plot(Radar_Ldeg,Radar_Bdeg,'gs','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',10);
xlabel('经度(°)');ylabel('纬度(°)');
legend([h1 h2],{'nonManeuver','Maneuver'});
grid on;

%2. 高度
figure;
for i = 1:numSumTarget
    H = dataBLH{i}(ReentryTime:200:end,3)/1e3;
    t = dataWGS84{i}(ReentryTime:200:end,1);
    
    if i == 1
        h1 = plot(t,H,'g+-');  % 绿色
    else
        h2 = plot(t,H,'mx--'); % 黑色
    end
    hold on;
end
grid on;
title('高度随时间变化');
xlabel('时间 (s)');ylabel('高度 (km)');
legend([h1 h2],{'nonManeuver','Maneuver'});
grid on;

%3. 雷达坐标系（东北天坐标系）三维坐标
figure;
for i = 1:numSumTarget
    X = dataRadar{i}(ReentryTime:200:end,1)/1e3;
    Y = dataRadar{i}(ReentryTime:200:end,2)/1e3;
    Z = dataRadar{i}(ReentryTime:200:end,3)/1e3;
    if i == 1
        h1 = plot3(X,Y,Z,'g-','Markersize',0.5);  % 绿色
    else
        h2 = plot3(X,Y,Z,'m-','Markersize',0.5); % 黑色
    end
    hold on;
end
grid on;
title('雷达坐标系目标轨迹');
xlabel('东 (km)'); ylabel('北 (km)'); zlabel('天 (km)');
legend([h1 h2],{'nonManeuver','Maneuver'});
grid on;

% %4. 径向距离
% figure;
% for i = 1:numSumTarget
%     X = dataRadar{i}(:,1)/1e3;
%     Y = dataRadar{i}(:,2)/1e3;
%     Z = dataRadar{i}(:,3)/1e3;
%     Range = sqrt(X.^2+Y.^2+Z.^2);
%     t = dataWGS84{i}(:,1);
%     if i == 1
%         h1 = plot(t,Range,'g+-','Markersize',0.5);  % 绿色
%     else
%         h2 = plot(t,Range,'mo-','Markersize',0.5); % 黑色
%     end
%     hold on;
% end
% grid on;
% title('径向距离随时间变化');
% xlabel('时间 (s)'); ylabel('径向距离 (km)'); 
% legend([h1 h2],{'nonManeuver','Maneuver'});
% grid on;
% 
% %5. 径向速度
% figure;
% for i = 1:numSumTarget
%     X = dataRadar{i}(:,1);
%     Y = dataRadar{i}(:,2);
%     Z = dataRadar{i}(:,3);
%     Range = sqrt(X.^2+Y.^2+Z.^2);
%     vx = dataRadar{i}(:,4);
%     vy = dataRadar{i}(:,5);
%     vz = dataRadar{i}(:,6);
%     vRange = (X.*vx+Y.*vy+Z.*vz)./Range;
%     t = dataWGS84{i}(:,1);
%     if i == 1
%         h1 = plot(t,vRange,'g+-','Markersize',0.5);  % 绿色
%     else
%         h2 = plot(t,vRange,'mo-','Markersize',0.5); % 黑色
%     end
%     hold on;
% end
% grid on;
% title('径向速度随时间变化');
% xlabel('时间 (s)'); ylabel('径向速度 (m/s)'); 
% legend([h1 h2],{'nonManeuver','Maneuver'});
% grid on;
% 
% %6. 空间速度
% figure;
% for i = 1:numSumTarget
%     vx = dataRadar{i}(:,4);
%     vy = dataRadar{i}(:,5);
%     vz = dataRadar{i}(:,6);
%     V = sqrt(vx.^2+vy.^2+vz.^2);
%     t = dataWGS84{i}(:,1);
%     if i == 1
%         h1 = plot(t,V,'g+','Markersize',0.5);  % 绿色
%     else
%         h2 = plot(t,V,'mo','Markersize',0.5); % 黑色
%     end
%     hold on;
% end
% grid on;
% title('空间速度随时间变化');
% xlabel('时间 (s)'); ylabel('空间速度 (m/s)'); 
% legend([h1 h2],{'nonManeuver','Maneuver'});
% grid on;

%7. 雷达坐标系加速度
figure;
ax = cell(1,2);ay=cell(1,2);az=cell(1,2);
for i = 1:numSumTarget
    vx = dataRadar{i}(ReentryTime:end,4);ax{i}=diff(vx)/dt;
    vy = dataRadar{i}(ReentryTime:end,5);ay{i}=diff(vy)/dt;
    vz = dataRadar{i}(ReentryTime:end,6);az{i}=diff(vz)/dt;
    t = dataWGS84{i}(ReentryTime+1:end,1);
    if i == 1
        h1 = plot(t(1:100:end),ax{i}(1:100:end),'g+-');hold on;
        h2 = plot(t(1:100:end),ay{i}(1:100:end),'b+-');hold on;
        h3 = plot(t(1:100:end),az{i}(1:100:end),'m+-');hold on;
    else
        h4 = plot(t(1:100:end),ax{i}(1:100:end),'go-');hold on;
        h5 = plot(t(1:100:end),ay{i}(1:100:end),'bo-');hold on;
        h6 = plot(t(1:100:end),az{i}(1:100:end),'mo-');hold on;
    end
end
grid on;
title('加速度随时间变化');
xlabel('时间 (s)'); ylabel('加速度 (m/s^2)'); 
legend([h1 h2 h3 h4 h5 h6],{'nonManeuver-ax','nonManeuver-ay','nonManeuver-az','Maneuver-ax','Maneuver-ay','Maneuver-az'});
grid on;

%8. 机动加速度
figure;
t = dataWGS84{1}(ReentryTime+1:end,1);
ax_diff = ax{2}(1:length(ax{1}))-ax{1}(1:end);
ay_diff = ay{2}(1:length(ay{1}))-ay{1}(1:end);
az_diff = az{2}(1:length(az{1}))-az{1}(1:end);
plot(t(100:100:end),ax_diff(100:100:end),'gs-');hold on;
plot(t(100:100:end),ay_diff(100:100:end),'bs-');hold on;
plot(t(100:100:end),az_diff(100:100:end),'ms-');hold on;
title('机动加速度随时间变化');
xlabel('时间 (s)'); ylabel('机动加速度 (m/s^2)'); 
legend('ax-diff','ay-diff','az-diff');
grid on;



