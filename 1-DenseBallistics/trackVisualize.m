function trackVisualize(settings)
numTrajectory=settings.iMisNum;
numSumWarhead=settings.iSumGroup;
numSumHeavyDecoy=settings.iSumHeavyDecoy;
numSumLightDecoy=settings.iSumLightDecoy;
numSumDebris=settings.iSumDebris;
numSumPieces=settings.iNumPieces3;
numSumTarget=numSumWarhead+numSumPieces+sum(numSumHeavyDecoy)+sum(numSumLightDecoy)+sum(numSumDebris);
%% //读取数据 //
filename = ['.\Final\truth',num2str(numTrajectory),'.mat'];
load(filename);

%% // 绘图 //
% 1. 经纬度
figure;
for i = 1:numSumTarget
    Bdeg = dataBLH{i}(:,1);
    Ldeg = dataBLH{i}(:,2);
    
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
    
    if typeTarget(i) == 1
        h1 = plot(Ldeg,Bdeg,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot(Ldeg,Bdeg,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot(Ldeg,Bdeg,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot(Ldeg,Bdeg,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot(Ldeg,Bdeg,'b-','Markersize',0.5); % 蓝色
    end
    hold on; 
end
Radar_Ldeg = settings.dLongRadar;
Radar_Bdeg = settings.dLatiRadar;
plot(Radar_Ldeg,Radar_Bdeg,'gs','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',10);
xlabel('经度(°)');ylabel('纬度(°)');
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;

%2. 高度
figure;
for i = 1:numSumTarget
    H = dataBLH{i}(:,3)/1e3;
    t = dataWGS84{i}(:,1);
    if typeTarget(i) == 1
        h1 = plot(t,H,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot(t,H,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot(t,H,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot(t,H,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot(t,H,'b-','Markersize',0.5); % 蓝色
    end
    hold on;
end
grid on;
title('高度随时间变化');
xlabel('时间 (s)');ylabel('高度 (km)');
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;

%3. 雷达坐标系（东北天坐标系）三维坐标
figure;
for i = 1:numSumTarget
    X = dataRadar{i}(:,1)/1e3;
    Y = dataRadar{i}(:,2)/1e3;
    Z = dataRadar{i}(:,3)/1e3;
    if typeTarget(i) == 1
        h1 = plot3(X,Y,Z,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot3(X,Y,Z,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot3(X,Y,Z,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot3(X,Y,Z,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot3(X,Y,Z,'b-','Markersize',0.5); % 蓝色
    end
    hold on;
end
grid on;
title('雷达坐标系目标轨迹');
xlabel('东 (km)'); ylabel('北 (km)'); zlabel('天 (km)');
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;

%4. 径向距离
figure;
for i = 1:numSumTarget
    X = dataRadar{i}(:,1)/1e3;
    Y = dataRadar{i}(:,2)/1e3;
    Z = dataRadar{i}(:,3)/1e3;
    Range = sqrt(X.^2+Y.^2+Z.^2);
    t = dataWGS84{i}(:,1);
    if typeTarget(i) == 1
        h1 = plot(t,Range,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot(t,Range,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot(t,Range,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot(t,Range,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot(t,Range,'b-','Markersize',0.5); % 蓝色
    end
    hold on;
end
grid on;
title('径向距离随时间变化');
xlabel('时间 (s)'); ylabel('径向距离 (km)'); 
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;

%5. 径向速度
figure;
for i = 1:numSumTarget
    X = dataRadar{i}(:,1);
    Y = dataRadar{i}(:,2);
    Z = dataRadar{i}(:,3);
    Range = sqrt(X.^2+Y.^2+Z.^2);
    vx = dataRadar{i}(:,4);
    vy = dataRadar{i}(:,5);
    vz = dataRadar{i}(:,6);
    vRange = (X.*vx+Y.*vy+Z.*vz)./Range;
    t = dataWGS84{i}(:,1);
    if typeTarget(i) == 1
        h1 = plot(t,vRange,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot(t,vRange,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot(t,vRange,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot(t,vRange,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot(t,vRange,'b-','Markersize',0.5); % 蓝色
    end
    hold on;
end
grid on;
title('径向速度随时间变化');
xlabel('时间 (s)'); ylabel('径向速度 (m/s)'); 
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;

%6. 空间速度
figure;
for i = 1:numSumTarget
    vx = dataRadar{i}(:,4);
    vy = dataRadar{i}(:,5);
    vz = dataRadar{i}(:,6);
    V = sqrt(vx.^2+vy.^2+vz.^2);
    t = dataWGS84{i}(:,1);
    if typeTarget(i) == 1
        h1 = plot(t,V,'g-','Markersize',0.5);  % 绿色
    elseif typeTarget(i) == 2
        h2 = plot(t,V,'k-','Markersize',0.5); % 黑色
    elseif typeTarget(i) == 3
        h3 = plot(t,V,'m-','Markersize',0.5);  % 红紫色
    elseif typeTarget(i) == 4
        h4 = plot(t,V,'c-','Markersize',0.5); % 蓝绿色
    else
        h5 = plot(t,V,'b-','Markersize',0.5); % 蓝色
    end
    hold on;
end
grid on;
title('空间速度随时间变化');
xlabel('时间 (s)'); ylabel('空间速度 (m/s)'); 
legend([h1 h2 h3 h4 h5],{'Warhead','HeavyDecoy','LightDecoy','Debris','Pieces'});
grid on;
