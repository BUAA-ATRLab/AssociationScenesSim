%Description:生成弹道导弹轨迹，可以根据设置包含多弹头、轻诱饵、重诱饵、碎片等

clear ;
close all;
%% //界面参数输入//
settings=setParameter();
environment=setEnvironment();

%% //生成基本参数// 
[launchBLH,landBLH,launchWGS84,landWGS84]=unifiedLaunchLandCoord(settings,environment);
%起落点经纬度，没包含起落点的高度，单位：度
launchLandLatiLong= [launchBLH(1),launchBLH(2),landBLH(1),landBLH(2)]; 
%发射点天文纬度BT,天文经度LamT和天文瞄准方位角AT
launchBLA=calculateBLA(launchBLH,landBLH,environment);

%% //轨迹仿真//
numGroup = 1;
warheadSim(launchBLA,launchLandLatiLong,numGroup,settings,environment);
warheadSimManeuver(launchBLA,launchLandLatiLong,numGroup,settings,environment);

%% // 数据存储与可视化 //
dataUnity(settings,environment);
trackVisualize(settings);
generateMeasurements(settings);