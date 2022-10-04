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

%% //多弹头部分//
iSumGroup = settings.iSumGroup;

numGroup=0;             %团编号
%1.1如果弹头个数超过三则强制改为三，小于一则强制改为一
if iSumGroup>3
    iSumGroup=3;
elseif iSumGroup<1
    iSumGroup=1;
end
%1.2根据弹头个数，调用弹头航迹函数
switch iSumGroup
    case 1
        numGroup=numGroup+1;
        warheadSim1(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
        pieces3Sim(launchBLA,launchWGS84,settings);
    case 2
        numGroup=numGroup+1;
        warheadSim1(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
        pieces3Sim(launchBLA,launchWGS84,settings);
        numGroup=numGroup+1;
        warheadSim2(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
    case 3
        numGroup=numGroup+1;
        warheadSim1(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
        pieces3Sim(launchBLA,launchWGS84,settings);
        numGroup=numGroup+1;
        warheadSim2(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
        numGroup=numGroup+1;
        warheadSim2(launchBLA,launchLandLatiLong,numGroup,settings,environment);
        decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment);
end

dataUnity(settings,environment);
trackVisualize(settings);
generateMeasurements(settings);

