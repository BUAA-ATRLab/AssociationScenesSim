%Description:���ɵ��������켣�����Ը������ð����൯ͷ�����ն������ն�����Ƭ��

clear ;
close all;
%% //�����������//
settings=setParameter();
environment=setEnvironment();

%% //���ɻ�������// 
[launchBLH,landBLH,launchWGS84,landWGS84]=unifiedLaunchLandCoord(settings,environment);
%����㾭γ�ȣ�û���������ĸ߶ȣ���λ����
launchLandLatiLong= [launchBLH(1),launchBLH(2),landBLH(1),landBLH(2)]; 
%���������γ��BT,���ľ���LamT��������׼��λ��AT
launchBLA=calculateBLA(launchBLH,landBLH,environment);

%% //�켣����//
numGroup = 1;
warheadSim(launchBLA,launchLandLatiLong,numGroup,settings,environment);
warheadSimManeuver(launchBLA,launchLandLatiLong,numGroup,settings,environment);

%% // ���ݴ洢����ӻ� //
dataUnity(settings,environment);
trackVisualize(settings);
generateMeasurements(settings);