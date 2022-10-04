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

%% //�൯ͷ����//
iSumGroup = settings.iSumGroup;

numGroup=0;             %�ű��
%1.1�����ͷ������������ǿ�Ƹ�Ϊ����С��һ��ǿ�Ƹ�Ϊһ
if iSumGroup>3
    iSumGroup=3;
elseif iSumGroup<1
    iSumGroup=1;
end
%1.2���ݵ�ͷ���������õ�ͷ��������
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

