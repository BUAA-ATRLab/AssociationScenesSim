%Date��2021.1.31
%Version��unifiedLaunchLandCoord.m
%Description:���ɻ������������������꣬ͳһת��ΪLLA��WGS84��������

function [launchBLH,landBLH,launchWGS84,landWGS84] = unifiedLaunchLandCoord(settings,environment)

    iTypeChos=settings.iTypeChos;
    if iTypeChos==1
        %��γ���������������
        launchBLH(1)=settings.dLatiLaunch;
        launchBLH(2)=settings.dLongLaunch;
        launchBLH(3)=settings.dAltiLaunch;
        
        landBLH(1)=settings.dLatiLand;
        landBLH(2)=settings.dLongLand;
        landBLH(3)=settings.dAltiLand;
        
        launchWGS84=BLH2WGS84(launchBLH,environment);
        landWGS84=BLH2WGS84(landBLH,environment);
    else
        %�״�������ϵ�������������
        radarBLH(1)=settings.dLatiRadar;
        radarBLH(2)=settings.dLongRadar;
        radarBLH(3)=settings.aAltiRadar;
        radarWGS84=BLH2WGS84(radarBLH,environment);
        
        %������״�������ת��Ϊ�״�ֱ������
        dAzLaunch=settings.aAzLaunch;
        dRnLaunch=settings.aRnLaunch;
        dAlLaunch=settings.aAlLaunch;
        dAzLand=settings.aAzLand;
        dRnLand=settings.aRnLand;
        dAlLand=settings.aAlLand;
        
        xLaunchCartesian=dRnLaunch*cos(dAlLaunch*pi/180)*cos(dAzLaunch*pi/180);
		yLaunchCartesian=dRnLaunch*sin(dAlLaunch*pi/180);
		zLaunchCartesian=dRnLaunch*cos(dAlLaunch*pi/180)*sin(dAzLaunch*pi/180);
		xLandCartesian=dRnLand*cos(dAlLand*pi/180)*cos(dAzLand*pi/180);
		yLandCartesian=dRnLand*sin(dAlLand*pi/180);
		zLandCartesian=dRnLand*cos(dAlLand*pi/180)*sin(dAzLand*pi/180);
        
        %������״�ֱ������ת��ΪWGS84����
        %��Need to be addressed������ԭ��ת����������������ѧ
        M11=-cos(radarBLH(2)*pi/180)*sin(radarBLH(1)*pi/180);
		M12=cos(radarBLH(2)*pi/180)*cos(radarBLH(1)*pi/180);
		M13=-sin(radarBLH(2)*pi/180);
		M21=-sin(radarBLH(2)*pi/180)*sin(radarBLH(1)*pi/180);
		M22=sin(radarBLH(2)*pi/180)*cos(radarBLH(1)*pi/180);
		M23=cos(radarBLH(2)*pi/180);
		M31=cos(radarBLH(1)*pi/180);
		M32=sin(radarBLH(1)*pi/180);
		M33=0;
        
        launchWGS84(1)=M11*xLaunchCartesian+M12*yLaunchCartesian+M13*zLaunchCartesian+radarWGS84(1);
		launchWGS84(1)=M21*xLaunchCartesian+M22*yLaunchCartesian+M23*zLaunchCartesian+radarWGS84(2);
		launchWGS84(1)=M31*xLaunchCartesian+M32*yLaunchCartesian+M33*zLaunchCartesian+radarWGS84(3);
		landWGS84(1)=M11*xLandCartesian+M12*yLandCartesian+M13*zLandCartesian+radarWGS84(1);
		landWGS84(2)=M21*xLandCartesian+M22*yLandCartesian+M23*zLandCartesian+radarWGS84(2);
		landWGS84(3)=M31*xLandCartesian+M32*yLandCartesian+M33*zLandCartesian+radarWGS84(3);
        
        %��WGS84����ת��ΪLLA����
        launchBLH=WGS842BLH(launchWGS84,environment);
        landBLH=WGS842BLH(landWGS84,environment);
    end
