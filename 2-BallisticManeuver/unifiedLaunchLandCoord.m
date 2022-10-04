%Date：2021.1.31
%Version：unifiedLaunchLandCoord.m
%Description:生成基本参数，起落点的坐标，统一转化为LLA和WGS84两种坐标

function [launchBLH,landBLH,launchWGS84,landWGS84] = unifiedLaunchLandCoord(settings,environment)

    iTypeChos=settings.iTypeChos;
    if iTypeChos==1
        %经纬高输入起落点坐标
        launchBLH(1)=settings.dLatiLaunch;
        launchBLH(2)=settings.dLongLaunch;
        launchBLH(3)=settings.dAltiLaunch;
        
        landBLH(1)=settings.dLatiLand;
        landBLH(2)=settings.dLongLand;
        landBLH(3)=settings.dAltiLand;
        
        launchWGS84=BLH2WGS84(launchBLH,environment);
        landWGS84=BLH2WGS84(landBLH,environment);
    else
        %雷达球坐标系输入起落点坐标
        radarBLH(1)=settings.dLatiRadar;
        radarBLH(2)=settings.dLongRadar;
        radarBLH(3)=settings.aAltiRadar;
        radarWGS84=BLH2WGS84(radarBLH,environment);
        
        %起落点雷达球坐标转换为雷达直角坐标
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
        
        %起落点雷达直角坐标转换为WGS84坐标
        %【Need to be addressed】坐标原点转换，弹道导弹弹道学
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
        
        %由WGS84坐标转换为LLA坐标
        launchBLH=WGS842BLH(launchWGS84,environment);
        landBLH=WGS842BLH(landWGS84,environment);
    end
