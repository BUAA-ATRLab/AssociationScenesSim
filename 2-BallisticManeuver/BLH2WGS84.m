%Version：BLH2WGS84.m
%Description: 地面一点纬度经度高度转换为WGS84坐标系坐标。经纬高约定为北纬为正，南纬为负，西经为负，东经为正。
%(B,L,H)->(X,Y,Z)
%地理坐标系计算机代数精密分析理论式(2.8)
function [coordWGS84] = BLH2WGS84(BLH,environment)
    a = environment.a;
    e2 = environment.e_2;
    
    lati = BLH(1)*pi/180;
    long = BLH(2)*pi/180;
    alti = BLH(3);
    
    N = a/sqrt(1-e2*power(sin(lati),2)); %卯酉圈曲率半径
    
    %lati[-pi/2,pi/2],long[-pi,pi]
    coordWGS84(1)=(N+alti)*cos(lati)*cos(long);
    coordWGS84(2)=(N+alti)*cos(lati)*sin(long);
    coordWGS84(3)=(N*(1-e2)+alti)*sin(lati);
end
    
    
    
    