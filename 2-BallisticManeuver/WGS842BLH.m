%Version：WGS842BLH.m
%Description: 地面一点WGS84坐标系坐标转换为纬度经度高度。经纬高约定为北纬为正，南纬为负，西经为负，东经为正。
%(X,Y,Z)->(B,L,H)(单位： 度)
%地理坐标系计算机代数精密分析理论式(2.9)

function [BLH] = WGS842BLH(coordWGS84,environment)
    a = environment.a;
    e2 = environment.e_2;
    
   %调整经度至范围[-pi,pi]
    if coordWGS84(1)>=0	        
        BLH(2)=atan(coordWGS84(2)/coordWGS84(1));
    elseif coordWGS84(2)>=0	
        BLH(2)=atan(coordWGS84(2)/coordWGS84(1))+pi;
    else	                    
        BLH(2)=atan(coordWGS84(2)/coordWGS84(1))-pi;
    end
    
    %迭代求解纬度
    B_tmp=atan(coordWGS84(3)/sqrt(power(coordWGS84(1),2)+power(coordWGS84(2),2)));
    B=atan((coordWGS84(3)+a*e2*tan(B_tmp)/sqrt(1+(1-e2)*power(tan(B_tmp),2)))/sqrt(power(coordWGS84(1),2)+power(coordWGS84(2),2)));
    
    while abs(B_tmp-B)>1e-3
        B_tmp=B;
        B=atan((coordWGS84(3)+a*e2*tan(B_tmp)/sqrt(1+(1-e2)*power(tan(B_tmp),2)))/sqrt(power(coordWGS84(1),2)+power(coordWGS84(2),2)));
    end
    BLH(1)=B;
    
    N=a/sqrt(1-e2*power(sin(B),2));
    BLH(3)=(sqrt(power(coordWGS84(1),2)+power(coordWGS84(2),2))/cos(B))-N;
    
    BLH(1)=BLH(1)*180/pi;
    BLH(2)=BLH(2)*180/pi;
end    
    
    
    
    
    
    