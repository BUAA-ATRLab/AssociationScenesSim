%Description:WGS84坐标系下的坐标转换为雷达直角坐标系(东北天)下的坐标
%速度坐标转换

function [coordR] = S2R_2(coordS,settings)
    %coordS: WGS坐标， 3*m
    %coordR: 东北天坐标， m*3
    B_rad = settings.dLatiRadar / 180 * pi;
    L_rad = settings.dLongRadar / 180 * pi;
    W = [           -sin(L_rad)              cos(L_rad)          0
         -cos(L_rad)*sin(B_rad)  -sin(L_rad)*sin(B_rad) cos(B_rad);
          cos(L_rad)*cos(B_rad)   sin(L_rad)*cos(B_rad) sin(B_rad)];  
    coordR = W * coordS';
end