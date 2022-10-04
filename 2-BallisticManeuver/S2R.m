%Description:WGS84坐标系下的坐标转换为雷达直角坐标系(东北天)下的坐标
%位置坐标转换

function [coordR] = S2R(coordS,settings,environment)
    %coordS: WGS坐标， 3*m
    %coordR: 东北天坐标，x-E,y-N,z-U m*3
    B_rad = settings.dLatiRadar / 180 * pi;
    L_rad = settings.dLongRadar / 180 * pi;
    H_rad = settings.dAltiRadar;
    e_2 = environment.e_2;
    a = environment.a;
    N = a / sqrt(1-e_2*(sin(B_rad)^2));
    Pos_rad = [(N + H_rad) * cos(B_rad) * cos(L_rad);
               (N + H_rad) * cos(B_rad) * sin(L_rad);
               (N * (1 - e_2) + H_rad) * sin(B_rad)];
    W = [           -sin(L_rad)              cos(L_rad)          0
         -cos(L_rad)*sin(B_rad)  -sin(L_rad)*sin(B_rad) cos(B_rad);
          cos(L_rad)*cos(B_rad)   sin(L_rad)*cos(B_rad) sin(B_rad)];  
    coordR = W * (coordS' - repmat(Pos_rad,1,size(coordS,1)));
end