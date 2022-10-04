%Description:WGS84����ϵ�µ�����ת��Ϊ�״�ֱ������ϵ(������)�µ�����
%�ٶ�����ת��

function [coordR] = S2R_2(coordS,settings)
    %coordS: WGS���꣬ 3*m
    %coordR: ���������꣬ m*3
    B_rad = settings.dLatiRadar / 180 * pi;
    L_rad = settings.dLongRadar / 180 * pi;
    W = [           -sin(L_rad)              cos(L_rad)          0
         -cos(L_rad)*sin(B_rad)  -sin(L_rad)*sin(B_rad) cos(B_rad);
          cos(L_rad)*cos(B_rad)   sin(L_rad)*cos(B_rad) sin(B_rad)];  
    coordR = W * coordS';
end