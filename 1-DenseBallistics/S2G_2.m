%Version：S2G_2.m
%Description:从地心直角坐标系到发射坐标系
%【L-2020/12/23】速度转换或角度转换
function [coordG] = S2G_2(launchBLA,coordS)
    
    BT=launchBLA(1);%天文维度
	LT=launchBLA(2);%天文经度
	AT=launchBLA(3);%天文瞄准方位角
    
	%求从launch坐标系转换到地心大地直角坐标系的转换矩阵(18页)
	D_StoG11=-sin(LT)*sin(AT)-sin(BT)*cos(AT)*cos(LT);
	D_StoG12=cos(LT)*sin(AT)-sin(BT)*cos(AT)*sin(LT);
	D_StoG13=cos(BT)*cos(AT);
	D_StoG21=cos(BT)*cos(LT);
	D_StoG22=sin(LT)*cos(BT);
	D_StoG23=sin(BT);
	D_StoG31=-sin(LT)*cos(AT)+sin(BT)*sin(AT)*cos(LT);
	D_StoG32=cos(LT)*cos(AT)+sin(BT)*sin(AT)*sin(LT);
	D_StoG33=-sin(AT)*cos(BT);
	
	coordG(1)=D_StoG11*coordS(1)+D_StoG12*coordS(2)+D_StoG13*coordS(3);
	coordG(2)=D_StoG21*coordS(1)+D_StoG22*coordS(2)+D_StoG23*coordS(3);
	coordG(3)=D_StoG31*coordS(1)+D_StoG32*coordS(2)+D_StoG33*coordS(3);
end
	
