%Version：G2S.m
%Description:发射坐标系到地心直角坐标系（84坐标系）
%位置坐标

function [coordS]=G2S(launchBLA,launchLandLatiLong,coordG,environment)
    a=environment.a;
    f=environment.f;
    
    BT=launchBLA(1);%天文维度
    LT=launchBLA(2);%天文经度
    AT=launchBLA(3);%天文瞄准方位角
    
    %求发射点在地心大地直角坐标系中的坐标
	fys_F=dimen_trans(launchLandLatiLong(1));%launch point的地心维度
	R_F=a*(1-f)*sqrt(1/(power((sin(fys_F)),2)+power((1-f),2)*power((cos(fys_F)),2)));%launch point的地心距离
	X0=R_F*cos(fys_F)*cos(LT);
	Y0=R_F*cos(fys_F)*sin(LT);
	Z0=R_F*sin(fys_F);
	
	%求从launch坐标系转换到地心大地直角坐标系的转换矩阵(18页)
	D_GtoS11=-sin(LT)*sin(AT)-sin(BT)*cos(AT)*cos(LT);
	D_GtoS12=cos(BT)*cos(LT);
	D_GtoS13=-sin(LT)*cos(AT)+sin(BT)*sin(AT)*cos(LT);
	D_GtoS21=cos(LT)*sin(AT)-sin(BT)*cos(AT)*sin(LT);
	D_GtoS22=sin(LT)*cos(BT);
	D_GtoS23=cos(LT)*cos(AT)+sin(BT)*sin(AT)*sin(LT);
	D_GtoS31=cos(BT)*cos(AT);
	D_GtoS32=sin(BT);
	D_GtoS33=-sin(AT)*cos(BT);
	
	coordS(1)=D_GtoS11*coordG(1)+D_GtoS12*coordG(2)+D_GtoS13*coordG(3)+X0;
	coordS(2)=D_GtoS21*coordG(1)+D_GtoS22*coordG(2)+D_GtoS23*coordG(3)+Y0;
	coordS(3)=D_GtoS31*coordG(1)+D_GtoS32*coordG(2)+D_GtoS33*coordG(3)+Z0;
end

    
    