%Version��G2S.m
%Description:��������ϵ������ֱ������ϵ��84����ϵ��
%λ������

function [coordS]=G2S(launchBLA,launchLandLatiLong,coordG,environment)
    a=environment.a;
    f=environment.f;
    
    BT=launchBLA(1);%����ά��
    LT=launchBLA(2);%���ľ���
    AT=launchBLA(3);%������׼��λ��
    
    %������ڵ��Ĵ��ֱ������ϵ�е�����
	fys_F=dimen_trans(launchLandLatiLong(1));%launch point�ĵ���ά��
	R_F=a*(1-f)*sqrt(1/(power((sin(fys_F)),2)+power((1-f),2)*power((cos(fys_F)),2)));%launch point�ĵ��ľ���
	X0=R_F*cos(fys_F)*cos(LT);
	Y0=R_F*cos(fys_F)*sin(LT);
	Z0=R_F*sin(fys_F);
	
	%���launch����ϵת�������Ĵ��ֱ������ϵ��ת������(18ҳ)
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

    
    