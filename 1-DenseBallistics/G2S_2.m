%Version��G2S_2.m
%Description:��������ϵ������ֱ������ϵ��84����ϵ��
%�ٶ����꣬��������
function [coordS]=G2S_2(launchBLA,coordG)    
    
    BT=launchBLA(1);%����ά��
    LT=launchBLA(2);%���ľ���
    AT=launchBLA(3);%������׼��λ��
	
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
	
	coordS(1)=D_GtoS11*coordG(1)+D_GtoS12*coordG(2)+D_GtoS13*coordG(3);
	coordS(2)=D_GtoS21*coordG(1)+D_GtoS22*coordG(2)+D_GtoS23*coordG(3);
	coordS(3)=D_GtoS31*coordG(1)+D_GtoS32*coordG(2)+D_GtoS33*coordG(3);
end