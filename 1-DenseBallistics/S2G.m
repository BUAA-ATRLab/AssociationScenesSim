%Version��S2G.m
%Description:����ֱ������ϵ�µ�����ת��Ϊ��������ϵ�µ�����
%λ������ת��


function [coordG] =S2G(launchBLA,launchWGS84,coordS)
	
	AT=launchBLA(3);%������׼��λ��
	BT=launchBLA(1);%����ά��
	LT=launchBLA(2);%���ľ���
	
	%���launch����ϵת�������Ĵ��ֱ������ϵ��ת������(18ҳ)
	D_StoG11=-sin(LT)*sin(AT)-sin(BT)*cos(AT)*cos(LT);
	D_StoG12=cos(LT)*sin(AT)-sin(BT)*cos(AT)*sin(LT);
	D_StoG13=cos(BT)*cos(AT);
	D_StoG21=cos(BT)*cos(LT);
	D_StoG22=sin(LT)*cos(BT);
	D_StoG23=sin(BT);
	D_StoG31=-sin(LT)*cos(AT)+sin(BT)*sin(AT)*cos(LT);
	D_StoG32=cos(LT)*cos(AT)+sin(BT)*sin(AT)*sin(LT);
	D_StoG33=-sin(AT)*cos(BT);
	
	coordG(1)=D_StoG11*(coordS(1)-launchWGS84(1))+D_StoG12*(coordS(2)-launchWGS84(2))+D_StoG13*(coordS(3)-launchWGS84(3));
	coordG(2)=D_StoG21*(coordS(1)-launchWGS84(1))+D_StoG22*(coordS(2)-launchWGS84(2))+D_StoG23*(coordS(3)-launchWGS84(3));
	coordG(3)=D_StoG31*(coordS(1)-launchWGS84(1))+D_StoG32*(coordS(2)-launchWGS84(2))+D_StoG33*(coordS(3)-launchWGS84(3));
end
	
	
	
