%Version��B2G.m
%Description:��������ϵת��Ϊ��������ϵ
%λ������ת��
function [coordG] = B2G(coord_d0,angle_d0,coordB)
    %coord_d0��missile body����ϵԭ����launch����ϵ�µ�����
    %angle_d0�ǲο�missile body�ĸ����ǡ�ƫ���ǡ�������
    %coordB��Ŀ����missile body����ϵ�µ�����
    %coordG��Ŀ����launch����ϵ�µ�����

	%���missile body����ϵת����launch����ϵ��ת������(15ҳ)
	

	D_DTtoFS11=cos(angle_d0(1))*cos(angle_d0(2));
	D_DTtoFS12=-sin(angle_d0(1))*cos(angle_d0(3))+cos(angle_d0(1))*sin(angle_d0(2))*sin(angle_d0(3));
	D_DTtoFS13=sin(angle_d0(1))*sin(angle_d0(3))+cos(angle_d0(1))*sin(angle_d0(2))*cos(angle_d0(3));
	D_DTtoFS21=sin(angle_d0(1))*cos(angle_d0(2));
	D_DTtoFS22=cos(angle_d0(1))*cos(angle_d0(3))+sin(angle_d0(1))*sin(angle_d0(2))*sin(angle_d0(3));
	D_DTtoFS23=-cos(angle_d0(1))*sin(angle_d0(3))+sin(angle_d0(1))*sin(angle_d0(2))*cos(angle_d0(3));
	D_DTtoFS31=-sin(angle_d0(2));
	D_DTtoFS32=cos(angle_d0(2))*sin(angle_d0(3));
	D_DTtoFS33=cos(angle_d0(2))*cos(angle_d0(3));
	
	coordG(1)=D_DTtoFS11*coordB(1)+D_DTtoFS12*coordB(2)+D_DTtoFS13*coordB(3)+coord_d0(1);
	coordG(2)=D_DTtoFS21*coordB(1)+D_DTtoFS22*coordB(2)+D_DTtoFS23*coordB(3)+coord_d0(2);
	coordG(3)=D_DTtoFS31*coordB(1)+D_DTtoFS32*coordB(2)+D_DTtoFS33*coordB(3)+coord_d0(3);
end