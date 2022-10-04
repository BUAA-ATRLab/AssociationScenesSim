%Version：B2G.m
%Description:弹体坐标系转换为发射坐标系
%位置坐标转换
function [coordG] = B2G(coord_d0,angle_d0,coordB)
    %coord_d0是missile body坐标系原点在launch坐标系下的坐标
    %angle_d0是参考missile body的俯仰角、偏航角、滚动角
    %coordB是目标在missile body坐标系下的坐标
    %coordG是目标在launch坐标系下的坐标

	%求从missile body坐标系转换到launch坐标系的转换矩阵(15页)
	

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