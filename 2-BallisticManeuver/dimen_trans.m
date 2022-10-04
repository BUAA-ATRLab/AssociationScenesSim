%Date：2021.2.1
%Version：dimen_trans.m
%Description: 天文维度到地心维度
function [fys] = dimen_trans(B_F)

	B=B_F*pi/180;
	xk0=0.0; %初始化x
	xk1=B-1/298.257*sin(2*xk0);
	while abs(xk1-xk0)/abs(xk1)>10e-8
		xk0=xk1;
		xk1=B-1/298.257*sin(2*xk0);
    end
    
	fys=xk1; % 单位：弧度
end

