%Date：2021.2.1
%Version：formula2.m
%Description:积分函数定义2,对cos(x)求积分。

function [f_t1] = formula2(t)

    theta_k=38.3333*pi/180;
    f_t1=cos(4*(pi/2-theta_k)*power((t-0.45),2.0)+theta_k);
end
