%Date：2021.2.1
%Version：formula1.m
%Description:积分函数定义1，对sin(x)求积分。

function [f_t1] = formula1(t)

    theta_k=38.3333*pi/180;
    f_t1=sin(4*(pi/2-theta_k)*power((t-0.45),2.0)+theta_k);
end