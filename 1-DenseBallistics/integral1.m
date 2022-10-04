%Date：2021.2.1
%Version：integral1.m
%Description:计算I1k的积分
function [result0] = integral1(up,low)

    result0= (up-low)/90*(7*formula1(low)+32*formula1((3*low+up)/4)+...
        12*formula1((up+low)/2)+32*formula1((low+3*up)/4)+7*formula1(up));
end