%Date：2021.3.1
%Version：rand_num.m
%Description:%生成最小数和最大数之间的一个随机值
function [a] = rand_num(min,max) 
    a1 = randi([min max],1,1);
    a = round(a1);
end