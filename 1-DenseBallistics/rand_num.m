%Date��2021.3.1
%Version��rand_num.m
%Description:%������С���������֮���һ�����ֵ
function [a] = rand_num(min,max) 
    a1 = randi([min max],1,1);
    a = round(a1);
end