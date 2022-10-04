%Date£º2021.2.1
%Version£ºintegral2.m
%Description:

function [result0] = integral2(up,low)
   result0=(up-low)/90*(7*formula2(low)+32*formula2((3*low+up)/4)+...
        12*formula2((up+low)/2)+32*formula2((low+3*up)/4)+7*formula2(up));
end