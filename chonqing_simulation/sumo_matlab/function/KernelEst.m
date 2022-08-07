function [f,xi] = KernelEst(x)
%KERNELEST 此处显示有关此函数的摘要
%   概率密度估计-核密度估计
x(x==0)=[];
if ~isempty(x)
    [f,xi] = ksdensity(x);
else
    f=0;
    xi=0;
end

