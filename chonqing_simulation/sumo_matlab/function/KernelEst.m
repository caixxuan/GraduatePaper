function [f,xi] = KernelEst(x)
%KERNELEST �˴���ʾ�йش˺�����ժҪ
%   �����ܶȹ���-���ܶȹ���
x(x==0)=[];
if ~isempty(x)
    [f,xi] = ksdensity(x);
else
    f=0;
    xi=0;
end

