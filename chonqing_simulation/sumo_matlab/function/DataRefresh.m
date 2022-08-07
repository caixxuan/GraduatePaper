function x_new = DataRefresh(x)
%DATAREFRESH 此处显示有关此函数的摘要
%   保留最新（右）的300个数据
x_new=x(max(end-300+1,1):end);
end

