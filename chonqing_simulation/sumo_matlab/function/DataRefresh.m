function x_new = DataRefresh(x)
%DATAREFRESH �˴���ʾ�йش˺�����ժҪ
%   �������£��ң���300������
x_new=x(max(end-300+1,1):end);
end

