%P0-����Э���Ӱ�첻��ֻҪ����0
%Q-��������Э���R-��������Э�����Ҫ����ȡֵ
%���ﾡ����Ϊ���������Ƚϴ���������Rֵ
% X=kalman_filter(elevation,1e-7,10e-1,334.5025,1);
% distance=linspace(0,120e3,length(elevation))';
% area(distance,X);hold on;
% plot(distance,elevation,'--','linewidth',3);
% legend('KalmanFilter','Origin');

distance=linspace(0,120e3,length(elevation))';
c = smoothdata(elevation,'gaussian',400);
area(distance/1000,c);hold on;
plot(distance/1000,elevation,'r--','linewidth',3);
legend('Smooth','Origin');
xlabel('Distance[km]');ylabel('Elevation[m]');
set(gca,'fontsize',15);

% function X = kalman_filter(data,Q,R,x0,P0)
% N = length(data);
% K = zeros(N,1);
% X = zeros(N,1);
% P = zeros(N,1);
% X(1) = x0;
% P(1) = P0;
% for i = 2:N
%     K(i) = P(i-1) / (P(i-1) + R);
%     X(i) = X(i-1) + K(i) * (data(i) - X(i-1));
%     P(i) = P(i-1) - K(i) * P(i-1) + Q;
% end
% end