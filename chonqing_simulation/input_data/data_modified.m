%% 
% clear;close all;
% load ('ego_vehicle_data.mat','v_act_record','v_act','distance','remain_dis','alph_cyc','ele','ele_endpoint','cutpoint','ele_timecut');
% n_total=length(v_act_record);
% n=length(distance);%ʵ�ʳ���
% s=distance;
% v_cyc=v_act;% -- kph
% t_cyc=[1:n]';
% start_time=n_total-n+1;%ʵ�ʳ�����ʱ��
% 
% %% ����ÿһsegment�ĳ���
% distance_edge=zeros(49,1);
% distance_edge=diff(cutpoint);
% sum(distance_edge)
% %% ����segmentID
% segmentID=zeros(n,1);
% j=1;
% for i=1:49
%     while distance(j)<cutpoint(i+1) && distance(j)>=cutpoint(i) && j<n
%         segmentID(j)=i;
%         j=j+1;
%     end
% end
% segmentID(end)=49;
% %% ����ÿһ��segment��ƽ���¶�
% meangrade=zeros(49,1);
% ele_timecut=[1;ele_timecut;t_cyc(end)];
% ele_endpoint=[0;ele_endpoint;s(end)];
% grade=diff(ele(ele_timecut))./diff(ele_endpoint);
% i=2;j=2;
% while j<=length(ele_endpoint)
%     if i>length(cutpoint)
%         break;
%     end
%     if cutpoint(i)<=ele_endpoint(j)
%         meangrade(i-1)=grade(j-1);
%         i=i+1;
%     else
%         j=j+1;
%     end
% end
% 
% %% ��¼ÿһ��roadID�Ĺ��ʵĸ����ܶȷֲ�
% load ('S:\Documents\graduate_paper\research\chonqing_simulation\sumo_matlab\outputdata\Pdmd_othervehicle.mat','Pdc','distance');
% distance=interp1([0:0.01:6150]',distance,[0:1:6150]');
% Pdmd_interp=interp1([0:0.01:6150]',Pdc,[0:1:6150]');
% Pdmd_distribution=zeros(length(cutpoint)-1,230);%�����ʾ��1~230kW
% timecut=zeros(length(cutpoint),1);
% for i=1:length(cutpoint)-1%�ҵ��ֶ�ʱ���
%     timecut(i)=min(find(distance>=cutpoint(i)/1000));
% end
% timecut(end)=length(distance);
% for i=1:length(cutpoint)-1
%     time=[timecut(i):timecut(i+1)]';
%     Pdmd_of_some_edge=Pdmd_interp(time);
%     Pdmd_of_some_edge(Pdmd_of_some_edge<=0)=[];
%     [f,x]=ksdensity(Pdmd_of_some_edge);%x�ǵȱȵ�
%     ff=interp1(x,f,[1:1:230]','pchip',0);
%     ff=ff/sum(ff);
%     Pdmd_distribution(i,:)=ff;
% end
% sum(Pdmd_distribution(18,:))
% 
% load ('traffic_meanspeed.mat','meanspeed_record');
% save('S:\Documents\graduate_paper\research\chonqing_simulation\input_data\DrivingCyle_and_TraffInfo.mat',...
%     'alph_cyc','t_cyc','v_cyc','alph_cyc','s','distance_edge','meangrade','meanspeed_record',...
%     'Pdmd_distribution','segmentID');