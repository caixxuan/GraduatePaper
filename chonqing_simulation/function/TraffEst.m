function [meanv_pre,Pdmd_distribution]=TraffEst(trafficspeed,RefSpd,VelDis,tf_x,tf_y,distance_edge,meang)
%TRAFFEST �˴���ʾ�йش˺�����ժҪ
%   ͨ��������ϢԤ��δ����ͨ����
%ע��RefSpdLmt�Ͳο����ٷֲ���time stamp��600s�������Ϊ300s

%% ����Ԥ��
RefSpd=RefSpd';
tf_x
tf_y
size(trafficspeed)
trafficspeed(tf_x:end,tf_y)*1;
meanv_tf=trafficspeed(tf_x:end,tf_y)*1;%�ɼ���ʣ���·�ľ��� * ϵ��
dist_remain=distance_edge(tf_x:end);%��ʣ������edge�յ�ľ���
time_pre=dist_remain./meanv_tf;%��ʣ��edge�յ��Ԥ��ʱ��
time_remain=cumsum(time_pre);
meanv_ref=zeros(length(meanv_tf),1);
for i=1:length(meanv_tf)
    meanv_ref(i)=RefSpd(tf_x+i-1,min(ceil(tf_y/(600/300))+fix(time_remain(i)/600),size(RefSpd,2)));%�ο�����
end
meanv_pre=zeros(length(meanv_tf),1);
w_tf=0.5;%Ȩ��
w_pre=1-w_tf;
meanv_pre=w_tf*meanv_ref+w_pre*meanv_ref;

%% ���ʷֲ�Ԥ�� 
% Pdmd_distribution -- n_edge �� 230
VelDis=VelDis';
VelDis_remain=cell(length(meanv_tf),1);
for i=1:length(meanv_tf)
    VelDis_remain{i,1} = VelDis{ tf_x+i-1,min( ceil(tf_y/2)+fix(time_remain(i)/600),size(VelDis,2)) };
end
Pdmd_distribution=zeros(length(meanv_tf),230);
ave_EFF=0.8;

for i=1:length(meanv_tf)
    if size(VelDis_remain{i,1},2)==100
        Froad=0.8*7.67.*(VelDis_remain{i,1}(1,:)*3.6).^2/21.15+14500*9.8*0.012*cos(meang(i))+14500*9.8*sin(meang(i));
        Pdis100=Froad.*VelDis_remain{i,1}(1,:)/ave_EFF/1000;%kW
        Pdis230=[1:1:230];
        Pdmd_distribution(i,:)=interp1(Pdis100,VelDis_remain{i}(2,:),Pdis230);
    else
        Froad=0.8*7.67*(meanv_pre(i)*3.6)^2/21.15+14500*9.8*0.012*cos(meang(i))+14500*9.8*sin(meang(i));
        Pdmd_ave=Froad*meanv_pre(i)/ave_EFF/1000;
        [f,x]=ksdensity(normrnd(Pdmd_ave,20,[1 230]));
        Pdmd_distribution(i,:)=interp1(x,f,[1:1:230]');
    end
end

end

