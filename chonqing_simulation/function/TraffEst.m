function [meanv_pre,Pdmd_distribution]=TraffEst(trafficspeed,RefSpd,VelDis,tf_x,tf_y,distance_edge,meang)
%TRAFFEST 此处显示有关此函数的摘要
%   通过过往信息预测未来交通数据
%注意RefSpdLmt和参考车速分布的time stamp是600s，其余均为300s

%% 车速预测
RefSpd=RefSpd';
tf_x
tf_y
size(trafficspeed)
trafficspeed(tf_x:end,tf_y)*1;
meanv_tf=trafficspeed(tf_x:end,tf_y)*1;%采集的剩余道路的均速 * 系数
dist_remain=distance_edge(tf_x:end);%到剩余所有edge终点的距离
time_pre=dist_remain./meanv_tf;%到剩余edge终点的预测时间
time_remain=cumsum(time_pre);
meanv_ref=zeros(length(meanv_tf),1);
for i=1:length(meanv_tf)
    meanv_ref(i)=RefSpd(tf_x+i-1,min(ceil(tf_y/(600/300))+fix(time_remain(i)/600),size(RefSpd,2)));%参考均速
end
meanv_pre=zeros(length(meanv_tf),1);
w_tf=0.5;%权重
w_pre=1-w_tf;
meanv_pre=w_tf*meanv_ref+w_pre*meanv_ref;

%% 功率分布预测 
% Pdmd_distribution -- n_edge × 230
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

