%% 
clear;close all;
load ('SUMOInfo.mat');
load ratio_PM.mat;
load firstdata0531.mat;
meangrade=zeros(42,1);
regR=zeros(42,1);

%% 
% clear;close all;
% load DrivingCyle_and_TraffInfo.mat;
% load firstdata0531.mat;
% load ratio_PM.mat;
% % kp=100000;
% regR=zeros(49,1);

%% 需要的目标车和交通数据
%目标车：
%alph_cyc -- 坡度
%s -- 里程
%v_cyc -- 车速 kph
%t_cyc-- 时间
%distance_edge -- 每一segment的长度
%meangrade -- 每一segment的平均坡度
%meanspeed_record -- 每一segment的平均车速 kph
%Pdmd_distribution -- Pdmd分布
%segmentID -- road segment辨识ID（1~61）