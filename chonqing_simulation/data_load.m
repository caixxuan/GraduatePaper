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

%% ��Ҫ��Ŀ�공�ͽ�ͨ����
%Ŀ�공��
%alph_cyc -- �¶�
%s -- ���
%v_cyc -- ���� kph
%t_cyc-- ʱ��
%distance_edge -- ÿһsegment�ĳ���
%meangrade -- ÿһsegment��ƽ���¶�
%meanspeed_record -- ÿһsegment��ƽ������ kph
%Pdmd_distribution -- Pdmd�ֲ�
%segmentID -- road segment��ʶID��1~61��