%120km��ע��getSlope�õ�����degree
clear; close all;
load ('RefSpdLmt.mat');
scenarioPath='S:/Documents/graduate_paper/research/chonqing_simulation/sumo_matlab/1.sumocfg';
traci.start(['sumo-gui -c ' '"' scenarioPath '"' ' --start']);
step=0;
v_act_record=[];distance=[];remain_dis=[];roadID=[];
distance_edge=zeros(42,1);%��·����
delta_step=1;%���沽��
num=1;
% spdlmt=zeros(42,1);

edgename=string(zeros(42,1));
lanename=string(zeros(42,1));
for id=0:41
    edgename(id+1)=strcat('gneE',num2str(id));
    tmp=strcat('gneE',num2str(id));
    lanename(id+1)=strcat(tmp,'_0');
end

%��¼���ٷֲ�
VelDis=cell(40,42);
%��¼ÿһ��·�ε�ǰ����س�����ID���ٶ����ݿ⣬ÿ��·ƥ��һ��
VehName=string(zeros(42,1));
SpdRec=cell(42,1);%ÿ��ʱ�����ȡ���ٶȷֲ��������µ�300������
spltime=1;%��ǰʱ�����

while (1)
    distance=[distance;traci.vehicle.getDistance('vehicle_0')];%�ۼ����
    remain_dis=[remain_dis;traci.vehicle.getDrivingDistance2D('vehicle_0',87211.57,678.90)];%ʣ�����
    temp=traci.vehicle.getRoadID('vehicle_0');%��¼Ŀ�공��ÿ��lane�е�λ��
    temp1=isstrprop(temp,'digit');
    temp2=temp(temp1);
    
    %��ȡ��·����
    for i=1:42
        distance_edge(i)=traci.lane.getLength(char(lanename(i)));
    end
    
    %��ȡÿһ·�ε��������
%         if(step==1)
%             for i=0:41
%                 lanename=strcat('gneE',num2str(i),'_0');
%                 spdlmt(i+1)=traci.lane.getMaxSpeed(lanename);
%             end
%         end
    
    %�ռ�ÿһ��·�ĳ�������
%     for i=1:42
%         VehID_list=traci.edge.getLastStepVehicleIDs(char(edgename(i)));
%         VehID_list=string(VehID_list);
%         if ~ismember( VehName(i),VehID_list )
%             if ~isempty(VehID_list)
%                 VehName(i)=VehID_list(1);
%             else
%                 VehName(i)="0";
%             end
%         end
%         if VehName(i)~="0"
%             SpdRec{i}(end+1)=traci.vehicle.getSpeed(char(VehName(i)));%���ٶ����ݿ���ӱ�׷�ٳ������ٶ�����
%         end
%     end
    
    %ʱ������600s
    if( mod(step,600)==0 )
        for i=1:42
            %ÿ(300s)����ÿһ��·����������
            traci.edge.setMaxSpeed(char(edgename(i)),RefSpdLmt(spltime,i));
        end
    end
%     if mod(step,300)==0
%         %�ռ�ÿһ��·�ĳ��ٷֲ�
%         [VelDis{spltime,i}(2,:),VelDis{spltime,i}(1,:)]=KernelEst(SpdRec{i});%�����ܶȹ���,��һ��x,�ڶ���f
%         tmp=DataRefresh(SpdRec{i});%���������µ�300������
%         SpdRec{i}=[];
%         SpdRec{i}=tmp;
%         spltime=spltime+1;
%     end
    traci.simulation.step(step)
    step=step+delta_step;
    v_act_record=[v_act_record;traci.vehicle.getSpeed('vehicle_0')*3.6];%kph
    roadID=[roadID;str2num(temp2)];%��¼Ŀ�공���ڵ�edgeID
end
traci.close();

distance(distance<0)=[];
remain_dis(remain_dis<0)=[];
v_act=v_act_record((length(v_act_record)-length(distance))+1:end);

