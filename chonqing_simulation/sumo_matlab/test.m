%120km，注意getSlope得到的是degree
clear; close all;
load ('RefSpdLmt.mat');
scenarioPath='S:/Documents/graduate_paper/research/chonqing_simulation/sumo_matlab/1.sumocfg';
traci.start(['sumo-gui -c ' '"' scenarioPath '"' ' --start']);
step=0;
v_act_record=[];distance=[];remain_dis=[];roadID=[];
distance_edge=zeros(42,1);%道路长度
delta_step=1;%仿真步长
num=1;
% spdlmt=zeros(42,1);

edgename=string(zeros(42,1));
lanename=string(zeros(42,1));
for id=0:41
    edgename(id+1)=strcat('gneE',num2str(id));
    tmp=strcat('gneE',num2str(id));
    lanename(id+1)=strcat(tmp,'_0');
end

%记录车速分布
VelDis=cell(40,42);
%记录每一子路段当前被监控车辆的ID和速度数据库，每条路匹配一行
VehName=string(zeros(42,1));
SpdRec=cell(42,1);%每个时间戳获取了速度分布后保留最新的300个数据
spltime=1;%当前时间戳步

while (1)
    distance=[distance;traci.vehicle.getDistance('vehicle_0')];%累计里程
    remain_dis=[remain_dis;traci.vehicle.getDrivingDistance2D('vehicle_0',87211.57,678.90)];%剩余里程
    temp=traci.vehicle.getRoadID('vehicle_0');%记录目标车在每段lane中的位置
    temp1=isstrprop(temp,'digit');
    temp2=temp(temp1);
    
    %获取道路长度
    for i=1:42
        distance_edge(i)=traci.lane.getLength(char(lanename(i)));
    end
    
    %获取每一路段的最高限速
%         if(step==1)
%             for i=0:41
%                 lanename=strcat('gneE',num2str(i),'_0');
%                 spdlmt(i+1)=traci.lane.getMaxSpeed(lanename);
%             end
%         end
    
    %收集每一段路的车速数据
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
%             SpdRec{i}(end+1)=traci.vehicle.getSpeed(char(VehName(i)));%向速度数据库添加被追踪车辆的速度数据
%         end
%     end
    
    %时间戳间距600s
    if( mod(step,600)==0 )
        for i=1:42
            %每(300s)更新每一段路的限速数据
            traci.edge.setMaxSpeed(char(edgename(i)),RefSpdLmt(spltime,i));
        end
    end
%     if mod(step,300)==0
%         %收集每一段路的车速分布
%         [VelDis{spltime,i}(2,:),VelDis{spltime,i}(1,:)]=KernelEst(SpdRec{i});%概率密度估计,第一行x,第二行f
%         tmp=DataRefresh(SpdRec{i});%仅保留最新的300个数据
%         SpdRec{i}=[];
%         SpdRec{i}=tmp;
%         spltime=spltime+1;
%     end
    traci.simulation.step(step)
    step=step+delta_step;
    v_act_record=[v_act_record;traci.vehicle.getSpeed('vehicle_0')*3.6];%kph
    roadID=[roadID;str2num(temp2)];%记录目标车所在的edgeID
end
traci.close();

distance(distance<0)=[];
remain_dis(remain_dis<0)=[];
v_act=v_act_record((length(v_act_record)-length(distance))+1:end);

