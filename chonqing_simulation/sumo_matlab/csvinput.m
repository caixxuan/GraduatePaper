clear;
filename='S:\Documents\graduate_paper\research\chonqing_simulation\sumo_matlab\edge_traffic.csv';
traffic=csvread(filename);
trafficspeed=traffic(:,13);
meanspeed=zeros(43, length(trafficspeed)/43);
start=1;
stop=start+42;
for j=1:length(trafficspeed)/43
    meanspeed(:,j)=trafficspeed(start:stop);
    start=stop+1;
    stop=start+42;
end
%注意meanspeed的第一列是0-300s，第二列是300-600s,...
meanspeed(meanspeed==0)=nan;%空值用左边或者右边的非空值来填充
for i=1:43
    for j=1:length(trafficspeed)/43
        if isnan(meanspeed(i,j))
            j_l=j;
            j_r=j;
            while j_l>=1 || j_r<=length(trafficspeed)/43
                j_l=j_l-1;
                j_r=j_r+1;
                if j_l>=1 && ~isnan(meanspeed(i,j_l)) 
                    meanspeed(i,j)=meanspeed(i,j_l);
                    break;
                end
                if  j_r<=length(trafficspeed)/43 && ~isnan(meanspeed(i,j_r))
                    meanspeed(i,j)=meanspeed(i,j_r);
                    break;
                end
            end
        end
    end
end
meanspeed=meanspeed(2:end,:);
save('S:\Documents\graduate_paper\research\chonqing_simulation\sumo_matlab\outputdata\traffic_meanspeed.mat','meanspeed');
%%

%%基于道路坡度进一步分段
load ('ego_vehicle_data.mat','edge_endpoint','cutpoint');
edge_endpoint=[0;edge_endpoint;cutpoint(end)];
meanspeed_matrix=zeros(length(cutpoint)-1,size(meanspeed,2));

for col=1:size(meanspeed,2)
    i=2;j=2;
    while j<=length(edge_endpoint)
        if i>length(cutpoint)
            break;
        end
        if cutpoint(i)<=edge_endpoint(j)
            meanspeed_matrix(i-1,col)=meanspeed(j-1,col);
            i=i+1;
        else
            j=j+1;
        end
    end
end
meanspeed_record=meanspeed_matrix*3.6; % -- kph
save('S:\Documents\graduate_paper\research\chonqing_simulation\sumo_matlab\outputdata\traffic_meanspeed.mat','meanspeed_record');