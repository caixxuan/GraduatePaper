load ('ego_vehicle_data.mat');
load ('elevation.mat');
%% 对alph_cyc进行modified
tmp=smooth(linspace(1,120e3*3500/length(elevation),3500),elevation(1:3500),1000);
elevation(1:3500)=tmp;
ele=interp1(linspace(1,120e3,length(elevation)),elevation,linspace(1,distance(end),length(distance)),'spline');
ele=ele';
ele=smooth(distance,ele,80);
alph_cyc=diff(ele)./diff(distance);
alph_cyc(alph_cyc<-0.05)=-0.05;
alph_cyc(alph_cyc>0.05)=0.05;
alph_cyc=[alph_cyc(1);alph_cyc];
alph_cyc=fillmissing(alph_cyc,'next');

%% 找到edge分割点和坡度分割点，以及最后的分割点
roadID=[0;roadID];
i=1;
while i<=length(roadID)-1
    j=i+1;
    while roadID(j)>=roadID(i)+2 && i<=length(roadID)
        roadID(j)=roadID(i);
        j=j+1;
        if roadID(j)==roadID(i)+1
            i=j;
            break;
        end
    end
    i=i+1;
end
%edge_endpoint表示道路分割点，ele_endpoint表示海拔分割点（里程）
edge_endpoint=[];
for i=1:length(roadID)-1
    if roadID(i)~=roadID(i+1)
        edge_endpoint=[edge_endpoint;distance(i)];
    end
end
ele_timecut=[1319;1688;2612;3146;3566;4506;5248];
for i=1:length(ele_timecut)
    ele_endpoint=distance(ele_timecut);
end
cutpoint=[edge_endpoint;ele_endpoint];
cutpoint=sort(cutpoint);
cutpoint=[0;cutpoint;distance(end)];

save('S:\Documents\graduate_paper\research\chonqing_simulation\sumo_matlab\outputdata\ego_vehicle_data.mat');