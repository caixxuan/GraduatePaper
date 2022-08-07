clear;
load ('DataRecord.mat');
load ('traffic_meanspeed.mat');

v_cyc=v_act;
alph_cyc=zeros(length(v_cyc),1);
t_cyc=[1:1:length(v_cyc)]';
s=distance;

for i=1:length(roadID)
    if roadID(i)>100
        roadID(i)=roadID(i-1);
    end
end
roadID=[0;roadID];

save('S:\Documents\graduate_paper\research\chonqing_simulation\SUMOInfo.mat',...
    't_cyc','v_cyc','alph_cyc','s','meanspeed','distance_edge','VelDis','RefSpdLmt','roadID');