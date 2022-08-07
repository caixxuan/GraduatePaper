%%植入到SIMULINK s_function中的在线SOC规划程序

%输入：
%soc_end -- 每次规划SOC的初始值 1
%GradeEdge -- 平均坡度 edge*1 // rad
%s_act -- 当前参考位置 // 单位m 1
%Qdmd_last -- 当前位置已经消耗掉的Pdmd积分
%time -- 实际时间s clock
%edge -- 当前所处edge的id（从1开始）
%trafficspeed -- 交通流平均时速kph  57*19 edge/公里数*采样次数/300s(第1s也进行了采样)
%PowerCenter -- 功率分布中心kW 57*100*19 edge*功率分布中心kW*采样次数
%PowerCounter -- 功率分布概率密度 57*100*19
%PM -- 拟合PM序列
%ratio -- 拟合PM系列
%Qnom -- 电池名义容量As

%输出：
%s_ref -- 参考future位置序列
%SOC_ref -- 参考SOC序列，横轴为s_ref; 纵轴为SOC_ref：
%//s_ref和SOC_ref构成SOC参考轨迹，每posi发生突变更新一次
%//（如果用里程数，可能不准，因为ego vehicle会换道，实际行驶里程会大于道路总长度）

%%主程序：
%求剩余路段的平均道路阻力Froad -- 剩余edge*1
tf_y=fix((time+1000)/300); %ty_y为当前时间time前的上一采样时间节点
tf_x=edge;%初始edge==1,tf_x为当前所在edge
meanv=0.8*traffspeed(tf_x:end,tf_y); % 平均车速（×公交车缩小系数） 剩余edge*1 -- kph
meang=GradeEdge(tf_x:end);
Froad=0.8*7.67.*meanv.^2/21.15+14500*9.8*0.012.*cos(meang)+14500*9.8.*sin(meang);
distance_of_each_edge=distance_edge(tf_x:end);% -- m
distance_of_each_edge(1)=sum(distance_edge(1:tf_x))-s_act;% -- 第一段的长度修正

%求剩余每段路期望的CDR -- 剩余edge*1
ave_EFF=0.8;%驱动系统平均效率
Qdmd_fut=sum(Froad.*distance_of_each_edge)/ave_EFF;%预测未来还需要的总能量
Qdmd_tot=Qdmd_last+Qdmd_fut;%更正的预测的总驱动能量J
ratio_con=Qdmd_tot/127.44e6;
PMcon=interp1(ratio,PM,ratio_con,'linear');
PMcon=PMcon+5;%手动修正
Pdmd1=[1:1:PMcon]'*1000;
Voc=425;Resis=0.0343;Papu1s=60e3;
sqe=-1/Voc/Qnom.*(1+Resis.*Pdmd1/Voc^2);
Pdmd2=[PMcon:1:230]'*1000;
sqh=-(Pdmd2-Papu1s)/Voc/Qnom./Pdmd2.*(1+Resis.*(Pdmd2-Papu1s)/Voc^2);
sq=[sqe;sqh];Pdis=[Pdmd1;Pdmd2]/1000;%单位kW，sq-Pdis/kW为CDR曲线 //230*1
PowerPre=Froad.*meanv/3.6/ave_EFF/1000;%预测剩余edge的功率需求kW
if PMcon<60%判断需要功率分布信息的路段
    DisturbRange=[PMcon-15:60+40]';
else
    DisturbRange=[60-20:PMcon+10]';
end

[Pdis,ind]=unique(Pdis);
sq=sq(ind);
n=length(Froad);%剩余edge数量
CDRpos=zeros(n,1);%初始化CDR值
CDRMethod=zeros(49,1);%预测方法鉴别，1表示功率分布，2表示平均功率
for tt=1:n
    if PowerPre(tt)>min(DisturbRange) && PowerPre(tt)<max(DisturbRange)%需要功率分布的路段
        CDRpos(tt)=sum( sq.*Pdmd_distribution(tf_x+tt-1,:)' )/sum( Pdmd_distribution(tf_x+tt-1,:) );
        CDRMethod(tf_x+tt-1)=1;
    else%只需要平均功率
        CDRpos(tt)=interp1(Pdis,sq,PowerPre(tt),'linear','extrap');
        CDRMethod(tf_x+tt-1)=2;
    end
end
CDRpos(isnan(CDRpos))=0;

%CDR修正
deltaQdmd=Froad.*distance_of_each_edge/ave_EFF;
deltaSOC_origin=CDRpos.*deltaQdmd;
% deltaSOC=CDRpos.*deltaQdmd.*(1+regR(tf_x:end))+deltaQdmd.*regR(tf_x:end)*4.65e-9;
deltaSOC_estimated=sum(CDRpos.*deltaQdmd);
deltaSOC_scaled=0.6/deltaSOC_estimated.*deltaSOC_origin;%放缩
CDR=deltaSOC_scaled./deltaQdmd;
CDR(CDR<-4.8e-9)=-4.8e-9;
CDRoutput(49-n+1:end)=CDR;

%计算总回收的SOC，delta_SOCreg -- const
delta_SOCreg=zeros(n,1);
for i = 1:n
    if Froad(i) < 0
        delta_Qdmdreg=ave_EFF*Froad(i)*distance_of_each_edge(i);
        delta_SOCreg(i)=abs(delta_Qdmdreg*(-4.65e-9));
    end
end

%每段里程矩阵delta_s,每一段的平均阻力矩阵Froad,每一段的CDR矩阵CDR -- 剩余edge*1
tmp=0;
for i=1:n
    if Froad(i) > 0
        tmp=tmp+Froad(i)*distance_of_each_edge(i)*CDR(i);
    end
end
C=((soc_end-0.3)+sum(delta_SOCreg))/tmp;%const
%每一段的SOC/delta_s斜率KK -- 剩余edge*1
KK=zeros(n,1);
for i=1:n
    if Froad(i)>0
        KK(i)=C*Froad(i)*CDR(i);
    else
        KK(i)=-abs(delta_SOCreg(i)/distance_of_each_edge(i));
    end
end
%每一段的delta_SOC, -- 剩余edge*1
delta_SOC=KK.*distance_of_each_edge;
%绘制point_wise规划曲线
b_ref=zeros(n+1,1);a_ref=zeros(n+1,1);
b_ref(1)=soc_end;a_ref(1)=s_act;
for i=1:n
    a_ref(i+1)=a_ref(i)+distance_of_each_edge(i);
    b_ref(i+1)=b_ref(i)-delta_SOC(i);
end
% s_ref=linspace(a_ref(1),a_ref(end),1000);%1000*1
% SOC_ref=interp1(a_ref,b_ref,s_ref,'linear');%1000*1

s_ref=linspace(s_act,a_ref(end),1000);
SOC_ref=0.3+(soc_end-0.3)/(s_act-a_ref(end))*(s_ref-a_ref(end));

% function CDR=PowerDis(center,counter,sq,Pdis)%edgeID*100*time
% pdf_Pdis=interp1(center,counter,Pdis,'pchip');
% indpo=find(Pdis>max(center));
% pdf_Pdis(indpo)=0;
% indne=find(Pdis<min(center));
% pdf_Pdis(indne)=0;
% CDR=sum(pdf_Pdis.*sq);% -- const
% end