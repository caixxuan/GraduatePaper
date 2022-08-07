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
meanv=traffspeed(tf_x:end,tf_y); % 平均车速 剩余edge*1 -- kph
meang=GradeEdge(tf_x:end);
Froad=0.8*7.67.*meanv.^2/21.15+14500*9.8*0.012.*cos(meang)+14500*9.8.*sin(meang);
distance_of_each_edge=distance_edge(tf_x:end);% -- m
distance_of_each_edge(1)=sum(distance_edge(1:tf_x))-s_act;% -- 第一段的长度修正
ave_EFF=0.7;%驱动系统平均效率
Voc=425;%平均电压
n=size(Froad,1);%剩余edge数量

%以下为附加项，对结果无影响
Qdmd_fut=sum(Froad.*1000)/ave_EFF;%预测未来还需要的总能量
Qdmd_tot=Qdmd_last+Qdmd_fut;%更正的预测的总驱动能量J
ratio_con=Qdmd_tot/127.44e6;
PMcon=interp1(ratio,PM,ratio_con,'spline');%预测的PM值，单位kW
CDRMethod=zeros(49,1);%预测方法鉴别，1表示功率分布，2表示平均功率

%计算总回收的SOC，delta_SOCreg -- const
delta_SOCreg=zeros(n,1);
for i = 1:length(Froad)
    if Froad(i) < 0
        delta_Qdmdreg=ave_EFF*Froad(i)*distance_of_each_edge(i);
        delta_SOCreg(i)=abs(delta_Qdmdreg/(Qnom*Voc));
    end
end

%每段里程矩阵delta_s,每一段的平均阻力矩阵Froad,每一段的CDR矩阵CDR -- 剩余edge*1
tmp=0;
for i=1:n
    if Froad(i)>0
        tmp=tmp+Froad(i)*distance_of_each_edge(i);
    end
end
C=((soc_end-0.3)+sum(delta_SOCreg))/tmp;%const
%每一段的SOC/delta_s斜率K -- 剩余edge*1
KK=zeros(n,1);
for i=1:n
    if Froad(i)>0
        KK(i)=C*Froad(i);
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
s_ref=linspace(a_ref(1),a_ref(end),1000);%1000*1
SOC_ref=interp1(a_ref,b_ref,s_ref);%1000*1