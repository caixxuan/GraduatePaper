%%ֲ�뵽SIMULINK s_function�е�����SOC�滮����

%���룺
%soc_end -- ÿ�ι滮SOC�ĳ�ʼֵ 1
%GradeEdge -- ƽ���¶� edge*1 // rad
%s_act -- ��ǰ�ο�λ�� // ��λm 1
%Qdmd_last -- ��ǰλ���Ѿ����ĵ���Pdmd����
%time -- ʵ��ʱ��s clock
%edge -- ��ǰ����edge��id����1��ʼ��
%trafficspeed -- ��ͨ��ƽ��ʱ��kph  57*19 edge/������*��������/300s(��1sҲ�����˲���)
%PowerCenter -- ���ʷֲ�����kW 57*100*19 edge*���ʷֲ�����kW*��������
%PowerCounter -- ���ʷֲ������ܶ� 57*100*19
%PM -- ���PM����
%ratio -- ���PMϵ��
%Qnom -- �����������As

%�����
%s_ref -- �ο�futureλ������
%SOC_ref -- �ο�SOC���У�����Ϊs_ref; ����ΪSOC_ref��
%//s_ref��SOC_ref����SOC�ο��켣��ÿposi����ͻ�����һ��
%//�����������������ܲ�׼����Ϊego vehicle�ỻ����ʵ����ʻ��̻���ڵ�·�ܳ��ȣ�

%%������
%��ʣ��·�ε�ƽ����·����Froad -- ʣ��edge*1
tf_y=fix((time+1000)/300); %ty_yΪ��ǰʱ��timeǰ����һ����ʱ��ڵ�
tf_x=edge;%��ʼedge==1,tf_xΪ��ǰ����edge
meanv=traffspeed(tf_x:end,tf_y); % ƽ������ ʣ��edge*1 -- kph
meang=GradeEdge(tf_x:end);
Froad=0.8*7.67.*meanv.^2/21.15+14500*9.8*0.012.*cos(meang)+14500*9.8.*sin(meang);
distance_of_each_edge=distance_edge(tf_x:end);% -- m
distance_of_each_edge(1)=sum(distance_edge(1:tf_x))-s_act;% -- ��һ�εĳ�������
ave_EFF=0.7;%����ϵͳƽ��Ч��
Voc=425;%ƽ����ѹ
n=size(Froad,1);%ʣ��edge����

%����Ϊ������Խ����Ӱ��
Qdmd_fut=sum(Froad.*1000)/ave_EFF;%Ԥ��δ������Ҫ��������
Qdmd_tot=Qdmd_last+Qdmd_fut;%������Ԥ�������������J
ratio_con=Qdmd_tot/127.44e6;
PMcon=interp1(ratio,PM,ratio_con,'spline');%Ԥ���PMֵ����λkW
CDRMethod=zeros(49,1);%Ԥ�ⷽ������1��ʾ���ʷֲ���2��ʾƽ������

%�����ܻ��յ�SOC��delta_SOCreg -- const
delta_SOCreg=zeros(n,1);
for i = 1:length(Froad)
    if Froad(i) < 0
        delta_Qdmdreg=ave_EFF*Froad(i)*distance_of_each_edge(i);
        delta_SOCreg(i)=abs(delta_Qdmdreg/(Qnom*Voc));
    end
end

%ÿ����̾���delta_s,ÿһ�ε�ƽ����������Froad,ÿһ�ε�CDR����CDR -- ʣ��edge*1
tmp=0;
for i=1:n
    if Froad(i)>0
        tmp=tmp+Froad(i)*distance_of_each_edge(i);
    end
end
C=((soc_end-0.3)+sum(delta_SOCreg))/tmp;%const
%ÿһ�ε�SOC/delta_sб��K -- ʣ��edge*1
KK=zeros(n,1);
for i=1:n
    if Froad(i)>0
        KK(i)=C*Froad(i);
    else
        KK(i)=-abs(delta_SOCreg(i)/distance_of_each_edge(i));
    end
end
%ÿһ�ε�delta_SOC, -- ʣ��edge*1
delta_SOC=KK.*distance_of_each_edge;
%����point_wise�滮����
b_ref=zeros(n+1,1);a_ref=zeros(n+1,1);
b_ref(1)=soc_end;a_ref(1)=s_act;
for i=1:n
    a_ref(i+1)=a_ref(i)+distance_of_each_edge(i);
    b_ref(i+1)=b_ref(i)-delta_SOC(i);
end
s_ref=linspace(a_ref(1),a_ref(end),1000);%1000*1
SOC_ref=interp1(a_ref,b_ref,s_ref);%1000*1