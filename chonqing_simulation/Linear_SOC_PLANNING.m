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
meanv=0.8*traffspeed(tf_x:end,tf_y); % ƽ�����٣�����������Сϵ���� ʣ��edge*1 -- kph
meang=GradeEdge(tf_x:end);
Froad=0.8*7.67.*meanv.^2/21.15+14500*9.8*0.012.*cos(meang)+14500*9.8.*sin(meang);
distance_of_each_edge=distance_edge(tf_x:end);% -- m
distance_of_each_edge(1)=sum(distance_edge(1:tf_x))-s_act;% -- ��һ�εĳ�������

%��ʣ��ÿ��·������CDR -- ʣ��edge*1
ave_EFF=0.8;%����ϵͳƽ��Ч��
Qdmd_fut=sum(Froad.*distance_of_each_edge)/ave_EFF;%Ԥ��δ������Ҫ��������
Qdmd_tot=Qdmd_last+Qdmd_fut;%������Ԥ�������������J
ratio_con=Qdmd_tot/127.44e6;
PMcon=interp1(ratio,PM,ratio_con,'linear');
PMcon=PMcon+5;%�ֶ�����
Pdmd1=[1:1:PMcon]'*1000;
Voc=425;Resis=0.0343;Papu1s=60e3;
sqe=-1/Voc/Qnom.*(1+Resis.*Pdmd1/Voc^2);
Pdmd2=[PMcon:1:230]'*1000;
sqh=-(Pdmd2-Papu1s)/Voc/Qnom./Pdmd2.*(1+Resis.*(Pdmd2-Papu1s)/Voc^2);
sq=[sqe;sqh];Pdis=[Pdmd1;Pdmd2]/1000;%��λkW��sq-Pdis/kWΪCDR���� //230*1
PowerPre=Froad.*meanv/3.6/ave_EFF/1000;%Ԥ��ʣ��edge�Ĺ�������kW
if PMcon<60%�ж���Ҫ���ʷֲ���Ϣ��·��
    DisturbRange=[PMcon-15:60+40]';
else
    DisturbRange=[60-20:PMcon+10]';
end

[Pdis,ind]=unique(Pdis);
sq=sq(ind);
n=length(Froad);%ʣ��edge����
CDRpos=zeros(n,1);%��ʼ��CDRֵ
CDRMethod=zeros(49,1);%Ԥ�ⷽ������1��ʾ���ʷֲ���2��ʾƽ������
for tt=1:n
    if PowerPre(tt)>min(DisturbRange) && PowerPre(tt)<max(DisturbRange)%��Ҫ���ʷֲ���·��
        CDRpos(tt)=sum( sq.*Pdmd_distribution(tf_x+tt-1,:)' )/sum( Pdmd_distribution(tf_x+tt-1,:) );
        CDRMethod(tf_x+tt-1)=1;
    else%ֻ��Ҫƽ������
        CDRpos(tt)=interp1(Pdis,sq,PowerPre(tt),'linear','extrap');
        CDRMethod(tf_x+tt-1)=2;
    end
end
CDRpos(isnan(CDRpos))=0;

%CDR����
deltaQdmd=Froad.*distance_of_each_edge/ave_EFF;
deltaSOC_origin=CDRpos.*deltaQdmd;
% deltaSOC=CDRpos.*deltaQdmd.*(1+regR(tf_x:end))+deltaQdmd.*regR(tf_x:end)*4.65e-9;
deltaSOC_estimated=sum(CDRpos.*deltaQdmd);
deltaSOC_scaled=0.6/deltaSOC_estimated.*deltaSOC_origin;%����
CDR=deltaSOC_scaled./deltaQdmd;
CDR(CDR<-4.8e-9)=-4.8e-9;
CDRoutput(49-n+1:end)=CDR;

%�����ܻ��յ�SOC��delta_SOCreg -- const
delta_SOCreg=zeros(n,1);
for i = 1:n
    if Froad(i) < 0
        delta_Qdmdreg=ave_EFF*Froad(i)*distance_of_each_edge(i);
        delta_SOCreg(i)=abs(delta_Qdmdreg*(-4.65e-9));
    end
end

%ÿ����̾���delta_s,ÿһ�ε�ƽ����������Froad,ÿһ�ε�CDR����CDR -- ʣ��edge*1
tmp=0;
for i=1:n
    if Froad(i) > 0
        tmp=tmp+Froad(i)*distance_of_each_edge(i)*CDR(i);
    end
end
C=((soc_end-0.3)+sum(delta_SOCreg))/tmp;%const
%ÿһ�ε�SOC/delta_sб��KK -- ʣ��edge*1
KK=zeros(n,1);
for i=1:n
    if Froad(i)>0
        KK(i)=C*Froad(i)*CDR(i);
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