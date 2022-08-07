%% SOC轨迹
clear
load ('NPMP_test2.mat','distance','soc_ref','SoC');
soc_ref(soc_ref==0)=0.9;
plot(distance,soc_ref,'--','color','blue');hold on;
plot(distance,SoC,'linewidth',2,'color','blue');hold on;
% load ('NPMP.mat','distance','SoC','soc_ref');
% soc_ref(soc_ref==0)=0.9;
% plot(distance,soc_ref,'--','color','green');hold on;
% plot(distance,SoC,'linewidth',2,'color','green');hold on;
% load ('LINPMP.mat','distance','soc_ref','SoC');
% soc_ref(soc_ref==0)=0.9;
% plot(distance,soc_ref,'--','color','r');hold on;
% plot(distance,SoC,'linewidth',2,'color','r');hold on;
load ('OptimalPMP.mat','distance','SoC');
plot(distance,SoC,'linewidth',2.5);
legend('规划','实际','最优轨迹');
xlabel('Distance[km]');ylabel('SOC[-]');xlim([0 distance(end)]);
%% % 累积油耗
load ('Experiment1_result.mat','CumulativeFC','distance');
plot(distance,CumulativeFC,'linewidth',2);hold on;
load ('OptimalPMP.mat','CumulativeFC','distance');
plot(distance,CumulativeFC,'linewidth',2);hold on;
legend('EX1','Opt');
%%油耗
%% 
clear
load NPMP_test2.mat;
FC=CumulativeFC(end);
penalty=(0.3-SoC(end))*Qmax*Ns*Np*3.8/(1000*0.36*46.1);
totalfuel_optimal=FC+penalty
%%PM分析
clear
load NPMP_test.mat;
plot(distance,PMcon,'linewidth',3);hold on;
load OptimalPMP.mat;
analyPM;plot(distance,PMreal,'linewidth',3);
legend('pre','opt');
%%交通车速
clear;
load ('DrivingCyle_and_TraffInfo.mat','meanspeed_record');
load ('ego_vehicle_data.mat','cutpoint');
traffspeed=zeros(1000, 24);%里程1000，次数24
mileage=linspace(0,cutpoint(end),1000)';
j=2;
i=1;
while i<=1000
    while mileage(i)<=cutpoint(j)
        traffspeed(i,:)=meanspeed_record(j-1,:);
        i=i+1;
    end
    j=j+1;
end
[XX,YY]=meshgrid([300:300:7200]',mileage);
surf(XX,YY,traffspeed);
ylabel('Distance[m]');xlabel('Time/×300s');
zlabel('Velocity/kph');
%%速度和高程
load ('OptimalPMP.mat','v','alph_cyc','distance','s');
yyaxis left;
plot(distance,v,'linewidth',1);
ylabel('Velocity [kph];');
yyaxis right;
height=cumtrapz(s,alph_cyc);
plot(s/1000,height,'linewidth',3);
ylabel('Elevation [m]');
ylim([0 600]);xlim([0 120]);
%% %%CDR曲线
clear;
PMcon=18;Qnom=504000;
Pdmd1=[1:1:PMcon]'*1000;
Voc=425;Resis=0.0343;Papu1s=60e3;
sqe=-1/Voc/Qnom.*(1+Resis.*Pdmd1/Voc^2);
Pdmd2=[PMcon:1:230]'*1000;
sqh=-(Pdmd2-Papu1s)/Voc/Qnom./Pdmd2.*(1+Resis.*(Pdmd2-Papu1s)/Voc^2);
sq=[sqe;sqh];Pdis=[Pdmd1;Pdmd2]/1000;
load ('OptimalPMP.mat','SoC','Pdc','time','t_cyc','ess_soc','ess_ro','Pb','Ns','Np','Vb');
Res=interp1(ess_soc,ess_ro,SoC);
Res=interp1(time,Res,t_cyc)*Ns/Np;Pb=interp1(time,Pb,t_cyc);
Vb=interp1(time,Vb,t_cyc);Pdc=interp1(time,Pdc,t_cyc);
CDR=-( (Vb-sqrt(Vb.^2-4.*Res.*Pb*1000))/2./Res/Qnom )./(Pdc*1000);
plot(Pdc,CDR,'o','linewidth',0.2,'markersize',3);hold on;
plot(Pdis,sq,'--','linewidth',3);hold on;
ylim([-5e-9 12e-9]);xlim([0 230]);xlabel('Pdmd[kW]');ylabel('CDR');
legend('Optimal','Estimated');
set(gca,'fontsize',15);

figure(2);
f=fit(Pdis, sq, 'smoothingspline', 'SmoothingParam', 5.517359953682451E-5);
% f=fit(Pdis, sq, 'smoothingspline', 'SmoothingParam', 5.51735995e-5);
plot(f,Pdis,sq);
xlabel('Pdmd');ylabel('CDR');set(gca,'fontsize',15);ylim([-5e-9 10e-9]);
ylim([-5e-9 2e-9]);
%CDR预测方法display
%% 
clear;
load ('NPMP.mat','CDRMethod','distance');
load ('DrivingCyle_and_TraffInfo.mat','meanspeed_record');
timecut=[];
num=1;
for i=1:length(distance)-1
    if distance(i)<5*num && distance(i+1)>=5*num
        timecut=[timecut;i];
        num=num+1;
    end
end
timecut=[300;timecut];%timecut的长度就是预测的次数，每5km预测一次
CDR_method=zeros(49,length(timecut));
for j=1:length(timecut)
    CDR_method(:,j)=CDRMethod(:,1,timecut(j)+200);
end
CDR_method(CDR_method==0)=3;
heatmap(CDR_method);
xlabel('Distance/×5[km]');ylabel('Road Segment[/]');
%% 
%计算实际的Ratio
load OptimalPMP.mat
Ratio=trapz(time,Pdc)/trapz(time,Pb)
%查看功率分布
%% 
load ('NPMP.mat','Pdc','edge_record');
Pdc_tmp=Pdc(edge_record==43);
Pdc_tmp(Pdc_tmp<=0)=[];
[f,x]=ksdensity(Pdc_tmp);
plot(x,f);hold on;
load ('DrivingCyle_and_TraffInfo.mat','Pdmd_distribution');
plot(Pdmd_distribution(43,:));

%% 对比Papu_cmd和Papu 
load OptimalPMP.mat;
Papu=interp1(time,Papu,t_cyc);
Papu_cmd2=interp1(time,Papu_cmd2,t_cyc);
distance=interp1(time,distance,t_cyc);
plot(distance,Papu);hold on;
plot(distance,Papu_cmd2);
%% 其他数据分析
load ('NPMP.mat','K');
k1=K(3);k2=K(2);k3=K(1);
P1=60e3;
Voc=425;
Qnom=504000;
R=0.0343;
deltaH=zeros(100,100);
Pdmd=linspace(1,230e3,100);
lambda=linspace(-14500,-11500,100);
for i=1:100
    for j=1:100
    deltaH(i,j)=-k1*P1-k2*P1^2-k3*P1^3-lambda(i)*P1/Voc/Qnom+lambda(i)*R*(-2*Pdmd(j)*P1+P1^2)/Voc^3/Qnom;
    end
end
[X,Y]=meshgrid(Pdmd,lambda);
contourf(X,Y,deltaH);hold on;

y=(k1*P1+k2*P1^2+k3*P1^3)./(R.*(-2.*Pdmd*P1+P1^2)/Voc^3/Qnom-P1/Voc/Qnom);
A=plot(Pdmd,y,'r','linewidth',2);hold on;
legend([A],'deltaH==0');

clear;
load ('NPMP.mat','Pdc','lambda','time','t_cyc');
hold on;
Pdc=interp1(time,Pdc,t_cyc);
lambda=interp1(time,lambda,t_cyc);
lambda(Pdc<0)=[];
Pdc(Pdc<0)=[];
lambda(1)=lambda(2);
plot(Pdc*1000,lambda,'ro','linewidth',2);
ylim([-14500 -11500]);
xlabel('Pdmd[W]');ylabel('lambda[-]');

load('NPMP_energy.mat');
lamda=lambda;
analyPM;
plot(distance,PMreal);
hold on;
plot(distance,Pdc);
legend('PM','Pdmd');
xlabel('Distance[m]');ylabel('Power[kW]');
ylim([-300 300]);
%% CDR预测精度，最优CDR和预测CDR之间的差距
clear;
load ('OptimalPMP.mat','Pdc','time','SoC');
distance_edge=ones(120,1)*1000;
load ('NPMP.mat','s','CDR_pre','edge_record');
CDRact=[];
deltaSOC=zeros(120,1);deltaQdmd=zeros(120,1);
for i=1:120
    [step,~]=find(edge_record==i);
    Pdc_tmp=Pdc(step);
    time_tmp=time(step);
    deltaSOC(i)=SoC(step(end))-SoC(step(1));
    deltaQdmd(i)=trapz(time_tmp,Pdc_tmp*1000);
    CDRact=[CDRact;deltaSOC(i)/deltaQdmd(i)];
end
CDRest=zeros(length(CDRact),fix(s(end)/5000)+1);
est_time=[];%预测节点
flag=0;
for i=1:length(s)-1
    if (s(i)<flag && s(i+1)>=flag) || flag==0
        est_time=[est_time;i];
        flag=flag+5000;
    end
end
est_time=est_time*100+800;%3s偏移
est_time=[est_time;length(Pdc)];
for i=1:length(est_time)
    CDRest(:,i)=CDR_pre(:,:,est_time(i));
end
CDRerror=zeros(size(CDRest,1),size(CDRest,2));
for i=1:size(CDRest,2)
    CDRerror(:,i)=(CDRest(:,i)-CDRact)./CDRact;
end
CDRerror(CDRerror==1)=NaN;
cutpoint=cumsum(distance_edge);
CDRest_tmp=CDRest(:,1);
CDRest_tmp=[CDRest_tmp;CDRest_tmp(end)];CDRact=[CDRact;CDRact(end)];
cutpoint=[0;cutpoint];
% stairs(cutpoint,CDRest_tmp,'linewidth',2);hold on;
% stairs(cutpoint,CDRact,'--','linewidth',2);
stairs(cutpoint, (CDRest_tmp-CDRact)./CDRact ,'linewidth',2);
ylim([-3 3]);
% legend('Est','Opt');
xlabel('Distance[m]');ylabel('CDR error');xlim([0 cutpoint(end)]);
set(gca,'fontsize',15);
std(CDRerror(6:end,2))
mean(CDRerror(6:end,2))
%% 数据分析
clear;
Cd=0.8;
A=7.67;
m=14500;
g=9.8;
f=0.012;
eta=0.8;
vel=[0:0.5:120/3.6];
Pdmd=(Cd*A.*(vel*3.6).^2/21.15+m*g*f).*vel/eta;
% plot(Pdmd,v,'linewidth',2);
xlabel('Pdmd[W]');
ylabel('v[mps]');

Papu=0;
Voc=425;
R=0.0343;
Qnom=504000;
k_low=-(Voc-real(sqrt(Voc^2-4*R.*(Pdmd-Papu))))/2/R/Qnom./vel;
plot(Pdmd,k_low,'linewidth',2);hold on;
Papu=75e3;
k_up=-(Voc-real(sqrt(Voc^2-4*R.*(Pdmd-Papu))))/2/R/Qnom./vel;
plot(Pdmd,k_up,'linewidth',2);hold on;
xlabel('Pdmd[W]');ylabel('k[m^{-1}]');
set(gca,'fontsize',12);
ylim([-4e-5 4e-5]);xlim([0 230e3]);
load ('OptimalPMP.mat','SoC','distance','Pdc','time','t_cyc','v','Vb','ess_soc','ess_ro','Pb','Qnom','Ns','Np');
Res=interp1(ess_soc,ess_ro,SoC);
Res=interp1(time,Res,t_cyc)*Ns/Np;Pb=interp1(time,Pb,t_cyc);
Vb=interp1(time,Vb,t_cyc);v=interp1(time,v,t_cyc);Pdc=interp1(time,Pdc,t_cyc);
k=-( (Vb-sqrt(Vb.^2-4.*Res.*Pb*1000))/2./Res/Qnom )./(v/3.6);
plot(Pdc*1000,k,'o','markersize',4,'color',[0.5 0.45 0.74],'linewidth',1.2);

%预测曲线
PMcon=18e3;
Pdmd1=Pdmd( 1:max(find(Pdmd<PMcon)) );
Voc=425;Resis=0.0343;Papu1s=60e3;
sqe=-1/Voc/Qnom.*(1+Resis.*Pdmd1/Voc^2);
Pdmd2=Pdmd( min(find(Pdmd>=PMcon)):end );
sqh=-(Pdmd2-Papu1s)/Voc/Qnom./Pdmd2.*(1+Resis.*(Pdmd2-Papu1s)/Voc^2);
CDR=[sqe,sqh];
k_est=CDR.*Pdmd./vel;
plot(Pdmd(6:46),k_est(6:46),'--','linewidth',3,'color','r');
legend('Low boundary','Up boundary','Optimal Point','Estimated Optimal Point');

%% 确定SOC轨迹的上边界
clear;
load ('NPMP.mat','Pdc','edge_record','time');
load ('OptimalPMP.mat','distance','SoC');
Cd=0.8;
A=7.67;
m=14500;
g=9.8;
f=0.012;
eta=0.8;
vel=[0:0.5:120/3.6];
Pdmd=(Cd*A.*(vel*3.6).^2/21.15+m*g*f).*vel/eta;
Papu=75e3;
Voc=425;
R=0.0343;
Qnom=504000;
k_up=-(Voc-real(sqrt(Voc^2-4*R.*(Pdmd-Papu))))/2/R/Qnom./vel;
k_up(k_up>2e-5)=2e-5;
Pdmd_mean=zeros(120,1);
for i=1:120
    Pdc_tmp=Pdc(edge_record==i);
    time_tmp=time(edge_record==i);
    Pdmd_mean(i)=trapz(time_tmp,Pdc_tmp*1000)/(time_tmp(end)-time_tmp(1));
end
k_upbound=zeros(120,1);
for i=1:120
    k_upbound(i)=interp1(Pdmd,k_up,Pdmd_mean(i),'linear');
end
k_upbound=fillmissing(k_upbound,'constant',1e-5);
load ('NPMP_test2.mat','distance','soc_ref');
figure(2);
soc_ref(soc_ref<0.1)=0.9;
A=plot(distance,soc_ref,'linewidth',2);hold on;
for i=0:119
    x_start=i;
    x_end=x_start+1;
    y_start=soc_ref(min(find(distance>=x_start)));
    y_end=k_upbound(i+1)*(x_end-x_start)*1000+y_start;
    B=plot([x_start x_end],[y_start y_end],'r','linewidth',1);hold on;
end
xlabel('Distance[km]');ylabel('SOC[-]');
title('规划方法：test2');
legend([A,B],'规划SOC','SOC上边界');
set(gca,'fontsize',15);

%% APU功率对比
load NPMP.mat;
subplot(2,1,1);
distance=interp1(time,distance,t_cyc);
Papu=interp1(time,Papu,t_cyc);
plot(distance,Papu);
title('无抗饱和');ylabel('Papu[kW]');xlim([0 distance(end)]);
set(gca,'fontsize',15);

load NPMP_AntiWindup.mat;
subplot(2,1,2);
distance=interp1(time,distance,t_cyc);
Papu=interp1(time,Papu,t_cyc);
plot(distance,Papu);
title('有抗饱和');ylabel('Papu[kW]');xlim([0 distance(end)]);
set(gca,'fontsize',15);
%% 平均车速预测精度
clear;
load ('NPMP.mat','traffspeed','time','edge_record','distance');
distance=distance*1000;
meanspeed=zeros(120,1);%实际
for i=1:120
    s_tmp=distance(edge_record==i);
    time_tmp=time(edge_record==i);
    meanspeed(i)=(s_tmp(end)-s_tmp(1)) / (time_tmp(end)-time_tmp(1)); %mps
end
meanspeed=meanspeed*3.6;
plot( (traffspeed(:,1)-meanspeed)./ meanspeed ,'o','linewidth',2);
ylabel('Error of velocity');
%分析各段Fi*CDRi的占比
%% 
clear
load ('NPMP.mat','Pdc','time','edge_record','distance','CDR_pre','s');
Qdmd=cumtrapz(time,Pdc*1000);
eta=0.8;
Fact=zeros(120,1);
for i=1:120
    index=find(edge_record==i);
    Qdmd_tmp=Qdmd(index);
    deltaQdmd=Qdmd_tmp(end)-Qdmd_tmp(1);
    Fact(i)=deltaQdmd*eta/1000;
end
load ('OptimalPMP.mat','time','SoC');
distance_edge=ones(120,1)*1000;
CDRact=[];
deltaSOC=zeros(120,1);deltaQdmd=zeros(120,1);
for i=1:120
    [step,~]=find(edge_record==i);
    Pdc_tmp=Pdc(step);
    time_tmp=time(step);
    deltaSOC(i)=SoC(step(end))-SoC(step(1));
    deltaQdmd(i)=trapz(time_tmp,Pdc_tmp*1000);
    CDRact=[CDRact;deltaSOC(i)/deltaQdmd(i)];
end
clear;
load ('OptimalPMP.mat','time','SoC','distance');
load ('NPMP.mat','edge_record');
K_act=zeros(120,1);
for i=1:120
    index=find(edge_record==i);
    SOC_tmp=SoC(index);
    dis_tmp=distance(index)*1000;
    K_act(i)=(SOC_tmp(end)-SOC_tmp(1))/(dis_tmp(end)-dis_tmp(1));
end
stairs([0:120]',[K_act;K_act(end)]);

K_est=[
   -0.0378
   -0.0398
   -0.0413
   -0.0166
   -0.0785
    0.0028
   -0.0328
   -0.0315
   -0.0465
   -0.0794
    0.0422
    0.0421
    0.0407
    0.0134
    0.0635
    0.0620
         0
    0.0481
    0.0053
    0.0623
    0.0711
    0.1030
    0.1013
    0.1086
    0.0990
    0.0985
    0.1022
   -0.0315
    0.0853
    0.0908
    0.1025
    0.0984
    0.1048
    0.0932
    0.1140
    0.1185
    0.1070
    0.1111
    0.1070
    0.1140
    0.0735
    0.1054
    0.1111
    0.1295
    0.0976
    0.1037
    0.1111
    0.1054
    0.1320
    0.0961
    0.0467
    0.0999
    0.1111
    0.1400
    0.1111
    0.1111
    0.1046
    0.1077
    0.1316
    0.0969
    0.0529
   -0.0051
   -0.0718
    0.1787
    0.2995
   -0.0623
   -0.2345
    0.1513
    0.0996
    0.1126
    0.0406
    0.0214
    0.0074
    0.0119
   -0.0033
    0.0080
    0.0002
    0.0083
    0.0221
   -0.1068
    0.1235
   -0.2155
    0.0232
    0.0480
    0.0543
    0.0684
   -0.0464
   -0.0022
   -0.0352
   -0.0714
   -0.0007
   -0.0136
   -0.0323
   -0.0813
    0.0290
   -0.1185
   -0.1203
    0.0205
   -0.0098
   -0.0735
    0.0574
    0.1031
    0.1134
    0.0942
    0.0903
    0.0874
    0.0398
    0.0937
    0.1007
    0.0932
    0.0927
    0.0377
    0.1111
    0.1111
    0.1111
    0.1111
    0.1000
    0.1206
    0.1407
    0.1111
]*1.0e-04;
K_est=-K_est;
hold on;
stairs([0:1:120]',[K_est;K_est(end)]);
legend('act','est');ylabel('K');
figure(2);
K_error=(K_est-K_act)./K_act;
stairs([0:1:120]',[K_error;K_error(end)]);
% ylim([-1 1]);
ylabel('Error');
mean(K_error)
std(K_error)