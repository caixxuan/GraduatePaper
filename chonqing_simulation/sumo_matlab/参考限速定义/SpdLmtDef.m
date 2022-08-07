%%定义参考限速
RefSpdLmt=zeros(16,42);
for i=1:4
       RefSpdLmt(i,:)=spdlmt;
end
%交通恶化
for i=5:10
    RefSpdLmt(i,:)=RefSpdLmt(i-1,:)-1;
end
%交通好转
for i=11:16
    RefSpdLmt(i,:)=RefSpdLmt(i-1,:)+1;
end
