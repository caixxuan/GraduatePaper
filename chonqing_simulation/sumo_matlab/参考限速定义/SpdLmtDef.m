%%����ο�����
RefSpdLmt=zeros(16,42);
for i=1:4
       RefSpdLmt(i,:)=spdlmt;
end
%��ͨ��
for i=5:10
    RefSpdLmt(i,:)=RefSpdLmt(i-1,:)-1;
end
%��ͨ��ת
for i=11:16
    RefSpdLmt(i,:)=RefSpdLmt(i-1,:)+1;
end
