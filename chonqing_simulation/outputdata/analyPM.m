k1=7.3213e-5;k2=-4.8287e-10;k3=4.1035e-15;k0=0.0808;k=[k3;k2;k1;k0];
Vpack=polyval(pv,SoC);Rpack=polyval(pr,SoC);
Pdd=Pdc;Pdd(Pdd<0)=0;

Papu1=(-(k2-lamda.*Rpack./Vpack.^3/Qnom)+real(sqrt((k2-lamda.*Rpack./Vpack.^3/Qnom).^2-3*k3.*(k1+lamda./Vpack/Qnom+2.*lamda.*Rpack./Vpack.^3/Qnom.*Pdd*1000))))/3/k3;%¶þ½×Ì©ÀÕÕ¹¿ª
a=4.*Rpack.*Papu1;
b=-(2.*Rpack*Qnom)./lamda.*(k1.*Papu1+k2.*Papu1.^2+k3.*Papu1.^3);
PMreal=1/4./Rpack.*(Vpack.^2-(a-b.^2).^2/4./b.^2)./1000;          