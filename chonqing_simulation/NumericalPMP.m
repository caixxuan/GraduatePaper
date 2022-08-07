if Pdmd>=0
    PapuCand = -1:0.5:76;
    Vb = interp1(ess_soc,ess_voc,SOC,'linear','extrap');
    Vb = Vb*Ns;
    Rb = interp1(ess_soc,ess_ro,SOC,'linear','extrap');
    Rb = Rb*Ns/Np;
    Pb = Pdmd-PapuCand;
    FR= polyval(K,PapuCand*1000);%三阶油耗系数K/W，五阶油耗系数p2/kW
    %     FR = p2(1).*PapuCand.^4+p2(2).*PapuCand.^3+p2(3).*PapuCand.^2+p2(4).*PapuCand + p2(5);
    Ib = real((Vb-sqrt(Vb.^2-4*1000*Pb.*Rb))./(2*Rb));
    H = FR-lambda.*Ib./(Qnom);
    [~,ind] = min(H);
    PapuCtrl = PapuCand(ind);
else
    PapuCtrl=0;
end