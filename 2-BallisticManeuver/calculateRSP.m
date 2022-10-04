%Date：2021.1.31
%Version：calculateRSP.m
%Description:输入高度，对应密度、音速、压强比

function [rou,sonic,p_p0]=calculateRSP(h,environment)
    r0=environment.r0;
    p0=environment.p0;
   	rou0=environment.rou0;
    
    H_km=0; % 位势高度,单位，km
	h_km=0; % 高度,单位，km
	T_k=0; % 运动学温度
	rou_rou0=0; % 密度比
	W=0;
    
	H_km=r0*h/(r0+h)/1000;
	h_km=h/1000;
	if h_km>=0 && h_km<=11.0191 
		W=1-H_km/44.3308;
		T_k=288.15*W;
		sonic=20.0468*sqrt(T_k);
		p_p0=power(W,5.2559);
		rou_rou0=power(W,4.2559);
		rou=rou_rou0*rou0;
    elseif h_km<=20.0631
		W=exp((14.9647-H_km)/6.3416);
		T_k=216.650;
		sonic=20.0468*sqrt(T_k);
		p_p0=0.11953*W;
		rou_rou0=0.15898*W;
		rou=rou_rou0*rou0;
    elseif h_km<=32.1619
		W=1+(H_km-24.9021)/221.552;
		T_k=221.552*W;
		sonic=20.0468*sqrt(T_k);
		p_p0=2.5158e-2*power(W,-34.1629);
		rou_rou0=3.2722e-2*power(W,-35.1629);
		rou=rou_rou0*rou0;
    elseif h_km<=47.3501
		W=1+(H_km-39.7499)/89.4107;
		T_k=250.350*W;
		sonic=20.0468*sqrt(T_k);
		p_p0=2.8338e-3*power(W,-12.2011);
		rou_rou0=3.2618e-3*power(W,-13.2011);
		rou=rou_rou0*rou0;
    elseif h_km<=51.4125
		W=exp((48.6252-H_km)/7.9223);
		T_k=270.650;
		sonic=20.0468*sqrt(T_k);
		p_p0=8.9155e-4*W;
		rou_rou0=9.4920e-4*W;
		rou=rou_rou0*rou0;
    elseif h_km<=71.8020
		W=1-(H_km-59.4390)/88.2218;
		T_k=247.021*W;
		sonic=20.0468*sqrt(T_k);
		p_p0=2.1671e-4*power(W,12.2011);
		rou_rou0=2.5280e-4*power(W,11.2011);
		rou=rou_rou0*rou0;
    elseif h_km<=86.0000
		W=1-(H_km-78.0303)/100.2950;
		T_k=200.590*W;
		sonic=20.0468*sqrt(T_k);
        p_p0=1.2274e-5*power(W,17.0816);
		rou_rou0=1.7632e-5*power(W,16.0816);
		rou=rou_rou0*rou0;
	else
		W=exp((87.2848-H_km)/5.4700);
		T_k=186.8700;
		sonic=20.0468*sqrt(T_k);
		p_p0=(2.2730+1.042e-3*H_km)*1e-6*W;
		rou_rou0=3.6411e-6*W;
		rou=rou_rou0*rou0;
    end
end
    
    