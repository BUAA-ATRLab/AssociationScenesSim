%Version��BLH2WGS84.m
%Description: ����һ��γ�Ⱦ��ȸ߶�ת��ΪWGS84����ϵ���ꡣ��γ��Լ��Ϊ��γΪ������γΪ��������Ϊ��������Ϊ����
%(B,L,H)->(X,Y,Z)
%��������ϵ������������ܷ�������ʽ(2.8)
function [coordWGS84] = BLH2WGS84(BLH,environment)
    a = environment.a;
    e2 = environment.e_2;
    
    lati = BLH(1)*pi/180;
    long = BLH(2)*pi/180;
    alti = BLH(3);
    
    N = a/sqrt(1-e2*power(sin(lati),2)); %î��Ȧ���ʰ뾶
    
    %lati[-pi/2,pi/2],long[-pi,pi]
    coordWGS84(1)=(N+alti)*cos(lati)*cos(long);
    coordWGS84(2)=(N+alti)*cos(lati)*sin(long);
    coordWGS84(3)=(N*(1-e2)+alti)*sin(lati);
end
    
    
    
    