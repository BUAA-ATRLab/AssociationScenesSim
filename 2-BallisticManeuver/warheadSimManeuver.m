%Version��warheadSim1.m
%Description: �������ɵ�ͷ�켣���ݣ����������Ρ����ɶ��Լ������
%launchBLA:����㾭γ���Լ���׼��λ��
%launchLandLatiLong��������Լ���㾭γ��
%numGroup���ű��
%������������ѧ
function [] = warheadSimManeuver(launchBLA,launchLandLatiLong,numGroup,settings,environment)

numTrajectory=settings.iMisNum;

latiLaunch=launchLandLatiLong(1);
longLaunch=launchLandLatiLong(2);
latiLand=launchLandLatiLong(3);
longLand=launchLandLatiLong(4);

%%%%BM ��ʼ���ò��� %%%%%%%%%

launchBLH=[latiLaunch,longLaunch,0];
landBLH=[latiLand,longLand,0];

launchWGS84=BLH2WGS84(launchBLH,environment);
landWGS84=BLH2WGS84(landBLH,environment);

%cos()=(A.*B)/(|A|*|B|)
paraTmp=(launchWGS84(1)*landWGS84(1)+launchWGS84(2)*landWGS84(2)+launchWGS84(3)*landWGS84(3))/...
    sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))/...
    sqrt(power(landWGS84(1),2)+power(landWGS84(2),2)+power(landWGS84(3),2));
angleLaunchLand=acos(paraTmp);%����������ĵ�ʸ���гɵ�Բ�Ľ�

rangeBM=angleLaunchLand*(sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))+...
    sqrt(power(landWGS84(1),2)+power(landWGS84(2),2)+power(landWGS84(3),2)))/2;

%KL����̹���ϵ����P271��trajectorymissiletrajectoryѧ��
%��Need to be addressed��ȡֵ����
if rangeBM<=2000e3%��λ��m
    KL=1.31;
elseif rangeBM<=3200e3
    KL=1.315;
else
    KL=1.30;
end

R_R=environment.R_R;
betaPassiveSegment=rangeBM/KL/R_R;%��������̽�
rangeBMkm=rangeBM/1000;
%Active Segment�յ��ٶȣ�m/s
VActiveSegmentEnd=11.9*sqrt(tan(rangeBMkm/222.4/KL*pi/180)*tan((45-rangeBMkm/444.8/KL)*pi/180))*1e3;
%miuk:������ v0:�������Ʊ�
[miuk,v0]=calculateMiukV0(betaPassiveSegment,VActiveSegmentEnd,settings,environment);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Active Segment%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pb0=settings.Pb0;
T0=v0*Pb0;  %����ʱ��ĸ�������
theta_k=pi/4-betaPassiveSegment/4;
g0=environment.g0;
delta_t=settings.delta_t;

t_k=T0*(1-miuk);  %Active Segment����ʱ��
num_point1=floor(t_k/delta_t)+1;

t = zeros(1,num_point1);%[���������ϳ�]%ʱ�������
miu = zeros(1,num_point1);%����������P264�� [������Active Segment����]
for i=1:num_point1
    t(i)=(i-1)*delta_t;
    miu(i)=1-t(i)/T0;
end

VActive = zeros(1,num_point1);%[������Active Segment�յ��ٶȼ������]
VxActive = zeros(1,num_point1);%[���������ϳ�]
VyActive = zeros(1,num_point1);%[���������ϳ�]
%launch����ϵ�������ָ����ĵ�λ����������
thetaXActive = zeros(1,num_point1);%[���������ϳ�]
thetaYActive = zeros(1,num_point1);%[���������ϳ�]
[VActive,~,VxActive,VyActive,thetaXActive,thetaYActive]=calculateVActiveEndByMiukV0(miuk,v0,betaPassiveSegment,settings,environment);

%%��������ϵ����
XActiveG = zeros(1,num_point1);%[���������ϳ�]
YActiveG = zeros(1,num_point1);%[���������ϳ�]

for i=1:num_point1
    if miu(i)==1
        XActiveG(i)=0;
        YActiveG(i)=0;
    elseif miu(i)<1 && miu(i)>=0.95
        XActiveG(i)=0;
        YActiveG(i)=YActiveG(i-1)+T0*VActive(i)*(miu(i-1)-miu(i));
    elseif miu(i)>=0.45
        XActiveG(i)=XActiveG(i-1)+T0*VActive(i)*integral2(miu(i-1),miu(i));
        YActiveG(i)=YActiveG(i-1)+T0*VActive(i)*integral1(miu(i-1),miu(i));
    else
        XActiveG(i)=XActiveG(i-1)+T0*cos(theta_k)*VActive(i)*(miu(i-1)-miu(i));
        YActiveG(i)=YActiveG(i-1)+T0*sin(theta_k)*VActive(i)*(miu(i-1)-miu(i));
    end
end

miu=[];

V_k=VActive(num_point1);%Active Segment�յ��ٶ�%(1,���������������������)
X_k=XActiveG(num_point1);%Active Segment�յ�����X��%(1,������Free section��ʼ�׶�)
Y_k=YActiveG(num_point1);%Active Segment�յ�����Y��%(1,������Free section��ʼ�׶�)

VActive=[];

%%λ������ת����84����ϵ(���ֱ������ϵ)
XActiveS = zeros(1,num_point1);%[���������ϳ�]
YActiveS = zeros(1,num_point1);%[���������ϳ�]
ZActiveS = zeros(1,num_point1);%[���������ϳ�]

coordS = zeros(1,3);
for i=1:num_point1
    coordS=G2S(launchBLA,launchLandLatiLong,[XActiveG(i),YActiveG(i),0],environment);
    XActiveS(i)=coordS(1);
    YActiveS(i)=coordS(2);
    ZActiveS(i)=coordS(3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
%%%%%%%%%%Free Section%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

fM=environment.fM;

%Active Segment�յ��ڵ�������ϵ�µ�����
%coordS=G2S(launchBLA,launchLandLatiLong,[X_k,Y_k,0],environment);
coordS=[XActiveS(num_point1),YActiveS(num_point1),ZActiveS(num_point1)];
R_k=sqrt(power(coordS(1),2)+power(coordS(2),2)+power(coordS(3),2));

v_energy=power(V_k,2)*R_k/fM;                                  %��������
P_semi_p=R_k*v_energy*power((cos(theta_k)),2);                 %��ͨ��
e_e=sqrt(1+v_energy*(v_energy-2)*power(cos(theta_k),2));       %ƫ����e
a_a=P_semi_p/(1-power(e_e,2));                                 %��Բ������a
b_b=P_semi_p/sqrt(1-power(e_e,2));                             %��Բ�̰���b

T_e=2*sqrt(power(a_a,3)/fM)*(acos((1-v_energy)/e_e)+e_e*sin(acos((1-v_energy)/e_e)));%Free section����ʱ��
E_k=acos((1-R_k/a_a)/e_e);                                                           %ƫ�����Ek%%P203
T_cycle=2*pi*power(a_a,1.5)/sqrt(fM);                                                %�ʵ�����Բ����һ�ܵ�ʱ��
t_p=t_k-T_cycle*(E_k-e_e*sin(E_k))/(2*pi);                                           %missile�ɾ����ص��ʱ��tp

wz=sqrt(fM/power(P_semi_p,3))*power(1-e_e,2);                                        %�������Heavy Decoy��������%warhead�Ƶ���ת�����ٶ�

num_point2=floor((T_e+t_k)/delta_t)-num_point1+2; %�ڶ��εĲ�������(���ɶ�)
t_2 = zeros(1,num_point2);%[���������ϳ��ܵ�ʱ��]

t_2(1)=t(num_point1);
for i=2:num_point2
    t_2(i)=t_2(1)+(i-1)*delta_t;
end

%��һʱ�̵�ƫ�����
E_t0=0;
E_t1=zeros(1,num_point2);%[��������һʱ��trajectory��Ǽ������]
for i=1:num_point2
    %�������⿪���շ����������ʱ�̵�ƫ�����P200
    %M=E-e*sinE
    E_t0=E_k;
    E_t1(i)=E_t0+((t_2(i)-t_p)*2*pi/T_cycle-E_t0+e_e*sin(E_t0))/(1-e_e*cos(E_t0));
    while abs(E_t1(i)-E_t0)/abs(E_t1(i))>1e-8
        E_t0=E_t1(i);
        E_t1(i)=E_t0+((t_2(i)-t_p)*2*pi/T_cycle-E_t0+e_e*sin(E_t0))/(1-e_e*cos(E_t0));
    end
end

%��һʱ�̵��ľ�r=a(1-ecosE)
rFree = zeros(1,num_point2);
for i=1:num_point2
    rFree(i)=a_a*(1-e_e*cos(E_t1(i)));
end

%��һʱ�̵�������r=P/(1-ecosf)
fFree = zeros(1,num_point2);%[������Reentry section֮ǰ���ѵ�һ��ֵ�����һ��ֵ���������������Ϳ����ͷ���]
for i=1:num_point2
    fFree(i)=acos((P_semi_p/rFree(i)-1)/e_e);
end

%��һʱ���ٶ�P203
VFree = zeros(1,num_point2);
for i=1:num_point2
    VFree(i)=sqrt(fM/a_a)*sqrt(1-power(e_e,2)*power(cos(E_t1(i)),2))/(1-e_e*cos(E_t1(i)));
end

%��һʱ��trajectory���
thetaFree = zeros(1,num_point2);
for i=1:num_point2
    thetaFree(i)=atan(e_e*sin(E_t1(i))/sqrt(1-power(e_e,2)));
end

E_t1=[];

%��һʱ�̷�������ϵ����
XFreeG = zeros(1,num_point2);%[������X_final�������]
YFreeG = zeros(1,num_point2);%[������Y_final�������]

XFreeG(1)=XActiveG(num_point1);
YFreeG(1)=YActiveG(num_point1);
XActiveG=[];
YActiveG=[];
for i=2:num_point2
    %P247Picture�����ε���ˮƽ����Ǳ仯��С���ԣ����ɶ˲��ܺ��ԣ�thetaFreeΪ�ٶ��뷢������ϵx��н�
    %ͳһת������������ϵ��
    XFreeG(i)=XFreeG(i-1)+VFree(i-1)*cos(thetaFree(i-1)-(fFree(i-1)-fFree(1)))*(t_2(i)-t_2(i-1));
    YFreeG(i)=YFreeG(i-1)+VFree(i-1)*sin(thetaFree(i-1)-(fFree(i-1)-fFree(1)))*(t_2(i)-t_2(i-1));
end

%λ�����귢������ϵת����84����ϵ�����ֱ�����꣩
XFreeS = zeros(1,num_point2);%[���������ϳ�]
YFreeS= zeros(1,num_point2);%[���������ϳ�]
ZFreeS = zeros(1,num_point2);%[���������ϳ�]

rTmp = 0;%��ʱ�Ĳ���������ת����ĵ��ľ�r
coordS = zeros(1,3);
for i=1:num_point2-1
    coordS=G2S(launchBLA,launchLandLatiLong,[XFreeG(i+1),YFreeG(i+1),0],environment);
    %����ת������һʱ�̵��ľ಻��
    %��һ���������ɶο�ʼ��
    rTmp=sqrt(power(coordS(1),2)+power(coordS(2),2)+power(coordS(3),2));
    XFreeS(i)=rFree(i+1)/rTmp*coordS(1);
    YFreeS(i)=rFree(i+1)/rTmp*coordS(2);
    ZFreeS(i)=rFree(i+1)/rTmp*coordS(3);
end

%��һʱ��ˮƽ�����ٶȺʹ�ֱ�����ٶ�
VxFree = zeros(1,num_point2);%[���������ϳ�]
VyFree = zeros(1,num_point2);%[���������ϳ�]

for i=1:num_point2%��һ�������������յ�
    VxFree(i)=VFree(i)*cos(thetaFree(i)-(fFree(i)-fFree(1)));
    VyFree(i)=VFree(i)*sin(thetaFree(i)-(fFree(i)-fFree(1)));
end

%launch����ϵ�������ָ����ĵ�λ����������
thetaXFree = zeros(1,num_point2);%[���������ϳ�]
thetaYFree = zeros(1,num_point2);%[���������ϳ�]

for i=1:num_point2
    thetaXFree(i)=cos(thetaFree(i)-(fFree(i)-fFree(1)));
    thetaYFree(i)=sin(thetaFree(i)-(fFree(i)-fFree(1)));
end

fFreeEnd=fFree(num_point2);
fFreeStart=fFree(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%/
%%%%%%%%%%Reentry section%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%/
%�����εķ���ʱ��P205
R_R=environment.R_R;
T_c=sqrt(power(a_a,3)/fM)*(acos((1-v_energy)/e_e)+acos((1-v_energy-(2-v_energy)*(R_k-R_R)/R_k)/e_e)+e_e*sin(acos((1-v_energy)/e_e))+sin(acos((1-v_energy-(2-v_energy)*(R_k-R_R)/R_k)/e_e)));
num_point3= floor((T_c+t_k)/delta_t)-num_point1-num_point2+3;
if rangeBMkm<=2000
    num_point3=round(num_point3*5);
else
    num_point3=round(num_point3*3);%%%%%%/���ڸ���landing point���ڵ�������%%%%%%%%%%
end

XReentryS = zeros(1,num_point3-1);%[���������ϳ�]
YReentryS = zeros(1,num_point3-1);%[���������ϳ�]
ZReentryS = zeros(1,num_point3-1);%[���������ϳ�]
VXReentryS = zeros(1,num_point3);%[���������ϳ�]
VYReentryS = zeros(1,num_point3);%[���������ϳ�]
VZReentryS = zeros(1,num_point3);%[���������ϳ�]
thetaXReentryS = zeros(1,num_point3);%[���������ϳ�]
thetaYReentryS = zeros(1,num_point3);%[���������ϳ�]
thetaZReentryS = zeros(1,num_point3);%[���������ϳ�]

%�������ľ࣬�ٶȣ�������
R_e=rFree(num_point2);
V_e=VFree(num_point2);
theta_e=thetaFree(num_point2);

%����ε���ֱ������
BLH_e=WGS842BLH([XFreeS(num_point2-1),YFreeS(num_point2-1),ZFreeS(num_point2-1)],environment);
h_e=BLH_e(3);
XYZ_e=[XFreeG(num_point2),YFreeG(num_point2),0];

XFreeG=[];
YFreeG=[];
rFree=[];
VFree=[];

t_3=zeros(1, num_point3);
t_3(1)=t_2(num_point2);
for i=2:num_point3
    t_3(i)=t_3(1)+(i-1)*delta_t;
end

mTarget=settings.mWarhead;

[XReentryS,YReentryS,ZReentryS,VXReentryS,VYReentryS,VZReentryS,thetaXReentryS,thetaYReentryS,thetaZReentryS,num_point3]=...
    ReentrySectionSimManeuver(R_e,V_e,theta_e,h_e,XYZ_e,num_point3,mTarget,fFreeStart,fFreeEnd,launchBLA,launchLandLatiLong,settings,environment);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
%%%%%%%%%%%��������%%%%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

%�����ݵ�(����֮�ͼ�ȥ�غϵ�������)
%�غϵĵ�Ϊ���ɶε�һ��������ε�һ��
numTotal=num_point1+num_point2+num_point3-2;
%printf("num_final=%d\n",num_final);

%ʱ��
tTotal = zeros(1,numTotal);%[���������] ��ʱ��

for i=1:num_point1
    tTotal(i)=t(i);
end
t=[];

for i=2:num_point2
    tTotal(num_point1-1+i)=t_2(i);
end
t_2=[];

for i=2:num_point3
    tTotal(num_point1+num_point2-2+i)=t_3(i);
end
t_3=[];

%λ��(���ֱ������ϵ)
coordX_84 = zeros(1,numTotal);%[���������] ������x
coordY_84 = zeros(1,numTotal);%[���������] ������y
coordZ_84 = zeros(1,numTotal);%[���������] ������z


for i=1:num_point1
    coordX_84(i)=XActiveS(i);
    coordY_84(i)=YActiveS(i);
    coordZ_84(i)=ZActiveS(i);
end

XActiveS=[];
YActiveS=[];
ZActiveS=[];

for i=2:num_point2
    coordX_84(num_point1-1+i)=XFreeS(i-1);
    coordY_84(num_point1-1+i)=YFreeS(i-1);
    coordZ_84(num_point1-1+i)=ZFreeS(i-1);
end

XFreeS=[];
YFreeS=[];
ZFreeS=[];

for i=2:num_point3
    coordX_84(num_point1+num_point2-2+i)=XReentryS(i-1);
    coordY_84(num_point1+num_point2-2+i)=YReentryS(i-1);
    coordZ_84(num_point1+num_point2-2+i)=ZReentryS(i-1);
end

XReentryS=[];
YReentryS=[];
ZReentryS=[];


%�ٶ�
VX_84 = zeros(1,numTotal);%[���������] ���ٶ�x
VY_84 = zeros(1,numTotal);%[���������] ���ٶ�y
VZ_84 = zeros(1,numTotal);%[���������] ���ٶ�z

VCoordS = zeros(1,3);
for i=1:num_point1
    VCoordS=G2S_2(launchBLA,[VxActive(i),VyActive(i),0]);
    VX_84(i)=VCoordS(1);
    VY_84(i)=VCoordS(2);
    VZ_84(i)=VCoordS(3);
end

VxActive=[];
VyActive=[];

for i=2:num_point2
    VCoordS=G2S_2(launchBLA,[VxFree(i),VyFree(i),0]);
    VX_84(num_point1-1+i)=VCoordS(1);
    VY_84(num_point1-1+i)=VCoordS(2);
    VZ_84(num_point1-1+i)=VCoordS(3);
end
VxFree=[];
VyFree=[];

for i=2:num_point3
    VX_84(num_point1+num_point2-2+i)=VXReentryS(i);
    VY_84(num_point1+num_point2-2+i)=VYReentryS(i);
    VZ_84(num_point1+num_point2-2+i)=VZReentryS(i);
end
VXReentryS=[];
VYReentryS=[];
VZReentryS=[];

%����(�����ָ����ĵ�λ����������)
directX_84 = zeros(1,numTotal);%[���������] �ܷ���x
directY_84 = zeros(1,numTotal);%[���������] �ܷ���y
directZ_84 = zeros(1,numTotal);%[���������] �ܷ���z

thetaCoordS = zeros(1,3);
for i=1:num_point1
    thetaCoordS = G2S_2(launchBLA,[thetaXActive(i),thetaYActive(i),0]);
    directX_84(i)=thetaCoordS(1);
    directY_84(i)=thetaCoordS(2);
    directZ_84(i)=thetaCoordS(3);
end

thetaXActive=[];
thetaYActive=[];

for i=2:num_point2
    thetaCoordS = G2S_2(launchBLA,[thetaXFree(i),thetaYFree(i),0]);
    directX_84(num_point1-1+i)=thetaCoordS(1);
    directY_84(num_point1-1+i)=thetaCoordS(2);
    directZ_84(num_point1-1+i)=thetaCoordS(3);
end
thetaXFree=[];
thetaYFree=[];

for i=2:num_point3
    directX_84(num_point1+num_point2-2+i)=thetaXReentryS(i);
    directY_84(num_point1+num_point2-2+i)=thetaYReentryS(i);
    directZ_84(num_point1+num_point2-2+i)=thetaZReentryS(i);
end
thetaXReentryS=[];
thetaYReentryS=[];
thetaZReentryS=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
%%%%%%%%%%%�ļ��洢%%%%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
% �洢��������
id=numTrajectory*1000+numGroup*100+1*10+1;
str1='.\mid\Missiles_Track\warheadManeuver';
ch1=num2str(numTrajectory);
ch2=num2str(numGroup);
ch3='11';
str2=[ch1,ch2,ch3,'_84.txt'];
str1A = [str1,str2];

fp = fopen(str1A, 'w');
if (fp==-1)
    error(['cannot open ',str1A, '\n']);
end

fprintf(fp,'%10s','t');
fprintf(fp,'%5s','');
fprintf(fp,'%10s','idBGTN');
fprintf(fp,'%5s','');fprintf(fp,'%15s','coordX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','coordY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','coordZ_84');
fprintf(fp,'%5s','');fprintf(fp,'%15s','VX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','VY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','VZ_84');
fprintf(fp,'%5s','');fprintf(fp,'%15s','directX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','directY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','directZ_84');
fprintf(fp,'\n');

for i=1:numTotal
    fprintf(fp,'%10.4f',tTotal(i));
    fprintf(fp,'%5s','');
    fprintf(fp,'%10.4d',id);
    fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordX_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordY_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordZ_84(i));
    fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VX_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VY_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VZ_84(i));
    fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directX_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directY_84(i));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directZ_84(i));
    fprintf(fp,'\n');
end
fclose(fp);


