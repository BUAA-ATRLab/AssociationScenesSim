%Version：warheadSim1.m
%Description: 用于生成弹头轨迹数据，包括主动段、自由段以及再入段
%launchBLA:发射点经纬度以及瞄准方位角
%launchLandLatiLong：发射点以及落点经纬度
%numGroup：团编号
%弹道导弹弹道学
function [] = warheadSimManeuver(launchBLA,launchLandLatiLong,numGroup,settings,environment)

numTrajectory=settings.iMisNum;

latiLaunch=launchLandLatiLong(1);
longLaunch=launchLandLatiLong(2);
latiLand=launchLandLatiLong(3);
longLand=launchLandLatiLong(4);

%%%%BM 初始设置参数 %%%%%%%%%

launchBLH=[latiLaunch,longLaunch,0];
landBLH=[latiLand,longLand,0];

launchWGS84=BLH2WGS84(launchBLH,environment);
landWGS84=BLH2WGS84(landBLH,environment);

%cos()=(A.*B)/(|A|*|B|)
paraTmp=(launchWGS84(1)*landWGS84(1)+launchWGS84(2)*landWGS84(2)+launchWGS84(3)*landWGS84(3))/...
    sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))/...
    sqrt(power(landWGS84(1),2)+power(landWGS84(2),2)+power(landWGS84(3),2));
angleLaunchLand=acos(paraTmp);%起落点与球心的矢径夹成的圆心角

rangeBM=angleLaunchLand*(sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))+...
    sqrt(power(landWGS84(1),2)+power(landWGS84(2),2)+power(landWGS84(3),2)))/2;

%KL：射程估算系数，P271（trajectorymissiletrajectory学）
%【Need to be addressed】取值很迷
if rangeBM<=2000e3%单位：m
    KL=1.31;
elseif rangeBM<=3200e3
    KL=1.315;
else
    KL=1.30;
end

R_R=environment.R_R;
betaPassiveSegment=rangeBM/KL/R_R;%被动段射程角
rangeBMkm=rangeBM/1000;
%Active Segment终点速度，m/s
VActiveSegmentEnd=11.9*sqrt(tan(rangeBMkm/222.4/KL*pi/180)*tan((45-rangeBMkm/444.8/KL)*pi/180))*1e3;
%miuk:质量比 v0:地面重推比
[miuk,v0]=calculateMiukV0(betaPassiveSegment,VActiveSegmentEnd,settings,environment);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Active Segment%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pb0=settings.Pb0;
T0=v0*Pb0;  %理想时间的辅助参数
theta_k=pi/4-betaPassiveSegment/4;
g0=environment.g0;
delta_t=settings.delta_t;

t_k=T0*(1-miuk);  %Active Segment飞行时间
num_point1=floor(t_k/delta_t)+1;

t = zeros(1,num_point1);%[保留到最后合成]%时间采样点
miu = zeros(1,num_point1);%辅助参数（P264） [保留到Active Segment结束]
for i=1:num_point1
    t(i)=(i-1)*delta_t;
    miu(i)=1-t(i)/T0;
end

VActive = zeros(1,num_point1);%[保留到Active Segment终点速度计算出来]
VxActive = zeros(1,num_point1);%[保留到最后合成]
VyActive = zeros(1,num_point1);%[保留到最后合成]
%launch坐标系下倾角所指方向的单位向量的坐标
thetaXActive = zeros(1,num_point1);%[保留到最后合成]
thetaYActive = zeros(1,num_point1);%[保留到最后合成]
[VActive,~,VxActive,VyActive,thetaXActive,thetaYActive]=calculateVActiveEndByMiukV0(miuk,v0,betaPassiveSegment,settings,environment);

%%发射坐标系坐标
XActiveG = zeros(1,num_point1);%[保留到最后合成]
YActiveG = zeros(1,num_point1);%[保留到最后合成]

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

V_k=VActive(num_point1);%Active Segment终点速度%(1,保留到能量参数计算出来)
X_k=XActiveG(num_point1);%Active Segment终点坐标X轴%(1,保留到Free section开始阶段)
Y_k=YActiveG(num_point1);%Active Segment终点坐标Y轴%(1,保留到Free section开始阶段)

VActive=[];

%%位置坐标转换成84坐标系(大地直角坐标系)
XActiveS = zeros(1,num_point1);%[保留到最后合成]
YActiveS = zeros(1,num_point1);%[保留到最后合成]
ZActiveS = zeros(1,num_point1);%[保留到最后合成]

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

%Active Segment终点在地心坐标系下的坐标
%coordS=G2S(launchBLA,launchLandLatiLong,[X_k,Y_k,0],environment);
coordS=[XActiveS(num_point1),YActiveS(num_point1),ZActiveS(num_point1)];
R_k=sqrt(power(coordS(1),2)+power(coordS(2),2)+power(coordS(3),2));

v_energy=power(V_k,2)*R_k/fM;                                  %能量参数
P_semi_p=R_k*v_energy*power((cos(theta_k)),2);                 %半通径
e_e=sqrt(1+v_energy*(v_energy-2)*power(cos(theta_k),2));       %偏心率e
a_a=P_semi_p/(1-power(e_e,2));                                 %椭圆长半轴a
b_b=P_semi_p/sqrt(1-power(e_e,2));                             %椭圆短半轴b

T_e=2*sqrt(power(a_a,3)/fM)*(acos((1-v_energy)/e_e)+e_e*sin(acos((1-v_energy)/e_e)));%Free section飞行时间
E_k=acos((1-R_k/a_a)/e_e);                                                           %偏近点角Ek%%P203
T_cycle=2*pi*power(a_a,1.5)/sqrt(fM);                                                %质点绕椭圆运行一周的时间
t_p=t_k-T_cycle*(E_k-e_e*sin(E_k))/(2*pi);                                           %missile飞经近地点的时间tp

wz=sqrt(fM/power(P_semi_p,3))*power(1-e_e,2);                                        %输出用于Heavy Decoy航迹生成%warhead绕地心转动角速度

num_point2=floor((T_e+t_k)/delta_t)-num_point1+2; %第二段的采样点数(自由段)
t_2 = zeros(1,num_point2);%[保留到最后合成总的时间]

t_2(1)=t(num_point1);
for i=2:num_point2
    t_2(i)=t_2(1)+(i-1)*delta_t;
end

%任一时刻的偏近点角
E_t0=0;
E_t1=zeros(1,num_point2);%[保留到任一时刻trajectory倾角计算出来]
for i=1:num_point2
    %迭代法解开普勒方程求解任意时刻的偏近点角P200
    %M=E-e*sinE
    E_t0=E_k;
    E_t1(i)=E_t0+((t_2(i)-t_p)*2*pi/T_cycle-E_t0+e_e*sin(E_t0))/(1-e_e*cos(E_t0));
    while abs(E_t1(i)-E_t0)/abs(E_t1(i))>1e-8
        E_t0=E_t1(i);
        E_t1(i)=E_t0+((t_2(i)-t_p)*2*pi/T_cycle-E_t0+e_e*sin(E_t0))/(1-e_e*cos(E_t0));
    end
end

%任一时刻地心距r=a(1-ecosE)
rFree = zeros(1,num_point2);
for i=1:num_point2
    rFree(i)=a_a*(1-e_e*cos(E_t1(i)));
end

%任一时刻的真近点角r=P/(1-ecosf)
fFree = zeros(1,num_point2);%[保留到Reentry section之前，把第一个值和最后一个值付给其他变量，就可以释放了]
for i=1:num_point2
    fFree(i)=acos((P_semi_p/rFree(i)-1)/e_e);
end

%任一时刻速度P203
VFree = zeros(1,num_point2);
for i=1:num_point2
    VFree(i)=sqrt(fM/a_a)*sqrt(1-power(e_e,2)*power(cos(E_t1(i)),2))/(1-e_e*cos(E_t1(i)));
end

%任一时刻trajectory倾角
thetaFree = zeros(1,num_point2);
for i=1:num_point2
    thetaFree(i)=atan(e_e*sin(E_t1(i))/sqrt(1-power(e_e,2)));
end

E_t1=[];

%任一时刻发射坐标系坐标
XFreeG = zeros(1,num_point2);%[保留到X_final计算出来]
YFreeG = zeros(1,num_point2);%[保留到Y_final计算出来]

XFreeG(1)=XActiveG(num_point1);
YFreeG(1)=YActiveG(num_point1);
XActiveG=[];
YActiveG=[];
for i=2:num_point2
    %P247Picture主动段当地水平线倾角变化较小忽略，自由端不能忽略，thetaFree为速度与发射坐标系x轴夹角
    %统一转化到发射坐标系下
    XFreeG(i)=XFreeG(i-1)+VFree(i-1)*cos(thetaFree(i-1)-(fFree(i-1)-fFree(1)))*(t_2(i)-t_2(i-1));
    YFreeG(i)=YFreeG(i-1)+VFree(i-1)*sin(thetaFree(i-1)-(fFree(i-1)-fFree(1)))*(t_2(i)-t_2(i-1));
end

%位置坐标发射坐标系转换成84坐标系（大地直角坐标）
XFreeS = zeros(1,num_point2);%[保留到最后合成]
YFreeS= zeros(1,num_point2);%[保留到最后合成]
ZFreeS = zeros(1,num_point2);%[保留到最后合成]

rTmp = 0;%临时的参数，坐标转换后的地心距r
coordS = zeros(1,3);
for i=1:num_point2-1
    coordS=G2S(launchBLA,launchLandLatiLong,[XFreeG(i+1),YFreeG(i+1),0],environment);
    %坐标转换后任一时刻地心距不变
    %第一个点是自由段开始点
    rTmp=sqrt(power(coordS(1),2)+power(coordS(2),2)+power(coordS(3),2));
    XFreeS(i)=rFree(i+1)/rTmp*coordS(1);
    YFreeS(i)=rFree(i+1)/rTmp*coordS(2);
    ZFreeS(i)=rFree(i+1)/rTmp*coordS(3);
end

%任一时刻水平方向速度和垂直方向速度
VxFree = zeros(1,num_point2);%[保留到最后合成]
VyFree = zeros(1,num_point2);%[保留到最后合成]

for i=1:num_point2%第一个点是主动段终点
    VxFree(i)=VFree(i)*cos(thetaFree(i)-(fFree(i)-fFree(1)));
    VyFree(i)=VFree(i)*sin(thetaFree(i)-(fFree(i)-fFree(1)));
end

%launch坐标系下倾角所指方向的单位向量的坐标
thetaXFree = zeros(1,num_point2);%[保留到最后合成]
thetaYFree = zeros(1,num_point2);%[保留到最后合成]

for i=1:num_point2
    thetaXFree(i)=cos(thetaFree(i)-(fFree(i)-fFree(1)));
    thetaYFree(i)=sin(thetaFree(i)-(fFree(i)-fFree(1)));
end

fFreeEnd=fFree(num_point2);
fFreeStart=fFree(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%/
%%%%%%%%%%Reentry section%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%/
%被动段的飞行时间P205
R_R=environment.R_R;
T_c=sqrt(power(a_a,3)/fM)*(acos((1-v_energy)/e_e)+acos((1-v_energy-(2-v_energy)*(R_k-R_R)/R_k)/e_e)+e_e*sin(acos((1-v_energy)/e_e))+sin(acos((1-v_energy-(2-v_energy)*(R_k-R_R)/R_k)/e_e)));
num_point3= floor((T_c+t_k)/delta_t)-num_point1-num_point2+3;
if rangeBMkm<=2000
    num_point3=round(num_point3*5);
else
    num_point3=round(num_point3*3);%%%%%%/用于改正landing point不在地面的情况%%%%%%%%%%
end

XReentryS = zeros(1,num_point3-1);%[保留到最后合成]
YReentryS = zeros(1,num_point3-1);%[保留到最后合成]
ZReentryS = zeros(1,num_point3-1);%[保留到最后合成]
VXReentryS = zeros(1,num_point3);%[保留到最后合成]
VYReentryS = zeros(1,num_point3);%[保留到最后合成]
VZReentryS = zeros(1,num_point3);%[保留到最后合成]
thetaXReentryS = zeros(1,num_point3);%[保留到最后合成]
thetaYReentryS = zeros(1,num_point3);%[保留到最后合成]
thetaZReentryS = zeros(1,num_point3);%[保留到最后合成]

%再入点地心距，速度，轨道倾角
R_e=rFree(num_point2);
V_e=VFree(num_point2);
theta_e=thetaFree(num_point2);

%再入段地心直角坐标
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
%%%%%%%%%%%最终生成%%%%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

%总数据点(三段之和减去重合的两个点)
%重合的点为自由段第一个和再入段第一个
numTotal=num_point1+num_point2+num_point3-2;
%printf("num_final=%d\n",num_final);

%时间
tTotal = zeros(1,numTotal);%[保留到最后] 总时间

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

%位置(大地直角坐标系)
coordX_84 = zeros(1,numTotal);%[保留到最后] 总坐标x
coordY_84 = zeros(1,numTotal);%[保留到最后] 总坐标y
coordZ_84 = zeros(1,numTotal);%[保留到最后] 总坐标z


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


%速度
VX_84 = zeros(1,numTotal);%[保留到最后] 总速度x
VY_84 = zeros(1,numTotal);%[保留到最后] 总速度y
VZ_84 = zeros(1,numTotal);%[保留到最后] 总速度z

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

%方向(倾角所指方向的单位向量的坐标)
directX_84 = zeros(1,numTotal);%[保留到最后] 总方向x
directY_84 = zeros(1,numTotal);%[保留到最后] 总方向y
directZ_84 = zeros(1,numTotal);%[保留到最后] 总方向z

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
%%%%%%%%%%%文件存储%%%%%%%%%%%%%%%%/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
% 存储航迹数据
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


