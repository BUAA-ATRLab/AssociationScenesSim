function [x, y, z, Vx, Vy, Vz] = CT_motion(para)
%--------------仿真参数---------------------
delta_t = para.delta_t;
N = para.Ttotal/delta_t;

x=zeros(1,N);   y=zeros(1,N);   z=zeros(1,N);
Vx=zeros(1,N);  Vy=zeros(1,N);  Vz=zeros(1,N);

x(1) = para.x0_boat;
y(1) = para.y0_boat;
z(1) = para.z0_boat;
Vx(1) = para.V_boat;
Vy(1) = 0;
Vz(1) = 0;

%------------匀速区--------------------------------
N1 = para.T1_boat/delta_t;
for ii = 2:N1
    x(ii) = x(ii-1)+Vx(ii-1)*delta_t;
    y(ii) = y(ii-1)+Vy(ii-1)*delta_t;
    Vx(ii) = Vx(ii-1);
    Vy(ii) = Vy(ii-1);
end

%------------转弯区--------------------------------
N2 = (para.T1_boat + para.T2_boat)/delta_t;
wr = para.wr_boat;
for ii= N1+1:N2
    x(ii) = x(ii-1)+sin(wr*delta_t)/wr*Vx(ii-1)+(cos(wr*delta_t)-1)/wr*Vy(ii-1);
    y(ii) = (1-cos(wr*delta_t))/wr*Vx(ii-1)+y(ii-1)+sin(wr*delta_t)/wr*Vy(ii-1);
    Vx(ii) = cos(wr*delta_t)*Vx(ii-1)-sin(wr*delta_t)*Vy(ii-1);
    Vy(ii) = sin(wr*delta_t)*Vx(ii-1)+cos(wr*delta_t)*Vy(ii-1);
end

%------------匀速区--------------------------------
for ii= N2+1:N
    x(ii) = x(ii-1)+Vx(ii-1)*delta_t;
    y(ii) = y(ii-1)+Vy(ii-1)*delta_t;
    Vx(ii) = Vx(ii-1);
    Vy(ii) = Vy(ii-1);
end
