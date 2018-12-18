clear all;
close all;
clc;
tic
%% Constants.
m = 30000;       % Mass of vehicles.
eff_e = 0.5;    % Engine efficiency.
eff_d = 0.8;    % Driveline efficiency.
Cr = 0.01;      % Coefficient of Rolling resistance.
T = 1;          % Discrete time intervals.
% TI= 300;         % Prediction time zone.
a1 = m/(eff_e*eff_d);
ix=5;           %Final drive ratio
i0=10;          %Gear ratio
rw=0.5;          %radius of wheel
y1=i0*ix/rw;
% b1 = g*Cr;

%% Drive Cycle
dr_cyc = xlsread('DriveCycles_Scaled',11,'A4:B400');
v1 = 0.44704*dr_cyc(:,2);
t = dr_cyc(:,1);
s = length(t);
TI= s;         % Prediction time zone.

%% 
g = 9.81*ones(s,1);       % Gravitational constant.
v2 = zeros(s,1);
a2 = zeros(s,1);
% E  = zeros(s,1);
p1 = zeros(s,1);
p2 = zeros(s,1);
vfinal = zeros(s,1);
p1(1) = 50;
p2(1) = 0;
E1total=0;
E2total=0;

A = [];
b = [];
Aeq = [];
beq = [];
ub=[];
opts=optimoptions('fmincon','Algorithm','sqp');
%% Optimization loop.
j=1;
 while j<s
    if s-j<TI
         TI=s-j;
    end
    x0 = v2(j:j+TI);
    lb=0.00001*ones(TI+1,1);
    [v2_new,E,exitflag]=fmincon(@(x) obj(x,T,g(j:j+TI),Cr,y1,m,eff_d),x0,A,b,Aeq,beq,lb,ub,@(x) constr(x,v1(j:j+TI),T,s,p1(j),p2(j),a2(j),vfinal(j)),opts);
     vfinal(j:j+TI,1)=v2_new;
     E2total=E+E2total;
     for i=j+1:j+TI
         p2(i) =  p2(i-1)+vfinal(i)*T;
         p1(i) =  p1(i-1)+v1(i)*T;
         a2(i) =  (vfinal(i) - vfinal(i-1))/T;
     end
     j=j+TI;
 end
    for i=2:s
    a11(i) = (v1(i) - v1(i-1))/T;       %Accleration of truck 1
    if a11(i)<=0
        eng_torque1(i)=0;
    else
        eng_torque1(i)=(m/y1)*(a11(i)+g(i)*Cr); %enigne torque for truck1 
    end
    eng_speed1(i)=y1*v1(i); % engine speed for truck1
    if a2(i)<=0 
        eng_torque(i)=0;
    else
        eng_torque(i)=(m/y1)*(a2(i)+g(i)*Cr);  %engine torque for truck 2
    end
    eng_speed(i)=y1*vfinal(i); %enigne speed for truck 2
    end   

%% Energy function.
load('enginedata.mat')
engspeed_rng= 0:(3200/16):3200;
engtorque_rng= 0:(1100/11):1100;
for i=1:s
         torque1=eng_torque1(i);
         speed1=eng_speed1(i);
         fcon1(i)= interp2(engspeed_rng,engtorque_rng,eng_fuel_map_gpkWh,speed1,torque1);  
end
fcon1=fcon1';
E1total=.001*(2*pi/60)*(m/eff_d)*(((v1.*fcon1)'*v1)/2+Cr*(g.*fcon1)'*v1)*(1/3600);
% E1total=(m/(eff_d*0.5))*((v1'*v1)/2 + Cr*g'*v1)*T;
%% Energy saved (%)
Esaved=100*((E1total-E2total)/E1total);
%% Plots.
plot(t,v1);
hold on;
plot(t,vfinal);
title('Velocity');
legend("v1","v2")

figure;
plot(t,p1);
hold on;
plot(t,p2);
title('position');
legend("P1","P2")

figure;
plot(t,a11);
hold on;
plot(t,a2);
title('acceleration');
legend("acc1","acc2")

figure;
plot(t,p1-p2);
title('postion difference');

figure;
plot(t,eng_torque1);
title('Engine Torque');

figure;
plot(t,eng_speed);
title('Engine speed');

figure;
Efinal=[E1total,E2total];
bar(Efinal);
legend("Energy Truck1 and Truck2");
title('Energy Consumption');
toc

