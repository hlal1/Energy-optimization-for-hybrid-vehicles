function [feq] = obj(x,T,g,Cr,y1,m,eff_d)
     acc(1)=0;
     eng_speed=y1*x;
     for i=2:length(x)
         acc(i)=(x(i)-x(i-1))/T;
         if acc(i)<=0
             eng_torque(i)=0;
         else
            eng_torque(i)=m/y1*(acc(i)+g(i)*Cr);
         end
      end     
     
     
     load('enginedata.mat')
    engspeed_rng= 0:(3200/16):3200;
    engtorque_rng= 0:(1100/11):1100;
     for i=1:length(x)
         torque=eng_torque(i);
         speed=eng_speed(i);
%          if torque<=0
%              torque=0;
%          end
%          if speed<=0
%             speed=0;
%          end
%          fcon(i)= interp2(engspeed_rng,engtorque_rng,eng_fuel_map_gpkWh,speed,torque); 
           fcon(i)=2; 
     end
     fcon=fcon';
%      feq=.001*(2*pi/60)*(m/eff_d)*(((x.*fcon)'*x)/2+Cr*(g.*fcon)'*x)*(1/3600);
     feq=.001*(2*pi/60)*(m/eff_d)*(((x.*fcon)'*x)/2+Cr*(g.*fcon)'*x);
end