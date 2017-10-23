function [sys,x0,str,ts] = Copy_of_Five_Level_m_FS(t,x,u,flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

%%------------------------------------------------------------------------

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 30;
sizes.NumInputs      = 14;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [1e-6 0];

global fs; fs=1000;             %开关频率
global tp; tp=1/fs;             %一个控制周期
global tss; tss=ts(1);%1e-6;          %仿真步长
global Tmax; Tmax=round(tp/tss/2);%计数器最大值

global T1;%%定时器T1
T1=Tmax;
global dltT1;%%取1或者-1,决定了T1的增减记数
dltT1=-1;

global T2;%%定时器T2
T2=Tmax/2;
global dltT2;%%取1或者-1,决定了T1的增减记数
dltT2=1;

global T3;%%定时器T3
T3=1;
global dltT3;%%取1或者-1,决定了T1的增减记数
dltT3=-1;

global T4;%%定时器T4
T4=Tmax/2;
global dltT4;%%取1或者-1,决定了T1的增减记数
dltT4=-1;

global CMP; CMP=zeros(1,12);
global flux_theta;flux_theta=0;

global cd;cd=1000e-6;
global udc; udc=[0 0 0];
global ucfa;ucfa=[0,0];
global ucfb;ucfb=[0,0];
global ucfc;ucfc=[0,0];
global io; io=[0,0,0];
global uc_ref;uc_ref=2400;
global uc_band;uc_band=5;

global level;level=[0,0,0];
global usin; usin=[0,0,0];
global vz; vz=0;

global Vmin0; Vmin0=0;
global Vmid0; Vmid0=0;
global Vmax0; Vmax0=0;
global Vzmin; Vzmin=0;
global Vzmax; Vzmax=0;
global Vz; Vz=zeros(1,13);
global Iminn; Iminn=zeros(2,13);
global Imidn; Imidn=zeros(2,13);
global Imaxn; Imaxn=zeros(2,13);
global In; In=zeros(2,13);
global In_ref; In_ref=0;
global diff_In; diff_In=zeros(1,13);

global Vmin; Vmin=zeros(1,13);
global Vmid; Vmid=zeros(1,13);
global Vmax; Vmax=zeros(1,13);
global uou; uou=zeros(1,13);
global uov; uov=zeros(1,13);
global uow; uow=zeros(1,13);
global num_vz; num_vz=1;
global best_vz; best_vz=1;
%%----------------------------------------------------------------------

function sys=mdlDerivatives(t,x,u)
sys = [];
function sys=mdlUpdate(t,x,u)
sys = [];
function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1; 
sys = t + sampleTime;
function sys=mdlTerminate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
    global Tmax;
    global T1;
    global dltT1;
    T1=T1+dltT1;
    if(T1<=0)%%保证T1的范围为0-Tmax;
        dltT1=-dltT1;
    end
    if(T1>=Tmax)%%保证T1的范围为0-Tmax;
        dltT1=-dltT1;
    end
    
    global T2;
    global dltT2;
    T2=T2+dltT2;
    if(T2<=0)%%保证T1的范围为0-Tmax;
        dltT2=-dltT2;
    end
    if(T2>=Tmax)%%保证T1的范围为0-Tmax;
        dltT2=-dltT2;
    end
    
    global T3;
    global dltT3;
    T3=T3+dltT3;
    if(T3<=0)%%保证T1的范围为0-Tmax;
        dltT3=-dltT3;
    end
    if(T3>=Tmax)%%保证T1的范围为0-Tmax;
        dltT3=-dltT3;
    end
    
    global T4;
    global dltT4;
    T4=T4+dltT4;
    if(T4<=0)%%保证T1的范围为0-Tmax;
        dltT4=-dltT4;
    end
    if(T4>=Tmax)%%保证T1的范围为0-Tmax;
        dltT4=-dltT4;
    end    
 %%-----------------------------------------------------------------
    global udc;
    udc=u(1:3);
    global ucfa;
    ucfa=u(4:5);
    global ucfb;
    ucfb=u(6:7);
    global ucfc;
    ucfc=u(8:9);
    global io;
    io(1:2)=u(10:11);
    io(3)=-io(1)-io(2);
    
    global uc_ref;
    uc_ref=(udc(1)+udc(2)+udc(3))/4;
    
    global tp;
    global cd;
    global CMP;
    global flux_theta;
    global usin;
    global vz;
    

    if (T1==Tmax) 
        %----------计算角度以及三相电压--------------------
        flux_theta = flux_theta + 40*2*pi*tp;      
        if (flux_theta> 2*pi)
          flux_theta = flux_theta - 2*pi;
        elseif (flux_theta< 0)
          flux_theta = flux_theta + 2*pi;
        end
%         usin(1)=0.9*sin(flux_theta)*2+2;  %调制比m=0.9
%         usin(2)=0.9*sin(flux_theta-2*pi/3)*2+2;
%         usin(3)=0.9*sin(flux_theta+2*pi/3)*2+2;
        usin(1)=u(12)/uc_ref;
        usin(2)=u(13)/uc_ref;
        usin(3)=u(14)/uc_ref;
        In_ref=cd*(udc(3)-udc(1))/tp;
%         vz=0;
        vz=zero_sequence_voltage(usin,io,In_ref);
        CMP(1:4) = [usin(1)/4, usin(1)/4, usin(1)/4, usin(1)/4]+vz/4;
        CMP(5:8) = [usin(2)/4, usin(2)/4, usin(2)/4, usin(2)/4]+vz/4;
        CMP(9:12)= [usin(3)/4, usin(3)/4, usin(3)/4, usin(3)/4]+vz/4;
        DCMP=zeros(1,12);
        DCMP(1:4) = FC_balancing([udc(2),ucfa(1),ucfa(2)],[uc_ref*2, uc_ref*2, uc_ref],io(1),CMP(1));
        DCMP(5:8) = FC_balancing([udc(2),ucfb(1),ucfb(2)],[uc_ref*2, uc_ref*2, uc_ref],io(2),CMP(5));
        DCMP(9:12) = FC_balancing([udc(2),ucfc(1),ucfc(2)],[uc_ref*2, uc_ref*2, uc_ref],io(3),CMP(9));
       CMP=CMP+DCMP;
%        In_ref=cd*(udc(3)-udc(1))/tp;
%        vz=zero_sequence_voltage(usin,io,In_ref);
%         CMP(1:4) = [usin(1)/4, usin(1)/4, usin(1)/4, usin(1)/4]+vz/4;
%         CMP(5:8) = [usin(2)/4, usin(2)/4, usin(2)/4, usin(2)/4]+vz/4;
%         CMP(9:12)= [usin(3)/4, usin(3)/4, usin(3)/4, usin(3)/4]+vz/4;
%        
    end
    
%      for i=0:2
%         if(T1<CMP(i*4+1)*(Tmax+1))
%             sys(i*10+1)=1;sys(i*10+6)=0;
%             sys(i*10+2)=1;sys(i*10+7)=0;
%         else
%             sys(i*10+1)=0;sys(i*10+6)=1;
%             sys(i*10+2)=0;sys(i*10+7)=1;
%         end
%         
%         if(T3<CMP(i*4+2)*(Tmax+1))
%             sys(i*10+3)=1;sys(i*10+8)=0;
%         else
%             sys(i*10+3)=0;sys(i*10+8)=1;
%         end
%         
%         if(T2<CMP(i*4+3)*(Tmax+1))
%             sys(i*10+4)=1;sys(i*10+9)=0;
%         else
%             sys(i*10+4)=0;sys(i*10+9)=1;
%         end        
%         
%         if(T4<CMP(i*4+4)*(Tmax+1))
%             sys(i*10+5)=1;sys(i*10+10)=0;
%         else
%             sys(i*10+5)=0;sys(i*10+10)=1;
%         end        
%     end
     for i=0:2
        if((T1<round(CMP(i*4+1)*(Tmax+0)))&&dltT1<0)||((T1<=round(CMP(i*4+1)*(Tmax+0)))&&dltT1>0)
            sys(i*10+1)=1;sys(i*10+6)=0;
            sys(i*10+2)=1;sys(i*10+7)=0;
        else
            sys(i*10+1)=0;sys(i*10+6)=1;
            sys(i*10+2)=0;sys(i*10+7)=1;
        end
        
        if((T2<round(CMP(i*4+2)*(Tmax+0)))&&dltT2<0)||((T2<=round(CMP(i*4+2)*(Tmax+0)))&&dltT2>0)
            sys(i*10+3)=1;sys(i*10+8)=0;
        else
            sys(i*10+3)=0;sys(i*10+8)=1;
        end
        
        if((T3<round(CMP(i*4+3)*(Tmax+0)))&&dltT3<0)||((T3<=round(CMP(i*4+3)*(Tmax+0)))&&dltT3>0)
            sys(i*10+4)=1;sys(i*10+9)=0;
        else
            sys(i*10+4)=0;sys(i*10+9)=1;
        end     
        
        if((T3<round(CMP(i*4+4)*(Tmax+0)))&&dltT3<0)||((T3<=round(CMP(i*4+4)*(Tmax+0)))&&dltT3>0)
            sys(i*10+5)=1;sys(i*10+10)=0;
        else
            sys(i*10+5)=0;sys(i*10+10)=1;
        end        
    end
      
%     sys(31)=0;
%     sys(32)=0;
%     sys(33)=0;
%     sys(34)=0;
% end mdlOutputs

function Inp=NP_current(Vph,Iph)
    if (Vph<=1)||(Vph>=3)
        Inp=(1-abs(Vph-2)/2)*Iph;
    else
        Inp=Iph/2;
    end
    
function Vz_best=zero_sequence_voltage(usin,io,In_ref)
%%%%%%%%%--------排序------
    Vmin0=usin(1);
    Vmax0=usin(1);
    order=[1,1,1];
    num_vz=1;
    best_vz=1;
    if (Vmin0>usin(2))
        Vmin0=usin(2);
        order(1)=2;
    end
    if (Vmin0>usin(3))
        Vmin0=usin(3);
        order(1)=3;
    end
    if (Vmax0<usin(2))
        Vmax0=usin(2);
        order(3)=2;
    end
    if (Vmax0<usin(3))
        Vmax0=usin(3);
        order(3)=3;
    end
    order(2)=6-order(1)-order(3);
    Vmid0=usin(order(2));
 %%%%%%%%---计算所有的关键零序分量，并根据选取的开关状态计算中点电流
    Vzmin=-Vmin0;
    Vzmax=4-Vmax0;

    Vz(1)=Vzmin;
    Vmin(1)=Vmin0+Vz(1);
    Vmid(1)=Vmid0+Vz(1);
    Vmax(1)=Vmax0+Vz(1);
    Iminn(1)=NP_current(Vmin(1),io(order(1)));
    Imidn(1)=NP_current(Vmid(1),io(order(2)));
    Imaxn(1)=NP_current(Vmax(1),io(order(3)));
    In(1)=Iminn(1)+Imidn(1)+Imaxn(1);
    diff_In(1)=(In(1)-In_ref);
    
    for i=2:13
        Vz(i)=Vz(i-1)+min([1+floor(Vmin(i-1))-Vmin(i-1),1+floor(Vmid(i-1))-Vmid(i-1),1+floor(Vmax(i-1))-Vmax(i-1)]);
        if (Vz(i)>(Vzmax+0.00001))
            num_vz=i-1;
            break;
        end
        Vmin(i)=Vmin0+Vz(i);
        Vmid(i)=Vmid0+Vz(i);
        Vmax(i)=Vmax0+Vz(i);
%         uou(i)= floor(usin(1)+Vz(i));
%         uov(i)= floor(usin(2)+Vz(i));
%         uow(i)= floor(usin(3)+Vz(i));
        Iminn(i)=NP_current(Vmin(i),io(order(1)));
        Imidn(i)=NP_current(Vmid(i),io(order(2)));
        Imaxn(i)=NP_current(Vmax(i),io(order(3)));
        In(i)=Iminn(i)+Imidn(i)+Imaxn(i);
        diff_In(i)=(In(i)-In_ref);
    end
    %         --将各个关键零序分量对应的中点电流与给定值相比较，选择最佳零序分量---
    
      
       best_vz=1;
       flag=0;
%        for i=1:num_vz-1
%            if (sign(diff_In(i))*sign(diff_In(i+1))<0)
%                best_vz=i;
%                Vz_best=(In_ref*(Vz(i)-Vz(i+1))-(Vz(i)*In(i+1)-Vz(i+1)*In(i)))/(In(i)-In(i+1));
%                flag=1;
%                break;
%            end   
%        end

          for i=1:num_vz
              if ((floor(Vmax(i))+floor(Vmid(i))+floor(Vmin(i)))>0)&&((floor(Vmax(i))+floor(Vmid(i))+floor(Vmin(i)))<9)  
                  if (i==1)||(best_vz==0)
                    best_vz=i;
                  end
                   if (abs(diff_In(i))<abs(diff_In(best_vz)))
                       best_vz=i;
                   end   
              end
           end
           Vz_best=Vz(best_vz);
%            index=order(flag_int(best_vz));
           
%            for i=1:num_vz
%                if (abs(diff_In(i))<abs(diff_In(best_vz)))
%                    if (4<=(uou(i)+uov(i)+uow(i))) && ((uou(i)+uov(i)+uow(i))<=7)
%                    best_vz=i;
%                    end
%                end   
%            end
%            Vz_best=Vz(best_vz);
           
function DCMP=FC_balancing(ufc,ufc_ref,io,usin)
    DCMP=[0 0 0 0];
    if (usin>0.5)
        delta=(1-usin)*0.2; %0.05
    else
        delta=(0.5-usin)*0.2;
    end
    
%       if (usin>0.5)
%         delta=(1-usin)*0.2;
%     else
%         delta=usin*0.2;
%       end
    
    if (ufc(1) < ufc_ref(1))
        DCMP(1)=DCMP(1)+delta*sign(io);
        DCMP(2)=DCMP(2)-delta/3*sign(io);
        DCMP(3)=DCMP(3)-delta/3*sign(io);
        DCMP(4)=DCMP(4)-delta/3*sign(io);
    else
        DCMP(1)=DCMP(1)-delta*sign(io);
        DCMP(2)=DCMP(2)+delta/3*sign(io);
        DCMP(3)=DCMP(3)+delta/3*sign(io);
        DCMP(4)=DCMP(4)+delta/3*sign(io);
    end
    
    if (ufc(2)< ufc_ref(2))
        DCMP(1)=DCMP(1)+delta*sign(io);
        DCMP(2)=DCMP(2)+delta*sign(io);
        DCMP(3)=DCMP(3)-delta*sign(io);
        DCMP(4)=DCMP(4)-delta*sign(io);
    else
        DCMP(1)=DCMP(1)-delta*sign(io);
        DCMP(2)=DCMP(2)-delta*sign(io);
        DCMP(3)=DCMP(3)+delta*sign(io);
        DCMP(4)=DCMP(4)+delta*sign(io);
    end
        
    if (ufc(3) < ufc_ref(3))
        DCMP(1)=DCMP(1)+delta/3*sign(io);
        DCMP(2)=DCMP(2)+delta/3*sign(io);
        DCMP(3)=DCMP(3)+delta/3*sign(io);
        DCMP(4)=DCMP(4)-delta*sign(io);
    else
        DCMP(1)=DCMP(1)-delta/3*sign(io);
        DCMP(2)=DCMP(2)-delta/3*sign(io);
        DCMP(3)=DCMP(3)-delta/3*sign(io);
        DCMP(4)=DCMP(4)+delta*sign(io);
    end
        
        


