function [sys,x0,str,ts] = ANPC_BTB_precharge_m(t,x,u,flag)
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
sizes.NumOutputs     = 78;
sizes.NumInputs      = 15;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [10e-6 0];
%-------------------------------------------------------------------------
global T1; T1=0;                 %%定时器T1
global dltT1; dltT1=1;           %%取1或者-1,决定了T1的增减记数
global fs; fs=2000;             %开关频率
global tp; tp=1/fs;             %一个控制周期
global tss; tss=ts(1);%1e-6;          %仿真步长
global Tmax; Tmax=round(tp/tss/2);%计数器最大值
%-------------------------------------------------------------------------
global cd; cd=4700e-6;          %母线电容
global cf; cf=6600e-6;          %悬浮电容
global Lg; Lg=5e-3;             %整流侧电感
global L1; L1 =  0.0138;        %定子漏感
global L2; L2 =  0.0138;        %转子漏感
global Lm; Lm = 0.299;          %互感
global Ls; Ls = L1 + Lm;        %定子自感
global Lr; Lr = L2 + Lm;        %转子自感
global psi_amp_ref; psi_amp_ref = 220*1.414/100/pi*0.8;%转子磁链额定幅值
global Rs;Rs=2.74;              %定子电阻
global Rr;Rr=2.54;              %转子电阻
global tao_r;tao_r=Lr/Rr;
global sigma;sigma=1-Lm^2/Ls/Lr;
global pole;pole=2;             %极对数
global J_motor; J_motor=0.00072;%转动惯量
global excition_flag; excition_flag=0;

%------------------------------输入量----------------------------------
global is_G; is_G=[0,0,0];            %三相电流
global io_M; io_M=[0,0,0];            %三相电流
global ufc_M;ufc_M=[0,0,0];           %悬浮电容电压
global ufc_G;ufc_G=[0,0,0];           %悬浮电容电压
global udc1; udc1=0;                  %上母线电容电压
global udc2; udc2=0;                  %下母线电容电压
global uab; uab=0;
global ubc;ubc=0;
global speed; speed=0;
%-------------网侧矢量控制使用变量----------------------------------------
%fs=2000Hz, PI参数：5/0.5;5/0.5;0.5/0.01
%fs=500Hz, PI参数：5/0.5;5/0.5;0.2/0.05
global id_G; id_G=0;
global iq_G; iq_G=0;
global usd_G;usd_G=0;
global usq_G;usq_G=0;
global usa;usa=0;
global usb;usb=0;
global usc;usc=0;
global angle_G;angle_G=0;

global UdcRef; UdcRef=600;
global id_ref_G; id_ref_G=0;
global iq_ref_G; iq_ref_G=0;
global ws; ws=2*pi*50;

global IdKp_G;IdKp_G=5;%%id调节器的kp
global IdKi_G;IdKi_G=1;%%id调节器的ki
global IdErrStore_G; IdErrStore_G=0;
global IdPIOutMax_G; IdPIOutMax_G=311;
global IdPIOut_G; IdPIOut_G=0;

global IqKp_G;IqKp_G=5;%%iq调节器的kp
global IqKi_G;IqKi_G=1;%%iq调节器的ki
global IqErrStore_G; IqErrStore_G=0;
global IqPIOutMax_G; IqPIOutMax_G=311;
global IqPIOut_G; IqPIOut_G=0;

global UdcKp_G;UdcKp_G=1;%%udc调节器的kp
global UdcKi_G;UdcKi_G=0.2;%%udc调节器的ki
global UdcErrStore_G; UdcErrStore_G=0;
global UdcPIOut_G; UdcPIOut_G=0;
global UdcPIOutMax_G; UdcPIOutMax_G=10;

global ua_ref_G; ua_ref_G=0;
global ub_ref_G; ub_ref_G=0;
global uc_ref_G; uc_ref_G=0;
global ud_ref_G; ud_ref_G=0;
global uq_ref_G; uq_ref_G=0;
%------------电机侧矢量控制使用变量-----------------------------------
global id_M; id_M=0;
global iq_M; iq_M=0;
global id_ref_M; id_ref_M=0;
global iq_ref_M; iq_ref_M=0;
global speed_ref; speed_ref=100;
global psi_rd;psi_rd=0;
global psi_rd_last;psi_rd_last=0;
global theta_s;theta_s=0;

global IdKp_M;IdKp_M=100;%%id调节器的kp
global IdKi_M;IdKi_M=1000;%%id调节器的ki
global IdErrStore_M; IdErrStore_M=0;
global IdPIOutMax_M; IdPIOutMax_M=311;
global IdPIOut_M; IdPIOut_M=0;

global IqKp_M;IqKp_M=100;%%iq调节器的kp
global IqKi_M;IqKi_M=1000;%%iq调节器的ki
global IqErrStore_M; IqErrStore_M=0;
global IqPIOutMax_M; IqPIOutMax_M=311;
global IqPIOut_M; IqPIOut_M=0;

global SpeedKp_M;SpeedKp_M=5;%%udc调节器的kp
global SpeedKi_M;SpeedKi_M=500;%%udc调节器的ki
global SpeedErrStore_M; SpeedErrStore_M=0;
global SpeedPIOutMax_M; SpeedPIOutMax_M=20;
global SpeedPIOut_M; SpeedPIOut_M=0;

global ua_ref_M; ua_ref_M=0;
global ub_ref_M; ub_ref_M=0;
global uc_ref_M; uc_ref_M=0;
global usd_ref_M; usd_ref_M=0;
global usq_ref_M; usq_ref_M=0;

%-----------------PWM控制使用变量------------------------------------------
global precharge_G; precharge_G=[0,0,0];
global precharge_M; precharge_M=[0,0,0];
global precharge_DC; precharge_DC=0;
global precharge_duty; precharge_duty=0.9;
global bridge_lock_M;bridge_lock_M=[0,0,0,0,0,0];
global bridge_lock_G;bridge_lock_G=[0,0,0,0,0,0];
global short; short = 0;

global PreCurKp;PreCurKp=0.5;%%udc调节器的kp
global PreCurKi;PreCurKi=0.01;%%udc调节器的ki
global PreCurErrStore; PreCurErrStore=0;
global PreCurPIOut; PreCurPIOut=0;
global PreCurPIOutMax; PreCurPIOutMax=0.5;

global usin_G; usin_G=[0,0,0];      %初始三相参考电压
global level_G;level_G=[0,0,0];     %三相参考电压层级
global redundant_G;redundant_G=[0,0,0];%冗余开关状态选择
global CMP_G; CMP_G=zeros(1,18);    %三相18对开关管的载波比较值

global usin_M; usin_M=[0,0,0];      %初始三相参考电压
global level_M;level_M=[0,0,0];     %三相参考电压层级
global redundant_M;redundant_M=[0,0,0];%冗余开关状态选择
global CMP_M; CMP_M=zeros(1,18);    %三相18对开关管的载波比较值

global ufc_ref;ufc_ref=150;     %悬浮电容电压参考值
%------------中点电压平衡使用变量---------------------------
global order; order=[1,1,1];    %排序数组
global Vmin0; Vmin0=0;          %初始三相参考电压最小值
global Vmid0; Vmid0=0;          %初始三相参考电压居中值
global Vmax0; Vmax0=0;          %初始三相参考电压最大值
global Vzmin; Vzmin=0;          %零序电压最小值
global Vzmax; Vzmax=0;          %零序电压最大值
global Vz; Vz=zeros(1,13);      %所有共模电压数组
global Iminn; Iminn=zeros(1,13);%最小参考电压那一相不同零序电压下对应的中点电流
global Imidn; Imidn=zeros(1,13);%居中参考电压那一相不同零序电压下对应的中点电流
global Imaxn; Imaxn=zeros(1,13);%最大参考电压那一相不同零序电压下对应的中点电流
global In; In=zeros(1,13);      %不同零序电压下对应的总中点电流
global iNP_G; iNP_G=0;          %
global In_ref; In_ref=0;        %中点电流参考值
global diff_In; diff_In=zeros(1,13);    %实际中点电流与参考值之差
global Vmin; Vmin=zeros(1,13);  %三相参考电压最小值叠加零序电压之后
global Vmid; Vmid=zeros(1,13);  %三相参考电压居中值叠加零序电压之后
global Vmax; Vmax=zeros(1,13);  %三相参考电压最大值叠加零序电压之后
global num_vz; num_vz=1;        %可选零序电压的个数
global best_vz; best_vz=1;      %最优零序电压的序号
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
    global T1;
    global dltT1;
    global udc1;
    udc1=u(1);
    global udc2;
    udc2=u(2);
    global udc;
    udc=udc1+udc2;
    global ufc_G;
    ufc_G(1:3)=u(3:5);
    global ufc_M;
    ufc_M(1:3)=u(6:8);
    global uab;
    uab=u(9);
    global ubc;
    ubc=u(10);
    global is_G;
    is_G(1:2)=u(11:12);
    is_G(3)=-u(11)-u(12);
    global io_M;
    io_M(1:2)=u(13:14);
    io_M(3)=-u(13)-u(14);
    global speed;
    speed=u(15);
    
    global id_G;
    global iq_G;
    global id_ref_G;
    global iq_ref_G;
    global UdcPIOut_G;
    global IdPIOut_G;
    global IqPIOut_G;
    %%------------------
    global tp;     %一个控制周期
    global Tmax;
    
    global CMP_G;
    global level_G;
    global usin_G;
    
    global CMP_M;
    global level_M;
    global usin_M;
    
    global order;
    global ufc_ref;
    global cd;
    global Vmin0;
    global Vmid0;
    global Vmax0;
    global Vz;
    global num_vz;
    global Vmin; 
    global Vmid; 
    global Vmax;
    global Iminn;
    global Imidn;
    global Imaxn;
    global In;
    global In_ref;
    global diff_In;
    global redundant_M;
    global best_vz;
    global usa;
    global usb;
    global usc;
    global usd_G;
    global usq_G;
    global angle_G;
    
    global ua_ref_G;
    global ub_ref_G;
    global uc_ref_G;
    global redundant_G;
    global ua_ref_M;
    global ub_ref_M;
    global uc_ref_M;
    global id_M;
    global iq_M;
    global id_ref_M;
    global iq_ref_M;
    global psi_rd;
    global SpeedPIOut_M;
    
    global precharge_M;
    global precharge_G;
    global precharge_duty;
    global precharge_DC;
    global UdcRef;
    global short;
    global PreCur;
    global PreCurKp;
    global PreCurKi;
    global PreCurErrStore;
    global PreCurPIOut;
    global PreCurPIOutMax;
    global bridge_lock_M;
    global bridge_lock_G;
    
%     global IdPIOut_M;
%     global IqPIOut_M;
    
 %%----------------------------------------------------------------   
    T1=T1+dltT1;
    if(T1<=0)           %%增计数，保证T1的范围为0-1000;
        dltT1=1;
    end
    if(T1>=Tmax)         %%减计数，保证T1的范围为0-1000;
        dltT1=-1;
    end
%%-----------------------------------------------------------------------
    if (T1==Tmax) 
        if (precharge_DC==0)
            if (udc>=UdcRef*0.8)
                precharge_DC=1;
                short=1;
            end
            
            if (ufc_G(1)>ufc_ref)
                precharge_G(1)=1;
                bridge_lock_G(1:2)=[0,0];
            end
            if (ufc_G(2)>ufc_ref)
                precharge_G(2)=1;
                bridge_lock_G(3:4)=[0,0];
            end
            if (ufc_G(3)>ufc_ref)
                precharge_G(3)=1;
                bridge_lock_G(5:6)=[0,0];
            end
            
            if (precharge_G(1)==0)
                CMP_G(1:6)=[1,1,1,1,1,0];
                bridge_lock_G(1:2)=[1,0];
            end
            if (precharge_G(2)==0)
                CMP_G(7:12)=[1,1,1,1,1,0];
                bridge_lock_G(3:4)=[1,0];
            end
            if (precharge_G(3)==0)
                CMP_G(13:18)=[1,1,1,1,1,0];
                bridge_lock_G(5:6)=[1,0];
            end
            
        else
        %-------------------PWM整流算法-----------------------------------
        bridge_lock_G=[1,1,1,1,1,1];
        vector_ctrl_grid();
        usin_G(1)=ua_ref_G/ufc_ref;
        usin_G(2)=ub_ref_G/ufc_ref;
        usin_G(3)=uc_ref_G/ufc_ref;
        usin_G=zero_seq_injection(usin_G);
        
        level_G(1)=PWM_level(usin_G(1));
        level_G(2)=PWM_level(usin_G(2));
        level_G(3)=PWM_level(usin_G(3));
        %------选择冗余开关状态---
        redundant_G = redundant_calc(-is_G,ufc_G,ufc_ref);
        iNP_G = NP_current(usin_G(1),-is_G(1),redundant_G(1)) + NP_current(usin_G(2),-is_G(2),redundant_G(2)) +NP_current(usin_G(3),-is_G(3),redundant_G(3));
        
        CMP_G(1:6)=switching_state(level_G(1),usin_G(1),usin_G(1),redundant_G(1));
        CMP_G(7:12)=switching_state(level_G(2),usin_G(2),usin_G(2),redundant_G(2));
        CMP_G(13:18)=switching_state(level_G(3),usin_G(3),usin_G(3),redundant_G(3));
        end
        if (precharge_DC==0)
            bridge_lock_M=[0,0,0,0,0,0];
        elseif ((precharge_M(1)==0) || (precharge_M(2)==0) || (precharge_M(3)==0))
            if (ufc_M(1)>ufc_ref)
                precharge_M(1)=1;
            end
            if (ufc_M(2)>ufc_ref)
                precharge_M(2)=1;
            end
            if (ufc_M(3)>ufc_ref)
                precharge_M(3)=1;
            end
            
        
            if precharge_M(1)==0
                PreCur = io_M(1);
                CMP_M(1:6)=[1,1,1,1,1,0];
                CMP_M(7:12)=[1,1,1,1,1,1]*(1-PreCurPIOut);
                CMP_M(13:18)=[1,1,1,1,1,1];
                bridge_lock_M=[1,1,0,1,0,0];
            elseif precharge_M(2)==0
                PreCur = io_M(2);
                CMP_M(1:6)=[1,1,1,1,1,1];
                CMP_M(7:12)=[1,1,1,1,1,0];
                CMP_M(13:18)=[1,1,1,1,1,1]*(1-PreCurPIOut);
                bridge_lock_M=[0,0,1,1,0,1];
            elseif precharge_M(3)==0
                PreCur = io_M(3);
                CMP_M(1:6)=[1,1,1,1,1,1]*(1-PreCurPIOut);
                CMP_M(7:12)=[1,1,1,1,1,1];
                CMP_M(13:18)=[1,1,1,1,1,0];
                bridge_lock_M=[0,1,0,0,1,1];
            end
                err_PreCur = 1-PreCur;
                PreCurErrStore = PreCurErrStore + err_PreCur;
                PreCurPIOut = PreCurKp*err_PreCur + PreCurKi*PreCurErrStore*tp;
                if (PreCurPIOut > PreCurPIOutMax)
                    PreCurPIOut = PreCurPIOutMax;
                    PreCurErrStore = PreCurErrStore - err_PreCur;
                elseif (PreCurPIOut < 0)
                    PreCurPIOut = 0;
                    PreCurErrStore = PreCurErrStore - err_PreCur;
                end
                
        else
        bridge_lock_M=[1,1,1,1,1,1];
        %----------逆变侧矢量控制算法--------------------------------------
        vector_ctrl_motor();
        usin_M(1)=ua_ref_M/ufc_ref;
        usin_M(2)=ub_ref_M/ufc_ref;
        usin_M(3)=uc_ref_M/ufc_ref;
        %------根据各相悬浮电容电压大小以及电流方向选择冗余开关状态-------
        redundant_M = redundant_calc(io_M,ufc_M,ufc_ref);
        %---------根据中点电压高低计算中点电流的给定值----------------
        In_ref=cd*(udc2-udc1)/tp - iNP_G;
        %-----将三相电压排序----------------------------------------
        Vmin0=usin_M(1);
        Vmax0=usin_M(1);
        order=[1,1,1];
        num_vz=1;
        best_vz=1;
        if (Vmin0>=usin_M(2))
            Vmin0=usin_M(2);
            order(1)=2;
        end
        if (Vmin0>=usin_M(3))
            Vmin0=usin_M(3);
            order(1)=3;
        end
        if (Vmax0<usin_M(2))
            Vmax0=usin_M(2);
            order(3)=2;
        end
        if (Vmax0<usin_M(3))
            Vmax0=usin_M(3);
            order(3)=3;
        end
        Vmid0=-Vmin0-Vmax0;
        order(2)=6-order(1)-order(3);
        %-----计算所有的关键零序分量，并根据选取的开关状态计算中点电流-------
        Vzmin=-2-Vmin0;     %计算可选最大和最小零序电压
        Vzmax=2-Vmax0;
        if (Vmid0>0)        %保证串联双管工作在基频
            Vzmin=max(-2-Vmin0,-Vmid0);
            Vzmax=min(-Vmin0,2-Vmax0);
        else
            Vzmin=max(-Vmax0,-2-Vmin0);
            Vzmax=min(2-Vmax0,-Vmid0);
        end
        %-----计算叠加最小零序电压之后的中点电流-------
        Vz(1)=Vzmin;
        Vmin(1)=Vmin0+Vz(1);
        Vmid(1)=Vmid0+Vz(1);
        Vmax(1)=Vmax0+Vz(1);
        Iminn(1)=NP_current(Vmin(1),io_M(order(1)),redundant_M(order(1)));
        Imidn(1)=NP_current(Vmid(1),io_M(order(2)),redundant_M(order(2)));
        Imaxn(1)=NP_current(Vmax(1),io_M(order(3)),redundant_M(order(3)));
        In(1)=Iminn(1)+Imidn(1)+Imaxn(1);
        diff_In(1)=abs(In(1)-In_ref);
        %----------计算叠加不同零序电压之后的中点电流-------------
        for i=2:13
            Vz(i)=Vz(i-1)+min([1+floor(Vmin(i-1))-Vmin(i-1),1+floor(Vmid(i-1))-Vmid(i-1),1+floor(Vmax(i-1))-Vmax(i-1)]);
            if (Vz(i)>Vzmax)
                num_vz=i-1;
                break;
            end
            Vmin(i)=Vmin0+Vz(i);
            Vmid(i)=Vmid0+Vz(i);
            Vmax(i)=Vmax0+Vz(i);
            
            Iminn(i)=NP_current(Vmin(i),io_M(order(1)),redundant_M(order(1)));
            Imidn(i)=NP_current(Vmid(i),io_M(order(2)),redundant_M(order(2)));
            Imaxn(i)=NP_current(Vmax(i),io_M(order(3)),redundant_M(order(3)));
            In(i)=Iminn(i)+Imidn(i)+Imaxn(i);
            diff_In(i)=abs(In(i)-In_ref);
        end
        %--将各个关键零序分量对应的中点电流与给定值相比较，选择最佳零序分量---

        best_vz=1;
           for i=1:num_vz
               if (diff_In(i)<diff_In(best_vz))
                   best_vz=i;
               end   
           end

        %-------根据最佳零序分量发PWM------------------------------------
%以下6行代码为不叠加零序电压的算法，根据需求注释掉或者启用      
%         level_M(1)=PWM_level(usin_M(1));
%         level_M(2)=PWM_level(usin_M(2));
%         level_M(3)=PWM_level(usin_M(3));
%         
%          CMP_M(1:6)=switching_state(level_M(1),usin_M(1),usin_M(1),redundant_M(1));
%          CMP_M(7:12)=switching_state(level_M(2),usin_M(2),usin_M(2),redundant_M(2));
%          CMP_M(13:18)=switching_state(level_M(3),usin_M(3),usin_M(3),redundant_M(3));
         
%%以下6行代码为叠加零序电压的算法，根据需求注释掉或者启用
        level_M(order(1))=PWM_level(Vmin(best_vz));
        level_M(order(2))=PWM_level(Vmid(best_vz));
        level_M(order(3))=PWM_level(Vmax(best_vz));
        
         CMP_M((1:6)+(order(1)-1)*6)=switching_state(level_M(order(1)),Vmin(best_vz),usin_M(order(1)),redundant_M(order(1)));
         CMP_M((1:6)+(order(2)-1)*6)=switching_state(level_M(order(2)),Vmid(best_vz),usin_M(order(2)),redundant_M(order(2)));
         CMP_M((1:6)+(order(3)-1)*6)=switching_state(level_M(order(3)),Vmax(best_vz),usin_M(order(3)),redundant_M(order(3)));
        end
    end
%------------------------------------发PWM-------------------------------
    for i=1:6
        if(T1<CMP_G(i)*(Tmax+1))
            sys(i)=bridge_lock_G(1);sys(i+6)=0;
        else
            sys(i)=0;sys(i+6)=bridge_lock_G(2);
        end
    end
    sys(3)=sys(3) && bridge_lock_G(2);
    sys(4)=sys(4) && bridge_lock_G(2);
    sys(7)=sys(7) && bridge_lock_G(1);
    sys(8)=sys(8) && bridge_lock_G(1);
    
    
    for i=7:12
        if(T1<CMP_G(i)*(Tmax+1))
            sys(i+6)=bridge_lock_G(3);sys(i+12)=0;
        else
            sys(i+6)=0;sys(i+12)=bridge_lock_G(4);
        end
    end
    
    sys(15)=sys(15) && bridge_lock_G(4);
    sys(16)=sys(16) && bridge_lock_G(4);
    sys(19)=sys(19) && bridge_lock_G(3);
    sys(20)=sys(20) && bridge_lock_G(3);
    
    for i=13:18
        if(T1<CMP_G(i)*(Tmax+1))
            sys(i+12)=bridge_lock_G(5);sys(i+18)=0;
        else
            sys(i+12)=0;sys(i+18)=bridge_lock_G(6);
        end
    end
    
     sys(27)=sys(27) && bridge_lock_G(6);
    sys(28)=sys(28) && bridge_lock_G(6);
    sys(31)=sys(31) && bridge_lock_G(5);
    sys(32)=sys(32) && bridge_lock_G(5);
    
    for i=1:6
        if(T1<CMP_M(i)*(Tmax+1))
            sys(i+36)=bridge_lock_M(1);sys(i+42)=0;
        else
            sys(i+36)=0;sys(i+42)=bridge_lock_M(2);
        end
    end
    
    for i=7:12
        if(T1<CMP_M(i)*(Tmax+1))
            sys(i+42)=bridge_lock_M(3);sys(i+48)=0;
        else
            sys(i+42)=0;sys(i+48)=bridge_lock_M(4);
        end
    end
    
    for i=13:18
        if(T1<CMP_M(i)*(Tmax+1))
            sys(i+48)=bridge_lock_M(5);sys(i+54)=0;
        else
            sys(i+48)=0;sys(i+54)=bridge_lock_M(6);
        end
    end
    
    sys(73)=short;
    sys(74)=id_G;
    sys(75)=iq_G;
    sys(76)=usd_G;
    sys(77)=usa;
    sys(78)=angle_G;
% end mdlOutputs
%%----------------------------------------------------------------------
%%----------------------------------------------------------------------
%%----------------------------------------------------------------------
%-------------计算层级函数---------
function level=PWM_level(usin)
    if (usin<=-1)
        level=0;
    elseif (usin>-1 && usin<=0)
        level=1;
    elseif (usin>0 && usin<=1)
        level=2;
    elseif (usin>1)
        level=3;
    end
%------根据各相悬浮电容电压大小以及电流方向选择冗余开关状态，电流以流出桥臂为正方向-------
function redundant=redundant_calc(io,uc,ufc_ref)
    if (((io(1)>0) && (uc(1)>ufc_ref)) || ((io(1)<0) && (uc(1)<ufc_ref)))
        redundant(1)=0;
    else
        redundant(1)=1;
    end
    if (((io(2)>0) && (uc(2)>ufc_ref)) || ((io(2)<0) && (uc(2)<ufc_ref)))
        redundant(2)=0;
    else
        redundant(2)=1;
    end
    if (((io(3)>0) && (uc(3)>ufc_ref)) || ((io(3)<0) && (uc(3)<ufc_ref)))
        redundant(3)=0;
    else
        redundant(3)=1;
    end
%-----------------根据层级和冗余开关状态发PWM脉冲------------------------
function cmp=switching_state(level,usin,usin0,redundant)
  if usin==0 && usin0>0
      level=2;
      usin=0;
  end
      
  if (level==0)
            if (redundant==0)
                cmp(1)=0;
                cmp(2)=0;
                cmp(3)=0;
                cmp(4)=0;
                cmp(5)=0;
                cmp(6)=usin+2;
            else
                cmp(1)=0;
                cmp(2)=0;
                cmp(3)=0;
                cmp(4)=0;
                cmp(5)=usin+2;
                cmp(6)=0;
            end
        elseif (level==1)
            if (redundant==0)
                cmp(1)=0;
                cmp(2)=0;
                cmp(3)=0;
                cmp(4)=0;
                cmp(5)=usin+1;
                cmp(6)=1;
            else
                cmp(1)=0;
                cmp(2)=0;
                cmp(3)=0;
                cmp(4)=0;
                cmp(5)=1;
                cmp(6)=usin+1;
            end
        elseif (level==2)
             if (redundant==0)
                cmp(1)=1;
                cmp(2)=1;
                cmp(3)=1;
                cmp(4)=1;
                cmp(5)=0;
                cmp(6)=usin;
            else
                cmp(1)=1;
                cmp(2)=1;
                cmp(3)=1;
                cmp(4)=1;
                cmp(5)=usin;
                cmp(6)=0;
            end
        elseif (level==3)
            if (redundant==0)
                cmp(1)=1;
                cmp(2)=1;
                cmp(3)=1;
                cmp(4)=1;
                cmp(5)=usin-1;
                cmp(6)=1;
            else
                cmp(1)=1;
                cmp(2)=1;
                cmp(3)=1;
                cmp(4)=1;
                cmp(5)=1;
                cmp(6)=usin-1;
            end
  end
  
%---------------------计算相中点电流--------------------------
function Inp=NP_current(Vph,Iph,redundant)
        if (Vph>=-2 && Vph<-1)
            Inp=(2+Vph)*redundant*Iph;
        elseif (Vph>=-1 && Vph<0)
            if (redundant==0)
                Inp=(1+Vph)*Iph;
            else
                Inp=Iph;
            end
        elseif (Vph>=0 && Vph<1)
            if (redundant==1)
                Inp=(1-Vph)*Iph;
            else
                Inp=Iph;
            end
        elseif (Vph>=1 && Vph<=2)
            Inp=(2-Vph)*(1-redundant)*Iph;
        else
            Inp=0;
        end
%%--------------------派克变换与反变换-------------------------------
function [de,qe]=park_transform(ds,qs,angle)
    de=ds*cos(angle)+qs*sin(angle);
    qe=qs*cos(angle)-ds*sin(angle);
function [ds,qs]=ipark_transform(de,qe,angle)
    ds=de*cos(angle)-qe*sin(angle);
    qs=qe*cos(angle)+de*sin(angle); 
function [ds,qs]=clarke_transform(as,bs,cs)
    ds=2/3*as-1/3*bs-1/3*cs;
    qs=(bs-cs)/1.73205080756887;
function [as,bs,cs]=iclarke_transform(ds,qs)
    as = ds;
    bs=(1.73205080756887*qs-ds)/2;
    cs = -(as + bs); 
%%-----------------------------网侧矢量控制------------------------------
function vector_ctrl_grid()
    global UdcRef;
    global id_ref_G;
    global iq_ref_G;
    global usd_G;usd_G=0;
    global usq_G;
    global usa;
    global usb;
    global usc;
    global angle_G;

    global UdcErrStore_G;
    global IdErrStore_G;
    global IqErrStore_G;
    global IdKp_G;
	global IdKi_G;
	global IqKp_G;
	global IqKi_G;
    global UdcKp_G;
    global UdcKi_G;
    global Lg;
    global UdcPIOutMax_G;
    global IdPIOutMax_G;
    global IqPIOutMax_G;
    global ud_ref_G;
    global uq_ref_G;
    global ws;
    global id_G;
    global iq_G;
    global UdcPIOut_G;
    global IdPIOut_G;
    global IqPIOut_G;
    
    global udc;
    global uab;
    global ubc;
    global is_G;
    global ua_ref_G;
    global ub_ref_G;
    global uc_ref_G;
    
    usa=(2*uab+ubc)/3;
    usb=(ubc-uab)/3;
    usc=-(usa+usb);
    [ualpha_G,ubeta_G]=clarke_transform(usa,usb,usc);
    angle_G=atan2(ubeta_G,ualpha_G);
    [usd_G,usq_G]=park_transform(ualpha_G,ubeta_G,angle_G);
    [ialpha_G,ibeta_G]=clarke_transform(is_G(1),is_G(2),is_G(3));
    [id_G,iq_G]=park_transform(ialpha_G,ibeta_G,angle_G);
    
    UdcErr_G=UdcRef-udc;
    UdcErrStore_G=UdcErrStore_G+UdcErr_G;
    UdcPIOut_G=UdcKp_G*UdcErr_G+UdcKi_G*UdcErrStore_G;
    if (UdcPIOut_G>UdcPIOutMax_G)
        UdcPIOut_G=UdcPIOutMax_G;
        UdcErrStore_G=UdcErrStore_G-UdcErr_G;
    elseif (UdcPIOut_G<-UdcPIOutMax_G)
        UdcPIOut_G=-UdcPIOutMax_G;
        UdcErrStore_G=UdcErrStore_G-UdcErr_G;
    end
            
    id_ref_G=UdcPIOut_G;
    IdErr_G=id_ref_G-id_G;
    IdErrStore_G=IdErrStore_G+IdErr_G;
    IdPIOut_G=IdKp_G*IdErr_G+IdKi_G*IdErrStore_G;
    if (IdPIOut_G>IdPIOutMax_G)
        IdPIOut_G=IdPIOutMax_G;
        IdErrStore_G=IdErrStore_G - IdErr_G;
    elseif (IdPIOut_G<-IdPIOutMax_G)
        IdPIOut_G=-IdPIOutMax_G;
        IdErrStore_G=IdErrStore_G - IdErr_G;
    end
    
    iq_ref_G=10;
    IqErr_G=iq_ref_G-iq_G;
    IqErrStore_G=IqErrStore_G+IqErr_G;
    IqPIOut_G=IqKp_G*IqErr_G+IqKi_G*IqErrStore_G;
    if (IqPIOut_G>IqPIOutMax_G)
        IqPIOut_G=IqPIOutMax_G;
        IqErrStore_G=IqErrStore_G - IqErr_G;
    elseif (IqPIOut_G<-IqPIOutMax_G)
        IqPIOut_G=-IqPIOutMax_G;
        IqErrStore_G=IqErrStore_G - IqErr_G;
    end
    
    ud_ref_G = usd_G - IdPIOut_G + iq_G*ws*Lg*0;
	uq_ref_G = usq_G - IqPIOut_G -id_G*ws*Lg*0 ;
    
    [ualphaRef_G,ubetaRef_G]=ipark_transform(ud_ref_G,uq_ref_G,angle_G);
    [ua_ref_G,ub_ref_G,uc_ref_G]=iclarke_transform(ualphaRef_G,ubetaRef_G);
%-------------磁链计算--------------------------------------
function psi=psi_calc(id,iq,psi_last,method)
    global Lm;
    global tao_r;
    global tp;
    if (method==1)
        psi=(Lm*id+tao_r/tp*psi_last)/(1+tao_r/tp);
    elseif (method==0)
        psi=psi_last;
    else
        psi=psi_last;
    end
%%-----------------------------电机侧矢量控制------------------------------   
function vector_ctrl_motor()
    global SpeedErrStore_M;
    global IdErrStore_M;
    global IqErrStore_M;
    global IdKp_M;
	global IdKi_M;
	global IqKp_M;
	global IqKi_M;
    global SpeedKp_M;
    global SpeedKi_M;

    global Lm;
    global Ls;
    global Lr;
    global Rr;
    global pole;
    global excition_flag;
	global tao_r;
    global sigma;
    
    global SpeedPIOutMax_M;
    global IdPIOutMax_M;
    global IqPIOutMax_M;
    global speed_ref
    global usd_ref_M;
    global usq_ref_M;
    global id_M;
    global iq_M;
    global id_ref_M;
    global iq_ref_M;
    global SpeedPIOut_M;
    global IdPIOut_M;
    global IqPIOut_M;

    global io_M;
    global ua_ref_M;
    global ub_ref_M;
    global uc_ref_M;
    
    global psi_rd_last;
    global psi_rd;
    global psi_amp_ref;
    global speed;
    global omega_r;
    global omega_s;
    global omega_sl;
    global theta_s;
    global tp;
%     global tao_r;
%     global sigma;
    
    if (psi_rd<0.1)
        omega_sl = 0;
        psi_rd = 0.1;
    else
        omega_sl = Lm/tao_r*iq_M/psi_rd;
    end
    
    omega_r = speed*pole;
    omega_s = omega_r + omega_sl;
    theta_s = theta_s + omega_s*tp;

    if (theta_s>2*pi)
        theta_s = theta_s - 2*pi;
    elseif (theta_s<0)
        theta_s = theta_s + 2*pi;
    end
    
    [i_alpha_M,i_beta_M]=clarke_transform(io_M(1),io_M(2),io_M(3));
    [id_M,iq_M]=park_transform(i_alpha_M,i_beta_M,theta_s);
%     psi_rd=(Lm*id_M+tao_r/tp*psi_rd_last)/(1+tao_r/tp);
    psi_rd=psi_calc(id_M,iq_M,psi_rd,1);
    
    err_speed = speed_ref-speed;
    SpeedErrStore_M = SpeedErrStore_M + err_speed;
    SpeedPIOut_M = SpeedKp_M*err_speed + SpeedKi_M*SpeedErrStore_M*tp;
    if (SpeedPIOut_M > SpeedPIOutMax_M)
        SpeedPIOut_M = SpeedPIOutMax_M;
        SpeedErrStore_M = SpeedErrStore_M - err_speed;
    elseif (SpeedPIOut_M < -SpeedPIOutMax_M)
        SpeedPIOut_M = -SpeedPIOutMax_M;
        SpeedErrStore_M = SpeedErrStore_M - err_speed;
    end
    
%     psi_rd_ref = 1;
    if (psi_rd<psi_amp_ref*0.8)&&(excition_flag==0)
        iq_ref_M = 0;
    else
        excition_flag=1;
        iq_ref_M = SpeedPIOut_M*Lr/(Lm*pole*psi_rd);
    end
    
    if (iq_ref_M>=10)
        iq_ref_M = 10;
    end
    err_iq_M = iq_ref_M - iq_M;
    IqErrStore_M = IqErrStore_M + err_iq_M;
    IqPIOut_M = IqKp_M*err_iq_M + IqKi_M*IqErrStore_M*tp;
    if (IqPIOut_M > IqPIOutMax_M)
        IqPIOut_M = IqPIOutMax_M;
        IqErrStore_M = IqErrStore_M - err_iq_M;
    elseif (IqPIOut_M<-IqPIOutMax_M)
        IqPIOut_M = -IqPIOutMax_M;
        IqErrStore_M = IqErrStore_M - err_iq_M;
    end
    
    id_ref_M = psi_amp_ref/Lm;
    err_id_M = id_ref_M - id_M;
    IdErrStore_M = IdErrStore_M + err_id_M;
    IdPIOut_M = IdKp_M*err_id_M + IdKi_M*IdErrStore_M*tp;
    if (IdPIOut_M > IdPIOutMax_M)
        IdPIOut_M = IdPIOutMax_M;
        IdErrStore_M = IdErrStore_M - err_id_M;
    elseif (IdPIOut_M < -IdPIOutMax_M)
        IdPIOut_M = -IdPIOutMax_M;
        IdErrStore_M = IdErrStore_M - err_id_M;
    end
    
    usd_c = Lm/Lr*(psi_rd-psi_rd_last)/tp - omega_s*sigma*Ls*iq_M;
    usq_c = omega_s*(sigma*Ls*id_M + Lm/Lr*psi_rd);
    usd_ref_M = IdPIOut_M + usd_c*1;
    usq_ref_M = IqPIOut_M + usq_c*1;
    psi_rd_last=psi_rd;
    
    [us_alpha_ref_M,us_beta_ref_M]=ipark_transform(usd_ref_M,usq_ref_M,theta_s);
    [ua_ref_M,ub_ref_M,uc_ref_M]=iclarke_transform(us_alpha_ref_M,us_beta_ref_M);


    
%-------------零序电压注入-----------------------------------   
function usin = zero_seq_injection(usin0)
        Vmax0=max(usin0);
        Vmin0=min(usin0);
        Vmid0=sum(usin0)-Vmax0-Vmin0;
        Vz=0;
%         if (Vmax0>=2)
%             Vz=2-Vmax0;
%         elseif (Vmin0<-2)
%                 Vz=-2-Vmin0;
%         end
            Vz=Vmid0/2;
        usin=usin0+Vz;
%%------------------------------------------------------------------
    