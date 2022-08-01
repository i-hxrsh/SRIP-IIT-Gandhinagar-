clc
tic
clear
close all
%% Initializing Variables
Rate_pt1_fast = [];
Rate_pt1_slow = [];
Rate_wash_fast = [];
Rate_wash_slow  = [];
Rate_pt2_fast = [];
Rate_pt2_slow = [];
%% Define plot style
clear s;
s.units = 'inches';
s.format = 'pdf';
s.Preview= 'none';
s.Width= '8'; % Figure width on canvas
s.Height= '5'; % % Figure height on canvas
s.Units= 'inches';
s.Color= 'rgb';
s.Background= 'w';
s.FixedfontSize= '12';
s.ScaledfontSize= 'auto';
s.FontMode= 'scaled';
s.FontSizeMin= '12';
s.FixedLineWidth= '1';
s.ScaledLineWidth= 'auto';
s.LineMode= 'none';
s.LineWidthMin= '0.1';
s.FontName= 'Times New Roman';% Might change this to something that is available
s.FontWeight= 'auto';
s.FontAngle= 'auto';
s.FontEncoding= 'latin1';
s.PSLevel= '3';
s.Renderer= 'painters';
s.Resolution= '300';
s.LineStyleMap= 'none';
s.ApplyStyle= '0';
s.Bounds= 'tight';
s.LockAxes= 'off';
s.LockAxesTicks= 'off';
s.ShowUI= 'off';
s.SeparateText= 'off';
% Skill learning vs adaptation
%% Initialization
num_trials=1;
ts=0.01;% Sampling time interval
ZZ=1000;
paradigm_trials=ZZ*num_trials;% Total number of trials in the paradigm
time=1;
duration=time;
export_plots_to_png=0;
export_plots_to_pdf=0;
numb_interval=400;
%% System definition
m1=0.2;% mass of link 1
m2=0.2;% mass of link 2
lc1=0.1;% midlength of link 1
lc2=0.1;% midlength of link 2
I1=1/3*m1*(2*lc1); % M. I. of link 1
I2=1/3*m2*(2*lc2); % M. I. of link 2
Q1=pi/4; % Equilibrium point of q1
Q2=-pi/2; % Equilibrium point of q2
d1=0.3;% Damping 1
d2=0.3;% Damping 2
% Inverse of inertia matrix
%   ||                 ||
%   \/                 \/
Dinv =[(m2*lc2^2 + I2)/(I1*I2 + 4*lc1*lc2^2*m2^2 + 4*I2*lc1*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - 4*lc1^2*lc2^2*m2^2*cos(Q2)^2),                           -(m2*lc2^2 + 2*lc1*m2*cos(Q2)*lc2 + I2)/(I1*I2 + 4*lc1*lc2^2*m2^2 + 4*I2*lc1*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - 4*lc1^2*lc2^2*m2^2*cos(Q2)^2);
    -(m2*lc2^2 + 2*lc1*m2*cos(Q2)*lc2 + I2)/(I1*I2 + 4*lc1*lc2^2*m2^2 + 4*I2*lc1*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - 4*lc1^2*lc2^2*m2^2*cos(Q2)^2), (m1*lc1^2 + 4*m2*cos(Q2)*lc1*lc2 + 4*m2*lc1 + m2*lc2^2 + I1 + I2)/(I1*I2 + 4*lc1*lc2^2*m2^2 + 4*I2*lc1*m2 + I2*lc1^2*m1 + I1*lc2^2*m2 + lc1^2*lc2^2*m1*m2 - 4*lc1^2*lc2^2*m2^2*cos(Q2)^2)];
a11=zeros(2,2);
a12=eye(2);
a21=a11;
a22=-Dinv*diag([d1,d2]);
b11=a11;
b12=Dinv*eye(2);
%% LAD: System based on 2R
Ac=[a11 a12;a21 a22]; % Continuous-time A matrix
Bc=[b11;b12]; % Continuous-time B matrix
A=0.5*(eye(size(Ac,1))+Ac*ts); % Discrete-time A matrix
B=Bc; % Discrete-time B matrix
C=[1 0 0 0;0 1 0 0];
[~,n]=size(A);
[l,m]=size(C*B);
%% Controller Equations Parameterization
i_index=0;
% for i=0.1:0.2:0.9
i_index=i_index+1;
lambda_ILC= 1;% Determination factor (Can you change this by explicit instructions?)
lambda_model=0.7;
lambda_FF=0.7;% Confidence on the internal learnt model
gamma_ILC=0.1;% Initial learning rate
gamma_model= 0.1; %Initial learning rate
gamma_FF=0.1; %|| Range: 0.05:0.1
nx=0.0;% Noise levels
ny=0.0;% Noise levels
%% Direction of reaching task
xi=0;
yi=0;
xf=0;
xfx=0;
yf=0.5;
%% Perturbation Definition
enableVMR=0;
th=0.5236; % 30 degree rotation
enableFFPT=1;
enableVLPT=0;
beta=-0.1;% FF Perturbation level
alpha=15;% VL Perturbation level
%% Trajectory Generation
% Reference Commands Generation (Minimum Jerk Trajectory, per R. Shadhmehr and
% S. P. Wise) code based on Supplementary documents for "Computational Neurobiology
% of Reaching and Pointing",
t=0;
yd1=[xf*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
ydx=[xfx*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
yd=0*ydx;
for loop_time=0:ts:time
    t=t+1;
    yd1(:,t)=[xi+(xf-xi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5);
        yi+(yf-yi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5)];
end
t=0;
for loop_time=0:ts:time
    t=t+1;
    ydx(:,t)=[xi+(xfx-xi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5);
        yi+(yf-yi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5)];
end
Tei=zeros(1,8);
Tef=zeros(1,8);
Moei=zeros(1,8);
Moef=zeros(1,8);
Spei=zeros(1,8);
Spef=zeros(1,8);
Tei_dash=zeros(1,8);
Tef_dash=zeros(1,8);
Moei_dash=zeros(1,8);
Moef_dash=zeros(1,8);
Spei_dash=zeros(1,8);
Spef_dash=zeros(1,8);
count=0;
numb_array=[2,5,10,20,50,100,200,300];
for numb_interval=[2,5,10,20,50,100,200,300]
flag=0;
flag_dash=0;
count=count+1;
%% Init. variables
norme=0;
plot_surf=0;
y_=zeros(size(yd));
uILC_=y_;
ymodel_=y_;
ymodel=ymodel_;
Y=ymodel(:,1:end);
Y_=Y;
uFF_=uILC_;
uFF=uFF_;
uILC=uFF;
u=uFF;
u_=u;
VMR=y_;
FFPT=Y_;
VLPT=0;
model_term_=uILC_;
yFF=y_;
yILC=y_;
TE=y_;
SPE=y_;
MOE=y_;
E=y_;
E_=zeros(1,paradigm_trials);
YendP=y_;
perception=VMR;
normy=zeros(1,length(loop_time));
normFF=zeros(1,length(loop_time));
normuILC=zeros(1,length(loop_time));
normX=zeros(1,length(loop_time));
normMru=zeros(1,length(loop_time));
normTE=normy;
normSPE=normy;
normMOE=normy;
normu=normy;
normuFF=normy;
normMruFF=normy;
normMruILC=normy;
IT=normy;
MT=normy;
RR=[cos(th) -sin(th);sin(th) cos(th)];
%% Across-the-Trials for loop
% fg1=figure(1);
% fg2=figure(2);
% fg3=figure(3);
% fg6=figure(6);
for zz=0:ZZ
    for k=(zz*num_trials+1):1:((zz+1)*(num_trials))
        x=zeros(n,length(yd));
        xILC=zeros(n,length(yd));
        xFF=xILC;
        y=C*x;
        %% Across-the-Time for loop
        t=0;
        for loop_time=0:ts:time-1*ts
            t=t+1;
            %% Perturbation Compututations
            
            if (rem(zz,2*numb_interval)<numb_interval) %(PERT PT)
                yd=ydx;
                FFPT=[beta;0];
            end
            if (rem(zz,2*numb_interval)>=numb_interval) %(PERT PT)
                yd=ydx;
                FFPT=[0;0];
            end
            %         end
            %         if (k>=num_trials*6+1)&&(k<=num_trials*7) %(PERT PT again)
            %             yd=ydx;
            %             if enableVMR==1
            %                 RR=[cos(th) -sin(th);sin(th) cos(th)];
            %             else
            %                 VMR=[0;0];
            %                 RR=eye(2);
            %             end
            %             if enableFFPT==1
            %                 FFPT=[beta;0];
            %             else
            %                 FFPT=[0;0];
            %             end
            %             if enableVLPT==1
            %                 VLPT=alpha;
            %             else
            %                 VLPT=0;
            %             end
            %         end
            if (zz<=(200)) %(Baseline)
                yd=ydx;
                FFPT=[0;0];
            end
            TE(:,t+2)=yd(:,t+2)-(RR*y_(:,t+2));
            MOE(:,t+2)=C*A*B*u_(:,t)-ymodel_(:,t+2);
            SPE(:,t+2)=(RR)*y_(:,t+2)-ymodel_(:,t+2);
            %% Error Definitions
            if (log10(norm(TE(:,t+2)))>=-10)
                %% Learning Controller Model
                uILC(:,t)=lambda_ILC*uILC_(:,t)+gamma_ILC*tanh(TE(:,t+2));
                ymodel(:,t+2)=lambda_model*ymodel_(:,t+2)+gamma_model*tanh(SPE(:,t+2));
                %             ymodel(:,t+2)=C*A*B*uILC_(:,t);%+gamma_model*tanh(SPE(:,t+2));
                uFF(:,t)=lambda_FF*uFF_(:,t)+gamma_FF*pinv(C*A*B)*tanh((MOE(:,t+2)));
                u(:,t)=1*uILC(:,t)+1*uFF(:,t);
            else
                u(:,t)=u_(:,t);
            end
            %% State-space System
            x(:,t+1)=A*x(:,t)+nx*rand(n,1)+B*(u(:,t)+pinv(C*A*B)*FFPT);
            y(:,t)=C*x(:,t)+ny*rand(l,1);
        end
        %% Save data for next iteration
        uILC_=uILC;% ILC data
        y_=y;% output data
        ymodel_=ymodel;% model data
        Y_=Y;
        uFF_=uFF;
        u_=u;
        normTE(1,k)=norm(TE(:,1:(time/ts)));
        normSPE(1,k)=norm(SPE(:,1:(time/ts)));
        normMOE(1,k)=norm(MOE(:,1:(time/ts)));
        
        if (k>200 && rem(k,2*numb_interval)==numb_interval &&flag==0)
            Tei(count)=normTE(1,k);
            Spei(count)=normSPE(1,k);
            Moei(count)=normMOE(1,k);
            flag=flag+1;
%         normTE(1,k)
%         normSPE(1,k)
%         normMOE(1,k)
        end
        if (k>200 && rem(k,2*numb_interval)==2*numb_interval-1 &&flag==1)
            Tef(count)=normTE(1,k);
            Spef(count)=normSPE(1,k);
            Moef(count)=normMOE(1,k);
            flag=flag+1;
%         normTE(1,k)
%         normSPE(1,k)
%         normMOE(1,k)
        end
        if (k>200 && rem(k,2*numb_interval)==0 &&flag_dash==0)
            Tei(count)=normTE(1,k);
            Spei(count)=normSPE(1,k);
            Moei(count)=normMOE(1,k);
            flag=flag+1;
%         normTE(1,k)
%         normSPE(1,k)
%         normMOE(1,k)
        end
        if (k>200 && rem(k,2*numb_interval)==numb_interval-1 &&flag_dash==1)
            Tef(count)=normTE(1,k);
            Spef(count)=normSPE(1,k);
            Moef(count)=normMOE(1,k);
            flag=flag+1;
%         normTE(1,k)
%         normSPE(1,k)
%         normMOE(1,k)
        end
        normuFF(1,k)=norm(uFF(:,1:(time/ts)-2));
        normu(1,k)=norm(u(:,1:(time/ts)-2));
        normuILC(1,k)=norm(uILC(:,1:(time/ts)-2));
        normX(1,k)=norm(C*A*A*x(:,1:(time/ts)-2));
        normMru(1,k)=norm(C*A*B*u(:,1:(time/ts)-2));
        normMruFF(1,k)=norm((1-lambda_FF-gamma_FF)*C*A*B*uFF(:,1:(time/ts)-2));
        normMruILC(1,k)=norm(C*A*B*uFF(:,1:(time/ts)-2)-ymodel_(:,1:(time/ts)-2));
        %% Auxilliary steps
        for k1=1:l
            E(k1,k)=(norm(abs(yd(k1,1:(time/ts))-y(k1,1:(time/ts)))).^2);
        end
        if norme==0
            E_(:,k)=norm(yd(:,1:(time/ts))-y(:,1:(time/ts)));
        end
        if norme==1
            E_(:,k)=norm(yd(:,1:(time/ts))-y(:,1:(time/ts)))/norm(yd(:,1:(time/ts)));
        end
        %     if (k==4*num_trials)
        %         y1=y;
        %         x1=x;
        %     end
        %     if (k==num_trials*5)
        %         y2=y;
        %         x2=x;
        %     end
        %     if (k==num_trials*6)
        %         y3=y;
        %         x3=x;
        %     end
        %     if (k==7*num_trials)
        %         y4=y;
        %         x4=x;
        %     end
        %     YendP(1,k)=y(1,end-1);
        %%
        %     set(0,'CurrentFigure',fg1);
        %     if (k<=(2*num_trials)) %(Baseline)
        %         if (rem(k,20)==0)||(k==(2*num_trials+1))
        %             hold on
        %             subplot(1,4,1)
        %             hold on
        %             plot(y(1,1:end-2),y(2,1:end-2),'b-','LineWidth',0.1)
        %             %             axis equal
        %             xlabel('X (m)')
        %             ylabel('Y (m)')
        %             title('Baseline')
        %             ylim([0 1.1])
        %         end
        %     end
        %     if (k>=(4*num_trials+1))&&(k<=(5*num_trials)) %(PERT PT)
        %         if (rem(k,20)==0)||(k==(4*num_trials+1))
        %             subplot(1,4,2)
        %             hold on
        %             plot(y(1,1:end-2),y(2,1:end-2),'b-','LineWidth',0.1)
        %             %             axis equal
        %             xlabel('X (m)')
        %             ylabel('Y (m)')
        %             title('PT1')
        %             ylim([0 1.1])
        %         end
        %     end
        %     if (k>=num_trials*5+1)&&(k<=num_trials*6) %(Washout)
        %         if (rem(k,20)==0)||(k==(5*num_trials+1))
        %             subplot(1,4,3)
        %             hold on
        %             plot(y(1,1:end-2),y(2,1:end-2),'b-','LineWidth',0.1)
        %             %             axis equal
        %             xlabel('X (m)')
        %             ylabel('Y (m)')
        %             title('Washout')
        %             ylim([0 1.1])
        %         end
        %     end
        %     if (k>=num_trials*6+1)&&(k<=num_trials*7) %(PERT PT again)
        %         if (rem(k,20)==0)||(k==(6*num_trials+1))
        %             subplot(1,4,4)
        %             hold on
        %             plot(y(1,1:end-2),y(2,1:end-2),'b-','LineWidth',0.1)
        %             %             axis equal
        %             xlabel('X (m)')
        %             ylabel('Y (m)')
        %             title('PT2')
        %             ylim([0 1.1])
        %         end
        %     end
    end
end
end

%% Retention and  Rate Calculations

% Retention = E_(num_trials*4)-E_(num_trials*6);
% Rate_pt1_fast =(log(E_(num_trials*4+2))- log(E_(num_trials*4+6))/4);
% Rate_pt1_slow =(log(E_(num_trials*4+51))- log(E_(num_trials*5))/49);
% Rate_wash_fast =(log(E_(num_trials*5+2))-log(E_(num_trials*5+6))/4);
% Rate_wash_slow =(log(E_(num_trials*5+51))-log(E_(num_trials*6))/49);
% Rate_pt2_fast =(log(E_(num_trials*6+2))- log(E_(num_trials*6+6))/4);
% Rate_pt2_slow =(log(E_(num_trials*6+51))- log(E_(num_trials*7))/49);
% Rates(i_index,:)= [Retention Rate_pt1_fast Rate_pt1_slow Rate_wash_fast Rate_wash_slow Rate_pt2_fast Rate_pt2_slow];
% RATES = array2table (Rates,'VariableNames',{'Retention','Rate_pt1_fast','Rate_pt1_slow','Rate_wash_fast','Rate_wash_slow','Rate_pt2_fast','Rate_pt2_slow'});

%     writetable(Rate,'Rate.xlsx','Sheet','gamma_FF');

%% Curve Fitting
%
% pt1_fast = E_((num_trials*4+3):(num_trials*4+9));
% pt1_slow = E_((num_trials*4+51):(num_trials*5));
% wash_fast = E_((num_trials*5+3):(num_trials*5+9));
% wash_slow = E_((num_trials*5+51):(num_trials*6));
% pt2_fast = E_((num_trials*6+3):(num_trials*6+9));
% pt2_slow = E_((num_trials*6+51):(num_trials*7));
% trials_fast = 1:1:length(pt1_fast);
% trials_slow = 1:1:length(pt1_slow);
% pt1 =  E_((num_trials*4+2):(num_trials*5));
% pt2 =  E_((num_trials*6+2):(num_trials*7));
% trials = 1:1:length(pt1);
%
% ft = fittype( 'a*exp(-x/b)+c', 'independent', 'x', 'dependent', 'y' );
% opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
% opts.Display = 'Off';
% opts.StartPoint = [0.00695245972821179 0.698283268497527 0.694065593384347];
%
% [xf1Data, yf1Data] = prepareCurveData( trials_fast, pt1_fast );
% [f1_fitresult, f1_gof] = fit( xf1Data, yf1Data, ft, opts );
%
% [xf2Data, yf2Data] = prepareCurveData( trials_fast, pt2_fast );
% [f2_fitresult, f2_gof] = fit( xf2Data, yf2Data, ft, opts );
%
% [xfwData, yfwData] = prepareCurveData( trials_fast, wash_fast );
% [fw_fitresult, fw_gof] = fit( xfwData, yfwData, ft, opts );
%
% [xs1Data, ys1Data] = prepareCurveData( trials_slow, pt1_slow );
% [s1_fitresult, s1_gof] = fit( xs1Data, ys1Data, ft, opts );
%
% [xs2Data, ys2Data] = prepareCurveData( trials_slow, pt2_slow );
% [s2_fitresult, s2_gof] = fit( xs2Data, ys2Data, ft, opts );
%
% [xswData, yswData] = prepareCurveData( trials_slow, wash_slow );
% [sw_fitresult, sw_gof] = fit( xswData, yswData, ft, opts );

%% Reference commands
% figure(1)
% hold on
% subplot(1,4,1)
% hold on
% plot(yd1(1,[1:end-2]),yd1(2,[1:end-2]),'r--','LineWidth',2)
% xlabel('X (m)')
% ylabel('Y (m)')
% ylim([0 0.6])
% title('Baseline')
% subplot(1,4,2)
% hold on
% plot(yd1(1,[1:end-2]),ydx(2,[1:end-2]),'r--','LineWidth',2)
% xlabel('X (m)')
% ylabel('Y (m)')
% title('PT1')
% ylim([0 0.6])
% subplot(1,4,3)
% hold on
% plot(yd1(1,[1:end-2]),yd1(2,[1:end-2]),'r--','LineWidth',2)
% ylim([0 0.6])
% xlabel('X (m)')
% ylabel('Y (m)')
% title('Washout')
% ylim([0 0.6])
% subplot(1,4,4)
% hold on
% plot(yd1(1,[1:end-2]),ydx(2,[1:end-2]),'r--','LineWidth',2)
% ylim([0 0.6])
% xlabel('X (m)')
% ylabel('Y (m)')
% title('PT2')
% %% Converged trajectories
% figure(1)
% hold on
% subplot(1,4,1)
% hold on
% plot(y1(1,[1:end-2]),y1(2,[1:end-2]),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,2)
% hold on
% plot(y2(1,[1:end-2]),y2(2,[1:end-2]),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,3)
% hold on
% plot(y3(1,[1:end-2]),y3(2,[1:end-2]),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,4)
% hold on
% plot(y4(1,[1:end-2]),y4(2,[1:end-2]),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% s.Width= '8'; % Figure width on canvas
% s.Height= '5';
% if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'ConvergedXY.pdf',s);
% end
%%
% figure(2)
% plot(normTE);hold on
% plot(normSPE);
% plot(normMOE)
% grid on
% % for kk=4:7
% %     xline(kk*num_trials,'k:','LineWidth',2)
% % end
% xlabel('Iterations (k)')
% ylabel('Norm of individual components')
% legend('TE','SPE','MOE','Location','northwest')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
% s.Width= '8'; % Figure width on canvas
% s.Height= '5';
% if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'ErrorProportions.pdf',s);
% end
% %% Error
% figure(3)
% subplot(211)
% stem(E_,'LineWidth',0.1,'MarkerSize',0.1)
% hold on
% plot(E_,'LineWidth',0.5)
% grid on
% xlabel('Iterations (k)')
% ylabel('||E||_2=||y_{ref,k}-y_k||_2')
% % for kk=4:7
% %     xline(kk*num_trials,'k:','LineWidth',2)
% % end
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
% subplot(212)
% stem(log(E_),'LineWidth',0.1,'MarkerSize',0.1)
% hold on
% plot(log(E_),'LineWidth',0.5)
% for kk=4:7
%     xline(kk*num_trials,'k:','LineWidth',2)
% end
% grid on
% hold on
% xlabel('Iterations (k)')
% ylabel('log(||E||_2)')
% s.Width= '8'; % Figure width on canvas
% s.Height= '5';
% if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'Errors.pdf',s);
% end
%% Baseline plot
figure(9)
subplot(211)
stem(1:paradigm_trials-3*num_trials,E_(:,1:length(1:paradigm_trials-3*num_trials)),'LineWidth',0.1,'MarkerSize',0.1)
hold on
plot(1:paradigm_trials-3*num_trials,E_(:,1:length(1:paradigm_trials-3*num_trials)),'LineWidth',0.5)
xlim([0*num_trials+1 2*num_trials])
grid on
xlabel('Iterations (k)')
ylabel('||E||_2=||y_{ref,k}-y_k||_2')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
subplot(212)
stem(1:paradigm_trials-3*num_trials,log(E_(:,1:length(1:paradigm_trials-3*num_trials))),'LineWidth',0.1,'MarkerSize',0.1)
hold on
plot(1:paradigm_trials-3*num_trials,log(E_(:,1:length(1:paradigm_trials-3*num_trials))),'LineWidth',0.5)
xlim([0*num_trials+1 2*num_trials])
grid on
hold on
xlabel('Iterations (k)')
ylabel('log(||E||_2)')
s.Width= '6'; % Figure width on canvas
s.Height= '4';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[6 4]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'BaselineError.pdf',s);
end
%%
figure(16)
plot(normX);hold on
plot(normMru);
plot(normMruFF)
plot(normMruILC)
grid on
for kk=4:7
    xline(kk*num_trials,'k:','LineWidth',2)
end
xlabel('Iterations (k)')
ylabel('Norm of CA^\rho x, M_{\rho} u')
legend('||CA^\rho x_k||_2','||M_{\rho} u_k||_2','||(1-\lambda_{FF}-\gamma_{FF})M_{\rho} u_{{\rm FF},k}||_2','||M_{\rho} u_{{\rm ILC},k}-y_{\rm model}||_2','Location','northwest')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
% if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'Mr.pdf',s);
% end
%%
figure(10)
plot(normu);hold on
plot(normuFF);
plot(normuILC)
grid on
% for kk=4:7
%     xline(kk*num_trials,'k:','LineWidth',2)
% end
xlabel('Iterations (k)')
ylabel('Norm of individual components')
legend('||u_k||_2','||u_{FF,k}||_2','||u_{ILC,k}||_2','Location','northwest')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'ContribBaseline.pdf',s);
hold off
end
%% Matced Trials
% figure(11)
% hold on
% plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'k:','LineWidth',2)
% plot(1:num_trials*1,((E_(num_trials*4+1:5*num_trials))),'m','LineWidth',1.5)
% plot(1:num_trials*1,((E_(num_trials*5+1:6*num_trials))),'g--','LineWidth',2)
% plot(1:num_trials*1,((E_(num_trials*6+1:7*num_trials))),'b','LineWidth',1.5)
% legend(['Baseline, \Sigma=',num2str(sum(((E_(num_trials*3+1:4*num_trials)))))],...
%    [' PT1, FR=',num2str(f1_fitresult.b),';SR=',num2str(s1_fitresult.b)],...
%    ['Washout,FR=',num2str(fw_fitresult.b),'; SR=',num2str(sw_fitresult.b)],...
%   [' PT2, FR=',num2str(f2_fitresult.b),'; SR=',num2str(s2_fitresult.b)])%,' PT', 'Washout', ' PT'}
% xlabel('Matched Iterations (k)')
% ylabel('||E||_2')
% grid on
% title(['\rho=',num2str(round(Retention,2)),',\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
% s.Width= '8'; % Figure width on canvas
% s.Height= '5';
% % if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'Matched_iterations_E.pdf',s)
% end
%
figure(12)

    if i == 0.1
        plot(normuFF,'-ro','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4);hold on
        plot(normu,'-mo','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
        plot(normuILC,'-bo','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
    end
    if i == 0.9
        plot(normuFF,'-r^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4);hold on
        plot(normu,'-m^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
        plot(normuILC,'-b^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
    end

plot(normuFF,'-r','LineWidth',0.5);hold on
plot(normu,'-m','LineWidth',0.5)
plot(normuILC,'-b','LineWidth',0.5)

grid on
xlabel('Iterations (k)')
ylabel('Norm of individual components')
legend('||u_{FF,k}||_2','||u_k||_2','||u_{ILC,k}||_2','Location','Northwest')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
s.Width= '6'; % Figure width on canvas
s.Height= '4';
xlim([400 500])


if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[6 6]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'SlowFast.pdf',s);
end
%%
figure(13)
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
hold on
plot(E_(:,301:end),'LineWidth',0.5)
grid on
xlabel('Iterations (k)')
ylabel('||E||_2=||y_{ref,k}-y_k||_2')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{model}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{model}=',num2str(lambda_model)])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
for kk=1:3
    xline(kk*num_trials,'k:','LineWidth',2)
end
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'Error_PT.pdf',s)
end
clear Rate_pt1_fast Rate_pt1_slow Rate_wash_fast Rate_wash_slow Rate_pt2_fast Rate_pt2_slow

%%
% figure(14)
% plot( f1_fitresult,'m-',xf1Data, yf1Data,'ko');hold on; plot( f2_fitresult,'b-', xf2Data, yf2Data,'k*'); hold on; plot( fw_fitresult,'g-', xfwData, yfwData,'k^');
% %     plot( f1_fitresult,'m-',xf1Data, yf1Data,'ko','DisplayName','pt1');hold on; plot( f2_fitresult,'b-', xf2Data, yf2Data,'k*','DisplayName','pt2' ); hold on; plot( fw_fitresult,'g-', xfwData, yfwData,'k^','DisplayName','washout');
% legend(  'pt1 data','pt1 fit', 'pt2 data','pt2 fit','Washout data','washout fit', 'Location', 'NorthEast');
% % Label axes
% xlabel( 'Iterations', 'Interpreter', 'none' );
% ylabel( '||E||_2', 'Interpreter', 'none' );
% title(['Rates: pt1= ',num2str(f1_fitresult.b),'; pt2= ',num2str(f2_fitresult.b),'; washout= ',num2str(fw_fitresult.b)]);
% grid on
%
% figure(15)
% plot( s1_fitresult,'m-',xs1Data, ys1Data,'ko');hold on; plot( s2_fitresult,'b-', xs2Data, ys2Data,'k*'); hold on; plot( sw_fitresult,'g-', xswData, yswData,'k^');
% %     plot( f1_fitresult,'m-',xf1Data, yf1Data,'ko','DisplayName','pt1');hold on; plot( f2_fitresult,'b-', xf2Data, yf2Data,'k*','DisplayName','pt2' ); hold on; plot( fw_fitresult,'g-', xfwData, yfwData,'k^','DisplayName','washout');
% legend(  'pt1 data','pt1 fit', 'pt2 data','pt2 fit','Washout data','washout fit', 'Location', 'NorthEast');
% % Label axes
% xlabel( 'Iterations', 'Interpreter', 'none' );
% ylabel( '||E||_2', 'Interpreter', 'none' );
% title(['Rates: pt1=',num2str(s1_fitresult.b),'; pt2= ',num2str(s2_fitresult.b),'; washout= ',num2str(sw_fitresult.b)]);
% grid on

% end
%
% saveas(figure(12),'gamma_FF');
%%
figure(15)

plot(numb_array,Tei,'-o');hold on
plot(numb_array,Tef,'-o');
plot(numb_array,Moei,'-o');
plot(numb_array,Moef,'-o');
plot(numb_array,Spei,'-o');
plot(numb_array,Spef,'-o'   );

grid on
xlabel('number of intervals')
ylabel('Norm of individual components (initial and final)')
% legend('||u_{FF,k}||_2','||u_k||_2','||u_{ILC,k}||_2','Location','Northwest')
legend('Tei','Tef','Moei','Moef','Spei','Spef')
title('Different errors during start and end of perturbations')

%%
figure(17)

plot(numb_array,Tei_dash,'-o');hold on
plot(numb_array,Tef_dash,'-o');
plot(numb_array,Moei_dash,'-o');
plot(numb_array,Moef_dash,'-o');
plot(numb_array,Spei_dash,'-o');
plot(numb_array,Spef_dash,'-o'   );

grid on
xlabel('number of intervals')
ylabel('Norm of individual components (initial and final)')
% legend('||u_{FF,k}||_2','||u_k||_2','||u_{ILC,k}||_2','Location','Northwest')
legend('Tei','Tef','Moei','Moef','Spei','Spef')
title('Different errors during start and end of trials without perturbations')

