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
Exp = zeros(1,14);
%% Define plot style
clear s;
s.units = 'inches';
s.format = 'pdf';
s.Preview= 'none';
s.Width= '16'; % Figure width on canvas
s.Height= '9'; % % Figure height on canvas
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

%% Simulation Loop
% for jj= [-1 -0.5 -0.1]
i_index=0;
% for ii= -0.5:0.1:0.5
ii=1;
i_index=i_index+1;

%% Initialization
num_trials=100;
ts=0.01;% Sampling time interval
paradigm_trials=700;% Total number of trials in the paradigm
t1=400; %pt1 onset
t2=t1+num_trials; %washout onset 
t3=t2+num_trials; %pt2 onset
t4=t3+num_trials;  % pt2 end
time=1;
duration=time;
export_plots_to_png=0;
export_plots_to_pdf=0;

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
lambda_ILC= 1;% Determination factor (Can you change this by explicit instructions?)
lambda_model=0.7;
lambda_FF=0.7;% Confidence on the internal learnt model
gamma_ILC=0.1;% Initial learning rate
gamma_model= 0.2; %Initial learning rate
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
enableFFPT=0;
enableVLPT=1;
beta= 0.5;% FF Perturbation level
alpha=0.0153;% VL Perturbation level
%% Trajectory Generation
% Reference Commands Generation (Minimum Jerk Trajectory, per R. Shadhmehr and
% S. P. Wise) code based on Supplementary documents for "Computational Neurobiology
% of Reaching and Pointing",
t=0;
yd1=[xf*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
ydx=[xfx*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
yref=0*ydx;
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
count=5;
for lambda_FF=[1,0.9,0.7,0.5,0.3,0.1]
% for lambda_FF=[0.7]
%% Init. variables
norme=0;
plot_surf=0;
y_=zeros(size(yref));
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
pred_term_=uILC_;
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
normTE=normy;
normSPE=normy;
normMOE=normy;
normu=normy;
normuFF=normy;
IT=normy;
MT=normy;
RR=[cos(th) -sin(th);sin(th) cos(th)];
%% Across-the-Trials for loop
% % fg1=figure(1);
% fg2=figure(2);
% fg3=figure(3);
% fg6=figure(6);

% for lambda_FF=[1,0.9,0.7,0.5,0.3,0.1]
% for lambda_FF=[0.7]
    count=count-1;
for k=1:paradigm_trials
    x=zeros(n,length(yref));
    xILC=zeros(n,length(yref));
    xFF=xILC;
    y=C*x;
    
    %% Across-the-Time for loop
    t=1;
    for loop_time=0:ts:time-2*ts
%         vdx=[0 1;0 0]*[[0;0] (diff(y_'))'];
        t=t+1;
        %% Perturbation Compututations
             
        if (k<=(4*num_trials)) %(Baseline)
            yref=ydx;
            VMR=[0;0];
            RR=eye(2);
            FFPT=[0;0];
            VLPT=0;
        end
        if (k>=(t1+1))&&(k<=(t2)) %(PERT PT)
            yref=ydx;
            if enableVMR==1
                RR=[cos(th) -sin(th);sin(th) cos(th)];
            else
                VMR=[0;0];
                RR=eye(2);
            end
            if enableFFPT==1
                FFPT=[beta;0];
            else
                FFPT=[0;0];
            end
            if enableVLPT==1
                VLPT=alpha;
            else
                VLPT=0;
            end
        end
        if (k>=t2+1)&&(k<=t3) %(Washout)
            VMR=[0;0];
            RR=eye(2);
            FFPT=[0;0];
            VLPT=0;
            yref=yd1;
        end
        if (k>=t3+1)&&(k<=t4) %(PERT PT again)
            yref=ydx;
            if enableVMR==1
                RR=[cos(th) -sin(th);sin(th) cos(th)];
            else
                VMR=[0;0];
                RR=eye(2);
            end
            if enableFFPT==1
                FFPT=[beta;0];
            else
                FFPT=[0;0];
            end
            if enableVLPT==1
                VLPT=alpha;
            else
                VLPT=0;
            end
        end
        TE(:,t+2)=yref(:,t+2)-(RR*y_(:,t+2));
        MOE(:,t+2)=C*A*B*u_(:,t)-ymodel_(:,t+2);
        SPE(:,t+2)=(RR)*y_(:,t+2)-ymodel_(:,t+2);
        %% Error Definitions
        if (log(norm(TE(:,t+2)))>=-10)
            %% Learning Controller Model
            uILC(:,t)=lambda_ILC*uILC_(:,t)+gamma_ILC*tanh(TE(:,t+2));
            ymodel(:,t+2)=lambda_model*ymodel_(:,t+2)+gamma_model*tanh(SPE(:,t+2));
            uFF(:,t)=lambda_FF*uFF_(:,t)+gamma_FF*pinv(C*A*B)*tanh((MOE(:,t+2)));
            u(:,t)=1*uILC(:,t)+1*uFF(:,t);
        else
            u(:,t)=u_(:,t);
        end
        %% State-space System
        y(:,t)=C*x(:,t)+ny*rand(l,1);
%         y_=y;
        vdx(:,t)=(y(:,t)-y(:,t-1))/ts;%[0 1;0 0]*[[0;0] (diff(y_'))'];
        x(:,t+1)=A*x(:,t)+nx*rand(n,1)+B*(u(:,t)+pinv(C*A*B)*FFPT+VLPT.*vdx(:,t));
    end
    %% Save data for next iteration
    uILC_=uILC;% ILC data
    y_=y;% output data
    ymodel_=ymodel;% pred data
    Y_=Y;
    uFF_=uFF;
    u_=u;
    normTE(1,k)=norm(TE(:,1:(time/ts)));
    normSPE(1,k)=norm(SPE(:,1:(time/ts)));
    normMOE(1,k)=norm(MOE(:,1:(time/ts)));
    normuFF(1,k)=norm(uFF(:,1:(time/ts)-2));
    normu(1,k)=norm(u(:,1:(time/ts)-2));
    normuILC(1,k)=norm(uILC(:,1:(time/ts)-2));
    %% Auxilliary steps
    for k1=1:l
        E(k1,k)=(norm(abs(yref(k1,1:(time/ts))-y(k1,1:(time/ts)))).^2);
    end
    if norme==0
        E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)));
    end
    if norme==1
        E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)))/norm(yref(:,1:(time/ts)));
    end
    if (k==t1)
        y1=y;
        x1=x;
    end
    if (k==t2)
        y2=y;
        x2=x;
    end
    if (k==t3)
        y3=y;
        x3=x;
    end
    if (k==t4)
        y4=y;
        x4=x;
    end
    YendP(1,k)=y(1,end-2);
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
%     if (k>=(t1+1))&&(k<=(t2)) %(PERT PT)
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
%     if (k>=t2+1)&&(k<=t3) %(Washout)
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
%     if (k>=t3+1)&&(k<=t4) %(PERT PT again)
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
%% Retention and  Rate Calculations

Retention = E_(t1+1)-E_(t3+1);
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
pt1_fast = E_((t1+3):(t1+9));
pt1_slow = E_((t1+num_trials/2+1):(t1+num_trials));
wash_fast = E_((t2+3):(t2+9));
wash_slow = E_((t2+num_trials/2+1):(t2+num_trials));
pt2_fast = E_((t3+3):(t3+9));
pt2_slow = E_((t3+num_trials/2+1):(t3+num_trials));
trials_fast = 1:1:length(pt1_fast);
trials_slow = 1:1:length(pt1_slow);
% pt1 =  E_((num_trials*4+2):(num_trials*5));
% pt2 =  E_((num_trials*6+2):(num_trials*7));
% trials = 1:1:length(pt1);

tempft = fittype('a*exp(-x/b)+c');
tempf=fit(trials_slow',wash_slow',tempft,'StartPoint',[0.22 23.9854 0.0035]);

ft = fittype( 'a*exp(-x/b)+c', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [tempf.a tempf.b tempf.c];

[xf1Data, yf1Data] = prepareCurveData( trials_fast, pt1_fast );
[f1_fitresult, f1_gof] = fit( xf1Data, yf1Data, ft, opts );

[xf2Data, yf2Data] = prepareCurveData( trials_fast, pt2_fast );
[f2_fitresult, f2_gof] = fit( xf2Data, yf2Data, ft, opts );

[xfwData, yfwData] = prepareCurveData( trials_fast, wash_fast );
[fw_fitresult, fw_gof] = fit( xfwData, yfwData, ft, opts );

[xs1Data, ys1Data] = prepareCurveData( trials_slow, pt1_slow );
[s1_fitresult, s1_gof] = fit( xs1Data, ys1Data, ft, opts );

[xs2Data, ys2Data] = prepareCurveData( trials_slow, pt2_slow );
[s2_fitresult, s2_gof] = fit( xs2Data, ys2Data, ft, opts );

[xswData, yswData] = prepareCurveData( trials_slow, wash_slow );
[sw_fitresult, sw_gof] = fit( xswData, yswData, ft, opts );

%%Storing Data
Exp(i_index,:) = [ii,Retention,E_(t1+1),E_(t2),f1_fitresult.b,s1_fitresult.b...
                  E_(t2+1),E_(t3),fw_fitresult.b,sw_fitresult.b...
                  E_(t3+1),E_(t4),f2_fitresult.b,s2_fitresult.b]
EXP = array2table (Exp,'VariableNames',{'Beta','Retention','E1_pt1','E2_pt1','Rate_pt1_fast','Rate_pt1_slow','E1_w','E2_w','Rate_wash_fast','Rate_wash_slow','E1_pt2','E2_pt2','Rate_pt2_fast','Rate_pt2_slow'});

%% Reference commands
% figure(1)
% hold on
% subplot(1,4,1)
% hold on
% plot(yd1(1,1:end-2),yd1(2,1:end-2),'r--','LineWidth',2)
% xlabel('X (m)')
% ylabel('Y (m)')
% ylim([0 0.6])
% title('Baseline')
% subplot(1,4,2)
% hold on
% plot(yd1(1,1:end-2),ydx(2,1:end-2),'r--','LineWidth',2)
% xlabel('X (m)')
% ylabel('Y (m)')
% title('PT1')
% ylim([0 0.6])
% subplot(1,4,3)
% hold on
% plot(yd1(1,1:end-2),yd1(2,1:end-2),'r--','LineWidth',2)
% ylim([0 0.6])
% xlabel('X (m)')
% ylabel('Y (m)')
% title('Washout')
% ylim([0 0.6])
% subplot(1,4,4)
% hold on
% plot(yd1(1,1:end-2),ydx(2,1:end-2),'r--','LineWidth',2)
% ylim([0 0.6])
% xlabel('X (m)')
% ylabel('Y (m)')
% title('PT2')
%% Converged trajectories
% figure(1)
% hold on
% subplot(1,4,1)
% hold on
% plot(y1(1,1:end-2),y1(2,1:end-2),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,2)
% hold on
% plot(y2(1,1:end-2),y2(2,1:end-2),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,3)
% hold on
% plot(y3(1,1:end-2),y3(2,1:end-2),'g-','LineWidth',1)
% % axis equal
% ylim([0 0.6])
% subplot(1,4,4)
% hold on
% plot(y4(1,1:end-2),y4(2,1:end-2),'g-','LineWidth',1)
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
% 
% figure(2)
% plot(normTE);hold on
% plot(normSPE);
% plot(normMOE)
% grid on
% for kk=4:7
%     xline(kk*num_trials,'k:','LineWidth',2)
% end
% xlabel('Iterations (k)')
% ylabel('Norm of individual components')
% legend('TE','SPE','MOE','Location','northwest')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
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
% for kk=4:7
%     xline(kk*num_trials,'k:','LineWidth',2)
% end
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
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
% %% Baseline plot
% figure(9)
% subplot(211)
% stem(1:paradigm_trials-3*num_trials,E_(:,1:length(1:paradigm_trials-3*num_trials)),'LineWidth',0.1,'MarkerSize',0.1)
% hold on
% plot(1:paradigm_trials-3*num_trials,E_(:,1:length(1:paradigm_trials-3*num_trials)),'LineWidth',0.5)
% xlim([0*num_trials+1 2*num_trials])
% grid on
% xlabel('Iterations (k)')
% ylabel('||E||_2=||y_{ref,k}-y_k||_2')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
% subplot(212)
% stem(1:paradigm_trials-3*num_trials,log(E_(:,1:length(1:paradigm_trials-3*num_trials))),'LineWidth',0.1,'MarkerSize',0.1)
% hold on
% plot(1:paradigm_trials-3*num_trials,log(E_(:,1:length(1:paradigm_trials-3*num_trials))),'LineWidth',0.5)
% xlim([0*num_trials+1 2*num_trials])
% grid on
% hold on
% xlabel('Iterations (k)')
% ylabel('log(||E||_2)')
% s.Width= '6'; % Figure width on canvas
% s.Height= '4';
% if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[6 4]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'BaselineError.pdf',s);
% end
% %%
% figure(10)
% plot(normu);hold on
% plot(normuFF);
% plot(normuILC)
% grid on
% for kk=4:7
%     xline(kk*num_trials,'k:','LineWidth',2)
% end
% xlabel('Iterations (k)')
% ylabel('Norm of individual components')
% legend('||u_k||_2','||u_{FF,k}||_2','||u_{ILC,k}||_2','Location','northwest')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
% s.Width= '8'; % Figure width on canvas
% s.Height= '5';
% if export_plots_to_pdf==1
%     chosenfigure=gcf;
%     set(chosenfigure,'PaperUnits','inches');
%     set(chosenfigure,'PaperPositionMode','auto');
%     set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
%     set(chosenfigure,'Units','inches');
%     hgexport(gcf,'ContribBaseline.pdf',s);
% end
% 
%% Matced Trials
figure(11)

if ii==0.1
    plot(1:(t2-t1),((E_(t1+1:t2))),'m','LineWidth',1.5,'Marker','o','MarkerEdgeColor','k','MarkerSize',4,'MarkerIndices',1:10:num_trials);
    plot(1:num_trials*1,((E_(t3+1:t4))),'b','LineWidth',1.5,'Marker','o','MarkerEdgeColor','k','MarkerSize',4,'MarkerIndices',1:10:num_trials);
end
if ii==0.9
    plot(1:(t2-t1),((E_(t1+1:t2))),'m','LineWidth',1.5,'Marker','^','MarkerEdgeColor','k','MarkerSize',4,'MarkerIndices',1:10:num_trials);
    plot(1:num_trials*1,((E_(t3+1:t4))),'b','LineWidth',1.5,'Marker','^','MarkerEdgeColor','k','MarkerSize',4,'MarkerIndices',1:10:num_trials);
end

if count==4
subplot(2,3,1)

hold on
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==3
subplot(2,3,2)

hold on 
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==2

subplot(2,3,3)
hold on 
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==1
subplot(2,3,4)

hold on
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==0

subplot(2,3,5)
hold on
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==-1

subplot(2,3,6)
hold on
plot(1:num_trials*1,((E_(num_trials*3+1:4*num_trials))),'Color','k','LineWidth',2);
plot(1:(t2-t1),((E_(t1+1:t2))),'Color','m','LineWidth',1.5);
plot(1:(t3-t2),((E_(t2+1:t3))),'Color','g','LineStyle','--','LineWidth',2);
plot(1:num_trials*1,((E_(t3+1:t4))),'Color','b','LineStyle','--','LineWidth',1.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
% str1=sprintf('Baseline lamda ILC = %.1f',lambda_ILC);
% str2=sprintf('PT1 lamda ILC = %.1f',lambda_ILC);
% str3=sprintf('Washout lamda ILC = %.1f',lambda_ILC);
% str4=sprintf('PT2 lamda ILC = %.1f',lambda_ILC);
% legend(str1,str2,str3,str4)
legend('Baseline',...
    'PT1 ',...
    'Washout ',...
    'PT2 ',...
    'Baseline ')%,' PT', 'Washout', ' PT'}
xlabel('Matched Iterations (k)')
ylabel('Norm of Error')%('||E||_2')
grid on
% title(['\rho=',num2str(round(Retention,2)),',\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
s.Width= '16'; % Figure width on canvas
s.Height= '9';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[16 9]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'Matched_iterations_E.pdf',s)
end
%%
figure(12)

    if ii == 0.1
        plot(normuFF,'-ro','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4);hold on
        plot(normu,'-mo','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
        plot(normuILC,'-bo','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
    end
    if ii == 0.9
        plot(normuFF,'-r^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4);hold on
        plot(normu,'-m^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
        plot(normuILC,'-b^','LineWidth',0.5,'MarkerEdgeColor','k','MarkerSize',4)
    end
   
if count==4
subplot(2,3,1)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end

if count==3
subplot(2,3,2)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end


if count==2
subplot(2,3,3)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end

if count==1
subplot(2,3,4)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==0
subplot(2,3,5)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==-1
subplot(2,3,6)

hold on
plot(normuFF,'LineWidth',0.5);
plot(normu','LineWidth',0.5);
plot(normuILC','LineWidth',0.5);
title('\lambda_F_F = '+string(lambda_FF))
hold off
end


grid on
xlabel('Iterations (k)')
ylabel('Norm of individual components')
legend('||u_{FF,k}||_2','||u_k||_2','||u_{ILC,k}||_2','Location','Northwest')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
s.Width= '16'; % Figure width on canvas
s.Height= '9';
xlim([400 500])
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[16 9]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'SlowFast.pdf',s);
end
%%
figure(13)
if count==4
subplot(2,3,1)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==3
subplot(2,3,2)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==2
subplot(2,3,3)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==1
subplot(2,3,4)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==0
subplot(2,3,5)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
if count==-1
subplot(2,3,6)
hold on
stem(E_(:,301:end),'LineWidth',0.1,'MarkerSize',0.1)
plot(E_(:,301:end),'LineWidth',0.5)
title('\lambda_F_F = '+string(lambda_FF))
hold off
end
grid on
xlabel('Iterations (k)')
ylabel('||E||_2=||y_{ref,k}-y_k||_2')
% title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
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

% %% Fast Rate
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
% 
% %% Slow Rate
% figure(15)
% plot( s1_fitresult,'m-',xs1Data, ys1Data,'ko');hold on; plot( s2_fitresult,'b-', xs2Data, ys2Data,'k*'); hold on; plot( sw_fitresult,'g-', xswData, yswData,'k^');
% %     plot( f1_fitresult,'m-',xf1Data, yf1Data,'ko','DisplayName','pt1');hold on; plot( f2_fitresult,'b-', xf2Data, yf2Data,'k*','DisplayName','pt2' ); hold on; plot( fw_fitresult,'g-', xfwData, yfwData,'k^','DisplayName','washout');
% legend(  'pt1 data','pt1 fit', 'pt2 data','pt2 fit','Washout data','washout fit', 'Location', 'NorthEast');
% % Label axes
% xlabel( 'Iterations', 'Interpreter', 'none' );
% ylabel( '||E||_2', 'Interpreter', 'none' );
% title(['Rates: pt1=',num2str(s1_fitresult.b),'; pt2= ',num2str(s2_fitresult.b),'; washout= ',num2str(sw_fitresult.b)]);
% grid on
% 
% clear tempf
% % end
% 
% figure(16) % Perturbation Level Simulation Plots
% % 
% % if jj==-1
% %     
% % subplot(1,3,1)
% % plot(Exp(:,1),Exp(:,2),'k^-');xlabel('\lambda_{pred}');ylabel('Retention');hold on;
% % subplot(1,3,2)
% % plot(Exp(:,1),Exp(:,5),'m^-');hold on;plot(Exp(:,1),Exp(:,13),'b*-');xlabel('\lambda_{pred}');ylabel('Fast Savings');legend('pt1', 'pt2');
% % subplot(1,3,3)
% % plot(Exp(:,1),Exp(:,6),'m^-');hold on;plot(Exp(:,1),Exp(:,14),'b*-');xlabel('\lambda_{pred}');ylabel('Slow Savings');legend('pt1', 'pt2');
% % 
% % end
% % 
% % if jj==-0.1
% %     
% % subplot(1,3,1)
% % plot(Exp(:,1),Exp(:,2),'ko-');xlabel('\lambda_{pred}');ylabel('Retention');hold on;
% % subplot(1,3,2)
% % plot(Exp(:,1),Exp(:,5),'mo-');hold on;plot(Exp(:,1),Exp(:,13),'b*-');xlabel('\lambda_{pred}');ylabel('Fast Savings');legend('pt1', 'pt2');
% % subplot(1,3,3)
% % plot(Exp(:,1),Exp(:,6),'mo-');hold on;plot(Exp(:,1),Exp(:,14),'b*-');xlabel('\lambda_{pred}');ylabel('Slow Savings');legend('pt1', 'pt2');
% % hold on;
% % end
% 
% subplot(1,3,1)
% plot(Exp(:,1),Exp(:,2),'k*-');xlabel('\beta');ylabel('Retention'); hold on;
% subplot(1,3,2)
% plot(Exp(:,1),Exp(:,5),'m*-');hold on;plot(Exp(:,1),Exp(:,13),'b*-');xlabel('\beta');ylabel('Fast Rate');legend('pt1', 'pt2');
% subplot(1,3,3)
% plot(Exp(:,1),Exp(:,6),'m*-');hold on;plot(Exp(:,1),Exp(:,14),'b*-');xlabel('\beta');ylabel('Slow Rate');legend('pt1', 'pt2');
% hold on;
% 
% % end
% %
% % saveas(figure(12),'gamma_FF');
end