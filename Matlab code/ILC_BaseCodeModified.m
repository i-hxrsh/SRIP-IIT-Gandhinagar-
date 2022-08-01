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

%% Simulation Loop
% for jj= [-1 -0.5 -0.1]
i_index=0;
% for ii= -0.5:0.1:0.5
ii=1;
i_index=i_index+1;

%% Initialization
num_trials=50;
ts=0.01;% Sampling time interval
t1=5*num_trials; %pt1 onset
t2=t1+num_trials; %washout onset 
t3=t2+num_trials; %pt2 onset
t4=t3+num_trials;  % pt2 end
paradigm_trials=t4;% Total number of trials in the paradigm
time=1;
duration=time;
export_plots_to_png=0;
export_plots_to_pdf=1;

%% System definition
m1=0.1;% mass of link 1
m2=0.1;% mass of link 2
lc1=0.15;% midlength of link 1
lc2=0.15;% midlength of link 2
l1=lc1*2;
l2=lc2*2;
I1=1/3*m1*(2*lc1)^2; % M. I. of link 1
I2=1/3*m2*(2*lc2)^2; % M. I. of link 2
Q1=pi/4; % Equilibrium point of q1
Q2=pi/2; % Equilibrium point of q2
d1=0.1;% Damping 1
d2=0.1;% Damping 2
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
Bc=[b11;1*b12]; % Continuous-time B matrix
A=1*(1*eye(size(Ac,1))+Ac*ts); % Discrete-time A matrix
% A_=0.5*(1*eye(size(Ac,1))+Ac*ts); %% Case 1
A_=0.5*(1*eye(size(Ac,1))+Ac*ts); %% Case 2
% A_=(0.5*eye(size(Ac,1))+[a11 1*a12;a21 -1*pinv(a22)*a22/ts]*ts); %% Case 2
B=Bc; % Discrete-time B matrix
C=[1 0 0 0;0 1 0 0];
% K=1*place(A,B,1.25*[0.3 0.0 0.5 0.0]); % Works for ts=0.1 THird value controls the shape of exponent, last two equal means osc response
K=0.8*place(A,B,0.15*eig(A));
% K=place(A,B,[0.95,0.95,0.25,0.25]);gamma_ILC=0.2
% K=place(A,B,eig(A));
[~,n]=size(A);
[l,m]=size(C*B);
%% Controller Equations ORIGINAL Parameterization
% lambda_ILC= 1;%  
% lambda_model=0.7;
% lambda_FF=0.7;% Confidence on the internal learnt model
% gamma_ILC=0.1;% Initial learning rate
% gamma_model= 0.2; %Initial learning rate
% gamma_FF=0.1; %|| Range: 0.05:0.1
% nx=0.0;% Noise levels
% ny=0.0;% Noise levels
%%
lambda_ILC= 1;%  
lambda_model=0.7;
lambda_FF=0.7;% Confidence on the internal learnt model
gamma_ILC=0.2;% Initial learning rate
gamma_model= 0.1; %Initial learning rate
gamma_FF=0.1; %|| Range: 0.05:0.1
nx=0.0;% Noise levels
ny=0.0;% Noise levels
%% Direction of reaching task
xi=0;
yi=0.1;
xf=0;
xfx=0;
yf=0.4;
%% Perturbation Definition
enableVMR=0;
th=0.5236; % 30 degree rotation
enableFFPT=1;
enableVLPT=0;
beta= 0.5;% FF Perturbation level
alpha=15;% VL Perturbation level
%% Trajectory Generation
% Reference Commands Generation (Minimum Jerk Trajectory, per R. Shadhmehr and
% S. P. Wise) code based on Supplementary documents for "Computational Neurobiology
% of Reaching and Pointing",
t=0;
yref=[xf*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
% yref=[xfx*ones(1,time/ts+2);yf*ones(1,time/ts+2)];
yrefT=0*yref;
for loop_time=0:ts:time+ts
    t=t+1;
    yrefT(:,t)=[xi+(xf-xi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5);
        yi+(yf-yi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5)];
    [yref(1,t),yref(2,t)]=tracgen(yrefT(1,t),yrefT(2,t),l1,l2);
end
t=0;
% for loop_time=0:ts:time
%     t=t+1;
%     yref(:,t)=[xi+(xfx-xi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5);
%         yi+(yf-yi)*(10*(loop_time/duration)^3-15*(loop_time/duration)^4+6*(loop_time/duration)^5)];
% end
% for rd=[0]
rd=0;
FF_flag=0;
for FF_flag=[0]
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
yt=y_;
yt_=yt;
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
fg1=figure(1);
fg2=figure(2);
% fg3=figure(3);
% fg6=figure(6);
diff(yref)
for k=1:paradigm_trials
%     x=zeros(n,length(yref));
x=[yref(1,1)*ones(1,length(yref));yref(2,1)*ones(1,length(yref));zeros(n/2,length(yref))];
    xILC=zeros(n,length(yref));
    xFF=xILC;
    y=C*x;
    
    %% Across-the-Time for loop
    t=0;
    for loop_time=0:ts:time-1*ts
        vdx=[0 1;0 0]*[[0;0] (diff(y_'))'];
        t=t+1;
        %% Perturbation Compututations
             
        if (k<=(4*num_trials)) %(Baseline)
            yref=yref;
            VMR=[0;0];
            RR=eye(2);
            FFPT=[0;0];
            VLPT=0;
        end
        if (k>=(t1+1))&&(k<=(t2)) %(PERT PT)
            yref=yref;
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
            if rd==0
             FFPT=[0;0];
            end
            VLPT=0;
            yref=yref;
            if rd==1
            FFPT=1/3*[beta;0];
            end
        end
        if (k>=t3+1)&&(k<=t4) %(PERT PT again)
            yref=yref;
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
%         if (log(norm(TE(:,t+2)))>=-10)
            %% Learning Controller Model
            uILC(:,t)=lambda_ILC*uILC_(:,t)+gamma_ILC*tanh(TE(:,t+2));
            ymodel(:,t+2)=lambda_model*ymodel_(:,t+2)+gamma_model*tanh(SPE(:,t+2));
            uFF(:,t)=lambda_FF*uFF_(:,t)+gamma_FF*pinv(C*A*B)*tanh((MOE(:,t+2)));
            u(:,t)=1*uILC(:,t)+FF_flag*uFF(:,t);
%         else
%             u(:,t)=u_(:,t);
%         end
        %% State-space System
                J=[-l1*sin(x(1,t))-l2*sin(x(1,t)+x(2,t)) -l2*sin(x(1,t)+x(2,t));
            l1*cos(x(1,t))+l2*cos(x(1,t)+x(2,t)) l2*cos(x(1,t)+x(2,t))];
        x(:,t+1)=A*x(:,t)+nx*rand(n,1)+B*(u(:,t)+(J'*FFPT+VLPT.*vdx(:,t)))-B*K*x(:,t);
%         x(:,t+1)=A*x(:,t)+nx*rand(n,1)+B*(u(:,t)+pinv(C*A*B)*FFPT+VLPT.*vdx(:,t));
        y(:,t)=C*x(:,t)+ny*rand(l,1);
        yt(1,t)=l1*cos(y(1,t))+l2*cos(y(1,t)+y(2,t));
        yt(2,t)=l1*sin(y(1,t))+l2*sin(y(1,t)+y(2,t));
    end
    %% Save data for next iteration
    uILC_=uILC;% ILC data
    yt_=yt;
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
        E(k1,k)=mean(abs(yrefT(k1,1:(time/ts))-yt(k1,1:(time/ts))));
    end
%     if norme==0
%         E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)));
E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)));
%     end
%     if norme==1
%         E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)))/norm(yref(:,1:(time/ts)));
% E_(:,k)=norm(yref(:,1:(time/ts))-y(:,1:(time/ts)))/norm(yref(:,1:(time/ts)));
%     end
    if (k==t1)
        x1=yt;
    end
    if (k==t2)
        x2=yt;
    end
    if (k==t3)
        x3=yt;
    end
    if (k==t4)
        x4=yt;
    end
    YendP(1,k)=y(1,end-2);
    %%
    set(0,'CurrentFigure',fg1);
    if (k<=(2*num_trials)) %(Baseline)
        if (rem(k,floor(0.1*num_trials))==0)||(k==(2*num_trials+1))
            hold on
            subplot(1,4,1)
            hold on
            plot(yt(1,1:end-2),yt(2,1:end-2),'b-','LineWidth',0.1)
            %             axis equal
            xlabel('X (m)')
            ylabel('Y (m)')
            title('Baseline')
            ylim([0.3 0.6])
            xlim([-0.1 0.1])
%             axis equal
        end
    end
    if (k>=(t1+1))&&(k<=(t2)) %(PERT PT)
        if (rem(k,floor(0.1*num_trials))==0)||(k==(4*num_trials+1))
            subplot(1,4,2)
            hold on
            plot(yt(1,1:end-2),yt(2,1:end-2),'b-','LineWidth',0.1)
            %             axis equal
            xlabel('X (m)')
            ylabel('Y (m)')
            title('PT1')
            ylim([0.3 0.6])
            xlim([-0.1 0.1])
%             axis equal
        end
    end
    if (k>=t2+1)&&(k<=t3) %(Washout)
        if (rem(k,floor(0.1*num_trials))==0)||(k==(5*num_trials+1))
            subplot(1,4,3)
            hold on
            plot(yt(1,1:end-2),yt(2,1:end-2),'b-','LineWidth',0.1)
            %             axis equal
            xlabel('X (m)')
            ylabel('Y (m)')
            title('Washout')
%             axis equal
            ylim([0.3 0.6])
            xlim([-0.1 0.1])
        end
    end
    if (k>=t3+1)&&(k<=t4) %(PERT PT again)
        if (rem(k,floor(0.1*num_trials))==0)||(k==(6*num_trials+1))
            subplot(1,4,4)
            hold on
            plot(yt(1,1:end-2),yt(2,1:end-2),'b-','LineWidth',0.1)
            %             axis equal
            xlabel('X (m)')
            ylabel('Y (m)')
            title('PT2')
%             axis equal
            ylim([0.3 0.6])
            xlim([-0.1 0.1])
        end
    end
end
%% Retention and  Rate Calculations

Retention = E_(t1+1)-E_(t3+1);
%% Curve Fitting
pt1_fast = E(1,(t1+3):(t1+(num_trials-3)));
pt1_slow = E(1,(t1+num_trials/2+1):(t1+num_trials));
wash_fast = E(1,(t2+3):(t2+(num_trials-3)));
wash_slow = E(1,(t2+num_trials/2+1):(t2+num_trials));
pt2_fast = E(1,(t3+3):(t3+(num_trials-3)));
pt2_slow = E(1,(t3+num_trials/2+1):(t3+num_trials));
trials_fast = 1:1:length(pt1_fast);
trials_slow = 1:1:length(pt1_slow);
pt1 =  E(1,(num_trials*4+2):(num_trials*5));
pt2 =  E(1,(num_trials*6+2):(num_trials*7));
trials = 1:1:length(pt1);

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
                  E_(t3+1),E_(t4),f2_fitresult.b,s2_fitresult.b];
EXP = array2table (Exp,'VariableNames',{'Beta','Retention','E1_pt1','E2_pt1','Rate_pt1_fast','Rate_pt1_slow','E1_w','E2_w','Rate_wash_fast','Rate_wash_slow','E1_pt2','E2_pt2','Rate_pt2_fast','Rate_pt2_slow'});

%% Reference commands
figure(1)
hold on
subplot(1,4,1)
hold on
plot(yrefT(1,1:end-2),yrefT(2,1:end-2),'r--','LineWidth',2)
xlabel('X (m)')
ylabel('Y (m)')
ylim([0 0.9])
title('Baseline')
subplot(1,4,2)
hold on
plot(yrefT(1,1:end-2),yrefT(2,1:end-2),'r--','LineWidth',2)
xlabel('X (m)')
ylabel('Y (m)')
title('PT1')
ylim([0 0.9])
subplot(1,4,3)
hold on
plot(yrefT(1,1:end-2),yrefT(2,1:end-2),'r--','LineWidth',2)
ylim([0 0.9])
xlabel('X (m)')
ylabel('Y (m)')
title('Washout')
ylim([0 0.9])
subplot(1,4,4)
hold on
plot(yrefT(1,1:end-2),yrefT(2,1:end-2),'r--','LineWidth',2)
ylim([0 0.9])
xlabel('X (m)')
ylabel('Y (m)')
title('PT2')
%% Converged trajectories
figure(1)
hold on
subplot(1,4,1)
hold on
plot(x1(1,1:end-2),x1(2,1:end-2),'g-','LineWidth',1)
% axis equal
ylim([0 0.9])
subplot(1,4,2)
hold on
plot(x2(1,1:end-2),x2(2,1:end-2),'g-','LineWidth',1)
% axis equal
ylim([0 0.9])
subplot(1,4,3)
hold on
plot(x3(1,1:end-2),x3(2,1:end-2),'g-','LineWidth',1)
% axis equal
ylim([0 0.9])
subplot(1,4,4)
hold on
plot(x4(1,1:end-2),x4(2,1:end-2),'g-','LineWidth',1)
% axis equal
ylim([0 0.9])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'ConvergedXY.pdf',s);
end
%%
figure(2)
plot(normTE);hold on
plot(normSPE);
plot(normMOE)
grid on
for kk=4:7
    xline(kk*num_trials,'k:','LineWidth',2)
end
xlabel('Iterations (k)')
ylabel('Norm of individual components')
legend('TE','SPE','MOE','Location','northwest')
title(['\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'ErrorProportions.pdf',s);
end
%% Matced Trials
figure(11)
hold on
plot(1:num_trials*1,((E(1,num_trials*3+1:4*num_trials))),'k:','LineWidth',2)
plot(1:(t2-t1),((E(1,t1+1:t2))),'m','LineWidth',1.5)
plot(1:(t3-t2),((E(1,t2+1:t3))),'g--','LineWidth',2)
plot(1:num_trials*1,((E(1,t3+1:t4))),'b','LineWidth',1.5)
fw_fitresult.b
legend(['Baseline, \Sigma=',num2str(sum(((E(num_trials*3+1:4*num_trials)))))],...
    [' PT1, FR=',num2str(f1_fitresult.b),';SR=',num2str(s1_fitresult.b)],...
    ['Washout,FR=',num2str(fw_fitresult.b),'; SR=',num2str(sw_fitresult.b)],...
    [' PT2, FR=',num2str(f2_fitresult.b),'; SR=',num2str(s2_fitresult.b)])%,' PT', 'Washout', ' PT'}
xlabel('Matched Iterations (k)')
ylabel('Norm of Error')%('||E||_2')
grid on
title(['\rho=',num2str(round(Retention,2)),',\gamma_{ILC}=',num2str(gamma_ILC),', \gamma_{FF}=',num2str(gamma_FF),', \gamma_{pred}=',num2str(gamma_model),', \lambda_{ILC}=',num2str(lambda_ILC),', \lambda_{FF}=',num2str(lambda_FF),', \lambda_{pred}=',num2str(lambda_model)])
s.Width= '8'; % Figure width on canvas
s.Height= '5';
if export_plots_to_pdf==1
    chosenfigure=gcf;
    set(chosenfigure,'PaperUnits','inches');
    set(chosenfigure,'PaperPositionMode','auto');
    set(chosenfigure,'PaperSize',[8 5]); % Canvas Size
    set(chosenfigure,'Units','inches');
    hgexport(gcf,'Matched_iterations_E.pdf',s)
end
end