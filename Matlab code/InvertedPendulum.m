M=0.5;
m=0.2;
b=0.1;
l=0.3;
I=0.006;
g=9.8;
p=I*(M+m)+M*m*l^2;

A=[0      1              0           0; 
   0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0; 
   0      0              0           1;
   0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
    
C = [1 0 0 0;
     0 0 1 0];
 
D = [0;
     0];

sys_ss=ss(A,B,C,D);
sys_tf=tf(sys_ss);
P_cart=sys_tf(1);
P_pend=sys_tf(2);
Kp=1;
Ki=1;
Kd=1;
C=pid(Kp,Ki,Kd);
poles=eig(A)

co=ctrb(sys_ss);
controllability=rank(co)