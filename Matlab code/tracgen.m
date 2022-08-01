function [q1,q2]=tracgen(x,y,l1,l2)
q2=acos((x.^2 + y.^2 - l1.^2 - l2.^2)/(2*l1*l2));
D=(x.^2 + y.^2 - l1.^2 - l2.^2)/(2*l1*l2);
% q2=atan(sqrt(1-D.^2)/D)
q1=(atan(y./x) - atan(l2*sin(q2)./(l1 + (l2*cos(q2)))));
%q2(end)=q2(end-1);

for i=1:1:length(q1)
    if(isnan(q1(i))==1)
        q1(i)=0;
    end
   
    if(isnan(q2(i))==1)
        q2(i)=0;
    end
end
end