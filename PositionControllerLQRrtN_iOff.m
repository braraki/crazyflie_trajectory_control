function [F_des,F_des_c,psi,x_out,Fi,Fp,yawspeed,q_out,w_out,YawLin,R_cw] = PositionControllerLQRrtN(z_q,z_e,setp,setp_yaw,a_ff,R_bw,m,F_lim,K_LQRp,flag,dt,stages)
% Below this height the integral terms are kept constants
INTEGRATOR_HEIGHT_LIMIT = 0.2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Posotion Controller
% PD controller
useStatefeedback=1;
persistent initialized x P ie iyaw R xa Pa q
numStates=6;
PsiS = setp_yaw;
if isempty(initialized)
    initialized =1;
    x=zeros(numStates,1);
    xa=zeros(numStates,1);
    P=ones(numStates);
    Pa=eye(numStates);
    ie=[0;0;0];
    iyaw=0;
    q=[0;0;0;1];
end

ie_offset = [7.4106;
   -5.6609;
   0];
iyaw_offset = 7.3386;

%% estimator
 refpoint=[setp.p;setp.v];
qv=0.5;
r=0.01;

%prediction attitude
%R_cw = RotFromQuatJ(q);


w = xa(4:6);
O=zeros(3);
I=eye(3);
Fc=[ -skew(w),I;
    O,O];

Phi = eye(6) + Fc * dt;


Ga=[ O;I];
Ha=[1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,1,0,0,0];
qw=5;
Qa=diag([qw,qw,qw]);
dq = quatPlusThetaJ(w*dt);
q = quatmultJ(dq,q);
Pa=Phi*Pa*Phi'+Ga*Qa*Ga';

R_cw = RotFromQuatJ(q);
rq = QuatFromRotJ(RotFromQuatH(z_q) * R_cw');
err_q=rq(1:3);
ra=0.1;
Ra=diag([ra,ra,ra]);
Sa=Ha*Pa*Ha'+Ra;
mahalanobis_att=err_q'/Sa*err_q;

if mahalanobis_att<6
    Ka=Pa*Ha'/Sa;
    x_apo=Ka*err_q;
    
    q = quatmultJ(quatPlusThetaJ(x_apo(1:3)), q);
    xa(4:6)=xa(4:6)+x_apo(4:6);
    
    Pa=(eye(6)-Ka*Ha)*Pa;
end
q_out=q;
R_bw = RotFromQuatJ(q);
w_out=xa(4:6);
st=1/2*dt^2;
% prediction Position
F=[1,0,0,dt,0,0;
    0,1,0,0,dt,0;
    0,0,1,0,0,dt;
    0,0,0,1,0,0;
    0,0,0,0,1,0;
    0,0,0,0,0,1];
B=[st,0,0;
    0,st,0;
    0,0,st;
    dt,0,0;
    0,dt,0;
    0,0,dt];
H=[1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,1,0,0,0];
Q=diag([qv,qv,qv]);
R=diag([r,r,r]);
G=[0,0,0;
    0,0,0;
    0,0,0;
    1,0,0;
    0,1,0;
    0,0,1];
useStatefeedback=1;
if useStatefeedback==1
    x=F*x-B*K_LQRp(:,1:6)*(x-refpoint);
else
    x=F*x;
end

if stages>1
%     for i=1:stages
%         xn(i*6-5:i*6,1)=F*x;
%     end
   % x=xn(1:6);
    P=F*P*F'+G*Q*G';
    
    err_p=[z_e(1);z_e(2);z_e(3)]-x(1:3);
    mahalanobis_pos=err_p'/Sa*err_p;
    
    if mahalanobis_pos<6
        Sp=H*P*H'+R;
        
        K=P*H'/Sp;
        x=x+K*err_p;
        P=(eye(6)-K*H)*P;
    end
    
    %% controller
    IntLim=[100;100;100];
    phi=atan2(R_bw(2,3),R_bw(3,3));
    theta=-asin(R_bw(1,3));
    psi=atan2(R_bw(1,2),R_bw(1,1));
    R_c = angle2dcm(0, 0, psi, 'XYZ'); % transform the force setpoint to the body frame
    eulerAngles = [phi; theta; psi];
  
    % error pos body
    e_pos_body=R_c*(x(1:3)-refpoint(1:3));
    F_des=-K_LQRp(:,1:6)*(x-refpoint)+m*a_ff;
    
    % error yaw
    YawLin=atan2(sin(PsiS - psi),cos(PsiS - psi));

    if x(3) < INTEGRATOR_HEIGHT_LIMIT
        intReset=1;
    else
        intReset=0;
    end
    
    %if x(3)>0.1
    if intReset==0
        if abs(ie(1))<IntLim(1)
            ie(1)=ie(1)+e_pos_body(1)*dt;
        else
            ie(1)=ie(1);
        end
        if abs(ie(2))<IntLim(2)
            ie(2)=ie(2)+e_pos_body(2)*dt;
        else
            ie(2)=ie(2);
        end
        if abs(ie(3))<IntLim(3)
            ie(3)=ie(3)+e_pos_body(3)*dt;
        else
            ie(3)=ie(3);
        end
        
        if abs(iyaw)<IntLim(3)
            iyaw=iyaw + YawLin*dt;
        else
            iyaw=iyaw;
        end
        
        
    else
        % If below height limit keep integrals cosntant
         ie=zeros(3,1);

         iyaw=0;
    end

    %end
    K_i=diag([0.1;0.1;0.1]);
    Fi=-K_LQRp(:,7:end)*(ie+ie_offset);
    Fp=R_c*F_des;
    
    %m*[0;0;9.81]
    % if abs(F_des(1))>=F_lim(1)
    %     F_des(1)=sign(F_des(1))*F_lim(1) ;
    % end
    % if abs(F_des(2))>=F_lim(2)
    %     F_des(2)=sign(F_des(2))*F_lim(2) ;
    % end
    % if abs(F_des(3))>=F_lim(3)
    %     F_des(3)=sign(F_des(3))*F_lim(3) ;
    % end
    
    
    %% yaw control
    
    yawspeed=0.8 * YawLin + 0.1*(iyaw+iyaw_offset);
    F_des_c=Fp+Fi;
    x_out=x;
    
%     disp('iyaw')
%     disp(iyaw)
%     disp('ie')
%     disp(ie)
    
end
