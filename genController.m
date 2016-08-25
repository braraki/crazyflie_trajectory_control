function [K_LQRp]= genControlelr(dt,m,Stages,q,r)



%% state space form of the Point mass model
%state costs
%q=[qx_p,qy_p,qz_p,qx_v,qy_v,qz_v;qx_i,qy_i,qz_i]
qx_p=q(1);qy_p=q(2);qz_p=q(3);qx_v=q(4);qy_v=q(5);
qz_v=q(6);qx_i=q(7);qy_i=q(8);qz_i=q(9);
% input costs\
rx=r(1); ry=r(2); rz=r(3);


% x_dot=Ax+Bu (pos and vel)---> masspoint
Ap=[0,0,0,1,0,0;
    0,0,0,0,1,0;
    0,0,0,0,0,1;
    0,0,0,0,0,0;
    0,0,0,0,0,0
    0,0,0,0,0,0];

Bp= [0,0,0;
    0,0,0;
    0,0,0;
    1/m,0,0;
    0,1/m,0;
    0,0,1/m];

Cp=[1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,1,0,0,0];
Dp=[];
Ai=[Ap,zeros(6,3);Cp,zeros(3)];
Bi=[Bp;zeros(3)];
Ci=[Cp,zeros(3)];
sys=ss(Ai,Bi,Ci,Dp);
sysd=c2d(sys,dt);

Ad=sysd.A;
Bd=sysd.B;

%sysd=c2d()




Qp=[qx_p,0,0,0,0,0;
    0,qy_p,0,0,0,0;
    0,0,qz_p,0,0,0;
    0,0,0,qx_v,0,0;
    0,0,0,0,qy_v,0;
    0,0,0,0,0,qz_v];

Rp=[rx,0,0;
    0,ry,0;
    0,0,rz];
% Bn=zeros((1+Stages*6),(1+Stages)*3);
% % for i=0:Stages
% %     An((i+1)*6-5:(i+1)*6,:)=Ad^i;
% %     
% % end
% % for i=1:(Stages+1)
% %     for j=1:Stages
% %         if j<i
% %             Bn((i)*6-5:(i)*6,(j)*3-2:(j)*3)=Ad^(i-j-1)*Bd;
% %         end
% %     end
% % end
An=kron(eye(Stages),Ad);
Bn=kron(ones(Stages,1),Bd);
Qn=kron(eye(Stages),Qp);
%Qn=blkdiag(zeros(6),Qn)

 Rn=(Rp);
% Rn=kron(zeros(3),Rp);

Qi=blkdiag(Qp,diag([qx_i,qy_i,qz_i]));

[K_LQRp,Sp,ep] = dlqr(Ad,Bd,Qi,Rn);


%MPCobj = mpc(sys,dt)
end

