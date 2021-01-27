function [beta_prediction,yawrate_prediction,late_prediction,yawe_prediction,steer_prediction]=...
    MPC_controller_bicycle(Nps, beta, Yawrate, lat_e , yaw_e,steer,curv_r_pre,vx_pred)

tic
x_prev=[beta;Yawrate;lat_e ;yaw_e ];
u_prev=[steer];

%% parameters
b=1.551/2;
L=2.660    ;
lf=2.660*0.5178;
lr=L-lf;
Ms=1570;
Mus=90;%both side
M=Ms+Mus*2;
hcg=0.4;%%%%%%%%%%%%%%%%%%
g=9.81;
Iz=1500;%%%%%%%%%%%%%%%%%%%%
cf=50000;
cr=50000;

Ts=0.1;
mu=1;

%% parameters (old)
% b=1.551/2;
% L=2.660    ;
% lf=2.660*0.5178;
% lr=L-lf;
% Ms=1570;
% Mus=90;%both side
% M=Ms+Mus*2;
% hcg=0.4;%%%%%%%%%%%%%%%%%%
% g=9.81;
% Iz=2500;%%%%%%%%%%%%%%%%%%%%
% cf=100000;
% cr=100000;
% 
% Ts=0.1;
% mu=1;
%% mpc gain
cont_dim=1;

steer_gain=100;

r=steer_gain;
R_bars=Eye_matrix(r,Nps);

Qm=diag([0 0 1 1]);
lamdam=diag([1 1 1 1])*0.8;

%% make weight for MPC
QS=Qm;
R_bars=[];
for i=1:Nps-1
    QS=[QS zeros((size(QS))*[1;0],(size(Qm))*[0;1]);zeros((size(QS))*[1;0],(size(Qm))*[0;1])' Qm*lamdam^i];
    
end
% QS=zeros(4*Nps,4*Nps);
% QS(5:8,5:8)=Qm;
for i=1:Nps
    
    R_bars=blkdiag(R_bars,r*(0.9^(i-1)));
end
%% control change cost
Du=Eye_matrix(eye(cont_dim),Nps);
Du(cont_dim+1:end,1:end-cont_dim)=Du(cont_dim+1:end,1:end-cont_dim)-Eye_matrix(eye(cont_dim),Nps-1);

Up=zeros(cont_dim*Nps,1);
Up(1:cont_dim)=u_prev;
%% Jacobian matrix for current state

E_state=[];
B_state=[];
for j=1:Nps
    Ac=[-(cf+cr)/(M*vx_pred(j)) (cr*lr-cf*lf)/(M*(vx_pred(j)^2))-1 0 0;...
        (cr*lr-cf*lf)/Iz -(cf*lf^2+cr*lr^2)/(Iz*vx_pred(j)) 0 0;...
        vx_pred(j) 0 0 -vx_pred(j);...
        0 1 0 0];
    Bc=[cf/M/vx_pred(j); cf*lf/Iz;0;0];
    Fc=[0;0;0;-curv_r_pre(j)*vx_pred(j)];
    
    Ad(:,:,j)=eye(4)+Ts*Ac;
    Bd=Ts*Bc;
    
    Ed1(:,:,j)=(Fc)*Ts;
    E_state=[E_state;Ed1(:,:,j)];
    
    B_state=blkdiag(B_state, Bd);
end


Cc=eye(4);
Cd=Cc;

%% MPC Matrix formulation
for i=1:Nps
    for j=1:i
        if j==1
            Sui=eye(size(Ad(:,:,1)));
            Sxi=Ad(:,:,1);
        else
            Sui=[Ad(:,:,j)*Sui eye(size(Ad(:,:,1)))];
            Sxi=Ad(:,:,j)*Sxi;
        end
    end
    if i==1
        Su_A=Sui;
        Sx_A=Sxi;
    else
        Su_A=[Su_A zeros(size(Su_A,1),size(Sui,2)-size(Su_A,2));Sui];
        Sx_A=[Sx_A;Sxi];
    end
end

Su=Eye_matrix(Cd,Nps)*Su_A*B_state;
Se=Eye_matrix(Cd,Nps)*Su_A;
Sx=Eye_matrix(Cd,Nps)*Sx_A;

%% Desired output
Desired_output=[];
for i=1:Nps
    Desired_output=[Desired_output;0;0;0;0];
end

%%
tic
Hs=Su'*QS*Su+Du'*R_bars*Du;
fs=-1*Su'*QS*(Desired_output-Sx*x_prev-Se*E_state)-Du'*R_bars*Up;
U=QPhild(Hs,fs,[],[]);
Y=Sx*x_prev+Su*U+Se*E_state;
%%
steer_MPC=U(1);
steer_prev=U(2);

Ny=4;
for i=1:length(Y)
    if mod(i,Ny)==1
        if i==1
            y_prediction=Y(i:i+Ny-1);
        else
            y_prediction=[y_prediction Y(i:i+Ny-1)];
        end
    end
end

beta_prediction=y_prediction(1,:);
yawrate_prediction=y_prediction(2,:);
late_prediction=y_prediction(3,:);
yawe_prediction=y_prediction(4,:);
steer_prediction=U;

t_quad=toc;
end