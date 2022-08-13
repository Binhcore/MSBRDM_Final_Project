syms q1 q2 q3 q4 q5 q6 qp1 qp2 qp3 qp4 qp5 qp6 qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 m1 m2 m3 m4 m5 m6 t g gx gy gz beta1 beta2 beta3 beta4 beta5 beta6

syms I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333 I411 I412 I413 I422 I423 I433 I511 I512 I513 I522 I523 I533 I611 I612 I613 I622 I623 I633

% Inertia tensor wrt. fixed frame

I1 = [I111 I112 I113; % symmetric
    I112 I122 I123;
    I113 I123 I133];

I2 = [I211 I212 I213;
    I212 I222 I223;
    I213 I223 I233];

I3 = [I311 I312 I313;
    I312 I322 I323;
    I313 I323 I333];

I4= [I411 I412 I413;
    I412 I422 I423;
    I413 I423 I433];

I5= [I511 I512 I513;
    I512 I522 I523;
    I513 I523 I533];

I6= [I611 I612 I613;
    I612 I622 I623;
    I613 I623 I633];

% Viscous Friction Matrix

Beta(1,1) = beta1;
Beta(2,2) = beta2;
Beta(3,3) = beta3;
Beta(4,4) = beta4;
Beta(5,5) = beta5;
Beta(6,6) = beta6;

%Joint Position Vector
Q=[q1; q2; q3; q4; q5; q6];

% Joint velocitiesfacebook.com/
Qp = [qp1; qp2; qp3; qp4; qp5; qp6];

% Joint accelerations
Qpp = [qpp1; qpp2; qpp3; qpp4; qpp5; qpp6];

% DH Table
% THETA ALPHA a d
DH = [q1, -pi/2, 0, L1;
    q2+pi/2, 0, -L3, 0;
    q3, 0, -L5-0.1157, 0;
    q4+pi/2, pi/2, 0 , L2;
    q5, -pi/2, 0, 0.1157;
    q6, 0, 0, L4];

DH_CM = [q1, 0, 0, L6;
    q2+ pi/2, 0, -L8, L7;
    q3, 0, -L10, L2/2;
    q4, 0, 0, L2;
    q5+ pi/2, 0, 0, 0.1157;
    q6+ pi/2, 0, 0, L4];

% Specify the Robot Base (with respect to the world coordinate frame in ROS)
% Here same as robot base
T0_W=eye(4);

% Compute the Relative Homogeneous Transformations

T1_0= Relative_H(DH(1,:));

T2_1= Relative_H(DH(2,:));

T3_2= Relative_H(DH(3,:));

T4_3= Relative_H(DH(4,:));

T5_4= Relative_H(DH(5,:));

T6_5= Relative_H(DH(6,:));



Tcm1_0= Relative_H(DH_CM(1,:));

Tcm2_1= Relative_H(DH_CM(2,:));

Tcm3_2= Relative_H(DH_CM(3,:));

Tcm4_3= Relative_H(DH_CM(4,:));

Tcm5_4= Relative_H(DH_CM(5,:));

Tcm6_5= Relative_H(DH_CM(6,:));

% Stack of Absolute Homogeneous Transformations
%T1_0=0;
T2_0= T1_0*T2_1;
T3_0= T2_0*T3_2;
T4_0= T3_0*T4_3;
T5_0= T4_0*T5_4;
T6_0= T5_0*T6_5;


%Tcm1_0=0;
Tcm2_0= T1_0*Tcm2_1;
Tcm3_0= T2_0*Tcm3_2;
Tcm4_0= T3_0*Tcm4_3;
Tcm5_0= T4_0*Tcm5_4;
Tcm6_0= T5_0*Tcm6_5;

% wrt world frame, here same as the base
T1_W= T1_0;
T2_W= T2_0;
T3_W= T3_0;
T4_W= T4_0;
T5_W= T5_0;
T6_W= T6_0;

Tcm1_W= Tcm1_0;
Tcm2_W= Tcm2_0;
Tcm3_W= Tcm3_0;
Tcm4_W= Tcm4_0;
Tcm5_W= Tcm5_0;
Tcm6_W= Tcm6_0;

% Get the position of the end-effector
Xef_W=T6_W(1:3,4);
 
% Build z-vectors for Jacobian
z0= [0;0;1];
z1= T1_0(1:3,3);
z2= T2_0(1:3,3);
z3= T3_0(1:3,3);
z4= T4_0(1:3,3);
z5= T5_0(1:3,3);

% from old task
% Jcm1_0= [[cross(z0,Tcm1_0(1:3,4));z0] [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]'];
% Jcm2_0= [cross(z0, Tcm2_0(1:3,4)), z1, [0;0;0], [0;0;0] ; z0, [0;0;0], [0;0;0], [0;0;0]]; % Jacobian of cm2
% Jcm3_0= [cross(z0, Tcm3_0(1:3,4)), z1, z2, [0;0;0]; z0, [0;0;0], [0;0;0], [0;0;0]];

% Jacobians for CoM1 - CoM6
Jcm1_0 = [[cross(z0, Tcm1_0(1:3,4));z0] [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]' ];
Jcm2_0 = [[cross(z0, Tcm2_0(1:3,4));z0] [cross(z1, Tcm1_0(1:3,4)) ; z1] [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]' ];
Jcm3_0 = [[cross(z0, Tcm3_0(1:3,4));z0] [cross(z1, Tcm2_0(1:3,4)) ; z1] [cross(z2, Tcm1_0(1:3,4)) ; z2] [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]'];
Jcm4_0 = [[cross(z0, Tcm4_0(1:3,4));z0] [cross(z1, Tcm3_0(1:3,4)) ; z1] [cross(z2, Tcm2_0(1:3,4)) ; z2] [cross(z3, Tcm1_0(1:3,4)) ; z3] [0 0 0 0 0 0]' [0 0 0 0 0 0]'];
Jcm5_0 = [[cross(z0, Tcm5_0(1:3,4));z0] [cross(z1, Tcm4_0(1:3,4)) ; z1] [cross(z2, Tcm3_0(1:3,4)) ; z2] [cross(z3, Tcm2_0(1:3,4)) ; z3] [cross(z4, Tcm1_0(1:3,4)); z4] [0 0 0 0 0 0]'];
Jcm6_0 = [[cross(z0, Tcm6_0(1:3,4));z0] [cross(z1, Tcm5_0(1:3,4)) ; z1] [cross(z2, Tcm4_0(1:3,4)) ; z2] [cross(z3, Tcm3_0(1:3,4)) ; z3] [cross(z4, Tcm2_0(1:3,4)); z4] [cross(z5, Tcm1_0(1:3,4)); z5]];

% Jacobian for the end effector 
J6_0= [cross(z0,T6_0(1:3,4)), cross(z1,T6_0(1:3,4)-T1_0(1:3,4)), cross(z2, T6_0(1:3,4)-T2_0(1:3,4)), cross(z3, T6_0(1:3,4)-T3_0(1:3,4)), cross(z4, T6_0(1:3,4)-T4_0(1:3,4)), cross(z5, T6_0(1:3,4)-T5_0(1:3,4)),  ;
z0, z1, z2, z3, z4, z5 ];

% Velocity for the end effector
Xp = J6_0*Qp ;


% Inertia Matrix (symbolic form)

M1 = m1*Jcm1_0(1:3,:).'*Jcm1_0(1:3,:) + Jcm1_0(4:6,:).'*Tcm1_0(1:3,1:3)*I1*Tcm1_0(1:3,1:3).'*Jcm1_0(4:6,:);
M2 = m2*Jcm2_0(1:3,:).'*Jcm2_0(1:3,:) + Jcm2_0(4:6,:).'*Tcm2_0(1:3,1:3)*I2*Tcm2_0(1:3,1:3).'*Jcm2_0(4:6,:);
M3 = m3*Jcm3_0(1:3,:).'*Jcm3_0(1:3,:) + Jcm3_0(4:6,:).'*Tcm3_0(1:3,1:3)*I3*Tcm3_0(1:3,1:3).'*Jcm3_0(4:6,:);
M4 = m4*Jcm4_0(1:3,:).'*Jcm4_0(1:3,:) + Jcm4_0(4:6,:).'*Tcm4_0(1:3,1:3)*I4*Tcm4_0(1:3,1:3).'*Jcm4_0(4:6,:);
M5 = m5*Jcm5_0(1:3,:).'*Jcm5_0(1:3,:) + Jcm5_0(4:6,:).'*Tcm5_0(1:3,1:3)*I5*Tcm5_0(1:3,1:3).'*Jcm5_0(4:6,:);
M6 = m6*Jcm6_0(1:3,:).'*Jcm6_0(1:3,:) + Jcm6_0(4:6,:).'*Tcm6_0(1:3,1:3)*I6*Tcm6_0(1:3,1:3).'*Jcm6_0(4:6,:);

M= simplify(M1+M2+M3+M4+M5+M6);


% Complete Centripetal and Coriolis Matrix (symbolic form)


for i=1:6

% C(k,j)= 0.5*sum((diff(M(k,j),Q(i)) + diff(M(k,i),Q(j)) - diff(M(i,j),Q(k)))*Qp(i))  
Cor11(i)= (diff(M(1,1),Q(i)) + diff(M(1,i),Q(1)) - diff(M(i,1),Q(1)))*Qp(i) ; % k=1, j=1
Cor12(i)= (diff(M(1,2),Q(i)) + diff(M(1,i),Q(2)) - diff(M(i,2),Q(1)))*Qp(i) ; % k=1, j=2
Cor13(i)= (diff(M(1,3),Q(i)) + diff(M(1,i),Q(3)) - diff(M(i,3),Q(1)))*Qp(i) ; % k=1, j=3
Cor14(i)= (diff(M(1,4),Q(i)) + diff(M(1,i),Q(4)) - diff(M(i,4),Q(1)))*Qp(i) ; % k=1, j=4
Cor15(i)= (diff(M(1,5),Q(i)) + diff(M(1,i),Q(5)) - diff(M(i,5),Q(1)))*Qp(i) ; % k=1, j=5
Cor16(i)= (diff(M(1,6),Q(i)) + diff(M(1,i),Q(6)) - diff(M(i,6),Q(1)))*Qp(i) ; % k=1, j=5

Cor21(i)= (diff(M(2,1),Q(i)) + diff(M(2,i),Q(1)) - diff(M(i,1),Q(2)))*Qp(i) ; % k=2, j=1
Cor22(i)= (diff(M(2,2),Q(i)) + diff(M(2,i),Q(2)) - diff(M(i,2),Q(2)))*Qp(i) ; % k=2, j=2
Cor23(i)= (diff(M(2,3),Q(i)) + diff(M(2,i),Q(3)) - diff(M(i,3),Q(2)))*Qp(i) ; % k=2, j=3
Cor24(i)= (diff(M(2,4),Q(i)) + diff(M(2,i),Q(4)) - diff(M(i,4),Q(2)))*Qp(i) ; % k=2, j=4
Cor25(i)= (diff(M(2,5),Q(i)) + diff(M(2,i),Q(5)) - diff(M(i,5),Q(2)))*Qp(i) ; % k=2, j=5
Cor26(i)= (diff(M(2,6),Q(i)) + diff(M(2,i),Q(6)) - diff(M(i,6),Q(2)))*Qp(i) ; % k=2, j=6

Cor31(i)= (diff(M(3,1),Q(i)) + diff(M(3,i),Q(1)) - diff(M(i,1),Q(3)))*Qp(i) ; % k=3, j=1
Cor32(i)= (diff(M(3,2),Q(i)) + diff(M(3,i),Q(2)) - diff(M(i,2),Q(3)))*Qp(i) ; % k=3, j=2
Cor33(i)= (diff(M(3,3),Q(i)) + diff(M(3,i),Q(3)) - diff(M(i,3),Q(3)))*Qp(i) ; % k=3, j=3
Cor34(i)= (diff(M(3,4),Q(i)) + diff(M(3,i),Q(4)) - diff(M(i,4),Q(3)))*Qp(i) ; % k=3, j=4
Cor35(i)= (diff(M(3,5),Q(i)) + diff(M(3,i),Q(5)) - diff(M(i,5),Q(3)))*Qp(i) ; % k=3, j=5
Cor36(i)= (diff(M(3,6),Q(i)) + diff(M(3,i),Q(6)) - diff(M(i,6),Q(3)))*Qp(i) ; % k=3, j=6

Cor41(i)= (diff(M(4,1),Q(i)) + diff(M(4,i),Q(1)) - diff(M(i,1),Q(4)))*Qp(i) ; % k=4, j=1
Cor42(i)= (diff(M(4,2),Q(i)) + diff(M(4,i),Q(2)) - diff(M(i,2),Q(4)))*Qp(i) ; % k=4, j=2
Cor43(i)= (diff(M(4,3),Q(i)) + diff(M(4,i),Q(3)) - diff(M(i,3),Q(4)))*Qp(i) ; % k=4, j=3
Cor44(i)= (diff(M(4,4),Q(i)) + diff(M(4,i),Q(4)) - diff(M(i,4),Q(4)))*Qp(i) ; % k=4, j=4
Cor45(i)= (diff(M(4,5),Q(i)) + diff(M(4,i),Q(5)) - diff(M(i,5),Q(4)))*Qp(i) ; % k=4, j=5
Cor46(i)= (diff(M(4,6),Q(i)) + diff(M(4,i),Q(6)) - diff(M(i,6),Q(4)))*Qp(i) ; % k=4, j=6

Cor51(i)= (diff(M(5,1),Q(i)) + diff(M(5,i),Q(1)) - diff(M(i,1),Q(5)))*Qp(i) ; % k=5, j=1
Cor52(i)= (diff(M(5,2),Q(i)) + diff(M(5,i),Q(2)) - diff(M(i,2),Q(5)))*Qp(i) ; % k=5, j=2
Cor53(i)= (diff(M(5,3),Q(i)) + diff(M(5,i),Q(3)) - diff(M(i,3),Q(5)))*Qp(i) ; % k=5, j=3
Cor54(i)= (diff(M(5,4),Q(i)) + diff(M(5,i),Q(4)) - diff(M(i,4),Q(5)))*Qp(i) ; % k=5, j=4
Cor55(i)= (diff(M(5,5),Q(i)) + diff(M(5,i),Q(5)) - diff(M(i,5),Q(5)))*Qp(i) ; % k=5, j=5
Cor56(i)= (diff(M(5,6),Q(i)) + diff(M(5,i),Q(6)) - diff(M(i,6),Q(5)))*Qp(i) ; % k=5, j=6

Cor61(i)= (diff(M(6,1),Q(i)) + diff(M(6,i),Q(1)) - diff(M(i,1),Q(6)))*Qp(i) ; % k=6, j=1
Cor62(i)= (diff(M(6,2),Q(i)) + diff(M(6,i),Q(2)) - diff(M(i,2),Q(6)))*Qp(i) ; % k=6, j=2
Cor63(i)= (diff(M(6,3),Q(i)) + diff(M(6,i),Q(3)) - diff(M(i,3),Q(6)))*Qp(i) ; % k=6, j=3
Cor64(i)= (diff(M(6,4),Q(i)) + diff(M(6,i),Q(4)) - diff(M(i,4),Q(6)))*Qp(i) ; % k=6, j=4
Cor65(i)= (diff(M(6,5),Q(i)) + diff(M(6,i),Q(5)) - diff(M(i,5),Q(6)))*Qp(i) ; % k=6, j=5
Cor66(i)= (diff(M(6,6),Q(i)) + diff(M(6,i),Q(6)) - diff(M(i,6),Q(6)))*Qp(i) ; % k=6, j=6


end

% Stack
C(1,1)=0.5*sum(Cor11);
C(1,2)=0.5*sum(Cor12);
C(1,3)=0.5*sum(Cor13);
C(1,4)=0.5*sum(Cor14);
C(1,5)=0.5*sum(Cor15);
C(1,6)=0.5*sum(Cor16);

C(2,1)=0.5*sum(Cor21);
C(2,2)=0.5*sum(Cor22);
C(2,3)=0.5*sum(Cor23);
C(2,4)=0.5*sum(Cor24);
C(2,5)=0.5*sum(Cor25);
C(2,6)=0.5*sum(Cor26);

C(3,1)=0.5*sum(Cor31);
C(3,2)=0.5*sum(Cor32);
C(3,3)=0.5*sum(Cor33);
C(3,4)=0.5*sum(Cor34);
C(3,5)=0.5*sum(Cor35);
C(3,6)=0.5*sum(Cor36);

C(4,1)=0.5*sum(Cor41);
C(4,2)=0.5*sum(Cor42);
C(4,3)=0.5*sum(Cor43);
C(4,4)=0.5*sum(Cor44);
C(4,5)=0.5*sum(Cor45);
C(4,6)=0.5*sum(Cor46);

C(5,1)=0.5*sum(Cor51);
C(5,2)=0.5*sum(Cor52);
C(5,3)=0.5*sum(Cor53);
C(5,4)=0.5*sum(Cor54);
C(5,5)=0.5*sum(Cor55);
C(5,6)=0.5*sum(Cor56);

C(6,1)=0.5*sum(Cor61);
C(6,2)=0.5*sum(Cor62);
C(6,3)=0.5*sum(Cor63);
C(6,4)=0.5*sum(Cor64);
C(6,5)=0.5*sum(Cor65);
C(6,6)=0.5*sum(Cor66);

% Skew symmetry verification
N=verification_N(M,C);


% Potential energy of cms
Pcm1= m1*[0;0;gz].'*Tcm1_0(1:3,4);
Pcm2= m2*[0;0;gz].'*Tcm2_0(1:3,4);
Pcm3= m3*[0;0;gz].'*Tcm3_0(1:3,4);
Pcm4= m4*[0;0;gz].'*Tcm4_0(1:3,4);
Pcm5= m5*[0;0;gz].'*Tcm5_0(1:3,4);
Pcm6= m6*[0;0;gz].'*Tcm6_0(1:3,4);

P = Pcm1+Pcm2+Pcm3+Pcm4+Pcm5+Pcm6;

% Gravitational Torques Vector (symbolic form)
G=[diff(P,Q(1)); diff(P,Q(2)); diff(P,Q(3)); diff(P,Q(4)); diff(P,Q(5)); diff(P,Q(6))];


% Dynamical equation
Tao=M*Qpp+C*Qp+G;

% Load Robot Regressor
%[Y,parameter]=regressor(expand(M),expand(C),expand(G),false);

load('parameter.mat') ;
load('Regressor.mat') ;



% Dynamical equation with damping dynamics
% Qpp=(M)\(Tao-C*Qp-G-Beta*Qp);
