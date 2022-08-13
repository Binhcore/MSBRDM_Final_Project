

syms q1r q2r q3r qp1r qp2r qp3r qpp1r qpp2r qpp3r

syms a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21 a22 a23 a24 a25 a26 a27 a28 a29 a30 a31 a32 a33 a34 a35 a36 a37 a38 a39 a40

syms q1 q2 q3 qp1 qp2 qp3 qpp1 qpp2 qpp3 L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 m1 m2 m3 t g gx gy gz beta1 beta2 beta3 I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333

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

% Viscous Friction Matrix

Beta(1,1) = beta1;
Beta(2,2) = beta2;
Beta(3,3) = beta3;

%Joint Position Vector
Q=[q1; q2; q3];

% Joint velocities
Qp = [qp1; qp2; qp3];

% Joint accelerations
Qpp = [qpp1; qpp2; qpp3];

% DH Table
% THETA ALPHA a d
DH = [q1, -pi/2, 0, L1;
    q2+pi/2, pi, -L3, L2;
    q3, 0, -L5, 0;
    q1, 0, 0, L6; % CM1
    q2+pi/2, 0, -L8, L7; % CM2
    q3, 0, -L10, 0.5*L2]; % CM3

% Specify the Robot Base (with respect to the world coordinate frame in ROS)
% Here same as robot base
T0_W=eye(4);

% Compute the Relative Homogeneous Transformations

T1_0= Relative_H(DH(1,:));

T2_1= Relative_H(DH(2,:));

T3_2= Relative_H(DH(3,:));

Tcm1_0= Relative_H(DH(4,:));

Tcm2_1= Relative_H(DH(5,:));

Tcm3_2= Relative_H(DH(6,:));

% Stack of Absolute Homogeneous Transformations
%T1_0=0;
T2_0= T1_0*T2_1;
T3_0= T2_0*T3_2;


%Tcm1_0=0;
Tcm2_0= T1_0*Tcm2_1;
Tcm3_0= T2_0*Tcm3_2;

% wrt world frame, here same as the base
T1_W= T1_0;
T2_W= T2_0;
T3_W= T3_0;

Tcm1_W= Tcm1_0;
Tcm2_W= Tcm2_0;
Tcm3_W= Tcm3_0;

% Get the position of the end-effector
Xef_W=T3_W(1:3,4);
 
% Build z-vectors for Jacobian
z0= [0;0;1];
z1= T1_0(1:3,3);
z2= T2_0(1:3,3);

% from old task
% Jcm1_0= [[cross(z0,Tcm1_0(1:3,4));z0] [0 0 0 0 0 0]' [0 0 0 0 0 0]' [0 0 0 0 0 0]'];
% Jcm2_0= [cross(z0, Tcm2_0(1:3,4)), z1, [0;0;0], [0;0;0] ; z0, [0;0;0], [0;0;0], [0;0;0]]; % Jacobian of cm2
% Jcm3_0= [cross(z0, Tcm3_0(1:3,4)), z1, z2, [0;0;0]; z0, [0;0;0], [0;0;0], [0;0;0]];

% Jacobians for CoM1, CoM2 and CoM3, different approach than for coordinate
% frames
Jcm1_0 = [[cross(z0, Tcm1_0(1:3,4));z0] [0 0 0 0 0 0]' [0 0 0 0 0 0]' ];
Jcm2_0 = [[cross(z0, Tcm2_0(1:3,4));z0] [cross(z1, Tcm1_0(1:3,4)) ; z1] [0 0 0 0 0 0]' ];
Jcm3_0 = [[cross(z0, Tcm3_0(1:3,4));z0] [cross(z1, Tcm2_0(1:3,4)) ; z1] [cross(z2, Tcm1_0(1:3,4)) ; z2]];

% Jacobian for the end effector 
J3_0= [cross(z0,T3_0(1:3,4)), cross(z1,T3_0(1:3,4)-T1_0(1:3,4)), cross(z2, T3_0(1:3,4)-T2_0(1:3,4));
z0, z1, z2 ];

% Velocity for the end effector
Xp = J3_0*Qp ;


% Inertia Matrix (symbolic form)

M1 = m1*Jcm1_0(1:3,1:3).'*Jcm1_0(1:3,1:3) + Jcm1_0(4:6,1:3).'*Tcm1_0(1:3,1:3)*I1*Tcm1_0(1:3,1:3).'*Jcm1_0(4:6,1:3);
M2 = m2*Jcm2_0(1:3,1:3).'*Jcm2_0(1:3,1:3) + Jcm2_0(4:6,1:3).'*Tcm2_0(1:3,1:3)*I2*Tcm2_0(1:3,1:3).'*Jcm2_0(4:6,1:3);
M3 = m3*Jcm3_0(1:3,1:3).'*Jcm3_0(1:3,1:3) + Jcm3_0(4:6,1:3).'*Tcm3_0(1:3,1:3)*I3*Tcm3_0(1:3,1:3).'*Jcm3_0(4:6,1:3);

M= simplify(M1+M2+M3);


% Complete Centripetal and Coriolis Matrix (symbolic form)


for i=1:3

% C(k,j)= 0.5*sum((diff(M(k,j),Q(i)) + diff(M(k,i),Q(j)) - diff(M(i,j),Q(k)))*Qp(i))  
Cor1(i)= (diff(M(1,1),Q(i)) + diff(M(1,i),Q(1)) - diff(M(i,1),Q(1)))*Qp(i) ; % k=1, j=1
Cor2(i)= (diff(M(1,2),Q(i)) + diff(M(1,i),Q(2)) - diff(M(i,2),Q(1)))*Qp(i) ; % k=1, j=2
Cor3(i)= (diff(M(1,3),Q(i)) + diff(M(1,i),Q(3)) - diff(M(i,3),Q(1)))*Qp(i) ; % k=1, j=3

Cor4(i)= (diff(M(2,1),Q(i)) + diff(M(2,i),Q(1)) - diff(M(i,1),Q(2)))*Qp(i) ; % k=2, j=1
Cor5(i)= (diff(M(2,2),Q(i)) + diff(M(2,i),Q(2)) - diff(M(i,2),Q(2)))*Qp(i) ; % k=2, j=2
Cor6(i)= (diff(M(2,3),Q(i)) + diff(M(2,i),Q(3)) - diff(M(i,3),Q(2)))*Qp(i) ; % k=2, j=3

Cor7(i)= (diff(M(3,1),Q(i)) + diff(M(3,i),Q(1)) - diff(M(i,1),Q(3)))*Qp(i) ; % k=3, j=1
Cor8(i)= (diff(M(3,2),Q(i)) + diff(M(3,i),Q(2)) - diff(M(i,2),Q(3)))*Qp(i) ; % k=3, j=2
Cor9(i)= (diff(M(3,3),Q(i)) + diff(M(3,i),Q(3)) - diff(M(i,3),Q(3)))*Qp(i) ; % k=3, j=3

end

% Stack
C(1,1)=0.5*sum(Cor1);
C(1,2)=0.5*sum(Cor2);
C(1,3)=0.5*sum(Cor3);
C(2,1)=0.5*sum(Cor4);
C(2,2)=0.5*sum(Cor5);
C(2,3)=0.5*sum(Cor6);
C(3,1)=0.5*sum(Cor7);
C(3,2)=0.5*sum(Cor8);
C(3,3)=0.5*sum(Cor9);

% Skew symmetry verification
N=verification_N(M,C);


% Potential energy of cms
Pcm1= m1*[0;0;gz].'*Tcm1_0(1:3,4);
Pcm2= m2*[0;0;gz].'*Tcm2_0(1:3,4);
Pcm3= m3*[0;0;gz].'*Tcm3_0(1:3,4);
P = Pcm1+Pcm2+Pcm3;

% Gravitational Torques Vector (symbolic form)
G=[diff(P,Q(1)); diff(P,Q(2)); diff(P,Q(3))];

% r index for distinction
Qr=[q1r;q2r;q3r];

Qpr=[qp1r;qp2r;qp3r];

Qppr=[qpp1r;qpp2r;qpp3r];

TAU= M*Qppr+C*Qpr+G;

%%
TAU_ex= expand(TAU);

% 

eqns=subs(TAU_ex, [m1 m2 m3 L1 L2 L3 L5 L6 L7 L8 L10 gz I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333], [a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21 a22 a23 a24 a25 a26 a27 a28 a29 a30]);

eqns2=subs(TAU, [m1 m2 m3 L1 L2 L3 L5 L6 L7 L8 L10 gz I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333], [a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21 a22 a23 a24 a25 a26 a27 a28 a29 a30]);

% TAU
Theta(1,1) = a1;

[A(1),b(1)]= equationsToMatrix(eqns(3),a1);

TAU_new(1)= b(1)+A(1);

Theta(2,1) = a2;

[A(2),b(2)]= equationsToMatrix(TAU_new(1),a2);

TAU_new(2) = b(2) + A(2);

TAU_new(2)= TAU_new(1)-coeffs(TAU_new(1), a3)-equationsToMatrix(TAU_new(1), a3);

Theta(3,1) = a5;

TAU_new(3)= coeffs(TAU_new(2),a4)-equationsToMatrix(TAU_new(2),a4);

Theta(4,1) = a6;

TAU_new(4)= coeffs(TAU_new(3),a5)-equationsToMatrix(TAU_new(3),a5);









