syms q1r q2r q3r qp1r qp2r qp3r qpp1r qpp2r qpp3r

syms a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21 a22 a23 a24 a25 a26 a27 a28 a29 a30 a31 a32 a33 a34 a35 a36 a37 a38 a39 a40

syms q1 q2 q3 qp1 qp2 qp3 qpp1 qpp2 qpp3 L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 m1 m2 m3 t g gx gy gz beta1 beta2 beta3 I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333

syms new1 new2 new3

eqns= [2*a1*a2*cos(q1)+a3/2*sin(q2)+a3*a4*sin(q2)*cos(q3+pi/2);
    4*qp1*qp1r*a5+qpp3r*cos(q3)*sin(q3);
    a1*a4*cos(q2)^2*cos(q3)+a3^2*sin(q1)+a3^2*sin(q2)];

Theta(1,1) = a1;

[A(1),b(1)]= equationsToMatrix(eqns(3),a1);

TAU_new(1)= A(1)-b(1);

% Theta(1,1) = a1;
% 
% [A,b]= equationsToMatrix(eqns,a1);
% 
% TAU_new(:,1)= b(1)+A(1);
% 
% Theta(2,1) = a2;
% 
% [A1,b1]= equationsToMatrix(TAU_new(:,1),a2);
% 
% TAU_new(:,2) = b(2) + A(2);