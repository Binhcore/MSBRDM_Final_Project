function tau = YrTheta_UR10_3DOF(u)
% Computes tau based on Regressor Matrix Y and Parameter Vector Theta.
% Y and Theta are automatically formed by 'robot_regressor.m' which is used
% in 'robot_ur10_anaysis.m'.
% This function is called by 'Dinamic_ur10_3DOF.m' to check if the
% regressor produces correct torques
% Input:
%   Vector u, containing all roboter states, lenght, masses and inertias
% Ouput:


    %% Extract parameters
    
    %Kinematic Parameters
    L1=u(1);
    L2=u(2);
    L3=u(3);
    L4=u(4);
    L5=u(5);
    L6=u(6);
    L7=u(7);
    L8=u(8);
    L9=u(9);
    L10=u(10);
    L11=L2-0.08;

    %Dynamic Parameters
    m1=u(11);
    m2=u(12);
    m3=u(13);

    I111=u(14);
    I112=u(15);
    I113=u(16);
    I122=u(17);
    I123=u(18);
    I133=u(19);

    I211=u(20);
    I212=u(21);
    I213=u(22);
    I222=u(23);
    I223=u(24);
    I233=u(25);

    I311=u(26);
    I312=u(27);
    I313=u(28);
    I322=u(29);
    I323=u(30);
    I333=u(31);

    %Gravity
    g=u(32);

    %Time
    t=u(33)

    %Gravity Vector
    gx=u(37);
    gy=u(38);
    gz=u(39);

    %Position
    q1=u(40);
    q2=u(41);
    q3=u(42);

    %Velocity
    q1p=u(43);
    q2p=u(44);
    q3p=u(45);

    %Reference Acceleration
    qr1p=u(46);
    qr2p=u(47);
    qr3p=u(48);

    %Reference Velocity
    qr1pp=u(49);
    qr2pp=u(50);
    qr3pp=u(51);

    % compute correct magnitue of gravity vector
    gn = g * [gx, gy, gz] ./ norm([gx, gy, gz]);
    gx=gn(1);
    gy=gn(2);
    gz=gn(3);

    %% Define parameter vector
    Theta(1,1)=I133;
    Theta(2,1)=I211;
    Theta(3,1)=I212;
    Theta(4,1)=I213;
    Theta(5,1)=I222;
    Theta(6,1)=I223;
    Theta(7,1)=I233;
    Theta(8,1)=I311;
    Theta(9,1)=I312;
    Theta(10,1)=I313;
    Theta(11,1)=I322;
    Theta(12,1)=I323;
    Theta(13,1)=I333;
    Theta(14,1)=L2^2*m3;
    Theta(15,1)=L3^2*m3;
    Theta(16,1)=L7^2*m2;
    Theta(17,1)=L8^2*m2;
    Theta(18,1)=L10^2*m3;
    Theta(19,1)=L11^2*m3;
    Theta(20,1)=L2*L3*m3;
    Theta(21,1)=L2*L10*m3;
    Theta(22,1)=L2*L11*m3;
    Theta(23,1)=L3*L10*m3;
    Theta(24,1)=L3*L11*m3;
    Theta(25,1)=L7*L8*m2;
    Theta(26,1)=L10*L11*m3;
    Theta(27,1)=L2*gx*m3;
    Theta(28,1)=L3*gx*m3;
    Theta(29,1)=L7*gx*m2;
    Theta(30,1)=L8*gx*m2;
    Theta(31,1)=L10*gx*m3;
    Theta(32,1)=L11*gx*m3;
    Theta(33,1)=L2*gy*m3;
    Theta(34,1)=L3*gy*m3;
    Theta(35,1)=L7*gy*m2;
    Theta(36,1)=L8*gy*m2;
    Theta(37,1)=L10*gy*m3;
    Theta(38,1)=L11*gy*m3;
    Theta(39,1)=L3*gz*m3;
    Theta(40,1)=L8*gz*m2;
    Theta(41,1)=L10*gz*m3;

    %% Define the matrix of states
    Yr = [[qr1pp, qr1pp/2 + (qr1pp*cos(2*q2))/2 - (q1p*qr2p*sin(2*q2))/2 - (q2p*qr1p*sin(2*q2))/2, - qr1pp*sin(2*q2) - q1p*qr2p*cos(2*q2) - q2p*qr1p*cos(2*q2), qr2pp*cos(q2) - q2p*qr2p*sin(q2), qr1pp/2 - (qr1pp*cos(2*q2))/2 + (q1p*qr2p*sin(2*q2))/2 + (q2p*qr1p*sin(2*q2))/2, - qr2pp*sin(q2) - q2p*qr2p*cos(q2), 0, qr1pp/2 + (qr1pp*cos(2*q2 - 2*q3))/2 - (q1p*qr2p*sin(2*q2 - 2*q3))/2 - (q2p*qr1p*sin(2*q2 - 2*q3))/2 + (q1p*qr3p*sin(2*q2 - 2*q3))/2 + (q3p*qr1p*sin(2*q2 - 2*q3))/2, qr1pp*sin(2*q2 - 2*q3) + q1p*qr2p*cos(2*q2 - 2*q3) + q2p*qr1p*cos(2*q2 - 2*q3) - q1p*qr3p*cos(2*q2 - 2*q3) - q3p*qr1p*cos(2*q2 - 2*q3), qr3pp*cos(q2 - q3) - qr2pp*cos(q2 - q3) + q2p*qr2p*sin(q2 - q3) - q2p*qr3p*sin(q2 - q3) - q3p*qr2p*sin(q2 - q3) + q3p*qr3p*sin(q2 - q3), qr1pp/2 - (qr1pp*cos(2*q2 - 2*q3))/2 + (q1p*qr2p*sin(2*q2 - 2*q3))/2 + (q2p*qr1p*sin(2*q2 - 2*q3))/2 - (q1p*qr3p*sin(2*q2 - 2*q3))/2 - (q3p*qr1p*sin(2*q2 - 2*q3))/2, qr3pp*sin(q2 - q3) - qr2pp*sin(q2 - q3) - q2p*qr2p*cos(q2 - q3) + q2p*qr3p*cos(q2 - q3) + q3p*qr2p*cos(q2 - q3) - q3p*qr3p*cos(q2 - q3), 0, qr1pp, qr1pp/2 - (qr1pp*cos(2*q2))/2 + (q1p*qr2p*sin(2*q2))/2 + (q2p*qr1p*sin(2*q2))/2, qr1pp, qr1pp/2 - (qr1pp*cos(2*q2))/2 + (q1p*qr2p*sin(2*q2))/2 + (q2p*qr1p*sin(2*q2))/2, qr1pp/2 - (qr1pp*cos(2*q2 - 2*q3))/2 + (q1p*qr2p*sin(2*q2 - 2*q3))/2 + (q2p*qr1p*sin(2*q2 - 2*q3))/2 - (q1p*qr3p*sin(2*q2 - 2*q3))/2 - (q3p*qr1p*sin(2*q2 - 2*q3))/2, qr1pp, q2p*qr2p*sin(q2) - qr2pp*cos(q2), qr3pp*cos(q2 - q3) - qr2pp*cos(q2 - q3) + q2p*qr2p*sin(q2 - q3) - q2p*qr3p*sin(q2 - q3) - q3p*qr2p*sin(q2 - q3) + q3p*qr3p*sin(q2 - q3), -2*qr1pp, qr1pp*cos(q3) - qr1pp*cos(2*q2 - q3) - (q1p*qr3p*sin(q3))/2 - (q3p*qr1p*sin(q3))/2 + q1p*qr2p*sin(2*q2 - q3) + q2p*qr1p*sin(2*q2 - q3) - (q1p*qr3p*sin(2*q2 - q3))/2 - (q3p*qr1p*sin(2*q2 - q3))/2, qr2pp*cos(q2) - q2p*qr2p*sin(q2), q2p*qr2p*sin(q2) - qr2pp*cos(q2), qr2pp*cos(q2 - q3) - qr3pp*cos(q2 - q3) - q2p*qr2p*sin(q2 - q3) + q2p*qr3p*sin(q2 - q3) + q3p*qr2p*sin(q2 - q3) - q3p*qr3p*sin(q2 - q3), -cos(q1), -sin(q1)*sin(q2), -cos(q1), -sin(q1)*sin(q2), -sin(q2 - q3)*sin(q1), cos(q1), -sin(q1), cos(q1)*sin(q2), -sin(q1), cos(q1)*sin(q2), sin(q2 - q3)*cos(q1), sin(q1), 0, 0, 0];...
          [0, (q1p*qr1p*sin(2*q2))/2, q1p*qr1p*cos(2*q2), qr1pp*cos(q2), -(q1p*qr1p*sin(2*q2))/2, -qr1pp*sin(q2), qr2pp, (q1p*qr1p*sin(2*q2 - 2*q3))/2, -q1p*qr1p*cos(2*q2 - 2*q3), -qr1pp*cos(q2 - q3), -(q1p*qr1p*sin(2*q2 - 2*q3))/2, -qr1pp*sin(q2 - q3), qr2pp - qr3pp, 0, qr2pp - (q1p*qr1p*sin(2*q2))/2, 0, qr2pp - (q1p*qr1p*sin(2*q2))/2, qr2pp - qr3pp - (q1p*qr1p*sin(2*q2 - 2*q3))/2, 0, -qr1pp*cos(q2), -qr1pp*cos(q2 - q3), 0, 2*qr2pp*cos(q3) - qr3pp*cos(q3) - q2p*qr3p*sin(q3) - q3p*qr2p*sin(q3) + q3p*qr3p*sin(q3) - q1p*qr1p*sin(2*q2 - q3), qr1pp*cos(q2), -qr1pp*cos(q2), qr1pp*cos(q2 - q3), 0, cos(q1)*cos(q2), 0, cos(q1)*cos(q2), cos(q2 - q3)*cos(q1), 0, 0, cos(q2)*sin(q1), 0, cos(q2)*sin(q1), cos(q2 - q3)*sin(q1), 0, -sin(q2), -sin(q2), -sin(q2 - q3)];...
          [0, 0, 0, 0, 0, 0, 0, -(q1p*qr1p*sin(2*q2 - 2*q3))/2, q1p*qr1p*cos(2*q2 - 2*q3), qr1pp*cos(q2 - q3), (q1p*qr1p*sin(2*q2 - 2*q3))/2, qr1pp*sin(q2 - q3), qr3pp - qr2pp, 0, 0, 0, 0, qr3pp - qr2pp + (q1p*qr1p*sin(2*q2 - 2*q3))/2, 0, 0, qr1pp*cos(q2 - q3), 0, (q1p*qr1p*sin(q3))/2 - qr2pp*cos(q3) + q2p*qr2p*sin(q3) + (q1p*qr1p*sin(2*q2 - q3))/2, 0, 0, -qr1pp*cos(q2 - q3), 0, 0, 0, 0, -cos(q2 - q3)*cos(q1), 0, 0, 0, 0, 0, -cos(q2 - q3)*sin(q1), 0, 0, 0, sin(q2 - q3)]];
    
    % Compute tau
    tau = Yr*Theta;
end