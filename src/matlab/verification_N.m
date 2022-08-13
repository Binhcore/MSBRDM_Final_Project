function N=verification_N(M,C)

%% Check M+ = transpose(M+)

% [r,c] = size(M);
% Pos_M = M >= 0 ;
% 
% if sum(Pos_M,'all') >= r*c
%     disp('M is positive.')
% else 
%     disp ('M is not positive.')
% end


% verification symmetry
if M(1,2) == M(2,1) && M(1,3) == M(3,1) && M(1,4) == M(4,1) && M(1,5) == M(5,1) && M(1,6) == M(6,1) && M(2,3) == M(3,2) && M(2,4) == M(4,2) && M(2,5) == M(5,2) && M(2,6) == M(6,2) && M(3,4) == M(4,3) && M(3,5) == M(5,3) && M(3,6) == M(6,3) && M(4,5) == M(5,4) && M(4,6) == M(6,4) && M(5,6) == M(6,5)
    disp('First verification successfull. M is symmetric.')
else 
    disp('First verification failed. M is not symmetric.')
end

%% Check N = Md - 2C
syms theta1t(t) theta2t(t) theta3t(t) theta4t(t) theta5t(t) theta6t(t) q1 q2 q3 q4 q5 q6 qp1 qp2 qp3 qp4 qp5 qp6

M_sub = subs(M, [q1 q2 q3 q4 q5 q6], [theta1t theta2t theta3t theta4t theta5t theta6t]); % preparation/substitution for differentiation of M
dM_sub = diff(M_sub,t); % differentiation of M with respect to time
dM = subs(dM_sub, [theta1t theta2t theta3t theta4t theta5t theta6t diff(theta1t(t),t) diff(theta2t(t),t) diff(theta3t(t),t) diff(theta4t(t),t) diff(theta5t(t),t) diff(theta6t(t),t)], [q1 q2 q3 q4 q5 q6 qp1 qp2 qp3 qp4 qp5 qp6]); % back substitution
N = dM - 2*C; % N matrix

%% Check Nij=-Nji
if simplify(N(1,2)+N(2,1)) == 0 && simplify(N(1,3)+N(3,1)) == 0 && simplify(N(1,4)+N(4,1))==0 && simplify(N(1,5)+N(5,1)) == 0 && simplify(N(1,6)+N(6,1)) == 0 && simplify(N(2,3)+N(3,2))==0 && simplify(N(2,4)+N(4,2))==0 && simplify(N(2,5)+N(5,2))==0 && simplify(N(2,6)+N(6,2))==0 && simplify(N(3,4)+N(4,3))==0 && simplify(N(3,5)+N(5,3))==0 && simplify(N(3,6)+N(6,3))==0 && simplify(N(4,5)+N(5,4))==0 && simplify(N(4,6)+N(6,4))==0 && simplify(N(5,6)+N(6,5))==0
    disp('Second verification successfull. N is skew symmetric.')
else 
    disp('Second verification failed. N is not skew symmetric.')
end


%% Check xT*N*x=0 with x is any vector from R(6,1)
vector= rand(6,1);
skew=simplify(vector'*N*vector);

if skew==0
    disp('Third verification successfull. N is skew symmetric.')
end





end

