function T_rel = Relative_H(DH)

% [r, c]= size(DH)
% 
% for i=1:r
% 
% T_rel(:,:,i) = [cos(DH(i,1)), -sin(DH(i,1))*cos(DH(i,2)), sin(DH(i,1))*sin(DH(i,2)), DH(i,3)*cos(DH(i,1));
%     sin(DH(i,1)), cos(DH(i,1))*cos(DH(i,2)), -cos(DH(i,1))*sin(DH(i,2)), DH(i,3)*sin(DH(i,1));
%     0, sin(DH(i,2)), cos(DH(i,2)), DH(i,4);
%     0, 0, 0, 1];
% end

T_rel= [cos(DH(1,1)), -sin(DH(1,1))*cos(DH(1,2)), sin(DH(1,1))*sin(DH(1,2)), DH(1,3)*cos(DH(1,1));
    sin(DH(1,1)), cos(DH(1,1))*cos(DH(1,2)), -cos(DH(1,1))*sin(DH(1,2)), DH(1,3)*sin(DH(1,1));
    0, sin(DH(1,2)), cos(DH(1,2)), DH(1,4);
    0, 0, 0, 1];


end

