function [R]=Trans_vect_to_matrix(r_vec)
 theta = norm(r_vec,2);
 r_vec = r_vec / theta ;
 A=eye(3);
 temp1=r_vec'*r_vec;
 temp2 = [0       -r_vec(3)  r_vec(2);
         r_vec(3)    0       -r_vec(1);
         -r_vec(2)  r_vec(1)    0      ];
 R=cos(theta)*A+(1-cos(theta))*temp1+sin(theta)*temp2;
    
 