I_T_L = importdata('I_T_L.txt');
I_R_L = I_T_L(1:3, 1:3);
euler_IRL = rotm2eul(I_R_L, 'xyz')*180/pi
I_t_L = I_T_L(1:3, 4)'*100