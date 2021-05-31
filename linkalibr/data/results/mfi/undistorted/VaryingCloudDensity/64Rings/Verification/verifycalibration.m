clear;
clc;

I_T_L_far1 = importdata('I_T_L_far1.txt');
I_R_L_far1 = I_T_L_far1(1:3, 1:3);
euler_I_R_L_far1 = rotm2eul(I_R_L_far1, 'xyz')*180/pi;
I_t_L_far1 = I_T_L_far1(1:3, 4)';
far1 = [euler_I_R_L_far1, I_t_L_far1]

I_T_L_far2 = importdata('I_T_L_far2.txt');
I_R_L_far2 = I_T_L_far2(1:3, 1:3);
euler_I_R_L_far2 = rotm2eul(I_R_L_far2, 'xyz')*180/pi;
I_t_L_far2 = I_T_L_far2(1:3, 4)';
far2 = [euler_I_R_L_far2, I_t_L_far2]

I_T_L_near1 = importdata('I_T_L_near1.txt');
I_R_L_near1 = I_T_L_near1(1:3, 1:3);
euler_I_R_L_near1 = rotm2eul(I_R_L_near1, 'xyz')*180/pi;
I_t_L_near1 = I_T_L_near1(1:3, 4)';
near1 = [euler_I_R_L_near1, I_t_L_near1]

I_T_L_near2 = importdata('I_T_L_near2.txt');
I_R_L_near2 = I_T_L_near2(1:3, 1:3);
euler_I_R_L_near2 = rotm2eul(I_R_L_near2, 'xyz')*180/pi;
I_t_L_near2 = I_T_L_near2(1:3, 4)';
near2 = [euler_I_R_L_near2, I_t_L_near2]

fprintf('Difference b/w far and near');

near1_I_T_L_far1 = inv(I_T_L_near1)*I_T_L_far1;
near1_I_R_L_far1 = near1_I_T_L_far1(1:3, 1:3);
euler_near1_I_R_L_far1 = rotm2eul(near1_I_R_L_far1, 'xyz')*180/pi;
near1_I_t_L_far1 = near1_I_T_L_far1(1:3, 4)';
diff_near1_far1 = [euler_near1_I_R_L_far1, near1_I_t_L_far1]

near1_I_T_L_far2 = inv(I_T_L_near1)*I_T_L_far2;
near1_I_R_L_far2 = near1_I_T_L_far2(1:3, 1:3);
euler_near1_I_R_L_far2 = rotm2eul(near1_I_R_L_far2, 'xyz')*180/pi;
near1_I_t_L_far2 = near1_I_T_L_far2(1:3, 4)';
diff_near1_far2 = [euler_near1_I_R_L_far2, near1_I_t_L_far2]

near2_I_T_L_far1 = inv(I_T_L_near2)*I_T_L_far1;
near2_I_R_L_far1 = near2_I_T_L_far1(1:3, 1:3);
euler_near2_I_R_L_far1 = rotm2eul(near2_I_R_L_far1, 'xyz')*180/pi;
near2_I_t_L_far1 = near2_I_T_L_far1(1:3, 4)';
diff_near2_far1 = [euler_near2_I_R_L_far1, near2_I_t_L_far1]

near2_I_T_L_far2 = inv(I_T_L_near2)*I_T_L_far2;
near2_I_R_L_far2 = near2_I_T_L_far2(1:3, 1:3);
euler_near2_I_R_L_far2 = rotm2eul(near2_I_R_L_far2, 'xyz')*180/pi;
near2_I_t_L_far2 = near2_I_T_L_far2(1:3, 4)';
diff_near2_far2 = [euler_near2_I_R_L_far2, near2_I_t_L_far2]

% I_R_L_far1 = I_T_L_far1(1:3, 1:3);
% euler_I_R_L_far1 = rotm2eul(I_R_L_far1, 'xyz')*180/pi
% I_t_L_far1 = I_T_L_far1(1:3, 4)'*100