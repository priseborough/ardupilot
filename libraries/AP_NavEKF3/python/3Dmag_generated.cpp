// Equations for 3D mag fusion
// X axis innovation variance
float SX0 = 2*q3;
float SX1 = 2*q2;
float SX2 = SX0*q0 + SX1*q1;
float SX3 = SX0*q1 - SX1*q0;
float SX4 = 2*magN;
float SX5 = SX0*magE - SX1*magD + SX4*q0;
float SX6 = SX0*magD + SX1*magE + SX4*q1;
float SX7 = 2*magD;
float SX8 = 2*magE;
float SX9 = -SX1*magN - SX7*q0 + SX8*q1;
float SX10 = -SX0*magN + SX7*q1 + SX8*q0;
float SX11 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);


innov_var_x = P[0][19]*SX5 + P[16][19]*SX11 + P[17][19]*SX2 + P[18][19]*SX3 + P[19][19] + P[1][19]*SX6 + P[2][19]*SX9 + P[3][19]*SX10 + R_MAG + SX10*(P[0][3]*SX5 + P[1][3]*SX6 + P[2][3]*SX9 + P[3][16]*SX11 + P[3][17]*SX2 + P[3][18]*SX3 + P[3][19] + P[3][3]*SX10) + SX11*(P[0][16]*SX5 + P[16][16]*SX11 + P[16][17]*SX2 + P[16][18]*SX3 + P[16][19] + P[1][16]*SX6 + P[2][16]*SX9 + P[3][16]*SX10) + SX2*(P[0][17]*SX5 + P[16][17]*SX11 + P[17][17]*SX2 + P[17][18]*SX3 + P[17][19] + P[1][17]*SX6 + P[2][17]*SX9 + P[3][17]*SX10) + SX3*(P[0][18]*SX5 + P[16][18]*SX11 + P[17][18]*SX2 + P[18][18]*SX3 + P[18][19] + P[1][18]*SX6 + P[2][18]*SX9 + P[3][18]*SX10) + SX5*(P[0][0]*SX5 + P[0][16]*SX11 + P[0][17]*SX2 + P[0][18]*SX3 + P[0][19] + P[0][1]*SX6 + P[0][2]*SX9 + P[0][3]*SX10) + SX6*(P[0][1]*SX5 + P[1][16]*SX11 + P[1][17]*SX2 + P[1][18]*SX3 + P[1][19] + P[1][1]*SX6 + P[1][2]*SX9 + P[1][3]*SX10) + SX9*(P[0][2]*SX5 + P[1][2]*SX6 + P[2][16]*SX11 + P[2][17]*SX2 + P[2][18]*SX3 + P[2][19] + P[2][2]*SX9 + P[2][3]*SX10);


// X axis observation matrix and kalman gain
float HKX0 = 2*magD;
float HKX1 = 2*magE;
float HKX2 = 2*magN;
float HKX3 = -HKX0*q2 + HKX1*q3 + HKX2*q0;
float HKX4 = HKX0*q3 + HKX1*q2 + HKX2*q1;
float HKX5 = -HKX0*q0 + HKX1*q1 - HKX2*q2;
float HKX6 = HKX0*q1 + HKX1*q0 - HKX2*q3;
float HKX7 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float HKX8 = 2*q0;
float HKX9 = 2*q1;
float HKX10 = HKX8*q3 + HKX9*q2;
float HKX11 = -HKX8*q2 + HKX9*q3;
float HKX12 = HKX10*P[0][17] + HKX11*P[0][18] + HKX3*P[0][0] + HKX4*P[0][1] + HKX5*P[0][2] + HKX6*P[0][3] + HKX7*P[0][16] + P[0][19];
float HKX13 = HKX10*P[17][18] + HKX11*P[18][18] + HKX3*P[0][18] + HKX4*P[1][18] + HKX5*P[2][18] + HKX6*P[3][18] + HKX7*P[16][18] + P[18][19];
float HKX14 = HKX10*P[17][17] + HKX11*P[17][18] + HKX3*P[0][17] + HKX4*P[1][17] + HKX5*P[2][17] + HKX6*P[3][17] + HKX7*P[16][17] + P[17][19];
float HKX15 = HKX10*P[2][17] + HKX11*P[2][18] + HKX3*P[0][2] + HKX4*P[1][2] + HKX5*P[2][2] + HKX6*P[2][3] + HKX7*P[2][16] + P[2][19];
float HKX16 = HKX10*P[3][17] + HKX11*P[3][18] + HKX3*P[0][3] + HKX4*P[1][3] + HKX5*P[2][3] + HKX6*P[3][3] + HKX7*P[3][16] + P[3][19];
float HKX17 = HKX10*P[1][17] + HKX11*P[1][18] + HKX3*P[0][1] + HKX4*P[1][1] + HKX5*P[1][2] + HKX6*P[1][3] + HKX7*P[1][16] + P[1][19];
float HKX18 = HKX10*P[16][17] + HKX11*P[16][18] + HKX3*P[0][16] + HKX4*P[1][16] + HKX5*P[2][16] + HKX6*P[3][16] + HKX7*P[16][16] + P[16][19];
float HKX19 = HKX10*P[17][19] + HKX11*P[18][19] + HKX3*P[0][19] + HKX4*P[1][19] + HKX5*P[2][19] + HKX6*P[3][19] + HKX7*P[16][19] + P[19][19];
float HKX20 = 1.0F/(HKX10*HKX14 + HKX11*HKX13 + HKX12*HKX3 + HKX15*HKX5 + HKX16*HKX6 + HKX17*HKX4 + HKX18*HKX7 + HKX19 + R_MAG);


H_MAG[0] = HKX3;
H_MAG[1] = HKX4;
H_MAG[2] = HKX5;
H_MAG[3] = HKX6;
H_MAG[4] = 0;
H_MAG[5] = 0;
H_MAG[6] = 0;
H_MAG[7] = 0;
H_MAG[8] = 0;
H_MAG[9] = 0;
H_MAG[10] = 0;
H_MAG[11] = 0;
H_MAG[12] = 0;
H_MAG[13] = 0;
H_MAG[14] = 0;
H_MAG[15] = 0;
H_MAG[16] = HKX7;
H_MAG[17] = HKX10;
H_MAG[18] = HKX11;
H_MAG[19] = 1;
H_MAG[20] = 0;
H_MAG[21] = 0;
H_MAG[22] = 0;
H_MAG[23] = 0;


Kfusion[0] = HKX12*HKX20;
Kfusion[1] = HKX17*HKX20;
Kfusion[2] = HKX15*HKX20;
Kfusion[3] = HKX16*HKX20;
Kfusion[4] = HKX20*(HKX10*P[4][17] + HKX11*P[4][18] + HKX3*P[0][4] + HKX4*P[1][4] + HKX5*P[2][4] + HKX6*P[3][4] + HKX7*P[4][16] + P[4][19]);
Kfusion[5] = HKX20*(HKX10*P[5][17] + HKX11*P[5][18] + HKX3*P[0][5] + HKX4*P[1][5] + HKX5*P[2][5] + HKX6*P[3][5] + HKX7*P[5][16] + P[5][19]);
Kfusion[6] = HKX20*(HKX10*P[6][17] + HKX11*P[6][18] + HKX3*P[0][6] + HKX4*P[1][6] + HKX5*P[2][6] + HKX6*P[3][6] + HKX7*P[6][16] + P[6][19]);
Kfusion[7] = HKX20*(HKX10*P[7][17] + HKX11*P[7][18] + HKX3*P[0][7] + HKX4*P[1][7] + HKX5*P[2][7] + HKX6*P[3][7] + HKX7*P[7][16] + P[7][19]);
Kfusion[8] = HKX20*(HKX10*P[8][17] + HKX11*P[8][18] + HKX3*P[0][8] + HKX4*P[1][8] + HKX5*P[2][8] + HKX6*P[3][8] + HKX7*P[8][16] + P[8][19]);
Kfusion[9] = HKX20*(HKX10*P[9][17] + HKX11*P[9][18] + HKX3*P[0][9] + HKX4*P[1][9] + HKX5*P[2][9] + HKX6*P[3][9] + HKX7*P[9][16] + P[9][19]);
Kfusion[10] = HKX20*(HKX10*P[10][17] + HKX11*P[10][18] + HKX3*P[0][10] + HKX4*P[1][10] + HKX5*P[2][10] + HKX6*P[3][10] + HKX7*P[10][16] + P[10][19]);
Kfusion[11] = HKX20*(HKX10*P[11][17] + HKX11*P[11][18] + HKX3*P[0][11] + HKX4*P[1][11] + HKX5*P[2][11] + HKX6*P[3][11] + HKX7*P[11][16] + P[11][19]);
Kfusion[12] = HKX20*(HKX10*P[12][17] + HKX11*P[12][18] + HKX3*P[0][12] + HKX4*P[1][12] + HKX5*P[2][12] + HKX6*P[3][12] + HKX7*P[12][16] + P[12][19]);
Kfusion[13] = HKX20*(HKX10*P[13][17] + HKX11*P[13][18] + HKX3*P[0][13] + HKX4*P[1][13] + HKX5*P[2][13] + HKX6*P[3][13] + HKX7*P[13][16] + P[13][19]);
Kfusion[14] = HKX20*(HKX10*P[14][17] + HKX11*P[14][18] + HKX3*P[0][14] + HKX4*P[1][14] + HKX5*P[2][14] + HKX6*P[3][14] + HKX7*P[14][16] + P[14][19]);
Kfusion[15] = HKX20*(HKX10*P[15][17] + HKX11*P[15][18] + HKX3*P[0][15] + HKX4*P[1][15] + HKX5*P[2][15] + HKX6*P[3][15] + HKX7*P[15][16] + P[15][19]);
Kfusion[16] = HKX18*HKX20;
Kfusion[17] = HKX14*HKX20;
Kfusion[18] = HKX13*HKX20;
Kfusion[19] = HKX19*HKX20;
Kfusion[20] = HKX20*(HKX10*P[17][20] + HKX11*P[18][20] + HKX3*P[0][20] + HKX4*P[1][20] + HKX5*P[2][20] + HKX6*P[3][20] + HKX7*P[16][20] + P[19][20]);
Kfusion[21] = HKX20*(HKX10*P[17][21] + HKX11*P[18][21] + HKX3*P[0][21] + HKX4*P[1][21] + HKX5*P[2][21] + HKX6*P[3][21] + HKX7*P[16][21] + P[19][21]);
Kfusion[22] = HKX20*(HKX10*P[17][22] + HKX11*P[18][22] + HKX3*P[0][22] + HKX4*P[1][22] + HKX5*P[2][22] + HKX6*P[3][22] + HKX7*P[16][22] + P[19][22]);
Kfusion[23] = HKX20*(HKX10*P[17][23] + HKX11*P[18][23] + HKX3*P[0][23] + HKX4*P[1][23] + HKX5*P[2][23] + HKX6*P[3][23] + HKX7*P[16][23] + P[19][23]);


// Y axis innovation variance
float SY0 = 2*q3;
float SY1 = 2*q1;
float SY2 = -SY0*q0 + SY1*q2;
float SY3 = SY0*q2 + SY1*q0;
float SY4 = 2*q0;
float SY5 = -SY0*magN + SY1*magD + SY4*magE;
float SY6 = 2*q2;
float SY7 = -SY1*magE + SY4*magD + SY6*magN;
float SY8 = SY0*magD + SY1*magN + SY6*magE;
float SY9 = -SY0*magE - SY4*magN + SY6*magD;
float SY10 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);


innov_var_y = P[0][20]*SY5 + P[16][20]*SY2 + P[17][20]*SY10 + P[18][20]*SY3 + P[1][20]*SY7 + P[20][20] + P[2][20]*SY8 + P[3][20]*SY9 + R_MAG + SY10*(P[0][17]*SY5 + P[16][17]*SY2 + P[17][17]*SY10 + P[17][18]*SY3 + P[17][20] + P[1][17]*SY7 + P[2][17]*SY8 + P[3][17]*SY9) + SY2*(P[0][16]*SY5 + P[16][16]*SY2 + P[16][17]*SY10 + P[16][18]*SY3 + P[16][20] + P[1][16]*SY7 + P[2][16]*SY8 + P[3][16]*SY9) + SY3*(P[0][18]*SY5 + P[16][18]*SY2 + P[17][18]*SY10 + P[18][18]*SY3 + P[18][20] + P[1][18]*SY7 + P[2][18]*SY8 + P[3][18]*SY9) + SY5*(P[0][0]*SY5 + P[0][16]*SY2 + P[0][17]*SY10 + P[0][18]*SY3 + P[0][1]*SY7 + P[0][20] + P[0][2]*SY8 + P[0][3]*SY9) + SY7*(P[0][1]*SY5 + P[1][16]*SY2 + P[1][17]*SY10 + P[1][18]*SY3 + P[1][1]*SY7 + P[1][20] + P[1][2]*SY8 + P[1][3]*SY9) + SY8*(P[0][2]*SY5 + P[1][2]*SY7 + P[2][16]*SY2 + P[2][17]*SY10 + P[2][18]*SY3 + P[2][20] + P[2][2]*SY8 + P[2][3]*SY9) + SY9*(P[0][3]*SY5 + P[1][3]*SY7 + P[2][3]*SY8 + P[3][16]*SY2 + P[3][17]*SY10 + P[3][18]*SY3 + P[3][20] + P[3][3]*SY9);


// Y axis observation matrix and kalman gain
float HKY0 = 2*q1;
float HKY1 = 2*q0;
float HKY2 = 2*magN;
float HKY3 = HKY0*magD + HKY1*magE - HKY2*q3;
float HKY4 = -HKY0*magE + HKY1*magD + HKY2*q2;
float HKY5 = 2*q3;
float HKY6 = 2*q2;
float HKY7 = HKY2*q1 + HKY5*magD + HKY6*magE;
float HKY8 = -HKY2*q0 - HKY5*magE + HKY6*magD;
float HKY9 = HKY0*q2 - HKY1*q3;
float HKY10 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
float HKY11 = HKY0*q0 + HKY5*q2;
float HKY12 = HKY10*P[0][17] + HKY11*P[0][18] + HKY3*P[0][0] + HKY4*P[0][1] + HKY7*P[0][2] + HKY8*P[0][3] + HKY9*P[0][16] + P[0][20];
float HKY13 = HKY10*P[16][17] + HKY11*P[16][18] + HKY3*P[0][16] + HKY4*P[1][16] + HKY7*P[2][16] + HKY8*P[3][16] + HKY9*P[16][16] + P[16][20];
float HKY14 = HKY10*P[17][18] + HKY11*P[18][18] + HKY3*P[0][18] + HKY4*P[1][18] + HKY7*P[2][18] + HKY8*P[3][18] + HKY9*P[16][18] + P[18][20];
float HKY15 = HKY10*P[1][17] + HKY11*P[1][18] + HKY3*P[0][1] + HKY4*P[1][1] + HKY7*P[1][2] + HKY8*P[1][3] + HKY9*P[1][16] + P[1][20];
float HKY16 = HKY10*P[3][17] + HKY11*P[3][18] + HKY3*P[0][3] + HKY4*P[1][3] + HKY7*P[2][3] + HKY8*P[3][3] + HKY9*P[3][16] + P[3][20];
float HKY17 = HKY10*P[2][17] + HKY11*P[2][18] + HKY3*P[0][2] + HKY4*P[1][2] + HKY7*P[2][2] + HKY8*P[2][3] + HKY9*P[2][16] + P[2][20];
float HKY18 = HKY10*P[17][17] + HKY11*P[17][18] + HKY3*P[0][17] + HKY4*P[1][17] + HKY7*P[2][17] + HKY8*P[3][17] + HKY9*P[16][17] + P[17][20];
float HKY19 = HKY10*P[17][20] + HKY11*P[18][20] + HKY3*P[0][20] + HKY4*P[1][20] + HKY7*P[2][20] + HKY8*P[3][20] + HKY9*P[16][20] + P[20][20];
float HKY20 = 1.0F/(HKY10*HKY18 + HKY11*HKY14 + HKY12*HKY3 + HKY13*HKY9 + HKY15*HKY4 + HKY16*HKY8 + HKY17*HKY7 + HKY19 + R_MAG);


H_MAG[0] = HKY3;
H_MAG[1] = HKY4;
H_MAG[2] = HKY7;
H_MAG[3] = HKY8;
H_MAG[4] = 0;
H_MAG[5] = 0;
H_MAG[6] = 0;
H_MAG[7] = 0;
H_MAG[8] = 0;
H_MAG[9] = 0;
H_MAG[10] = 0;
H_MAG[11] = 0;
H_MAG[12] = 0;
H_MAG[13] = 0;
H_MAG[14] = 0;
H_MAG[15] = 0;
H_MAG[16] = HKY9;
H_MAG[17] = HKY10;
H_MAG[18] = HKY11;
H_MAG[19] = 0;
H_MAG[20] = 1;
H_MAG[21] = 0;
H_MAG[22] = 0;
H_MAG[23] = 0;


Kfusion[0] = HKY12*HKY20;
Kfusion[1] = HKY15*HKY20;
Kfusion[2] = HKY17*HKY20;
Kfusion[3] = HKY16*HKY20;
Kfusion[4] = HKY20*(HKY10*P[4][17] + HKY11*P[4][18] + HKY3*P[0][4] + HKY4*P[1][4] + HKY7*P[2][4] + HKY8*P[3][4] + HKY9*P[4][16] + P[4][20]);
Kfusion[5] = HKY20*(HKY10*P[5][17] + HKY11*P[5][18] + HKY3*P[0][5] + HKY4*P[1][5] + HKY7*P[2][5] + HKY8*P[3][5] + HKY9*P[5][16] + P[5][20]);
Kfusion[6] = HKY20*(HKY10*P[6][17] + HKY11*P[6][18] + HKY3*P[0][6] + HKY4*P[1][6] + HKY7*P[2][6] + HKY8*P[3][6] + HKY9*P[6][16] + P[6][20]);
Kfusion[7] = HKY20*(HKY10*P[7][17] + HKY11*P[7][18] + HKY3*P[0][7] + HKY4*P[1][7] + HKY7*P[2][7] + HKY8*P[3][7] + HKY9*P[7][16] + P[7][20]);
Kfusion[8] = HKY20*(HKY10*P[8][17] + HKY11*P[8][18] + HKY3*P[0][8] + HKY4*P[1][8] + HKY7*P[2][8] + HKY8*P[3][8] + HKY9*P[8][16] + P[8][20]);
Kfusion[9] = HKY20*(HKY10*P[9][17] + HKY11*P[9][18] + HKY3*P[0][9] + HKY4*P[1][9] + HKY7*P[2][9] + HKY8*P[3][9] + HKY9*P[9][16] + P[9][20]);
Kfusion[10] = HKY20*(HKY10*P[10][17] + HKY11*P[10][18] + HKY3*P[0][10] + HKY4*P[1][10] + HKY7*P[2][10] + HKY8*P[3][10] + HKY9*P[10][16] + P[10][20]);
Kfusion[11] = HKY20*(HKY10*P[11][17] + HKY11*P[11][18] + HKY3*P[0][11] + HKY4*P[1][11] + HKY7*P[2][11] + HKY8*P[3][11] + HKY9*P[11][16] + P[11][20]);
Kfusion[12] = HKY20*(HKY10*P[12][17] + HKY11*P[12][18] + HKY3*P[0][12] + HKY4*P[1][12] + HKY7*P[2][12] + HKY8*P[3][12] + HKY9*P[12][16] + P[12][20]);
Kfusion[13] = HKY20*(HKY10*P[13][17] + HKY11*P[13][18] + HKY3*P[0][13] + HKY4*P[1][13] + HKY7*P[2][13] + HKY8*P[3][13] + HKY9*P[13][16] + P[13][20]);
Kfusion[14] = HKY20*(HKY10*P[14][17] + HKY11*P[14][18] + HKY3*P[0][14] + HKY4*P[1][14] + HKY7*P[2][14] + HKY8*P[3][14] + HKY9*P[14][16] + P[14][20]);
Kfusion[15] = HKY20*(HKY10*P[15][17] + HKY11*P[15][18] + HKY3*P[0][15] + HKY4*P[1][15] + HKY7*P[2][15] + HKY8*P[3][15] + HKY9*P[15][16] + P[15][20]);
Kfusion[16] = HKY13*HKY20;
Kfusion[17] = HKY18*HKY20;
Kfusion[18] = HKY14*HKY20;
Kfusion[19] = HKY20*(HKY10*P[17][19] + HKY11*P[18][19] + HKY3*P[0][19] + HKY4*P[1][19] + HKY7*P[2][19] + HKY8*P[3][19] + HKY9*P[16][19] + P[19][20]);
Kfusion[20] = HKY19*HKY20;
Kfusion[21] = HKY20*(HKY10*P[17][21] + HKY11*P[18][21] + HKY3*P[0][21] + HKY4*P[1][21] + HKY7*P[2][21] + HKY8*P[3][21] + HKY9*P[16][21] + P[20][21]);
Kfusion[22] = HKY20*(HKY10*P[17][22] + HKY11*P[18][22] + HKY3*P[0][22] + HKY4*P[1][22] + HKY7*P[2][22] + HKY8*P[3][22] + HKY9*P[16][22] + P[20][22]);
Kfusion[23] = HKY20*(HKY10*P[17][23] + HKY11*P[18][23] + HKY3*P[0][23] + HKY4*P[1][23] + HKY7*P[2][23] + HKY8*P[3][23] + HKY9*P[16][23] + P[20][23]);


// Z axis innovation variance
float SZ0 = 2*q0;
float SZ1 = 2*q1;
float SZ2 = SZ0*q2 + SZ1*q3;
float SZ3 = 2*q2;
float SZ4 = -SZ1*q0 + SZ3*q3;
float SZ5 = SZ0*magD - SZ1*magE + SZ3*magN;
float SZ6 = 2*q3;
float SZ7 = -SZ0*magE - SZ1*magD + SZ6*magN;
float SZ8 = SZ0*magN - SZ3*magD + SZ6*magE;
float SZ9 = SZ1*magN + SZ3*magE + SZ6*magD;
float SZ10 = powf(q0, 2) - powf(q1, 2) - powf(q2, 2) + powf(q3, 2);


innov_var_z = P[0][21]*SZ5 + P[16][21]*SZ2 + P[17][21]*SZ4 + P[18][21]*SZ10 + P[1][21]*SZ7 + P[21][21] + P[2][21]*SZ8 + P[3][21]*SZ9 + R_MAG + SZ10*(P[0][18]*SZ5 + P[16][18]*SZ2 + P[17][18]*SZ4 + P[18][18]*SZ10 + P[18][21] + P[1][18]*SZ7 + P[2][18]*SZ8 + P[3][18]*SZ9) + SZ2*(P[0][16]*SZ5 + P[16][16]*SZ2 + P[16][17]*SZ4 + P[16][18]*SZ10 + P[16][21] + P[1][16]*SZ7 + P[2][16]*SZ8 + P[3][16]*SZ9) + SZ4*(P[0][17]*SZ5 + P[16][17]*SZ2 + P[17][17]*SZ4 + P[17][18]*SZ10 + P[17][21] + P[1][17]*SZ7 + P[2][17]*SZ8 + P[3][17]*SZ9) + SZ5*(P[0][0]*SZ5 + P[0][16]*SZ2 + P[0][17]*SZ4 + P[0][18]*SZ10 + P[0][1]*SZ7 + P[0][21] + P[0][2]*SZ8 + P[0][3]*SZ9) + SZ7*(P[0][1]*SZ5 + P[1][16]*SZ2 + P[1][17]*SZ4 + P[1][18]*SZ10 + P[1][1]*SZ7 + P[1][21] + P[1][2]*SZ8 + P[1][3]*SZ9) + SZ8*(P[0][2]*SZ5 + P[1][2]*SZ7 + P[2][16]*SZ2 + P[2][17]*SZ4 + P[2][18]*SZ10 + P[2][21] + P[2][2]*SZ8 + P[2][3]*SZ9) + SZ9*(P[0][3]*SZ5 + P[1][3]*SZ7 + P[2][3]*SZ8 + P[3][16]*SZ2 + P[3][17]*SZ4 + P[3][18]*SZ10 + P[3][21] + P[3][3]*SZ9);


// Z axis observation matrix and kalman gain
float HKZ0 = 2*q0;
float HKZ1 = 2*q1;
float HKZ2 = 2*magN;
float HKZ3 = HKZ0*magD - HKZ1*magE + HKZ2*q2;
float HKZ4 = -HKZ0*magE - HKZ1*magD + HKZ2*q3;
float HKZ5 = 2*magD;
float HKZ6 = 2*magE;
float HKZ7 = HKZ0*magN - HKZ5*q2 + HKZ6*q3;
float HKZ8 = HKZ1*magN + HKZ5*q3 + HKZ6*q2;
float HKZ9 = HKZ0*q2 + HKZ1*q3;
float HKZ10 = -HKZ1*q0 + 2*q2*q3;
float HKZ11 = powf(q0, 2) - powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
float HKZ12 = HKZ10*P[0][17] + HKZ11*P[0][18] + HKZ3*P[0][0] + HKZ4*P[0][1] + HKZ7*P[0][2] + HKZ8*P[0][3] + HKZ9*P[0][16] + P[0][21];
float HKZ13 = HKZ10*P[17][17] + HKZ11*P[17][18] + HKZ3*P[0][17] + HKZ4*P[1][17] + HKZ7*P[2][17] + HKZ8*P[3][17] + HKZ9*P[16][17] + P[17][21];
float HKZ14 = HKZ10*P[16][17] + HKZ11*P[16][18] + HKZ3*P[0][16] + HKZ4*P[1][16] + HKZ7*P[2][16] + HKZ8*P[3][16] + HKZ9*P[16][16] + P[16][21];
float HKZ15 = HKZ10*P[1][17] + HKZ11*P[1][18] + HKZ3*P[0][1] + HKZ4*P[1][1] + HKZ7*P[1][2] + HKZ8*P[1][3] + HKZ9*P[1][16] + P[1][21];
float HKZ16 = HKZ10*P[2][17] + HKZ11*P[2][18] + HKZ3*P[0][2] + HKZ4*P[1][2] + HKZ7*P[2][2] + HKZ8*P[2][3] + HKZ9*P[2][16] + P[2][21];
float HKZ17 = HKZ10*P[3][17] + HKZ11*P[3][18] + HKZ3*P[0][3] + HKZ4*P[1][3] + HKZ7*P[2][3] + HKZ8*P[3][3] + HKZ9*P[3][16] + P[3][21];
float HKZ18 = HKZ10*P[17][18] + HKZ11*P[18][18] + HKZ3*P[0][18] + HKZ4*P[1][18] + HKZ7*P[2][18] + HKZ8*P[3][18] + HKZ9*P[16][18] + P[18][21];
float HKZ19 = HKZ10*P[17][21] + HKZ11*P[18][21] + HKZ3*P[0][21] + HKZ4*P[1][21] + HKZ7*P[2][21] + HKZ8*P[3][21] + HKZ9*P[16][21] + P[21][21];
float HKZ20 = 1.0F/(HKZ10*HKZ13 + HKZ11*HKZ18 + HKZ12*HKZ3 + HKZ14*HKZ9 + HKZ15*HKZ4 + HKZ16*HKZ7 + HKZ17*HKZ8 + HKZ19 + R_MAG);


H_MAG[0] = HKZ3;
H_MAG[1] = HKZ4;
H_MAG[2] = HKZ7;
H_MAG[3] = HKZ8;
H_MAG[4] = 0;
H_MAG[5] = 0;
H_MAG[6] = 0;
H_MAG[7] = 0;
H_MAG[8] = 0;
H_MAG[9] = 0;
H_MAG[10] = 0;
H_MAG[11] = 0;
H_MAG[12] = 0;
H_MAG[13] = 0;
H_MAG[14] = 0;
H_MAG[15] = 0;
H_MAG[16] = HKZ9;
H_MAG[17] = HKZ10;
H_MAG[18] = HKZ11;
H_MAG[19] = 0;
H_MAG[20] = 0;
H_MAG[21] = 1;
H_MAG[22] = 0;
H_MAG[23] = 0;


Kfusion[0] = HKZ12*HKZ20;
Kfusion[1] = HKZ15*HKZ20;
Kfusion[2] = HKZ16*HKZ20;
Kfusion[3] = HKZ17*HKZ20;
Kfusion[4] = HKZ20*(HKZ10*P[4][17] + HKZ11*P[4][18] + HKZ3*P[0][4] + HKZ4*P[1][4] + HKZ7*P[2][4] + HKZ8*P[3][4] + HKZ9*P[4][16] + P[4][21]);
Kfusion[5] = HKZ20*(HKZ10*P[5][17] + HKZ11*P[5][18] + HKZ3*P[0][5] + HKZ4*P[1][5] + HKZ7*P[2][5] + HKZ8*P[3][5] + HKZ9*P[5][16] + P[5][21]);
Kfusion[6] = HKZ20*(HKZ10*P[6][17] + HKZ11*P[6][18] + HKZ3*P[0][6] + HKZ4*P[1][6] + HKZ7*P[2][6] + HKZ8*P[3][6] + HKZ9*P[6][16] + P[6][21]);
Kfusion[7] = HKZ20*(HKZ10*P[7][17] + HKZ11*P[7][18] + HKZ3*P[0][7] + HKZ4*P[1][7] + HKZ7*P[2][7] + HKZ8*P[3][7] + HKZ9*P[7][16] + P[7][21]);
Kfusion[8] = HKZ20*(HKZ10*P[8][17] + HKZ11*P[8][18] + HKZ3*P[0][8] + HKZ4*P[1][8] + HKZ7*P[2][8] + HKZ8*P[3][8] + HKZ9*P[8][16] + P[8][21]);
Kfusion[9] = HKZ20*(HKZ10*P[9][17] + HKZ11*P[9][18] + HKZ3*P[0][9] + HKZ4*P[1][9] + HKZ7*P[2][9] + HKZ8*P[3][9] + HKZ9*P[9][16] + P[9][21]);
Kfusion[10] = HKZ20*(HKZ10*P[10][17] + HKZ11*P[10][18] + HKZ3*P[0][10] + HKZ4*P[1][10] + HKZ7*P[2][10] + HKZ8*P[3][10] + HKZ9*P[10][16] + P[10][21]);
Kfusion[11] = HKZ20*(HKZ10*P[11][17] + HKZ11*P[11][18] + HKZ3*P[0][11] + HKZ4*P[1][11] + HKZ7*P[2][11] + HKZ8*P[3][11] + HKZ9*P[11][16] + P[11][21]);
Kfusion[12] = HKZ20*(HKZ10*P[12][17] + HKZ11*P[12][18] + HKZ3*P[0][12] + HKZ4*P[1][12] + HKZ7*P[2][12] + HKZ8*P[3][12] + HKZ9*P[12][16] + P[12][21]);
Kfusion[13] = HKZ20*(HKZ10*P[13][17] + HKZ11*P[13][18] + HKZ3*P[0][13] + HKZ4*P[1][13] + HKZ7*P[2][13] + HKZ8*P[3][13] + HKZ9*P[13][16] + P[13][21]);
Kfusion[14] = HKZ20*(HKZ10*P[14][17] + HKZ11*P[14][18] + HKZ3*P[0][14] + HKZ4*P[1][14] + HKZ7*P[2][14] + HKZ8*P[3][14] + HKZ9*P[14][16] + P[14][21]);
Kfusion[15] = HKZ20*(HKZ10*P[15][17] + HKZ11*P[15][18] + HKZ3*P[0][15] + HKZ4*P[1][15] + HKZ7*P[2][15] + HKZ8*P[3][15] + HKZ9*P[15][16] + P[15][21]);
Kfusion[16] = HKZ14*HKZ20;
Kfusion[17] = HKZ13*HKZ20;
Kfusion[18] = HKZ18*HKZ20;
Kfusion[19] = HKZ20*(HKZ10*P[17][19] + HKZ11*P[18][19] + HKZ3*P[0][19] + HKZ4*P[1][19] + HKZ7*P[2][19] + HKZ8*P[3][19] + HKZ9*P[16][19] + P[19][21]);
Kfusion[20] = HKZ20*(HKZ10*P[17][20] + HKZ11*P[18][20] + HKZ3*P[0][20] + HKZ4*P[1][20] + HKZ7*P[2][20] + HKZ8*P[3][20] + HKZ9*P[16][20] + P[20][21]);
Kfusion[21] = HKZ19*HKZ20;
Kfusion[22] = HKZ20*(HKZ10*P[17][22] + HKZ11*P[18][22] + HKZ3*P[0][22] + HKZ4*P[1][22] + HKZ7*P[2][22] + HKZ8*P[3][22] + HKZ9*P[16][22] + P[21][22]);
Kfusion[23] = HKZ20*(HKZ10*P[17][23] + HKZ11*P[18][23] + HKZ3*P[0][23] + HKZ4*P[1][23] + HKZ7*P[2][23] + HKZ8*P[3][23] + HKZ9*P[16][23] + P[21][23]);


