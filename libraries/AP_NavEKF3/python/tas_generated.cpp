// Equations for TAS fusion
// TAS innovation variance
float S0 = ve - vwe;
float S1 = vn - vwn;
float S2 = powf(powf(S0, 2) + powf(S1, 2) + powf(vd, 2), -1.0F/2.0F);
float S3 = S2*vd;
float S4 = S1*S2;
float S5 = S0*S2;
float S6 = S2*(-vn + vwn);
float S7 = S2*(-ve + vwe);


innov_var = R_TAS + S3*(P[4][6]*S4 + P[5][6]*S5 + P[6][22]*S6 + P[6][23]*S7 + P[6][6]*S3) + S4*(P[4][22]*S6 + P[4][23]*S7 + P[4][4]*S4 + P[4][5]*S5 + P[4][6]*S3) + S5*(P[4][5]*S4 + P[5][22]*S6 + P[5][23]*S7 + P[5][5]*S5 + P[5][6]*S3) + S6*(P[22][22]*S6 + P[22][23]*S7 + P[4][22]*S4 + P[5][22]*S5 + P[6][22]*S3) + S7*(P[22][23]*S6 + P[23][23]*S7 + P[4][23]*S4 + P[5][23]*S5 + P[6][23]*S3);


// TAS observation matrix and kalman gain
float HK0 = vn - vwn;
float HK1 = ve - vwe;
float HK2 = powf(powf(HK0, 2) + powf(HK1, 2) + powf(vd, 2), -1.0F/2.0F);
float HK3 = HK0*HK2;
float HK4 = HK1*HK2;
float HK5 = HK2*vd;
float HK6 = HK2*(-vn + vwn);
float HK7 = HK2*(-ve + vwe);
float HK8 = HK3*P[4][6] + HK4*P[5][6] + HK5*P[6][6] + HK6*P[6][22] + HK7*P[6][23];
float HK9 = HK3*P[4][23] + HK4*P[5][23] + HK5*P[6][23] + HK6*P[22][23] + HK7*P[23][23];
float HK10 = HK3*P[4][5] + HK4*P[5][5] + HK5*P[5][6] + HK6*P[5][22] + HK7*P[5][23];
float HK11 = HK3*P[4][22] + HK4*P[5][22] + HK5*P[6][22] + HK6*P[22][22] + HK7*P[22][23];
float HK12 = HK3*P[4][4] + HK4*P[4][5] + HK5*P[4][6] + HK6*P[4][22] + HK7*P[4][23];
float HK13 = 1.0F/(HK10*HK4 + HK11*HK6 + HK12*HK3 + HK5*HK8 + HK7*HK9 + R_TAS);


H_TAS[0] = 0;
H_TAS[1] = 0;
H_TAS[2] = 0;
H_TAS[3] = 0;
H_TAS[4] = HK3;
H_TAS[5] = HK4;
H_TAS[6] = HK5;
H_TAS[7] = 0;
H_TAS[8] = 0;
H_TAS[9] = 0;
H_TAS[10] = 0;
H_TAS[11] = 0;
H_TAS[12] = 0;
H_TAS[13] = 0;
H_TAS[14] = 0;
H_TAS[15] = 0;
H_TAS[16] = 0;
H_TAS[17] = 0;
H_TAS[18] = 0;
H_TAS[19] = 0;
H_TAS[20] = 0;
H_TAS[21] = 0;
H_TAS[22] = HK6;
H_TAS[23] = HK7;


Kfusion[0] = HK13*(HK3*P[0][4] + HK4*P[0][5] + HK5*P[0][6] + HK6*P[0][22] + HK7*P[0][23]);
Kfusion[1] = HK13*(HK3*P[1][4] + HK4*P[1][5] + HK5*P[1][6] + HK6*P[1][22] + HK7*P[1][23]);
Kfusion[2] = HK13*(HK3*P[2][4] + HK4*P[2][5] + HK5*P[2][6] + HK6*P[2][22] + HK7*P[2][23]);
Kfusion[3] = HK13*(HK3*P[3][4] + HK4*P[3][5] + HK5*P[3][6] + HK6*P[3][22] + HK7*P[3][23]);
Kfusion[4] = HK12*HK13;
Kfusion[5] = HK10*HK13;
Kfusion[6] = HK13*HK8;
Kfusion[7] = HK13*(HK3*P[4][7] + HK4*P[5][7] + HK5*P[6][7] + HK6*P[7][22] + HK7*P[7][23]);
Kfusion[8] = HK13*(HK3*P[4][8] + HK4*P[5][8] + HK5*P[6][8] + HK6*P[8][22] + HK7*P[8][23]);
Kfusion[9] = HK13*(HK3*P[4][9] + HK4*P[5][9] + HK5*P[6][9] + HK6*P[9][22] + HK7*P[9][23]);
Kfusion[10] = HK13*(HK3*P[4][10] + HK4*P[5][10] + HK5*P[6][10] + HK6*P[10][22] + HK7*P[10][23]);
Kfusion[11] = HK13*(HK3*P[4][11] + HK4*P[5][11] + HK5*P[6][11] + HK6*P[11][22] + HK7*P[11][23]);
Kfusion[12] = HK13*(HK3*P[4][12] + HK4*P[5][12] + HK5*P[6][12] + HK6*P[12][22] + HK7*P[12][23]);
Kfusion[13] = HK13*(HK3*P[4][13] + HK4*P[5][13] + HK5*P[6][13] + HK6*P[13][22] + HK7*P[13][23]);
Kfusion[14] = HK13*(HK3*P[4][14] + HK4*P[5][14] + HK5*P[6][14] + HK6*P[14][22] + HK7*P[14][23]);
Kfusion[15] = HK13*(HK3*P[4][15] + HK4*P[5][15] + HK5*P[6][15] + HK6*P[15][22] + HK7*P[15][23]);
Kfusion[16] = HK13*(HK3*P[4][16] + HK4*P[5][16] + HK5*P[6][16] + HK6*P[16][22] + HK7*P[16][23]);
Kfusion[17] = HK13*(HK3*P[4][17] + HK4*P[5][17] + HK5*P[6][17] + HK6*P[17][22] + HK7*P[17][23]);
Kfusion[18] = HK13*(HK3*P[4][18] + HK4*P[5][18] + HK5*P[6][18] + HK6*P[18][22] + HK7*P[18][23]);
Kfusion[19] = HK13*(HK3*P[4][19] + HK4*P[5][19] + HK5*P[6][19] + HK6*P[19][22] + HK7*P[19][23]);
Kfusion[20] = HK13*(HK3*P[4][20] + HK4*P[5][20] + HK5*P[6][20] + HK6*P[20][22] + HK7*P[20][23]);
Kfusion[21] = HK13*(HK3*P[4][21] + HK4*P[5][21] + HK5*P[6][21] + HK6*P[21][22] + HK7*P[21][23]);
Kfusion[22] = HK11*HK13;
Kfusion[23] = HK13*HK9;


