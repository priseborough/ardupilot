// Equations for sideslip fusion
// sideslip innovation variance
float S0 = 2*q0;
float S1 = 2*q3;
float S2 = S0*q1 + S1*q2;
float S3 = S0*q2;
float S4 = 2*q1;
float S5 = S4*q3;
float S6 = ve - vwe;
float S7 = S0*q3;
float S8 = S4*q2;
float S9 = S7 + S8;
float S10 = vn - vwn;
float S11 = powf(q0, 2);
float S12 = powf(q3, 2);
float S13 = S11 - S12;
float S14 = powf(q1, 2);
float S15 = powf(q2, 2);
float S16 = S14 - S15;
float S17 = S13 + S16;
float S18 = S10*S17 + S6*S9 + vd*(-S3 + S5);
float S19 = 1.0F/S18;
float S20 = -S7;
float S21 = S20 + S8;
float S22 = -S14 + S15;
float S23 = S13 + S22;
float S24 = (S10*S21 + S2*vd + S23*S6)/powf(S18, 2);
float S25 = S19*S2 + S24*(S3 - S5);
float S26 = -S11 + S12;
float S27 = S19*S21 + S24*(S22 + S26);
float S28 = -S8;
float S29 = S19*S23 + S24*(S20 + S28);
float S30 = S17*S24 + S19*(S28 + S7);
float S31 = S19*(S16 + S26) + S24*S9;
float S32 = S4*vd;
float S33 = S0*S6;
float S34 = S1*S10;
float S35 = 2*q2;
float S36 = -S0*S10 - S1*S6 + S35*vd;
float S37 = S19*(S32 + S33 - S34) + S24*S36;
float S38 = S0*vd + S10*S35 - S4*S6;
float S39 = S1*vd;
float S40 = S10*S4;
float S41 = S35*S6;
float S42 = S19*S38 + S24*(-S39 - S40 - S41);
float S43 = S19*(S39 + S40 + S41) + S24*S38;
float S44 = S19*S36 + S24*(-S32 - S33 + S34);


innov_var = R_BETA + S25*(P[0][6]*S37 + P[1][6]*S42 + P[2][6]*S43 + P[3][6]*S44 + P[4][6]*S27 + P[5][6]*S29 + P[6][22]*S30 + P[6][23]*S31 + P[6][6]*S25) + S27*(P[0][4]*S37 + P[1][4]*S42 + P[2][4]*S43 + P[3][4]*S44 + P[4][22]*S30 + P[4][23]*S31 + P[4][4]*S27 + P[4][5]*S29 + P[4][6]*S25) + S29*(P[0][5]*S37 + P[1][5]*S42 + P[2][5]*S43 + P[3][5]*S44 + P[4][5]*S27 + P[5][22]*S30 + P[5][23]*S31 + P[5][5]*S29 + P[5][6]*S25) + S30*(P[0][22]*S37 + P[1][22]*S42 + P[22][22]*S30 + P[22][23]*S31 + P[2][22]*S43 + P[3][22]*S44 + P[4][22]*S27 + P[5][22]*S29 + P[6][22]*S25) + S31*(P[0][23]*S37 + P[1][23]*S42 + P[22][23]*S30 + P[23][23]*S31 + P[2][23]*S43 + P[3][23]*S44 + P[4][23]*S27 + P[5][23]*S29 + P[6][23]*S25) + S37*(P[0][0]*S37 + P[0][1]*S42 + P[0][22]*S30 + P[0][23]*S31 + P[0][2]*S43 + P[0][3]*S44 + P[0][4]*S27 + P[0][5]*S29 + P[0][6]*S25) + S42*(P[0][1]*S37 + P[1][1]*S42 + P[1][22]*S30 + P[1][23]*S31 + P[1][2]*S43 + P[1][3]*S44 + P[1][4]*S27 + P[1][5]*S29 + P[1][6]*S25) + S43*(P[0][2]*S37 + P[1][2]*S42 + P[2][22]*S30 + P[2][23]*S31 + P[2][2]*S43 + P[2][3]*S44 + P[2][4]*S27 + P[2][5]*S29 + P[2][6]*S25) + S44*(P[0][3]*S37 + P[1][3]*S42 + P[2][3]*S43 + P[3][22]*S30 + P[3][23]*S31 + P[3][3]*S44 + P[3][4]*S27 + P[3][5]*S29 + P[3][6]*S25);


// sideslip observation matrix and kalman gain
float HK0 = 2*q1;
float HK1 = HK0*vd;
float HK2 = ve - vwe;
float HK3 = 2*q0;
float HK4 = HK2*HK3;
float HK5 = vn - vwn;
float HK6 = 2*q3;
float HK7 = HK5*HK6;
float HK8 = HK3*q2;
float HK9 = HK0*q3;
float HK10 = HK3*q3;
float HK11 = HK0*q2;
float HK12 = HK10 + HK11;
float HK13 = powf(q0, 2);
float HK14 = powf(q3, 2);
float HK15 = HK13 - HK14;
float HK16 = powf(q1, 2);
float HK17 = powf(q2, 2);
float HK18 = HK16 - HK17;
float HK19 = HK15 + HK18;
float HK20 = HK12*HK2 + HK19*HK5 + vd*(-HK8 + HK9);
float HK21 = 1.0F/HK20;
float HK22 = 2*q2;
float HK23 = -HK2*HK6 + HK22*vd - HK3*HK5;
float HK24 = HK3*q1 + HK6*q2;
float HK25 = -HK10;
float HK26 = HK11 + HK25;
float HK27 = -HK16 + HK17;
float HK28 = HK15 + HK27;
float HK29 = (HK2*HK28 + HK24*vd + HK26*HK5)/powf(HK20, 2);
float HK30 = HK21*(HK1 + HK4 - HK7) + HK23*HK29;
float HK31 = -HK0*HK2 + HK22*HK5 + HK3*vd;
float HK32 = HK6*vd;
float HK33 = HK0*HK5;
float HK34 = HK2*HK22;
float HK35 = HK21*HK31 + HK29*(-HK32 - HK33 - HK34);
float HK36 = HK21*(HK32 + HK33 + HK34) + HK29*HK31;
float HK37 = HK21*HK23 + HK29*(-HK1 - HK4 + HK7);
float HK38 = -HK13 + HK14;
float HK39 = HK21*HK26 + HK29*(HK27 + HK38);
float HK40 = -HK11;
float HK41 = HK21*HK28 + HK29*(HK25 + HK40);
float HK42 = HK21*HK24 + HK29*(HK8 - HK9);
float HK43 = HK19*HK29 + HK21*(HK10 + HK40);
float HK44 = HK12*HK29 + HK21*(HK18 + HK38);
float HK45 = HK30*P[0][0] + HK35*P[0][1] + HK36*P[0][2] + HK37*P[0][3] + HK39*P[0][4] + HK41*P[0][5] + HK42*P[0][6] + HK43*P[0][22] + HK44*P[0][23];
float HK46 = HK30*P[0][6] + HK35*P[1][6] + HK36*P[2][6] + HK37*P[3][6] + HK39*P[4][6] + HK41*P[5][6] + HK42*P[6][6] + HK43*P[6][22] + HK44*P[6][23];
float HK47 = HK30*P[0][4] + HK35*P[1][4] + HK36*P[2][4] + HK37*P[3][4] + HK39*P[4][4] + HK41*P[4][5] + HK42*P[4][6] + HK43*P[4][22] + HK44*P[4][23];
float HK48 = HK30*P[0][22] + HK35*P[1][22] + HK36*P[2][22] + HK37*P[3][22] + HK39*P[4][22] + HK41*P[5][22] + HK42*P[6][22] + HK43*P[22][22] + HK44*P[22][23];
float HK49 = HK30*P[0][5] + HK35*P[1][5] + HK36*P[2][5] + HK37*P[3][5] + HK39*P[4][5] + HK41*P[5][5] + HK42*P[5][6] + HK43*P[5][22] + HK44*P[5][23];
float HK50 = HK30*P[0][23] + HK35*P[1][23] + HK36*P[2][23] + HK37*P[3][23] + HK39*P[4][23] + HK41*P[5][23] + HK42*P[6][23] + HK43*P[22][23] + HK44*P[23][23];
float HK51 = HK30*P[0][1] + HK35*P[1][1] + HK36*P[1][2] + HK37*P[1][3] + HK39*P[1][4] + HK41*P[1][5] + HK42*P[1][6] + HK43*P[1][22] + HK44*P[1][23];
float HK52 = HK30*P[0][2] + HK35*P[1][2] + HK36*P[2][2] + HK37*P[2][3] + HK39*P[2][4] + HK41*P[2][5] + HK42*P[2][6] + HK43*P[2][22] + HK44*P[2][23];
float HK53 = HK30*P[0][3] + HK35*P[1][3] + HK36*P[2][3] + HK37*P[3][3] + HK39*P[3][4] + HK41*P[3][5] + HK42*P[3][6] + HK43*P[3][22] + HK44*P[3][23];
float HK54 = 1.0F/(HK30*HK45 + HK35*HK51 + HK36*HK52 + HK37*HK53 + HK39*HK47 + HK41*HK49 + HK42*HK46 + HK43*HK48 + HK44*HK50 + R_BETA);


H_BETA[0] = HK30;
H_BETA[1] = HK35;
H_BETA[2] = HK36;
H_BETA[3] = HK37;
H_BETA[4] = HK39;
H_BETA[5] = HK41;
H_BETA[6] = HK42;
H_BETA[7] = 0;
H_BETA[8] = 0;
H_BETA[9] = 0;
H_BETA[10] = 0;
H_BETA[11] = 0;
H_BETA[12] = 0;
H_BETA[13] = 0;
H_BETA[14] = 0;
H_BETA[15] = 0;
H_BETA[16] = 0;
H_BETA[17] = 0;
H_BETA[18] = 0;
H_BETA[19] = 0;
H_BETA[20] = 0;
H_BETA[21] = 0;
H_BETA[22] = HK43;
H_BETA[23] = HK44;


Kfusion[0] = HK45*HK54;
Kfusion[1] = HK51*HK54;
Kfusion[2] = HK52*HK54;
Kfusion[3] = HK53*HK54;
Kfusion[4] = HK47*HK54;
Kfusion[5] = HK49*HK54;
Kfusion[6] = HK46*HK54;
Kfusion[7] = HK54*(HK30*P[0][7] + HK35*P[1][7] + HK36*P[2][7] + HK37*P[3][7] + HK39*P[4][7] + HK41*P[5][7] + HK42*P[6][7] + HK43*P[7][22] + HK44*P[7][23]);
Kfusion[8] = HK54*(HK30*P[0][8] + HK35*P[1][8] + HK36*P[2][8] + HK37*P[3][8] + HK39*P[4][8] + HK41*P[5][8] + HK42*P[6][8] + HK43*P[8][22] + HK44*P[8][23]);
Kfusion[9] = HK54*(HK30*P[0][9] + HK35*P[1][9] + HK36*P[2][9] + HK37*P[3][9] + HK39*P[4][9] + HK41*P[5][9] + HK42*P[6][9] + HK43*P[9][22] + HK44*P[9][23]);
Kfusion[10] = HK54*(HK30*P[0][10] + HK35*P[1][10] + HK36*P[2][10] + HK37*P[3][10] + HK39*P[4][10] + HK41*P[5][10] + HK42*P[6][10] + HK43*P[10][22] + HK44*P[10][23]);
Kfusion[11] = HK54*(HK30*P[0][11] + HK35*P[1][11] + HK36*P[2][11] + HK37*P[3][11] + HK39*P[4][11] + HK41*P[5][11] + HK42*P[6][11] + HK43*P[11][22] + HK44*P[11][23]);
Kfusion[12] = HK54*(HK30*P[0][12] + HK35*P[1][12] + HK36*P[2][12] + HK37*P[3][12] + HK39*P[4][12] + HK41*P[5][12] + HK42*P[6][12] + HK43*P[12][22] + HK44*P[12][23]);
Kfusion[13] = HK54*(HK30*P[0][13] + HK35*P[1][13] + HK36*P[2][13] + HK37*P[3][13] + HK39*P[4][13] + HK41*P[5][13] + HK42*P[6][13] + HK43*P[13][22] + HK44*P[13][23]);
Kfusion[14] = HK54*(HK30*P[0][14] + HK35*P[1][14] + HK36*P[2][14] + HK37*P[3][14] + HK39*P[4][14] + HK41*P[5][14] + HK42*P[6][14] + HK43*P[14][22] + HK44*P[14][23]);
Kfusion[15] = HK54*(HK30*P[0][15] + HK35*P[1][15] + HK36*P[2][15] + HK37*P[3][15] + HK39*P[4][15] + HK41*P[5][15] + HK42*P[6][15] + HK43*P[15][22] + HK44*P[15][23]);
Kfusion[16] = HK54*(HK30*P[0][16] + HK35*P[1][16] + HK36*P[2][16] + HK37*P[3][16] + HK39*P[4][16] + HK41*P[5][16] + HK42*P[6][16] + HK43*P[16][22] + HK44*P[16][23]);
Kfusion[17] = HK54*(HK30*P[0][17] + HK35*P[1][17] + HK36*P[2][17] + HK37*P[3][17] + HK39*P[4][17] + HK41*P[5][17] + HK42*P[6][17] + HK43*P[17][22] + HK44*P[17][23]);
Kfusion[18] = HK54*(HK30*P[0][18] + HK35*P[1][18] + HK36*P[2][18] + HK37*P[3][18] + HK39*P[4][18] + HK41*P[5][18] + HK42*P[6][18] + HK43*P[18][22] + HK44*P[18][23]);
Kfusion[19] = HK54*(HK30*P[0][19] + HK35*P[1][19] + HK36*P[2][19] + HK37*P[3][19] + HK39*P[4][19] + HK41*P[5][19] + HK42*P[6][19] + HK43*P[19][22] + HK44*P[19][23]);
Kfusion[20] = HK54*(HK30*P[0][20] + HK35*P[1][20] + HK36*P[2][20] + HK37*P[3][20] + HK39*P[4][20] + HK41*P[5][20] + HK42*P[6][20] + HK43*P[20][22] + HK44*P[20][23]);
Kfusion[21] = HK54*(HK30*P[0][21] + HK35*P[1][21] + HK36*P[2][21] + HK37*P[3][21] + HK39*P[4][21] + HK41*P[5][21] + HK42*P[6][21] + HK43*P[21][22] + HK44*P[21][23]);
Kfusion[22] = HK48*HK54;
Kfusion[23] = HK50*HK54;


