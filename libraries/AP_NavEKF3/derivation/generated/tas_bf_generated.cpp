// Axis 0 equations
// Sub Expressions
const ftype HK0 = q2*vd;
const ftype HK1 = ve - vwe;
const ftype HK2 = HK1*q3;
const ftype HK3 = HK1*q2 + q3*vd;
const ftype HK4 = q0*vd;
const ftype HK5 = HK1*q1;
const ftype HK6 = 2*vn - 2*vwn;
const ftype HK7 = HK6*q2;
const ftype HK8 = HK1*q0 - HK6*q3 + q1*vd;
const ftype HK9 = 2*powf(q2, 2);
const ftype HK10 = 2*powf(q3, 2);
const ftype HK11 = q0*q3 + q1*q2;
const ftype HK12 = 2*HK11;
const ftype HK13 = q1*q3;
const ftype HK14 = q0*q2;
const ftype HK15 = HK10 + HK9 - 1;
const ftype HK16 = 2*HK11;
const ftype HK17 = -2*HK13 + 2*HK14;
const ftype HK18 = 2*HK3;
const ftype HK19 = 2*HK0 - 2*HK2;
const ftype HK20 = 2*HK8;
const ftype HK21 = 2*HK4 - 2*HK5 + 2*HK7;
const ftype HK22 = -HK15*P[0][22] + HK15*P[0][4] + HK16*P[0][23] - HK16*P[0][5] + HK17*P[0][6] - HK18*P[0][1] + HK19*P[0][0] - HK20*P[0][3] + HK21*P[0][2];
const ftype HK23 = HK16*P[5][23];
const ftype HK24 = -HK15*P[22][23] + HK15*P[4][23] + HK16*P[23][23] + HK17*P[6][23] - HK18*P[1][23] + HK19*P[0][23] - HK20*P[3][23] + HK21*P[2][23] - HK23;
const ftype HK25 = HK15*P[4][5] - HK15*P[5][22] - HK16*P[5][5] + HK17*P[5][6] - HK18*P[1][5] + HK19*P[0][5] - HK20*P[3][5] + HK21*P[2][5] + HK23;
const ftype HK26 = HK15*P[4][6] - HK15*P[6][22] - HK16*P[5][6] + HK16*P[6][23] + HK17*P[6][6] - HK18*P[1][6] + HK19*P[0][6] - HK20*P[3][6] + HK21*P[2][6];
const ftype HK27 = -HK15*P[1][22] + HK15*P[1][4] + HK16*P[1][23] - HK16*P[1][5] + HK17*P[1][6] - HK18*P[1][1] + HK19*P[0][1] - HK20*P[1][3] + HK21*P[1][2];
const ftype HK28 = HK15*P[4][22];
const ftype HK29 = HK15*P[4][4] + HK16*P[4][23] - HK16*P[4][5] + HK17*P[4][6] - HK18*P[1][4] + HK19*P[0][4] - HK20*P[3][4] + HK21*P[2][4] - HK28;
const ftype HK30 = -HK15*P[22][22] + HK16*P[22][23] - HK16*P[5][22] + HK17*P[6][22] - HK18*P[1][22] + HK19*P[0][22] - HK20*P[3][22] + HK21*P[2][22] + HK28;
const ftype HK31 = -HK15*P[3][22] + HK15*P[3][4] + HK16*P[3][23] - HK16*P[3][5] + HK17*P[3][6] - HK18*P[1][3] + HK19*P[0][3] - HK20*P[3][3] + HK21*P[2][3];
const ftype HK32 = -HK15*P[2][22] + HK15*P[2][4] + HK16*P[2][23] - HK16*P[2][5] + HK17*P[2][6] - HK18*P[1][2] + HK19*P[0][2] - HK20*P[2][3] + HK21*P[2][2];
const ftype HK33 = 1.0F/(HK15*HK29 - HK15*HK30 + HK16*HK24 - HK16*HK25 + HK17*HK26 - HK18*HK27 + HK19*HK22 - HK20*HK31 + HK21*HK32 + R_OBS);


// Observation Jacobians
Hfusion[0] = -2*HK0 + 2*HK2;
Hfusion[1] = 2*HK3;
Hfusion[2] = -2*HK4 + 2*HK5 - 2*HK7;
Hfusion[3] = 2*HK8;
Hfusion[4] = -HK10 - HK9 + 1;
Hfusion[5] = HK12;
Hfusion[6] = 2*HK13 - 2*HK14;
Hfusion[7] = 0;
Hfusion[8] = 0;
Hfusion[9] = 0;
Hfusion[10] = 0;
Hfusion[11] = 0;
Hfusion[12] = 0;
Hfusion[13] = 0;
Hfusion[14] = 0;
Hfusion[15] = 0;
Hfusion[16] = 0;
Hfusion[17] = 0;
Hfusion[18] = 0;
Hfusion[19] = 0;
Hfusion[20] = 0;
Hfusion[21] = 0;
Hfusion[22] = HK15;
Hfusion[23] = -HK12;


// Kalman gains
Kfusion[0] = -HK22*HK33;
Kfusion[1] = -HK27*HK33;
Kfusion[2] = -HK32*HK33;
Kfusion[3] = -HK31*HK33;
Kfusion[4] = -HK29*HK33;
Kfusion[5] = -HK25*HK33;
Kfusion[6] = -HK26*HK33;
Kfusion[7] = -HK33*(HK15*P[4][7] - HK15*P[7][22] - HK16*P[5][7] + HK16*P[7][23] + HK17*P[6][7] - HK18*P[1][7] + HK19*P[0][7] - HK20*P[3][7] + HK21*P[2][7]);
Kfusion[8] = -HK33*(HK15*P[4][8] - HK15*P[8][22] - HK16*P[5][8] + HK16*P[8][23] + HK17*P[6][8] - HK18*P[1][8] + HK19*P[0][8] - HK20*P[3][8] + HK21*P[2][8]);
Kfusion[9] = -HK33*(HK15*P[4][9] - HK15*P[9][22] - HK16*P[5][9] + HK16*P[9][23] + HK17*P[6][9] - HK18*P[1][9] + HK19*P[0][9] - HK20*P[3][9] + HK21*P[2][9]);
Kfusion[10] = -HK33*(-HK15*P[10][22] + HK15*P[4][10] + HK16*P[10][23] - HK16*P[5][10] + HK17*P[6][10] - HK18*P[1][10] + HK19*P[0][10] - HK20*P[3][10] + HK21*P[2][10]);
Kfusion[11] = -HK33*(-HK15*P[11][22] + HK15*P[4][11] + HK16*P[11][23] - HK16*P[5][11] + HK17*P[6][11] - HK18*P[1][11] + HK19*P[0][11] - HK20*P[3][11] + HK21*P[2][11]);
Kfusion[12] = -HK33*(-HK15*P[12][22] + HK15*P[4][12] + HK16*P[12][23] - HK16*P[5][12] + HK17*P[6][12] - HK18*P[1][12] + HK19*P[0][12] - HK20*P[3][12] + HK21*P[2][12]);
Kfusion[13] = -HK33*(-HK15*P[13][22] + HK15*P[4][13] + HK16*P[13][23] - HK16*P[5][13] + HK17*P[6][13] - HK18*P[1][13] + HK19*P[0][13] - HK20*P[3][13] + HK21*P[2][13]);
Kfusion[14] = -HK33*(-HK15*P[14][22] + HK15*P[4][14] + HK16*P[14][23] - HK16*P[5][14] + HK17*P[6][14] - HK18*P[1][14] + HK19*P[0][14] - HK20*P[3][14] + HK21*P[2][14]);
Kfusion[15] = -HK33*(-HK15*P[15][22] + HK15*P[4][15] + HK16*P[15][23] - HK16*P[5][15] + HK17*P[6][15] - HK18*P[1][15] + HK19*P[0][15] - HK20*P[3][15] + HK21*P[2][15]);
Kfusion[16] = -HK33*(-HK15*P[16][22] + HK15*P[4][16] + HK16*P[16][23] - HK16*P[5][16] + HK17*P[6][16] - HK18*P[1][16] + HK19*P[0][16] - HK20*P[3][16] + HK21*P[2][16]);
Kfusion[17] = -HK33*(-HK15*P[17][22] + HK15*P[4][17] + HK16*P[17][23] - HK16*P[5][17] + HK17*P[6][17] - HK18*P[1][17] + HK19*P[0][17] - HK20*P[3][17] + HK21*P[2][17]);
Kfusion[18] = -HK33*(-HK15*P[18][22] + HK15*P[4][18] + HK16*P[18][23] - HK16*P[5][18] + HK17*P[6][18] - HK18*P[1][18] + HK19*P[0][18] - HK20*P[3][18] + HK21*P[2][18]);
Kfusion[19] = -HK33*(-HK15*P[19][22] + HK15*P[4][19] + HK16*P[19][23] - HK16*P[5][19] + HK17*P[6][19] - HK18*P[1][19] + HK19*P[0][19] - HK20*P[3][19] + HK21*P[2][19]);
Kfusion[20] = -HK33*(-HK15*P[20][22] + HK15*P[4][20] + HK16*P[20][23] - HK16*P[5][20] + HK17*P[6][20] - HK18*P[1][20] + HK19*P[0][20] - HK20*P[3][20] + HK21*P[2][20]);
Kfusion[21] = -HK33*(-HK15*P[21][22] + HK15*P[4][21] + HK16*P[21][23] - HK16*P[5][21] + HK17*P[6][21] - HK18*P[1][21] + HK19*P[0][21] - HK20*P[3][21] + HK21*P[2][21]);
Kfusion[22] = -HK30*HK33;
Kfusion[23] = -HK24*HK33;


// Axis 1 equations
// Sub Expressions
const ftype HK0 = vn - vwn;
const ftype HK1 = -HK0*q3 + q1*vd;
const ftype HK2 = 2*ve - 2*vwe;
const ftype HK3 = HK0*q2 - HK2*q1 + q0*vd;
const ftype HK4 = HK0*q1 + q3*vd;
const ftype HK5 = q2*vd;
const ftype HK6 = HK0*q0;
const ftype HK7 = HK2*q3;
const ftype HK8 = q1*q2;
const ftype HK9 = q0*q3;
const ftype HK10 = 2*powf(q1, 2);
const ftype HK11 = 2*powf(q3, 2);
const ftype HK12 = q0*q1 + q2*q3;
const ftype HK13 = -HK8 + HK9;
const ftype HK14 = HK10 + HK11 - 1;
const ftype HK15 = 2*HK12;
const ftype HK16 = 2*HK13;
const ftype HK17 = 2*HK4;
const ftype HK18 = 2*HK1;
const ftype HK19 = 2*HK3;
const ftype HK20 = -2*HK5 + 2*HK6 + 2*HK7;
const ftype HK21 = HK14*P[0][23] - HK14*P[0][5] + HK15*P[0][6] + HK16*P[0][22] - HK16*P[0][4] + HK17*P[0][2] + HK18*P[0][0] + HK19*P[0][1] - HK20*P[0][3];
const ftype HK22 = -HK14*P[5][6] + HK14*P[6][23] + HK15*P[6][6] - HK16*P[4][6] + HK16*P[6][22] + HK17*P[2][6] + HK18*P[0][6] + HK19*P[1][6] - HK20*P[3][6];
const ftype HK23 = HK16*P[4][22];
const ftype HK24 = HK14*P[22][23] - HK14*P[5][22] + HK15*P[6][22] + HK16*P[22][22] + HK17*P[2][22] + HK18*P[0][22] + HK19*P[1][22] - HK20*P[3][22] - HK23;
const ftype HK25 = HK14*P[4][23] - HK14*P[4][5] + HK15*P[4][6] - HK16*P[4][4] + HK17*P[2][4] + HK18*P[0][4] + HK19*P[1][4] - HK20*P[3][4] + HK23;
const ftype HK26 = HK14*P[2][23] - HK14*P[2][5] + HK15*P[2][6] + HK16*P[2][22] - HK16*P[2][4] + HK17*P[2][2] + HK18*P[0][2] + HK19*P[1][2] - HK20*P[2][3];
const ftype HK27 = HK14*P[5][23];
const ftype HK28 = HK14*P[23][23] + HK15*P[6][23] + HK16*P[22][23] - HK16*P[4][23] + HK17*P[2][23] + HK18*P[0][23] + HK19*P[1][23] - HK20*P[3][23] - HK27;
const ftype HK29 = -HK14*P[5][5] + HK15*P[5][6] - HK16*P[4][5] + HK16*P[5][22] + HK17*P[2][5] + HK18*P[0][5] + HK19*P[1][5] - HK20*P[3][5] + HK27;
const ftype HK30 = HK14*P[1][23] - HK14*P[1][5] + HK15*P[1][6] + HK16*P[1][22] - HK16*P[1][4] + HK17*P[1][2] + HK18*P[0][1] + HK19*P[1][1] - HK20*P[1][3];
const ftype HK31 = HK14*P[3][23] - HK14*P[3][5] + HK15*P[3][6] + HK16*P[3][22] - HK16*P[3][4] + HK17*P[2][3] + HK18*P[0][3] + HK19*P[1][3] - HK20*P[3][3];
const ftype HK32 = 1.0F/(HK14*HK28 - HK14*HK29 + HK15*HK22 + HK16*HK24 - HK16*HK25 + HK17*HK26 + HK18*HK21 + HK19*HK30 - HK20*HK31 + R_OBS);


// Observation Jacobians
Hfusion[0] = 2*HK1;
Hfusion[1] = 2*HK3;
Hfusion[2] = 2*HK4;
Hfusion[3] = 2*HK5 - 2*HK6 - 2*HK7;
Hfusion[4] = 2*HK8 - 2*HK9;
Hfusion[5] = -HK10 - HK11 + 1;
Hfusion[6] = 2*HK12;
Hfusion[7] = 0;
Hfusion[8] = 0;
Hfusion[9] = 0;
Hfusion[10] = 0;
Hfusion[11] = 0;
Hfusion[12] = 0;
Hfusion[13] = 0;
Hfusion[14] = 0;
Hfusion[15] = 0;
Hfusion[16] = 0;
Hfusion[17] = 0;
Hfusion[18] = 0;
Hfusion[19] = 0;
Hfusion[20] = 0;
Hfusion[21] = 0;
Hfusion[22] = 2*HK13;
Hfusion[23] = HK14;


// Kalman gains
Kfusion[0] = HK21*HK32;
Kfusion[1] = HK30*HK32;
Kfusion[2] = HK26*HK32;
Kfusion[3] = HK31*HK32;
Kfusion[4] = HK25*HK32;
Kfusion[5] = HK29*HK32;
Kfusion[6] = HK22*HK32;
Kfusion[7] = HK32*(-HK14*P[5][7] + HK14*P[7][23] + HK15*P[6][7] - HK16*P[4][7] + HK16*P[7][22] + HK17*P[2][7] + HK18*P[0][7] + HK19*P[1][7] - HK20*P[3][7]);
Kfusion[8] = HK32*(-HK14*P[5][8] + HK14*P[8][23] + HK15*P[6][8] - HK16*P[4][8] + HK16*P[8][22] + HK17*P[2][8] + HK18*P[0][8] + HK19*P[1][8] - HK20*P[3][8]);
Kfusion[9] = HK32*(-HK14*P[5][9] + HK14*P[9][23] + HK15*P[6][9] - HK16*P[4][9] + HK16*P[9][22] + HK17*P[2][9] + HK18*P[0][9] + HK19*P[1][9] - HK20*P[3][9]);
Kfusion[10] = HK32*(HK14*P[10][23] - HK14*P[5][10] + HK15*P[6][10] + HK16*P[10][22] - HK16*P[4][10] + HK17*P[2][10] + HK18*P[0][10] + HK19*P[1][10] - HK20*P[3][10]);
Kfusion[11] = HK32*(HK14*P[11][23] - HK14*P[5][11] + HK15*P[6][11] + HK16*P[11][22] - HK16*P[4][11] + HK17*P[2][11] + HK18*P[0][11] + HK19*P[1][11] - HK20*P[3][11]);
Kfusion[12] = HK32*(HK14*P[12][23] - HK14*P[5][12] + HK15*P[6][12] + HK16*P[12][22] - HK16*P[4][12] + HK17*P[2][12] + HK18*P[0][12] + HK19*P[1][12] - HK20*P[3][12]);
Kfusion[13] = HK32*(HK14*P[13][23] - HK14*P[5][13] + HK15*P[6][13] + HK16*P[13][22] - HK16*P[4][13] + HK17*P[2][13] + HK18*P[0][13] + HK19*P[1][13] - HK20*P[3][13]);
Kfusion[14] = HK32*(HK14*P[14][23] - HK14*P[5][14] + HK15*P[6][14] + HK16*P[14][22] - HK16*P[4][14] + HK17*P[2][14] + HK18*P[0][14] + HK19*P[1][14] - HK20*P[3][14]);
Kfusion[15] = HK32*(HK14*P[15][23] - HK14*P[5][15] + HK15*P[6][15] + HK16*P[15][22] - HK16*P[4][15] + HK17*P[2][15] + HK18*P[0][15] + HK19*P[1][15] - HK20*P[3][15]);
Kfusion[16] = HK32*(HK14*P[16][23] - HK14*P[5][16] + HK15*P[6][16] + HK16*P[16][22] - HK16*P[4][16] + HK17*P[2][16] + HK18*P[0][16] + HK19*P[1][16] - HK20*P[3][16]);
Kfusion[17] = HK32*(HK14*P[17][23] - HK14*P[5][17] + HK15*P[6][17] + HK16*P[17][22] - HK16*P[4][17] + HK17*P[2][17] + HK18*P[0][17] + HK19*P[1][17] - HK20*P[3][17]);
Kfusion[18] = HK32*(HK14*P[18][23] - HK14*P[5][18] + HK15*P[6][18] + HK16*P[18][22] - HK16*P[4][18] + HK17*P[2][18] + HK18*P[0][18] + HK19*P[1][18] - HK20*P[3][18]);
Kfusion[19] = HK32*(HK14*P[19][23] - HK14*P[5][19] + HK15*P[6][19] + HK16*P[19][22] - HK16*P[4][19] + HK17*P[2][19] + HK18*P[0][19] + HK19*P[1][19] - HK20*P[3][19]);
Kfusion[20] = HK32*(HK14*P[20][23] - HK14*P[5][20] + HK15*P[6][20] + HK16*P[20][22] - HK16*P[4][20] + HK17*P[2][20] + HK18*P[0][20] + HK19*P[1][20] - HK20*P[3][20]);
Kfusion[21] = HK32*(HK14*P[21][23] - HK14*P[5][21] + HK15*P[6][21] + HK16*P[21][22] - HK16*P[4][21] + HK17*P[2][21] + HK18*P[0][21] + HK19*P[1][21] - HK20*P[3][21]);
Kfusion[22] = HK24*HK32;
Kfusion[23] = HK28*HK32;


