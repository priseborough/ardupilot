t2 = scaleLog*2.0;
t3 = exp(t2);
t4 = exp(scaleLog);
t5 = P[8][8]*t3;
t6 = P[22][8]*pe*t3;
t7 = P[8][22]*pe*t3;
t8 = pe*pe;
t9 = P[22][22]*t3*t8;
t10 = R_POS+t5+t6+t7+t9;
t11 = 1.0/t10;
A0[0][0] = t4*t11*(P[0][8]+P[0][22]*pe);
A0[1][0] = t4*t11*(P[1][8]+P[1][22]*pe);
A0[2][0] = t4*t11*(P[2][8]+P[2][22]*pe);
A0[3][0] = t4*t11*(P[3][8]+P[3][22]*pe);
A0[4][0] = t4*t11*(P[4][8]+P[4][22]*pe);
A0[5][0] = t4*t11*(P[5][8]+P[5][22]*pe);
A0[6][0] = t4*t11*(P[6][8]+P[6][22]*pe);
A0[7][0] = t4*t11*(P[7][8]+P[7][22]*pe);
A0[8][0] = t4*t11*(P[8][8]+P[8][22]*pe);
A0[9][0] = t4*t11*(P[9][8]+P[9][22]*pe);
A0[10][0] = t4*t11*(P[10][8]+P[10][22]*pe);
A0[11][0] = t4*t11*(P[11][8]+P[11][22]*pe);
A0[12][0] = t4*t11*(P[12][8]+P[12][22]*pe);
A0[13][0] = t4*t11*(P[13][8]+P[13][22]*pe);
A0[14][0] = t4*t11*(P[14][8]+P[14][22]*pe);
A0[15][0] = t4*t11*(P[15][8]+P[15][22]*pe);
A0[16][0] = t4*t11*(P[16][8]+P[16][22]*pe);
A0[17][0] = t4*t11*(P[17][8]+P[17][22]*pe);
A0[18][0] = t4*t11*(P[18][8]+P[18][22]*pe);
A0[19][0] = t4*t11*(P[19][8]+P[19][22]*pe);
A0[20][0] = t4*t11*(P[20][8]+P[20][22]*pe);
A0[21][0] = t4*t11*(P[21][8]+P[21][22]*pe);
A0[22][0] = t4*t11*(P[22][8]+P[22][22]*pe);
A0[23][0] = t4*t11*(P[23][8]+P[23][22]*pe);
A0[24][0] = t4*t11*(P[24][8]+P[24][22]*pe);
