t2 = magE*magE;
t3 = magN*magN;
t4 = t2+t3;
t5 = P[16][16]*t2;
t6 = P[17][17]*t3;
t7 = t2*t2;
t8 = R_DECL*t7;
t9 = t3*t3;
t10 = R_DECL*t9;
t11 = R_DECL*t2*t3*2.0;
t14 = P[16][17]*magE*magN;
t15 = P[17][16]*magE*magN;
t12 = t5+t6+t8+t10+t11-t14-t15;
t13 = 1.0/t12;
t16 = conjugate(magE);
t17 = conjugate(magN);
t18 = t16*t16;
t19 = t17*t17;
t20 = t18+t19;
t21 = 1.0/t20;
A0[0][0] = -t4*t13*(P[0][16]*magE-P[0][17]*magN);
A0[1][0] = -t4*t13*(P[1][16]*magE-P[1][17]*magN);
A0[2][0] = -t4*t13*(P[2][16]*magE-P[2][17]*magN);
A0[3][0] = -t4*t13*(P[3][16]*magE-P[3][17]*magN);
A0[4][0] = -t4*t13*(P[4][16]*magE-P[4][17]*magN);
A0[5][0] = -t4*t13*(P[5][16]*magE-P[5][17]*magN);
A0[6][0] = -t4*t13*(P[6][16]*magE-P[6][17]*magN);
A0[7][0] = -t4*t13*(P[7][16]*magE-P[7][17]*magN);
A0[8][0] = -t4*t13*(P[8][16]*magE-P[8][17]*magN);
A0[9][0] = -t4*t13*(P[9][16]*magE-P[9][17]*magN);
A0[10][0] = -t4*t13*(P[10][16]*magE-P[10][17]*magN);
A0[11][0] = -t4*t13*(P[11][16]*magE-P[11][17]*magN);
A0[12][0] = -t4*t13*(P[12][16]*magE-P[12][17]*magN);
A0[13][0] = -t4*t13*(P[13][16]*magE-P[13][17]*magN);
A0[14][0] = -t4*t13*(P[14][16]*magE-P[14][17]*magN);
A0[15][0] = -t4*t13*(P[15][16]*magE-P[15][17]*magN);
A0[16][0] = -t4*t13*(P[16][16]*magE-P[16][17]*magN);
A0[16][1] = -t16*t21;
A0[17][0] = -t4*t13*(P[17][16]*magE-P[17][17]*magN);
A0[17][1] = t17*t21;
A0[18][0] = -t4*t13*(P[18][16]*magE-P[18][17]*magN);
A0[19][0] = -t4*t13*(P[19][16]*magE-P[19][17]*magN);
A0[20][0] = -t4*t13*(P[20][16]*magE-P[20][17]*magN);
A0[21][0] = -t4*t13*(P[21][16]*magE-P[21][17]*magN);
A0[22][0] = -t4*t13*(P[22][16]*magE-P[22][17]*magN);
A0[23][0] = -t4*t13*(P[23][16]*magE-P[23][17]*magN);
A0[24][0] = -t4*t13*(P[24][16]*magE-P[24][17]*magN);
