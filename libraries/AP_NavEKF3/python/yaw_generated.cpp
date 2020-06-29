// calculate 321 yaw observation matrix - option A
float SA0 = 2*q3;
float SA1 = 2*q2;
float SA2 = SA0*q0 + SA1*q1;
float SA3 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float SA4 = powf(SA3, -2);
float SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
float SA6 = 1.0F/SA3;
float SA7 = SA2*SA4;
float SA8 = 2*SA7;
float SA9 = 2*SA6;


H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
H_YAW[1] = SA5*(SA1*SA6 - SA8*q1);
H_YAW[2] = SA5*(SA1*SA7 + SA9*q1);
H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 321 yaw observation matrix - option B
float SB0 = 2*q0;
float SB1 = 2*q1;
float SB2 = SB0*q3 + SB1*q2;
float SB3 = powf(SB2, -2);
float SB4 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
float SB5 = 1.0F/(SB3*powf(SB4, 2) + 1);
float SB6 = 1.0F/SB2;
float SB7 = SB3*SB4;
float SB8 = 2*SB7;
float SB9 = 2*SB6;


H_YAW[0] = -SB5*(SB0*SB6 - SB8*q3);
H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
H_YAW[3] = -SB5*(-SB0*SB7 - SB9*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 312 yaw observation matrix - option A
float SA0 = 2*q3;
float SA1 = 2*q2;
float SA2 = SA0*q0 - SA1*q1;
float SA3 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
float SA4 = powf(SA3, -2);
float SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
float SA6 = 1.0F/SA3;
float SA7 = SA2*SA4;
float SA8 = 2*SA7;
float SA9 = 2*SA6;


H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
H_YAW[1] = SA5*(-SA1*SA6 + SA8*q1);
H_YAW[2] = SA5*(-SA1*SA7 - SA9*q1);
H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 312 yaw observation matrix - option B
float SB0 = 2*q0;
float SB1 = 2*q1;
float SB2 = -SB0*q3 + SB1*q2;
float SB3 = powf(SB2, -2);
float SB4 = -powf(q0, 2) + powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
float SB5 = 1.0F/(SB3*powf(SB4, 2) + 1);
float SB6 = 1.0F/SB2;
float SB7 = SB3*SB4;
float SB8 = 2*SB7;
float SB9 = 2*SB6;


H_YAW[0] = -SB5*(-SB0*SB6 + SB8*q3);
H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
H_YAW[3] = -SB5*(SB0*SB7 + SB9*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


