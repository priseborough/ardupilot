// Equations for covariance matrix prediction, without process noise!
float PS0 = powf(q1, 2);
float PS1 = 0.25F*daxVar;
float PS2 = powf(q2, 2);
float PS3 = 0.25F*dayVar;
float PS4 = powf(q3, 2);
float PS5 = 0.25F*dazVar;
float PS6 = 0.5F*q1;
float PS7 = 0.5F*q2;
float PS8 = PS7*P[10][11];
float PS9 = 0.5F*q3;
float PS10 = PS9*P[10][12];
float PS11 = 0.5F*dax - 0.5F*dax_b;
float PS12 = 0.5F*day - 0.5F*day_b;
float PS13 = 0.5F*daz - 0.5F*daz_b;
float PS14 = PS10 - PS11*P[1][10] - PS12*P[2][10] - PS13*P[3][10] + PS6*P[10][10] + PS8 + P[0][10];
float PS15 = PS6*P[10][11];
float PS16 = PS9*P[11][12];
float PS17 = -PS11*P[1][11] - PS12*P[2][11] - PS13*P[3][11] + PS15 + PS16 + PS7*P[11][11] + P[0][11];
float PS18 = PS6*P[10][12];
float PS19 = PS7*P[11][12];
float PS20 = -PS11*P[1][12] - PS12*P[2][12] - PS13*P[3][12] + PS18 + PS19 + PS9*P[12][12] + P[0][12];
float PS21 = PS12*P[1][2];
float PS22 = -PS13*P[1][3];
float PS23 = -PS11*P[1][1] - PS21 + PS22 + PS6*P[1][10] + PS7*P[1][11] + PS9*P[1][12] + P[0][1];
float PS24 = -PS11*P[1][2];
float PS25 = PS13*P[2][3];
float PS26 = -PS12*P[2][2] + PS24 - PS25 + PS6*P[2][10] + PS7*P[2][11] + PS9*P[2][12] + P[0][2];
float PS27 = PS11*P[1][3];
float PS28 = -PS12*P[2][3];
float PS29 = -PS13*P[3][3] - PS27 + PS28 + PS6*P[3][10] + PS7*P[3][11] + PS9*P[3][12] + P[0][3];
float PS30 = PS11*P[0][1];
float PS31 = PS12*P[0][2];
float PS32 = PS13*P[0][3];
float PS33 = -PS30 - PS31 - PS32 + PS6*P[0][10] + PS7*P[0][11] + PS9*P[0][12] + P[0][0];
float PS34 = 0.5F*q0;
float PS35 = q2*q3;
float PS36 = q0*q1;
float PS37 = q1*q3;
float PS38 = q0*q2;
float PS39 = q1*q2;
float PS40 = q0*q3;
float PS41 = -PS2;
float PS42 = powf(q0, 2);
float PS43 = -PS4 + PS42;
float PS44 = PS0 + PS41 + PS43;
float PS45 = -PS11*P[1][13] - PS12*P[2][13] - PS13*P[3][13] + PS6*P[10][13] + PS7*P[11][13] + PS9*P[12][13] + P[0][13];
float PS46 = PS37 + PS38;
float PS47 = -PS11*P[1][15] - PS12*P[2][15] - PS13*P[3][15] + PS6*P[10][15] + PS7*P[11][15] + PS9*P[12][15] + P[0][15];
float PS48 = 2*PS47;
float PS49 = dvy - dvy_b;
float PS50 = dvx - dvx_b;
float PS51 = dvz - dvz_b;
float PS52 = PS49*q0 + PS50*q3 - PS51*q1;
float PS53 = 2*PS29;
float PS54 = -PS39 + PS40;
float PS55 = -PS11*P[1][14] - PS12*P[2][14] - PS13*P[3][14] + PS6*P[10][14] + PS7*P[11][14] + PS9*P[12][14] + P[0][14];
float PS56 = 2*PS55;
float PS57 = -PS49*q3 + PS50*q0 + PS51*q2;
float PS58 = 2*PS33;
float PS59 = PS49*q1 - PS50*q2 + PS51*q0;
float PS60 = 2*PS59;
float PS61 = PS49*q2 + PS50*q1 + PS51*q3;
float PS62 = 2*PS61;
float PS63 = -PS11*P[1][4] - PS12*P[2][4] - PS13*P[3][4] + PS6*P[4][10] + PS7*P[4][11] + PS9*P[4][12] + P[0][4];
float PS64 = -PS0;
float PS65 = PS2 + PS43 + PS64;
float PS66 = PS39 + PS40;
float PS67 = 2*PS45;
float PS68 = -PS35 + PS36;
float PS69 = -PS11*P[1][5] - PS12*P[2][5] - PS13*P[3][5] + PS6*P[5][10] + PS7*P[5][11] + PS9*P[5][12] + P[0][5];
float PS70 = PS4 + PS41 + PS42 + PS64;
float PS71 = PS35 + PS36;
float PS72 = 2*PS57;
float PS73 = -PS37 + PS38;
float PS74 = 2*PS52;
float PS75 = -PS11*P[1][6] - PS12*P[2][6] - PS13*P[3][6] + PS6*P[6][10] + PS7*P[6][11] + PS9*P[6][12] + P[0][6];
float PS76 = -PS34*P[10][11];
float PS77 = PS11*P[0][11] - PS12*P[3][11] + PS13*P[2][11] - PS19 + PS76 + PS9*P[11][11] + P[1][11];
float PS78 = PS13*P[0][2];
float PS79 = PS12*P[0][3];
float PS80 = PS11*P[0][0] - PS34*P[0][10] - PS7*P[0][12] + PS78 - PS79 + PS9*P[0][11] + P[0][1];
float PS81 = PS11*P[0][2];
float PS82 = PS13*P[2][2] + PS28 - PS34*P[2][10] - PS7*P[2][12] + PS81 + PS9*P[2][11] + P[1][2];
float PS83 = PS9*P[10][11];
float PS84 = PS7*P[10][12];
float PS85 = PS11*P[0][10] - PS12*P[3][10] + PS13*P[2][10] - PS34*P[10][10] + PS83 - PS84 + P[1][10];
float PS86 = -PS34*P[10][12];
float PS87 = PS11*P[0][12] - PS12*P[3][12] + PS13*P[2][12] + PS16 - PS7*P[12][12] + PS86 + P[1][12];
float PS88 = PS11*P[0][3];
float PS89 = -PS12*P[3][3] + PS25 - PS34*P[3][10] - PS7*P[3][12] + PS88 + PS9*P[3][11] + P[1][3];
float PS90 = PS13*P[1][2];
float PS91 = PS12*P[1][3];
float PS92 = PS30 - PS34*P[1][10] - PS7*P[1][12] + PS9*P[1][11] + PS90 - PS91 + P[1][1];
float PS93 = PS11*P[0][13] - PS12*P[3][13] + PS13*P[2][13] - PS34*P[10][13] - PS7*P[12][13] + PS9*P[11][13] + P[1][13];
float PS94 = PS11*P[0][15] - PS12*P[3][15] + PS13*P[2][15] - PS34*P[10][15] - PS7*P[12][15] + PS9*P[11][15] + P[1][15];
float PS95 = 2*PS94;
float PS96 = PS11*P[0][14] - PS12*P[3][14] + PS13*P[2][14] - PS34*P[10][14] - PS7*P[12][14] + PS9*P[11][14] + P[1][14];
float PS97 = 2*PS96;
float PS98 = PS11*P[0][4] - PS12*P[3][4] + PS13*P[2][4] - PS34*P[4][10] - PS7*P[4][12] + PS9*P[4][11] + P[1][4];
float PS99 = 2*PS93;
float PS100 = PS11*P[0][5] - PS12*P[3][5] + PS13*P[2][5] - PS34*P[5][10] - PS7*P[5][12] + PS9*P[5][11] + P[1][5];
float PS101 = PS11*P[0][6] - PS12*P[3][6] + PS13*P[2][6] - PS34*P[6][10] - PS7*P[6][12] + PS9*P[6][11] + P[1][6];
float PS102 = -PS34*P[11][12];
float PS103 = -PS10 + PS102 + PS11*P[3][12] + PS12*P[0][12] - PS13*P[1][12] + PS6*P[12][12] + P[2][12];
float PS104 = PS11*P[3][3] + PS22 - PS34*P[3][11] + PS6*P[3][12] + PS79 - PS9*P[3][10] + P[2][3];
float PS105 = PS13*P[0][1];
float PS106 = -PS105 + PS12*P[0][0] - PS34*P[0][11] + PS6*P[0][12] + PS88 - PS9*P[0][10] + P[0][2];
float PS107 = PS6*P[11][12];
float PS108 = PS107 + PS11*P[3][11] + PS12*P[0][11] - PS13*P[1][11] - PS34*P[11][11] - PS83 + P[2][11];
float PS109 = PS11*P[3][10] + PS12*P[0][10] - PS13*P[1][10] + PS18 + PS76 - PS9*P[10][10] + P[2][10];
float PS110 = PS12*P[0][1];
float PS111 = PS110 - PS13*P[1][1] + PS27 - PS34*P[1][11] + PS6*P[1][12] - PS9*P[1][10] + P[1][2];
float PS112 = PS11*P[2][3];
float PS113 = PS112 + PS31 - PS34*P[2][11] + PS6*P[2][12] - PS9*P[2][10] - PS90 + P[2][2];
float PS114 = PS11*P[3][13] + PS12*P[0][13] - PS13*P[1][13] - PS34*P[11][13] + PS6*P[12][13] - PS9*P[10][13] + P[2][13];
float PS115 = PS11*P[3][15] + PS12*P[0][15] - PS13*P[1][15] - PS34*P[11][15] + PS6*P[12][15] - PS9*P[10][15] + P[2][15];
float PS116 = 2*PS115;
float PS117 = PS11*P[3][14] + PS12*P[0][14] - PS13*P[1][14] - PS34*P[11][14] + PS6*P[12][14] - PS9*P[10][14] + P[2][14];
float PS118 = 2*PS117;
float PS119 = PS11*P[3][4] + PS12*P[0][4] - PS13*P[1][4] - PS34*P[4][11] + PS6*P[4][12] - PS9*P[4][10] + P[2][4];
float PS120 = 2*PS114;
float PS121 = PS11*P[3][5] + PS12*P[0][5] - PS13*P[1][5] - PS34*P[5][11] + PS6*P[5][12] - PS9*P[5][10] + P[2][5];
float PS122 = PS11*P[3][6] + PS12*P[0][6] - PS13*P[1][6] - PS34*P[6][11] + PS6*P[6][12] - PS9*P[6][10] + P[2][6];
float PS123 = -PS11*P[2][10] + PS12*P[1][10] + PS13*P[0][10] - PS15 + PS7*P[10][10] + PS86 + P[3][10];
float PS124 = PS105 + PS12*P[1][1] + PS24 - PS34*P[1][12] - PS6*P[1][11] + PS7*P[1][10] + P[1][3];
float PS125 = PS110 + PS13*P[0][0] - PS34*P[0][12] - PS6*P[0][11] + PS7*P[0][10] - PS81 + P[0][3];
float PS126 = -PS107 - PS11*P[2][12] + PS12*P[1][12] + PS13*P[0][12] - PS34*P[12][12] + PS84 + P[3][12];
float PS127 = PS102 - PS11*P[2][11] + PS12*P[1][11] + PS13*P[0][11] - PS6*P[11][11] + PS8 + P[3][11];
float PS128 = -PS11*P[2][2] + PS21 - PS34*P[2][12] - PS6*P[2][11] + PS7*P[2][10] + PS78 + P[2][3];
float PS129 = -PS112 + PS32 - PS34*P[3][12] - PS6*P[3][11] + PS7*P[3][10] + PS91 + P[3][3];
float PS130 = -PS11*P[2][13] + PS12*P[1][13] + PS13*P[0][13] - PS34*P[12][13] - PS6*P[11][13] + PS7*P[10][13] + P[3][13];
float PS131 = -PS11*P[2][15] + PS12*P[1][15] + PS13*P[0][15] - PS34*P[12][15] - PS6*P[11][15] + PS7*P[10][15] + P[3][15];
float PS132 = 2*PS131;
float PS133 = -PS11*P[2][14] + PS12*P[1][14] + PS13*P[0][14] - PS34*P[12][14] - PS6*P[11][14] + PS7*P[10][14] + P[3][14];
float PS134 = 2*PS133;
float PS135 = -PS11*P[2][4] + PS12*P[1][4] + PS13*P[0][4] - PS34*P[4][12] - PS6*P[4][11] + PS7*P[4][10] + P[3][4];
float PS136 = 2*PS130;
float PS137 = -PS11*P[2][5] + PS12*P[1][5] + PS13*P[0][5] - PS34*P[5][12] - PS6*P[5][11] + PS7*P[5][10] + P[3][5];
float PS138 = -PS11*P[2][6] + PS12*P[1][6] + PS13*P[0][6] - PS34*P[6][12] - PS6*P[6][11] + PS7*P[6][10] + P[3][6];
float PS139 = 2*PS46;
float PS140 = 2*PS54;
float PS141 = -PS139*P[13][15] + PS140*P[13][14] - PS44*P[13][13] + PS60*P[2][13] + PS62*P[1][13] + PS72*P[0][13] - PS74*P[3][13] + P[4][13];
float PS142 = -PS139*P[15][15] + PS140*P[14][15] - PS44*P[13][15] + PS60*P[2][15] + PS62*P[1][15] + PS72*P[0][15] - PS74*P[3][15] + P[4][15];
float PS143 = PS62*P[1][3];
float PS144 = PS72*P[0][3];
float PS145 = -PS139*P[3][15] + PS140*P[3][14] + PS143 + PS144 - PS44*P[3][13] + PS60*P[2][3] - PS74*P[3][3] + P[3][4];
float PS146 = -PS139*P[14][15] + PS140*P[14][14] - PS44*P[13][14] + PS60*P[2][14] + PS62*P[1][14] + PS72*P[0][14] - PS74*P[3][14] + P[4][14];
float PS147 = PS60*P[0][2];
float PS148 = PS74*P[0][3];
float PS149 = -PS139*P[0][15] + PS140*P[0][14] + PS147 - PS148 - PS44*P[0][13] + PS62*P[0][1] + PS72*P[0][0] + P[0][4];
float PS150 = PS62*P[1][2];
float PS151 = PS72*P[0][2];
float PS152 = -PS139*P[2][15] + PS140*P[2][14] + PS150 + PS151 - PS44*P[2][13] + PS60*P[2][2] - PS74*P[2][3] + P[2][4];
float PS153 = PS60*P[1][2];
float PS154 = PS74*P[1][3];
float PS155 = -PS139*P[1][15] + PS140*P[1][14] + PS153 - PS154 - PS44*P[1][13] + PS62*P[1][1] + PS72*P[0][1] + P[1][4];
float PS156 = 4*dvyVar;
float PS157 = 4*dvzVar;
float PS158 = -PS139*P[4][15] + PS140*P[4][14] - PS44*P[4][13] + PS60*P[2][4] + PS62*P[1][4] + PS72*P[0][4] - PS74*P[3][4] + P[4][4];
float PS159 = 2*PS141;
float PS160 = 2*PS68;
float PS161 = PS65*dvyVar;
float PS162 = 2*PS66;
float PS163 = PS44*dvxVar;
float PS164 = -PS139*P[5][15] + PS140*P[5][14] - PS44*P[5][13] + PS60*P[2][5] + PS62*P[1][5] + PS72*P[0][5] - PS74*P[3][5] + P[4][5];
float PS165 = 2*PS71;
float PS166 = 2*PS73;
float PS167 = PS70*dvzVar;
float PS168 = -PS139*P[6][15] + PS140*P[6][14] - PS44*P[6][13] + PS60*P[2][6] + PS62*P[1][6] + PS72*P[0][6] - PS74*P[3][6] + P[4][6];
float PS169 = PS160*P[14][15] - PS162*P[13][14] - PS60*P[1][14] + PS62*P[2][14] - PS65*P[14][14] + PS72*P[3][14] + PS74*P[0][14] + P[5][14];
float PS170 = PS160*P[13][15] - PS162*P[13][13] - PS60*P[1][13] + PS62*P[2][13] - PS65*P[13][14] + PS72*P[3][13] + PS74*P[0][13] + P[5][13];
float PS171 = PS74*P[0][1];
float PS172 = PS150 + PS160*P[1][15] - PS162*P[1][13] + PS171 - PS60*P[1][1] - PS65*P[1][14] + PS72*P[1][3] + P[1][5];
float PS173 = PS160*P[15][15] - PS162*P[13][15] - PS60*P[1][15] + PS62*P[2][15] - PS65*P[14][15] + PS72*P[3][15] + PS74*P[0][15] + P[5][15];
float PS174 = PS62*P[2][3];
float PS175 = PS148 + PS160*P[3][15] - PS162*P[3][13] + PS174 - PS60*P[1][3] - PS65*P[3][14] + PS72*P[3][3] + P[3][5];
float PS176 = PS60*P[0][1];
float PS177 = PS144 + PS160*P[0][15] - PS162*P[0][13] - PS176 + PS62*P[0][2] - PS65*P[0][14] + PS74*P[0][0] + P[0][5];
float PS178 = PS72*P[2][3];
float PS179 = -PS153 + PS160*P[2][15] - PS162*P[2][13] + PS178 + PS62*P[2][2] - PS65*P[2][14] + PS74*P[0][2] + P[2][5];
float PS180 = 4*dvxVar;
float PS181 = PS160*P[5][15] - PS162*P[5][13] - PS60*P[1][5] + PS62*P[2][5] - PS65*P[5][14] + PS72*P[3][5] + PS74*P[0][5] + P[5][5];
float PS182 = PS160*P[6][15] - PS162*P[6][13] - PS60*P[1][6] + PS62*P[2][6] - PS65*P[6][14] + PS72*P[3][6] + PS74*P[0][6] + P[5][6];
float PS183 = -PS165*P[14][15] + PS166*P[13][15] + PS60*P[0][15] + PS62*P[3][15] - PS70*P[15][15] - PS72*P[2][15] + PS74*P[1][15] + P[6][15];
float PS184 = -PS165*P[14][14] + PS166*P[13][14] + PS60*P[0][14] + PS62*P[3][14] - PS70*P[14][15] - PS72*P[2][14] + PS74*P[1][14] + P[6][14];
float PS185 = -PS165*P[13][14] + PS166*P[13][13] + PS60*P[0][13] + PS62*P[3][13] - PS70*P[13][15] - PS72*P[2][13] + PS74*P[1][13] + P[6][13];
float PS186 = -PS165*P[6][14] + PS166*P[6][13] + PS60*P[0][6] + PS62*P[3][6] - PS70*P[6][15] - PS72*P[2][6] + PS74*P[1][6] + P[6][6];


nextP[0][0] = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
nextP[0][1] = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
nextP[1][1] = PS1*PS42 + PS11*PS80 - PS12*PS89 + PS13*PS82 + PS2*PS5 + PS3*PS4 - PS34*PS85 - PS7*PS87 + PS77*PS9 + PS92;
nextP[0][2] = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
nextP[1][2] = PS1*PS40 + PS11*PS89 + PS12*PS80 - PS13*PS92 - PS3*PS40 - PS34*PS77 - PS39*PS5 + PS6*PS87 + PS82 - PS85*PS9;
nextP[2][2] = PS0*PS5 + PS1*PS4 + PS103*PS6 + PS104*PS11 + PS106*PS12 - PS108*PS34 - PS109*PS9 - PS111*PS13 + PS113 + PS3*PS42;
nextP[0][3] = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
nextP[1][3] = -PS1*PS38 - PS11*PS82 + PS12*PS92 + PS13*PS80 - PS3*PS37 - PS34*PS87 + PS38*PS5 - PS6*PS77 + PS7*PS85 + PS89;
nextP[2][3] = -PS1*PS35 - PS103*PS34 + PS104 + PS106*PS13 - PS108*PS6 + PS109*PS7 - PS11*PS113 + PS111*PS12 + PS3*PS36 - PS36*PS5;
nextP[3][3] = PS0*PS3 + PS1*PS2 - PS11*PS128 + PS12*PS124 + PS123*PS7 + PS125*PS13 - PS126*PS34 - PS127*PS6 + PS129 + PS42*PS5;
nextP[0][4] = PS23*PS62 + PS26*PS60 - PS44*PS45 - PS46*PS48 - PS52*PS53 + PS54*PS56 + PS57*PS58 + PS63;
nextP[1][4] = -PS44*PS93 - PS46*PS95 + PS54*PS97 + PS60*PS82 + PS62*PS92 + PS72*PS80 - PS74*PS89 + PS98;
nextP[2][4] = -PS104*PS74 + PS106*PS72 + PS111*PS62 + PS113*PS60 - PS114*PS44 - PS116*PS46 + PS118*PS54 + PS119;
nextP[3][4] = PS124*PS62 + PS125*PS72 + PS128*PS60 - PS129*PS74 - PS130*PS44 - PS132*PS46 + PS134*PS54 + PS135;
nextP[4][4] = -PS139*PS142 + PS140*PS146 - PS141*PS44 - PS145*PS74 + PS149*PS72 + PS152*PS60 + PS155*PS62 + PS156*powf(PS54, 2) + PS157*powf(PS46, 2) + PS158 + powf(PS44, 2)*dvxVar;
nextP[0][5] = -PS23*PS60 + PS26*PS62 + PS48*PS68 + PS52*PS58 + PS53*PS57 - PS55*PS65 - PS66*PS67 + PS69;
nextP[1][5] = PS100 - PS60*PS92 + PS62*PS82 - PS65*PS96 - PS66*PS99 + PS68*PS95 + PS72*PS89 + PS74*PS80;
nextP[2][5] = PS104*PS72 + PS106*PS74 - PS111*PS60 + PS113*PS62 + PS116*PS68 - PS117*PS65 - PS120*PS66 + PS121;
nextP[3][5] = -PS124*PS60 + PS125*PS74 + PS128*PS62 + PS129*PS72 + PS132*PS68 - PS133*PS65 - PS136*PS66 + PS137;
nextP[4][5] = -PS140*PS161 + PS142*PS160 + PS145*PS72 - PS146*PS65 + PS149*PS74 + PS152*PS62 - PS155*PS60 - PS157*PS46*PS68 - PS159*PS66 + PS162*PS163 + PS164;
nextP[5][5] = PS157*powf(PS68, 2) + PS160*PS173 - PS162*PS170 - PS169*PS65 - PS172*PS60 + PS175*PS72 + PS177*PS74 + PS179*PS62 + PS180*powf(PS66, 2) + PS181 + powf(PS65, 2)*dvyVar;
nextP[0][6] = PS23*PS74 - PS26*PS72 - PS47*PS70 + PS53*PS61 - PS56*PS71 + PS58*PS59 + PS67*PS73 + PS75;
nextP[1][6] = PS101 + PS60*PS80 + PS62*PS89 - PS70*PS94 - PS71*PS97 - PS72*PS82 + PS73*PS99 + PS74*PS92;
nextP[2][6] = PS104*PS62 + PS106*PS60 + PS111*PS74 - PS113*PS72 - PS115*PS70 - PS118*PS71 + PS120*PS73 + PS122;
nextP[3][6] = PS124*PS74 + PS125*PS60 - PS128*PS72 + PS129*PS62 - PS131*PS70 - PS134*PS71 + PS136*PS73 + PS138;
nextP[4][6] = PS139*PS167 - PS142*PS70 + PS145*PS62 - PS146*PS165 + PS149*PS60 - PS152*PS72 + PS155*PS74 - PS156*PS54*PS71 + PS159*PS73 - PS163*PS166 + PS168;
nextP[5][6] = -PS160*PS167 + PS161*PS165 - PS165*PS169 + PS166*PS170 + PS172*PS74 - PS173*PS70 + PS175*PS62 + PS177*PS60 - PS179*PS72 - PS180*PS66*PS73 + PS182;
nextP[6][6] = PS156*powf(PS71, 2) - PS165*PS184 + PS166*PS185 + PS180*powf(PS73, 2) - PS183*PS70 + PS186 + PS60*(-PS151 - PS165*P[0][14] + PS166*P[0][13] + PS171 + PS60*P[0][0] + PS62*P[0][3] - PS70*P[0][15] + P[0][6]) + PS62*(PS154 - PS165*P[3][14] + PS166*P[3][13] - PS178 + PS60*P[0][3] + PS62*P[3][3] - PS70*P[3][15] + P[3][6]) + powf(PS70, 2)*dvzVar - PS72*(PS147 - PS165*P[2][14] + PS166*P[2][13] + PS174 - PS70*P[2][15] - PS72*P[2][2] + PS74*P[1][2] + P[2][6]) + PS74*(PS143 - PS165*P[1][14] + PS166*P[1][13] + PS176 - PS70*P[1][15] - PS72*P[1][2] + PS74*P[1][1] + P[1][6]);
nextP[0][7] = -PS11*P[1][7] - PS12*P[2][7] - PS13*P[3][7] + PS6*P[7][10] + PS63*dt + PS7*P[7][11] + PS9*P[7][12] + P[0][7];
nextP[1][7] = PS11*P[0][7] - PS12*P[3][7] + PS13*P[2][7] - PS34*P[7][10] - PS7*P[7][12] + PS9*P[7][11] + PS98*dt + P[1][7];
nextP[2][7] = PS11*P[3][7] + PS119*dt + PS12*P[0][7] - PS13*P[1][7] - PS34*P[7][11] + PS6*P[7][12] - PS9*P[7][10] + P[2][7];
nextP[3][7] = -PS11*P[2][7] + PS12*P[1][7] + PS13*P[0][7] + PS135*dt - PS34*P[7][12] - PS6*P[7][11] + PS7*P[7][10] + P[3][7];
nextP[4][7] = -PS139*P[7][15] + PS140*P[7][14] + PS158*dt - PS44*P[7][13] + PS60*P[2][7] + PS62*P[1][7] + PS72*P[0][7] - PS74*P[3][7] + P[4][7];
nextP[5][7] = PS160*P[7][15] - PS162*P[7][13] - PS60*P[1][7] + PS62*P[2][7] - PS65*P[7][14] + PS72*P[3][7] + PS74*P[0][7] + P[5][7] + dt*(PS160*P[4][15] - PS162*P[4][13] - PS60*P[1][4] + PS62*P[2][4] - PS65*P[4][14] + PS72*P[3][4] + PS74*P[0][4] + P[4][5]);
nextP[6][7] = -PS165*P[7][14] + PS166*P[7][13] + PS60*P[0][7] + PS62*P[3][7] - PS70*P[7][15] - PS72*P[2][7] + PS74*P[1][7] + P[6][7] + dt*(-PS165*P[4][14] + PS166*P[4][13] + PS60*P[0][4] + PS62*P[3][4] - PS70*P[4][15] - PS72*P[2][4] + PS74*P[1][4] + P[4][6]);
nextP[7][7] = P[4][7]*dt + P[7][7] + dt*(P[4][4]*dt + P[4][7]);
nextP[0][8] = -PS11*P[1][8] - PS12*P[2][8] - PS13*P[3][8] + PS6*P[8][10] + PS69*dt + PS7*P[8][11] + PS9*P[8][12] + P[0][8];
nextP[1][8] = PS100*dt + PS11*P[0][8] - PS12*P[3][8] + PS13*P[2][8] - PS34*P[8][10] - PS7*P[8][12] + PS9*P[8][11] + P[1][8];
nextP[2][8] = PS11*P[3][8] + PS12*P[0][8] + PS121*dt - PS13*P[1][8] - PS34*P[8][11] + PS6*P[8][12] - PS9*P[8][10] + P[2][8];
nextP[3][8] = -PS11*P[2][8] + PS12*P[1][8] + PS13*P[0][8] + PS137*dt - PS34*P[8][12] - PS6*P[8][11] + PS7*P[8][10] + P[3][8];
nextP[4][8] = -PS139*P[8][15] + PS140*P[8][14] + PS164*dt - PS44*P[8][13] + PS60*P[2][8] + PS62*P[1][8] + PS72*P[0][8] - PS74*P[3][8] + P[4][8];
nextP[5][8] = PS160*P[8][15] - PS162*P[8][13] + PS181*dt - PS60*P[1][8] + PS62*P[2][8] - PS65*P[8][14] + PS72*P[3][8] + PS74*P[0][8] + P[5][8];
nextP[6][8] = -PS165*P[8][14] + PS166*P[8][13] + PS60*P[0][8] + PS62*P[3][8] - PS70*P[8][15] - PS72*P[2][8] + PS74*P[1][8] + P[6][8] + dt*(-PS165*P[5][14] + PS166*P[5][13] + PS60*P[0][5] + PS62*P[3][5] - PS70*P[5][15] - PS72*P[2][5] + PS74*P[1][5] + P[5][6]);
nextP[7][8] = P[4][8]*dt + P[7][8] + dt*(P[4][5]*dt + P[5][7]);
nextP[8][8] = P[5][8]*dt + P[8][8] + dt*(P[5][5]*dt + P[5][8]);
nextP[0][9] = -PS11*P[1][9] - PS12*P[2][9] - PS13*P[3][9] + PS6*P[9][10] + PS7*P[9][11] + PS75*dt + PS9*P[9][12] + P[0][9];
nextP[1][9] = PS101*dt + PS11*P[0][9] - PS12*P[3][9] + PS13*P[2][9] - PS34*P[9][10] - PS7*P[9][12] + PS9*P[9][11] + P[1][9];
nextP[2][9] = PS11*P[3][9] + PS12*P[0][9] + PS122*dt - PS13*P[1][9] - PS34*P[9][11] + PS6*P[9][12] - PS9*P[9][10] + P[2][9];
nextP[3][9] = -PS11*P[2][9] + PS12*P[1][9] + PS13*P[0][9] + PS138*dt - PS34*P[9][12] - PS6*P[9][11] + PS7*P[9][10] + P[3][9];
nextP[4][9] = -PS139*P[9][15] + PS140*P[9][14] + PS168*dt - PS44*P[9][13] + PS60*P[2][9] + PS62*P[1][9] + PS72*P[0][9] - PS74*P[3][9] + P[4][9];
nextP[5][9] = PS160*P[9][15] - PS162*P[9][13] + PS182*dt - PS60*P[1][9] + PS62*P[2][9] - PS65*P[9][14] + PS72*P[3][9] + PS74*P[0][9] + P[5][9];
nextP[6][9] = -PS165*P[9][14] + PS166*P[9][13] + PS186*dt + PS60*P[0][9] + PS62*P[3][9] - PS70*P[9][15] - PS72*P[2][9] + PS74*P[1][9] + P[6][9];
nextP[7][9] = P[4][9]*dt + P[7][9] + dt*(P[4][6]*dt + P[6][7]);
nextP[8][9] = P[5][9]*dt + P[8][9] + dt*(P[5][6]*dt + P[6][8]);
nextP[9][9] = P[6][9]*dt + P[9][9] + dt*(P[6][6]*dt + P[6][9]);
nextP[0][10] = PS14;
nextP[1][10] = PS85;
nextP[2][10] = PS109;
nextP[3][10] = PS123;
nextP[4][10] = -PS139*P[10][15] + PS140*P[10][14] - PS44*P[10][13] + PS60*P[2][10] + PS62*P[1][10] + PS72*P[0][10] - PS74*P[3][10] + P[4][10];
nextP[5][10] = PS160*P[10][15] - PS162*P[10][13] - PS60*P[1][10] + PS62*P[2][10] - PS65*P[10][14] + PS72*P[3][10] + PS74*P[0][10] + P[5][10];
nextP[6][10] = -PS165*P[10][14] + PS166*P[10][13] + PS60*P[0][10] + PS62*P[3][10] - PS70*P[10][15] - PS72*P[2][10] + PS74*P[1][10] + P[6][10];
nextP[7][10] = P[4][10]*dt + P[7][10];
nextP[8][10] = P[5][10]*dt + P[8][10];
nextP[9][10] = P[6][10]*dt + P[9][10];
nextP[10][10] = P[10][10];
nextP[0][11] = PS17;
nextP[1][11] = PS77;
nextP[2][11] = PS108;
nextP[3][11] = PS127;
nextP[4][11] = -PS139*P[11][15] + PS140*P[11][14] - PS44*P[11][13] + PS60*P[2][11] + PS62*P[1][11] + PS72*P[0][11] - PS74*P[3][11] + P[4][11];
nextP[5][11] = PS160*P[11][15] - PS162*P[11][13] - PS60*P[1][11] + PS62*P[2][11] - PS65*P[11][14] + PS72*P[3][11] + PS74*P[0][11] + P[5][11];
nextP[6][11] = -PS165*P[11][14] + PS166*P[11][13] + PS60*P[0][11] + PS62*P[3][11] - PS70*P[11][15] - PS72*P[2][11] + PS74*P[1][11] + P[6][11];
nextP[7][11] = P[4][11]*dt + P[7][11];
nextP[8][11] = P[5][11]*dt + P[8][11];
nextP[9][11] = P[6][11]*dt + P[9][11];
nextP[10][11] = P[10][11];
nextP[11][11] = P[11][11];
nextP[0][12] = PS20;
nextP[1][12] = PS87;
nextP[2][12] = PS103;
nextP[3][12] = PS126;
nextP[4][12] = -PS139*P[12][15] + PS140*P[12][14] - PS44*P[12][13] + PS60*P[2][12] + PS62*P[1][12] + PS72*P[0][12] - PS74*P[3][12] + P[4][12];
nextP[5][12] = PS160*P[12][15] - PS162*P[12][13] - PS60*P[1][12] + PS62*P[2][12] - PS65*P[12][14] + PS72*P[3][12] + PS74*P[0][12] + P[5][12];
nextP[6][12] = -PS165*P[12][14] + PS166*P[12][13] + PS60*P[0][12] + PS62*P[3][12] - PS70*P[12][15] - PS72*P[2][12] + PS74*P[1][12] + P[6][12];
nextP[7][12] = P[4][12]*dt + P[7][12];
nextP[8][12] = P[5][12]*dt + P[8][12];
nextP[9][12] = P[6][12]*dt + P[9][12];
nextP[10][12] = P[10][12];
nextP[11][12] = P[11][12];
nextP[12][12] = P[12][12];
nextP[0][13] = PS45;
nextP[1][13] = PS93;
nextP[2][13] = PS114;
nextP[3][13] = PS130;
nextP[4][13] = PS141;
nextP[5][13] = PS170;
nextP[6][13] = PS185;
nextP[7][13] = P[4][13]*dt + P[7][13];
nextP[8][13] = P[5][13]*dt + P[8][13];
nextP[9][13] = P[6][13]*dt + P[9][13];
nextP[10][13] = P[10][13];
nextP[11][13] = P[11][13];
nextP[12][13] = P[12][13];
nextP[13][13] = P[13][13];
nextP[0][14] = PS55;
nextP[1][14] = PS96;
nextP[2][14] = PS117;
nextP[3][14] = PS133;
nextP[4][14] = PS146;
nextP[5][14] = PS169;
nextP[6][14] = PS184;
nextP[7][14] = P[4][14]*dt + P[7][14];
nextP[8][14] = P[5][14]*dt + P[8][14];
nextP[9][14] = P[6][14]*dt + P[9][14];
nextP[10][14] = P[10][14];
nextP[11][14] = P[11][14];
nextP[12][14] = P[12][14];
nextP[13][14] = P[13][14];
nextP[14][14] = P[14][14];
nextP[0][15] = PS47;
nextP[1][15] = PS94;
nextP[2][15] = PS115;
nextP[3][15] = PS131;
nextP[4][15] = PS142;
nextP[5][15] = PS173;
nextP[6][15] = PS183;
nextP[7][15] = P[4][15]*dt + P[7][15];
nextP[8][15] = P[5][15]*dt + P[8][15];
nextP[9][15] = P[6][15]*dt + P[9][15];
nextP[10][15] = P[10][15];
nextP[11][15] = P[11][15];
nextP[12][15] = P[12][15];
nextP[13][15] = P[13][15];
nextP[14][15] = P[14][15];
nextP[15][15] = P[15][15];
nextP[0][16] = -PS11*P[1][16] - PS12*P[2][16] - PS13*P[3][16] + PS6*P[10][16] + PS7*P[11][16] + PS9*P[12][16] + P[0][16];
nextP[1][16] = PS11*P[0][16] - PS12*P[3][16] + PS13*P[2][16] - PS34*P[10][16] - PS7*P[12][16] + PS9*P[11][16] + P[1][16];
nextP[2][16] = PS11*P[3][16] + PS12*P[0][16] - PS13*P[1][16] - PS34*P[11][16] + PS6*P[12][16] - PS9*P[10][16] + P[2][16];
nextP[3][16] = -PS11*P[2][16] + PS12*P[1][16] + PS13*P[0][16] - PS34*P[12][16] - PS6*P[11][16] + PS7*P[10][16] + P[3][16];
nextP[4][16] = -PS139*P[15][16] + PS140*P[14][16] - PS44*P[13][16] + PS60*P[2][16] + PS62*P[1][16] + PS72*P[0][16] - PS74*P[3][16] + P[4][16];
nextP[5][16] = PS160*P[15][16] - PS162*P[13][16] - PS60*P[1][16] + PS62*P[2][16] - PS65*P[14][16] + PS72*P[3][16] + PS74*P[0][16] + P[5][16];
nextP[6][16] = -PS165*P[14][16] + PS166*P[13][16] + PS60*P[0][16] + PS62*P[3][16] - PS70*P[15][16] - PS72*P[2][16] + PS74*P[1][16] + P[6][16];
nextP[7][16] = P[4][16]*dt + P[7][16];
nextP[8][16] = P[5][16]*dt + P[8][16];
nextP[9][16] = P[6][16]*dt + P[9][16];
nextP[10][16] = P[10][16];
nextP[11][16] = P[11][16];
nextP[12][16] = P[12][16];
nextP[13][16] = P[13][16];
nextP[14][16] = P[14][16];
nextP[15][16] = P[15][16];
nextP[16][16] = P[16][16];
nextP[0][17] = -PS11*P[1][17] - PS12*P[2][17] - PS13*P[3][17] + PS6*P[10][17] + PS7*P[11][17] + PS9*P[12][17] + P[0][17];
nextP[1][17] = PS11*P[0][17] - PS12*P[3][17] + PS13*P[2][17] - PS34*P[10][17] - PS7*P[12][17] + PS9*P[11][17] + P[1][17];
nextP[2][17] = PS11*P[3][17] + PS12*P[0][17] - PS13*P[1][17] - PS34*P[11][17] + PS6*P[12][17] - PS9*P[10][17] + P[2][17];
nextP[3][17] = -PS11*P[2][17] + PS12*P[1][17] + PS13*P[0][17] - PS34*P[12][17] - PS6*P[11][17] + PS7*P[10][17] + P[3][17];
nextP[4][17] = -PS139*P[15][17] + PS140*P[14][17] - PS44*P[13][17] + PS60*P[2][17] + PS62*P[1][17] + PS72*P[0][17] - PS74*P[3][17] + P[4][17];
nextP[5][17] = PS160*P[15][17] - PS162*P[13][17] - PS60*P[1][17] + PS62*P[2][17] - PS65*P[14][17] + PS72*P[3][17] + PS74*P[0][17] + P[5][17];
nextP[6][17] = -PS165*P[14][17] + PS166*P[13][17] + PS60*P[0][17] + PS62*P[3][17] - PS70*P[15][17] - PS72*P[2][17] + PS74*P[1][17] + P[6][17];
nextP[7][17] = P[4][17]*dt + P[7][17];
nextP[8][17] = P[5][17]*dt + P[8][17];
nextP[9][17] = P[6][17]*dt + P[9][17];
nextP[10][17] = P[10][17];
nextP[11][17] = P[11][17];
nextP[12][17] = P[12][17];
nextP[13][17] = P[13][17];
nextP[14][17] = P[14][17];
nextP[15][17] = P[15][17];
nextP[16][17] = P[16][17];
nextP[17][17] = P[17][17];
nextP[0][18] = -PS11*P[1][18] - PS12*P[2][18] - PS13*P[3][18] + PS6*P[10][18] + PS7*P[11][18] + PS9*P[12][18] + P[0][18];
nextP[1][18] = PS11*P[0][18] - PS12*P[3][18] + PS13*P[2][18] - PS34*P[10][18] - PS7*P[12][18] + PS9*P[11][18] + P[1][18];
nextP[2][18] = PS11*P[3][18] + PS12*P[0][18] - PS13*P[1][18] - PS34*P[11][18] + PS6*P[12][18] - PS9*P[10][18] + P[2][18];
nextP[3][18] = -PS11*P[2][18] + PS12*P[1][18] + PS13*P[0][18] - PS34*P[12][18] - PS6*P[11][18] + PS7*P[10][18] + P[3][18];
nextP[4][18] = -PS139*P[15][18] + PS140*P[14][18] - PS44*P[13][18] + PS60*P[2][18] + PS62*P[1][18] + PS72*P[0][18] - PS74*P[3][18] + P[4][18];
nextP[5][18] = PS160*P[15][18] - PS162*P[13][18] - PS60*P[1][18] + PS62*P[2][18] - PS65*P[14][18] + PS72*P[3][18] + PS74*P[0][18] + P[5][18];
nextP[6][18] = -PS165*P[14][18] + PS166*P[13][18] + PS60*P[0][18] + PS62*P[3][18] - PS70*P[15][18] - PS72*P[2][18] + PS74*P[1][18] + P[6][18];
nextP[7][18] = P[4][18]*dt + P[7][18];
nextP[8][18] = P[5][18]*dt + P[8][18];
nextP[9][18] = P[6][18]*dt + P[9][18];
nextP[10][18] = P[10][18];
nextP[11][18] = P[11][18];
nextP[12][18] = P[12][18];
nextP[13][18] = P[13][18];
nextP[14][18] = P[14][18];
nextP[15][18] = P[15][18];
nextP[16][18] = P[16][18];
nextP[17][18] = P[17][18];
nextP[18][18] = P[18][18];
nextP[0][19] = -PS11*P[1][19] - PS12*P[2][19] - PS13*P[3][19] + PS6*P[10][19] + PS7*P[11][19] + PS9*P[12][19] + P[0][19];
nextP[1][19] = PS11*P[0][19] - PS12*P[3][19] + PS13*P[2][19] - PS34*P[10][19] - PS7*P[12][19] + PS9*P[11][19] + P[1][19];
nextP[2][19] = PS11*P[3][19] + PS12*P[0][19] - PS13*P[1][19] - PS34*P[11][19] + PS6*P[12][19] - PS9*P[10][19] + P[2][19];
nextP[3][19] = -PS11*P[2][19] + PS12*P[1][19] + PS13*P[0][19] - PS34*P[12][19] - PS6*P[11][19] + PS7*P[10][19] + P[3][19];
nextP[4][19] = -PS139*P[15][19] + PS140*P[14][19] - PS44*P[13][19] + PS60*P[2][19] + PS62*P[1][19] + PS72*P[0][19] - PS74*P[3][19] + P[4][19];
nextP[5][19] = PS160*P[15][19] - PS162*P[13][19] - PS60*P[1][19] + PS62*P[2][19] - PS65*P[14][19] + PS72*P[3][19] + PS74*P[0][19] + P[5][19];
nextP[6][19] = -PS165*P[14][19] + PS166*P[13][19] + PS60*P[0][19] + PS62*P[3][19] - PS70*P[15][19] - PS72*P[2][19] + PS74*P[1][19] + P[6][19];
nextP[7][19] = P[4][19]*dt + P[7][19];
nextP[8][19] = P[5][19]*dt + P[8][19];
nextP[9][19] = P[6][19]*dt + P[9][19];
nextP[10][19] = P[10][19];
nextP[11][19] = P[11][19];
nextP[12][19] = P[12][19];
nextP[13][19] = P[13][19];
nextP[14][19] = P[14][19];
nextP[15][19] = P[15][19];
nextP[16][19] = P[16][19];
nextP[17][19] = P[17][19];
nextP[18][19] = P[18][19];
nextP[19][19] = P[19][19];
nextP[0][20] = -PS11*P[1][20] - PS12*P[2][20] - PS13*P[3][20] + PS6*P[10][20] + PS7*P[11][20] + PS9*P[12][20] + P[0][20];
nextP[1][20] = PS11*P[0][20] - PS12*P[3][20] + PS13*P[2][20] - PS34*P[10][20] - PS7*P[12][20] + PS9*P[11][20] + P[1][20];
nextP[2][20] = PS11*P[3][20] + PS12*P[0][20] - PS13*P[1][20] - PS34*P[11][20] + PS6*P[12][20] - PS9*P[10][20] + P[2][20];
nextP[3][20] = -PS11*P[2][20] + PS12*P[1][20] + PS13*P[0][20] - PS34*P[12][20] - PS6*P[11][20] + PS7*P[10][20] + P[3][20];
nextP[4][20] = -PS139*P[15][20] + PS140*P[14][20] - PS44*P[13][20] + PS60*P[2][20] + PS62*P[1][20] + PS72*P[0][20] - PS74*P[3][20] + P[4][20];
nextP[5][20] = PS160*P[15][20] - PS162*P[13][20] - PS60*P[1][20] + PS62*P[2][20] - PS65*P[14][20] + PS72*P[3][20] + PS74*P[0][20] + P[5][20];
nextP[6][20] = -PS165*P[14][20] + PS166*P[13][20] + PS60*P[0][20] + PS62*P[3][20] - PS70*P[15][20] - PS72*P[2][20] + PS74*P[1][20] + P[6][20];
nextP[7][20] = P[4][20]*dt + P[7][20];
nextP[8][20] = P[5][20]*dt + P[8][20];
nextP[9][20] = P[6][20]*dt + P[9][20];
nextP[10][20] = P[10][20];
nextP[11][20] = P[11][20];
nextP[12][20] = P[12][20];
nextP[13][20] = P[13][20];
nextP[14][20] = P[14][20];
nextP[15][20] = P[15][20];
nextP[16][20] = P[16][20];
nextP[17][20] = P[17][20];
nextP[18][20] = P[18][20];
nextP[19][20] = P[19][20];
nextP[20][20] = P[20][20];
nextP[0][21] = -PS11*P[1][21] - PS12*P[2][21] - PS13*P[3][21] + PS6*P[10][21] + PS7*P[11][21] + PS9*P[12][21] + P[0][21];
nextP[1][21] = PS11*P[0][21] - PS12*P[3][21] + PS13*P[2][21] - PS34*P[10][21] - PS7*P[12][21] + PS9*P[11][21] + P[1][21];
nextP[2][21] = PS11*P[3][21] + PS12*P[0][21] - PS13*P[1][21] - PS34*P[11][21] + PS6*P[12][21] - PS9*P[10][21] + P[2][21];
nextP[3][21] = -PS11*P[2][21] + PS12*P[1][21] + PS13*P[0][21] - PS34*P[12][21] - PS6*P[11][21] + PS7*P[10][21] + P[3][21];
nextP[4][21] = -PS139*P[15][21] + PS140*P[14][21] - PS44*P[13][21] + PS60*P[2][21] + PS62*P[1][21] + PS72*P[0][21] - PS74*P[3][21] + P[4][21];
nextP[5][21] = PS160*P[15][21] - PS162*P[13][21] - PS60*P[1][21] + PS62*P[2][21] - PS65*P[14][21] + PS72*P[3][21] + PS74*P[0][21] + P[5][21];
nextP[6][21] = -PS165*P[14][21] + PS166*P[13][21] + PS60*P[0][21] + PS62*P[3][21] - PS70*P[15][21] - PS72*P[2][21] + PS74*P[1][21] + P[6][21];
nextP[7][21] = P[4][21]*dt + P[7][21];
nextP[8][21] = P[5][21]*dt + P[8][21];
nextP[9][21] = P[6][21]*dt + P[9][21];
nextP[10][21] = P[10][21];
nextP[11][21] = P[11][21];
nextP[12][21] = P[12][21];
nextP[13][21] = P[13][21];
nextP[14][21] = P[14][21];
nextP[15][21] = P[15][21];
nextP[16][21] = P[16][21];
nextP[17][21] = P[17][21];
nextP[18][21] = P[18][21];
nextP[19][21] = P[19][21];
nextP[20][21] = P[20][21];
nextP[21][21] = P[21][21];
nextP[0][22] = -PS11*P[1][22] - PS12*P[2][22] - PS13*P[3][22] + PS6*P[10][22] + PS7*P[11][22] + PS9*P[12][22] + P[0][22];
nextP[1][22] = PS11*P[0][22] - PS12*P[3][22] + PS13*P[2][22] - PS34*P[10][22] - PS7*P[12][22] + PS9*P[11][22] + P[1][22];
nextP[2][22] = PS11*P[3][22] + PS12*P[0][22] - PS13*P[1][22] - PS34*P[11][22] + PS6*P[12][22] - PS9*P[10][22] + P[2][22];
nextP[3][22] = -PS11*P[2][22] + PS12*P[1][22] + PS13*P[0][22] - PS34*P[12][22] - PS6*P[11][22] + PS7*P[10][22] + P[3][22];
nextP[4][22] = -PS139*P[15][22] + PS140*P[14][22] - PS44*P[13][22] + PS60*P[2][22] + PS62*P[1][22] + PS72*P[0][22] - PS74*P[3][22] + P[4][22];
nextP[5][22] = PS160*P[15][22] - PS162*P[13][22] - PS60*P[1][22] + PS62*P[2][22] - PS65*P[14][22] + PS72*P[3][22] + PS74*P[0][22] + P[5][22];
nextP[6][22] = -PS165*P[14][22] + PS166*P[13][22] + PS60*P[0][22] + PS62*P[3][22] - PS70*P[15][22] - PS72*P[2][22] + PS74*P[1][22] + P[6][22];
nextP[7][22] = P[4][22]*dt + P[7][22];
nextP[8][22] = P[5][22]*dt + P[8][22];
nextP[9][22] = P[6][22]*dt + P[9][22];
nextP[10][22] = P[10][22];
nextP[11][22] = P[11][22];
nextP[12][22] = P[12][22];
nextP[13][22] = P[13][22];
nextP[14][22] = P[14][22];
nextP[15][22] = P[15][22];
nextP[16][22] = P[16][22];
nextP[17][22] = P[17][22];
nextP[18][22] = P[18][22];
nextP[19][22] = P[19][22];
nextP[20][22] = P[20][22];
nextP[21][22] = P[21][22];
nextP[22][22] = P[22][22];
nextP[0][23] = -PS11*P[1][23] - PS12*P[2][23] - PS13*P[3][23] + PS6*P[10][23] + PS7*P[11][23] + PS9*P[12][23] + P[0][23];
nextP[1][23] = PS11*P[0][23] - PS12*P[3][23] + PS13*P[2][23] - PS34*P[10][23] - PS7*P[12][23] + PS9*P[11][23] + P[1][23];
nextP[2][23] = PS11*P[3][23] + PS12*P[0][23] - PS13*P[1][23] - PS34*P[11][23] + PS6*P[12][23] - PS9*P[10][23] + P[2][23];
nextP[3][23] = -PS11*P[2][23] + PS12*P[1][23] + PS13*P[0][23] - PS34*P[12][23] - PS6*P[11][23] + PS7*P[10][23] + P[3][23];
nextP[4][23] = -PS139*P[15][23] + PS140*P[14][23] - PS44*P[13][23] + PS60*P[2][23] + PS62*P[1][23] + PS72*P[0][23] - PS74*P[3][23] + P[4][23];
nextP[5][23] = PS160*P[15][23] - PS162*P[13][23] - PS60*P[1][23] + PS62*P[2][23] - PS65*P[14][23] + PS72*P[3][23] + PS74*P[0][23] + P[5][23];
nextP[6][23] = -PS165*P[14][23] + PS166*P[13][23] + PS60*P[0][23] + PS62*P[3][23] - PS70*P[15][23] - PS72*P[2][23] + PS74*P[1][23] + P[6][23];
nextP[7][23] = P[4][23]*dt + P[7][23];
nextP[8][23] = P[5][23]*dt + P[8][23];
nextP[9][23] = P[6][23]*dt + P[9][23];
nextP[10][23] = P[10][23];
nextP[11][23] = P[11][23];
nextP[12][23] = P[12][23];
nextP[13][23] = P[13][23];
nextP[14][23] = P[14][23];
nextP[15][23] = P[15][23];
nextP[16][23] = P[16][23];
nextP[17][23] = P[17][23];
nextP[18][23] = P[18][23];
nextP[19][23] = P[19][23];
nextP[20][23] = P[20][23];
nextP[21][23] = P[21][23];
nextP[22][23] = P[22][23];
nextP[23][23] = P[23][23];


