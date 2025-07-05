% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
% clc
% clear
% close all

%%  Excel Sheets Data
% filename = 'Boeing_FC5'; %% put here the location of your excel sheet
filename = 'Lockheed_Jetstar_FC9'; %% put here the location of your excel sheet
% filename = 'Jetstar_FC10'; %% put here the location of your excel sheet

aircraft_data=xlsread(filename,'B2:B61'); %#ok here B2:B61 means read the excel sheet from cell B2 to cell B61

% initial conditions
s0 = aircraft_data(4:15);
sdot0 = zeros(12,1);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);

U0 = s0(1);
V0 = s0(2);
W0 = s0(3);
P0 = s0(4);
Q0 = s0(5);
R0 = s0(6);
PHI0 = s0(7);
THETA0 = s0(8);
PSI0 = s0(9);
X0 = 0;
Y0 = 0;
Z0 = -500; % this is to have a good visualization in flightgear
ALPHA0 = THETA0;

% control actions values
% da = aircraft_data(57);
% dr = aircraft_data(58);
% de = aircraft_data(59);
% dth = aircraft_data(60);
% dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];

% gravity, mass & inertia
m = aircraft_data(51);
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);    Ixy=0;  Iyz=0;
I = [Ixx , -Ixy , -Ixz ;
    -Ixy , Iyy , -Iyz ;
    -Ixz , -Iyz , Izz];
invI=inv(I);

% stability derivatives Longitudinal motion
SD_Long = aircraft_data(21:36);
SD_Long_final = SD_Long;

% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);
G=1/(1-Ixz^2/(Ixx*Izz));
dash_mat=[G, G*Ixz/Ixx; G*Ixz/Izz, G];
dash_mat_inv=inv(dash_mat);

LB_DASH = SD_Lat_dash(3);
NB_DASH = SD_Lat_dash(4);
LP_DASH = SD_Lat_dash(5);
NP_DASH = SD_Lat_dash(6);
LR_DASH = SD_Lat_dash(7);
NR_DASH = SD_Lat_dash(8);
LDA_DASH = SD_Lat_dash(11);
NDA_DASH = SD_Lat_dash(12);
LDR_DASH = SD_Lat_dash(13);
NDR_DASH = SD_Lat_dash(14);

B=dash_mat\[LB_DASH; NB_DASH];
P=dash_mat\[LP_DASH; NP_DASH];
R=dash_mat\[LR_DASH; NR_DASH];
DA=dash_mat\[LDA_DASH; NDA_DASH];
DR=dash_mat\[LDR_DASH;NDR_DASH];

YDA=SD_Lat_dash(9)*Vto;
YDR=SD_Lat_dash(10)*Vto;
SD_Lat=[SD_Lat_dash(1);SD_Lat_dash(2);B(1);B(2);P(1);P(2);R(1);R(2);YDA;YDR;DA(1);DA(2);DR(1);DR(2)];
%           YV              YB         LB   NB   LP   NP   LR   NR   Y_DA Y_DR  LDA  NDA   LDR   NDR
SD_Lat_final = [ SD_Lat(1) ; SD_Lat(3) ; SD_Lat(4) ; SD_Lat(5) ; SD_Lat(6) ;...
                 SD_Lat(7) ; SD_Lat(8) ; SD_Lat(9:10) ; SD_Lat(11) ; SD_Lat(12) ;...
                  SD_Lat(13) ; SD_Lat(14) ];

% initial gravity force
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];

XU = SD_Long_final(1);
ZU = SD_Long_final(2);
MU = SD_Long_final(3);
XW = SD_Long_final(4);
ZW = SD_Long_final(5);
MW = SD_Long_final(6);
ZWD = SD_Long_final(7);
ZQ = SD_Long_final(8);
MWD = SD_Long_final(9);
MQ = SD_Long_final(10);
XDE = SD_Long_final(11);
ZDE = SD_Long_final(12);
MDE = SD_Long_final(13);
XDTH = SD_Long_final(14);
ZDTH = SD_Long_final(15);
MDTH = SD_Long_final(16);

YV = SD_Lat_final(1);
LB = SD_Lat_final(2);
NB = SD_Lat_final(3);
LP = SD_Lat_final(4);
NP = SD_Lat_final(5);
LR = SD_Lat_final(6);
NR = SD_Lat_final(7);
YDA = SD_Lat_final(8);
YDR = SD_Lat_final(9);
LDA = SD_Lat_final(10);
NDA = SD_Lat_final(11);
LDR = SD_Lat_final(12);
NDR = SD_Lat_final(13);

LV_DASH = LB_DASH / Vto;
NV_DASH = NB_DASH / Vto;
LV = LB / Vto;
NV = NB / Vto;

YP = 0;
YR = 0;

States_Matrix = [XU,   0,   XW,    0,   0,   0,    0;
                  0,   YV,    0,   YP,  0,  YR,    0;
                 ZU,   0,   ZW,    0,  ZQ,   0,   ZWD;
                  0,   LV,    0,   LP,  0,  LR,    0;
                 MU,   0,   MW,    0,  MQ,   0,   MWD;
                  0,   NV,   0,    NP,  0,  NR,    0];

Controls_Matrix = [0,    XDE,   XDTH,      0;
                   YDA,  0,      0,      YDR;
                   0,    ZDE,   ZDTH,      0;
                   LDA,  0,      0       LDR;
                   0,    MDE,    MDTH,     0;
                   NDA,  0,      0,       NDR];


rho = 5.87e-4; % (slugs/ft^3)
CD_alpha = 0.7;
CD = CD_alpha * ALPHA0;
Qinf = 117; % (PSF)
S = 542.5; % (FT^2)
Thrust0 = 0.5*rho*Vto^2*S*CD; % (LBF)
Thrustmax = 14800; % (LBF)
