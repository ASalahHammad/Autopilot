% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
% clc
% clear
% close all

%%  Excel Sheets Data
% filename = 'Boeing_FC5'; %% put here the location of your excel sheet
filename = 'Lockheed_Jetstar_FC9'; %% put here the location of your excel sheet

aircraft_data=xlsread(filename,'B2:B61'); %#ok here B2:B61 means read the excel sheet from cell B2 to cell B61

% initial conditions
s0 = aircraft_data(4:15);
sdot0 = zeros(12,1);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);

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
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy , -Iyz ;...
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

B=dash_mat\[SD_Lat_dash(3);SD_Lat_dash(4)];
P=dash_mat\[SD_Lat_dash(5);SD_Lat_dash(6)];
R=dash_mat\[SD_Lat_dash(7);SD_Lat_dash(8)];
DA=dash_mat\[SD_Lat_dash(11);SD_Lat_dash(12)];
DR=dash_mat\[SD_Lat_dash(13);SD_Lat_dash(14)];

YDA=SD_Lat_dash(9)*Vto;
YDR=SD_Lat_dash(10)*Vto;
SD_Lat=[SD_Lat_dash(1);SD_Lat_dash(2);B(1);B(2);P(1);P(2);R(1);R(2);YDA;YDR;DA(1);DA(2);DR(1);DR(2)];
%           YV              YB         LB   NB   LP   NP   LR   NR   Y_DA Y_DR  LDA  NDA   LDR   NDR
SD_Lat_final = [ SD_Lat(1) ; SD_Lat(3) ; SD_Lat(4) ; SD_Lat(5) ; SD_Lat(6) ;...
                 SD_Lat(7) ; SD_Lat(8) ; SD_Lat(9:10) ; SD_Lat(11) ; SD_Lat(12) ;...
                  SD_Lat(13) ; SD_Lat(14) ];

% initial gravity force
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];

Xu = SD_Long_final(1);
Zu = SD_Long_final(2);
Mu = SD_Long_final(3);
Xw = SD_Long_final(4);
Zw = SD_Long_final(5);
Mw = SD_Long_final(6);
Zwd = SD_Long_final(7);
Zq = SD_Long_final(8);
Mwd = SD_Long_final(9);
Mq = SD_Long_final(10);
XDE = SD_Long_final(11);
ZDE = SD_Long_final(12);
MDE = SD_Long_final(13);
XDTH = SD_Long_final(14);
ZDTH = SD_Long_final(15);
MDTH = SD_Long_final(16);

Yv = SD_Lat_final(1);
LB = SD_Lat_final(2);
NB = SD_Lat_final(3);
Lp = SD_Lat_final(4);
Np = SD_Lat_final(5);
Lr = SD_Lat_final(6);
Nr = SD_Lat_final(7);
YDA = SD_Lat_final(8);
YDR = SD_Lat_final(9);
LDA = SD_Lat_final(10);
NDA = SD_Lat_final(11);
LDR = SD_Lat_final(12);
NDR = SD_Lat_final(13);

Lv = LB / Vto;
Nv = NB / Vto;

Yp = 0;
Yr = 0;

States_Matrix = [Xu,   0,   Xw,    0,   0,   0,    0;
                  0,   Yv,    0,   Yp,  0,  Yr,    0;
                 Zu,   0,   Zw,    0,  Zq,   0,   Zwd;
                  0,   Lv,    0,   Lp,  0,  Lr,    0;
                 Mu,   0,   Mw,    0,  Mq,   0,   Mwd;
                  0,   Nv,   0,    Np,  0,  Nr,    0];

Controls_Matrix = [0,    XDE,   XDTH,      0;
                   YDA,  0,      0,      YDR;
                   0,    ZDE,   ZDTH,      0;
                   LDA,  0,      0       LDR;
                   0,    MDE,    MDTH,     0;
                   NDA,  0,      0,       NDR];
