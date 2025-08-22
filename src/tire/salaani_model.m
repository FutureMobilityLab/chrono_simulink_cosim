function [FY, FX, MZ, MX ] = Salaani_Model (Tire, ALPHA, S, GAMMA, FZ)
% ### SAE AXIS Figure 1. ###
% Input : Tire - "structure for tire parameters
% ALPhA - lateral slip angle in (rad)
% S - longitudinal slip [-1 to 1]
% GAMMA - inclination angle (rad)
% Fz > 0 -Tire normal force in (lbs) - must be positive
%
% Output: FX - Longitudinal Force (lbs)
% FY - Lateral force (lbs)
% MZ - Tire aligning torque (ft-lbs)
% MX - Tire overturning moment (ft-lbs)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Written by Dr. Mohamed Kamel Salaani for SAE Paper 2007-01-0816 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
S = min (abs (S) ,0.99) * sign (S); FZ2 = FZ * FZ
%
%%%%% Physical Parameters from Appendix B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% tire longitudinal stiffness
CSO = Tire.CSO; Eta = Tire.Eta; FzxO = Tire.FzxO;
% tire lateral stiffness
CA1 = Tire.CA1; CA2 = Tire.CA2; CAm = Tire.CAm; Fzym = Tire.CAFzm;
% inclination angle lateral force stiffness
GAMMA1 = Tire.GAMMA1; GAMMA 2 = Tire.GAMMA2 ;
% aligning moment pneumatic trail
Tz2 = Tire.tz2 ; Tzl = Tire.tzl;
% aligning moment constants
m1 = Tire.m1; mO = Tire.mO;
% sliding force eccentricity
Epsx = Tire.Epsx;
% overturning moment arm
Tx3 = Tire.Tx3; Tx2 = Tire.Tx2; Txl = Tire.Txl;
% road to test friction rate
MURAT IO = Tire . MUNOM / Tire . MUNTEST;
% longitudinal peak coefficient of friction
MuxO = Tire.MuxO;Muxl = Tire.Muxl; Mux2 = Tire.Mux2;
% Lateral peak coefficient of friction
MuyO = Tire.MuyO; Muyl = Tire.Muyl; Muy2 = Tire.Muy2;
% minimum load for valid peak friction values
FzmuO = Tire.FzO;
% longitudinal sliding coefficient of friction
DMUx3 = Tire.KMUx3; DMUx2 = Tire.KMUx2; DMUxl = Tire.KMUxl; Eps_sx = Tire.Lsx;
% lateral sliding coefficient of friction
DMU y 3 = Tire . KMUy3 ; DMU y 2 = Tire.KMUy2; DMUyl = Tire.KMUyl; Eps_sy = Tire.Lsy;
%
%%%%%% Empirical physical properties formulae %%%%%%%%%%%%%%%%%%%%%
%
% pneumatic trail (Equation 29)
Tz = Tzl*FZ2 + Tz2*FZ;
% overturning moment arm (Equation 30)
Tx = Txl *FZ2 + Tx2 *FZ + Tx3;
% lateral stiffness (Equation 26)
CA = CAm* (1-exp (CAI* (FZ/Fzym) . A2 + CA2* (FZ/Fzym) ) ) ;
% longitudinal stiffness (Equation 27)
CS = CSO* (FZ/Fzx0) . AEta;
% Inclination angle lateral force stiffness (Equation 31)
FYGAMMA = GAMMAI * FZ + GAMMA 2 * FZ 2 ;
% longitudinal peak coefficient of friction (Equation 22)
if FZ < FzmuO
FZ1 = FzmuO;
else
FZ1 = FZ;
end
MUXp= (MUNOM/MUTEST) *MuxO* (FZl/FzmuO) . ^ (Mux2+Muxl*log (FZl/FzmuO ) ) ;
% lateral peak coefficient of friction (Equation 22)
MUYp= (MUNOM/MUTEST) *MuyO* (FZl/FzmuO) . ^ (Muy2+Muyl*log (FZl/FzmuO ) ) ;
% longitudinal decay of friction (Equation 23)
DMUx = DMUxl*FZ2 + DMUx2*FZ + DMUx3;
% lateral decay of friction (Equation 23)
DMU y = DMUyl*FZ2 + DMUy2*FZ + DMU y 3 ;
% adjust for plysteer
ALPHA = ALPHA - Tire . PLYSTEER;
% sliding coefficient of friction (Equation 24)
Slip = min (1, sqrt ( (sin (ALPHA) ) A2+ (S*cos (ALPHA) ) A2) ) ;
MNU y = MUYp * (1 - DMU y * Slip) *Eps_sy;
MNUx = MUXp * (1 - DMUx * Slip) *Eps_sx;
% effect of slip on longitudinal stiffness
CSp = CS + (CA - CS) *Slip;
o.
"o
%%%%%%%%%%%%%%%%%%% Salaani ' s Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
o,
"o
% adhesion potential rate (Equation 11)
%
SIGMA = sqrt ( (CA*tan (ALPHA) / (1-S) /MUYp/FZ) A2 + (CS*S/ (1-S) /MUXp/FZ) A2) ;
o
"o
% adhesion and sliding functions (Equations 18 and 19)
O.
O
SIGMA2 = SIGMA* SIGMA;
SIGMAU = (1 - SIGMA2) / (1 + SIGMA2) ;
F_a = 4/pi*SIGMA/ (SIGMA2 + 1)A2;
F_s = 1/pi* (pi/2 - SIGMAU*sqrt (1-SIGMAUA2) - asin (SIGMAU) ) ;
o.
"o
% lateral and longitudinal forces (Equations 14 and 15)
%
SIGMAm = sqrt ( (CA * tan (ALPHA) /MUYp) A2 + (CS * S/MUXp) A2 );
SIGMASm = sqrt ( (CA * tan (ALPHA) /MNUy) A2 + (CSp * S/MNUx) A2 );
if (SIGMAm < 1.0e-6) then
FY = 0;
FX = 0;
else
FY = FZ * CA*tan (ALPHA) * (F_a/ SIGMAm + F_s/SIGMASm) ;
FX = - FZ * S * ( CS *F_a/ SIGMAm + CSp*F_s/SIGMASm) ;
end
o
o
% aligning torque (Equation 20)
o.
'S
MZ = Tz*CA*tan (ALPHA) / (ml*SIGMA2 + mO)A2 + FY*Epsx*F_s;
%
% overturning moment (Equation 21)
%
MX = -Tx*FY;
%
% Camber effect added to lateral force (Equation 32)
%
FY = FY + FYGAMMA * GAMMA * ( 1 - F_s ) ;