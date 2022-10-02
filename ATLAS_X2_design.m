clear;
close all;

AR_range = 5:0.25:10; 
b_range = 0.6:0.05:3;
R_range = 0.12:0.01:0.5;
N_blades_range = 2:1:4;

config=zeros(length(AR_range),length(b_range),length(R_range),length(N_blades_range),15);

for i=1:length(AR_range)
    config(i,:,:,:,1)=AR_range(i);
end

for i=1:length(b_range)
    config(:,i,:,:,2)=b_range(i);
end

for i=1:length(R_range)
    config(:,:,i,:,3)=R_range(i);
end

for i=1:length(N_blades_range)
    config(:,:,:,i,4)=N_blades_range(i);
end

for i=1:length(AR_range)
    for j=1:length(b_range)
        for k=1:length(R_range)
            for l=1:length(N_blades_range)
        config(i,j,k,l,5) = ((config(1,j,1,1,2))^2)/config(i,1,1,1,1);
        kk = k_factor(config(i,1,1,1,1));
        config(i,:,:,:,6) = kk;
        CD0 = CD0_estimate(config(i,1,1,1,1),config(1,j,1,1,2));
        config(i,j,k,l,7) = CD0;
        rho = 1.219; %density at 55m
        W = weight_estimate(config(i,1,1,1,1),config(1,j,1,1,2));
        config(i,j,k,l,8) = W;
        WS = config(i,j,k,l,8)/config(i,j,k,l,5);
        config(i,j,k,l,9) = WS;
        A_rotor = 4*pi*config(1,1,k,1,3)^2;
        config(i,j,k,l,10) = A_rotor;
        DL = config(i,j,k,l,8)/config(i,j,k,l,10);
        config(i,j,k,l,11) = DL;
        
        % Cruzeiro a 25 m/s
        
        eta_p_cruise = 1; 
        PW_CRUISE = 1/eta_p_cruise * (  (rho * 25^3 * CD0 / 2 / (WS) ) + ( 2 * kk * WS / rho / 25 ) );
        
        % Curva a 2,5G
        
        V_turn = Speed(kk,WS,rho,CD0);
        V = V_turn; %Velocity during the turn
        n = 2.5; %load factor
        eta_p_turn = 0.8; %nota: se a eficência fôr 1 e raio grande, esta curva coincide com a de cruise (mas a eficência será sempre um pouco menor a curvar)
        q = 0.5*rho*V^2; %dynamic pressure
        PW_TURN = 1 / eta_p_turn * ( rho * V^3*CD0 / 2 / WS + 2 * kk *n^2 / rho / V * WS );
        
        % Subida vertical a 1 m/s
        
        rho2 = 1.222; %density média entre 0m e 50m
        Omega = 900;
        V_tip = Omega*config(1,1,k,1,3);
        Solidity = config(1,1,1,l,4)*0.025/pi/config(1,1,k,1,3);
        C_d_rotor = 0.02;
        Ind_power_factor = 1.15;
        
        PW_climb = 1 - Ind_power_factor .* 1 ./ 2 + Ind_power_factor .* sqrt(1.^2 + 2 .* DL ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        
        % Descida vertical a 2 m/s
        
        ind_hover_v = sqrt(W/2/rho2/A_rotor);
        
        if (-2*ind_hover_v <= -2)
            PW_descent = -2 - Ind_power_factor .* (-2) ./ 2 - Ind_power_factor .* sqrt((-2).^2 - 2 .* DL ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        else
            ind_desc_v = ind_hover_v*(Ind_power_factor -1.125*(-2)/ind_hover_v -1.372*(-2/ind_hover_v)^2 -1.718*(-2/ind_hover_v)^3 -0.655*(-2/ind_hover_v)^4);
            PW_descent = (-2 + Ind_power_factor*ind_desc_v) + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        end
        
        % Minimum power required
        
        PW_f = max(PW_CRUISE,PW_TURN);
        P_f = PW_f*W;
        PW_v = max(PW_climb,PW_descent);
        P_v = PW_v*W;
        config(i,j,k,l,12) = PW_f;
        config(i,j,k,l,13) = P_f;
        config(i,j,k,l,14) = PW_v;
        config(i,j,k,l,15) = P_v;
        
            end
        end
    end
end


function k = k_factor(AR)
% =========================================================================
% =========================================================================
% Description:
% Calculates Oswald Coefficient (e) and returns de value of k, used to
% calculate the drag coefficient induced by the lift force;
% =========================================================================
% Inputs:
% AR [scalar] - Aspect Ratio;
% sweep [scalar] - Sweep Angle;
% =========================================================================
% Outputs:
% k [scalar] - used to calculate the total drag coefficient using the
% formula CD = CD0 + k*CL^2;
% =========================================================================
% =========================================================================
%valid for small sweep angles, <10º
%conversion from degrees to radians
sweep_rad = 3*pi/180; % flecha de 3º
%Ver equação (9-91) - General Aviation
e = 2 / ( 2-AR+sqrt(4+AR^2*(1+sweep_rad^2)));
k = 1/(pi*e*AR);
end

function CD0 = CD0_estimate(AR,b)
% =========================================================================
% =========================================================================
% Description: Used to calculate an estimate of the zero-lift drag 
% coefficient 
% =========================================================================
% Inputs:
% S [scalar] - Wing Surface Area;
% AR [scalar] - Aspect Ratio;
% sweep [scalar] - Sweep Angle;
% m_payload [scalar] - Payload Weight;
% rodas [scalar] - type of wheels choosen (with different diameters);
% tail_tri [scalar] - number of wheels implemented in the tail;
% =========================================================================
% Outputs:
% CD0 [scalar] - estimate value of the zero-lift drag coefficient;
% =========================================================================
% =========================================================================
% Useful Links:
% http://adl.stanford.edu/sandbox/groups/aa241x/wiki/e054d/attachments/31ca0/performanceanddrag.pdf
% https://aerotoolbox.com/drag-polar/#more-1325
% https://basedados.aeroubi.pt/pluginfile.php/548/mod_resource/content/0/Chapter%203.%20Drag%20Force%20and%20its%20Coefficient.pdf

%% Escoamento
%kinematic viscosity
miu = 1.5*10^-5; 
%velocity (can vary between these values [10 12 15 17 19 21])
V = 25; 

%% Geometria
%Wing
%wingspan
bw = b;
%average wing chord
cw = bw/AR;
%area 
Sw = bw*cw;

%Tail
%horizontal stabilizer area
Sh = 1.3*cw^2;
%horizontal stabilizer average chord
ch = sqrt(Sh/4);
%vertical stabilizer average chord
cv = 0.2; %0.15
%vertical stabilizer area
Sv = 0.06;

%% Cd Asa
%Reynolds Number relative to the wing
Rew = cw*V/miu;
%turbulent flow 't', 'laminar flow 'l', 'm' 20% laminar
%choosing turbulent flow ensures more plausible results
flow = 't'; 
%the next expressions are derive from fluid mechanics, and relate the
%Reynolds number with the shear coefficient
if flow == 'm'
     Cf = 1.327/sqrt(Rew*0.2)+...
            (0.455/(log10(Rew))^2.58 - 0.455/(log10(Rew*0.2))^2.58);
elseif flow == 'l'
    Cf = 1.327/sqrt(Rew);
else
    Cf = 0.455/(log10(Rew))^2.58;
end
%RLS corresponds to a correction factor due to the interference between
%wing and fuselage
%1.063 for a sweep angle of 10º 
RLS = 1.065; %for a sweep angle of 3º
%wet aera - surface wing area emerged in the flow
S_wet = 2.04070732 * Sw; % valor-t/c, 2.04 - 10%, 2.048 - 12% 
%total drag coefficient in the wing
CDw = RLS*Cf*( 1+1.2*0.12+100*0.12^4 )*S_wet/Sw; %Aerotoolbox

%% Cd Estabilizador Vertical
%Reynolds Number relative to the vertical stabilizer
Retv = cv*V/miu;

flow = 't'; %turbulent flow 't', 'laminar flow 'l'

if flow == 't'
    Cf = 0.455/(log10(Retv))^2.58;
else
    Cf = 1.327/sqrt(Retv);
end

RLS = 1.065; 
S_wet = 2.04 * Sv; % valor-t/c, 2.04 - 10%, 2.048 - 12%
%total drag coefficient in the vertical stabilizer
CDtv = RLS*Cf*( 1+1.2*0.10+100*0.10^4 )*S_wet/Sw;

%% Cd Estabilizador Horizontal
%Reynolds Number relative to the horizontal stabilizer
Reth = ch*V/miu;

flow = 't'; %turbulent flow 't', 'laminar flow 'l'

if flow == 't'
    Cf = 0.455/(log10(Reth))^2.58;
else
    Cf = 1.327/sqrt(Reth);
end

RLS = 1.065; 
S_wet = 2.04 * Sh; % valor-t/c, 2.04 - 10%, 2.048 - 12%
%total drag coefficient in the horizontal stabilizer
CDth = RLS*Cf*( 1+1.2*0.10+100*0.10^4 )*S_wet/Sw;

%% Cd Struts
%number of struts
n_struts = 2;
S_struts = 0.2*0.010;
%total drag coefficient in the struts
CDst = n_struts*1*S_struts/Sw;

%% Cd Cargo Bay
%number of blood bags of with mass 300 g 
%nsacos = m_payload/0.3;
%L = 0.09; %bag width
%H = 0.02; %bag height
%number of layers of bags
%nEmpilhamento = 2;
%cargo bay diameter (assuming a cylindrical shape)
%diametroCB = sqrt(L^2+(H*nEmpilhamento)^2);
%V = 20; %estimate flow velocity
%length 
%CompTotal = 0.25+0.16*floor((1+nsacos)/2);
%Reynolds Number relative to the cargo bay
%Re = 1.1225*V*CompTotal/(1.78938*10^-5);
%shear coefficient in the cargo bay
%Cf=0.455/(log10(Re))^2.58;
%wet surface of the cargo bay
%Swet = pi()*diametroCB*CompTotal;
%fld = 1+60/(CompTotal/diametroCB)^3+0.0025*(CompTotal/diametroCB);
%total drag coefficient of the cargo bay
%CD_CB = Cf*fld*Swet/S;
%Old version of the model to calculate drag coefficient in the cargo bay
%CD_CB = (6.68*10^-4)*m_payload^2-(4.57*10^-4)*m_payload+(4.07*10^-3);

%% Cálculos finais

CDsup = CDw+CDtv+CDth;
CD_extra = CDst;
%minimum drag coefficient of the airplane 
CDtot = CDsup + CD_extra;% + CD_CB;
%correction factor
CD0 = 1.1*CDtot;

end

function W = weight_estimate(AR,b)
% =========================================================================
% =========================================================================
% Description: Used to calculate an estimate of the total weight of the
% airplane;
% =========================================================================
% Inputs:
% S [scalar] - Wing Surface Area;
% AR [scalar] - Aspect Ratio;
% sweep [scalar] - Sweep Angle;
% m_payload [scalar] - Payload Weight;
% =========================================================================
% Outputs:
% W [scalar] - estimate value of the total weight of the airplane;
% =========================================================================
% =========================================================================    
    % Wing
%Average wing chord
    cw = b/AR;
%Area 
    S = b*cw;
    
    % Tail 
%Distance between wing and tail
    lh = 0.5*b/1.3;
%Horizontal Stabilizer Area
    Sh = 1.3*cw^2;
    
%Fuselage length
    f = 1.25*lh;

    %The following expressions are obtained via an empirical approach and
    %based on results from previous airplanes
    %% Asa
    m_w = 0.33*(S^1.6*AR^0.9)^0.73;
    
    %% Cargo Bay + Fuselagem
    % carbon tube ~ 0.286kg/m
    
    m_fus = 0.8*(0.286*f);
    m_cb = 0.0467*2 + 0.6;  
    
    %% Cauda 
    m_t = 2*Sh;
    
    %% Eletrónica
    %estimated without taking into account the connectors
    m_ele = 1.893; 
        
    %% Total
    m_aero = m_fus + m_cb + m_t + m_ele + m_w;
    
    W = (m_aero+2)*9.81;
    
end

function V_Carson = Speed(k,WS,rho,CD0)
% =========================================================================
% =========================================================================
% Description: Used to calculate three different velocitys during flight
% =========================================================================
% Inputs:
% k [scalar] - k factor;
% W [scalar] - Weight;
% W_ [scalar] - DUARTE;
% S [scalar] - Wing surface area; 
% CD0 [scalar] - value of the zero-lift drag coefficient;
% T2,T1,T0 [scalars] - quadratic coefficients relative to the curve of 
% thrust as a funtion of velocity
% =========================================================================
% Outputs:
% V_Max [scalar] - when thrust and drag reach the same value, i.e. 
% cruise velocity;
% V_LD [scalar] - velocity that maximizes the value L/D (saves battery);
% V_Carson [scalar] - velocity used during a turn;
% =========================================================================
% =========================================================================

%Certain velocitys for simple drag model
V_LD = sqrt(2*WS*sqrt(k/CD0)/rho);
V_Carson = 1.32*V_LD;

end