clear;
close all;

AR_range = 5:0.25:10; 
b_range = 1.5:0.05:2.85;
R_range = 0.1:0.01:0.5; %gama de raios de cada rotor
N_blades_range = 2:1:4; %gama de número de blades de cada rotor

config=zeros(length(AR_range),length(b_range),length(R_range),length(N_blades_range),16);

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
        config(i,j,k,l,5) = ((config(1,j,1,1,2))^2)/config(i,1,1,1,1); %área da asa
        kk = k_factor(config(i,1,1,1,1));
        config(i,:,:,:,6) = kk; %lift induced drag constant
        CD0 = CD0_estimate(config(i,1,1,1,1),config(1,j,1,1,2),config(1,1,k,1,3),25);
        config(i,j,k,l,7) = CD0;
        rho = 1.219; %density at 55m
        W = weight_estimate(config(i,1,1,1,1),config(1,j,1,1,2),config(1,1,k,1,3),config(1,1,1,l,4));
        config(i,j,k,l,8) = W;
        WS = config(i,j,k,l,8)/config(i,j,k,l,5);
        config(i,j,k,l,9) = WS; %wing loading
        A_rotor = 4*pi*config(1,1,k,1,3)^2; %área total dos 4 rotores
        config(i,j,k,l,10) = A_rotor;
        DL = config(i,j,k,l,8)/config(i,j,k,l,10);
        config(i,j,k,l,11) = DL; %disk loading
        
        % Cruzeiro a 25 m/s
        
        eta_p_cruise = 1; 
        PW_CRUISE = 1/eta_p_cruise * (  (rho * 25^3 * CD0 / 2 / (WS) ) + ( 2 * kk * WS / rho / 25 ) );
        
        % Curva a 2,5G
        
        V_turn = Speed(kk,WS,rho,CD0);
        V = V_turn; %Velocity during the turn
        n = 2.5; %load factor
        eta_p_turn = 0.8; %nota: se a eficiência for 1 e raio grande, esta curva coincide com a de cruise (mas a eficiência será sempre um pouco menor a curvar)
        q = 0.5*rho*V^2; %dynamic pressure
        PW_TURN = 1 / eta_p_turn * ( rho * V^3*CD0 / 2 / WS + 2 * kk *n^2 / rho / V * WS );
        
        % Subida vertical a 1 m/s
        
        rho2 = 1.222; %density média entre 0m e 50m
        Omega = 7165/60*(2*pi); %máxima rotação do motor VTOL considerado é igual a 7165 rpm
        V_tip = Omega*config(1,1,k,1,3); %velocidade na tip para cada rotor
        Solidity = config(1,1,1,l,4)*0.02/pi/config(1,1,k,1,3); %solidity de cada rotor, considerando a corda média de cada hélice sendo igual a 2cm (aproximação das hélices consideradas)
        C_d_rotor = 0.02; %aproximação utilizada de base drag coefficient de cada rotor
        Ind_power_factor = 1.15;
        
        PW_climb = 1 - Ind_power_factor .* 1 ./ 2 + Ind_power_factor .* sqrt(1.^2 + 2 .* DL ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        
        % Descida vertical a 2 m/s
        
        ind_hover_v = sqrt(W/2/rho2/A_rotor); %velocidade induzida em hover
        
        if (ind_hover_v <= 1)
            PW_descent = -2 - Ind_power_factor .* (-2) ./ 2 - Ind_power_factor .* sqrt((-2).^2 - 2 .* DL ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        else
            ind_desc_v = ind_hover_v*(Ind_power_factor -1.125*(-2)/ind_hover_v -1.372*(-2/ind_hover_v)^2 -1.718*(-2/ind_hover_v)^3 -0.655*(-2/ind_hover_v)^4); %velocidade induzida em descida
            PW_descent = (-2 + Ind_power_factor*ind_desc_v) + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ DL ./ 8;
        end
        
        % Minimum power required
        
        PW_f = max(PW_CRUISE,PW_TURN); %power-to-weight necessário para voo em fixed wing
        P_f = PW_f*W;
        PW_v = max(PW_climb,PW_descent); %power-to-weight necessário para voo em VTOL
        P_v = PW_v*W;
        config(i,j,k,l,12) = PW_f;
        config(i,j,k,l,13) = P_f;
        config(i,j,k,l,14) = PW_v;
        config(i,j,k,l,15) = P_v;
        %fixed wing - total de 26km à velocidade de 25m/s e curva de 180º
        %efetuada à velocidade determinada V_turn e com raio de 17.8m
        %(extrapolado do relatório de PI)
        %VTOL - descida e subida dos 55m - subida a 1m/s e descida a 2m/s;
        %como usa-se meios diferentes de propulsão para voo horizontal e
        %vertical, não se consegue incorporar as duas fases de transição
        %neste modelo
        Energy_spent = P_f*(26000/25 + pi*17.8/V_turn)+P_v*(55/1 + 55/2); %energia necessária para efetuar o perfil de voo completo
        config(i,j,k,l,16) = Energy_spent;
        
        %configuração ideal é a que consome menos energia (consumo
        %específico de energia é um dos parâmetros a ser avaliado na
        %competição)
        minimo = config(1,1,1,1,16);
        for ii=1:length(AR_range)
            for jj=1:length(b_range)
                for kk=1:length(R_range)
                    for ll=1:length(N_blades_range)
                        if config(ii,jj,kk,ll,16) < minimo
                            minimo = config(ii,jj,kk,ll,16);
                            i_ideal = ii;
                            j_ideal = jj;
                            k_ideal = kk;
                            l_ideal = ll;
                        end
                    end
                end
            end
        end
        
        
            end
        end
    end
end

fprintf('AR = %i\n',config(i_ideal,j_ideal,k_ideal,l_ideal,1));
fprintf('Envergadura = %.2f m\n',config(i_ideal,j_ideal,k_ideal,l_ideal,2));
fprintf('Raio de cada rotor = %.2f m\n',config(i_ideal,j_ideal,k_ideal,l_ideal,3));
fprintf('Nº de hélices de cada rotor = %i\n',config(i_ideal,j_ideal,k_ideal,l_ideal,4));
fprintf('Área da asa = %.4f m^2\n',config(i_ideal,j_ideal,k_ideal,l_ideal,5));
fprintf('CD0 = %.4f\n',config(i_ideal,j_ideal,k_ideal,l_ideal,7));
fprintf('Massa da aeronave (MTOM) = %.4f kg\n',config(i_ideal,j_ideal,k_ideal,l_ideal,8)/9.81);
fprintf('Wing loading, W/S = %.4f N/m^2\n',config(i_ideal,j_ideal,k_ideal,l_ideal,9));
fprintf('Área total dos 4 rotores = %.4f m^2\n',config(i_ideal,j_ideal,k_ideal,l_ideal,10));
fprintf('Disk loading, W/A = %.4f N/m^2\n',config(i_ideal,j_ideal,k_ideal,l_ideal,11));
fprintf('Power loading, W/P, para voo em fixed wing = %.4f N/W\n',1/config(i_ideal,j_ideal,k_ideal,l_ideal,12));
fprintf('Power necessário em fixed wing = %.4f W\n',config(i_ideal,j_ideal,k_ideal,l_ideal,13));
fprintf('Power loading, W/P, para voo VTOL = %.4f N/W\n',1/config(i_ideal,j_ideal,k_ideal,l_ideal,14));
fprintf('Power necessário em VTOL = %.4f W\n',config(i_ideal,j_ideal,k_ideal,l_ideal,15));
fprintf('Energia total gasta no perfil de voo escolhido = %.3f kJ\n',config(i_ideal,j_ideal,k_ideal,l_ideal,16)/1000);


%%% GRÁFICO DE DESIGN POINT PARA A CONFIGURAÇÃO IDEAL
wl = 0:5:2000;
dl = 0:5:4000;
pl = 0:0.0005:0.8;
[plf_grid, wl_grid] = meshgrid(pl, wl);
[plv_grid, dl_grid] = meshgrid(pl, dl);
cf = ones(length(wl), length(pl));
cv = ones(length(dl), length(pl));

% Configure plot
colors = {'#0072BD','#D95319','#EDB120','#7E2F8E','#77AC30','#4DBEEE','#A2142F'};
figure();
yyaxis right;
legend;
hold on;
a = gca;
a.Title.String = 'Design Point';
a.XLim = [0 pl(end)];
a.XLabel.String = 'W/P';
a.YLim = [0 dl(end)];
a.YLabel.String = 'W/A';
a.LineStyleOrder = '-';
colororder(colors)
yyaxis left;
a.YLim = [0 wl(end)];
a.YLabel.String = 'W/S';
a.LineStyleOrder = '-';
colororder(colors)

k = config(i_ideal,j_ideal,k_ideal,l_ideal,6);
wl_design = config(i_ideal,j_ideal,k_ideal,l_ideal,9);
dl_design = config(i_ideal,j_ideal,k_ideal,l_ideal,11);
fpl_design = 1/config(i_ideal,j_ideal,k_ideal,l_ideal,12);
vpl_design = 1/config(i_ideal,j_ideal,k_ideal,l_ideal,14);

yyaxis left;
forward_region = cf;
vertical_region = cv;


%Constraint que maximiza a performance para long range
range_constraint = 0.5 .* rho .* 25^2 .* sqrt(config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ k);
forward_region0 = wl_grid < 0.5 .* rho .* 25^2 .* sqrt(config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ k);
yyaxis left;
plot([pl(1) pl(end)], [range_constraint range_constraint], 'DisplayName', "Range constraint");


cruise_speed_constraint = eta_p_cruise ./ (rho .* 25.^3 .* config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ 2 ./ wl + 2 .* k .* wl ./ rho ./ 25);
forward_region1 = plf_grid < eta_p_cruise ./ (rho .* 25.^3 .* config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ 2 ./ wl_grid + 2 .* k .* wl_grid ./ rho ./ 25);
yyaxis left;
plot(cruise_speed_constraint, wl, 'b','DisplayName', "Cruise speed constraint");

turn_constraint = 1./ (1 ./ eta_p_turn .* ( rho .* Speed(k,wl,rho,config(i_ideal,j_ideal,k_ideal,l_ideal,7)).^3.*config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ 2 ./ wl + 2 .* k .*n.^2 ./ rho ./ Speed(k,wl,rho,config(i_ideal,j_ideal,k_ideal,l_ideal,7)) .* wl )) ;
forward_region2 = plf_grid < 1./ (1 ./ eta_p_turn .* ( rho .* Speed(k,wl_grid,rho,config(i_ideal,j_ideal,k_ideal,l_ideal,7)).^3.*config(i_ideal,j_ideal,k_ideal,l_ideal,7) ./ 2 ./ wl_grid + 2 .* k .*n.^2 ./ rho ./ Speed(k,wl_grid,rho,config(i_ideal,j_ideal,k_ideal,l_ideal,7)) .* wl_grid ));
yyaxis left;
plot(turn_constraint, wl, 'g', 'DisplayName', "Turn constraint");

V_tip = Omega*config(1,1,k_ideal,1,3);
Solidity = config(1,1,1,l_ideal,4)*0.02/pi/config(1,1,k_ideal,1,3);
climb_constraint = 1./ (1 - Ind_power_factor ./ 2 + Ind_power_factor .* sqrt(1 + 2 .* dl ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ dl ./ 8) ;
vertical_region1 = plv_grid < 1./ (1 - Ind_power_factor ./ 2 + Ind_power_factor .* sqrt(1+ 2 .* dl_grid ./ rho2) ./ 2 + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ dl_grid ./ 8);
yyaxis right;
plot(climb_constraint, dl, 'r', 'DisplayName', "Vertical climb constraint");

if (sqrt(config(i_ideal,j_ideal,k_ideal,l_ideal,8)/2/rho2/config(i_ideal,j_ideal,k_ideal,l_ideal,10)) <= 1)
    descent_constraint = 1./ (-2 - Ind_power_factor .* (-2) ./ 2 - Ind_power_factor .* sqrt((-2).^2 - 2 .* dl ./ rho2) ./ 2 + rho2 .* Omega*config(1,1,k_ideal,1,3).^3 .* config(1,1,1,l_ideal,4)*0.02/pi/config(1,1,k_ideal,1,3) .* C_d_rotor ./ dl ./ 8) ;
    vertical_region2 = plv_grid < 1./ (-2 - Ind_power_factor .* (-2) ./ 2 - Ind_power_factor .* sqrt((-2).^2 - 2 .* dl_grid ./ rho2) ./ 2 + rho2 .* Omega*config(1,1,k_ideal,1,3).^3 .* config(1,1,1,l_ideal,4)*0.02/pi/config(1,1,k_ideal,1,3) .* C_d_rotor ./ dl_grid ./ 8);
else
    ind_hover_v = sqrt(config(i_ideal,j_ideal,k_ideal,l_ideal,8)/2/rho2/config(i_ideal,j_ideal,k_ideal,l_ideal,10));
    ind_desc_v = ind_hover_v*(Ind_power_factor -1.125*(-2)/ind_hover_v -1.372*(-2/ind_hover_v)^2 -1.718*(-2/ind_hover_v)^3 -0.655*(-2/ind_hover_v)^4);
    V_tip = Omega*config(1,1,k_ideal,1,3);
    Solidity = config(1,1,1,l_ideal,4)*0.02/pi/config(1,1,k_ideal,1,3);
    descent_constraint = 1./ (-2 + Ind_power_factor.*ind_desc_v + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ dl ./ 8) ;
    vertical_region2 = plv_grid < 1./ (-2 + Ind_power_factor.*ind_desc_v + rho2 .* V_tip.^3 .* Solidity .* C_d_rotor ./ dl_grid ./ 8) ;
end
yyaxis right;
plot(descent_constraint, dl, 'y', 'DisplayName', "Vertical descent constraint");


cf = cf .* forward_region0 .* forward_region1 .* forward_region2;
cv = cv .* vertical_region1 .* vertical_region2;

% Plot feasible design region
cf(~cf) = NaN;
yyaxis left;
surf(pl, wl, cf, 'FaceAlpha', 0.2, 'FaceColor', '#0072BD', 'EdgeColor', 'none', 'DisplayName', 'Forward Flight Design Space');
cv(~cv) = NaN;
yyaxis right;
surf(pl, dl, cv, 'FaceAlpha', 0.2, 'FaceColor', '#D95319', 'EdgeColor', 'none', 'DisplayName', 'Vertical Flight Design Space');

% Plot design point
yyaxis left;
scatter(fpl_design, wl_design, 'filled', 'MarkerEdgeColor', '#0072BD', 'MarkerFaceColor', '#0072BD', 'DisplayName', 'Forward Flight Design Point');
yyaxis right;
scatter(vpl_design, dl_design, 'filled', 'MarkerEdgeColor', '#D95319', 'MarkerFaceColor', '#D95319', 'DisplayName', 'Vertical Flight Design Point');


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

function CD0 = CD0_estimate(AR,b,R,V)
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

%% Geometria
%Wing
%wingspan
bw = b;
%average wing chord
cw = bw/AR;
%area 
Sw = bw*cw;
%Distance between wing and tail
lh = 0.5*bw/1.3;

%Tail
%horizontal stabilizer area
Sh = StabilityCorrigida(Sw,cw,lh);
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

%% Cd fuselagem
f = 1.25*lh; %fuselage length
A_top = 0.2136*f; %Área vista do topo tendo em conta dimensões da CB desenhada
A_side = 0.1999*f; %Área vista de lado tendo em conta dimensões da CB desenhada
S_wet = 1.7*(A_top+A_side);
fineness_rat = f/0.29; %diâmetro equivalente da CB desenhada é igual a 29cm
Q = 1.2; %fator de interferência retirado do relatório de PI
F = 1 + 60/fineness_rat^3 + fineness_rat/400; %fator de forma
M = V/340; %número de Mach
Rex = V*f/miu;
C_f = 0.455/(((log(Rex))^2.58)*((1+0.144*M^2)^0.65));
CD_CB=S_wet*C_f*F*Q/Sw;

%% Cd booms
b = 1.56*f; %comprimento dos booms tendo em conta a sua dimensão relativa à fuselagem (extrapolado do relatório de PI)
A_topp = 0.03*b; %diâmetro dos booms retirado do relatório de PI
S_wett = 1.7*2*A_topp;
fin_rat = b/0.03;
Qq = 1.1;
Ff = 1 + 60/fin_rat^3 + fin_rat/400;
Rexx = V*b/miu;
C_ff = 0.455/(((log(Rexx))^2.58)*((1+0.144*M^2)^0.65));
CD_boom = 2*S_wett*C_ff*Ff*Qq/Sw; %CD0 dos dois booms

%% Cd rotors
CDrot = 4*0.1*2*R*0.02/Sw; %CD0 de cada rotor: CD0 = 0.1*(Diâmetro_rotor*corda_rotor)/Área_asa

%% Cálculos finais

CDsup = CDw+CDtv+CDth;
%minimum drag coefficient of the airplane 
CDtot = CDsup + CDrot + CD_CB + CD_boom;
%correction factor
CD0 = 1.1*CDtot;

end

function W = weight_estimate(AR,b,R,Nblad)
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
    Sh = StabilityCorrigida(S,cw,lh);
    
%Fuselage length
    f = 1.25*lh;

    %The following expressions are obtained via an empirical approach and
    %based on results from previous airplanes
    %% Asa
    m_w = 0.33*(S^1.6*AR^0.9)^0.73;
    
    %% Cargo Bay + Fuselagem e booms
    % carbon tube ~ 0.286kg/m
    
    payload = 2;
    m_fus = 0.8*(0.286*f);
    m_cb = 0.0467*payload + 0.6;  
    m_booms = 2*0.11*1.56*f; %valor de massa por comprimento do boom igual a 0.11 kg/m extrapolado do relatório de PI
    m_st = payload + m_fus + m_cb + m_booms;
    
    %% Cauda 
    m_t = 2*Sh;
    
    %% Aviónica, motores e bateria
    
    %todas as massas seguintes (exceto a massa das helices do rotor) foram
    %retiradas do relatório de PI, onde motores foram escolhidos e estudos
    %de eletrónica e bateria foram efetuados
    m_avion = 0.8;
    m_motores = 0.218+4*0.132;
    m_helice_push = 0.08;
    m_helice_rotor = 4*Nblad*0.1*R; %o valor de 0,1kg/m de massa por comprimento de cada blade foi mais uma vez extrapolado do relatório de PI
    m_bateria = 2.7;
    m_out = m_avion + m_motores + m_helice_push + m_helice_rotor + m_bateria;
        
    %% Total
    m_aero = m_st + m_t + m_out + m_w;
    
    W = m_aero*9.81;
    
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

function [Sh] = StabilityCorrigida(Sw,Cmw,lh)
% =========================================================================
% =========================================================================
% Description: function that creates a tail capable of stabilizing the wing
% that is being tested, using the equation of the coefficient of
% stabilization, whose value is 0.5
% =========================================================================
% Inputs: Sw: wing area; Cmw: mean chord of the wing; TR: taper ratio; lh:
% distance between the aerodynamic center of the wing the the tail
% =========================================================================
% Outputs: Sh: area of the horizontal stabilizer
% =========================================================================

% Correction factor for the wing's area for wings with sweep
fator_correcao= -0.32003*Sw+0.21792;

% Defining the wing geometry from the data
TR = 0.47;
c_root= (2*Cmw)/(1+TR);

% Horizontal stability with a 0.5 coefficient
cmac = (2/3)*c_root*( (1+TR+TR^2) / (1+TR) ); %mean aerodynamic chord

SL=0.5*Sw*cmac;
Sh=SL/lh; %Area of the horizontal stabilizer before correction

% Applying the correction factor to the tail's area
Sh= Sh+fator_correcao; 

end