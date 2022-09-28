%% V3.1 Optimum Design
% =========================================================================
% =========================================================================
% Description: this program was developed as means of obtaining the design
% point bearing in mind the constraints imposed by the Air Cargo Challenge
% (ACC).
% The optimum design point will be the one that maximizes the score and is
% a fuction of the following % variables:
% -AR: Aspect Ratio;
% -S: Wing Surface Area;
% -Altura: Height [0,100]m achieved within 60s after takeoff;
% -Gear: Discrete variable: 0-> Taildragger; 1-> Tricycle;
% -Payload: [0, 4.2]kg.
% =========================================================================
% =========================================================================

%%
clear;
close all;
%% Obtaining additional variables
% =========================================================================
% =========================================================================
% Description: commented section of the code that allows for a faster
% gathering of important data given a specific set of variables.
% =========================================================================
% =========================================================================
%
% Setting variables 
% AR = 7.4250;
% S = 0.5812;
% sweep =3;
% 
% "avalia" will yield  the score (pontos), gear, hight(altura), payload and
% rolamento (wheel diameter 10 or 20cm) for a specific AR and S pair
% [pontos, gear,altura,payload,rolamento] = avalia([AR,S]);
%
% "sacagg" will yield the climb angle
% [gg] = sacagg(AR,S,gear,altura,payload,rolamento);
%
% "weight_estimate" will yield the total weight (peso) of the aircraft.
% Peso = weight_estimate(S,AR,sweep,payload);

%% 1. Variables Definition and Data Compilation
% =========================================================================
% =========================================================================
% Description: in this section  we start by defining both an AR a S range
% whose performance will be evalueted. Evaluating functions will be
% invoqued an the data will be plotted in a surface graph, with color encoding,
% allowing to build a 3D surface plot of the score with the following
% variables:
% AR: x-axis;
% S: y-axis;
% Score: z-axis;
% -Hight [0,100]m achieved within 60s after takeoff - color encoded;
% -Gear: Discrete variable: 0-> Tail Dragger; 1-> Tricycle - color encoded;
% -Payload: [0, 4.2]kg - color encoded
% =========================================================================
% =========================================================================
%
% Defining AR e S range
AR_range = 5:0.25:10; 
S_range = 0.45:0.025:0.75;

% Total number of combinations for AR and S
NPoints = length(AR_range)*length(S_range);

% Allocating memory for the vector "answer", which will allow to evaluate all
% the previous combinations
answer = zeros(length(AR_range),length(S_range));
% "colour" is a tensor that allows for an RGB represenation of 3 variables,
% being them:
% Height: 
% Gear: Tail Dragger: stays green <-> Tricycle: yellowish;
% Payload: an increase in payload will translate into blueish color;
% Height: a greater height will translate into a greener color
colour = zeros(length(AR_range),length(S_range),3);


l=1;
 for i=1:length(AR_range)
    for j=1:length(S_range)
        answer(i,j) = general_Geometry(AR_range(i),S_range(j),1);
        if answer(i,j) == 1
            [answer(i,j),gear,altura,payload] = avalia([AR_range(i),S_range(j)]);
            colour(i,j,1) =gear/2;
            colour(i,j,2) =altura/100;
            colour(i,j,3) =payload/3.6;
        end
        l = l+1;
    end
 end
 
% Surface Plot of Design Point
% Variables: AR, S, Score
[AR,S] = meshgrid(AR_range,S_range);
surf(AR',S',answer, colour);
xlabel('AR');
ylabel ('S');
zlabel ('Score');
title('Design Space');
% Hold on: allows plotting of the optimum point, calculated in the "2.
% Gradien" Section
hold on

%% 2. Gradient
fun = @avalia;
x0 = [6,0.5];
[x,fval,exitflag,output] = fminsearch(@avalia,x0);
[pontos, gear,altura,payload] = avalia(x);
W = weight_estimate(x(2),x(1),5,payload);
plot3(x(1),x(2),pontos,'.');

%% 3.Evaluating Functions
function [pontos, gear, altura, payload,rolamento] = avalia(x)
% =========================================================================
% =========================================================================
% Description: For a given combination of wing surface area (S) and aspect
% ratio (AR), evaluates which configuration maximizes the points and gives 
% some extra information about it, for instance the height achieved during
% climb, the optimum payload wheight and the presence or not and type of
% wheels in the tail.
% =========================================================================
% Inputs:
% x [vector, dimension 2] - values of aspect ratio and surface area 
% relative to the wing;
% =========================================================================
% Outputs:
% pontos [scalar] - points given to the best configuration for the input 
% parameters;
% gear [scalar] - 1 if it has tail dragger and 0 if it doesn't. 
% altura [scalar] - Height achieved during climb;
% payload [scalar] - Optimum Payload wheight; 
% rolamento [scalar] - Type of wheels used (they differ in diameter).
% =========================================================================
% =========================================================================
    AR_wing = x(1);
    S_wing = x(2);
    %tri -> 1
%checks if the configuration with a rear wheel fits the box    
    tri = general_Geometry(AR_wing, S_wing,1);
    % tail Dragger -> 0
%checks if the configuration with tail dragger fits the box    
    tailDragger = general_Geometry(AR_wing, S_wing,0);
%if the configuration doesn't fit either way than it recieves 0 points
    if tri  == 0 && tailDragger == 0
        pontos = 0;
        gear = 1;
        altura = 0;
        payload = 0;
        rolamento = 0;
%in case a tail dragger aproach can fit the box than the points are calculated        
    elseif tri  == 0 && ~(tailDragger == 0)
%evaluates the best configuration and its points        
        [pontos, altura, payload,rolamento] = contas(AR_wing, S_wing,0);
%for the "Gradient" part of the program to work the points need to became the
%negative so that the function can search for a minimum with corresponds to 
%the best pontuation        
        pontos = - pontos;
%no back wheel -> trail dragger        
        gear = 0;
%in case a trycicle aproach can fit the box than the points are calculated         
    elseif ~(tri  == 0) && tailDragger == 0
%evaluates the best configuration and its points          
        [pontos, altura, payload,rolamento] = contas(AR_wing, S_wing,1);
        pontos = - pontos;
%has 3 wheels         
        gear = 1;
    else 
%both configurations are possible, so it calculates the points for each one        
        [resposta0, altura0, payload0,rolamento0] = contas(AR_wing, S_wing,0);
        [resposta1, altura1, payload1,rolamento1] = contas(AR_wing, S_wing,1);
%checks which has the most points        
        pontos = -max([resposta0,resposta1]);
%now that the answer for which is the best configuration is answered then 
% its characteristics are saved in the output variables 
        if pontos == -resposta0
            gear = 0;
            altura = altura0;
            payload = payload0;
            rolamento = rolamento0;
        else
            gear = 1;  
            altura = altura1;
            payload = payload1;
            rolamento= rolamento1;
        end
    end
end

function [gg] = (AR,S,gear,altura,payload,rolamento)
% =========================================================================
% =========================================================================
% Description: acquires de value of gg i.e. the climb angle;
% =========================================================================
% Inputs:
% AR [scalar] - Aspect Ratio;
% S [scalar] - Wing Surface Area;
% gear [scalar] - Number of wheels implemented in the tail;
% altura [scalar] - Intended height to achieve during climb;
% payload [scalar] - Payload Weight;
% rolamento [scalar] - Type of wheels choosen;
% =========================================================================
% Outputs:
% gg [scalar] - climb angle (in degrees);
% =========================================================================
% =========================================================================
    %acceptable prechosen angle of sweep
    sweep = 3;                       
    %quadratic coefficients relative to the curve of thrust as a funtion of
    %velocity, during take-off, obtained in the script
    %"script_para-gráfico_3D"
    T2_to = -0.0146;
    T1_to = -0.1652;
    T0_to = 16.855;
    %obtains an estimate value of the zero-lift drag coefficient 
    CD0 = CD0_estimate(S,AR,sweep,payload, rolamento, gear);
    %obtains an estimate value of the weight of the airplane 
    W = weight_estimate(S,AR,sweep,payload);
    %obtains the value of k used to calculate the drag induced by the lift
    k = k_factor(AR,sweep);
    
    fc0 = -4e-6*W + 0.8028;
    fc5 = -5e-6*W + 0.7968;
    fc8 = -4e-6*W + 0.7943;
    %forma simples
    W_ = W/((fc0 + fc5+ fc8)/3);
    
    %obtains the velocity which maximizes L/D
    [~, V_LD, ~] = Speed(k,W,W_,S,CD0,T2_to,T1_to,T0_to);
    %correction factor
    aux = 1.1*V_LD;    
    %rate of climb assuming constant climb for 55 seconds
    RC = altura/55;
    %climb angle
    gg=atan(RC/aux)*180/pi;
end

function [pontos, altura, payload,rolamento] = contas(AR_wing, S_wing, tail_tri)
% =========================================================================
% =========================================================================
% Description: Fills in all the needed information in "config" and
% "c_check" matrices. Through the usage of the equations avaliable in the
% book "General Aviation", it's possible to complete the matrix c_check for
% every configuration. This matrix consists of 7 performance tests. 1
% indicates that there's enough power to perform the maneuver and 0 it
% doesn't. The 7 tests are in order: Stall velocity inferior to 10, enough
% power to perform climb, enough power to perform 60 meter take-off, enough 
% power to perform 40 meter take-off, enough power to perform cruise at
% maximum velociy, enough power to turn, enough power to turn using the
% least battery possible. The config contains the following 11 features:
% Wing Surface Area, Payload Weight, Maximum Height, Aspect Ratio, K
% factor, Weight, Zero-Lift Drag Coefficient, Maximum Velocity (Cruise),
% Weight/Surface Area, Type of Wheels, Number of Wheels in the tail.
% =========================================================================
% Inputs:
% AR_wing [scalar] - Wing Aspect Ratio;
% S_wing [scalar] - Wing Surface Area;
% tail_tri [scalar] - Number of wheels implemented in the tail;
% =========================================================================
% Outputs:
% pontos [scalar] - points given to a certain configuration depending on 
% its performance;
% altura [scalar] - Intended height to achieve during climb;
% payload [scalar] - Payload Weight;
% rolamento [scalar] - Type of Wheels used;
% =========================================================================
% =========================================================================    
%vector which contains the number of blood bags    
    payload = 1:42;
%each bag weights 100 grams    
    m_payload = payload*0.1;
%vector which contains the intended height during climb in meters 
    hi = 30:10:100;
%type of wheel chosen
    rol = [0,1];
%all the possible configurations
    M = length(payload)*length(hi)*2;
%11 parameters, 6 independent and the other 5 dependent from the 
%previous ones 
    N = 11;
%alocates memory for the different possible configurations
    config = zeros(M,N);
%sorts the "config" matrix puting the configurations with the highest
%points on top
    sorted = zeros(M,N+2); % +1 para points e bonus
%both "sorted" and "Final" matrices contain only the configurations able to
%perform all the flight phases
    Final = zeros(M,N+2); % +1 para points e bonus
%matrix which indicates a bonus for configurations able to take-off within
%a 40 meters run
    car40 = zeros(M,1);

%matrix which checks if all the flight phases are accomplishable
%phases: stall, climb, to60, to40, turn, land , cruise
    c_check= zeros(M,7);

    c=1;
%fill the "config" matrix with all the different combinations of
%independent parameters
    for j=1:length(payload)
        for k=1:length(hi)
                for ii = 1:2
                        config(c,1) = S_wing;
                        config(c,2) = m_payload(j);
                        config(c,3) = hi(k);
                        config(c,4) = AR_wing;
                        config(c,10) = rol(ii);
                        config(c,11) = tail_tri;
                        c=c+1;
                end
        end
    end

    %% Estimativas iniciais
%Calculate Oswald Coefficient, Returns k
    for i=1:M
        config(i,5) = k_factor(config(i,4),3);
    end

%Weight estimate
    for i=1:M
        config(i,6) = weight_estimate(config(i,1),config(i,4),3,config(i,2));
%Ratio W/S for the wing        
        config(i,9) = config(i,6)/config(i,1);
    end

%CD0 estimate
    for i=1:M
        config(i,7) = CD0_estimate(config(i,1),config(i,4),3,config(i,2), config(i,10), config(i,11));
    end
    
    %% Available power Curves

%The following triplets of numbers correpond to the quadratic coefficients 
%of the curves which relate Thrust with Velocity in the form T=aV^2+bV+c
%Data from 2021-2022
%Power curve during cruise
T2_lt = -0.0186479;
T1_lt = - 0.0074113;
T0_lt = 14.8848145;


%% Check Flight Path

%Data
rho = 1.225;    %[kg/m^3]
g = 9.81;       %[m/s^2]
%miu = 0.07;     %friction constant
%conditions when the stall happens
CL_max = 1.6;
%maximum Cl during take-off
CL_TO = 1.3;
%gg=zeros(length(config));


for i = 1:length(config)
    
    %Value assignment
    S = config(i,1);
    W = config(i,6);
    CD0 = config(i,7);
    k = config(i,5);
    h = config(i,3);
    AR = config(i,4);
    WS = config(i,9);
    c_rol = [0.08 , 0.1];
    miu = c_rol(config(i,10)+1);
    tail_tri = config(i,11);
%drag model used during take-off
    CD_TO = CD0+k*CL_TO^2+0.04;    
    
%    fc0 = -4e-6*W + 0.8028;
%    fc5 = -5e-6*W + 0.7968;
%    fc8 = -4e-6*W + 0.7943;
    
%Simple form
    %WS = WS/((fc0 + fc5+ fc8)/3);
    %W_ = W/((fc0 + fc5+ fc8)/3);  %caso se queira o W para calcular o lift
    W_ = W;
    
    %% 1 - Check Stall Speed - Ver página 67, Capitulo 3, General Aviation
    
    stall_limit = 10; %maximum limit (m/s)
    v_stall = sqrt (WS/(0.5*rho*CL_max));
    
    if v_stall < stall_limit            
        c_check(i,1)=1; %stalls below safe limit
    else
        c_check(i,1)=0; %velocity above the safe limit
    end
   
    
    
end

%% Sort Results
[s1, s2] = size(c_check);
k = 1;
for i = 1:s1
    n1 = 0;
    for j=1:s2
        n1 = c_check(i,j) + n1; 
    end
%If all the flight phases are accomplishable or every one of them except 
%for the 40 meters take-off then the airplane is 'ok'  
    if (n1 == s2) || ((n1 == s2-1) && c_check(i,4) == 0)
        ok = 1;
    else
        ok = 0;
    end
%If the airplane is 'ok' then it enters on the "sorted" vector    
    if ok == 1 
%nnz(c_check(i,:)) == s2 || ( ( nnz(c_check(i,:)) == s2-1 && c_check(i,4)==0) )
%matrix with relevant flight characteristics for score calculations        
        sorted(k,1:N) = config(i,:);
%bonus given to airplanes which can perform the take-off within a 40 meters run        
        if c_check(i,4)==1
            car40(k)=1;
        end
        k=k+1;
    end
end
%eleminates the remaining combinations 
sorted(k:end,:) = []; 
car40(k:end,:) = []; 
Final(k:end,:) = []; 

%% Score Calculation
%As the points depend on the performance from other teams, at each step
%it's assumed that at least one of the 3 evaluated parameters is maximized 

%a configuration is not stable with a payload weight superior to 3.6 kg
payload_max = 3.6; %36*.1;
%does not take the wind into account 
V_max = 23;%22.3; %22;
%the maximum height corresponds to 100 meters and a pre-score of 1200 points  
ps_max = 1200;
[s1, ~] = size(sorted);

for i=1:s1
    m_payload = sorted(i,2);
    V_cruise = sorted(i,8);
    h = sorted(i,3);
%10% bonus for airplanes which perform a take-off in less than 40 meters
    if car40(i) == 1
        mult = 1.1;
    else
        mult = 1;
    end
%based on the regulations of the competetion, the score is calculated for
%each configuration
    if m_payload < payload_max
    score(1) = 1000*(m_payload /payload_max);
    else 
        score(1) = 1000;
    end
    
    if V_cruise < V_max
        score(2) = 1000*(V_cruise/V_max);
    else
        score(2) = 1000;
    end
    
    pscore = -3.92*10^-5*h^4+ 1.08*10^-2 *h^3 -1.156*h^2 +64.2*h -537;
    score(3) = 1000*(pscore/ps_max);
    
    FinalScore = round( (score(1)+score(2)+score(3))*mult);
%fills the final score for each combination and the existence of bonus     
    sorted(i,N+1) = FinalScore;
    sorted(i,N+2) = mult;
end
%organizes the matrix "sorted" putting the configurations with the most
%points on top
[A, index] = sort(sorted(:,N+1),'descend');

for i=1:s1
    Final(i,:)=sorted(index(i),:);
end
%takes the 3 evaluated features out from the configuration with the
%highest points (first line from "Final" matrix)
if ~isempty(Final)
    pontos = Final(1, 12);
    altura = Final(1, 3);
    payload = Final(1, 2);
    rolamento = Final(1,10);
else
%in case "Final" matrix is empty
    pontos = 0;
    altura = 0;
    payload = 0;
    rolamento = 0;
end
end

%% 4. Geometry functions
function [answer] = general_Geometry(AR_wing, S_wing,tail_tri)
%% Fixed Parameters
% =========================================================================
% =========================================================================
% Description: defining fixed variables that make up to the size
% restrictions, which will be essential to ensure that the airplane will
% fit within the size restriction box and, if it fits, making sure it
% allows for a stable configuration.
% =========================================================================
% =========================================================================

% Size restriction box side length (m)
l_box = 1.5;

% Propeller diameter (m)
d_propeller = 0.254;

% Motor longitudinal length (m)
l_motor = 0.05;

% Wing sweep angle (degree)
sweep_wing = 6.42;

% Wing Taper Ratio
TR_wing = 0.47;

    %% Main Calculations

    % Geometry Filter
    Solution = geometry_box_check(l_box, d_propeller, l_motor, AR_wing, S_wing, sweep_wing, TR_wing);
    
    if not(any(Solution))
        answer=0;
    else
        
    % Stability Filter
    M2=completa_matriz(Solution, S_wing, sqrt(S_wing*AR_wing), sqrt(S_wing/AR_wing), sweep_wing, TR_wing,tail_tri);
    
    % Matriz_Ordenada = ordena(M2);
    
        if any(M2)
            answer=1; 
        else
            answer=0;
        end
    end
end

function [Solution] = geometry_box_check(l_box, d_propeller, l_motor, AR_wing, S_wing, sweep_wing, TR_wing)
% =========================================================================
% =========================================================================
% Description:
% Given an initial estimate of the Aspect Ratio of a wing, the Design
% Point parameters derived from this Aspect Ratio, Wing Area, an initial
% estimate of the Wing Leading Edge, Sweep angle, a Wing Taper Ratio and the
% box and engine-propeller parameters (box side length, propeller diameter
% and motor longitudinal length), it indicates which angles of the box that
% can fit the desired wing. It returns a matrix, Solution, nx3, with the
% 1st column indicating the angle of the box, theta, 2nd column indicating
% the distance from the Wing Root Leading Edge to a box corner, 3rd column the 
% y position of the Wing Tip Leading Edge, for illustration purposes.
% =========================================================================
% Inputs:
% l_box [scalar] - box side length, in meters;
% d_propeller [scalar] - diameter of the propeller, in meters;
% l_motor [scalar] - motor longitudinal length, in meters;
% AR_wing [scalar] - wing Aspect Ratio;
% S_wing [scalar] - wing area, in meters squared;
% sweep_wing [scalar] - wing Leading Edge Sweep angle, in degrees;
% TR_wing [scalar] - wing Taper Ratio;
% =========================================================================
% Outputs:
% Solution [matrix, nx3] - combinations of all the possible box
% orientations that can fit the required wing geometry. 1st column
% indicating the angle of the box, theta, 2nd column indicating the Wing 
% Root Leading Edge to a box corner, 3rd column the y position of the Wing
% Tip Leading Edge, for illustration purposes.
% =========================================================================
% Side note: in this section all the points will be defined as row vectors 
% of length 2, being the first column (1) for the x coordinate and the
% second column (2) for the y coordinate: 
% Px: Px(1)-> x coordinate and px(2)-> y coordinate
% =========================================================================
% =========================================================================


%% Wing Geometrical Parameters
% The equations used to derive the formulas are between squared brackets []

% Wing span - obtained from Design Point (m) [AR=(b^2)/S]
% b_wing = 2.27;
b_wing = sqrt(S_wing*AR_wing);

% Wing mean chord - obtained from Design Point (m) [S=b*c_mean]
% c_mean_wing = 0.2483;
c_mean_wing = sqrt(S_wing/AR_wing);

% Wing Tip Chord (m) [TR=c_tip/c_root] and [c_mean = (cr + ct)/2]
c_tip_wing = 2*c_mean_wing/( 1 + 1/TR_wing );

% Wing Root Chord (m) [TR=c_tip/c_root]
c_root_wing = c_tip_wing/TR_wing;

% Vector for
Solution=zeros(10000,6);
%% Main Cycle

% Counter
i = 1;

% Box angle iteration (degree)
for gamma = 10:1:170
    
        % Complementary Box Angle (degree)
        theta = 180 - gamma;
        
        % Distance from the corner of the box to the motor (m)
        k_min = d_propeller/(2*tand(theta/2)) + l_motor;
                
        % =====================================
        % Geometry: Coordinates of the edges of the box in the xy plane
        
        % Half of the opposite diagonal of angle gamma (m)
        l_diagonal_gamma = l_box*sind(gamma/2);
        
        % Half of the opposite diagonal of angle theta (m) (m)
        l_diagonal_theta = l_box*sind(theta/2);
        
        % Point 1 (in negative x axis)
        P1(1) = -l_diagonal_theta;
        P1(2) = 0;
        
        % Point 2 (in positive y axis)
        P2(1) = 0;
        P2(2) = l_diagonal_gamma;
        
        % Point 3 (in positive x axis)
        P3(1) = l_diagonal_theta;
        P3(2) = 0;
        
        % Point 4 (in negative y axis)
        P4(1) = 0;
        P4(2) = -l_diagonal_gamma;
        
        % Point 5 (motor)
        P_motor(1) = 0;
        P_motor(2) = l_diagonal_gamma - k_min;
        
        % =====================================

        % Distance between the edge of the box and the leading edge of the
        % wing (root) iteration, from k_min to Half of the opposite 
        % diagonal of angle gamma
        for k = k_min:0.005:l_diagonal_gamma
            
                % saves box angle data (degree)
                Solution(i,1) = theta;
                % saves distance from the corner of the box to the motor (m)
                Solution(i,2) = k;             
                
                % Leading Edge Point of the wing at the root (m)
                P_LE(1) = 0; % x coordinate
                P_LE(2) = l_diagonal_gamma - k; % tcoordinate
            
            	% checks if the wing fits in the box, Span wise
                [y_LE_tip, Flag_b] = check_span(P1, P2, sweep_wing, P_LE, b_wing);
                
                % saves the wing tip y position
                Solution(i,3) = y_LE_tip;  
                
                % saves Span Flag
                Solution(i,4) = Flag_b;
                
                % checks if the wing fits in the box, Chord wise
                Flag_c = check_tip_chord(P1, P4, y_LE_tip, b_wing, c_tip_wing);
                
                % saves Chord Flag 
                Solution(i,5) = Flag_c;
                
                % saves the general Flag of the wing, in other words, if
                % the this particular wing fits in the box it saves a 0
                % (Flag Down) in this 6th column, it the wing doesn't fit,
                % span or tip-chord wise, it saves a 1 (Flag Up)
                Solution(i,6) = or(Flag_b, Flag_c);
                
                
                % updates Counter
                i = i + 1;
        end
end

%% Data Processing

% Removes rows with Flag_geometry = 1, in other words, removes the rows
% where the wing cannot fit in the box
Solution(Solution(:, 6)== 1, :) = [];

% removes the Flags columns from the matrix
Solution(:,4) = [];
Solution(:,4) = [];
Solution(:,4) = [];

% reorders matrix so theta increases
Solution = flip(Solution);


%% Variable clearance

% clears all the variables that are not relevant
clear Flag_b
clear Flag_c
clear gamma
clear i
clear k
clear k_min
clear l_diagonal_gamma
clear l_diagonal_theta
clear P1
clear P2
clear P3
clear P4
clear P_LE
clear P_motor
clear theta
clear y_inter

end

function [M2] = completa_matriz(Solution, Sw, bw, corda_media, flecha_graus, TR_wing,tail_tri)
% =========================================================================
% =========================================================================
% Description: the programm builds an initial M2 matrix in which columns one finds
% the parameters for a specific wing and the rows are the different box
% configurations that fit this specific wing. Then the programm proceeds to
% test if, for each box configuration, one can manage to fit a tail that
% stabilizes the airplane
% =========================================================================
% Inputs: matrix "Solution" where we can find all the different
% configurations that fit the current wing. The rest of the inputs are wing
% parameters, Sw: Area of the Wing; bw: wing span; corda_media: mean chord;
% flecha_graus: sweep in degrees; TR_wing: wing taper ratio; tail_tri: if
% it is a tricycle or tail dragger configuration.
% =========================================================================
% Outputs: M2 matrix: a matrix with 4 columnsd: (:,1)-theta angle of the
% box; (:,2)-distance from the from wing's leading edge at the the root and
% the vertex of P1; (:,3)-maximum distance between the wing's aerodynamic 
% and the tail's aerodynamic center; (:,4)-horizontal stabilizer area
% =========================================================================
% =========================================================================


% Gathers the number of different box configurations that fit the wing we
% are testing, in other words, the rows of "Solution" which will be saved
% in "n"
[n,m]=size(Solution);
j=1; %linha da matriz nova

% M2 Matrix, a matrix with a number of rows equal to total number of
% different box configurations that fit the wing we are testing
M2 = zeros(n,4);


for i=1:n
    lh_max = procura_lh_max(TR_wing, Solution(i,1), Solution(i,2), Sw, bw, corda_media, flecha_graus);
    if tail_tri == 0 % tail dragger
       lh_max = lh_max/cosd(15); 
    end
    if lh_max ~= 0 %se a configuração não for viável
        M2(j,1)=Solution(i,1); %theta
        M2(j,2)=Solution(i,2); %k
        M2(j,3)=lh_max; %lh_max
        M2(j,4)= StabilityCorrigida(Sw,corda_media,TR_wing,lh_max); %Sh
        
        j=j+1;
    end
    i=i+1;
end

end

function [Final] = ordena(M)
if M ~=0
    Final = sortrows(M,4);
else Final=0;
end

end

function [lh_max] = procura_lh_max(TR, theta,k, Sw, bw, corda_media, flecha_graus)
% =========================================================================
% =========================================================================
% Description: tests a specific box configuration and a specific wing.
% Proceeds to check if there is any tail configuration that fits in the
% box finding the maximum distance between the aerodynamic center
% of the wing an that of the tail.
% =========================================================================
% Inputs:
% =========================================================================
% Outputs: 
% =========================================================================
% INPUT: Taper ratio, ângulo theta da caixa, distância entre o bordo de
% ataque e o vértice mais próximo da caixa
% OUTPUT: variável que toma o valor 1 se for possível (e o CC), o lh_max
% para o qual é possível (no caso de a primeira ter o valor 0, o segundo
% output é irrelevante) e a área do estabilizador horizontal


c_tip= (2*corda_media)/(1+1/TR); %tip chord
c_root= (2*corda_media)/(1+TR); % root chord

% half of gamma angle calculation from theta
gama= 90-theta/2; 
% box diagonal from P2 to P4 in meters
diagonal_menor= 3*sind(gama);

% Computing the distance between the leading edge of the wing and its
% aerodynamic center
if TR>0.375
        x_ca = c_tip/4 + 2*bw/(3*pi)*tand(flecha_graus);
else
        x_ca =  c_tip/4 + bw*(1+2*TR)/(6+6*TR)*tand(flecha_graus);
end
% Distance from P2 to the aerodynamic center
l=x_ca+k;
lh_max= 0.9 *(diagonal_menor-l) /cosd(15);

% We set the possible configuration to zero, which will be raised to 1 if
% we find a stable configuration for it within our box limitaitons.
configuracao_possivel=0; 

while (lh_max > (0.35*bw) && configuracao_possivel==0) 
        Sh= StabilityCorrigida(Sw,corda_media,TR,lh_max);
        if verificar_estabilidade(l, lh_max, theta, Sh)==1
            configuracao_possivel=1;
        else
            % we reduce the required lh_max and check again to see if the
            % new configuration fits in the box
            lh_max=lh_max-0.01;
        end
end
if configuracao_possivel==0
    lh_max=0;
end
end

function [cabe_na_caixa] = verificar_estabilidade(l, lh, theta, Sh)
% =========================================================================
% =========================================================================
% Description: Checks if, for the given wing area of the horizontal
% stabilizer (Sh), the  distance from P2 to the AC of the wing (l);
% =========================================================================
% Inputs: distance between the AC fo the wing and the AC of the tail, and the angle
% theta of the box, the configuration fits in the box.
% =========================================================================
% Outputs: 0-doesn't fit; 1-fits.
% =========================================================================


% half of gamma angle calculation from theta
gama= 90-theta/2;
% box diagonal from P2 to P4 in meters
diagonal_menor= 3*sind(gama);

% Distance between P2 and the aerodynamic center of the tail
Xcac= l+lh;
% Distance between P4 and the aerodynamic center of the tail
x=diagonal_menor-Xcac;

% Verifies if l+lh fits in current box configuration
if x <0
    cabe_na_caixa=0;
else
    if Sh < ((2*x)/(tand(gama)+0.5))^2
        cabe_na_caixa=1;
    else
        cabe_na_caixa=0;
    end
end
end

function [Sh] = StabilityCorrigida(Sw,Cmw,TR,lh)
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
c_tip= (2*Cmw)/(1+1/TR);
c_root= (2*Cmw)/(1+TR);
b= Sw/Cmw;

% Horizontal stability with a 0.5 coefficient
cmac = (2/3)*c_root*( (1+TR+TR^2) / (1+TR) ); %mean aerodynamic chord

SL=0.5*Sw*cmac;
Sh=SL/lh; %Area of the horizontal stabilizer before correction

% Applying the correction factor to the tail's area
Sh= Sh+fator_correcao; 
end