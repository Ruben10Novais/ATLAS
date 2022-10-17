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

%% 4. Geometry functions
function [Final] = ordena(M)
if M ~=0
    Final = sortrows(M,4);
else Final=0;
end

end
