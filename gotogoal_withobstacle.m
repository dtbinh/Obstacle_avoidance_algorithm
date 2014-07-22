% Obstacle avoidance algorithm simulation
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% H J Jayakrishnan, July 10 2014
% e-mail : hjjkrishnan@gmail.com
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% CAUTION: Read 'ReadMe' before trying to attempt simulation.

% obstacle co-ordinates

x_obs=4;
y_obs=1;

%clearance radius of obstacle

radius=.5;

%origin co-ordinates

x_org=0;
y_org=0;

plot(x_org,y_org,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','b')
hold on

% destination co-ordinates

x_dest=4.5;
y_dest=2;

plot(x_dest,y_dest,'p','LineWidth',2,'MarkerSize',16,...
    'MarkerEdgeColor','k','MarkerFaceColor','r')
axis ([-5 5 -5 5]); 
grid on
hold on

% Using symbolic tool box to solve point of tangency

 syms p q
 [p,q]= solve(p*(p-x_obs)+q*(q-y_obs)==0, (p-x_obs)^2+(q-y_obs)^2==radius^2, 'PrincipalValue',true)
 
% points of tangency

 x_tan=vpa(p);
 y_tan=vpa(q);
 
% go to target

go(x_tan,y_tan,0,0);
go(x_dest,y_dest,x_tan,y_tan);




        