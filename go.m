function [] = go(x_dest,y_dest,x_org,y_org)

%UAV final destination
r=x_dest;
s=y_dest;

%UAV initial position
a=x_org;
b=y_org;

%UAV current co-ordinates
x_old=a;
y_old=b;
x_new=0;
y_new=0;

%gain
k=5;

%time step
t=.1;

%error threshold
error_threshold=.1;

% max iteration
Max_iteration=50;

%initial heading
phi_old=0;
phi_new=0;

%obstacle co-ordinates
x_obs=4;
y_obs=1;

plot(x_obs,y_obs,'o','MarkerFaceColor','b')
viscircles([x_obs,y_obs],.5)
axis equal
hold on


    for m = 1: Max_iteration
    
       
         % Stop simulation if destination is reached
        if norm([r;s] - [x_new;y_new],2) < error_threshold
            v = 0;
            
        else
            v = 1;
        end
           
             
      %  phi_desired=vpa(atan((s-y_new)/(r-x_new)));
        
        
        switch r >= x_new
        case 1
            phi_desired = vpa(atan((s-y_new)/(r-x_new)));
        case 0
            switch s >= y_new
                case 1
                    phi_desired = (pi/2) + ...
                        vpa(atan(abs((s-y_new)/(r-x_new))));
                case 0
                    phi_desired = -(pi/2) - ...
                        vpa(atan(abs((s-y_new)/(r-x_new))));
            end
        end
        error=phi_desired-phi_old;
        % w: angular velocity
       
        w=k*error;
                
        % unicycle model system dynamics
        x_new=x_old + v*cos(phi_old) * t;
        y_new=y_old + v*sin(phi_old) * t;
        phi_new=t*w+phi_old;
        
        pause(t);
                       
          
        plot(x_new,y_new,'*');
        hold on
        x_old=x_new;
        y_old=y_new;
        phi_old=phi_new;
             
        
    end   
end
