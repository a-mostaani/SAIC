function [ps,err,ter] = envir_gc(ps,pa,n,noa,goal_set)
%The function takes the position state and position action of agents and
%gives the ps of agents at the next step as the output

%environment tells if an error in movement has occurred, an error is when
%the agent is right most or left most colomns of gridworld and it decides to
%move to right or left respectively

%environment tells if we are in the terminal state
%initialization
    err=zeros(noa,1);
    ter=0;
    p_coord=zeros(noa,2); 
    a_coord=zeros(noa,2);
    up_flag=zeros(noa,1);
    down_flag=zeros(noa,1);
    right_flag=zeros(noa,1);
    left_flag=zeros(noa,1);
%converting the position into coordinates   
    %extracting y coordinates (in direction of up and down)
    for i=1:noa
        p_coord(i,2)=fix((ps(i)-.05)/n)+1; %-0.05 is only added to keep the
                                            
        
        %checking if the agent is on the upper and lower edges of grid 
        %world and prevent going out!
        if p_coord(i,2)==n
            up_flag(i)=1;
        elseif p_coord(i,2)==1
            down_flag(i)=1;
        end
    end
    
    %extracting x coordinates (in direction of right and left)
    for i=1:noa
        p_coord(i,1)=rem(ps(i),n);
        %correction of coordinates meaning
        %       when rem(ps(i),n)==3 it means that the agents is on the
        %       rightmost edge of the gridworld...
        if p_coord(i,1)==0
            p_coord(i,1)=n;
        end
                
        %checking if the agent is on the left or right edges of grid 
        %world and prevent going out!
        if p_coord(i,1)==n
            right_flag(i)=1;
        elseif p_coord(i,1)==1
            left_flag(i)=1;
        end
    end
    
%converting action into coordinates
    for i=1:noa
        switch pa(i)
            case 1 %move right
                %making sure if agent remains in the grid
                if right_flag(i)==1
                    err(i)=1;
                    a_coord(i,:)=[0,0];
                else
                    a_coord(i,:)=[1,0];
                end
            case 2 %move left
                %making sure if agent remains in the grid
                if left_flag(i)==1
                    err(i)=1;
                    a_coord(i,:)=[0,0];
                else
                    a_coord(i,:)=[-1,0];
                end
            case 3 %move up
                %making sure if agent remains in the grid
                if up_flag(i)==1
                    err(i)=1;
                    a_coord(i,:)=[0,0];
                else
                    a_coord(i,:)=[0,1];
                end
            case 4 %move down
                %making sure if agent remains in the grid
                if down_flag(i)==1
                    err(i)=1;
                    a_coord(i,:)=[0,0];
                else
                    a_coord(i,:)=[0,-1];
                end
            case 5 %stop for one step
                a_coord(i,:)=[0,0];
        end
    end
    

%moving
    %moving (using coordinates)
    for i=1:noa
        p_coord(i,:)=p_coord(i,:)+a_coord(i,:);
    end
    
    %interpreting coordinate position to ps
    for i=1:noa
        ps(i)=p_coord(i,1)+(n*(p_coord(i,2)-1));
        
        %check if terminal state has been achived;
        if ps(i)== goal_set
            ter=ter+1;
        end
    end
    
end

