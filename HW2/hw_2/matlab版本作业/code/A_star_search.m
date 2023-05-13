function [path,OPEN,CLOSED] = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);


    parents_node=[xStart yStart];
    target_node=[xTarget yTarget];
    %initial open list
    %|X val |Y val ||h(n) |g(n)|f(n)|
    OPEN_COUNT=1;
    goal_distance=distance(parents_node(1),parents_node(2),xTarget,yTarget);
    path_cost=0;
    hn=goal_distance;%optimal estimate cost from n to the target, using manhattan
    gn=path_cost;%real cost from initial to n, using 1 or sqrt2
    fn=hn+gn;%estimate cost
    OPEN(OPEN_COUNT,:)=insert_open(xStart,yStart,xStart,yStart,hn,gn,fn);
    % IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %(1,xval,yval,parent_xval,parent_yval,hn,gn,fn)

    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=OPEN(OPEN_COUNT,2);
    CLOSED(CLOSED_COUNT,2)=OPEN(OPEN_COUNT,3);
    [~,min_idx]=min(OPEN(:,8));
    parents_node=OPEN(min_idx,2:3);
    path = [];
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flag = 1;
while(flag)
    if isempty(OPEN)
        disp("Program failed. No path found.");
        break;
    end
    [~, min_idx] = min(OPEN(:,8));
    parents_node = OPEN(min_idx, 2:3);
    % Find the child nodes
    child_nodes = expand_array(parents_node(1), parents_node(2), gn, xTarget, yTarget, CLOSED, MAX_X, MAX_Y);
    % Check if those child_nodes are in the open list; if not, add them to it
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag, openlist_idx] = ismember(child_node(1:2), OPEN(:,2:3), 'rows');
        if in_flag
            if child_node(:,4) < OPEN(openlist_idx, 7) % If g of child < g of open
                % Update OPEN values
                OPEN(openlist_idx, 4:5) = parents_node;
                OPEN(openlist_idx, 7) = child_node(:, 4);
                OPEN(openlist_idx, 8) = child_node(:, 5);
            end
        else% If not in the open list and closed list, add to the open list
            % OPEN_COUNT = OPEN_COUNT + 1;
            new_row = insert_open(child_node(1), child_node(2), parents_node(1), parents_node(2), child_node(3), child_node(4), child_node(5));
            OPEN(end+1,:) = new_row;
        end
    end
    
    % Move the current node to CLOSED
    CLOSED_COUNT = CLOSED_COUNT + 1;
    CLOSED(CLOSED_COUNT, 1:2) = OPEN(min_idx, 2:3);
    path = [path;OPEN(min_idx, 2:3)];

    % Check if the target node has been reached
    if isequal(parents_node, target_node)
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1:2) = OPEN(min_idx, 2:3);
        flag = 0; % Exit the while loop
    else
        OPEN(min_idx, :) = [];
    end
end


