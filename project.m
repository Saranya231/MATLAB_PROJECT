% 🕹️ Random Walk Maze Game with Moving Obstacles + Randomized Path
clear; clc; close all;

%% Parameters
boundary = 20;      % Game boundary
numObstacles = 5;   % Number of moving obstacles
numWalls = 5;       % Number of random walls

% Create figure
fig = figure('KeyPressFcn',@keyPress, 'CloseRequestFcn',@closeGame);
hold on;
axis equal;
xlim([-boundary boundary]);
ylim([-boundary boundary]);
grid on;
title('🎮 Escape the Maze: Dodge the Red Blocks! 🟩➡️🏁');
xlabel('X Position'); ylabel('Y Position');

% Draw boundary
rectangle('Position',[-boundary -boundary 2*boundary 2*boundary],...
          'EdgeColor','r','LineWidth',2);

%% Random Maze Walls
walls = zeros(numWalls,4); % [x y width height]
for i=1:numWalls
    w = randi([3 6]);      % random width
    h = randi([3 6]);      % random height
    x = randi([-boundary+2 boundary-8]);
    y = randi([-boundary+2 boundary-8]);
    walls(i,:) = [x y w h];
    rectangle('Position',walls(i,:),'FaceColor',[0.2 0.2 0.2]); % gray wall
end

%% Random Start & Target Positions (not inside walls)
valid = false;
while ~valid
    startPos = [randi([-boundary+2 boundary-2]), randi([-boundary+2 boundary-2])];
    if ~hitsWall(startPos(1),startPos(2),walls)
        valid = true;
    end
end

valid = false;
while ~valid
    targetPos = [randi([-boundary+2 boundary-2]), randi([-boundary+2 boundary-2])];
    if ~hitsWall(targetPos(1),targetPos(2),walls) && ~isequal(targetPos,startPos)
        valid = true;
    end
end

%% Initial Player
state.x = startPos(1); 
state.y = startPos(2);
state.boundary = boundary;
state.alive = true;
state.won = false;
state.walls = walls;

% Plot player
state.player = plot(state.x,state.y,'ko','MarkerFaceColor','g','MarkerSize',10);

%% Target
state.target = targetPos;
plot(state.target(1),state.target(2),'p','MarkerSize',15,'MarkerFaceColor','y'); % 🏁 target

%% Obstacles with velocities
state.obstacles = randi([-boundary+5 boundary-5],numObstacles,2);
state.velocities = randi([-1 1],numObstacles,2);   % random directions
state.velocities(state.velocities==0) = 1;         % avoid standing still
state.obstaclePlots = plot(state.obstacles(:,1),state.obstacles(:,2),'rs','MarkerFaceColor','r','MarkerSize',10);

% Save state
guidata(fig,state);

%% --- Timer for Moving Obstacles ---
t = timer('ExecutionMode','fixedRate','Period',0.2,'TimerFcn',{@moveObstacles,fig});
start(t);

%% --- Player Movement Callback ---
function keyPress(src,event)
    state = guidata(src);
    if isempty(state) || ~isstruct(state), return; end
    if ~state.alive || state.won, return; end
    
    % Candidate move
    newX = state.x; newY = state.y;
    switch event.Key
        case 'rightarrow', newX = state.x + 1;
        case 'leftarrow',  newX = state.x - 1;
        case 'uparrow',    newY = state.y + 1;
        case 'downarrow',  newY = state.y - 1;
        otherwise, return;
    end
    
    % Check collision with walls
    if ~hitsWall(newX,newY,state.walls)
        state.x = newX; state.y = newY;
        set(state.player,'XData',state.x,'YData',state.y);
        plot(state.x,state.y,'b.');  % Trail
    end
    
    % Check Game Status
    state = checkGameStatus(state,src);
    guidata(src,state);
end

%% --- Obstacle Movement Function (bouncing across area) ---
function moveObstacles(~,~,fig)
    if ~ishandle(fig), return; end
    state = guidata(fig);
    if isempty(state) || ~isstruct(state), return; end
    if ~state.alive || state.won, return; end
    
    % Update positions
    state.obstacles = state.obstacles + state.velocities;
    
    % Bounce off boundary
    for i = 1:size(state.obstacles,1)
        if abs(state.obstacles(i,1)) >= state.boundary
            state.velocities(i,1) = -state.velocities(i,1);
        end
        if abs(state.obstacles(i,2)) >= state.boundary
            state.velocities(i,2) = -state.velocities(i,2);
        end
    end
    
    % Update plot
    set(state.obstaclePlots,'XData',state.obstacles(:,1),'YData',state.obstacles(:,2));
    
    % Check status
    state = checkGameStatus(state,fig);
    guidata(fig,state);
end

%% --- Check Wall Collision ---
function hit = hitsWall(x,y,walls)
    hit = false;
    for i=1:size(walls,1)
        wx = walls(i,1); wy = walls(i,2);
        ww = walls(i,3); wh = walls(i,4);
        if x >= wx && x <= wx+ww && y >= wy && y <= wy+wh
            hit = true; return;
        end
    end
end

%% --- Game Status Check ---
function state = checkGameStatus(state,fig)
    % Out of bounds
    if abs(state.x) > state.boundary || abs(state.y) > state.boundary
        text(state.x,state.y,'🚨 Out of Bounds!','Color','r','FontSize',14,'FontWeight','bold');
        disp('Game Over - Out of Bounds!');
        state.alive = false;
    end
    
    % Collision with obstacle
    if any(ismember(state.obstacles,[state.x state.y],'rows'))
        text(state.x,state.y,'💥 Hit Obstacle!','Color','r','FontSize',14,'FontWeight','bold');
        disp('Game Over - Hit an Obstacle!');
        state.alive = false;
    end
    
    % Reached target
    if isequal([state.x state.y],state.target)
        text(state.x,state.y,'🎉 You Win! 🏁','Color','g','FontSize',14,'FontWeight','bold');
        disp('Congratulations! You reached the target 🏁');
        state.won = true;
    end
end

%% --- Clean Up on Close ---
function closeGame(fig,~)
    timers = timerfindall;
    if ~isempty(timers)
        stop(timers);
        delete(timers);
    end
    delete(fig);
end
