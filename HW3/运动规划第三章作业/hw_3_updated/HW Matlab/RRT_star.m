%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clc
clear all; close all;
x_I=1; y_I=1;           % 设置初始点
x_G=700; y_G=700;       % 设置目标点（可尝试修改终点）
Thr=50;                 % 设置目标点阈值
Delta= 30;              % 设置扩展步长
radius = 50; % Define the radius for the neighborhood search
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % 从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树，作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%地图x轴长度
yL=size(Imp,1);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
bFind = false;

for iter = 1:30000
    % x_rand=[];
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    x_rand = [randi([1, xL]), randi([1, yL])];
    % x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    dist=[];
    for j=1:length(T.v)
        tmp=norm([T.v(j).x - x_rand(1), T.v(j).y - x_rand(2)]);
        dist=[dist tmp];
    end
    [val, idx] = min(dist);
    x_near.x = T.v(idx).x;
    x_near.y = T.v(idx).y;
    % x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    theta = atan2(x_rand(2)-x_near.y, x_rand(1)-x_near.x);
    x_new.x = x_near.x + Delta*cos(theta);
    x_new.y = x_near.y + Delta*sin(theta);
    %检查节点是否是collision-free
    if ~collisionChecking([x_near.x,x_near.y],[x_new.x,x_new.y],Imp) 

         continue;
    end
     X_near = [];
    for j=1:length(T.v)
        if norm([T.v(j).x - x_new.x, T.v(j).y - x_new.y]) <= radius
            X_near = [X_near, j];
        end
    end
    
    if isempty(X_near)
        x_new.indPrev = idx;
        x_new.cost = val + Delta;
    else
        min_cost = inf;
        for j = 1:length(X_near)
            cost = T.v(X_near(j)).dist + norm([T.v(X_near(j)).x - x_new.x, T.v(X_near(j)).y - x_new.y]);
            if cost < min_cost
                min_cost = cost;
                x_new.indPrev = X_near(j);
                x_new.cost = cost;
            end
        end
    end
    count=count+1;
    %Step 4: 将x_new插入树T 
    %提示：新节点x_new的父节点是x_near
    x_new.indPrev = idx;
    T.v(count).x = x_new.x;
    T.v(count).y = x_new.y;
    T.v(count).dist = x_new.cost;
    % The distance from the root to the new node
    T.v(count).indPrev = x_new.indPrev;  % <-- added this line
    %Step 5:检查是否到达目标点附近 
    for j = 1:length(X_near)
        cost = T.v(count).dist + norm([T.v(count).x - T.v(X_near(j)).x, T.v(count).y - T.v(X_near(j)).y]);
        if cost < T.v(X_near(j)).dist
            T.v(X_near(j)).indPrev = count;
            T.v(X_near(j)).dist = cost;
        end
    end
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    % disp('xxxx')
    if norm([x_new.x - x_G, x_new.y - y_G]) <= Thr
        % disp('+++++')
        bFind = true;
        break;
    end
    %Step 6:将x_near和x_new之间的路径画出来
    %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
    %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
    plot([x_near.x, x_new.x], [x_near.y, x_new.y], 'b'); hold on;
    pause(0.05); %暂停一会，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if bFind
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
