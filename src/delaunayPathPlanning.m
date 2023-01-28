clc
clear

%% 加载锥桶坐标数据
conePosTable = readtable("cone_position_data.csv");
innerIndex = conePosTable.type == 11;
innerConePosition = [conePosTable(innerIndex, "x_coor"), ...
                         conePosTable(innerIndex, "y_coor")];
outerIndex = conePosTable.type == 2;
outerConePosition = [conePosTable(outerIndex, "x_coor"), ...
                         conePosTable(outerIndex, "y_coor")];

%% 数据预处理
[m, nc] = size(innerConePosition); % 内/外侧锥桶位置数据的size
P = zeros(2 * m, nc); % 初始化由内侧坐标与外侧坐标组成的 P 矩阵
% 将内外侧坐标交替合并
P(1:2:2 * m, :) = innerConePosition.Variables;
P(2:2:2 * m, :) = outerConePosition.Variables;

interval = 10; % 每次规划时考虑的路段长度
interpolationNum = 100; % 插值点数量
pathCount = idivide(uint16(2 * m), interval, 'floor') * interpolationNum; % 最终规划路径坐标点个数
xPos = zeros(1, pathCount); % 数值向量 xPos，用于每次迭代后存储规划的 x 坐标
yPos = zeros(1, pathCount); % 数值向量 yPos，用于每次迭代后存储规划的 y 坐标

for i = interval:interval:2 * m
    % 在每次循环中进行一个路段的路径规划，路段区间长度由interval控制
    %% 构建 Delaunay 三角形
    pointIndex = ((abs((i - 1) - interval)):i); % 属于该局部路段的点集的索引
    delaTria = delaunayTriangulation(P(pointIndex, :)); % 为该路段的点集创建 Delaunay 三角剖分
    delaTriaPoints = delaTria.Points; % 顶点坐标
    [mc, ~] = size(delaTriaPoints); % size

    figure(1) % 绘制 Delaunay 三角剖分
    triplot(delaTria, 'k')
    grid on
    ax = gca;
    ax.GridColor = [0, 0, 0]; % [R, G, B]
    xlabel('x(m)')
    ylabel('y (m)')
    set(gca, 'Color', '#EEEEEE')
    title('Delaunay Triangulation')
    hold on

    %% 移除外部三角形
    if rem(interval, 2) == 0
        % 当区间为偶数时的内外侧约束
        cIn = [2, 1; (1:2:mc - 3)', (3:2:(mc))'; (mc - 1), mc];
        cOut = [(2:2:(mc - 2))', (4:2:mc)'];
    else
        % 区间为奇数时的内外侧约束
        cIn = [2, 1; (1:2:mc - 2)', (3:2:(mc))'; (mc - 1), mc];
        cOut = [(2:2:(mc - 2))', (4:2:mc)'];
    end

    C = [cIn; cOut]; % 创建一个连接约束边界的矩阵
    TR = delaunayTriangulation(delaTriaPoints, C); % 带约束的 Delaunay 三角剖分
    TRC = TR.ConnectivityList; % 三角剖分连接矩阵
    TL = isInterior(TR); % 三角形是否在边界内的逻辑值
    TC = TR.ConnectivityList(TL, :); % 三角剖分连接矩阵
    [~, pt] = sort(sum(TC, 2)); % （可选项） The rows of connectivity matrix are arranged in ascending sum of rows...
    % 确保三角形按渐进顺序连接
    TS = TC(pt, :); % 基于行的升序连接矩阵
    sortedTria = triangulation(TS, delaTriaPoints); % 基于已排序的连接矩阵创建的三角剖分

    figure(2) % 绘制去除外部三角形的 Delaunay 三角剖分
    triplot(sortedTria, 'k')
    grid on
    ax = gca;
    ax.GridColor = [0, 0, 0]; % [R, G, B]
    xlabel('x(m)')
    ylabel('y (m)')
    set(gca, 'Color', '#EEEEEE')
    title('Delaunay Triangulation without Outliers')
    hold on

    %% 寻找内边中心点
    xPoints = sortedTria.Points(:, 1);
    yPoints = sortedTria.Points(:, 2);

    E = edges(sortedTria); % 三角剖分边缘
    isEven = rem(E, 2) == 0; % 忽略边界边缘
    Eeven = E(any(isEven, 2), :);
    isOdd = rem(Eeven, 2) ~= 0;
    Eodd = Eeven(any(isOdd, 2), :);

    xMidpoints = ((xPoints((Eodd(:, 1))) + xPoints((Eodd(:, 2)))) / 2); % x 中点坐标
    yMidpoints = ((yPoints((Eodd(:, 1))) + yPoints((Eodd(:, 2)))) / 2); % y 中点坐标
    PMidpoints = [xMidpoints, yMidpoints]; % 中点坐标

    %% 中心点插值
    distancematrix = squareform(pdist(PMidpoints));
    distancesteps = zeros(length(PMidpoints) - 1, 1);

    for j = 2:length(PMidpoints)
        distancesteps(j - 1, 1) = distancematrix(j, j - 1);
    end

    totalDistance = sum(distancesteps); % 总经过距离
    distbp = cumsum([0; distancesteps]); % 每个路径点间的距离
    gradbp = linspace(0, totalDistance, interpolationNum); % 线性间距向量
    xq = interp1(distbp, xMidpoints, gradbp, 'spline'); % 插值 x 坐标
    yq = interp1(distbp, yMidpoints, gradbp, 'spline'); % 插值 y 坐标

    startCount = (i / interval - 1) * interpolationNum + 1;
    endCount = i / interval * interpolationNum;
    xPos(:, startCount:endCount) = xq; % 在每次迭代后存储获得的 x 中点
    yPos(:, startCount:endCount) = yq; % 在每次迭代后存储获得的 y 中点

    figure(3) % 动态绘制路径规划
    % 用于展示规划路径全貌的子图
    pos1 = [0.1, 0.15, 0.5, 0.7];
    subplot('Position', pos1)
    pathPlanPlot(innerConePosition, outerConePosition, ...
        P, delaTria, sortedTria, xMidpoints, yMidpoints, cIn, cOut, xq, yq)
    title(['Path planning based on constrained Delaunay' ...
               newline ' triangulation'])

    % 用于展示规划路径局部细节的子图
    pos2 = [0.7, 0.15, 0.25, 0.7];
    subplot('Position', pos2)
    pathPlanPlot(innerConePosition, outerConePosition, ...
        P, delaTria, sortedTria, xMidpoints, yMidpoints, cIn, cOut, xq, yq)
    xlim([min(min(xPoints(1:2:(mc - 1)), xPoints(2:2:mc))) ...
              max(max(xPoints(1:2:(mc - 1)), xPoints(2:2:mc)))])
    ylim([min(min(yPoints(1:2:(mc - 1)), yPoints(2:2:mc))) ...
              max(max(yPoints(1:2:(mc - 1)), yPoints(2:2:mc)))])

end

% figure(3)的图例
h = legend('blueCone', 'redCone', 'start', 'midpoint', 'internal edges', ...
'inner boundary', 'outer boundary', 'planned path');

plannedPath = [xPos', yPos']; % 组合出最终规划路径
