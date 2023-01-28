% 动态绘图函数
function [] = pathPlanPlot(innerConePosition, outerConePosition, P, DT, TO, xmp, ymp, cIn, cOut, xq, yq)

    plot(innerConePosition.x_coor, innerConePosition.y_coor, '.b', 'MarkerFaceColor', 'b')
    hold on
    plot(outerConePosition.x_coor, outerConePosition.y_coor, '.r', 'MarkerFaceColor', 'r')
    plot(P(1, 1), P(1, 2), '|', 'MarkerEdgeColor', '#F57F4B', 'MarkerSize', 15, 'LineWidth', 5)
    grid on
    ax = gca;
    ax.GridColor = [0, 0, 0]; % [R, G, B]
    xlabel('x(m)')
    ylabel('y (m)')
    set(gca, 'Color', '#EEEEEE')

    hold on
    plot(xmp, ymp, '*k')
    drawnow

    hold on
    triplot(TO, 'Color', '#00AAAA') % 三角网格
    drawnow

    hold on
    plot(DT.Points(cOut', 1), DT.Points(cOut', 2), ...
        'Color', '#E54E5D', 'LineWidth', 2) % 左侧红锥桶边界线
    plot(DT.Points(cIn', 1), DT.Points(cIn', 2), ...
        'Color', '#037CD2', 'LineWidth', 2) % 右侧蓝锥桶边界线
    drawnow

    hold on
    plot(xq, yq, 'Color', '#927FD3', 'LineWidth', 2.5) % 期望路径
    drawnow

end
