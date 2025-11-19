function plot_simulation_outputs(resultats, params)
%PLOT_SIMULATION_OUTPUTS Affiche les indicateurs requis.

% --------- Courbe de couverture ---------
figure('Name','Couverture vs temps');
plot(resultats.time, 100*resultats.coverageHistory, 'LineWidth', 2);
grid on;
xlabel('Temps (s)');
ylabel('Surface couverte (%)');
title('Evolution de la couverture au cours du nettoyage');
ylim([0 100]);
if ~isnan(resultats.timeTo70)
    hold on;
    yline(70, '--r', '70 %');
    xline(resultats.timeTo70, '--k', sprintf('%.1f s', resultats.timeTo70));
end

% --------- Carte et trajectoire ---------
figure('Name','Carte et trajectoire');
map = resultats.map;
gridSize = size(map.visited,1);
xAxis = linspace(-params.roomSize/2, params.roomSize/2, gridSize);
yAxis = linspace(-params.roomSize/2, params.roomSize/2, gridSize);
displayMap = zeros(gridSize);
displayMap(map.free & ~map.visited & ~map.obstacle) = 1;   % libre
% état "visité" prioritaire
visitedMask = map.visited & ~map.obstacle;
displayMap(visitedMask) = 2;
displayMap(map.obstacle) = 3;
imagesc(xAxis, yAxis, displayMap);
axis xy equal tight;
colormap([0.2 0.2 0.2; 0.6 0.8 1.0; 0.3 0.9 0.3; 0.8 0.2 0.2]);
colorbar('Ticks',0:3,'TickLabels',{'Inconnu','Libre','Visité','Obstacle'});
hold on;
plot(resultats.path(:,1), resultats.path(:,2), 'w-', 'LineWidth', 1.5);
theta = linspace(0, 2*pi, 60);
circle = params.obstacleCenter + params.obstacleRadius*[cos(theta); sin(theta)]';
plot(circle(:,1), circle(:,2), '--k', 'LineWidth', 1.2);
plot(resultats.path(1,1), resultats.path(1,2), 'wo', 'MarkerFaceColor','g');
plot(resultats.finalPose.position(1), resultats.finalPose.position(2), 'ws', 'MarkerFaceColor','m');
title('Grille d''occupation, obstacle et trajectoire du robot');
xlabel('x (m)');
ylabel('y (m)');
legend({'Trajectoire','Obstacle','Départ','Arrivée'},'Location','southoutside');
end
