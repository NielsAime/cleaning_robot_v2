%% MAIN_SIMULATION Lance une simulation de robot aspirateur autonome
%  Script principal à exécuter dans MATLAB/Octave.
%  Il configure les paramètres, lance la simulation et trace les résultats.
%
%  Utilisation :
%    >> main_simulation
%
%  Le script repose sur les fonctions :
%    - default_simulation_params
%    - run_cleaning_simulation
%    - plot_simulation_outputs
%
%  Toutes les valeurs sont exprimées en unités SI (m, s, rad).

clear; close all; clc;

% Paramètres généraux de la simulation
params = default_simulation_params();

% Lancement de la simulation principale
resultats = run_cleaning_simulation(params);

% Affichage des indicateurs clefs
fprintf('Couverture finale : %.1f %%\n', 100*resultats.coverageHistory(end));
if isnan(resultats.timeTo70)
    fprintf('70 %% de couverture non atteint en %.1f s\n', params.simTime);
else
    fprintf('Temps pour atteindre 70 %% : %.1f s\n', resultats.timeTo70);
end

% Visualisation
plot_simulation_outputs(resultats, params);
