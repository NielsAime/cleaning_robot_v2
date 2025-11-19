function params = default_simulation_params()
%DEFAULT_SIMULATION_PARAMS Crée la configuration nominale de la simulation.
%
% Cette structure rassemble toutes les constantes physiques, moteurs et
% options de contrôle utilisées par run_cleaning_simulation.

% ------------------ Environnement ------------------
params.roomSize = 20.0;          % salle carrée (m)
params.cellSize = 0.5;           % résolution de la grille d'occupation (m)
params.obstacleCenter = [3, 3];
params.obstacleRadius = 0.25;    % cylindre fixé
params.coverageTarget = 0.70;    % pourcentage recherché
params.simTime = 600;            % durée max (s)
params.dt = 0.1;                 % pas d'intégration (s)
params.sensorRange = 4.0;        % portée du capteur ultrason (m)
params.wallThreshold = 0.6;      % distance déclenchant le suivi de mur
params.wallFollowDuration = 8.0; % temps minimal dans l'état mur (s)
params.sweepThreshold = 0.45;    % couverture déclenchant les balayages
params.sweepSpacing = 0.45;      % écart entre bandes (m)
params.obstacleHandlingTime = 4; % temps pour la manoeuvre locale (s)

% ------------------ Robot --------------------------
params.robotRadius = 0.20;
params.wheelRadius = 0.05;
params.wheelBase = 0.35;
params.maxSpeed = 0.30;          % vitesse linéaire max (m/s)
params.maxAngular = 1.2;         % limite de vitesse angulaire (rad/s)

% ------------------ Pondérations du scan ------------
params.scanAngles = linspace(-pi/3, pi/3, 7);
params.weightDistance = 0.7;
params.weightUnvisited = 0.3;

% ------------------ Moteurs DC ----------------------
params.leftMotor = struct('J',1,'f',1,'R',1,'L',1,'Km',1,'Ke',1);
params.rightMotor = struct('J',1,'f',1,'R',1,'L',1,'Km',1,'Ke',1.2);
params.speedController = struct('Kp',5,'Ki',25,'uMax',12);

end
