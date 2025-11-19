function resultats = run_cleaning_simulation(params)
%RUN_CLEANING_SIMULATION Boucle principale pour le robot aspirateur.
%   resultats = RUN_CLEANING_SIMULATION(params) renvoie les trajectoires,
%   cartes et indicateurs obtenus pour la configuration "params".
%
%   La fonction implémente :
%     - Modèle cinématique unicycle
%     - Moteurs DC indépendants avec régulateurs PI
%     - Capteur ultrason mono-directionnel
%     - Cartographie par grille et calcul de couverture
%     - Stratégie de contrôle hybride (FSM)

rng('shuffle');

nSteps = floor(params.simTime/params.dt) + 1;
gridSize = round(params.roomSize / params.cellSize);

map.visited = false(gridSize);
map.free = false(gridSize);
map.obstacle = false(gridSize);
map.obstacle = mark_static_obstacle(map.obstacle, params);

freeCells = sum(~map.obstacle, 'all');

[position, theta] = random_initial_pose(params);

state.name = 'FORWARD_SCAN';
state.timer = 0;
state.sweepDir = 1;
state.targetY = position(2);

motorLeft = struct('omega',0,'current',0,'integrator',0);
motorRight = struct('omega',0,'current',0,'integrator',0);

path = zeros(nSteps, 2);
coverageHistory = zeros(nSteps,1);
time = (0:nSteps-1)' * params.dt;
sensorHistory = zeros(nSteps,1);
stateHistory = strings(nSteps,1);

coverage70 = NaN;

for k = 1:nSteps
    robot.position = position;
    robot.theta = theta;

    % Mesure ultrason
    sensorDist = ultrasonic_distance(robot.position, robot.theta, params);
    sensorHistory(k) = sensorDist;

    % Mise à jour de la carte et de la couverture
    map = update_occupancy(map, robot, sensorDist, params);
    visitedCells = sum(map.visited & ~map.obstacle, 'all');
    coverage = visitedCells / freeCells;
    coverageHistory(k) = coverage;
    if isnan(coverage70) && coverage >= params.coverageTarget
        coverage70 = time(k);
    end

    % Commande hybride
    [state, vCmd, wCmd] = state_machine_controller(state, robot, sensorDist, coverage, map, params);
    stateHistory(k) = state.name;

    % Conversion vers vitesses roue
    vLeftRef = vCmd - 0.5*params.wheelBase*wCmd;
    vRightRef = vCmd + 0.5*params.wheelBase*wCmd;
    omegaLeftRef = vLeftRef / params.wheelRadius;
    omegaRightRef = vRightRef / params.wheelRadius;

    [motorLeft, omegaLeft] = dc_motor_step(motorLeft, omegaLeftRef, params.leftMotor, params.speedController, params.dt);
    [motorRight, omegaRight] = dc_motor_step(motorRight, omegaRightRef, params.rightMotor, params.speedController, params.dt);

    vLeft = omegaLeft * params.wheelRadius;
    vRight = omegaRight * params.wheelRadius;
    v = 0.5 * (vLeft + vRight);
    w = (vRight - vLeft) / params.wheelBase;

    % Intégration cinématique
    theta = wrapToPi(theta + params.dt * w);
    position = position + params.dt * v * [cos(theta); sin(theta)];
    [position, theta] = apply_collision_constraints(position, theta, params);

    path(k,:) = position';
    state.timer = state.timer + params.dt;
end

resultats.time = time;
resultats.coverageHistory = coverageHistory;
resultats.sensorHistory = sensorHistory;
resultats.path = path;
resultats.map = map;
resultats.stateHistory = stateHistory;
resultats.timeTo70 = coverage70;
resultats.finalPose = struct('position', position, 'theta', theta);

end

% -------------------------------------------------------------------------
function [state, vCmd, wCmd] = state_machine_controller(state, robot, sensorDist, coverage, map, params)
%STATE_MACHINE_CONTROLLER Implémente la logique de la FSM hybride.

switch state.name
    case 'FORWARD_SCAN'
        desiredHeading = select_heading(robot, map, params);
        wCmd = heading_controller(desiredHeading, robot.theta);
        speedFactor = max(0.2, 1 - abs(wCmd)/params.maxAngular);
        vCmd = params.maxSpeed * speedFactor;
        if sensorDist < params.wallThreshold
            state.name = 'WALL_FOLLOW';
            state.timer = 0;
        elseif coverage > params.sweepThreshold
            state.name = 'SWEEP';
            state.timer = 0;
            state.sweepDir = sign(cos(robot.theta));
            if state.sweepDir == 0, state.sweepDir = 1; end
            state.targetY = robot.position(2);
        end
    case 'WALL_FOLLOW'
        err = params.wallThreshold - sensorDist;
        wCmd = -1.5 * err;
        vCmd = 0.15 + 0.25 * max(0, min(sensorDist, 1.0));
        if sensorDist < 0.35
            wCmd = 1.0;
        end
        if sensorDist > 1.2 || state.timer > params.wallFollowDuration
            state.name = 'FORWARD_SCAN';
            state.timer = 0;
        end
    case 'SWEEP'
        margin = params.roomSize/2 - params.robotRadius - 0.2;
        if ~isfield(state,'sweepDir') || state.sweepDir == 0
            state.sweepDir = 1;
        end
        if ~isfield(state,'targetY')
            state.targetY = robot.position(2);
        end
        targetX = state.sweepDir * margin;
        desiredPoint = [targetX, state.targetY];
        vec = desiredPoint - robot.position;
        desiredHeading = atan2(vec(2), vec(1));
        wCmd = heading_controller(desiredHeading, robot.theta);
        vCmd = 0.9 * params.maxSpeed;
        if abs(robot.position(1) - targetX) < 0.2
            newY = state.targetY + params.sweepSpacing;
            if abs(newY) > margin
                state.name = 'FORWARD_SCAN';
                state.timer = 0;
            else
                state.targetY = newY;
                state.sweepDir = -state.sweepDir;
            end
        end
        if sensorDist < 0.35
            state.name = 'OBSTACLE_HANDLING';
            state.timer = 0;
            state.recovery = 'SWEEP';
        end
    case 'OBSTACLE_HANDLING'
        wCmd = 0.8;
        vCmd = 0.05;
        if state.timer > params.obstacleHandlingTime && sensorDist > 0.8
            if isfield(state,'recovery')
                state.name = state.recovery;
            else
                state.name = 'FORWARD_SCAN';
            end
            state.timer = 0;
        end
    otherwise
        vCmd = 0;
        wCmd = 0;
        state.name = 'FORWARD_SCAN';
        state.timer = 0;
end

vCmd = max(0, min(vCmd, params.maxSpeed));
wCmd = max(-params.maxAngular, min(params.maxAngular, wCmd));

end

% -------------------------------------------------------------------------
function heading = select_heading(robot, map, params)
%SELECT_HEADING Choisit la direction la plus prometteuse.

bestScore = -inf;
heading = robot.theta;
for ang = params.scanAngles
    dir = wrapToPi(robot.theta + ang);
    dist = ultrasonic_distance(robot.position, dir, params);
    unvisited = count_unvisited_cells(robot.position, dir, map, params);
    score = params.weightDistance * (dist / params.sensorRange) + ...
            params.weightUnvisited * unvisited;
    if score > bestScore
        bestScore = score;
        heading = dir;
    end
end
end

% -------------------------------------------------------------------------
function ratio = count_unvisited_cells(position, direction, map, params)
%COUNT_UNVISITED_CELLS Estime la proportion de cellules non visitées sur le rayon.

maxDist = params.sensorRange;
nbSamples = 12;
count = 0;
for k = 1:nbSamples
    d = (k/nbSamples) * maxDist;
    point = position + d * [cos(direction); sin(direction)];
    [ri, rj, valid] = world_to_cell(point, params);
    if valid && ~map.obstacle(ri, rj) && ~map.visited(ri, rj)
        count = count + 1;
    end
end
ratio = count / nbSamples;
end

% -------------------------------------------------------------------------
function w = heading_controller(desired, current)
%HEADING_CONTROLLER Boucle proportionnelle simple.
err = wrapToPi(desired - current);
w = 1.5 * err;
end

% -------------------------------------------------------------------------
function map = update_occupancy(map, robot, sensorDist, params)
%UPDATE_OCCUPANCY Met à jour la grille (visité / libre / obstacle).

[ci, cj, valid] = world_to_cell(robot.position, params);
if valid
    map.visited(ci, cj) = true;
end

rayLength = min(sensorDist, params.sensorRange);
nbPoints = max(1, ceil(rayLength / params.cellSize));
for n = 1:nbPoints
    d = (n - 0.5) * params.cellSize;
    if d >= rayLength
        break;
    end
    point = robot.position + d * [cos(robot.theta); sin(robot.theta)];
    [ri, rj, ok] = world_to_cell(point, params);
    if ok && ~map.obstacle(ri, rj)
        map.free(ri, rj) = true;
    end
end

if sensorDist < params.sensorRange
    impact = robot.position + sensorDist * [cos(robot.theta); sin(robot.theta)];
    [ri, rj, ok] = world_to_cell(impact, params);
    if ok
        map.obstacle(ri, rj) = true;
    end
end
end

% -------------------------------------------------------------------------
function [motorState, omega] = dc_motor_step(motorState, omegaRef, motorParams, ctrlParams, dt)
%DC_MOTOR_STEP Modèle d'actionneur avec régulateur PI.

error = omegaRef - motorState.omega;
motorState.integrator = motorState.integrator + error * dt;
control = ctrlParams.Kp * error + ctrlParams.Ki * motorState.integrator;
control = max(-ctrlParams.uMax, min(ctrlParams.uMax, control));

domega = (motorParams.Km * motorState.current - motorParams.f * motorState.omega) / motorParams.J;
dcurrent = (control - motorParams.Ke * motorState.omega - motorParams.R * motorState.current) / motorParams.L;

motorState.omega = motorState.omega + dt * domega;
motorState.current = motorState.current + dt * dcurrent;
omega = motorState.omega;
end

% -------------------------------------------------------------------------
function [position, theta] = apply_collision_constraints(position, theta, params)
%APPLY_COLLISION_CONSTRAINTS Maintien le robot dans la pièce et hors de l'obstacle.

margin = params.roomSize/2 - params.robotRadius;
if position(1) > margin
    position(1) = margin;
    theta = pi - theta;
elseif position(1) < -margin
    position(1) = -margin;
    theta = pi - theta;
end
if position(2) > margin
    position(2) = margin;
    theta = -theta;
elseif position(2) < -margin
    position(2) = -margin;
    theta = -theta;
end

diff = position - params.obstacleCenter';
dist = norm(diff);
minDist = params.robotRadius + params.obstacleRadius;
if dist < minDist
    if dist == 0
        dir = [1;0];
    else
        dir = diff / dist;
    end
    position = params.obstacleCenter' + dir * minDist;
    theta = atan2(dir(2), dir(1));
end
end

% -------------------------------------------------------------------------
function dist = ultrasonic_distance(position, direction, params)
%ULTRASONIC_DISTANCE Calcule la distance jusqu'au premier obstacle (mur ou cylindre).

% Distance aux murs
roomHalf = params.roomSize/2;
rayDir = [cos(direction); sin(direction)];

dists = [];
if abs(rayDir(1)) > 1e-6
    tx1 = (roomHalf - position(1)) / rayDir(1);
    tx2 = (-roomHalf - position(1)) / rayDir(1);
    dists = [dists, tx1, tx2];
end
if abs(rayDir(2)) > 1e-6
    ty1 = (roomHalf - position(2)) / rayDir(2);
    ty2 = (-roomHalf - position(2)) / rayDir(2);
    dists = [dists, ty1, ty2];
end

% Distance à l'obstacle circulaire
rel = position - params.obstacleCenter';
a = dot(rayDir, rayDir);
b = 2 * dot(rayDir, rel);
c = dot(rel, rel) - (params.obstacleRadius)^2;
disc = b^2 - 4*a*c;
if disc >= 0
    t1 = (-b - sqrt(disc)) / (2*a);
    t2 = (-b + sqrt(disc)) / (2*a);
    dists = [dists, t1, t2];
end

positive = dists(dists > 0);
if isempty(positive)
    dist = params.sensorRange;
else
    dist = min([positive, params.sensorRange]);
end

end

% -------------------------------------------------------------------------
function [ci, cj, valid] = world_to_cell(point, params)
%WORLD_TO_CELL Convertit des coordonnées mètres vers indices de grille.

gridSize = round(params.roomSize / params.cellSize);
col = floor((point(1) + params.roomSize/2) / params.cellSize) + 1;
row = floor((point(2) + params.roomSize/2) / params.cellSize) + 1;
valid = row >= 1 && row <= gridSize && col >= 1 && col <= gridSize;
if valid
    ci = row;
    cj = col;
else
    ci = 1;
    cj = 1;
end
end

% -------------------------------------------------------------------------
function [position, theta] = random_initial_pose(params)
%RANDOM_INITIAL_POSE Tire une pose valide à l'intérieur de la pièce.
margin = params.roomSize/2 - params.robotRadius - 0.5;
valid = false;
while ~valid
    position = [-margin + 2*margin*rand; -margin + 2*margin*rand];
    diff = position - params.obstacleCenter';
    valid = norm(diff) > (params.obstacleRadius + params.robotRadius + 0.1);
end
theta = wrapToPi(2*pi*rand - pi);
end

% -------------------------------------------------------------------------
function obstacleMask = mark_static_obstacle(mask, params)
%MARK_STATIC_OBSTACLE Inscrit l'obstacle cylindrique dans la grille.

gridSize = size(mask,1);
for i = 1:gridSize
    for j = 1:gridSize
        x = (j-0.5)*params.cellSize - params.roomSize/2;
        y = (i-0.5)*params.cellSize - params.roomSize/2;
        if hypot(x - params.obstacleCenter(1), y - params.obstacleCenter(2)) <= params.obstacleRadius
            mask(i,j) = true;
        end
    end
end
obstacleMask = mask;
end
