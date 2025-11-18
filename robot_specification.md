# Autonomous Vacuum Cleaner Robot – System Specification (Optimized)

## 1. Objective
Design, model, and simulate an autonomous vacuum cleaning robot. Goal: minimize time to reach **70% surface coverage** in a **20×20 m** room using:
- Perfect odometry
- Single forward ultrasonic sensor
- Differential drive (2 DC motors)
- Mapping + hybrid coverage strategy

## 2. Environment
- Room: 20×20 m, center (0,0)
- Borders: rigid walls
- Obstacle: cylinder, center (3,3), radius 0.25 m
- Initial pose: random, inside room, not overlapping obstacle

## 3. Robot
### 3.1 Geometry & Motion
- Circular body, radius 0.20 m
- Differential drive: two driven rear wheels + front caster
- Wheel radius R = 0.05 m
- Wheelbase L ≈ 0.30–0.40 m
- Max linear speed: 0.3 m/s

### 3.2 Sensors
**Ultrasonic (forward)**  
- Ray-based distance to nearest obstacle  
- No lateral sensing  

**Odometry**  
- Perfect (X, Y, θ)  

## 4. Actuation & Motor Models
### 4.1 Kinematic Conversion
v, ω → wheel speeds:

```
v_g = v - (L/2)*ω
v_d = v + (L/2)*ω

ω_g = v_g / R
ω_d = v_d / R
```

### 4.2 DC Motor Parameters
Left motor:
- J=1, f=1, R=1, L=1, Km=1, Ke=1  

Right motor:
- J=1, f=1, R=1, L=1, Km=1, Ke=1.2  

Each wheel under independent PI speed control.

## 5. Mapping
Occupancy grid:
- Resolution 0.5 m → 40×40 cells  
- States: occupied, free, visited

Ultrasonic ray-cast updates free/occupied. Robot cell marked visited each step.

Coverage:
```
Coverage(t) = visited_cells / total_free_cells
```

## 6. Control Strategy (FSM)
Hybrid strategy: exploration + obstacle handling + optimized sweep.

### STATES
#### FORWARD_SCAN
- Rotate to scan ±60°
- Score(θ) = w1*d(θ) + w2*unvisited_cells(θ)
- Move toward max-score direction (fast)
- → WALL_FOLLOW if obstacle ahead

#### WALL_FOLLOW
- Slight left turns to keep obstacle on right
- Loop/time limited
- Map updated
- → FORWARD_SCAN or SWEEP depending on map state

#### SWEEP (zig-zag)
- Coverage-optimized sweeping based on grid
- Straight fast motion, lateral shifts ≈ robot diameter
- → OBSTACLE_HANDLING if unexpected obstacle

#### OBSTACLE_HANDLING
- Local contouring + map update
- → Back to SWEEP

## 7. State Diagram (ASCII)

```
        +-------------------+
        |   FORWARD_SCAN   |
        +---------+---------+
                  |
     obstacle     v
           +--------------+
           | WALL_FOLLOW  |
           +------+-------+
                  |
     loop/open    v
           +--------------+
           |    SWEEP     |
           +------+-------+
                  |
      unexpected  v
      obstacle  +-------------------+
                | OBSTACLE_HANDLING |
                +---------+---------+
                          |
                          +--> SWEEP
```

## 8. Simulation Architecture
- FSM controller (Stateflow recommended)
- Unicycle kinematics block
- Left & right DC motor blocks
- PI speed controllers
- Ultrasonic ray model
- Occupancy grid update module
- Coverage computation block
- Map + path visualization

## 9. Required Outputs
- Coverage(t) curve
- Time to reach 70%
- Final coverage map (visited/free/obstacle)
- Full trajectory plot
- Reproducible `.m` launcher + `.slx` model

## 10. Constraints & Assumptions
- Perfect odometry
- No dynamic obstacles
- Ultrasonic is ideal ray
- Robot cannot start outside room or inside obstacle
