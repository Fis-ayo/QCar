# System Architecture

Report-ready image: `docs/system_architecture.png`  
Standalone Mermaid source: `docs/system_architecture.mmd`

## Deployment view

```mermaid
flowchart LR
    subgraph PC[Local Windows computer — 192.168.0.153]
        L[run_all_cars.bat]
        S[python/start.py]
        C[Intersection coordinator\nUDP 18750]
        O[Observer displays]
        CL[Coordinator log]
        L --> S
        S --> C
        C --> CL
        S --> O
    end

    subgraph A[QCar .151]
        VA[vehicle_control.py]
        YA[yolo_server.py]
        CA[lane_camera_server.py\nmeasurement only]
        SA[Sensors and actuators]
    end

    subgraph B[QCar .152]
        VB[vehicle_control.py]
        YB[yolo_server.py]
        SB[Sensors and actuators]
    end

    S -->|SSH/SCP deployment and launch| A
    S -->|SSH/SCP deployment and launch| B
    VA <-->|vehicle state / grant| C
    VB <-->|vehicle state / grant| C
    YA -->|detections| VA
    YB -->|detections| VB
    CA -.->|diagnostic stream only| O
    YA -.->|probe image| O
    VA <--> SA
    VB <--> SB
```

The coordinator runs automatically on the local computer. It does not send
steering commands. It grants or denies permission to occupy a configured
conflict zone; each QCar retains responsibility for stopping and tracking its
route.

## Vehicle control and information flow

```mermaid
flowchart TD
    RN[road_network.json] --> RR[RoadNetwork validation]
    RR --> RP[Fixed route selection]
    RP --> GEN[SDCS path generation]
    GEN --> PM[Path validation and conservative repair]

    GPS[GPS / LiDAR scan matching] --> EKF[QCar EKF]
    IMU[Gyroscope] --> EKF
    ODO[Motor tachometer and steering] --> EKF
    EKF --> ST[Estimated vehicle state]

    PM --> SC[Stanley-style steering controller]
    ST --> SC
    ST --> IC[Intersection state machine]
    RN --> IC
    COORD[Central coordinator] <-->|UDP state and grant| IC

    YOLO[YOLO detections and depth] --> SAFE[Obstacle speed restriction]
    IC --> SPEED[Speed permission / stop request]
    SAFE --> SPEED
    ST --> SPEED

    SC --> CMD[Steering command]
    SPEED --> CMD
    CMD --> HW[QCar actuators]

    CSI[Front CSI detector] -. measurement only .-> DIAG[Lane diagnostics]
    ST --> DIAG
    PM --> DIAG
```

## CZ1 decision sequence

```mermaid
sequenceDiagram
    participant A as QCar A
    participant C as Coordinator
    participant B as QCar B

    A->>C: approaching CZ1
    B->>C: approaching CZ1
    C-->>A: grant
    C-->>B: wait
    A->>C: occupying
    B->>B: remain stopped at stop line
    A->>C: exited / release
    C-->>B: grant
    B->>C: occupying
    B->>C: exited / release
```

If the current owner becomes stale, the coordinator locks the zone instead of
granting it to another vehicle. If that same owner resumes communication, its
ownership can be restored without creating simultaneous ownership.

## Safety authority

Speed may be reduced to zero by either the intersection state machine or YOLO
obstacle logic. A reservation grant is permission to enter the conflict zone,
not permission to ignore an observed obstacle. The front CSI lane detector is
currently diagnostic and has no control authority.

## Staged hybrid lane safety

```mermaid
flowchart LR
    MAP[Waypoint route and EKF pose] --> BASE[Baseline path controller]
    CSI[Front CSI yellow-boundary measurement] --> CONF[Confidence and debounce]
    CONF --> WARN[Current: warning and log only]
    CONF --> LIMIT[Candidate 5: earlier slowdown / latched safety stop]
    BASE --> ACT[Steering and nominal speed]
    LIMIT -. speed restriction only .-> ACT
    YOLO[YOLO obstacle restriction] --> ACT
    CZ[Intersection permission] --> ACT
```

Waypoint navigation remains responsible for route selection. Candidate 5 is
enabled explicitly for `.151` in the launcher and awaits physical validation.
The camera safety layer cannot choose a route, grant an intersection movement,
or override YOLO/CZ1 stopping. Camera steering fusion is intentionally
excluded from this stage.
