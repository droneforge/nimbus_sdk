# Droneforge SDK -- App Experience

## Log-In Screen
Firstly is the log-in screen. You can create an account in under a minute. We ask for an account so your agents, runs, and settings follow you across devices and upgrades. It also makes it possible to share agents with the community and pull them back later.

If you already have an account, sign in and you will land in the main workspace.

<img width="2489" height="1457" alt="login" src="https://github.com/user-attachments/assets/e0e32ecc-2cb9-411d-b314-3a9fefa51184" />


## Build Tab
This is where you create agents. The left sidebar is your list of agents. The "+" button adds a new one. Click any agent in the sidebar to load it into the editor. If you want to fly a specific agent later, make sure it is selected here before you deploy.

<img width="2489" height="1457" alt="agent" src="https://github.com/user-attachments/assets/bd67be2e-4a03-41a7-b541-ad67cdf0e48a" />


## Build Buttons
You will see a short set of action buttons in the build view. Here's their functionality:
- Add node: adds a new building block to your agent.
- Vision modes: this is specifically where you can select AI models to run real time with your agent. It turns on and off the inference of specific models.
- Download: grab your agent file so you can share it or back it up.
- Save: writes your latest changes. Make a habit of saving before you switch tabs.

## Expressions
Expressions are the way you tell the agent what to do. Think of them like short, clear instructions that connect inputs (like sensors) to outputs (like controls). If you have used spreadsheets or basic scripting, this will feel familiar.

Connections are the links between nodes. When you connect one block to another, you are saying "use this output as that input." You will see a line between blocks so you always know what is feeding what.

Syntax and available functions will expand as the SDK grows, but the workflow stays the same: create a block, connect it, and describe the behavior in an expression. If you are unsure, start with a simple expression and build up.

Here is the expression guide in plain English, with what you can reference and how it evaluates.

### Quick rules
- An expression can be a single formula like `input * 0.5`, which halves the `input` value. 
- You can also assign outputs directly: `roll = 992; throttle = 200`. Use semicolons to do multiple assignments in one block.
- Use parentheses to control order of operations, like `(a + b) * 2`.

### Output naming (where values land)
You can write to outputs in two ways:
- Use your port names: `ax = ...`, `out_roll = ...`, or whatever the block output is named.
- Use standard control names: `roll`, `pitch`, `yaw`, `throttle`, `aux1`, `aux2`.

### Control output basics (roll, pitch, yaw, throttle)
These four are the core control outputs. They all use the same CRSF range:
- Range is 172 to 1811.
- Throttle low is 172.
- Roll, pitch, and yaw are centered at 992.
- Values above 992 push right/forward/clockwise; values below 992 push left/back/counterclockwise.

### Inputs and parameters (what you can read)
Inputs arrive by name, so you can reference them directly, like `speed` or `error`. You also get:
- Any block parameter you defined in the UI (by key name).

### Built-in constants
- `pi` or `PI`
- Flight mode values: `angle`, `horizon`, `acro` must be set as aux2 declaration in the control node. 

### Time and battery
- `time` (seconds since this node started running)
- `battery` or `battery_level` (percentage)
- `battery_voltage` (volts)

### Attitude and IMU
- `attitude_roll`, `attitude_pitch`, `attitude_yaw` (degrees)
- `sensor_roll`, `sensor_pitch`, `sensor_yaw` (aliases for attitude)

### Vision target (current frame)
- `target_x`, `target_y` (target center)
- `target_width`, `target_height`, `target_diag`
- `target_confidence`
- `target_visible` (1 or 0)

### Vision target (history)
- `target_x_last_n`, `target_y_last_n`
- `target_width_last_n`, `target_height_last_n`, `target_diag_last_n`
- `target_confidence_last_n`
- `target_visible_last_n` (1 or 0)
- `frames_since_detection`

### Height estimation (VIHO)
- `floor_distance`, `ceiling_distance`
- `height_data_valid` (1 or 0)
- `viho_height` (ceiling minus floor)

### Depth navigation
- `nav_target_x`, `nav_target_y`, `nav_max_depth`
- `nav_grid_row`, `nav_grid_col`
- `nav_data_valid` (1 or 0)
- `nav_left_depth`, `nav_right_depth`, `nav_center_depth`

### Optical flow
- `flow_x`, `flow_y`, `flow_magnitude`, `flow_angle`
- `flow_valid` (1 or 0)

### Rangefinder
- `rangefinder_distance`, `rangefinder_strength`
- `rangefinder_valid` (1 or 0)

### Velocity, acceleration, altitude
- `vel_x`, `vel_y`, `vel_z`, `vel_valid` (1 or 0)
- `accel_x`, `accel_y`, `accel_z`, `accel_valid` (1 or 0)
- `altitude`, `altitude_valid` (1 or 0)

### Optical range (optrange)
- `optrange_distance`
- `optrange_velX`, `optrange_velY`, `optrange_vz_measured`
- `optrange_valid` (1 or 0)

### Operators you can use
- Math: `+ - * /`
- Comparisons: `== != < <= > >=`
- Logic: `&&` (and), `||` (or)

### Functions (with quick explanations)
- `if(condition, true_value, false_value)`
- `lin(start, end, duration)` ramps from start to end over time
- `pid(setpoint, process_variable, kp, ki, kd [, max_output])`
- `mem(value [, condition [, reset [, edge_once]]])` stores a value and holds it
- `once(expr)` evaluates once and then caches it
- `acceleration(desired_accel [, motor_efficiency])` returns throttle needed for that vertical acceleration
- `rand()` random 0..1
- `randn()` random normal (mean 0, std 1)
- `randint(max)` integer 0..max-1
- `randrange(max)` float 0..max
- `sin`, `cos`, `tan`, `sqrt`, `abs`, `floor`, `ceil`, `sign`, `min(a,b)`, `max(a,b)`

### Small, practical examples
Simple math into output 0 (this halves the input):
```txt
input * 0.5
```

Control outputs with assignments:
```txt
roll = 992; pitch = 992; yaw = 992; throttle = 200
```

Center on a target when it is visible:
```txt
yaw = pid(320, target_x, 0.8, 0.0, 0.1, 300)
```

Smooth ramp over 2 seconds:
```txt
throttle = lin(172, 600, 2.0)
```

Latch a flag the first time a target appears:
```txt
aux1 = mem(1800, target_visible, 0, 1)
```

Randomized hover nudge (evaluated once):
```txt
roll = once(randrange(50)) - 25
```
## Deploying Your Agent
When you are ready to fly, open the Deploy tab. The agent you have loaded is shown in the top left. If it is not the right one, go back to Build and select it.

If you are arming the drone via expressions, make sure those conditions are correct before you press play.

Click Play to deploy the loaded agent. Click Pause to stop it. Always keep a hand on your safety switch or emergency stop if you have one.

## Deploy Tab
This is your live control and monitoring view.
- Video feed: if it is black, jump to VRX Controls below.
- Nimbus connection indicator: top right. If this is not green, you are not connected.
- Four channel status: right side. This confirms your control channels are live.
- Drone profile: set the mass and KV of the drone so the physics are correct.
- Play and Pause: start and stop the active agent.

## VRX Controls
Use these controls to connect Nimbus to your drone.
- Bind: put your drone into bind mode by unplugging and plugging it back in three times. You should see three quick flashes. When you see the flashes turn into single flashes,Nimbus is connected, click Bind.
- Scan: this cycles through the 5.8 GHz bands until it finds your drone's transmission.

## Flight Tab
Use this tab to review telemetry and past flights. You can download the logs and use them to train new models or debug behavior.

## Community Tab 
This is where you share agents and browse what others have built. If you found something useful, you can download it and make it your own.
