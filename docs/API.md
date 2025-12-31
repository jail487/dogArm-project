# dogArm API Reference

## Firmware API

### Serial Communication Protocol

All commands are sent as ASCII strings terminated with newline (`\n`).

#### Commands

##### HOME
Home the robot to its reference position.

**Format**: `HOME\n`

**Response**: `OK:Homed\n`

**Example**:
```
> HOME
< OK:Homed
```

---

##### MOVE
Move to specified Cartesian coordinates.

**Format**: `MOVE:x,y,z\n`

**Parameters**:
- `x`: X coordinate in mm (float)
- `y`: Y coordinate in mm (float)
- `z`: Z coordinate in mm (float)

**Response**: `OK:Moving\n` or `ERROR:message\n`

**Example**:
```
> MOVE:100,200,5
< OK:Moving
< OK:Movement complete
```

---

##### SPEED
Set movement speed.

**Format**: `SPEED:value\n`

**Parameters**:
- `value`: Speed in steps per second (float)

**Response**: `OK:Speed set\n`

**Example**:
```
> SPEED:500
< OK:Speed set
```

---

##### PEN
Control pen/brush position.

**Format**: `PEN:UP\n` or `PEN:DOWN\n`

**Response**: `OK:Pen up\n` or `OK:Pen down\n`

**Example**:
```
> PEN:DOWN
< OK:Pen down
```

---

##### STATUS
Get current robot status.

**Format**: `STATUS\n`

**Response**: `STATUS:Homed:0/1,Moving:0/1\n`

**Example**:
```
> STATUS
< STATUS:Homed:1,Moving:0
```

---

##### POS
Get current robot position.

**Format**: `POS\n`

**Response**: `POS:x,y,z\n`

**Example**:
```
> POS
< POS:100.5,200.3,5.0
```

---

##### STOP
Emergency stop - immediately halt all motion.

**Format**: `STOP\n`

**Response**: `OK:Stopped\n`

**Example**:
```
> STOP
< OK:Stopped
```

---

## Python Software API

### RobotController Class

Main interface for controlling the robot.

#### Constructor

```python
RobotController(port: str, baudrate: int = 115200, timeout: float = 1.0)
```

**Parameters**:
- `port`: Serial port name (e.g., '/dev/ttyUSB0', 'COM3')
- `baudrate`: Serial communication baudrate (default: 115200)
- `timeout`: Serial read timeout in seconds (default: 1.0)

---

#### Methods

##### connect()
Connect to the robot.

```python
def connect(self) -> bool
```

**Returns**: True if connection successful

**Example**:
```python
robot = RobotController('/dev/ttyUSB0')
if robot.connect():
    print("Connected!")
```

---

##### disconnect()
Disconnect from the robot.

```python
def disconnect(self)
```

---

##### home()
Home the robot.

```python
def home(self) -> bool
```

**Returns**: True if homing successful

---

##### move_to()
Move to specified Cartesian coordinates.

```python
def move_to(self, x: float, y: float, z: float) -> bool
```

**Parameters**:
- `x`: X coordinate in mm
- `y`: Y coordinate in mm
- `z`: Z coordinate in mm

**Returns**: True if move command accepted

---

##### pen_up() / pen_down()
Control pen position.

```python
def pen_up(self) -> bool
def pen_down(self) -> bool
```

**Returns**: True if command successful

---

##### set_speed()
Set movement speed.

```python
def set_speed(self, speed: float) -> bool
```

**Parameters**:
- `speed`: Speed in steps per second

**Returns**: True if speed set successfully

---

##### get_status()
Get current robot status.

```python
def get_status(self) -> Optional[dict]
```

**Returns**: Dictionary with status information or None

**Example**:
```python
status = robot.get_status()
print(f"Homed: {status['homed']}")
print(f"Moving: {status['moving']}")
```

---

##### get_position()
Get current robot position.

```python
def get_position(self) -> Optional[Tuple[float, float, float]]
```

**Returns**: Tuple of (x, y, z) coordinates or None

---

### InverseKinematics Class

Inverse kinematics solver.

#### Constructor

```python
InverseKinematics(link1_length: float = 200.0, 
                 link2_length: float = 200.0,
                 base_width: float = 100.0)
```

---

#### Methods

##### solve()
Solve inverse kinematics for target position.

```python
def solve(self, x: float, y: float) -> Optional[Tuple[float, float]]
```

**Parameters**:
- `x`: Target X coordinate in mm
- `y`: Target Y coordinate in mm

**Returns**: Tuple of (theta1, theta2) joint angles in degrees, or None

---

### CalligraphyPlanner Class

Path planner for calligraphy writing.

#### Constructor

```python
CalligraphyPlanner(workspace_width: float = 200.0, 
                  workspace_height: float = 200.0)
```

---

#### Methods

##### create_simple_character()
Create a path for a simple character.

```python
def create_simple_character(self, character: str) -> List[PathPoint]
```

**Parameters**:
- `character`: Chinese character to write

**Returns**: List of PathPoint objects

---

##### plan_path()
Plan a path from stroke data.

```python
def plan_path(self, strokes: List[List[Tuple[float, float]]], 
             scale: float = 1.0,
             offset_x: float = 0.0,
             offset_y: float = 200.0) -> List[PathPoint]
```

**Parameters**:
- `strokes`: List of strokes, each stroke is a list of (x, y) points
- `scale`: Scaling factor
- `offset_x`: X offset
- `offset_y`: Y offset

**Returns**: List of PathPoint objects

---

##### interpolate_path()
Interpolate path for smooth motion.

```python
def interpolate_path(self, path: List[PathPoint], 
                    step_size: float = 5.0) -> List[PathPoint]
```

**Parameters**:
- `path`: Original path
- `step_size`: Maximum distance between points in mm

**Returns**: Interpolated path
