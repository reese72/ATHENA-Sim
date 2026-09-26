# Complete Build Specification: ATHENA-Sim Hardware-in-the-Loop Architecture

## Overview
Build a two-process hardware-in-the-loop (HIL) simulation system for the ATHENA vehicle consisting of:
1. **controller.py** — Flight controller running independently with full PID control logic
2. **simulator.py** — Physics engine and sensor simulation running independently
3. **config.json** — Centralized configuration for all parameters
4. **utils.py** — Shared utilities (data structures, validation, logging)
5. **requirements.txt** — Python dependencies
6. **README.md** — Setup and usage instructions

Both processes communicate via TCP sockets over localhost using JSON-serialized messages.

---

## Part 1: Physics Engine Corrections & Enhancements

### 1.1 Force Calculation Fixes

**Current Issues to Fix:**
- Drag vector sign logic is backwards (lines 75-80 in Main.py)
- Thrust vector calculation mixes coordinate systems inconsistently
- Gravity is only applied after thrust phase instead of continuously
- Missing mass loss from propellant burndown
- No sensor noise or realistic measurement delays
- Acceleration calculation order violates physics simulation best practices

**Corrected Physics Pipeline (per simulation step):**

1. **Compute Thrust Force**
   - Read throttle command from controller (0-100%)
   - Compute thrust magnitude: `T = (throttle/100) * max_thrust`
   - Use gimbal commands to tilt thrust vector
   - Resolve into body-frame components: [T_x, T_y, T_z]
   - Transform to inertial frame if needed (for non-zero pitch/roll attitudes)

2. **Compute Aerodynamic Drag**
   - **CRITICAL FIX**: Drag always opposes velocity
   - For each axis (x, y, z):
     - Compute dynamic pressure: `q = 0.5 * ρ * v²` where ρ from atmosphere(altitude)
     - Compute effective area: blend frontal_area and side_area based on attitude angles
     - Drag force magnitude: `F_drag = Cd * A_eff * q`
     - **Direction**: Always oppose velocity: `F_drag_component = -sign(v_component) * F_drag`
     - (NOT: `if velocity > 0 then multiply by -1` — this is backwards)

3. **Compute Gravitational Force**
   - Apply continuously (not just when has_left_ground)
   - Only acts on Z-axis (vertical): `F_g = -mass * g = -mass * 9.81`

4. **Sum All Forces**
   - Total force vector: `F_total = F_thrust + F_drag + F_gravity`

5. **Compute Accelerations**
   - Linear acceleration: `a = F_total / mass`
   - Rotational dynamics:
     - Torque from off-center thrust: `τ = r × F` (cross product)
     - Moment of inertia: `I = (1/12)*m*L² + (1/4)*m*R²` (solid cylinder approximation)
     - Angular acceleration: `α = τ / I`

6. **Integrate Using RK4 or Euler (your choice)**
   - For Euler (simple but less accurate):
     ```
     v_new = v_old + a * dt
     x_new = x_old + v_new * dt
     ```
   - For RK4 (more accurate, recommended):
     - Compute k1, k2, k3, k4 slopes
     - Weighted average: `v_new = v_old + (k1 + 2*k2 + 2*k3 + k4)/6 * dt`

### 1.2 Mass Loss from Propellant Burndown

**Add to simulator:**

```
propellant_mass = 0.15  # kg (total propellant)
initial_mass = 0.3      # kg (dry mass + propellant)
current_mass = initial_mass

# During each simulation step:
if throttle > 0:
    # Compute mass flow rate from thrust curve
    # Typical: mdot = thrust / (specific_impulse * g)
    # For this rocket, estimate from thrust profile
    mass_flow_rate = compute_mass_flow(thrust_magnitude)  # kg/s
    current_mass -= mass_flow_rate * step
    
    # Ensure mass doesn't go below dry mass
    current_mass = max(current_mass, initial_mass - propellant_mass)
```

This affects:
- Acceleration (inversely proportional to mass)
- Moment of inertia (changes with mass distribution as fuel burns)

### 1.3 Coordinate System Standardization

**Establish Clear Convention (Body Frame vs Inertial Frame):**

- **Inertial Frame (World)**: X (East), Y (North), Z (Altitude/Up)
- **Body Frame**: X (Forward/Nose), Y (Right Wing), Z (Down/Belly)
- **Attitude Angles**:
  - Pitch (θ): rotation around Y-axis (nose up/down)
  - Roll (φ): rotation around X-axis (wing up/down)
  - Yaw (ψ): rotation around Z-axis (heading)

**In this simulation**, simplify to 2D (no yaw):
- `pitch_angle` replaces `launch_angle_x_degrees` (in radians internally, convert to degrees for display)
- `roll_angle` replaces `launch_angle_y_degrees`
- `pitch_rate` and `roll_rate` are angular velocities in rad/s

**Thrust Vector Transformation:**
```
# Gimbal commands [gimbal_pitch, gimbal_roll] in body frame
# Compute force in body-frame, then rotate to inertial frame
# Body-frame force (with gimbal deflection):
F_body_x = thrust * cos(gimbal_pitch) * cos(gimbal_roll)
F_body_y = thrust * sin(gimbal_roll)
F_body_z = -thrust * sin(gimbal_pitch) * cos(gimbal_roll)

# Rotate from body to inertial using rotation matrix R(pitch, roll)
F_inertial = R(pitch, roll) @ F_body
```

---

## Part 2: Sensor Simulation & Data Validation

### 2.1 Realistic Sensor Simulation

**Implement sensor suite:**

1. **Inertial Measurement Unit (IMU)**
   - Outputs: 3-axis accelerometer, 3-axis gyroscope
   - Add noise: Gaussian white noise (std dev ~0.01 m/s² for accel, ~0.001 rad/s for gyro)
   - Simulate bias: Small constant offset per axis
   - Simulate saturation: Max measureable (e.g., ±30 m/s² for accel)

   ```python
   def add_sensor_noise(true_value, sensor_type='accel'):
       noise = np.random.normal(0, noise_std[sensor_type])
       bias = sensor_bias[sensor_type]
       saturated = np.clip(true_value + noise + bias, 
                          -sensor_max[sensor_type], 
                          sensor_max[sensor_type])
       return saturated
   ```

2. **Barometric Altimeter**
   - Measures altitude from atmospheric pressure
   - Add noise: ~±2 meter standard deviation
   - Slow response (lag): Filter with 1-2 second time constant

   ```python
   def barometer_filter(altitude, dt, tau=1.0):
       filtered = filtered_prev + (altitude - filtered_prev) * (dt / tau)
       return filtered
   ```

3. **Gimbal Position Feedback**
   - Measure actual gimbal angle (with rate limit)
   - Add quantization: Discrete steps (e.g., 0.1 degree resolution)

### 2.2 Sensor Latency & Delay

**Simulate realistic communication delay:**

```python
class SensorBuffer:
    def __init__(self, delay_ms, step_size_ms):
        self.delay_steps = int(delay_ms / step_size_ms)
        self.buffer = deque(maxlen=self.delay_steps)
    
    def add(self, measurement):
        self.buffer.append(measurement)
    
    def get_delayed(self):
        # Return measurement from delay_steps ago
        if len(self.buffer) == self.delay_steps:
            return self.buffer[0]  # Oldest in buffer
        else:
            return self.buffer[0] if self.buffer else None
```

**Controller delay:** Add ~50ms latency on sensor readings sent to controller  
**Actuator delay:** Add ~20ms latency on gimbal/throttle command application

### 2.3 Data Validation & Error Handling

**In simulator message validation:**
```python
def validate_state_message(state):
    required_keys = ['time', 'altitude', 'position', 'velocity', 'attitude', ...]
    assert all(k in state for k in required_keys), f"Missing keys in state"
    
    # Sanity checks
    assert -100 < state['altitude'] < 10000, f"Altitude out of bounds"
    assert all(-100 < v < 500 for v in state['velocity']), f"Velocity out of bounds"
    
    return True

def validate_command_message(cmd):
    assert 0 <= cmd['throttle'] <= 100, f"Throttle out of range"
    assert -30 < cmd['gimbal_x'] < 30, f"Gimbal X out of range"
    assert -30 < cmd['gimbal_y'] < 30, f"Gimbal Y out of range"
    return True
```

---

## Part 3: IPC Protocol & Communication

### 3.1 Socket Communication Pattern

**Protocol: Request-Response over TCP**

1. Simulator listens on `localhost:5555`
2. Controller connects as client
3. Each cycle:
   - Simulator sends state message (blocking)
   - Controller receives and processes
   - Controller sends command message (blocking)
   - Simulator receives and updates

**Message Format: JSON**

**State Message (Simulator → Controller):**
```json
{
  "time": 12.34567,
  "altitude": 150.25,
  "vertical_velocity": 45.3,
  "vertical_acceleration": -2.1,
  "position": [10.5, -8.3, 150.25],
  "velocity": [2.1, -1.5, 45.3],
  "attitude": [0.125, -0.087],
  "attitude_rate": [0.001, 0.002],
  "gimbal_actual": [5.2, -3.1],
  "throttle_actual": 75,
  "timestamp": 12.34567
}
```

**Command Message (Controller → Simulator):**
```json
{
  "gimbal_x_setpoint": 5.5,
  "gimbal_y_setpoint": -3.2,
  "throttle_setpoint": 75,
  "timestamp": 12.34567,
  "controller_status": "armed"
}
```

### 3.2 Connection Management

**Simulator side:**
```python
def accept_connection(self):
    try:
        self.client_sock, addr = self.server_sock.accept()
        self.client_sock.settimeout(2.0)  # 2-second timeout
        logger.info(f"Controller connected from {addr}")
    except socket.timeout:
        logger.error("Controller connection timeout")
        self.shutdown()

def send_state(self, state_dict):
    try:
        msg = json.dumps(state_dict) + '\n'  # Newline for framing
        self.client_sock.send(msg.encode('utf-8'))
    except (socket.timeout, BrokenPipeError) as e:
        logger.error(f"Send error: {e}")
        self.reconnect()

def receive_command(self, timeout=1.0):
    try:
        data = self.client_sock.recv(4096).decode('utf-8')
        command = json.loads(data.strip())
        validate_command_message(command)
        return command
    except (socket.timeout, json.JSONDecodeError, AssertionError) as e:
        logger.warning(f"Receive error: {e}, using safe fallback (0% throttle, 0° gimbal)")
        return {'gimbal_x_setpoint': 0, 'gimbal_y_setpoint': 0, 'throttle_setpoint': 0}
```

**Controller side:**
```python
def connect_to_simulator(self, host='localhost', port=5555, max_retries=5):
    for attempt in range(max_retries):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((host, port))
            logger.info("Connected to simulator")
            return
        except ConnectionRefusedError:
            logger.warning(f"Connection attempt {attempt+1}/{max_retries} failed, retrying...")
            time.sleep(1)
    raise RuntimeError("Failed to connect to simulator after retries")

def read_state(self):
    try:
        data = self.sock.recv(4096).decode('utf-8')
        state = json.loads(data.strip())
        validate_state_message(state)
        return state
    except socket.timeout:
        logger.error("State receive timeout")
        raise

def send_command(self, cmd):
    try:
        msg = json.dumps(cmd) + '\n'
        self.sock.send(msg.encode('utf-8'))
    except socket.timeout:
        logger.error("Command send timeout")
        raise
```

---

## Part 4: Controller Implementation

### 4.1 PID Control Loop Structure

**Main controller loop (runs at ~50ms cycle time, or every N simulator steps):**

```python
def control_loop(self):
    while self.armed and self.sim_time < self.max_sim_time:
        # Step 1: Receive sensor state from simulator
        state = self.read_state()
        self.sim_time = state['time']
        
        # Step 2: Update error states
        self.altitude_error = self.altitude_setpoint - state['altitude']
        self.pitch_error = self.pitch_setpoint - state['attitude'][0]
        self.roll_error = self.roll_setpoint - state['attitude'][1]
        
        # Step 3: Update integral and derivative terms (with anti-windup)
        dt = state['time'] - self.last_time
        self.update_pid_terms(dt)
        
        # Step 4: Compute PID outputs
        gimbal_pitch_cmd = self.pid_pitch.compute(self.pitch_error)
        gimbal_roll_cmd = self.pid_roll.compute(self.roll_error)
        throttle_cmd = self.pid_throttle.compute(self.altitude_error)
        
        # Step 5: Apply rate limits (actuator constraints)
        gimbal_pitch_cmd = self.rate_limit(gimbal_pitch_cmd, 
                                           self.gimbal_pitch_cmd_prev,
                                           max_rate=20.0,  # deg/s
                                           dt=dt)
        gimbal_roll_cmd = self.rate_limit(gimbal_roll_cmd,
                                          self.gimbal_roll_cmd_prev,
                                          max_rate=20.0,
                                          dt=dt)
        throttle_cmd = self.rate_limit(throttle_cmd,
                                       self.throttle_cmd_prev,
                                       max_rate=30.0,  # %/s
                                       dt=dt)
        
        # Step 6: Apply gimbal and throttle limits
        gimbal_pitch_cmd = np.clip(gimbal_pitch_cmd, -20, 20)
        gimbal_roll_cmd = np.clip(gimbal_roll_cmd, -20, 20)
        throttle_cmd = np.clip(throttle_cmd, 0, 100)
        
        # Step 7: Send commands to simulator
        command = {
            'gimbal_x_setpoint': gimbal_pitch_cmd,
            'gimbal_y_setpoint': gimbal_roll_cmd,
            'throttle_setpoint': throttle_cmd,
            'timestamp': self.sim_time,
            'controller_status': 'armed'
        }
        self.send_command(command)
        
        # Step 8: Logging (optional)
        if self.sim_time % 0.1 < dt:  # Log every 0.1s
            self.log_telemetry(state, command)
        
        self.last_time = state['time']
```

### 4.2 PID Class with Anti-Windup

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit  # (min, max) tuple
        self.integral_limit = integral_limit
        
        self.integral = 0
        self.prev_error = 0
    
    def compute(self, error, dt=0.05):
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.integral_limit:
            self.integral = np.clip(self.integral, 
                                   -self.integral_limit, 
                                   self.integral_limit)
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        
        # Total output
        output = p_term + i_term + d_term
        
        # Output limiting
        if self.output_limit:
            output = np.clip(output, self.output_limit[0], self.output_limit[1])
        
        self.prev_error = error
        return output
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
```

### 4.3 PID Tuning Parameters (from Main.py, with validation)

**Pitch Control (X-axis):**
- Kp = 0.11
- Ki = 0.01
- Kd = 0.04
- Output limit: ±20 degrees

**Roll Control (Y-axis):**
- Kp = 0.11
- Ki = 0.01
- Kd = 0.04
- Output limit: ±20 degrees

**Altitude/Throttle Control:**
- Kp = 0.0 (disabled in original, should be tuned)
- Ki = 0.0
- Kd = 0.0
- Output limit: 0-100%

**Add to config.json with documented tuning procedure:**
```
"pid_tuning_notes": "Use Ziegler-Nichols method: increase Kp until oscillation, then tune Ki and Kd"
```

---

## Part 5: Simulation Engine Core

### 5.1 Physics Integration Loop

```python
def step_physics(self, dt):
    """Integrate one physics time step using RK4"""
    
    # Current state vector
    state = [self.x, self.y, self.z, 
             self.vx, self.vy, self.vz,
             self.pitch, self.roll,
             self.pitch_rate, self.roll_rate,
             self.mass]
    
    # Compute derivatives at 4 points
    k1 = self.compute_derivatives(state, self.gimbal_x_cmd, self.gimbal_y_cmd, self.throttle_cmd)
    k2 = self.compute_derivatives([s + 0.5*dt*k for s, k in zip(state, k1)], 
                                  self.gimbal_x_cmd, self.gimbal_y_cmd, self.throttle_cmd)
    k3 = self.compute_derivatives([s + 0.5*dt*k for s, k in zip(state, k2)],
                                  self.gimbal_x_cmd, self.gimbal_y_cmd, self.throttle_cmd)
    k4 = self.compute_derivatives([s + dt*k for s, k in zip(state, k3)],
                                  self.gimbal_x_cmd, self.gimbal_y_cmd, self.throttle_cmd)
    
    # RK4 update
    for i in range(len(state)):
        state[i] += (dt/6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
    
    # Extract updated state
    self.x, self.y, self.z = state[0:3]
    self.vx, self.vy, self.vz = state[3:6]
    self.pitch, self.roll = state[6:8]
    self.pitch_rate, self.roll_rate = state[8:10]
    self.mass = state[10]
    
    # Clamp altitutde to ground
    if self.z < 0:
        self.z = 0
        self.vz = max(0, self.vz)  # Prevent sinking
        if abs(self.vz) < 0.1:
            self.vx = self.vy = self.vz = 0  # Stop all motion on ground

def compute_derivatives(self, state, gimbal_x, gimbal_y, throttle):
    """Compute state derivatives: [dx/dt, dy/dt, ..., dmass/dt]"""
    x, y, z, vx, vy, vz, pitch, roll, pitch_rate, roll_rate, mass = state
    
    # 1. Compute thrust force
    thrust_mag = (throttle / 100.0) * self.max_thrust
    fx_thrust, fy_thrust, fz_thrust = self.compute_thrust_vector(gimbal_x, gimbal_y, 
                                                                   pitch, roll, thrust_mag)
    
    # 2. Compute drag force
    fx_drag, fy_drag, fz_drag = self.compute_drag_vector([vx, vy, vz], pitch, roll, z)
    
    # 3. Compute gravity
    fz_gravity = -mass * 9.81
    
    # 4. Sum forces
    fx_total = fx_thrust + fx_drag
    fy_total = fy_thrust + fy_drag
    fz_total = fz_thrust + fz_drag + fz_gravity
    
    # 5. Compute accelerations
    ax = fx_total / mass
    ay = fy_total / mass
    az = fz_total / mass
    
    # 6. Compute torques and angular accelerations
    tau_pitch = fy_thrust * self.cg_to_gimbal_distance  # From off-center thrust
    tau_roll = fx_thrust * self.cg_to_gimbal_distance
    
    # Moment of inertia (cylinder)
    moi = (mass * self.length**2) / 12.0 + (mass * self.radius**2) / 4.0
    
    pitch_accel = tau_pitch / moi
    roll_accel = tau_roll / moi
    
    # 7. Compute mass flow rate (propellant consumption)
    if throttle > 0:
        dmass_dt = -self.compute_mass_flow_rate(thrust_mag)
    else:
        dmass_dt = 0
    
    # Return derivatives
    return [vx, vy, vz,              # d(pos)/dt = vel
            ax, ay, az,              # d(vel)/dt = accel
            pitch_rate, roll_rate,   # d(attitude)/dt = attitude_rate
            pitch_accel, roll_accel, # d(attitude_rate)/dt = angular_accel
            dmass_dt]                # d(mass)/dt = -mass_flow_rate
```

### 5.2 Aerodynamic Drag Calculation (CORRECTED)

```python
def compute_drag_vector(self, velocity, pitch, roll, altitude):
    """
    Compute drag forces that OPPOSE velocity.
    CRITICAL: Drag direction = -sign(v_component) not if(v>0) multiply by -1
    """
    vx, vy, vz = velocity
    
    # Atmospheric density at altitude
    rho = self.atmosphere_density(altitude)
    
    # Effective area based on attitude
    # Simplified: interpolate between frontal and side area
    alpha = pitch  # pitch angle from horizontal
    beta = roll    # roll angle
    
    # More frontal area exposed if pitched nose-up
    frontal_weight = abs(np.sin(alpha))
    side_weight = abs(np.cos(alpha))
    
    ax_eff = frontal_weight * self.frontal_area + side_weight * self.side_area
    ay_eff = side_weight * self.side_area + frontal_weight * self.frontal_area
    az_eff = frontal_weight * self.frontal_area + side_weight * self.side_area
    
    # Drag force magnitude for each axis: F_drag = 0.5 * rho * Cd * A * v^2
    fx_mag = 0.5 * rho * self.cd * ax_eff * (vx ** 2)
    fy_mag = 0.5 * rho * self.cd * ay_eff * (vy ** 2)
    fz_mag = 0.5 * rho * self.cd * az_eff * (vz ** 2)
    
    # CORRECTED: Apply opposing direction
    fx_drag = -np.sign(vx) * fx_mag if vx != 0 else 0
    fy_drag = -np.sign(vy) * fy_mag if vy != 0 else 0
    fz_drag = -np.sign(vz) * fz_mag if vz != 0 else 0
    
    return fx_drag, fy_drag, fz_drag

def atmosphere_density(self, altitude):
    """Exponential atmosphere model"""
    rho_sea_level = 1.225  # kg/m^3
    scale_height = 8435.0  # meters
    return rho_sea_level * np.exp(-altitude / scale_height)
```

### 5.3 Thrust Vector Calculation

```python
def compute_thrust_vector(self, gimbal_pitch, gimbal_roll, pitch, roll, thrust_mag):
    """
    Compute thrust force in inertial frame.
    
    gimbal_pitch, gimbal_roll: Gimbal deflection angles (degrees)
    pitch, roll: Vehicle attitude angles (radians)
    thrust_mag: Total thrust magnitude (Newtons)
    """
    # Convert gimbal angles to radians
    gimbal_pitch_rad = np.radians(gimbal_pitch)
    gimbal_roll_rad = np.radians(gimbal_roll)
    
    # Thrust in body frame (with gimbal deflection)
    # Gimbal tilts thrust vector away from +Z (nose) in body frame
    fx_body = thrust_mag * np.sin(gimbal_pitch_rad) * np.cos(gimbal_roll_rad)
    fy_body = thrust_mag * np.sin(gimbal_roll_rad)
    fz_body = thrust_mag * np.cos(gimbal_pitch_rad) * np.cos(gimbal_roll_rad)
    
    # Rotation matrix from body to inertial (pitch and roll only)
    # Rotation order: first pitch (around Y), then roll (around X)
    R = self.rotation_matrix(pitch, roll)
    
    # Transform to inertial frame
    F_body = np.array([fx_body, fy_body, fz_body])
    F_inertial = R @ F_body
    
    return F_inertial[0], F_inertial[1], F_inertial[2]

def rotation_matrix(self, pitch, roll):
    """DCM: Inertial to Body (with pitch and roll only)"""
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    
    # Body to Inertial (transpose for Inertial to Body conversion)
    R = np.array([
        [cp,           sr*sp,     -cr*sp],
        [0,            cr,        sr],
        [sp,          -sr*cp,     cr*cp]
    ])
    return R
```

### 5.4 Mass Flow Rate & Propellant Model

```python
def compute_mass_flow_rate(self, thrust_mag):
    """
    Compute mass consumption rate from thrust.
    
    Model: mdot = thrust / (I_sp * g)
    where I_sp = specific impulse (seconds)
    
    For ATHENA, fit I_sp from the Thrust(t) function in Main.py
    """
    # Typical solid rocket: I_sp = 200-300 seconds
    # Estimate from thrust curve: 75N peak thrust
    isp = 250.0  # seconds (tunable from rocket motor data)
    g = 9.81
    
    mdot = thrust_mag / (isp * g)
    return mdot
```

---

## Part 6: Configuration File (config.json)

```json
{
  "simulation": {
    "step_size_ms": 0.05,
    "max_simulation_time_s": 120,
    "integrator": "rk4",
    "ground_altitude_m": 0,
    "wind_model": "none"
  },
  
  "rocket": {
    "dry_mass_kg": 0.15,
    "propellant_mass_kg": 0.15,
    "total_mass_kg": 0.3,
    "length_m": 0.35,
    "radius_m": 0.076,
    "cg_to_gimbal_distance_m": 0.1,
    "max_thrust_n": 75,
    "specific_impulse_s": 250
  },
  
  "aerodynamics": {
    "frontal_area_m2": 0.0182,
    "side_area_m2": 0.027,
    "drag_coefficient": 0.55,
    "atmosphere_model": "exponential",
    "sea_level_density_kg_m3": 1.225,
    "scale_height_m": 8435
  },
  
  "thrust_profile": {
    "model": "polyfit",
    "description": "Fit from Main.py Thrust(t) function",
    "phases": [
      {"time_s": [0, 0.2], "throttle_pct": [0, 100], "description": "Ramp to full"},
      {"time_s": [0.2, 0.25], "throttle_pct": [100, 0], "description": "Burnout"},
      {"time_s": [0.25, 1.85], "throttle_pct": [60, 60], "description": "Tail-off"},
      {"time_s": [1.85, 120], "throttle_pct": [0, 0], "description": "No thrust"}
    ]
  },
  
  "gimbal": {
    "rate_limit_deg_per_s": 20,
    "pitch_limit_deg": 20,
    "roll_limit_deg": 20,
    "actuation_delay_ms": 20
  },
  
  "throttle": {
    "rate_limit_pct_per_s": 30,
    "actuation_delay_ms": 20
  },
  
  "controllers": {
    "pitch": {
      "kp": 0.11,
      "ki": 0.01,
      "kd": 0.04,
      "integral_limit": 10,
      "output_limit": [-20, 20]
    },
    "roll": {
      "kp": 0.11,
      "ki": 0.01,
      "kd": 0.04,
      "integral_limit": 10,
      "output_limit": [-20, 20]
    },
    "altitude": {
      "kp": 0.0,
      "ki": 0.0,
      "kd": 0.0,
      "integral_limit": 100,
      "output_limit": [0, 100],
      "enabled": false,
      "note": "Disabled by default; tune if using closed-loop throttle"
    }
  },
  
  "sensors": {
    "imu": {
      "accel_noise_std_m_s2": 0.01,
      "accel_bias_m_s2": [0.0, 0.0, 0.0],
      "accel_max_m_s2": 30,
      "gyro_noise_std_rad_s": 0.001,
      "gyro_bias_rad_s": [0.0, 0.0, 0.0],
      "gyro_max_rad_s": 5.0,
      "sample_rate_hz": 100
    },
    "barometer": {
      "altitude_noise_std_m": 2.0,
      "altitude_bias_m": 0.0,
      "response_tau_s": 1.0,
      "sample_rate_hz": 20
    },
    "gimbal_feedback": {
      "quantization_deg": 0.1,
      "noise_std_deg": 0.05,
      "sample_rate_hz": 50
    }
  },
  
  "communication": {
    "host": "localhost",
    "port": 5555,
    "socket_timeout_s": 2.0,
    "message_frame_char": "\n",
    "state_message_frequency_hz": 200,
    "command_message_frequency_hz": 200
  },
  
  "logging": {
    "enabled": true,
    "log_file": "simulation.log",
    "log_level": "INFO",
    "telemetry_csv": "telemetry.csv",
    "telemetry_columns": [
      "time", "altitude", "vertical_velocity", "position_x", "position_y", "position_z",
      "velocity_x", "velocity_y", "velocity_z",
      "pitch", "roll", "pitch_rate", "roll_rate",
      "gimbal_x_cmd", "gimbal_y_cmd", "gimbal_x_actual", "gimbal_y_actual",
      "throttle_cmd", "throttle_actual", "thrust_n", "mass_kg"
    ],
    "log_frequency_hz": 10
  },
  
  "initial_conditions": {
    "position_m": [0, 0, 0],
    "velocity_m_s": [0, 0, 0],
    "attitude_rad": [0, 0],
    "attitude_rate_rad_s": [0, 0],
    "launch_angle_pitch_deg": 5,
    "launch_angle_roll_deg": -3
  }
}
```

---

## Part 7: Utilities Module (utils.py)

```python
import logging
import json
from pathlib import Path

class ConfigLoader:
    @staticmethod
    def load(config_file='config.json'):
        with open(config_file, 'r') as f:
            return json.load(f)

class Logger:
    @staticmethod
    def setup(name, log_file='simulation.log', level=logging.INFO):
        logger = logging.getLogger(name)
        logger.setLevel(level)
        
        # File handler
        fh = logging.FileHandler(log_file)
        fh.setLevel(level)
        
        # Console handler
        ch = logging.StreamHandler()
        ch.setLevel(level)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        logger.addHandler(fh)
        logger.addHandler(ch)
        
        return logger

class TelemetryRecorder:
    def __init__(self, csv_file, columns):
        self.csv_file = csv_file
        self.columns = columns
        self.data = []
        
        # Write header
        with open(csv_file, 'w') as f:
            f.write(','.join(columns) + '\n')
    
    def record(self, data_dict):
        row = [str(data_dict.get(col, '')) for col in self.columns]
        with open(self.csv_file, 'a') as f:
            f.write(','.join(row) + '\n')

def validate_state(state):
    """Check state dictionary for required fields and reasonable values"""
    required = ['time', 'altitude', 'velocity', 'attitude', 'position']
    for key in required:
        assert key in state, f"Missing key: {key}"
    
    # Sanity checks
    assert -1000 < state['altitude'] < 50000, f"Altitude out of bounds: {state['altitude']}"
    assert all(-500 < v < 500 for v in state['velocity']), f"Velocity out of bounds"
    
    return True

def validate_command(cmd):
    """Check command dictionary"""
    required = ['gimbal_x_setpoint', 'gimbal_y_setpoint', 'throttle_setpoint']
    for key in required:
        assert key in cmd, f"Missing key: {key}"
    
    assert 0 <= cmd['throttle_setpoint'] <= 100, "Throttle out of range"
    assert -30 < cmd['gimbal_x_setpoint'] < 30, "Gimbal X out of range"
    assert -30 < cmd['gimbal_y_setpoint'] < 30, "Gimbal Y out of range"
    
    return True
```

---

## Part 8: Execution & Testing

### 8.1 Startup Sequence

1. **Start Simulator Process**
   ```bash
   python simulator.py --config config.json
   ```
   - Loads configuration
   - Initializes physics engine
   - Listens for controller connection on localhost:5555

2. **Start Controller Process** (in separate terminal)
   ```bash
   python controller.py --config config.json
   ```
   - Connects to simulator
   - Begins control loop
   - Sends gimbal/throttle commands every ~50ms

3. **Simulation runs until:**
   - Altitude drops below 0 (landed)
   - Max simulation time reached
   - One process crashes (implement graceful shutdown)

### 8.2 Testing Checklist

- [ ] Physics accuracy: Compare trajectory with known reference (e.g., idealized ballistic trajectory without thrust)
- [ ] Sensor noise: Verify noise levels match specifications
- [ ] PID tuning: Verify vehicle stabilizes under PID control
- [ ] Command limits: Confirm gimbal/throttle limits are enforced
- [ ] IPC reliability: Test with simulated packet loss, delays
- [ ] Logging: Verify CSV telemetry matches computed state
- [ ] Graceful degradation: Test controller behavior with no throttle, saturated gimbal
- [ ] Performance: Ensure simulation runs faster than real-time (dt=0.05ms per step)

### 8.3 Validation Test Cases

**Test 1: Ballistic Flight (no controller)**
- Set throttle to 0%, verify parabolic trajectory

**Test 2: Stable Hover (if possible)**
- Set throttle to maintain altitude, verify gimbal stabilizes rocket to vertical

**Test 3: Sensor Dropout**
- Simulate timeout from controller, verify simulator safe-states to 0% throttle

**Test 4: Mass Burndown**
- Verify acceleration increases as fuel burns

---

## Part 9: File Structure

```
ATHENA-Sim/
├── simulator.py              # Physics engine & sensor simulation
├── controller.py             # Flight controller
├── utils.py                  # Shared utilities
├── config.json               # Configuration (all parameters)
├── requirements.txt          # Python dependencies
├── README.md                 # Setup & usage instructions
├── BUILD_SIMULATION_PROMPT.md # (This file)
├── logs/
│   ├── simulation.log        # Simulator debug log
│   └── controller.log        # Controller debug log
└── results/
    ├── telemetry.csv         # Recorded sensor data and commands
    └── plots/
        ├── trajectory_3d.png
        ├── altitude_vs_time.png
        ├── gimbal_angles.png
        └── velocities.png
```

---

## Part 10: Requirements & Dependencies

```
# requirements.txt
numpy==1.24.3
matplotlib==3.7.1
scipy==1.11.1
```

---

## Part 11: Summary of Physics Corrections

| Issue | Original | Corrected |
|-------|----------|-----------|
| Drag sign | `if v > 0: F *= -1` | `F = -sign(v) * |F|` |
| Gravity | Only applied when airborne | Applied continuously |
| Mass loss | Not modeled | Tracks propellant burndown |
| Acceleration order | Thrust, then drag, then gravity in separate sections | Single unified force summation |
| Attitude angles | Named confusingly ("launch angle") | Renamed to pitch/roll, stored in radians |
| Gimbal application | Direct to angles | Applied through rotational dynamics |
| Drag area | Fixed per axis | Varies with attitude angles |

---

## End of Specification

This document provides a complete blueprint for building a production-ready hardware-in-the-loop simulation system. The LLM should implement all sections following the physics corrections and architecture patterns outlined above.
