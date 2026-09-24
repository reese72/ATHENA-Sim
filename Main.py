import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as Math
import random

# PID Controller Function
def PID(P, I, D, error, rate, integral):
    return (P * error + I * integral + D * rate)

# General purpose function for thrust simulation, may need modified args
def Thrust(t):
    if t <= 0.2:
        return t * 75
    elif t > 0.2 and t <= 0.25:
        return 15 - (210 * (t - 0.2))
    elif t > 0.25 and t < 1.85:
        return 4.5
    else:
        return 0

def slop(true_value, tolerance):
    return true_value + random.randint(-1 * tolerance, tolerance)

# Ascent Rate Function
def ascent_rate(time):
    if time < 42:
        rate = ((161 * (time ** 4)) / 100000000) - ((33 * (time ** 2)) / 5000) + ((7 * time) / 50)
        if rate < 0:
            rate /= 2.5
    else:
        rate = -0.1
    return rate

# Force Vectors Calculation
def force_vectors(angles, thrust):
    Xangle = angles[0]
    Yangle = angles[1]

    alpha = np.radians(Xangle + 90)
    beta = np.radians(Yangle)

    # Initial velocity components
    x_vector = Math.cos(alpha) * Math.cos(beta) * thrust
    z_vector = Math.sin(alpha) * Math.cos(beta) * thrust
    y_vector = Math.sin(beta) * thrust

    return x_vector, y_vector, z_vector

def atmosphere(height):
    start_density = 1.225  # approximate density of atmosphere at sea level, kg/m^3
    # the atmospheric thinning is modeled as an exponential decay
    alt_density = start_density * (2.71828 ** (-0.00012 * (height + 250)))
    return alt_density

def Drag_vectors(angles, Velocities, Side_Area, Frontal_Area, Cd, Altitude):
    Xangle = angles[0]
    Yangle = angles[1]

    alpha = np.radians(Xangle + 90)
    beta = np.radians(Yangle)

    # Initial velocity components
    x_area = Math.cos(alpha) * Math.cos(beta) * Frontal_Area + (1 - Math.cos(alpha) * Math.cos(beta)) * Side_Area
    z_area = Math.sin(alpha) * Math.cos(beta) * Frontal_Area + (1 - Math.sin(alpha) * Math.cos(beta)) * Side_Area
    y_area = Math.sin(beta) * Frontal_Area + (1 - Math.sin(beta)) * Side_Area

    Density = atmosphere(Altitude)

    x_vector = 0.5 * Cd * x_area * (Velocities[0] ** 2) * Density
    y_vector = 0.5 * Cd * y_area * (Velocities[1] ** 2) * Density
    z_vector = 0.5 * Cd * z_area * (Velocities[2] ** 2) * Density

    if Velocities[0] > 0:
        x_vector *= -1
    if Velocities[1] > 0:
        y_vector *= -1
    if Velocities[2] > 0:
        z_vector *= -1

    return x_vector, y_vector, z_vector

# Change in Rotational Velocity Calculation
def change_in_rotational_velocity(force, distance, mass, length, time_step, radius):
    torque = force * distance
    moment_of_inertia = (mass * (length ** 2) / 12) + (mass * (radius ** 2) / 4)
    angular_acceleration = torque / moment_of_inertia
    change_in_rotational_velocity = angular_acceleration * time_step
    return change_in_rotational_velocity

# Plot Initialization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-20, 20])
ax.set_ylim([-20, 20])
ax.set_zlim([0, 20])

fig2 = plt.figure()
x_angle_plot = fig2.add_subplot(111)
x_angle_plot.set_ylim([-90, 90])

fig3 = plt.figure()
y_angle_plot = fig3.add_subplot(111)
y_angle_plot.set_ylim([-90, 90])

fig4 = plt.figure()
altitude_plot = fig4.add_subplot(111)

figure2, axis2 = plt.subplots(2, 2)
figure, axis = plt.subplots(2, 2)

# Simulation Parameters
step = 0.00005  # Time step in seconds
time = 0  # Initial time in seconds
PID_Control = True
Throttle_control = False

# Rocket Physical Properties
mass = 0.3  # Mass in kg
length = 0.35  # Length in meters
radius = 0.076  # Radius in meters
cg_to_gimbal_distance = 0.1  # Distance from center of gravity to gimbal in meters
Side_Area = 0.027 # Side Area in m^2
Frontal_Area = 0.0182 # Frontal Area in m^2
Cd = 0.55 # Drag Coefficient

# Initial Conditions
x = 0  # Initial x position in meters
y = 0  # Initial y position in meters
z = 0  # Initial z position in meters
Throttle_Setpoint = 0  # Throttle set-point in %
Throttle = 0  # Throttle in %
Max_thrust = 75  # Maximum thrust in N
thrust = 0  # Thrust in N
x_velocity = 0  # Initial x velocity in m/s
y_velocity = 0  # Initial y velocity in m/s
z_velocity = 0  # Initial z velocity in m/s
rotational_velocity = [0, 0]  # Initial rotational velocities in rad/s
vector_angles = [0, 0]  # Initial vector angles in degrees
vector_angles_setpoint = [0, 0]  # Initial vector angles set-point in degrees
thrust_vectors = [0, 0, 0]  # Thrust vectors
acc_vector = []  # Acceleration vector
gravity_acceleration = -9.81  # Gravity acceleration in m/s^2
launch_angle_x_degrees = 5  # Launch angle in degrees for x-axis
launch_angle_y_degrees = -3  # Launch angle in degrees for y-axis

gimbal_rate = 20  # Gimbal rate in deg/s
throttle_rate = 30  # Throttle rate in %/s
x_gimbal_limit = 20  # Gimbal limit for x-axis in degrees
y_gimbal_limit = 20  # Gimbal limit for y-axis in degrees

# PID Controller Gains
x_axis_P = 0.11 # Proportional gain for x-axis
x_axis_I = 0.01 # Integral gain for x-axis
x_axis_D = 0.04 # Derivative gain for x-axis

y_axis_P = 0.11 # Proportional gain for y-axis
y_axis_I = 0.01 # Integral gain for y-axis
y_axis_D = 0.04 # Derivative gain for y-axis

throttle_P = 0 # Proportional gain for throttle
throttle_I = 0 # Integral gain for throttle
throttle_D = 0 # Derivative gain for throttle

# Controller parameters
Controller_delay = 0.05 # Controller delay in seconds
Control_cylce_time = 0.08 # Control cycle time in seconds
Delayed_Inputs = [[], []]


# PID Controller Integrals
x_integral = 0
y_integral = 0
throttle_integral = 0

# Plotting Parameters
plot_frequency = 200
plot_frequency_tvc = 100
Plot_Counter = 0
Plot_Counter2 = 0
Plot_Counter3 = 0

# Ground Check
has_left_ground = False

# Simulation Loop
while z > -0.01:
    Plot_Counter += 1
    Plot_Counter2 += 1
    Plot_Counter3 += 1

    if z > 0:
        has_left_ground = True
    else:
        has_left_ground = False
        x_velocity = 0

    x_integral += launch_angle_x_degrees * step
    y_integral += launch_angle_y_degrees * step
    throttle_integral += (ascent_rate(time) - z_velocity) * step

    for i in range(2):
        if vector_angles[i] < vector_angles_setpoint[i]:
            vector_angles[i] += gimbal_rate * step
        elif vector_angles[i] > vector_angles_setpoint[i]:
            vector_angles[i] -= gimbal_rate * step

    if PID_Control:
        delayed_x = PID(x_axis_P, x_axis_I, x_axis_D, launch_angle_x_degrees, rotational_velocity[0], x_integral)
        delayed_y = -PID(y_axis_P, y_axis_I, y_axis_D, launch_angle_y_degrees, rotational_velocity[1], y_integral)

        # Append to the respective lists
        Delayed_Inputs[0].append(delayed_x)
        Delayed_Inputs[1].append(delayed_y)

        delay_index = int(time * (1 / step)) - int(Controller_delay * (1 / step))
        if 0 <= delay_index < len(Delayed_Inputs[0]):
            if (time % Control_cylce_time) < 1e-4:
                vector_angles_setpoint[0] = Delayed_Inputs[0][delay_index]
                vector_angles_setpoint[1] = Delayed_Inputs[1][delay_index]
        else:
            vector_angles_setpoint[0] = 0  # fallback value
            vector_angles_setpoint[1] = 0  # fallback value


    vector_angles[0] = max(min(vector_angles[0], x_gimbal_limit), -x_gimbal_limit)
    vector_angles[1] = max(min(vector_angles[1], y_gimbal_limit), -y_gimbal_limit)

    if Throttle_control:
        Throttle_Setpoint = min(PID(throttle_P, throttle_I, throttle_D, ascent_rate(time) - z_velocity, thrust_vectors[2], throttle_integral), 100)
        if Throttle < Throttle_Setpoint:
            Throttle += throttle_rate * step
        elif Throttle > Throttle_Setpoint:
            Throttle -= throttle_rate * step
        thrust = Throttle / 100 * Max_thrust
    else:
        thrust = Thrust(time)



    thrust_vectors = force_vectors(vector_angles, thrust)
    rotational_velocity[0] += math.degrees(change_in_rotational_velocity(thrust_vectors[0], cg_to_gimbal_distance, mass, length, step, radius))
    rotational_velocity[1] += math.degrees(change_in_rotational_velocity(thrust_vectors[1], cg_to_gimbal_distance, mass, length, step, radius))

    launch_angle_x_degrees += rotational_velocity[0] * step
    launch_angle_y_degrees += rotational_velocity[1] * step

    print("Simulation at t: " + format(time, '.4f') + "s")
    if Thrust(time) > 0:
        acc_vector = force_vectors([launch_angle_x_degrees, launch_angle_y_degrees], thrust_vectors[2])
        color = 'bo'
    else:
        max_thrust = 0
        acc_vector = [0, 0, 0]
        color = 'ro'

    if Plot_Counter == plot_frequency:
        ax.plot([x], [y], [z], color)
        Plot_Counter = 0
    if Plot_Counter3 == plot_frequency_tvc:
        axis[0, 0].plot([time], [Thrust(time)], color)
        axis[0, 1].plot([time], [vector_angles[0]], color)
        axis[1, 0].plot([time], [vector_angles[1]], color)
        axis[1, 1].plot([time], z_velocity, color)
        Plot_Counter3 = 0

    if Plot_Counter2 == plot_frequency:
        x_angle_plot.plot([time], [launch_angle_x_degrees], color)
        y_angle_plot.plot([time], [launch_angle_y_degrees], color)
        altitude_plot.plot([time], [z], color)

        axis2[0, 0].plot([time], [x_velocity], color)
        axis2[0, 1].plot([time], [y_velocity], color)
        axis2[1, 0].plot([time], [x],  color)
        axis2[1, 1].plot([time], [y], color)
        Plot_Counter2 = 0

    z_velocity += acc_vector[2] / mass * step
    x_velocity += acc_vector[0] / mass * step
    y_velocity += acc_vector[1] / mass * step

    drag_vector = Drag_vectors([launch_angle_x_degrees,launch_angle_y_degrees], [x_velocity, y_velocity, z_velocity], Side_Area, Frontal_Area, Cd, z)

    x_velocity += drag_vector[0] / mass * step
    y_velocity += drag_vector[1] / mass * step
    z_velocity += drag_vector[2] / mass * step


    if has_left_ground:
        z_velocity += gravity_acceleration * step

    x += x_velocity * step
    y += y_velocity * step
    z += z_velocity * step

    time += step

# Plot Labels and Titles
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Rocket Trajectory')

x_angle_plot.set_xlabel('Time')
x_angle_plot.set_ylabel('X Angle')

y_angle_plot.set_xlabel('Time')
y_angle_plot.set_ylabel('Y Angle')

altitude_plot.set_xlabel('Time')
altitude_plot.set_ylabel('Altitude (m)')

axis[0,0].set_xlabel('Time')
axis[0,0].set_ylabel('Throttle')

axis[0,1].set_xlabel('Time')
axis[0,1].set_ylabel('X Vector Angle')

axis[1,0].set_xlabel('Time')
axis[1,0].set_ylabel('Y Vector Angle')

axis[1,1].set_xlabel('Time')
axis[1,1].set_ylabel('Z Velocity')

axis2[0,0].set_ylim([-1, 1])
axis2[0,1].set_ylim([-1, 1])
axis2[1,0].set_ylim([-1, 1])
axis2[1,1].set_ylim([-1, 1])

axis2[0,0].set_xlabel('Time')
axis2[0,0].set_ylabel('X Velocity')

axis2[0,1].set_xlabel('Time')
axis2[0,1].set_ylabel('Y Velocity')

axis2[1,0].set_xlabel('Time')
axis2[1,0].set_ylabel('X Position')

axis2[1,1].set_xlabel('Time')
axis2[1,1].set_ylabel('Y Position')

plt.show()
