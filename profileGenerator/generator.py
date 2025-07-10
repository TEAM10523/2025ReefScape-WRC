import casadi as ca
import json
import matplotlib.pyplot as plt

# Number of control intervals (discretization steps)
N = 200

# Create optimization problem
opti = ca.Opti()

# ----- Decision Variable for Total Time -----
# Instead of fixing T, we now let the solver choose it.
T = opti.variable()
opti.set_initial(T, 5.0)       # initial guess
opti.subject_to(T > 0.1)         # enforce T to be positive
opti.subject_to(T <= 10.0)       # Enforcing a reasonable upper bound for time
dt = T / N                     # note: dt now depends on T

# ----- Define System Variables -----
# Elevator variables
elev_x = opti.variable(N+1)   # Position
elev_v = opti.variable(N+1)   # Velocity
elev_a = opti.variable(N+1)   # Acceleration
elev_jerk = opti.variable(N)  # Jerk (control input)

# Arm variables
arm_x = opti.variable(N+1)    # Position
arm_v = opti.variable(N+1)    # Velocity
arm_a = opti.variable(N+1)    # Acceleration
arm_jerk = opti.variable(N)   # Jerk (control input)
# Decision variables
x_elevator = opti.variable(N)  # Elevator position
x_arm = opti.variable(N)  # Arm position



# ----- System Dynamics -----
for k in range(N):
    # Elevator dynamics (using dt = T/N)
    opti.subject_to(elev_x[k+1] == elev_x[k] + elev_v[k]*dt)
    opti.subject_to(elev_v[k+1] == elev_v[k] + elev_a[k]*dt)
    opti.subject_to(elev_a[k+1] == elev_a[k] + elev_jerk[k]*dt)
    
    # Arm dynamics
    opti.subject_to(arm_x[k+1] == arm_x[k] + arm_v[k]*dt)
    opti.subject_to(arm_v[k+1] == arm_v[k] + arm_a[k]*dt)
    opti.subject_to(arm_a[k+1] == arm_a[k] + arm_jerk[k]*dt)

# ----- Robot Capability Constraints -----
# (These can be adjusted for your system.)
elevator_min_x = 0   # m
elevator_max_x = 1.51   # m
elevator_max_vel = 7   # m/s
elevator_max_acc = 12   # m/s²
elevator_max_jerk = 1000  # m/s³
w_elevator_x_cost = 0
w_elevator_v_cost = 0.0001
w_elevator_a_cost = 0

arm_min_x = 0   # m
arm_max_x = 3.2   # m
arm_max_vel = 50        # rad/s
arm_max_acc = 500.0        # rad/s²
arm_max_jerk = 1000     # rad/s³
w_arm_x_cost = 0
w_arm_v_cost = 0.0001
w_arm_a_cost = 0

waypoints = [
    (0, 1.5708), 
    (0.6, 1.75), 
    (0.744, 1.98),   

]
path_Name="Rest2L3.json"
for k in range(N+1):
    # Elevator velocity and acceleration limits 
    opti.subject_to(elev_x[k] <= elevator_max_x)
    opti.subject_to(elev_x[k] >= elevator_min_x)
    
    opti.subject_to(elev_v[k] <= elevator_max_vel)
    opti.subject_to(elev_v[k] >= -elevator_max_vel)
    opti.subject_to(elev_a[k] <= elevator_max_acc)
    opti.subject_to(elev_a[k] >= -elevator_max_acc)
    
    # Arm velocity and acceleration limits
    opti.subject_to(arm_x[k] <= arm_max_x)
    opti.subject_to(arm_x[k] >= arm_min_x)
    opti.subject_to(arm_v[k] <= arm_max_vel)
    opti.subject_to(arm_v[k] >= -arm_max_vel)
    opti.subject_to(arm_a[k] <= arm_max_acc)
    opti.subject_to(arm_a[k] >= -arm_max_acc)

for k in range(N):
    # Jerk (control) limits
    opti.subject_to(elev_jerk[k] <= elevator_max_jerk)
    opti.subject_to(elev_jerk[k] >= -elevator_max_jerk)
    opti.subject_to(arm_jerk[k] <= arm_max_jerk)
    opti.subject_to(arm_jerk[k] >= -arm_max_jerk)

# ----- Waypoint (Pass-Through) Constraints -----
# Provide a list of waypoints as (elevator_position, arm_position).
# The first and last waypoints represent the start and goal.


M = len(waypoints)
# Assign indices for each waypoint along the N+1 grid (here evenly spaced)
indices = [int(round(j * N / (M - 1))) for j in range(M)]
for idx, (elev_pt, arm_pt) in zip(indices, waypoints):
    opti.subject_to(elev_x[idx] == elev_pt)
    opti.subject_to(arm_x[idx] == arm_pt)

# Enforce boundary conditions on velocity and acceleration at the start and end.
opti.subject_to(elev_v[0] == 0.0)
opti.subject_to(elev_a[0] == 0.0)
opti.subject_to(arm_v[0] == 0.0)
opti.subject_to(arm_a[0] == 0.0)

opti.subject_to(elev_v[-1] == 0.0)
opti.subject_to(elev_a[-1] == 0.0)
opti.subject_to(arm_v[-1] == 0.0)
opti.subject_to(arm_a[-1] == 0.0)

cost_acc = (ca.sumsqr(elev_a) * w_elevator_a_cost + ca.sumsqr(arm_a) * w_arm_a_cost)
cost_v = (ca.sumsqr(elev_v) * w_elevator_v_cost + ca.sumsqr(arm_v) * w_arm_v_cost)
cost_x = (ca.sumsqr(elev_x) * w_elevator_x_cost + ca.sumsqr(arm_x) * w_arm_x_cost)


# ----- Objective -----
# Here we minimize the total time T.
cost = T + cost_acc + cost_v + cost_x
opti.minimize(cost)

# ----- Solve the Problem -----
opti.solver('ipopt')  # Use the IPOPT solver
sol = opti.solve()

# ----- Extract the Results -----
T_opt = sol.value(T)
dt_opt = T_opt / N

elev_x_opt = sol.value(elev_x)
elev_v_opt = sol.value(elev_v)
elev_a_opt = sol.value(elev_a)
elev_jerk_opt = sol.value(elev_jerk)

arm_x_opt = sol.value(arm_x)
arm_v_opt = sol.value(arm_v)
arm_a_opt = sol.value(arm_a)
arm_jerk_opt = sol.value(arm_jerk)

# Create a time array based on the optimized total time.
time = [i*dt_opt for i in range(N+1)]

# ----- Save the Results into JSON -----
profile = []

for i in range(N+1):
    # Store the position, velocity, and acceleration for both elevator and arm at each time step.
    profile.append({
        "t": time[i],
        'x0':elev_x_opt[i],
        "x1": arm_x_opt[i],
        'v0': elev_v_opt[i],
        "v1": arm_v_opt[i],
        'a0': elev_a_opt[i],
        "a1": arm_a_opt[i]
    })

# Save the profile as a JSON file
with open(path_Name, "w") as f:
    json.dump(profile, f, indent=4)

# ----- Plot the Results -----
plt.figure(figsize=(12, 8))

# Elevator Plots
plt.subplot(2, 3, 1)
plt.plot(time, elev_x_opt, marker='o')
plt.title('Elevator Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')

plt.subplot(2, 3, 2)
plt.plot(time, elev_v_opt, marker='o')
plt.title('Elevator Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

plt.subplot(2, 3, 3)
plt.plot(time, elev_a_opt, marker='o')
plt.title('Elevator Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')

# Arm Plots
plt.subplot(2, 3, 4)
plt.plot(time, arm_x_opt, marker='o')
plt.title('Arm Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')

plt.subplot(2, 3, 5)
plt.plot(time, arm_v_opt, marker='o')
plt.title('Arm Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')

plt.subplot(2, 3, 6)
plt.plot(time, arm_a_opt, marker='o')
plt.title('Arm Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (rad/s²)')

plt.tight_layout()
plt.show()

print(f"Optimized total time: {T_opt:.3f} s")
