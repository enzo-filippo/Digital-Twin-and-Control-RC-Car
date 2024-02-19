import numpy as np

def calculate_drifting_force(slip_angle, v, c):
    """
    Calculate the drifting (lateral) force for a vehicle.
    
    Parameters:
    - slip_angle: The slip angle of the tire (in radians).
    - v: Velocity of the vehicle (m/s).
    - c: Tire-road interaction coefficient.

    Returns:
    - F_drift: The magnitude of the drifting force (N).
    """
    F_drift = c * slip_angle * v
    return F_drift

def update_vehicle_state_with_drift(x, y, theta, v, delta, L, dt, c):
    """
    Update vehicle state based on the bicycle model incorporating drifting forces.

    Parameters:
    - x, y: Current position of the vehicle.
    - theta: Current orientation of the vehicle (in radians).
    - v: Velocity of the vehicle.
    - delta: Steering angle of the front wheels (in radians).
    - L: Wheelbase of the vehicle.
    - dt: Time step for the update.
    - c: Tire-road interaction coefficient for calculating drifting forces.

    Returns:
    - x_new, y_new: New position of the vehicle.
    - theta_new: New orientation of the vehicle.
    """
    # Assume a simple model for slip angle calculation
    # For real applications, this would be derived from more complex dynamics
    slip_angle_front = delta
    slip_angle_rear = 0  # Assuming the rear slip angle is small for simplicity

    # Calculate drifting forces at the front and rear
    F_drift_front = calculate_drifting_force(slip_angle_front, v, c)
    F_drift_rear = calculate_drifting_force(slip_angle_rear, v, c)

    # Adjust orientation based on the drifting forces
    # This is a simplified representation; real dynamics would be more complex
    theta_new = theta + (v / L) * np.tan(delta) * dt + (F_drift_front - F_drift_rear) / L * dt
    
    # Update the position
    x_new = x + v * np.cos(theta_new) * dt
    y_new = y + v * np.sin(theta_new) * dt

    return x_new, y_new, theta_new

# Example usage
x_initial = 0.0
y_initial = 0.0
theta_initial = np.pi / 4  # 45 degrees in radians
v = 10.0  # Velocity in m/s
delta = np.pi / 18  # Steering angle of 10 degrees in radians
L = 2.5  # Wheelbase in meters
dt = 1.0  # Time step in seconds
c = 1500  # Tire-road interaction coefficient

# Update the vehicle state with drift
x_new, y_new, theta_new = update_vehicle_state_with_drift(x_initial, y_initial, theta_initial, v, delta, L, dt, c)

print(f"New position: ({x_new:.2f}, {y_new:.2f})")
print(f"New orientation: {np.degrees(theta_new):.2f} degrees")
