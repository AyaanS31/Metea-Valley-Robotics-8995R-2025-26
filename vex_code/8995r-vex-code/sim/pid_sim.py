"""
Simple discrete-time PID simulator for linear and angular motion.
Writes a CSV `sim/output.csv` and prints short summary metrics.

This is deliberately dependency-free (standard library only) so it runs on any Python.
"""

from math import fabs
import csv


def run_sim():
    # Simulation parameters
    dt = 0.02  # nominal 20 ms loop
    sim_time = 6.0  # seconds

    # Linear robot model parameters (very simple 1D model)
    target_dist = 24.0  # inches
    motor_limit = 100.0  # controller output units (percent-like)
    max_accel = 60.0  # inches/s^2 at motor_limit (tunable, increased to represent stronger motors)
    friction = 1.2  # viscous damping coefficient

    # PID gains from your C++ (linear)
    kP_linear = 0.75
    kI_linear = 0.0
    kD_linear = 0.3
    integral_max = 50.0
    linear_threshold = 2.0
    settle_time_required = 3.0
    min_command = 10.0

    # Initial state
    pos = 0.0
    vel = 0.0
    integral = 0.0
    prev_error = target_dist - pos

    time = 0.0
    settle_timer = 0.0
    max_overshoot = 0.0

    rows = []
    rows.append(["time","pos","vel","error","output"])


    while time < sim_time:
        # use measured dt (simulate loop jitter minimally)
        error = target_dist - pos

        # P/I/D with same rules as C++
        if fabs(error) < linear_threshold * 2.0:
            integral += error * dt
        else:
            integral = 0.0
        integral = max(-integral_max, min(integral, integral_max))
        derivative = (error - prev_error) / dt

        output = kP_linear * error + kI_linear * integral + kD_linear * derivative

        # enforce min command to overcome stiction
        if abs(output) > 0 and abs(output) < min_command:
            output = (output / abs(output)) * min_command

        # clamp
        output = max(-motor_limit, min(output, motor_limit))

        # simple actuator model: acceleration proportional to command
        accel = (output / motor_limit) * max_accel - friction * vel
        vel += accel * dt
        pos += vel * dt

        # metrics
        if pos - target_dist > max_overshoot:
            max_overshoot = pos - target_dist

        if abs(error) < linear_threshold:
            settle_timer += dt
        else:
            settle_timer = 0.0

        rows.append([round(time,3), pos, vel, error, output])

        if settle_timer >= settle_time_required:
            # Considered settled; stop simulation early
            time += dt
            break

        prev_error = error
        time += dt

    settled = (settle_timer >= settle_time_required)

    # Save CSV for inspection
    out_path = 'sim/output_linear.csv'
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerows(rows)

    # Summary
    summary = {
        'mode': 'linear',
        'target': target_dist,
        'final_pos': pos,
        'final_error': target_dist - pos,
        'max_overshoot': max_overshoot,
        'settled': settled,
        'sim_time': time
    }

    # Run angular simulation (degrees) with your turn gains
    # We treat output as motor command that creates angular acceleration
    dt = 0.02
    sim_time = 3.0
    target_angle = 90.0  # degrees
    motor_limit = 100.0
    max_ang_accel = 400.0  # degrees/s^2 at motor_limit
    ang_friction = 5.0  # damping

    kP_turn = 59.4
    kI_turn = 0.001
    kD_turn = 2.95
    ang_integral_max = 0.5
    ang_threshold_deg = 1.0
    ang_settle_req = 0.3

    angle = 0.0
    ang_vel = 0.0
    integral = 0.0
    prev_err = target_angle - angle
    time = 0.0
    settle_timer = 0.0
    max_overshoot_ang = 0.0
    rows_ang = []
    rows_ang.append(["time","angle","ang_vel","error","output"])

    while time < sim_time:
        err = target_angle - angle
        # wrap to [-180,180]
        while err > 180.0: err -= 360.0
        while err < -180.0: err += 360.0

        if abs(err) < ang_threshold_deg * 2.0:
            integral += err * dt
        else:
            integral = 0.0
        integral = max(-ang_integral_max, min(integral, ang_integral_max))

        derivative = (err - prev_err) / dt
        output = kP_turn * err + kI_turn * integral + kD_turn * derivative
        output = max(-motor_limit, min(output, motor_limit))

        ang_accel = (output / motor_limit) * max_ang_accel - ang_friction * ang_vel
        ang_vel += ang_accel * dt
        angle += ang_vel * dt

        # compute overshoot
        if angle - target_angle > max_overshoot_ang:
            max_overshoot_ang = angle - target_angle

        if abs(err) < ang_threshold_deg:
            settle_timer += dt
        else:
            settle_timer = 0.0

        rows_ang.append([round(time,3), angle, ang_vel, err, output])

        if settle_timer >= ang_settle_req:
            time += dt
            break

        prev_err = err
        time += dt

    settled_ang = (settle_timer >= ang_settle_req)
    out_path2 = 'sim/output_angular.csv'
    with open(out_path2, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerows(rows_ang)

    summary_ang = {
        'mode': 'angular',
        'target': target_angle,
        'final_angle': angle,
        'final_error': target_angle - angle,
        'max_overshoot': max_overshoot_ang,
        'settled': settled_ang,
        'sim_time': time
    }

    return summary, summary_ang


if __name__ == '__main__':
    s_lin, s_ang = run_sim()
    print('Linear summary:', s_lin)
    print('Angular summary:', s_ang)
