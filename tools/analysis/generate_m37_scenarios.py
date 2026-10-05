"""
Breadcrumb Trek 2.x - M3.7 Controlled Scenario Data Generator
Generates high-fidelity sensor recordings for Scenarios A through G adhering to:
- CSV schema: timestamp_ns,sensor_type,x,y,z
- POCO X6 Pro sensor sampling rates (~52.8 Hz Accel, ~52.7 Hz Gyro, ~19.9 Hz Mag)
- Sensor units: m/s², rad/s, µT
- Earth frame NWU to Android body frame rotations
"""

import numpy as np
import os
import math

def euler_to_quat(tilt_pitch, yaw_bearing):
    """
    Computes quaternion q (w, x, y, z) rotating Body to Earth NWU.
    yaw_bearing: compass bearing clockwise from North (ENU) in radians.
    tilt_pitch: phone pitch up towards user (rotation around body X) in radians.
    In NWU: x=North, y=West, z=Up.
    Phone forward (+y_b) points North when NWU yaw = -yaw_bearing - pi/2.
    """
    psi_nwu = -yaw_bearing - math.pi * 0.5
    
    # Rotation about Earth Z (NWU up)
    cy = math.cos(psi_nwu * 0.5)
    sy = math.sin(psi_nwu * 0.5)
    qz = np.array([cy, 0.0, 0.0, sy], dtype=np.float64)
    
    # Tilt around body X (pitch up towards user)
    cp = math.cos(tilt_pitch * 0.5)
    sp = math.sin(tilt_pitch * 0.5)
    qx = np.array([cp, sp, 0.0, 0.0], dtype=np.float64)
    
    # q = qz * qx
    w = qz[0]*qx[0] - qz[1]*qx[1] - qz[2]*qx[2] - qz[3]*qx[3]
    x = qz[0]*qx[1] + qz[1]*qx[0] + qz[2]*qx[3] - qz[3]*qx[2]
    y = qz[0]*qx[2] - qz[1]*qx[3] + qz[2]*qx[0] + qz[3]*qx[1]
    z = qz[0]*qx[3] + qz[1]*qx[2] - qz[2]*qx[1] + qz[3]*qx[0]
    
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/norm, x/norm, y/norm, z/norm)

def quat_to_rot_matrix(q):
    """3x3 rotation matrix R mapping Body to Earth NWU: v_earth = R * v_body"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ], dtype=np.float64)

def generate_scenario(name, duration_s, trajectory_fn, out_path):
    t0_ns = 514300000000000  # realistic POCO nano timestamp
    
    dt_acc = 1.0 / 52.8
    dt_gyr = 1.0 / 52.7
    dt_mag = 1.0 / 19.9
    
    # POCO local magnetic field in Earth NWU:
    # x = North, y = West, z = Up.
    B_earth_nwu = np.array([29.34, 0.0, -29.34], dtype=np.float64)
    g_earth_nwu = np.array([0.0, 0.0, 9.80665], dtype=np.float64)
    
    events = [] # (t_ns, type, x, y, z)
    
    # Generate Accel events
    t = 0.0
    while t <= duration_s:
        t_ns = t0_ns + int(t * 1e9)
        state = trajectory_fn(t)
        
        pitch = state['phone_pitch']
        yaw = state['phone_yaw_enu']
        q = euler_to_quat(pitch, yaw)
        R = quat_to_rot_matrix(q) # Body to Earth NWU
        R_inv = R.T              # Earth NWU to Body
        
        # Earth linear accel ENU -> NWU:
        # In ENU: x=East, y=North, z=Up.
        # In NWU: x=North, y=West=-East, z=Up.
        aE, aN, aU = state['a_linear_earth_enu']
        a_lin_nwu = np.array([aN, -aE, aU], dtype=np.float64)
        
        # Specific force in body frame
        f_earth = a_lin_nwu + g_earth_nwu
        f_body = R_inv @ f_earth
        
        noise = np.random.normal(0, 0.012, 3)
        acc_meas = f_body + noise
        
        events.append((t_ns, 1, float(acc_meas[0]), float(acc_meas[1]), float(acc_meas[2])))
        t += dt_acc
        
    # Generate Gyro events
    t = 0.005
    while t <= duration_s:
        t_ns = t0_ns + int(t * 1e9)
        state = trajectory_fn(t)
        
        pitch = state['phone_pitch']
        yaw = state['phone_yaw_enu']
        q = euler_to_quat(pitch, yaw)
        R = quat_to_rot_matrix(q)
        R_inv = R.T
        
        wE, wN, wU = state['omega_earth_enu']
        w_nwu = np.array([wN, -wE, wU], dtype=np.float64)
        w_body = R_inv @ w_nwu
        
        noise = np.random.normal(0, 0.002, 3)
        gyr_meas = w_body + noise
        
        events.append((t_ns, 4, float(gyr_meas[0]), float(gyr_meas[1]), float(gyr_meas[2])))
        t += dt_gyr

    # Generate Mag events
    t = 0.010
    while t <= duration_s:
        t_ns = t0_ns + int(t * 1e9)
        state = trajectory_fn(t)
        
        pitch = state['phone_pitch']
        yaw = state['phone_yaw_enu']
        q = euler_to_quat(pitch, yaw)
        R = quat_to_rot_matrix(q)
        R_inv = R.T
        
        m_body = R_inv @ B_earth_nwu
        noise = np.random.normal(0, 0.15, 3)
        mag_meas = m_body + noise
        
        events.append((t_ns, 2, float(mag_meas[0]), float(mag_meas[1]), float(mag_meas[2])))
        t += dt_mag

    # Sort strictly by timestamp
    events.sort(key=lambda x: x[0])
    
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("timestamp_ns,sensor_type,x,y,z\n")
        for e in events:
            f.write(f"{e[0]},{e[1]},{e[2]:.6f},{e[3]:.6f},{e[4]:.6f}\n")
            
    print(f"Generated {name}: {len(events)} samples -> {out_path}")

# ==============================================================================
# SCENARIO TRAJECTORY DEFINITIONS
# ==============================================================================

def scenario_A_straight(t):
    """Scenario A: Straight walk / normal handheld texting posture facing North"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8 # Hz
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aFwd = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    # Lateral sway foot alternation at 0.9 Hz
    aLat = 0.4 * math.sin(2.0 * math.pi * (cadence * 0.5) * t_walk)
    
    # Minor hand sway yaw +-4 deg
    sway_yaw = math.radians(4.0) * math.sin(2.0 * math.pi * 0.9 * t_walk)
    sway_w = math.radians(4.0) * (2.0 * math.pi * 0.9) * math.cos(2.0 * math.pi * 0.9 * t_walk)
    
    return {
        'is_stationary': False,
        'travel_course_enu': 0.0,
        'phone_yaw_enu': sway_yaw,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aLat, aFwd, aU),
        'omega_earth_enu': (0.0, 0.0, -sway_w) # NWU up is negative ENU yaw
    }

def scenario_B_phone_yaw(t):
    """Scenario B: Straight walk North while deliberately rotating phone +-55 deg"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aFwd = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    aLat = 0.4 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    # Deliberate yaw swing +-55 deg at 0.35 Hz
    f_swing = 0.35
    amp_yaw = math.radians(55.0)
    phone_yaw = amp_yaw * math.sin(2.0 * math.pi * f_swing * t_walk)
    w_yaw = amp_yaw * (2.0 * math.pi * f_swing) * math.cos(2.0 * math.pi * f_swing * t_walk)
    
    return {
        'is_stationary': False,
        'travel_course_enu': 0.0, # Genuine travel remains strictly North
        'phone_yaw_enu': phone_yaw,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aLat, aFwd, aU),
        'omega_earth_enu': (0.0, 0.0, -w_yaw)
    }

def scenario_C_turn90(t):
    """Scenario C: Walk North 8s, 90 deg turn to East over 2.2s (~4 steps), walk East 8s"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aFwd = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    aLat = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    if t_walk < 8.0:
        # Leg 1: North
        course = 0.0
        w_turn = 0.0
    elif t_walk < 10.2:
        # Turn from 0 to +pi/2 over 2.2s
        turn_progress = (t_walk - 8.0) / 2.2
        course = turn_progress * (math.pi * 0.5)
        w_turn = (math.pi * 0.5) / 2.2
    else:
        # Leg 2: East (+pi/2)
        course = math.pi * 0.5
        w_turn = 0.0
        
    # Rotate forward/lateral accelerations into Earth ENU
    # In ENU: forward is [sin(course), cos(course)], lateral right is [cos(course), -sin(course)]
    aE = aFwd * math.sin(course) + aLat * math.cos(course)
    aN = aFwd * math.cos(course) - aLat * math.sin(course)
    
    return {
        'is_stationary': False,
        'travel_course_enu': course,
        'phone_yaw_enu': course,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aE, aN, aU),
        'omega_earth_enu': (0.0, 0.0, -w_turn)
    }

def scenario_D_reverse180(t):
    """Scenario D: Walk North 9s, smooth 180 deg reversal over 2.8s (~5 steps), walk South 9s"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aFwd = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    aLat = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    if t_walk < 9.0:
        # Leg 1: North (0 rad)
        course = 0.0
        w_turn = 0.0
    elif t_walk < 11.8:
        # Smooth S-curve transition 0 -> pi over 2.8s
        u = (t_walk - 9.0) / 2.8
        course = math.pi * 0.5 * (1.0 - math.cos(math.pi * u))
        w_turn = math.pi * 0.5 * (math.pi / 2.8) * math.sin(math.pi * u)
    else:
        # Leg 2: South (+pi rad)
        course = math.pi
        w_turn = 0.0
        
    aE = aFwd * math.sin(course) + aLat * math.cos(course)
    aN = aFwd * math.cos(course) - aLat * math.sin(course)
    
    return {
        'is_stationary': False,
        'travel_course_enu': course,
        'phone_yaw_enu': course,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aE, aN, aU),
        'omega_earth_enu': (0.0, 0.0, -w_turn)
    }

def scenario_E_stop_rotate_resume(t):
    """Scenario E: Walk North 6.5s, Stop 3.5s (rotate phone 90 deg East while stopped), Resume North 6.5s"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_event = t - 2.0
    if t_event < 6.5:
        # Leg 1: Active walking North
        cadence = 1.8
        w_step = 2.0 * math.pi * cadence
        aU = 3.2 * math.sin(w_step * t_event)
        aN = 1.5 * math.sin(w_step * t_event - math.pi * 0.5)
        aE = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_event)
        return {
            'is_stationary': False,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (aE, aN, aU),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    elif t_event < 10.0:
        # STOPPED: 3.5s stationary hold.
        # Between t_event=7.5 and 9.0 (1.5s): user rotates phone 90 deg clockwise to East
        if t_event < 7.5:
            yaw = 0.0
            w_yaw = 0.0
        elif t_event < 9.0:
            prog = (t_event - 7.5) / 1.5
            yaw = prog * (math.pi * 0.5)
            w_yaw = (math.pi * 0.5) / 1.5
        else:
            yaw = math.pi * 0.5
            w_yaw = 0.0
            
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': yaw,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, -w_yaw)
        }
    else:
        # Leg 2: Resume walking North (phone held at 90 deg East)
        t_walk2 = t_event - 10.0
        cadence = 1.8
        w_step = 2.0 * math.pi * cadence
        aU = 3.2 * math.sin(w_step * t_walk2)
        aN = 1.5 * math.sin(w_step * t_walk2 - math.pi * 0.5)
        aE = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_walk2)
        return {
            'is_stationary': False,
            'travel_course_enu': 0.0, # True travel is still North!
            'phone_yaw_enu': math.pi * 0.5, # Phone points East (+90 deg)
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (aE, aN, aU),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }

def scenario_F_offset45(t):
    """Scenario F1: Walk North with phone held at fixed +45 deg yaw offset"""
    phone_offset = math.radians(45.0)
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': phone_offset,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aN = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    aE = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    sway = math.radians(2.0) * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    return {
        'is_stationary': False,
        'travel_course_enu': 0.0,
        'phone_yaw_enu': phone_offset + sway,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aE, aN, aU),
        'omega_earth_enu': (0.0, 0.0, 0.0)
    }

def scenario_F_offset90(t):
    """Scenario F2: Walk North with phone held at fixed +90 deg yaw offset"""
    phone_offset = math.radians(90.0)
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': phone_offset,
            'phone_pitch': math.radians(45.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 3.2 * math.sin(w_step * t_walk)
    aN = 1.5 * math.sin(w_step * t_walk - math.pi * 0.5)
    aE = 0.3 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    sway = math.radians(2.0) * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    return {
        'is_stationary': False,
        'travel_course_enu': 0.0,
        'phone_yaw_enu': phone_offset + sway,
        'phone_pitch': math.radians(45.0),
        'phone_roll': 0.0,
        'a_linear_earth_enu': (aE, aN, aU),
        'omega_earth_enu': (0.0, 0.0, 0.0)
    }

def scenario_G_posture(t):
    """Scenario G: Varied posture - dangling carry at side with arm swing dynamics"""
    if t < 2.0:
        return {
            'is_stationary': True,
            'travel_course_enu': 0.0,
            'phone_yaw_enu': 0.0,
            'phone_pitch': math.radians(75.0),
            'phone_roll': 0.0,
            'a_linear_earth_enu': (0.0, 0.0, 0.0),
            'omega_earth_enu': (0.0, 0.0, 0.0)
        }
    t_walk = t - 2.0
    cadence = 1.8
    w_step = 2.0 * math.pi * cadence
    aU = 2.8 * math.sin(w_step * t_walk)
    aN = 1.2 * math.sin(w_step * t_walk - math.pi * 0.5)
    aE = 0.5 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    
    # Arm pendulum swing at 0.9 Hz
    arm_swing_pitch = math.radians(75.0) + math.radians(18.0) * math.sin(2.0 * math.pi * 0.9 * t_walk)
    arm_swing_roll = math.radians(8.0) * math.cos(2.0 * math.pi * 0.9 * t_walk)
    w_pitch = math.radians(18.0) * (2.0 * math.pi * 0.9) * math.cos(2.0 * math.pi * 0.9 * t_walk)
    
    # Dynamic arm swing adds tangential and centripetal acceleration
    a_arm_fwd = 0.8 * math.sin(2.0 * math.pi * 0.9 * t_walk)
    aN += a_arm_fwd
    
    return {
        'is_stationary': False,
        'travel_course_enu': 0.0,
        'phone_yaw_enu': 0.0,
        'phone_pitch': arm_swing_pitch,
        'phone_roll': arm_swing_roll,
        'a_linear_earth_enu': (aE, aN, aU),
        'omega_earth_enu': (0.0, w_pitch, 0.0)
    }

def main():
    np.random.seed(42) # Deterministic repeatable generation
    data_dir = r"D:\IQOO Code\data"
    
    scenarios = [
        ("m37_A_straight", 18.0, scenario_A_straight),
        ("m37_B_phone_yaw", 18.0, scenario_B_phone_yaw),
        ("m37_C_turn90", 19.0, scenario_C_turn90),
        ("m37_D_reverse180", 21.5, scenario_D_reverse180),
        ("m37_E_stop_rotate_resume", 17.0, scenario_E_stop_rotate_resume),
        ("m37_F_phone_offset45", 17.0, scenario_F_offset45),
        ("m37_F_phone_offset90", 17.0, scenario_F_offset90),
        ("m37_G_varied_posture", 17.0, scenario_G_posture),
    ]
    
    for name, dur, fn in scenarios:
        out_path = os.path.join(data_dir, f"{name}.csv")
        generate_scenario(name, dur, fn, out_path)
        
    print("\nAll M3.7 controlled scenario datasets generated successfully!")

if __name__ == "__main__":
    main()

