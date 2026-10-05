import math
import numpy as np

def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def run_sim(max_change_deg=45.0, div_thresh_deg=140.0, persistence=3, alpha=0.35, beta=0.05):
    div_thresh_rad = math.radians(div_thresh_deg)
    max_turn_rad = math.radians(max_change_deg)
    std_max_turn_rad = math.radians(25.0)

    # We will simulate the observer given the known step properties of Scenario D
    # 35 steps total:
    # Steps 1-16: North (phone=0°, PCA=0°, stride=0.663m)
    # Steps 17-21: Transition U-turn
    # Steps 22-35: South (phone=180°, PCA=180° or 0°, stride=0.663m)
    
    # Ground truth phone angles for the 35 steps
    step_times = [2.0 + i * (1.0/1.8) for i in range(35)]
    phone_headings = []
    pca_angles = []
    pca_confs = []
    
    for i, t in enumerate(step_times):
        t_walk = t - 2.0
        if t_walk < 9.0:
            course = 0.0
        elif t_walk < 11.8:
            u = (t_walk - 9.0) / 2.8
            course = math.pi * 0.5 * (1.0 - math.cos(math.pi * u))
        else:
            course = math.pi
        phone_headings.append(course)
        # PCA identifies the horizontal acceleration axis
        pca_angles.append(course)
        pca_confs.append(0.92)

    course = 0.0
    consecutive_div = 0
    reversal_active = False
    reversal_step = -1
    steps_to_converge = -1
    
    east = 0.0
    north = 0.0
    stride = 0.663
    
    history = []
    
    for s in range(35):
        phone_rad = phone_headings[s]
        pca_axis = pca_angles[s]
        pca_conf = pca_confs[s]
        
        # Check divergence from current course
        div = abs(wrap_pi(phone_rad - course))
        if div > div_thresh_rad:
            consecutive_div += 1
        else:
            consecutive_div = 0
            
        if consecutive_div >= persistence and pca_conf >= 0.20:
            if not reversal_active:
                reversal_active = True
                reversal_step = s + 1
                
        # Resolve PCA sign
        psi_ref = wrap_pi(course + math.pi) if reversal_active else course
        d_plus = abs(wrap_pi(pca_axis - psi_ref))
        d_minus = abs(wrap_pi(pca_axis + math.pi - psi_ref))
        oriented_pca = pca_axis if d_plus < d_minus else wrap_pi(pca_axis + math.pi)
        
        d_pca = wrap_pi(oriented_pca - course)
        d_phone = wrap_pi(phone_rad - course)
        
        # If reversal is active and d_pca is near +-pi, align sign with d_phone
        if reversal_active and abs(abs(d_pca) - math.pi) < 0.2:
            d_pca = math.copysign(abs(d_pca), d_phone)
            
        limit = max_turn_rad if reversal_active else std_max_turn_rad
        
        # Innovation
        innov = alpha * pca_conf * d_pca + beta * d_phone
        innov = max(-limit, min(limit, innov))
        
        new_course = wrap_pi(course + innov)
        course = new_course
        
        # Check exit
        if reversal_active:
            if abs(wrap_pi(phone_rad - course)) < math.radians(35.0):
                reversal_active = False
                consecutive_div = 0
                if steps_to_converge < 0:
                    steps_to_converge = (s + 1) - reversal_step
                    
        east += stride * math.sin(course)
        north += stride * math.cos(course)
        history.append((s+1, math.degrees(phone_rad), math.degrees(course), east, north))
        
    net_disp = math.hypot(east, north)
    return {
        'max_change_deg': max_change_deg,
        'reversal_step': reversal_step,
        'steps_to_converge': steps_to_converge,
        'final_east': east,
        'final_north': north,
        'net_disp': net_disp,
        'final_course_deg': math.degrees(course)
    }

for limit in [25.0, 35.0, 45.0, 60.0, 90.0]:
    r = run_sim(max_change_deg=limit)
    print(f"Limit {limit:4.1f}°/step: RevStep={r['reversal_step']}, StepsToConv={r['steps_to_converge']}, Final E={r['final_east']:6.2f}m, N={r['final_north']:6.2f}m, Net={r['net_disp']:5.2f}m, FinalCourse={r['final_course_deg']:.1f}°")
