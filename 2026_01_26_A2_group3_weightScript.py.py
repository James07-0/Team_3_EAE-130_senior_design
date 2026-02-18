
import numpy as np

def empty_weight_fraction(W0):
   
    A = 2.34
    C = -0.13
    return A * (W0 ** C)

def fuel_fraction_cruise(range_nmi, speed_fps, sfc, L_D):
   
    range_ft = range_nmi * 6076.12  # convert nmi to feet
    sfc_per_sec = sfc / 3600.0      # convert per hour to per second
    return np.exp(-range_ft * sfc_per_sec / (speed_fps * L_D))

def fuel_fraction_loiter(endurance_hrs, sfc, L_D):
 
    return np.exp(-endurance_hrs * sfc / L_D)

def estimate_weight(W_crew, W_payload, cruise_range, cruise_speed, cruise_LD, 
                   loiter_time, loiter_LD, sfc, W0_guess=50000):
 
    
    # Mission segment fractions (warmup, takeoff, climb, descent, landing)
    W1_W0 = 0.97   # warmup/takeoff
    W2_W1 = 0.985  # climb
    W5_W4 = 0.995  # descent/landing
    
    W0 = W0_guess
    tolerance = 1e-6
    error = 1.0
    iterations = 0
    
    while error > tolerance and iterations < 100:
        # Empty weight fraction
        We_W0 = empty_weight_fraction(W0)
        
        # Fuel fractions
        W3_W2 = fuel_fraction_cruise(cruise_range, cruise_speed, sfc, cruise_LD)
        W4_W3 = fuel_fraction_loiter(loiter_time, sfc, loiter_LD)
        
        # Total mission fuel fraction
        W6_W0 = W1_W0 * W2_W1 * W3_W2 * W4_W3 * W5_W4
        Wf_W0 = 1.06 * (1 - W6_W0)  # 6% reserve
    
        W0_new = (W_crew + W_payload) / (1 - Wf_W0 - We_W0)
        
        # Check convergence
        error = abs((W0_new - W0) / W0_new)
        W0 = W0_new
        iterations += 1
    
    We = We_W0 * W0
    Wf = Wf_W0 * W0
    
    return W0, We, Wf, iterations


# Ordnance weights (lbs)
aim120c = 335
aim9x = 190
mk83_jdam = 1100 #estimation with 1000lb plus guidance system 

strike_ordnance = 4*mk83_jdam + 2*aim9x 

# Inputs
W_crew = 200        # pilot 
W_avionics = 2500    # from RFP
W_payload = W_avionics + strike_ordnance

# Concept 1 twin engine
print("\n Twin Engine")


W0_1, We_1, Wf_1, iters_1 = estimate_weight(
    W_crew=W_crew,
    W_payload=W_payload,
    cruise_range= 1000,   #nmi
    cruise_speed=850,    # ft/s
    cruise_LD=8,      
    loiter_time=0.333,   # hrs 
    loiter_LD=8,        
    sfc=0.95,          
    W0_guess=60000
)

print(f"Converged in {iters_1} iterations")
print(f"TOGW:         {W0_1:,.0f} lbs")
print(f"Empty Weight: {We_1:,.0f} lbs ({We_1/W0_1:.3f})")
print(f"Fuel Weight:  {Wf_1:,.0f} lbs ({Wf_1/W0_1:.3f})")

# Concept 2 singleengine
print("\n Single engine")


W0_2, We_2, Wf_2, iters_2 = estimate_weight(
    W_crew=W_crew,
    W_payload=W_payload,
    cruise_range=1000,   # nmi 
    cruise_speed=850,    # ft/s
    cruise_LD=8,        
    loiter_time=0.333,   # hrs
    loiter_LD=8,        
    sfc=0.85,            # More efficient engine
    W0_guess=65000
)

print(f"Converged in {iters_2} iterations")
print(f"TOGW:         {W0_2:,.0f} lbs")
print(f"Empty Weight: {We_2:,.0f} lbs ({We_2/W0_2:.3f})")
print(f"Fuel Weight:  {Wf_2:,.0f} lbs ({Wf_2/W0_2:.3f})")

