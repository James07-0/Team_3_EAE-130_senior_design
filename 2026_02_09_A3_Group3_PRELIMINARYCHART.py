
import numpy as np
import matplotlib.pyplot as plt

# VARIABLES


# Geometry
S_ref = 972.0
AR = 2.096

# Aerodynamics - Subsonic
C_D0_subsonic = 0.0063
e_subsonic = 0.8
k_subsonic = 1.0 / (np.pi * e_subsonic * AR)

# Aerodynamics - Supersonic (Mach 2.0)
C_D0_wave = 0.0071
C_D0_supersonic = C_D0_subsonic + C_D0_wave
e_supersonic = 0.4
k_supersonic = 1.0 / (np.pi * e_supersonic * AR)

print(k_supersonic)
# High-lift configurations
C_L_max_TO = 1.4
C_L_max_land = 1.6
C_D0_takeoff = 0.0495

# Constants
g = 32.174
R = 1716.59
rho_SL = 0.002377

# Atmospheric properties function
def get_atm(h):
    if h <= 36089:
        T = 518.67 - 0.00356616 * h
        P = 2116.22 * (T / 518.67)**5.2561
    else:
        T = 389.97
        P = 2116.22 * 0.2234 * np.exp((36089 - h) / 20806)
    rho = P / (R * T)
    sigma = rho / rho_SL
    return sigma, rho, T

# Altitudes
h_SL = 0
h_climb = 0
h_cruise = 20000
h_dash = 30000
h_turn = 20000
h_ceiling = 65000

sigma_SL, rho_SL, T_SL = get_atm(h_SL)
sigma_climb, rho_climb, _ = get_atm(h_climb)
sigma_cruise, rho_cruise, _ = get_atm(h_cruise)
sigma_dash, rho_dash, T_dash = get_atm(h_dash)
sigma_turn, rho_turn, _ = get_atm(h_turn)
sigma_ceiling, rho_ceiling, _ = get_atm(h_ceiling)

# Speeds
V_app_fps = 130 * 1.688          # Approach speed: 130 knots
V_catapult_end_fps = 170         # Catapult end speed
V_climb_fps = 350 * 1.688        # Climb speed
V_cruise_fps = 500 * 1.688       # Cruise speed
V_turn_fps = 400 * 1.688         # Turn speed

# Sea level dash speed 
Mach_dash_SL = 0.9
a_SL = np.sqrt(1.4 * R * T_SL)   # Speed of sound at sea level
V_dash_SL_fps = Mach_dash_SL * a_SL

Mach_dash_high = 2.0
a_dash = np.sqrt(1.4 * R * T_dash)
V_dash_high_fps = Mach_dash_high * a_dash


C_L_ceiling = np.sqrt(C_D0_subsonic / (3 * k_subsonic))
print(C_L_ceiling)
# Climb parameters
ROC_fps = 35000 / 60.0           
G_climb = ROC_fps / V_climb_fps
G_catapult = 0.015
ROC_ceiling_fps = 100 / 60.0     

# Turn 
turn_rate_rad_s = 8.0 * np.pi / 180.0
n_turn = np.sqrt(1 + (V_turn_fps * turn_rate_rad_s / g)**2)

# Dynamic pressures
q_catapult = 0.5 * rho_SL * V_catapult_end_fps**2
q_climb = 0.5 * rho_climb * V_climb_fps**2
q_cruise = 0.5 * rho_cruise * V_cruise_fps**2
q_dash_SL = 0.5 * rho_SL * V_dash_SL_fps**2         # Sea level dash
q_dash_high = 0.5 * rho_dash * V_dash_high_fps**2   # High altitude dash
q_turn = 0.5 * rho_turn * V_turn_fps**2

# Wing loading range
WS = np.linspace(10, 150, 500)

# Current design
W_TO = 54000
current_WS = W_TO / S_ref


# CONSTRAINT EQUATIONS


# Landing: W/S = (1/2)*rho*V_app^2*C_L_max / 1.1^2
WS_max_landing = (0.5 * rho_SL * V_app_fps**2 * C_L_max_land) / (1.1**2)

# Catapult: T/W = (q*CD0)/(W/S) + k*(W/S)/q + G
TW_catapult = (q_catapult * C_D0_takeoff) / WS + k_subsonic * WS / q_catapult + G_catapult

# Climb (sea level): T/W = (q*CD0)/(W/S) + k*(W/S)/q + G
TW_climb = (q_climb * C_D0_subsonic) / WS + k_subsonic * WS / q_climb + G_climb

# Cruise: T/W = [(q*CD0)/(W/S) + k*(W/S)/q] / sigma
TW_cruise = ((q_cruise * C_D0_subsonic) / WS + k_subsonic * WS / q_cruise) / sigma_cruise

# Sea Level Dash (Mach 0.9): T/W = (q*CD0)/(W/S) + k*(W/S)/q

TW_dash_SL = (q_dash_SL * C_D0_subsonic) / WS + k_subsonic * WS / q_dash_SL


TW_dash_high = ((q_dash_high * C_D0_supersonic) / WS + k_supersonic * WS / q_dash_high) / sigma_dash


TW_turn = ((q_turn * C_D0_subsonic) / WS + k_subsonic * (n_turn**2) * WS / q_turn) / sigma_turn


q_ceiling = WS / C_L_ceiling
# Velocity at ceiling from dynamic pressure
V_ceiling = np.sqrt(2 * q_ceiling / rho_ceiling)
# Thrust required at ceiling altitude
TW_ceiling_altitude = (q_ceiling * C_D0_subsonic) / WS + k_subsonic * WS / q_ceiling + ROC_ceiling_fps / V_ceiling
# Convert to sea level thrust 
TW_ceiling = TW_ceiling_altitude / sigma_ceiling

# Design Envelope: maximum T/W required across all constraints
TW_envelope = np.maximum.reduce([TW_catapult, TW_climb, TW_cruise, TW_dash_SL, TW_dash_high, TW_turn, TW_ceiling])


idx = np.argmin(np.abs(WS - current_WS))
required_TW = TW_envelope[idx]

#Plot

plt.figure(figsize=(15, 10))

plt.plot(WS, TW_catapult, 'r-', linewidth=2.5, label='Catapult Launch')
plt.plot(WS, TW_climb, 'g-', linewidth=2.5, label='Climb (Sea Level @ 35,000 ft/min)')
plt.plot(WS, TW_cruise, 'c-', linewidth=2.5, label='Cruise (20k ft)')
plt.plot(WS, TW_dash_SL, 'orange', linewidth=2.5, label=f'Sea Level Dash (Mach {Mach_dash_SL})')
plt.plot(WS, TW_dash_high, 'm-', linewidth=2.5, label=f'High Alt Dash (Mach {Mach_dash_high})')
plt.plot(WS, TW_turn, 'b-', linewidth=2.5, label=f'Sustained Turn (8deg/s,)')
plt.plot(WS, TW_ceiling, 'y-', linewidth=2.5, label=f' Ceiling ({h_ceiling/1000:.0f}k ft)')
plt.axvline(x=WS_max_landing, color='darkred', linestyle='--', linewidth=3.0, 
            label=f'Landing Limit (130 kt)')

WS_feasible = WS[WS <= WS_max_landing]
TW_feasible = TW_envelope[WS <= WS_max_landing]
plt.fill_between(WS_feasible, TW_feasible, np.max(TW_envelope)*1.5,
                 alpha=0.15, color='green', label='Feasible Region')


plt.plot(current_WS, required_TW, 'ko', markersize=14, markeredgewidth=2.5,
         markerfacecolor='yellow', markeredgecolor='black',
         label=f'Design Point (W/S={current_WS:.1f}, T/W={required_TW:.3f})')

plt.xlabel('Wing Loading, W/S (lb/ft²)', fontsize=16, fontweight='bold')
plt.ylabel('Thrust to Weight Ratio, T/W', fontsize=16, fontweight='bold')
plt.title('Carrier Based Strike Fighter Constraint Diagram', 
          fontsize=18, fontweight='bold')
plt.xlim(0, 150)
plt.ylim(0, min(np.max(TW_envelope) * 1.3, 2.0))
plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
plt.legend(loc='upper right', fontsize=10, framealpha=0.95, ncol=2)
plt.tight_layout()
plt.savefig('constraint_diagram.png', dpi=300, bbox_inches='tight')
plt.show()