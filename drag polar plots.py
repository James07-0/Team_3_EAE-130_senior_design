import numpy as np
import matplotlib.pyplot as plt

AR = 2.096
s = 39.8
s_ref = 972
S_wet = 2148.9
c_f = 0.0040
C_D_0 = 0.0063

# Adjust C_Lmax for each flight configuration
cL_clean = np.linspace(-0.26,0.26,100)
cL_takeoff = np.linspace(-1.4,1.4,100)
cL_landing = np.linspace(-1.6,1.6,100)

# Clean configuration
def calculate_induced_drag_coefficient(AR, e):
    return 1/(np.pi*AR*e)
e_clean = 0.8
coef_clean = calculate_induced_drag_coefficient(AR, e_clean)
print("Induced drag coefficient for clean configuration:", coef_clean)
clean = C_D_0 + coef_clean*cL_clean*cL_clean

# Takeoff with flaps
e_takeoff = 0.8
delta_CD0_takeoff_flaps = 0.015 # additional drag due to takeoff flaps
coef_takeoff = calculate_induced_drag_coefficient(AR, e_takeoff)
print("Induced drag coefficient for takeoff configuration:", coef_takeoff)
takeoff_flaps = C_D_0 + delta_CD0_takeoff_flaps + coef_takeoff*cL_takeoff*cL_takeoff 

# Takeoff with flaps and gear
e_gear = e_clean # Assuming landing gear does not affect the efficiency factor
delta_CD0_gear = 0.02818 # additional drag due to landing gear
coef_gear = calculate_induced_drag_coefficient(AR, e_gear)
takeoff_fg = C_D_0 + delta_CD0_takeoff_flaps + delta_CD0_gear + coef_takeoff*cL_takeoff*cL_takeoff 

# Landing configuration
e_landing = 0.8
delta_CD0_landing = 0.065 # additional drag due to landing flaps and gear
coef_landing = calculate_induced_drag_coefficient(AR, e_landing)
print("Induced drag coefficient for landing configuration:", coef_landing)
landing_flaps = C_D_0 + delta_CD0_landing + coef_landing*cL_landing*cL_landing

# Landing with flaps and gear
e_gear = e_clean # Assuming landing gear does not affect the efficiency factor
landing_fg = C_D_0 + delta_CD0_landing + delta_CD0_gear + coef_landing*cL_landing*cL_landing

plt.figure(figsize=(16,9))
plt.title('Drag Polars')
plt.xlabel("$C_D$")
plt.ylabel("$C_L$")
plt.plot(clean, cL_clean, label='Clean', linestyle='-', linewidth=2)
plt.plot(takeoff_flaps, cL_takeoff, label='Takeoff with flaps', linestyle='-', linewidth=2)
plt.plot(takeoff_fg, cL_takeoff, label='Takeoff with flaps + gear', linestyle='-', linewidth=2)
plt.plot(landing_flaps, cL_landing, label='Landing with flaps', linestyle='-', linewidth=2)
plt.plot(landing_fg, cL_landing, label='Landing with flaps + gear', linestyle='-', linewidth=2)
plt.legend(loc='best')
plt.show()

