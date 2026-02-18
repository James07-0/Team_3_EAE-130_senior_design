

import numpy as np
import matplotlib.pyplot as plt



AR = 2.096            
e = 0.8              


CD0_clean = 0.0063         
CD0_TO_flaps = 0.0213        
CD0_TO_gear = 0.0495      
CD0_land_flaps = 0.0713     
CD0_land_gear = 0.0995       

CL_min = -2.0        
CL_max = 2.0          
num_points = 500      





k = 1.0 / (np.pi * e * AR)


CL = np.linspace(CL_min, CL_max, num_points)


CD_clean = CD0_clean + k * CL**2


CD_TO_gear_up = CD0_TO_flaps + k * CL**2


CD_TO_gear_down = CD0_TO_gear + k * CL**2


CD_land_gear_up = CD0_land_flaps + k * CL**2


CD_land_gear_down = CD0_land_gear + k * CL**2


plt.figure(figsize=(14, 10))

plt.plot(CD_clean, CL, 'b-', linewidth=3.0, label='Clean (cruise)')
plt.plot(CD_TO_gear_up, CL, 'g-', linewidth=3.0, label='Takeoff Flaps + Gear Up')
plt.plot(CD_TO_gear_down, CL, 'g--', linewidth=3.0, label='Takeoff Flaps + Gear Down')
plt.plot(CD_land_gear_up, CL, 'r-', linewidth=3.0, label='Landing Flaps + Gear Up')
plt.plot(CD_land_gear_down, CL, 'r--', linewidth=3.0, label='Landing Flaps + Gear Down')


plt.xlabel('Drag Coefficient, $C_D$', fontsize=16, fontweight='bold')
plt.ylabel('Lift Coefficient, $C_L$', fontsize=16, fontweight='bold')
plt.title('Drag Polar - Carrier-Based Strike Fighter\nAll Flight Configurations', 
          fontsize=18, fontweight='bold', pad=20)
plt.grid(True, alpha=0.3, linestyle='--', linewidth=1.0)
plt.legend(loc='upper left', fontsize=13, framealpha=0.95, edgecolor='black', fancybox=True)


plt.xlim(left=0, right=max(CD_land_gear_down)*1.1)
plt.ylim(bottom=CL_min, top=CL_max)

plt.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)




plt.show()

