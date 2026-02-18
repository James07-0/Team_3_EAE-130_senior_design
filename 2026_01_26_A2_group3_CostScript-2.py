import numpy as np

# Main Variables
W_Airframe = 28136 # empty weight of plane
V_h = 1334 # knots
Q_Prod = 500 # production
F_Cert = 1 # not applicable
F_Cf =  1.03 
F_Comp = 1 # Aircraft not 100% composites
F_Press = 1.03
F_HyE = 1 # Not hybrid-electric propulsion
Cpi = 1.83 # Inflation
Fta = 3 # Number of aircraft prototypes
R_e = 115 # Engineering hourly wage in dollars

H_e = 4.86*W_Airframe**0.777 * V_h**0.894 * Q_Prod**0.163 # Engineering 

C_dev = 91.3*W_Airframe**0.630* V_h**1.3 # Development costs

C_ft = 2498 * W_Airframe**0.325 * V_h**0.822 * Fta**1.21 # Flight test 

Rdte_total = H_e * R_e * Cpi + C_dev + C_ft # Total RDT&E costs


print(f"RDT&E Total: $ {Rdte_total:,.0f}")

# Flyaway costs
H_m = 7.37 * W_Airframe**0.82 * V_h**0.484 * 1**0.641/20 # Tooling hours for a single plane 
R_m = 98 # Manufacturing hourly wage in dollars
H_q = 0.133 # Quality control 
R_q = 108 # Quality control hourly wage in dollars
C_m = 22.1*W_Airframe**0.921 * V_h**0.621 # Material cost
T_max = 40000 # lbf for P&W XA-103 engine
M_max = 2 # Max mach number for engine, left at 2.0
T_turbineinlet = 3900 # Estimate for Xa-103 engine in degrees Rankine
C_eng = 3112 * (0.043 * T_max + 243.25 * M_max + 0.969 * T_turbineinlet - 2228) #Cost of engine development
N_eng = 1 # Single engine aircraft estimate 
C_avionics = 2500 * 4000 # 2500 lb of avionics * $6000 per lb for total
C_flyawaysingle = H_m * R_m * Cpi + H_q * R_q * Cpi + C_m + C_eng * N_eng + C_avionics # total flyaway cost for one plane
print("Single Engine Price")
print(f"Flyaway Cost:  $ {C_flyawaysingle:,.0f}")


# Main Variables
W_Airframedual  = 30761 # empty weight of plane
V_h = 1334 # knots
Q_Prod = 500 # production
F_Cert = 1 # not applicable
F_Cf =  1.03 
F_Comp = 1 # Aircraft not 100% composites
F_Press = 1.03
F_HyE = 1 # Not hybrid-electric propulsion
Cpi = 1.83 # Inflation
Fta = 3 # Number of aircraft prototypes
R_e = 115 # Engineering hourly wage in dollars

H_edual = 4.86*W_Airframedual**0.777 * V_h**0.894 * Q_Prod**0.163 # Engineering 

C_devdual = 91.3*W_Airframedual**0.630* V_h**1.3 # Development costs

C_ftdual = 2498 * W_Airframedual**0.325 * V_h**0.822 * Fta**1.21 # Flight test 

Rdte_totaldual = H_edual * R_e * Cpi + C_devdual + C_ftdual# Total RDT&E costs

print("Dual engine ")
print (f"RDTE DUAL {Rdte_totaldual:,.0f}")

# Dual engine changed variables
W_Airframe = 30761 #empty weight of plane
N_eng2 = 2
H_m2 = 7.37 * W_Airframe**0.82 * V_h**0.484 * 1**0.641/20 # Tooling hours for a single plane 
C_flyawaydual = H_m2 * R_m * Cpi + H_q * R_q * Cpi + C_m + C_eng * N_eng2 + C_avionics # Total flyaway cost for one plane with dual engines

print(f"Flyaway Cost:  $ {C_flyawaydual:,.0f}")