from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt


# 1) YOUR WEIGHT ESTIMATION CODE (unchanged, just organized)

def empty_weight_fraction(W0: float) -> float:
    A = 2.34
    C = -0.13
    return A * (W0 ** C)  # We/W0


def fuel_fraction_cruise(range_nmi: float, speed_fps: float, sfc: float, L_D: float) -> float:
    range_ft = range_nmi * 6076.12       # nmi -> ft
    sfc_per_sec = sfc / 3600.0           # 1/hr -> 1/s
    return np.exp(-range_ft * sfc_per_sec / (speed_fps * L_D))


def fuel_fraction_loiter(endurance_hrs: float, sfc: float, L_D: float) -> float:
    return np.exp(-endurance_hrs * sfc / L_D)


def estimate_weight(
    W_crew: float,
    W_payload: float,
    cruise_range: float,
    cruise_speed: float,
    cruise_LD: float,
    loiter_time: float,
    loiter_LD: float,
    sfc: float,
    W0_guess: float = 50000.0
) -> tuple[float, float, float, int]:
   

    # Mission segment fractions (warmup, takeoff, climb, descent, landing)
    W1_W0 = 0.97   # warmup/takeoff
    W2_W1 = 0.985  # climb
    W5_W4 = 0.995  # descent/landing

    W0 = float(W0_guess)
    tolerance = 1e-6
    error = 1.0
    iterations = 0

    while error > tolerance and iterations < 100:
        We_W0 = empty_weight_fraction(W0)

        W3_W2 = fuel_fraction_cruise(cruise_range, cruise_speed, sfc, cruise_LD)
        W4_W3 = fuel_fraction_loiter(loiter_time, sfc, loiter_LD)

        W6_W0 = W1_W0 * W2_W1 * W3_W2 * W4_W3 * W5_W4
        Wf_W0 = 1.06 * (1 - W6_W0)  # 6% reserve

        W0_new = (W_crew + W_payload) / (1 - Wf_W0 - We_W0)

        error = abs((W0_new - W0) / max(W0_new, 1e-9))
        W0 = W0_new
        iterations += 1

    We = We_W0 * W0
    Wf = Wf_W0 * W0
    return W0, We, Wf, iterations


# 2) CLIMB CONSTRAINT


def climb_TW_required(WS: float, rho: float, ROC: float) -> float:
   

    # From your constraint page
    AR = 2.096
    e = 0.8
    CD0 = 0.0056
    CL_climb = 1.08

    k = 1.0 / (np.pi * e * AR)

    V = np.sqrt(2.0 * WS / (rho * CL_climb))  # ft/s
    q = 0.5 * rho * V**2                      # lbf/ft^2

    DW = (q * CD0) / WS + (k * WS) / q
    TW = DW + ROC / V
    return float(TW)



# 3) NESTED SOLVER: inner loop = YOUR weight iteration

class MissionConfig:
  
   
    def __init__(self, engine: str = "twin"):
        # Ordnance weights (lbs)
        aim120c = 335
        aim9x = 190
        mk83_jdam = 1100  # 1000 lb + guidance estimate

        strike_ordnance = 4 * mk83_jdam + 2 * aim9x

        self.W_crew = 200
        self.W_avionics = 2500
        self.W_payload = self.W_avionics + strike_ordnance

        self.cruise_range = 1000.0  # nmi
        self.cruise_speed = 850.0   # ft/s
        self.cruise_LD = 8.0

        self.loiter_time = 0.333    # hr
        self.loiter_LD = 8.0

        # engine option from your script
        if engine.lower() == "single":
            self.sfc = 0.85
            self.W0_guess = 65000.0
        else:
            self.sfc = 0.95
            self.W0_guess = 60000.0


def solve_weight_inner_loop(S: float, T: float, mission: MissionConfig, W0_guess: float) -> float:
    
    W0, We, Wf, iters = estimate_weight(
        W_crew=mission.W_crew,
        W_payload=mission.W_payload,
        cruise_range=mission.cruise_range,
        cruise_speed=mission.cruise_speed,
        cruise_LD=mission.cruise_LD,
        loiter_time=mission.loiter_time,
        loiter_LD=mission.loiter_LD,
        sfc=mission.sfc,
        W0_guess=W0_guess
    )
    return float(W0)


def solve_T_for_S(
    S: float,
    mission: MissionConfig,
    rho_climb: float,
    ROC_req: float,
    T_guess: float = 40000.0,
    tol_T: float = 1e-3,
    max_outer: int = 60
) -> tuple[float, float, float]:
    """
    Outer loop: T convergence at fixed S.
    Returns: (T, W0, TW_req)
    """

    T = float(T_guess)
    W0_guess = mission.W0_guess  # initial guess for the weight solver

    for _ in range(max_outer):
        W0 = solve_weight_inner_loop(S, T, mission, W0_guess=W0_guess)

        WS = W0 / S
        TW_req = climb_TW_required(WS, rho=rho_climb, ROC=ROC_req)
        T_new = TW_req * W0

        if abs(T_new - T) / max(T, 1e-9) < tol_T:
            return float(T_new), float(W0), float(TW_req)

        # update guesses to improve convergence speed
        T = float(T_new)
        W0_guess = W0

    return float(T), float(W0), float(TW_req)



# 4) DRIVER: compute curve, plot, save CSV


def main():
    # Choose which concept you’re plotting:
    # "twin" (sfc=0.95, W0_guess=60000) or "single" (sfc=0.85, W0_guess=65000)
    mission = MissionConfig(engine="twin")

    # Climb assumptions (edit if your team specified altitude/ROC)
    rho_climb = 0.002377  # slug/ft^3
    ROC_req = 33.3        # ft/s (~2000 ft/min)

    # Wing-area grid
    S_grid = np.linspace(600.0, 1400.0, 40)

    T_vals, W_vals, WS_vals, TW_vals = [], [], [], []

    T_guess = 40000.0
    for S in S_grid:
        T, W0, TW = solve_T_for_S(S, mission, rho_climb, ROC_req, T_guess=T_guess)
        T_vals.append(T)
        W_vals.append(W0)
        WS_vals.append(W0 / S)
        TW_vals.append(TW)

        T_guess = T  # warm-start next S

    S_arr = np.array(S_grid)
    T_arr = np.array(T_vals)
    W_arr = np.array(W_vals)
    WS_arr = np.array(WS_vals)
    TW_arr = np.array(TW_vals)

    # Save CSV
    out = np.column_stack([S_arr, T_arr, W_arr, WS_arr, TW_arr])
    np.savetxt(
        "climb_constraint_T_vs_S.csv",
        out,
        delimiter=",",
        header="S_ft2,T_lbf,W0_lbf,WS_lbf_ft2,TW_required",
        comments=""
    )
    print("Saved: climb_constraint_T_vs_S.csv")

    # Plot T vs S
    plt.figure()
    plt.plot(S_arr, T_arr, marker="o")
    plt.xlabel("Wing Area S (ft^2)")
    plt.ylabel("Required Thrust T (lbf)")
    plt.title("Climb Constraint: T vs S (nested iteration)")
    plt.grid(True)
    plt.show()

    # Optional: classic view
    plt.figure()
    plt.plot(WS_arr, TW_arr, marker="o")
    plt.xlabel("Wing Loading W/S (lbf/ft^2)")
    plt.ylabel("Required T/W")
    plt.title("Climb Constraint: T/W vs W/S")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
