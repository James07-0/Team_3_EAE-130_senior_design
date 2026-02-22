from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1) SIMPLE WEIGHT ESTIMATION (mission fractions + regression We/W0)
# ============================================================

def empty_weight_fraction(W0: float) -> float:
    # regression: We/W0 = A * W0^C  (your original)
    A = 2.34
    C = -0.13
    return A * (W0 ** C)

def fuel_fraction_cruise(range_nmi, speed_fps, sfc, L_D):
    range_ft = range_nmi * 6076.12
    sfc_per_sec = sfc / 3600.0
    return np.exp(-range_ft * sfc_per_sec / (speed_fps * L_D))

def fuel_fraction_loiter(endurance_hrs, sfc, L_D):
    return np.exp(-endurance_hrs * sfc / L_D)

def mission_weight_fractions(cruise_range, cruise_speed, cruise_LD,
                             loiter_time, loiter_LD, sfc):
    """
    Returns WL/W0 (= W6/W0) and Wf/W0 using your mission segment setup.
    """
    W1_W0 = 0.97
    W2_W1 = 0.985
    W5_W4 = 0.995

    W3_W2 = fuel_fraction_cruise(cruise_range, cruise_speed, sfc, cruise_LD)
    W4_W3 = fuel_fraction_loiter(loiter_time, sfc, loiter_LD)

    W6_W0 = W1_W0 * W2_W1 * W3_W2 * W4_W3 * W5_W4
    Wf_W0 = 1.06 * (1 - W6_W0)
    return W6_W0, Wf_W0

def estimate_weight(W_crew, W_payload,
                    cruise_range, cruise_speed, cruise_LD,
                    loiter_time, loiter_LD, sfc,
                    W0_guess=50000.0):
    WL_W0, Wf_W0 = mission_weight_fractions(
        cruise_range, cruise_speed, cruise_LD,
        loiter_time, loiter_LD, sfc
    )

    W0 = float(W0_guess)
    for _ in range(100):
        We = empty_weight_fraction(W0) * W0
        Wf = Wf_W0 * W0
        W0_new = We + Wf + W_payload + W_crew
        if abs(W0_new - W0) / max(W0, 1.0) < 1e-6:
            W0 = W0_new
            break
        W0 = W0_new

    return W0, WL_W0
# ============================================================
# 2) RUNWAY LANDING FIELD LENGTH CONSTRAINT (5150 foot zone)
# ============================================================
def landing_WS_allowed(S, s_land, rho, g, CD, v_TD, F_avg):
    numerator = s_land * rho * g * CD
    inside = 1.0 + 0.5 * rho * S * CD * v_TD**2 / max(F_avg, 1.0)
    denominator = np.log(max(inside, 1.0000001))
    return numerator / max(denominator, 1e-9)

def landing_W0S_allowed_carrier(
    S: float,
    s_land_ft: float,
    rho: float,
    g: float,
    CD: float,
    v_TD: float,
    F_avg: float,
    WL_W0: float
):
    WL_over_S_max = landing_WS_allowed(S, s_land_ft, rho, g, CD, v_TD, F_avg)
    W0_over_S_max = WL_over_S_max / max(WL_W0, 1e-9)
    return W0_over_S_max, v_TD, None

# ============================================================
# 3) MISSION CONFIG
# ============================================================
class MissionConfig:
    def __init__(self, engine="twin"):
        aim9x = 190
        mk83_jdam = 1100
        strike_ordnance = 4 * mk83_jdam + 2 * aim9x

        self.W_crew = 200
        self.W_avionics = 2500
        self.W_payload = self.W_avionics + strike_ordnance

        self.cruise_range = 1000
        self.cruise_speed = 850
        self.cruise_LD = 8

        self.loiter_time = 0.333
        self.loiter_LD = 8

        if engine == "single":
            self.sfc = 0.85
            self.W0_guess = 65000
        else:
            self.sfc = 0.95
            self.W0_guess = 60000

# ============================================================
# 4) DRIVER / PLOT
# ============================================================
def main():
    mission = MissionConfig(engine="twin")

    # ---- Carrier landing parameters ----
    s_land = 300          # ft (use your carrier-deck stopping distance if you want; 5150 is long for a carrier)
    rho = 0.002377          # slug/ft^3
    g = 32.174             # ft/s^2

    CD = 0.5855               # <-- landing configuration drag coefficient (you choose)
    v_TD = 188.0           # <-- touchdown speed (ft/s) you choose
    F_avg = 127000.0        # <-- average hook force (lbf) you choose

    # Design thrust loading (for mapping into T–S)
    T_over_W_design = 1.169

    # Mission W0 and WL/W0
    W0, WL_W0 = estimate_weight(
        mission.W_crew,
        mission.W_payload,
        mission.cruise_range,
        mission.cruise_speed,
        mission.cruise_LD,
        mission.loiter_time,
        mission.loiter_LD,
        mission.sfc,
        mission.W0_guess
    )
    print(f"W0 = {W0:.1f} lbf,  WL/W0 = {WL_W0:.4f}")

    # Wing area grid
    S_grid = np.linspace(600, 1400, 40)

    # Carrier landing constraint gives a W0/S that varies with S
    W0S_curve = np.zeros_like(S_grid, dtype=float)
    for i, S in enumerate(S_grid):
        W0S_curve[i], _, _ = landing_W0S_allowed_carrier(
            S=S,
            s_land_ft=s_land,
            rho=rho,
            g=g,
            CD=CD,
            v_TD=v_TD,
            F_avg=F_avg,
            WL_W0=WL_W0
        )
    # Map to thrust vs wing area
    T_from_landing = T_over_W_design * W0S_curve * S_grid
    # Plot
    plt.figure()
    plt.plot(S_grid, T_from_landing, marker='o')
    plt.xlabel("Wing Area S (ft²)")
    plt.ylabel("Thrust T (lbf)")
    plt.title("T–S plot: Carrier arresting-gear landing constraint")
    plt.xlim(S_grid.min(), S_grid.max())
    plt.ylim(0, 1.05*np.max(T_from_landing))
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()