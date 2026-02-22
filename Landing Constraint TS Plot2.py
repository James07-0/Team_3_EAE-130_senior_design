from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# 1) WEIGHT ESTIMATION
# ============================================================

def empty_weight_fraction(W0: float) -> float:
    A = 2.34
    C = -0.13
    return A * (W0 ** C)


def fuel_fraction_cruise(range_nmi, speed_fps, sfc, L_D):
    range_ft = range_nmi * 6076.12
    sfc_per_sec = sfc / 3600.0
    return np.exp(-range_ft * sfc_per_sec / (speed_fps * L_D))


def fuel_fraction_loiter(endurance_hrs, sfc, L_D):
    return np.exp(-endurance_hrs * sfc / L_D)


def estimate_weight(W_crew, W_payload, cruise_range, cruise_speed,
                    cruise_LD, loiter_time, loiter_LD, sfc, W0_guess):

    W1_W0 = 0.97
    W2_W1 = 0.985
    W5_W4 = 0.995

    W0 = float(W0_guess)
    error = 1.0

    while error > 1e-6:
        We_W0 = empty_weight_fraction(W0)
        W3_W2 = fuel_fraction_cruise(cruise_range, cruise_speed, sfc, cruise_LD)
        W4_W3 = fuel_fraction_loiter(loiter_time, sfc, loiter_LD)

        W6_W0 = W1_W0 * W2_W1 * W3_W2 * W4_W3 * W5_W4
        Wf_W0 = 1.06 * (1 - W6_W0)

        W0_new = (W_crew + W_payload) / (1 - Wf_W0 - We_W0)
        error = abs((W0_new - W0) / W0_new)
        W0 = W0_new

    return W0, W6_W0


# ============================================================
# 2) LANDING CONSTRAINT
# ============================================================

def landing_WS_allowed(S, s_land, rho, g, CD, v_TD, F_avg):
    numerator = s_land * rho * g * CD
    inside = 1 + 0.5 * rho * S * CD * v_TD**2 / F_avg
    denominator = np.log(inside)
    return numerator / denominator

# ============================================================
# 3) MISSION CONFIG
# ============================================================

class MissionConfig:
    def __init__(self, engine="twin"):
        aim120c = 335
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
# 4) DRIVER: Thrust vs Wing Area with Landing Constraint
# ============================================================

def main():
    mission = MissionConfig(engine="twin")

    # Compute design weight
    W0 = estimate_weight(
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

    print(f"Design Weight W0 = {W0:.1f} lbf")

    # Wing area grid
    S_grid = np.linspace(500, 1500, 50)

    # Landing parameters
    s_land = 5150
    rho = 0.002377
    g = 32.174
    CD = 0.5855
    v_TD = 188
    F_hook = 160000 #pounds
    W_land = W6_W0 * W0

    WS_allowed = []
    WS_actual = []
    T_required = []

    for S in S_grid:
        F_avg = 0.8 * F_hook

        WS_allow = landing_WS_allowed(S, s_land, rho, g, CD, v_TD, F_avg)
        WS_allowed.append(WS_allow)

        WS_act = W_land / S
        WS_actual.append(WS_act)

        # Thrust requirement from design T/W
        m = 5.0
        c = 20000.0
        T_required.append(m * S + c)

    WS_allowed = np.array(WS_allowed)
    WS_actual = np.array(WS_actual)
    T_required = np.array(T_required)

    # Plot Thrust vs Wing Area
    plt.figure()
    plt.plot(S_grid, T_required, label="Required Thrust (design T/W)")

    # Shade infeasible landing region
    infeasible = WS_actual > WS_allowed
    plt.fill_between(S_grid, 0, T_required.max(),
                     where=infeasible, color='red', alpha=0.3,
                     label="Landing Infeasible")

    plt.xlabel("Wing Area S (ft²)")
    plt.ylabel("Thrust T (lbf)")
    plt.title("Thrust vs Wing Area with Landing Constraint")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()