from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt


#1) weight estimation 

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


def estimate_weight(
    W_crew,
    W_payload,
    cruise_range,
    cruise_speed,
    cruise_LD,
    loiter_time,
    loiter_LD,
    sfc,
    W0_guess=50000.0
):

    W1_W0 = 0.97
    W2_W1 = 0.985
    W5_W4 = 0.995

    W0 = W0_guess
    tol = 1e-6

    for _ in range(100):

        We_W0 = empty_weight_fraction(W0)

        W3_W2 = fuel_fraction_cruise(cruise_range, cruise_speed, sfc, cruise_LD)
        W4_W3 = fuel_fraction_loiter(loiter_time, sfc, loiter_LD)

        W6_W0 = W1_W0 * W2_W1 * W3_W2 * W4_W3 * W5_W4
        Wf_W0 = 1.06 * (1 - W6_W0)

        W0_new = (W_crew + W_payload) / (1 - Wf_W0 - We_W0)

        if abs(W0_new - W0) / max(W0_new, 1e-9) < tol:
            W0 = W0_new
            break

        W0 = W0_new

    We = We_W0 * W0
    Wf = Wf_W0 * W0

    return W0, We, Wf



# 2)mission configuration 

class MissionConfig:

    def __init__(self, engine="twin"):

        aim9x = 190
        mk83 = 1100

        strike = 4 * mk83 + 2 * aim9x

        self.W_crew = 200
        self.W_avionics = 2500
        self.W_payload = self.W_avionics + strike

        self.cruise_range = 1000.0
        self.cruise_speed = 850.0
        self.cruise_LD = 8.0

        self.loiter_time = 0.333
        self.loiter_LD = 8.0

        if engine.lower() == "single":
            self.sfc = 0.85
            self.W0_guess = 65000.0
        else:
            self.sfc = 0.95
            self.W0_guess = 60000.0



#3) t/w requirment 

def turn_TW_required(WS, rho, V, n):

    AR = 2.096
    e = 0.8
    CD0 = 0.0063

    k = 1 / (np.pi * e * AR)
    q = 0.5 * rho * V**2

    TW = (q * CD0) / WS + (k * n**2 * WS) / q
    return TW


#4) find thrust 

def solve_T_for_S_turn(S, mission, rho, V, n, T_guess=40000.0):

    T = T_guess
    W0_guess = mission.W0_guess

    for _ in range(60):

        W0, _, _ = estimate_weight(
            mission.W_crew,
            mission.W_payload,
            mission.cruise_range,
            mission.cruise_speed,
            mission.cruise_LD,
            mission.loiter_time,
            mission.loiter_LD,
            mission.sfc,
            W0_guess
        )

        WS = W0 / S
        TW = turn_TW_required(WS, rho, V, n)

        T_new = TW * W0

        if abs(T_new - T) / max(T, 1e-9) < 1e-3:
            return T_new, W0, TW

        T = T_new
        W0_guess = W0

    return T, W0, TW


#5) driver

def main():

    mission = MissionConfig(engine="twin")

    # ---- TURN CONDITIONS ----
    rho = 0.002377     # slug/ft^3 (sea level)
    V = 250.0          # ft/s (turn speed)
    n = 7.0            # load factor

    S_grid = np.linspace(600, 1400, 40)

    T_list = []
    W_list = []
    WS_list = []
    TW_list = []

    T_guess = 40000.0

    for S in S_grid:

        T, W0, TW = solve_T_for_S_turn(S, mission, rho, V, n, T_guess)

        T_list.append(T)
        W_list.append(W0)
        WS_list.append(W0 / S)
        TW_list.append(TW)

        T_guess = T

    S_arr = np.array(S_grid)
    T_arr = np.array(T_list)
    WS_arr = np.array(WS_list)
    TW_arr = np.array(TW_list)

    # -------- PLOTS --------

    plt.figure()
    plt.plot(S_arr, T_arr, marker='o')
    plt.xlabel("Wing Area S (ft^2)")
    plt.ylabel("Required Thrust T (lbf)")
    plt.title(f"Turning Constraint — {n}g Turn")
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(WS_arr, TW_arr, marker='o')
    plt.xlabel("Wing Loading W/S (lb/ft^2)")
    plt.ylabel("Required T/W")
    plt.title(f"Turning Constraint — {n}g Turn")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
