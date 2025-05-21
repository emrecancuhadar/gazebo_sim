import numpy as np

def packing_ratio(w0: float, rho_p: float, delta: float) -> float:
    """
    β = p_b / p_p  where p_b = w0 / δ
    - w0: oven-dry fuel load (lb/ft²)
    - rho_p: particle density (lb/ft³)
    - delta: fuel depth (ft)
    """
    rho_b = w0 / delta
    return rho_b / rho_p

def optimum_packing_ratio(sigma: float) -> float:
    """
    β_opt = 3.348 * o^(-0.8189)
    - sigma: surface-area-to-volume ratio (ft²/ft³)
    """
    return 3.348 * sigma**-0.8189

def optimum_reaction_velocity(sigma: float, beta: float, beta_opt: float) -> float:
    """
    Γ' = σ^(1.5) / (495 + 0.0594·σ^(1.5))  ×  (β/β_opt)^A  ×  exp[A·(1 – β/β_opt)]
    where A = 133·σ^(–0.7913)
    - sigma: surface‐area‐to‐volume ratio
    - beta: packing ratio
    - beta_opt: optimum packing ratio
    """
    A = 133 * sigma**-0.7913
    gamma_max = sigma**1.5 / (495 + 0.0594 * sigma**1.5)
    rel = beta / beta_opt
    return gamma_max * np.power(rel, A) * np.exp(A * (1.0 - rel))

def moisture_damping_coefficient(M_f: float, M_x: float) -> float:
    """
    η_M = 1 – 2.59·r + 5.11·r^2 – 3.52·r^3
    where r = min(M_f/M_x, 1.0)
    - M_f: actual fuel moisture content (%)
    - M_x: moisture of extinction (%)
    """
    r = np.minimum(M_f / M_x, 1.0)
    return 1 - 2.59*r + 5.11*r**2 - 3.52*r**3

def mineral_damping_coefficient(S_e: float) -> float:
    """
    η_s = min(0.174·S_e^(–0.19), 1.0)
    - S_e: effective (inorganic) mineral content fraction
    """
    return np.minimum(0.174 * S_e**-0.19, 1.0)

def reaction_velocity(gamma_opt: float, eta_M: float, eta_s: float) -> float:
    """
    Γ = Γ' · η_M · η_s
    - gamma_opt: optimum reaction velocity Γ'
    - eta_M: moisture damping
    - eta_s: mineral damping
    """
    return gamma_opt * eta_M * eta_s

def reaction_intensity(gamma: float, w: float, h: float) -> float:
    """
    I_R = Γ · w · h
    - gamma: reaction velocity (min⁻¹)
    - w: net fuel load (lb/ft²)
    - h: low heat content (Btu/lb)
    """
    return gamma * w * h

def propagating_flux_ratio(sigma: float, beta: float) -> float:
    """
    Compute ξ = exp[(0.792 + 0.681*sqrt(sigma))*(beta + 0.1)] / (192 + 0.2595*sigma)

    Args:
        sigma: surface-area-to-volume ratio (ft²/ft³)
        beta: mean packing ratio

    Returns:
        Propagating flux ratio (dimensionless)
    """
    sqrt_sigma = np.sqrt(sigma)
    exponent    = (0.792 + 0.681 * sqrt_sigma) * (beta + 0.1)
    numerator   = np.exp(exponent)
    denominator = 192 + 0.2595 * sigma
    return numerator / denominator

def wind_factor(sigma: float,
                beta: float,
                beta_op: float,
                U: float) -> float:
    """
    Compute Rothermel wind factor ϕw.

    Args:
      sigma   (ft²/ft³): surface-area-to-volume ratio
      beta    (–)     : packing ratio
      beta_op (–)     : optimum packing ratio
      U       (mi/h)  : midflame wind speed

    Returns:
      float: wind factor ϕw (dimensionless)
    """
    # geometry constants
    C = 7.47 * np.exp(-0.133 * sigma**0.55)
    B = 0.02526 * sigma**0.54
    E = 0.715 * np.exp(-3.59e-4 * sigma)

    return C * (U ** B) * ((beta / beta_op) ** (-E))

def slope_factor_from_angle(beta: float, slope_angle_rad: float) -> float:
    """
    Compute slope factor φ_s given packing ratio and slope angle in radians.

    Args:
      beta            (–)  : mean packing ratio of the fuel bed
      slope_angle_rad (rad): slope angle in radians

    Returns:
      float : slope factor φ_s (dimensionless)
    """
    return 5.275 * np.power(beta, -0.3) * np.power(np.tan(slope_angle_rad), 2)

def bulk_density(w0: float, delta: float) -> float:
    """
    Compute the fuel‐bed bulk density ρ_b.

    ρ_b = w0 / delta

    Args:
        w0 (float): oven‐dry fuel load (lb/ft²)
        delta (float): fuel‐bed depth (ft)

    Returns:
        float: bulk density (lb/ft³)
    """
    return w0 / delta

def effective_heating_number(sigma: float) -> float:
    """
    Compute the effective heating number ε for Rothermel's fire spread model.

    ε = exp(–138 / σ)

    Args:
        sigma (float): characteristic surface-area-to-volume ratio (ft²/ft³)

    Returns:
        float: effective heating number (dimensionless)
    """
    return np.exp(-138.0 / sigma)

def heat_of_preignition(M_f: float) -> float:
    """
    Compute the heat of preignition Q_ig for Rothermel's fire spread model.

    Q_ig = 250 + 1116 * M_f

    Args:
        M_f (float): fuel moisture content (fraction, e.g., 0.04 for 4%)

    Returns:
        float: heat of preignition (Btu per pound of fuel)
    """
    return 250.0 + 1116.0 * M_f

def live_moisture_of_extinction(
    w_dead: float,  sigma_dead: float,
    w_live: float,  sigma_live: float,
    Mf_dead: float, Mx_dead: float
) -> float:
    """
    Albini (1976a) eqn for live‐fuel moisture of extinction Mx_live.

      Mx_live = 2.9·W·(1 - Mf_dead/Mx_dead) - 0.226,  min = Mx_dead

    where W = (w_dead·exp(-138/σ_dead)) /
              (w_live·exp(-500/σ_live))

    Returns:
      Mx_live (fraction)
    """
    # 1) dead-to-live load ratio
    W = (w_dead * np.exp(-138.0 / sigma_dead)) \
      / (w_live * np.exp(-500.0 / sigma_live))
    # 2) Albini’s formula
    Mx2 = 2.9 * W * (1.0 - Mf_dead / Mx_dead) - 0.226
    # 3) enforce minimum
    return np.maximum(Mx2, Mx_dead)

def rate_of_spread(
    IR: float,      # reaction intensity (I_R)
    xi: float,      # propagating flux ratio (ξ)
    rho_b: float,   # bulk density (ρ_b)
    epsilon: float, # effective heating number (ε)
    Qig: float,     # heat of preignition (Q_ig)
    phi_w: float,   # wind factor (ϕ_w)
    phi_s: float    # slope factor (ϕ_s)
) -> float:
    """
    Compute the surface-fire rate of spread R (ft/min) via Rothermel's model:
      R0 = (IR * xi) / (rho_b * epsilon * Qig)
      R  = R0 * (1 + phi_w + phi_s)

    Returns:
        float: rate of spread R in ft/min
    """
    R0 = (IR * xi) / (rho_b * epsilon * Qig)
    return R0 * (1 + phi_w + phi_s)

# Auxiliary outputs
def residence_time(sigma):
    return 384 / sigma

def heat_per_unit_area(I_R, t_r):
    return I_R * t_r

def fireline_intensity(h, w, R):
    return h * w * R

def flame_length(I_B):
    return 0.45 * I_B**0.46
