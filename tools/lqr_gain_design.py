#!/usr/bin/env python3
"""离线离散 LQR：植物理从 config.h 读，Q/R 从 lqr_weights.yaml 读。ESP32 不解 Riccati。"""
from __future__ import print_function

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HEADER = ROOT / "include" / "config.h"
DEFAULT_WEIGHTS = Path(__file__).resolve().parent / "lqr_weights.yaml"

_NUM = r"([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)[fF]?"


def cfg_number(src, name):
    m = re.search(
        r"^\s*constexpr\s+\w+\s+" + re.escape(name) + r"\s*=\s*" + _NUM,
        src,
        re.M,
    )
    if not m:
        raise SystemExit("config.h 找不到 constexpr " + name)
    return float(m.group(1))


def load_plant(header_path):
    src = Path(header_path).read_text(encoding="utf-8")
    hz = cfg_number(src, "kCtrlHz")
    plant = {
        "M": cfg_number(src, "kBodyMassKg"),
        "m": cfg_number(src, "kWheelMassKg"),
        "L": cfg_number(src, "kComHeightM"),
        "I": cfg_number(src, "kBodyPitchInertia"),
        "r": cfg_number(src, "kWheelRadiusM"),
        "g": cfg_number(src, "kGravity"),
        "dt": 1.0 / hz,
        "hz": hz,
    }
    return plant, src


def continuous_ab(p):
    """连续 A,B。状态 x=[θ, θ̇, s, ṡ]，输入 u=τ（两轮轮轴力矩之和，N·m）。

    直立线性化（与 docs/lqr_balance_theory.html §4.5–4.6 同一套）：
        Mq [s̈; θ̈] = [0; M g L] θ + [1/r; -1] τ
    质量矩阵（拉格朗日 q=(s,θ)）：
        Mq = [[ M+m ,  M L ],
              [ M L ,  I+M L² ]]
      M=车体，m=轮侧等效（含 I_w/r²），L=质心到轮轴，I=车体绕质心俯仰惯量。

    Δ = det(Mq) = (M+m)(I+M L²) - (M L)²
    Mq^{-1} = (1/Δ) [[ I+M L² ,  -M L ],
                     [ -M L    ,  M+m  ]]
    重力只进 θ̈、s̈ 对 θ 的通道；力矩进 B：
        a21 = (M+m) M g L / Δ                 # θ̈ 对 θ
        a41 = -(M L) M g L / Δ = -M² g L² / Δ # s̈ 对 θ
        b2  = -(M L/r + M + m) / Δ            # θ̈ 对 τ
        b4  = ((I+M L²)/r + M L) / Δ          # s̈ 对 τ
    下面用 solve(Mq, ·) 算 Mq^{-1}×右端，与上式等价，避免手写 Δ。
    """
    M, m, L, I, r, g = p["M"], p["m"], p["L"], p["I"], p["r"], p["g"]
    Mq = np.array([[M + m, M * L], [M * L, I + M * L * L]], dtype=float)
    grav = np.array([0.0, M * g * L])       # 重力项，只进 θ 方程
    btau = np.array([1.0 / r, -1.0])        # Q_s=τ/r，Q_θ=-τ
    # Mq^{-1} * grav → [s̈; θ̈] 对 θ 的系数；Mq^{-1} * btau → 对 τ 的系数
    acc_th = np.linalg.solve(Mq, grav)
    acc_u = np.linalg.solve(Mq, btau)
    # A 中：θ̈ = a21 θ，s̈ = a41 θ；B 中：θ̈+=b2 τ，s̈+=b4 τ
    # （行序对应 x 的第 2、4 行，故下标 21/41、b2/b4）
    a21, a41 = acc_th[1], acc_th[0]  # a21=θ̈/θ，a41=s̈/θ
    b2, b4 = acc_u[1], acc_u[0]      # b2=θ̈/τ，b4=s̈/τ
    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [a21, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [a41, 0.0, 0.0, 0.0],
        ]
    )
    B = np.array([[0.0], [b2], [0.0], [b4]])
    return A, B, Mq


def _expm(A):
    nrm = np.linalg.norm(A, ord=np.inf)
    s = 0
    if nrm > 0.5:
        s = int(math.ceil(math.log(nrm / 0.5, 2)))
    As = A / (2**s)
    n = A.shape[0]
    X = np.eye(n)
    term = np.eye(n)
    for k in range(1, 24):
        term = term @ As / k
        X = X + term
        if np.linalg.norm(term, ord=np.inf) < 1e-16:
            break
    for _ in range(s):
        X = X @ X
    return X


def zoh(A, B, dt):
    try:
        from scipy.signal import cont2discrete

        Ad, Bd, *_ = cont2discrete((A, B, np.eye(4), np.zeros((4, 1))), dt=dt, method="zoh")
        return np.asarray(Ad, dtype=float), np.asarray(Bd, dtype=float)
    except ImportError:
        n, m = A.shape[0], B.shape[1]
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A * dt
        M[:n, n:] = B * dt
        E = _expm(M)
        return E[:n, :n], E[:n, n:]


def _dlyap(Ad, Q):
    n = Ad.shape[0]
    rhs = Q.reshape(-1, order="F")
    lhs = np.eye(n * n) - np.kron(Ad.T, Ad.T)
    P = np.linalg.solve(lhs, rhs).reshape(n, n, order="F")
    return 0.5 * (P + P.T)


def _dare_sda(Ad, Bd, Q, R, maxit=80):
    n = Ad.shape[0]
    I = np.eye(n)
    Ak = np.array(Ad, dtype=float)
    G = Bd @ np.linalg.solve(R, Bd.T)
    H = np.array(Q, dtype=float)
    G = 0.5 * (G + G.T)
    H = 0.5 * (H + H.T)
    for _ in range(maxit):
        M = I + G @ H
        Minv = np.linalg.inv(M)
        An = Ak @ Minv @ Ak
        Gn = G + Ak @ Minv @ G @ Ak.T
        Hn = H + Ak.T @ H @ Minv @ Ak
        Gn = 0.5 * (Gn + Gn.T)
        Hn = 0.5 * (Hn + Hn.T)
        if np.linalg.norm(Hn - H, ord="fro") < 1e-12 * (1.0 + np.linalg.norm(Hn, ord="fro")):
            return Hn
        Ak, G, H = An, Gn, Hn
    return H


def _dare_kleinman(Ad, Bd, Q, R, P0):
    K = np.linalg.solve(R + Bd.T @ P0 @ Bd, Bd.T @ P0 @ Ad)
    P = P0
    for _ in range(40):
        Acl = Ad - Bd @ K
        if np.max(np.abs(np.linalg.eigvals(Acl))) >= 1.0 - 1e-12:
            break
        P = _dlyap(Acl, Q + K.T @ R @ K)
        Kn = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        if np.linalg.norm(Kn - K, ord="fro") < 1e-10:
            return P
        K = Kn
    return P


def dare(Ad, Bd, Q, R):
    try:
        from scipy.linalg import solve_discrete_are

        return np.asarray(solve_discrete_are(Ad, Bd, Q, R), dtype=float)
    except ImportError:
        P = _dare_sda(Ad, Bd, Q, R)
        P = _dare_kleinman(Ad, Bd, Q, R, P)
        K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        if np.max(np.abs(np.linalg.eigvals(Ad - Bd @ K))) >= 1.0:
            raise SystemExit("DARE 未得到稳定增益，检查 (A,B) 与 Q/R")
        return P


def bryson_qr(w):
    b = w["bryson"]
    th = math.radians(float(b["theta_deg"]))
    om = float(b["omega_rps"])
    s = float(b["s_m"])
    v = float(b["v_mps"])
    tau = float(b["tau_nm"])
    bars = np.array([th, om, s, v], dtype=float)
    if np.any(bars <= 0.0) or tau <= 0.0:
        raise SystemExit("Bryson 上限必须 > 0")
    Q = np.diag(1.0 / (bars * bars))
    R = np.array([[1.0 / (tau * tau)]])
    Q = Q * np.diag(np.array(w.get("q_scale", [1, 1, 1, 1]), dtype=float))
    R = R * float(w.get("r_scale", 1.0))
    return Q, R


def patch_header(src, k):
    names = ("kLqrPitch", "kLqrPitchRate", "kLqrPos", "kLqrVel")
    out = src
    for name, val in zip(names, k):
        out, n = re.subn(
            r"(constexpr float " + name + r"\s*=\s*)([^;]+)(;)",
            r"\g<1>{:.8g}f\g<3>".format(val),
            out,
            count=1,
        )
        if n != 1:
            raise SystemExit("无法写入 " + name)
    return out


def main():
    ap = argparse.ArgumentParser(description="从 config.h + lqr_weights.yaml 算离散 LQR 增益")
    ap.add_argument("--header", default=str(DEFAULT_HEADER))
    ap.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    ap.add_argument("--write", action="store_true", help="把 k=-K_sp 写回 config.h")
    args = ap.parse_args()

    plant, header_src = load_plant(args.header)
    with open(args.weights, encoding="utf-8") as f:
        weights = yaml.safe_load(f)

    A, B, Mq = continuous_ab(plant)
    Ad, Bd = zoh(A, B, plant["dt"])
    Q, R = bryson_qr(weights)
    P = dare(Ad, Bd, Q, R)
    Ksp = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    Ksp = np.asarray(Ksp, dtype=float).reshape(1, 4)
    k_fw = (-Ksp).ravel()

    ol = np.linalg.eigvals(A)
    cl = np.linalg.eigvals(Ad - Bd @ Ksp)

    print("plant  M={M} m={m} L={L} I={I} r={r} g={g}  dt={dt} ({hz:g} Hz)".format(**plant))
    print("Mq =\n", Mq)
    print("a21={:.4g}  a41={:.4g}  (开环应有 a21>0)".format(A[1, 0], A[3, 0]))
    print("开环 eig(A) =", np.array2string(ol, precision=4))
    print("Q diag =", np.diag(Q))
    print("R =", float(R[0, 0]))
    print("K_sp (u=-K x) =", Ksp.ravel())
    print("固件 k = -K_sp =", k_fw)
    print("闭环 |λ| =", np.abs(cl), "  max={:.6f}".format(np.max(np.abs(cl))))

    if not np.any(np.real(ol) > 1e-6):
        raise SystemExit("开环没有正实部极点，检查 M,m,L 与符号")
    if k_fw[0] <= 0.0:
        raise SystemExit("k_pitch<=0，不要写固件；先查 θ/τ 符号")
    if np.max(np.abs(cl)) >= 1.0:
        raise SystemExit("闭环有 |λ|>=1，不要写固件；改 Q/R")

    print("\n串口（m 3，不断电有效）：")
    print("k {:.8g} {:.8g} {:.8g} {:.8g}".format(k_fw[0], k_fw[1], k_fw[2], k_fw[3]))
    print("config.h 默认：")
    print("constexpr float kLqrPitch     = {:.8g}f;".format(k_fw[0]))
    print("constexpr float kLqrPitchRate = {:.8g}f;".format(k_fw[1]))
    print("constexpr float kLqrPos       = {:.8g}f;".format(k_fw[2]))
    print("constexpr float kLqrVel       = {:.8g}f;".format(k_fw[3]))

    if args.write:
        Path(args.header).write_text(patch_header(header_src, k_fw), encoding="utf-8")
        print("已写入", args.header)


if __name__ == "__main__":
    main()
