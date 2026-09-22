"""joint_map.py の検証。母艦 (PC) でも走る。pigpio も IMU も要らない。

やること:
  1. rl_to_printpupper / printpupper_to_rl の往復一致
  2. **変換の正しさの本体**: この変換を通した PrintPupper の
     compute_unparallel_link_knee (ホーン長を 50mm に揃えたもの) が、
     pupper_rl の env/deploy.py の knee_servo_angle と定数オフセットだけの差に
     なることを 20,000 姿勢で確認する。同じ機構を別の座標系で書いた2実装が
     一致するので、脚の並び・角度規約・符号がすべて合っていることの証明になる。
  3. **ホーン長の食い違いの影響**: 実機 v0.2 は 60mm、pupper_rl の学習側が
     仮定していたのは 50mm。その差がサーボ角で何度になるかを出す。
  4. 方策が出しうる関節指令が PWM 飽和しないかを、実機の
     ServoCalibration.py の中立値込みで数える。

使い方:
    python3 src/rl/check_mapping.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))          # src/ (Config.py など)

from Config import Configuration, ServoParams
from compute_unparallel_link_knee import compute_unparallel_link_knee

import joint_map as jm
from pupper_rl_deploy import DEFAULT_POSE, ACTION_SCALE, RECOVER_ACTION_SCALE

# HardwareInterface の中身の写し (pigpio / RPi.GPIO を import せずに検査するため)。
# HardwareInterface.extrapolation_unparallel_actuator_positions と
# angle_to_pwmdutycycle を変えたら、ここも合わせること。
_SP = ServoParams()
MULT = _SP.servo_multipliers
NEUTRAL = _SP.neutral_angles


def knee_servo_pp(culk, joint_angles, leg):
    """HardwareInterface と同じ手順で膝サーボ角 kneeX を出す。"""
    rad1 = joint_angles[1, leg] * MULT[1, leg]
    rad2 = joint_angles[2, leg] * MULT[2, leg]
    if leg in (0, 2):                       # FR, BR = ミラー脚
        theta_leg = -((math.pi / 2) - rad1)
        mirror = True
    else:                                   # FL, BL
        theta_leg = (math.pi / 2) + rad1
        mirror = False
    return culk.compute(theta_leg, rad2, mirror)


def pwm_usec(angle, axis, leg):
    """HardwareInterface.angle_to_pwmdutycycle と同じ (clip しない生値)。"""
    dev = (angle - NEUTRAL[axis, leg]) * MULT[axis, leg]
    return _SP.pwm_usec_neutral + _SP.pwm_usec_per_rad * dev


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    cfg = Configuration()
    culk60 = compute_unparallel_link_knee(cfg)          # 実機 v0.2: L4 = 60mm
    culk50 = compute_unparallel_link_knee(cfg)
    culk50.L4 = 0.050                                   # pupper_rl の学習側の仮定

    print("=" * 72)
    print("1. 往復変換")
    rng = np.random.default_rng(0)
    q = DEFAULT_POSE + rng.uniform(-0.5, 0.5, 12)
    err = np.abs(jm.printpupper_to_rl(jm.rl_to_printpupper(q)) - q).max()
    print(f"   rl -> PrintPupper -> rl の最大差 {err:.2e}  "
          f"{'OK' if err < 1e-12 else 'NG'}")
    a0 = np.degrees(jm.rl_to_printpupper(DEFAULT_POSE))
    print(f"   基準姿勢 -> abduction {a0[0,0]:.1f} / hip {a0[1,0]:.1f} / "
          f"knee(beta) {a0[2,0]:.1f} [deg]  (期待 0 / 45 / 90)")

    try:
        from pupper_rl_deploy import knee_servo_angle as rl_knee
    except ImportError:
        rl_knee = None

    for scale, name in ((ACTION_SCALE, "歩行"), (RECOVER_ACTION_SCALE, "復帰")):
        print("=" * 72)
        print(f"2-4. 可動域 {name} (基準姿勢 +- {scale} rad)  N=20000")
        d_ref, d_horn, sat, nan60, nan50, n = [], [], 0, 0, 0, 20000
        for _ in range(n):
            q12 = DEFAULT_POSE + rng.uniform(-scale, scale, 12)
            ja = jm.rl_to_printpupper(q12)
            for leg in range(4):
                k60 = knee_servo_pp(culk60, ja, leg)
                k50 = knee_servo_pp(culk50, ja, leg)
                if math.isnan(k60):
                    nan60 += 1
                if math.isnan(k50):
                    nan50 += 1
                if not (math.isnan(k60) or math.isnan(k50)):
                    d_horn.append(abs(wrap(k60 - k50)))
                # pupper_rl の独立実装との照合 (ホーン 50mm に揃えて比較)
                if rl_knee is not None and not math.isnan(k50):
                    rl_leg = jm.RL_LEG_OF_PP_COL[leg]
                    r = rl_knee(q12[rl_leg * 3 + 1], q12[rl_leg * 3 + 2])
                    if r is not None:
                        # ミラー脚は PrintPupper 側が出力も反転している
                        k = -k50 if leg in (0, 2) else k50
                        d_ref.append(wrap(k - r))
                # PWM 飽和 (実機の中立値込み)。膝は kneeX を書き戻した後の値。
                for axis in range(3):
                    v = ja[axis, leg]
                    if axis == 2:
                        if math.isnan(k60):
                            continue
                        v = k60 * MULT[2, leg]
                    us = pwm_usec(v, axis, leg)
                    if us < _SP.pwm_usec_min or us > _SP.pwm_usec_max:
                        sat += 1

        tot = n * 4
        if d_ref:
            d_ref = np.array(d_ref)
            off = np.median(d_ref)
            spread = np.abs(d_ref - off).max()
            ok = spread < np.radians(0.01)
            print(f"   [2] pupper_rl の knee_servo_angle との差: "
                  f"定数 {math.degrees(off):+.4f} deg、"
                  f"そこからのばらつき {math.degrees(spread):.2e} deg  "
                  f"{'OK (同じ機構と確認)' if ok else 'NG: 規約がずれている'}")
        d_horn = np.array(d_horn)
        print(f"   [3] ホーン 60mm(実機) vs 50mm(学習側の仮定) のサーボ角差: "
              f"中央 {math.degrees(np.median(d_horn)):.2f} / "
              f"95% {math.degrees(np.percentile(d_horn, 95)):.2f} / "
              f"最大 {math.degrees(d_horn.max()):.2f} deg")
        print(f"       リンク解なし: 60mm {100*nan60/tot:.2f}% / "
              f"50mm {100*nan50/tot:.2f}%")
        print(f"   [4] PWM 飽和: {100*sat/(tot*3):.2f}% "
              f"(12関節 x {n} 姿勢、ServoCalibration.py の中立値込み)")

    print("=" * 72)
    print("注: [3][4] は可動域の箱から一様に引いた姿勢での値で、方策が実際に")
    print("    通る軌道の統計ではない (pupper_rl 側の実測は 8秒 x 256本で")
    print("    リンク実現不能 0.71% / PWM 飽和 0.00%)。ここで見たいのは")
    print("    「歩行の可動域ならホーン長の食い違いは数度で収まる」こと。")


if __name__ == "__main__":
    main()
