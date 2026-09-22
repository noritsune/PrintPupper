"""実機に載せる経路をまるごと通して、値が正気か確かめる。母艦でも走る。

  合成 IMU -> ObsBuilder(38) -> NpzPolicy -> 関節角 -> joint_map
       -> compute_unparallel_link_knee(60mm) -> PWM [us]

見るもの:
  1. ~/pupper_rl があれば、あちらの NumpyPolicy (.pkl) + ObsBuilder と
     行動列が一致するか。ここが一致すれば、このリポジトリに持ち込んだ
     方策と観測の組み立ては上流と同一。
  2. 静止 IMU を食わせたとき、関節指令が基準姿勢の近くに収まるか。
  3. PWM が飽和しないか、1 ステップあたりの変化が大きすぎないか。

使い方:
    python3 src/rl/check_chain.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from Config import Configuration, ServoParams
from compute_unparallel_link_knee import compute_unparallel_link_knee

import joint_map as jm
from numpy_policy import NpzPolicy
from pupper_rl_deploy import ObsBuilder, DEFAULT_POSE
from check_mapping import knee_servo_pp, pwm_usec

G = 9.80665
CTRL_DT = 0.02
GAIT_FREQ = 1.2
STEPS = 500


def imu_stream(n):
    """合成 IMU。前半は正立で静止、後半はゆっくり 40 度まで傾ける。

    傾けるのは、傾きゲートの帯 (up 0.90 -> 0.50) に入ったときに可動域が
    0.25 -> 1.6 rad へ滑らかに切り替わることを確かめるため。
    """
    for k in range(n):
        if k < n // 2:
            tilt = 0.0
        else:
            tilt = math.radians(40.0) * (k - n // 2) / (n // 2)
        # ロール方向に tilt だけ傾いた機体から見た重力方向
        grav = np.array([0.0, math.sin(tilt), -math.cos(tilt)], np.float32)
        gyro = np.zeros(3, np.float32)
        accel = (-grav * G).astype(np.float32)
        yield gyro, accel, grav


def main():
    cfg = Configuration()
    culk = compute_unparallel_link_knee(cfg)
    sp = ServoParams()
    pol = NpzPolicy(os.path.join(HERE, "policy_u_stand.npz"), obs_dim=38)
    ob = ObsBuilder(gait_freq=GAIT_FREQ, unified=True)
    cmd = np.array([0.15, 0.0, 0.0], np.float32)

    ref = None
    ref_ob = None
    ckpt = os.path.expanduser("~/pupper_rl/out/train/u_stand/ckpt/final.pkl")
    if os.path.exists(ckpt):
        try:
            sys.path.insert(0, os.path.expanduser("~/pupper_rl/scripts"))
            sys.path.insert(0, os.path.expanduser("~/pupper_rl"))
            from policy import NumpyPolicy
            from env.deploy import ObsBuilder as RefObsBuilder
            ref = NumpyPolicy(ckpt)
            ref_ob = RefObsBuilder(gait_freq=GAIT_FREQ, unified=True)
        except Exception as e:
            print(f"(上流との照合はとばす: {e})")
            ref = None

    d_act = 0.0
    q_all, us_all, nan_knee = [], [], 0
    prev_us = None
    max_us_step = 0.0
    scales = []

    for gyro, accel, grav in imu_stream(STEPS):
        obs = ob.step(gyro, accel, grav, cmd)
        act = pol(obs)
        q = ob.commit(act)
        scales.append(ob._scale)
        if ref is not None:
            r_obs = ref_ob.step(gyro, accel, grav, cmd)
            r_act = ref(r_obs)
            ref_ob.commit(r_act)
            d_act = max(d_act, float(np.abs(act - r_act).max()))

        ja = jm.rl_to_printpupper(q)
        q_all.append(q)
        us = np.empty((3, 4))
        for leg in range(4):
            k = knee_servo_pp(culk, ja, leg)
            if math.isnan(k):
                nan_knee += 1
                k = 0.0
            for axis in range(3):
                v = k * sp.servo_multipliers[2, leg] if axis == 2 else ja[axis, leg]
                us[axis, leg] = pwm_usec(v, axis, leg)
        us_all.append(us)
        if prev_us is not None:
            max_us_step = max(max_us_step, float(np.abs(us - prev_us).max()))
        prev_us = us

    q_all = np.array(q_all)
    us_all = np.array(us_all)
    half = STEPS // 2

    print("=" * 72)
    if ref is not None:
        ok = d_act < 1e-6
        print(f"1. 上流 (~/pupper_rl の NumpyPolicy + ObsBuilder) との行動の差: "
              f"{d_act:.2e}  {'OK' if ok else 'NG: 持ち込みがずれている'}")
    else:
        print("1. 上流が見つからないので照合なし (実機では正常)")

    dev = np.abs(q_all[:half] - DEFAULT_POSE)
    print("=" * 72)
    print(f"2. 正立・静止で {half} step:")
    print(f"   基準姿勢からのずれ 中央 {np.degrees(np.median(dev)):.1f} deg / "
          f"最大 {np.degrees(dev.max()):.1f} deg")
    print(f"   可動域スケール 正立側 {min(scales[:half]):.3f} rad (期待 0.250)")
    print(f"   可動域スケール 傾けた側 最大 {max(scales[half:]):.3f} rad "
          f"(帯に入れば 0.25 -> 1.6 へ動く)")

    print("=" * 72)
    sat = ((us_all < sp.pwm_usec_min) | (us_all > sp.pwm_usec_max)).mean()
    print(f"3. PWM: 範囲 {us_all.min():.0f} - {us_all.max():.0f} us "
          f"(許容 {sp.pwm_usec_min} - {sp.pwm_usec_max})")
    print(f"   飽和 {100*sat:.2f}%  リンク解なし {nan_knee} 回 / {STEPS*4}")
    rate = max_us_step / sp.pwm_usec_per_rad / CTRL_DT
    print(f"   1 step あたりの最大変化 {max_us_step:.0f} us "
          f"(= {math.degrees(max_us_step / sp.pwm_usec_per_rad):.1f} deg / 20ms "
          f"= 目標角の変化率 {rate:.1f} rad/s)")
    print("   ※ これは**指令の変化率**であって関節が出す速度ではない。サーボは")
    print("     追従が間に合わなければ遅れるだけで、学習側の PD も同じ形で遅れる")
    print("     (sim の実測は関節速度 99%点 7.1 rad/s)。桁で外れていなければ正常。")
    if rate > 30.0:
        print("   警告: 目標角の変化率が大きすぎる。観測の組み立てを疑うこと")


if __name__ == "__main__":
    main()
