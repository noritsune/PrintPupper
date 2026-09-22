"""強化学習で獲得した方策で歩く。従来の Controller (歩容生成 + IK) は使わない。

方策は ~/pupper_rl で学習したもの (out/train/u_stand)。段差歩行と転倒復帰を
1つのネットワークに統合してあり、**外から与えるモード信号は無い**。
入力は IMU だけ (関節エンコーダは使わない)。

    IMU -> ObsBuilder(38次元) -> NpzPolicy -> 関節角 -> joint_map
        -> HardwareInterface.set_actuator_positions -> pigpio PWM

従来の run_robot.py との違い:
  - 制御周期は 50 Hz (config.dt = 0.01 の 100 Hz ではない)。方策がその周期で
    学習されているので変えてはいけない。IMU と姿勢推定だけ 200 Hz で回す。
  - 歩容は方策の中にある。trot / rest の状態遷移も z 高さもピッチも使わない。
    ジョイスティックからは速度指令 [vx, vy, wz] だけを取る。
  - 膝の4節リンクと PWM 変換は従来どおり HardwareInterface に任せる
    (実機 v0.2 の寸法を持っているのはこちら。src/rl/check_mapping.py 参照)。

使い方:
    python3 src/run_robot_rl.py                 # ジョイスティックで操縦
    python3 src/run_robot_rl.py --no-servo      # サーボを動かさず値だけ見る
    python3 src/run_robot_rl.py --no-joystick --cmd 0.15 0 0   # 固定指令
    python3 src/run_robot_rl.py --bench         # 推論時間だけ測る (母艦でも可)

安全:
  - 起動直後はサーボを動かさない。L1 で活性化したあと --ramp 秒かけて
    中立姿勢から方策の基準姿勢へ滑らかに移す。
  - 既定では**転倒復帰をしない**。傾きゲートが下がったまま --fall-timeout 秒
    続いたらサーボを切って待機に戻る。復帰まで試すなら --recover を付ける。
    (復帰は可動域を 1.6 rad まで開くので、最初の通電では外すこと)
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "rl"))

from rl.numpy_policy import NpzPolicy
from rl.pupper_rl_deploy import ObsBuilder, DEFAULT_POSE, upright_gate
from rl import joint_map as jm

POLICY_NPZ = os.path.join(HERE, "rl", "policy_u_stand.npz")

CTRL_DT = 0.02              # 50 Hz。方策の学習周期。変えないこと
IMU_SUBSTEPS = 4            # IMU と姿勢推定は 200 Hz
IMU_DT = CTRL_DT / IMU_SUBSTEPS

# 歩容位相の周波数。学習は 0.7-1.2Hz を一様に引いているが、1.2 固定が最良
# (40mm 段差の踏破率 33% -> 54%)。位相は機体が内部で作る量なので固定してよい。
GAIT_FREQ = 1.2

# 学習時の速度指令の範囲 (pupper_rl の env/constants.py CMD_VX/VY/WZ)。
# ジョイスティックの最大値 (0.32 / 0.16 / 1.2) はこれを少し超えるので必ず切る。
CMD_LIMIT = np.array([0.30, 0.15, 1.00], np.float32)

# 傾きゲートがこれを下回ったら「転んでいる」とみなす (学習側と同じしきい値)
GATE_FALLEN = 0.5


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-servo", action="store_true",
                    help="サーボに出力しない。IMU と方策の値だけ表示する")
    ap.add_argument("--no-joystick", action="store_true",
                    help="ジョイスティックを使わず --cmd の固定指令で動かす")
    ap.add_argument("--cmd", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("VX", "VY", "WZ"), help="--no-joystick のときの指令")
    ap.add_argument("--recover", action="store_true",
                    help="転倒復帰も許す (可動域が 1.6 rad まで開く)")
    ap.add_argument("--fall-timeout", type=float, default=0.5,
                    help="転んだまま何秒でサーボを切るか (--recover 時は無効)")
    ap.add_argument("--ramp", type=float, default=1.5,
                    help="活性化時に基準姿勢へ移す秒数")
    ap.add_argument("--secs", type=float, default=0.0,
                    help="この秒数で自動停止 (0 で無制限)")
    ap.add_argument("--bench", action="store_true",
                    help="推論時間だけ測って終わる (ハードウェア不要)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--print-hz", type=float, default=2.0)
    return ap.parse_args()


def bench(n=2000):
    pol = NpzPolicy(POLICY_NPZ)
    rng = np.random.default_rng(0)
    obs = rng.normal(size=(n, pol.obs_dim)).astype(np.float32)
    for i in range(100):
        pol(obs[i])
    t = time.perf_counter()
    for o in obs:
        pol(o)
    dt = (time.perf_counter() - t) / n
    print(f"方策の1推論: {dt * 1e6:.1f} us  ({1 / dt:.0f} Hz 相当)")
    print(f"制御周期 {CTRL_DT * 1000:.0f} ms に対して {100 * dt / CTRL_DT:.2f} %")
    if dt > 0.002:
        print("警告: 2ms を超えている。numpy が BLAS 無しでビルドされていないか確認")


def main():
    args = parse_args()
    if args.bench:
        bench()
        return

    from Config import Configuration
    from State import RState, State
    from rl.imu import Imu

    config = Configuration()
    pol = NpzPolicy(POLICY_NPZ, obs_dim=38)
    ob = ObsBuilder(gait_freq=GAIT_FREQ, unified=True)

    print("IMU を初期化...", end="", flush=True)
    imu = Imu(bus=args.i2c_bus)
    print(f" {imu.chip}")
    print("機体を水平に静止させたままジャイロを校正中 (2秒)...")
    cal = imu.calibrate_gyro(2.0)
    print(f"  ジャイロバイアス {np.degrees(cal['gyro_bias'])} deg/s")
    print(f"  |accel| {cal['accel_norm']:.3f} m/s^2 (期待 9.807)")
    print(f"  取り付け傾き {cal['tilt_deg']:.2f} deg "
          f"{'OK' if cal['tilt_deg'] <= 1.7 else '<- 学習の想定 1.7deg を超えている'}")

    hw = None
    if not args.no_servo:
        from HardwareInterface import HardwareInterface
        hw = HardwareInterface(config)

    state = State()
    joystick = None
    if not args.no_joystick:
        from JoystickInterface import JoystickInterface
        print("ジョイスティックを待っています...", end="", flush=True)
        joystick = JoystickInterface(config)
        joystick.set_color(config.ps4_deactivated_color)
        print(" OK")

    fixed_cmd = np.clip(np.array(args.cmd, np.float32), -CMD_LIMIT, CMD_LIMIT)

    try:
        while True:
            if not wait_for_activate(args, config, state, hw, joystick, imu):
                break
            run_policy(args, config, state, hw, joystick, imu, pol, ob, fixed_cmd)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        shutdown(hw, joystick, config, imu)


def wait_for_activate(args, config, state, hw, joystick, imu):
    """L1 が押されるまで待つ。--no-joystick ならすぐ始める。"""
    if joystick is None:
        input("Enter で開始 (Ctrl-C で終了) ")
        return True
    print("L1 で活性化")
    blink = 0
    WAIT_DT = 0.01          # この待機ループの周期。imu.update に渡す dt と揃える
    while True:
        imu.update(WAIT_DT)
        command = joystick.get_command(state)
        if command.activate_event == 1:
            return True
        if blink % 100 == 0 and hw is not None:
            hw.set_led_green(bool((blink // 100) % 2))
        blink += 1
        time.sleep(WAIT_DT)


def run_policy(args, config, state, hw, joystick, imu, pol, ob, fixed_cmd):
    """活性化してから停止するまでの 50 Hz ループ。"""
    ob.reset()
    if hw is not None:
        hw.set_led_green(True)
        ramp_to_nominal(hw, args.ramp)
    if joystick is not None:
        joystick.set_color(config.ps4_activated_color)
    print("方策 開始")

    t0 = time.perf_counter()
    next_tick = t0
    fallen_since = None
    worst_late = 0.0
    last_print = t0
    steps = 0

    while True:
        # --- IMU と姿勢推定を 200 Hz で回す -------------------------------
        for _ in range(IMU_SUBSTEPS):
            imu.update(IMU_DT)

        # --- 指令 ---------------------------------------------------------
        if joystick is not None:
            command = joystick.get_command(state)
            if command.activate_event:
                print("方策 停止 (L1)")
                break
            cmd = np.array([command.horizontal_velocity[0],
                            command.horizontal_velocity[1],
                            command.yaw_rate], np.float32)
            cmd = np.clip(cmd, -CMD_LIMIT, CMD_LIMIT)
        else:
            cmd = fixed_cmd

        # --- 推論。順序は step -> 推論 -> commit を崩さないこと ------------
        obs = ob.step(imu.gyro, imu.accel, imu.gravity, cmd)
        action = pol(obs)
        joint_rad = ob.commit(action)

        # --- 転倒の扱い ---------------------------------------------------
        gate = upright_gate(imu.gravity)
        now = time.perf_counter()
        if gate < GATE_FALLEN and not args.recover:
            fallen_since = fallen_since or now
            if now - fallen_since > args.fall_timeout:
                print(f"転倒を検出 (ゲート {gate:.2f})。--recover が無いので停止")
                break
        else:
            fallen_since = None

        # --- 出力 ---------------------------------------------------------
        if hw is not None:
            hw.set_actuator_positions(jm.rl_to_printpupper(joint_rad))

        steps += 1
        if args.secs and (now - t0) > args.secs:
            print(f"--secs {args.secs} に到達")
            break

        if now - last_print > 1.0 / args.print_hz:
            last_print = now
            up = -imu.gravity[2]
            print(f"t {now-t0:6.1f}s  指令 [{cmd[0]:+.2f} {cmd[1]:+.2f} {cmd[2]:+.2f}]  "
                  f"水平度 {up:+.2f} ゲート {gate:.2f} 可動域 {ob._scale:.2f}rad  "
                  f"遅れ最大 {worst_late*1000:.1f}ms  加速度棄却 {imu.accel_rejected}")

        # --- 次の刻みまで待つ ---------------------------------------------
        next_tick += CTRL_DT
        late = time.perf_counter() - next_tick
        worst_late = max(worst_late, late)
        if late > 0:
            # 学習側が見込んでいる遅延は 1-3 step (20-60ms)。そこを超えたら
            # 間引かずに位相を作り直す (遅れを溜めない)
            if late > 3 * CTRL_DT:
                print(f"警告: ループが {late*1000:.0f}ms 遅れた。刻みを作り直す")
                next_tick = time.perf_counter() + CTRL_DT
        else:
            time.sleep(-late)

    hz = steps / max(time.perf_counter() - t0, 1e-6)
    print(f"方策 終了  {steps} step / 実効 {hz:.1f} Hz / 遅れ最大 {worst_late*1000:.1f}ms")
    if hw is not None:
        ramp_to_nominal(hw, 0.8)
        hw.set_led_green(False)
    if joystick is not None:
        joystick.set_color(config.ps4_deactivated_color)


def ramp_to_nominal(hw, secs):
    """サーボの中立姿勢から方策の基準姿勢へ、指定秒かけて滑らかに移す。

    通電直後にいきなり基準姿勢を指令すると 12個が同時に飛んで機体が跳ねる。
    """
    steps = max(int(secs / CTRL_DT), 1)
    # 中立 = abduction 0 / hip 0 / knee(beta) pi は脚が伸びきった姿勢なので、
    # そこからではなく「膝だけ基準値・他は 0」から始める
    start = np.array([0.0, 0.0, -np.pi / 2] * 4)
    for i in range(steps + 1):
        a = i / steps
        q = (1 - a) * start + a * DEFAULT_POSE
        hw.set_actuator_positions(jm.rl_to_printpupper(q))
        time.sleep(CTRL_DT)


def shutdown(hw, joystick, config, imu):
    print("サーボを解放")
    if hw is not None:
        hw.deactivate()
        hw.set_led_green(False)
        hw.set_led_blue(False)
    if joystick is not None:
        joystick.set_color(config.ps4_deactivated_color)
    try:
        imu.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
