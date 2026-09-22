# 強化学習方策での歩行 (`run_robot_rl.py`)

`~/pupper_rl` で学習した方策で歩く。従来の `run_robot.py` の歩容生成 + 逆運動学
(`Controller` / `Kinematics`) は使わない。

```
IMU(MPU6050/9250) -> 姿勢推定 -> ObsBuilder(38次元) -> 方策(MLP) -> 関節角
   -> joint_map -> HardwareInterface.set_actuator_positions -> pigpio PWM
```

## 何が要るか

- **IMU だけ**。関節エンコーダは使わない (方策の自己受容感覚は自分の行動履歴)。
  6軸 IMU を I2C に付け、`rl/imu.py` の `AXIS_MAP` / `AXIS_SIGN` を取り付け向きに
  合わせる。機体座標は **x 前 / y 左 / z 上**。
- Python: `numpy`, `smbus2`, `pigpio` (既存の依存に `smbus2` が増えるだけ)。
- 方策ファイル `rl/policy_u_stand.npz` (208KB) はこのリポジトリに含めてある。

ラズパイ3 model B で **1推論 0.3-0.7ms** 程度。制御周期 20ms のうち数 % で、
計算は余っている。

## 使い方

```bash
# 推論時間だけ測る (ハードウェア不要。母艦でも走る)
python3 src/run_robot_rl.py --bench

# サーボを動かさず、IMU と方策の値だけ見る (最初はこれ)
python3 src/run_robot_rl.py --no-servo

# 固定指令で前進 (ジョイスティック無し)
python3 src/run_robot_rl.py --no-joystick --cmd 0.15 0 0 --secs 5

# ジョイスティックで操縦 (L1 で活性化 / もう一度 L1 で停止)
python3 src/run_robot_rl.py
```

左スティックが `vx, vy`、右スティックが `wz`。指令は学習時の範囲
(`vx +-0.30 / vy +-0.15 / wz +-1.00 m/s, rad/s`) に切ってから方策に渡す。
ジョイスティックの最大値はこれを少し超えるため。

**既定では転倒復帰をしない。** 転んだまま 0.5 秒たつとサーボを切って待機に戻る。
復帰まで試すなら `--recover` を付ける (可動域が 0.25 -> 1.6 rad に開くので、
最初の通電では外すこと)。

## 立ち上げの順番

```bash
python3 src/rl/check_mapping.py     # 関節規約の変換が正しいか (母艦で可)
python3 src/rl/check_chain.py       # 経路まるごと (母艦で可)
python3 src/run_robot_rl.py --bench # ラズパイ上での推論時間
python3 src/run_robot_rl.py --no-servo   # IMU の向きと傾きゲートを目視
# ここで機体を手で傾け、「水平度」が +1 -> 0 -> -1 と動き、
# 「可動域」が 0.25 -> 1.6 に切り替わることを確認する
python3 src/run_robot_rl.py --no-joystick --cmd 0 0 0 --secs 5   # 静止
python3 src/run_robot_rl.py --no-joystick --cmd 0.15 0 0 --secs 5 # 前進
```

起動時に IMU を **水平に静止**させておくこと。ジャイロバイアスを 2 秒測り、
そのときの取り付け傾きも表示する。学習の想定は **1.7 度以内**で、ここを超えると
歩行中に復帰側の可動域へ落ちることがある。

## 関節角の規約 (`joint_map.py`)

2つのスタックは関節の数え方も脚の並びも違う。接続点はここ1か所だけ。

| | pupper_rl (MuJoCo) | PrintPupper v0.2 |
|---|---|---|
| 脚の並び | lf, rf, lh, rh | 列 0-3 = FR, FL, BR, BL |
| 股 | `q_hip` | `abduction_angle` |
| 腿 | `q_upper` | `hip_angle` |
| 膝 | `q_lower` | `beta` (L1-L2 の内角) |

```
abduction_angle = q_hip
hip_angle       = q_upper
beta            = q_lower + pi
```

基準姿勢 `[0, pi/4, -pi/2]` は `abduction 0 / hip 45deg / beta 90deg` になる。

`check_mapping.py` がこれを数値で裏付ける: ホーン長を揃えると、この変換を
通した PrintPupper の `compute_unparallel_link_knee` と pupper_rl の
`env/deploy.py` の `knee_servo_angle` が **ちょうど 90.0000 deg** オフセットで
一致する (20,000 姿勢でばらつき 1e-13 deg)。別々に書かれた2実装が一致するので、
脚の並び・角度規約・符号がすべて合っている。

## 既知の食い違い: 膝サーボホーンの長さ

`rl/pupper_rl_deploy.py` (= `~/pupper_rl/env/deploy.py` の逐語コピー) の
4節リンク計算は **PrintPupper_ROS という別フォークの寸法**で書かれていて、
ホーンが `_L3 = 50mm`。実機 v0.2 は `Config.LEG_UNPRALLEL_L4 = 60mm`。

そのため**実機では PrintPupper 自身の `HardwareInterface` (60mm) を通す**。
`pupper_rl_deploy.py` のうち使ってよいのは上半分 (`ObsBuilder` /
`upright_gate` / `unified_action_scale` / `DEFAULT_POSE`) だけ。

影響 (`check_mapping.py` の実測、サーボ角の差):

| 可動域 | 中央 | 95% | 最大 |
|---|---|---|---|
| 歩行 (+-0.25 rad) | 1.4 deg | 3.8 deg | **4.2 deg** |
| 復帰 (+-1.6 rad) | 7.0 deg | 24.6 deg | 51.7 deg |

**歩行では 4.2 度以内**に収まるので、そのまま載せてよい。ただし学習側の
`linkage_infeasible` ペナルティは 50mm で形を決めていたので、実機 v0.2 の
実現可能域とは厳密には一致しない。復帰 (`--recover`) まで詰めるなら、
`~/pupper_rl/env/linkage.py` の `L3_LEN` を 60.0、`OFF_X/OFF_Y` を
v0.2 の値に直して学習し直すのが筋。

## ファイル

| | |
|---|---|
| `../run_robot_rl.py` | 50Hz 制御ループ本体 |
| `numpy_policy.py` | 方策の推論 (numpy だけ。silu であることに注意) |
| `policy_u_stand.npz` | 重み。`~/pupper_rl/scripts/export_npz.py` が書き出す |
| `pupper_rl_deploy.py` | `~/pupper_rl/env/deploy.py` の逐語コピー |
| `imu.py` | MPU6050/9250 + 相補フィルタ |
| `joint_map.py` | 関節規約の変換 |
| `check_mapping.py` | 変換の検証 (母艦で走る) |
| `check_chain.py` | 経路まるごとの検証 (母艦で走る) |

`pupper_rl_deploy.py` は上流と diff が取れるよう**書き換えないこと**。
上流を直したら丸ごとコピーし直す。方策を学習し直したら:

```bash
cd ~/pupper_rl && python scripts/export_npz.py --ckpt out/train/<run>/ckpt/final.pkl \
    --out ~/PrintPupper/src/rl/policy_u_stand.npz
```
