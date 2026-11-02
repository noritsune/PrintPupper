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

## ルールベース歩行との切り替え (`robot.service`)

`robot.service` で起動する `run_robot.py` は、従来のルールベース歩行 (既定) と
方策歩行をジョイスティックで切り替えられる。

| 操作 | 動作 |
|---|---|
| 11ボタン | ルールベース歩行 |
| 12ボタン | 方策歩行 |
| 青LED | 方策歩行のとき点灯、ルールベースのとき消灯 |

- 待機中に押すとモードを選ぶだけで、L1 で選んだモードが活性化する。
- 歩行中に押すと、いまの姿勢から 0.8 秒かけて相手の姿勢へ移り、そのまま切り替わる。
- 方策歩行は転倒復帰を有効にして動く (`--recover` 相当)。起き上がれないまま 4 秒たつと
  基準姿勢に戻してやり直し、起き上がるか L1 で止めるまで繰り返す。
- L1 で止めたときは、方策歩行を選んだまま待機に戻る。
- IMU の初期化とジャイロ校正は起動時に 1 回だけ行う (起動時は機体を水平に静止)。
  IMU が見つからないときは方策歩行を無効にして、ルールベースだけで動く。
- 11/12 は USB ゲームパッド (F710 の D モード) のボタン番号。12 は R3 (お手の PUSH) と
  同じボタンなので、ルールベースで押すと方策歩行に切り替わる。PS4 コントローラーでは
  切り替えられない。

## 使い方 (単体で動かす)

`robot.service` (`run_robot.py`) と同じサーボを使うので、先に止める。
pigpiod は `robot.service` が起動しているので、止めると一緒に止まる。

```bash
sudo systemctl stop robot
sudo pigpiod
```

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
最初の通電では外すこと)。`--recover` では、転んだまま `--retry-secs` 秒 (既定 4 秒)
たつと基準姿勢に戻し、行動履歴を消してからもう一度試す。起き上がれるまで繰り返す。

転んでいる間は左右を反転して方策に渡す (`ObsBuilder` の `mirror_fallen`、学習側も同じ規則)。
倒れ込んで水平度 (`-重力 z`) が 0.5 以下になった最初のステップで重力 y を見て、
左下 (y > 0) なら IMU の値と行動履歴を左右反転して観測にし、方策が出した行動も
反転し戻してから関節に送る。向きはゲートが 0.5 を超える (起き上がる) まで固定する。
方策から見ると転倒は常に右下側になるので、左下と右下で起き上がり方が同じになる。
反転中は状態表示に `左右反転` が出る。

## 立ち上げの順番

```bash
python3 src/rl/check_mapping.py     # 関節規約の変換が正しいか (母艦で可)
python3 src/rl/check_chain.py       # 経路まるごと (母艦で可)
python3 src/run_robot_rl.py --bench # ラズパイ上での推論時間
python3 src/run_robot_rl.py --no-servo   # IMU の向きと傾きゲートを目視
# ここで機体を手で傾け、「水平度」が +1 -> 0 -> -1 と動き、
# 「可動域」が 0.25 -> 1.6 に切り替わることを確認する。
# 「重力」は機体座標 (x 前 / y 左 / z 上) で、左を下にすると [0 +1 0]、
# 右を下にすると [0 -1 0]、鼻先を下げると x が + になる
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

## 膝サーボホーンの長さ

方策 `policy_u_stand.npz` は `~/pupper_rl/out/train/m60_stand` から書き出したもので、
学習側の `env/linkage.py` は実機 v0.2 と同じホーン `L3_LEN = 60mm`
(`Config.LEG_UNPRALLEL_L4`) で `linkage_infeasible` ペナルティを掛けている。
`OFF_X/OFF_Y = (5, -25)` は v0.2 の `UNPRALLEL_ofstX/Y = (-5, 25)` と座標系の
向きが違うだけで同じ位置。

一方、`rl/pupper_rl_deploy.py` (= `~/pupper_rl/env/deploy.py` の逐語コピー) の
下半分の4節リンク計算は **PrintPupper_ROS という別フォークの寸法**
(`_L3 = 50mm`) のまま。そのため**実機では PrintPupper 自身の
`HardwareInterface` を通す**。`pupper_rl_deploy.py` のうち使ってよいのは上半分
(`ObsBuilder` / `upright_gate` / `unified_action_scale` / `DEFAULT_POSE`) だけ。

`check_mapping.py` の [3] は、この 50mm 版と実機 60mm のサーボ角の差を出す
(歩行の可動域で最大 4.2 deg、復帰の可動域で最大 51.7 deg)。

シミュレーションでの成績 (`~/pupper_rl` の評価スクリプト、ホーン 60mm、脚と胴体の自己衝突ありで評価):

| | 値 |
|---|---|
| 転倒復帰 (`recover_test --env unified`, 256本, 8 秒、やり直しなし) | 63.7% (仰向け 61.0 / 横倒し 75.0 / 正立 63.6) |
| 横倒しからの復帰 (左下 / 右下、4 秒ごとにやり直して 20 秒以内) | 86.6 / 85.2 %、成功までの時間の中央値 3.3 / 3.2 秒 |
| 仰向けからの復帰 (4 秒ごとにやり直して 20 秒以内) | 84.2 %、成功までの時間の中央値 6.4 秒 |
| 段差踏破 (`step_test --env unified --gait-freq 1.2 1.2`) | 10mm 100 / 20mm 91.7 / 30mm 50.0 / 40mm 8.3 % |
| 速度の追従率 (前進 0.2 / 後退 0.2 / 横 左 0.1 / 横 右 0.1 m/s) | 79 / 73 / 104 / 74 % |
| 4節リンク実現不能 / PWM 飽和 / トルク飽和 | 2.04% / 0.00% / 10.6% |

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
上流を直したら丸ごとコピーし直す。方策を学習し直したら、WSL の `~/pupper_rl` で
`deploy/policy_u_stand.npz` に書き出し、Windows から `-PullPolicy` 付きで同期する
(リポジトリの `src/rl/policy_u_stand.npz` を差し替えてからロボに送る):

```bash
cd ~/pupper_rl && python scripts/export_npz.py --ckpt out/train/<run>/ckpt/final.pkl
```

```powershell
.\sync_to_robot.ps1 -PullPolicy
```
