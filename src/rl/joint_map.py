"""pupper_rl (MuJoCo) の関節角 <-> PrintPupper v0.2 の joint_angles[axis, leg]。

2つのスタックは関節角の規約も脚の並びも違う。ここが唯一の接続点で、
**ここを間違えると静止指令でいきなり倒れる**。導出と数値検証は
src/rl/check_mapping.py にある (ホーン長を 50mm に揃えると、この変換を通した
PrintPupper の compute_unparallel_link_knee と pupper_rl の
env/deploy.py の knee_servo_angle が **ちょうど 90.0000 deg** オフセットで
一致する。20,000 姿勢でのばらつき 8e-05 deg)。

-------------------------------------------------------------------------
脚の並び
-------------------------------------------------------------------------
  pupper_rl  : lf, rf, lh, rh            (左前, 右前, 左後, 右後)
  PrintPupper: 列 0..3 = FR, FL, BR, BL  (Config.py のピン表と
               HardwareInterface.initialize_pwm の print と同じ)

-------------------------------------------------------------------------
関節角
-------------------------------------------------------------------------
MuJoCo 側 (model/pupper_unified.xml): 4脚とも hip 軸 = (1,0,0)、
upper_leg / lower_leg 軸 = (0,1,0)。基準姿勢は [0, pi/4, -pi/2]。

PrintPupper 側 (Kinematics.leg_explicit_inverse_kinematics の戻り値):
  axis 0 abduction_angle : 脚平面が鉛直なら 0
  axis 1 hip_angle       : 大腿の、鉛直下向きからの角度
  axis 2 knee_angle      : L1 と L2 のなす**内角 beta** (0..pi、まっすぐで pi)

対応 (4脚とも同じ。左右前後の符号は servo_multipliers が持っている):

    abduction_angle = q_hip
    hip_angle       = q_upper
    beta            = q_lower + pi

基準姿勢 [0, pi/4, -pi/2] は abduction 0 / hip 45deg / beta 90deg になり、
PrintPupper の既定の立ち姿勢と同じ形。q_lower の可動域 [-2.9671, -0.5934] に
対して beta は [0.174, 2.548] で、常に (0, pi) に収まるので折り返しは要らない。
"""
import numpy as np

# pupper_rl の脚順 (lf, rf, lh, rh) から見た、PrintPupper の列番号
#   PrintPupper 列: 0=FR, 1=FL, 2=BR, 3=BL
PP_COL_OF_RL_LEG = np.array([1, 0, 3, 2])      # lf->FL, rf->FR, lh->BL, rh->BR
RL_LEG_OF_PP_COL = np.array([1, 0, 3, 2])      # 対合なので同じ並び

AXIS_ABDUCTION, AXIS_HIP, AXIS_KNEE = 0, 1, 2


def rl_to_printpupper(q12):
    """pupper_rl の 12 関節角 [rad] -> PrintPupper の joint_angles (3, 4)。

    q12: (12,) lf, rf, lh, rh の順に (hip, upper_leg, lower_leg)。
         env/deploy.py の ObsBuilder.joint_targets() が返すもの。
    戻り: (3, 4) float64。HardwareInterface.set_actuator_positions にそのまま渡す。
          **set_actuator_positions は配列を破壊的に書き換える**ので、
          毎回新しい配列を返している。
    """
    q = np.asarray(q12, np.float64).reshape(4, 3)
    out = np.empty((3, 4), np.float64)
    for rl_leg in range(4):
        col = PP_COL_OF_RL_LEG[rl_leg]
        out[AXIS_ABDUCTION, col] = q[rl_leg, 0]
        out[AXIS_HIP, col] = q[rl_leg, 1]
        out[AXIS_KNEE, col] = q[rl_leg, 2] + np.pi
    return out


def printpupper_to_rl(joint_angles):
    """逆変換。検証とキャリブレーション確認に使う。"""
    a = np.asarray(joint_angles, np.float64).reshape(3, 4)
    q = np.empty((4, 3), np.float64)
    for rl_leg in range(4):
        col = PP_COL_OF_RL_LEG[rl_leg]
        q[rl_leg, 0] = a[AXIS_ABDUCTION, col]
        q[rl_leg, 1] = a[AXIS_HIP, col]
        q[rl_leg, 2] = a[AXIS_KNEE, col] - np.pi
    return q.ravel()
