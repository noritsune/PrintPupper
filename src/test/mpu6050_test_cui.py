from mpu6050 import mpu6050
from time import sleep
import numpy as np

# カルマンフィルタ用パラメータ
Q_angle = 0.001  # 角度のプロセスノイズ
Q_gyro = 0.003   # ジャイロバイアスのプロセスノイズ
R_angle = 0.03   # 角度の観測ノイズ

def get_acc_angle(accel_data):
    # 加速度からピッチ・ロール角度を算出（ラジアン）
    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']
    pitch = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    roll = np.arctan2(ax, np.sqrt(ay**2 + az**2))
    return pitch, roll

# カルマンフィルタ状態
kalman_pitch = 0.0
kalman_roll = 0.0
bias_pitch = 0.0
bias_roll = 0.0
P_pitch = np.zeros((2,2))
P_roll = np.zeros((2,2))

def kalman_filter(angle, rate, dt, x_angle, x_bias, P):
    # 予測
    rate_unbiased = rate - x_bias
    x_angle += dt * rate_unbiased
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_gyro * dt
    # 更新
    y = angle - x_angle
    S = P[0][0] + R_angle
    K = [P[0][0]/S, P[1][0]/S]
    x_angle += K[0] * y
    x_bias  += K[1] * y
    P00_temp = P[0][0]
    P01_temp = P[0][1]
    P[0][0] -= K[0] * P00_temp
    P[0][1] -= K[0] * P01_temp
    P[1][0] -= K[1] * P00_temp
    P[1][1] -= K[1] * P01_temp
    return x_angle, x_bias, P

sensor = mpu6050(0x68)
prev_time = None
while True:
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()
    temp = sensor.get_temp()
    now = None
    try:
        now = sleep._time()
    except:
        now = None
    if prev_time is None:
        dt = 0.2
    else:
        dt = 0.2 if now is None else max(0.01, min(0.5, now - prev_time))
    prev_time = now
    # 加速度から角度
    acc_pitch, acc_roll = get_acc_angle(accel_data)
    # ジャイロ値（deg/s→rad/s）
    gyro_pitch = gyro_data['x'] * np.pi / 180.0
    gyro_roll = gyro_data['y'] * np.pi / 180.0
    # カルマンフィルタ
    kalman_pitch, bias_pitch, P_pitch = kalman_filter(acc_pitch, gyro_pitch, dt, kalman_pitch, bias_pitch, P_pitch)
    kalman_roll, bias_roll, P_roll = kalman_filter(acc_roll, gyro_roll, dt, kalman_roll, bias_roll, P_roll)
    print("PITCH(deg): %7.2f ROLL(deg): %7.2f" % (kalman_pitch*180/np.pi, kalman_roll*180/np.pi), end=' ')
    print("GYR x" + "%7.2f" %  gyro_data['x'] + " y" + "%7.2f" %  gyro_data['y'] + " z" + "%7.2f" %  gyro_data['z'], end=' ')
    print("ACC X" + "%7.2f" % accel_data['x'] + " Y" + "%7.2f" % accel_data['y'] + " Z" + "%7.2f" % accel_data['z'], end=' ')
    print("TEMP " + "%4.1f" % temp + "C")
    sleep(0.2)
