import numpy as np
from mpu6050 import mpu6050

class Mpu6050Kalman:
    def __init__(self, address=0x68):
        self.sensor = mpu6050(address)
        self.Q_angle = 0.001
        self.Q_gyro = 0.003
        self.R_angle = 0.03
        self.kalman_pitch = 0.0
        self.kalman_roll = 0.0
        self.bias_pitch = 0.0
        self.bias_roll = 0.0
        self.P_pitch = np.zeros((2,2))
        self.P_roll = np.zeros((2,2))
        self.prev_time = None

    def get_acc_angle(self, accel_data):
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']
        pitch = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        roll = np.arctan2(ax, np.sqrt(ay**2 + az**2))
        return pitch, roll

    def kalman_filter(self, angle, rate, dt, x_angle, x_bias, P):
        Q_angle = self.Q_angle
        Q_gyro = self.Q_gyro
        R_angle = self.R_angle
        rate_unbiased = rate - x_bias
        x_angle += dt * rate_unbiased
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += Q_gyro * dt
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

    def get_pitch_roll(self):
        gyro_data = self.sensor.get_gyro_data()
        accel_data = self.sensor.get_accel_data()
        now = None
        import time
        now = time.time()
        if self.prev_time is None:
            dt = 0.02
        else:
            dt = max(0.005, min(0.1, now - self.prev_time))
        self.prev_time = now
        acc_pitch, acc_roll = self.get_acc_angle(accel_data)
        gyro_pitch = gyro_data['x'] * np.pi / 180.0
        gyro_roll = gyro_data['y'] * np.pi / 180.0
        self.kalman_pitch, self.bias_pitch, self.P_pitch = self.kalman_filter(acc_pitch, gyro_pitch, dt, self.kalman_pitch, self.bias_pitch, self.P_pitch)
        self.kalman_roll, self.bias_roll, self.P_roll = self.kalman_filter(acc_roll, gyro_roll, dt, self.kalman_roll, self.bias_roll, self.P_roll)
        return self.kalman_pitch, self.kalman_roll
