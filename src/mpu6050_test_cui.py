from mpu6050_kalman import Mpu6050Kalman
from time import sleep

# main
sensor = Mpu6050Kalman(0x68)
while True:
    pitch, roll = sensor.get_pitch_roll()
    gyro_data = sensor.sensor.get_gyro_data()
    accel_data = sensor.sensor.get_accel_data()
    temp = sensor.sensor.get_temp()
    print("PITCH(deg): %7.2f ROLL(deg): %7.2f" % (pitch, roll), end=' ')
    print("GYR x" + "%7.2f" %  gyro_data['x'] + " y" + "%7.2f" %  gyro_data['y'] + " z" + "%7.2f" %  gyro_data['z'], end=' ')
    print("ACC X" + "%7.2f" % accel_data['x'] + " Y" + "%7.2f" % accel_data['y'] + " Z" + "%7.2f" % accel_data['z'], end=' ')
    print("TEMP " + "%4.1f" % temp + "C")
    sleep(0.2)
