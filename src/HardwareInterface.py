import math
import numpy as np
import pigpio
import RPi.GPIO as GPIO
from Config import ServoParams, PWMParams
from compute_unparallel_link_knee import compute_unparallel_link_knee

LED_GREEN_GPIO = 0
LED_BLUE_GPIO = 1

class DummyPigpio:
   def set_PWM_frequency(self, user_gpio, frequency):
       return int(0)
   
   def set_PWM_range(self, user_gpio, range_):
       return int(0)
   
   def set_PWM_dutycycle(self, user_gpio, dutycycle):
       return int(0)

class HardwareInterface:
    def __init__(self, config):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
        GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)
        GPIO.output(LED_GREEN_GPIO, False)
        GPIO.output(LED_BLUE_GPIO, False)
        print('GPIO LED GREEN [ ', LED_GREEN_GPIO, 'pin ]', sep='')
        print('GPIO LED BLUE  [ ', LED_BLUE_GPIO, 'pin ]', sep='')

        self.config = config
        self.pigpio = pigpio.pi()
        # self.pigpio = DummyPigpio()    # for debug run
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm()
        self.culk = compute_unparallel_link_knee(self.config)
        self.compute_normal_kneeX = np.zeros(4)
        return

    def set_actuator_positions(self, joint_angles):
        self.extrapolation_unparallel_actuator_positions(joint_angles)
        self.send_servo_commands(joint_angles)
        return

    def set_actuator_position(self, joint_angle, axis, leg):
        self.send_servo_command(joint_angle, axis, leg)
        return

    def set_actuator_position_noncab(self, joint_angle, axis, leg):
        self.send_servo_command_noncab(joint_angle, axis, leg)
        return

    def deactivate(self):
        self.deactivate_servos()
        return

    def initialize_pwm(self):
        print('GPIO ', self.servo_params.pwm_freq, 'Hz ', self.servo_params.pwm_usec_range, 'range ', self.servo_params.pwm_usec_neutral, 'neutral',  sep='')
        for leg_index in range(4):
            if leg_index == 0:
                print('GPIO FR-[ ', end='')
            if leg_index == 1:
                print('GPIO FL-[ ', end='')
            if leg_index == 2:
                print('GPIO BR-[ ', end='')
            if leg_index == 3:
                print('GPIO BL-[ ', end='')
            for axis_index in range(3):
                self.pigpio.set_PWM_frequency(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_freq)
                self.pigpio.set_PWM_range(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_usec_range)
                print('{:02d}'.format(self.pwm_params.pins[axis_index, leg_index]), 'pin ', sep='', end='')
            print(']')

        # Initialize arm PWM channels
        print('GPIO ARM-[ ', end='')
        for pin in self.pwm_params.arm_pins:
            self.pigpio.set_PWM_frequency(pin, self.servo_params.pwm_freq)
            self.pigpio.set_PWM_range(pin, self.servo_params.pwm_usec_range)
            print(f"{pin:02d}pin ", end='')
        print(']')
        
        return

    def deactivate_servos(self):
        for leg_index in range(4):
            for axis_index in range(3):
                self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], 0)
        return

    def angle_to_pwmdutycycle(self, angle, axis_index, leg_index):
        neutral = self.servo_params.neutral_angles[axis_index, leg_index]
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = (angle - neutral) * multi
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)

    def angle_to_pwmdutycycle_noncab(self, angle, axis_index, leg_index):
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = angle * multi
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)

    def send_servo_command(self, joint_angle, axis, leg):
        duty_cycle = self.angle_to_pwmdutycycle(joint_angle, axis, leg)
        self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis, leg], duty_cycle)
        return

    def send_servo_command_noncab(self, joint_angle, axis, leg):
        duty_cycle = self.angle_to_pwmdutycycle_noncab(joint_angle, axis, leg)
        self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis, leg], duty_cycle)
        return

    def send_servo_commands(self, joint_angles):
        for leg_index in range(4):
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                duty_cycle = self.angle_to_pwmdutycycle(angle, axis_index, leg_index)
                self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], duty_cycle)
        return

    def set_led_green(self, onoff):
        GPIO.output(LED_GREEN_GPIO, onoff)
        return

    def set_led_blue(self, onoff):
        GPIO.output(LED_BLUE_GPIO, onoff)
        return

    # 非平行リンク機構の導入 ------------------------------------------------------------------------
    # for PrintPupper v0.2's unparallel link mechanism
    def extrapolation_unparallel_actuator_positions(self, joint_angles):
        debug = False
        for leg_index in range(4):
            rad0 = joint_angles[0, leg_index] * self.servo_params.servo_multipliers[0, leg_index]
            rad1 = joint_angles[1, leg_index] * self.servo_params.servo_multipliers[1, leg_index]
            rad2 = joint_angles[2, leg_index] * self.servo_params.servo_multipliers[2, leg_index]
            coxa  = rad0
            knee  = rad2

            # compute_unparallel_link_knee の表現型に変換してから非平行リンクを介した場合の knee サーボ角度を再計算
            if leg_index == 0 or leg_index == 2:
                leg = (math.pi / 2) - rad1
                leg = -leg
                mirror = True
            else:
                leg = (math.pi / 2) + rad1
                mirror = False
            kneeX = self.culk.compute(leg, knee, mirror)

            # 安全機構 非平行リンク機構に起因する計算不能（逆運動学上で届かない座標点）発生時に、
            # サーボを誤動作させないために前フレームの正常値で角度指示する
            if math.isnan(kneeX):
                debug = True
                kneeX = self.compute_normal_kneeX[leg_index]
                print("compute Err!")
            else:
                self.compute_normal_kneeX[leg_index] = kneeX

            if debug:
                print(f"LEG{leg_index} : coxa {coxa:+07.2f}({math.degrees(coxa):+07.2f}), ", end="")
                print(f"leg {leg:+07.2f}({math.degrees(leg):+07.2f}), ", end="")
                print(f"knee {knee:+07.2f}({math.degrees(knee):+07.2f}), ", end="")
                print(f"kneeX {kneeX:+07.2f}({math.degrees(kneeX):+07.2f})")
                if leg_index == 3:
                        print("")

            # 再計算結果 knee 角度を指示行列に戻す
            joint_angles[2, leg_index] = kneeX * self.servo_params.servo_multipliers[2, leg_index]
        return
    
    # arm_angles: アームの各関節角度リスト 土台, 肩, 肘, 手首(Pitch), 手首(Roll), 指
    # アームのサーボに反映する
    def set_arm_joint_angles(self, arm_angles):
        # サーボの数は関節の数に加えて肩(右)の分だけ1つ多い
        # npの配列をコピーする
        servo_angles = arm_angles.copy()
        servo_angles = np.append(servo_angles, -arm_angles[1])

        # print('arm servo angles (deg):', np.round(np.degrees(servo_angles), 3))

        for i, servo_angle in enumerate(servo_angles):
            self.pigpio.set_PWM_dutycycle(
                self.pwm_params.arm_pins[i],
                self.angle_to_arm_pwmdutycycle(servo_angle)
            )
        return
    
    def angle_to_arm_pwmdutycycle(self, angle):
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)
