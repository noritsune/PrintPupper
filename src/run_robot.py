import sys
import time
import argparse
from Controller import Controller
from JoystickInterface import JoystickInterface
from State import State
from HardwareInterface import HardwareInterface
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics
from State import BehaviorState, State
from run_robot_caliblate_mode import run_robot_caliblate_mode
from mpu6050_kalman import Mpu6050Kalman

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        mpu_kalman = Mpu6050Kalman(0x68)

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Wait until the activate button has been pressed
    joystick_interface.set_color(config.ps4_deactivated_color)
    while True:
        wait_loop = 0
        led_blink = 0

        print("Waiting for L1 to activate robot.")
        print(hardware_interface.servo_params.neutral_angle_degrees)
        wait_loop_first = False
        while True:
            if (wait_loop % (25 if wait_loop_first else 100)) == 0:
                hardware_interface.set_led_green(bool(led_blink % 2))
                led_blink += 1
            wait_loop +=1
            command = joystick_interface.get_command(state)
            # PS4 gamepad from bluetooth mode as slow blink / USB gamepad as first blink
            if command.joy_ps4_usb:
                wait_loop_first = False
            else:
                wait_loop_first = True
            if command.activate_event == 1:
                break
            if command.caliblate_mode_event:
                command.caliblate_mode_event = False
                run_robot_caliblate_mode(config, hardware_interface, joystick_interface)
                sys.exit()
            time.sleep(0.01)

        print("Robot activated.")
        hardware_interface.set_led_green(True)
        joystick_interface.set_color(config.ps4_activated_color)
        while True:
            d_time = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                # hardware_interface.deactivate()
                joystick_interface.set_color(config.ps4_deactivated_color)
                state.behavior_state = BehaviorState.REST
                hardware_interface.set_led_blue(False)
                break

            if command.trot_event:
                if state.behavior_state == BehaviorState.REST:
                    joystick_interface.set_color(config.ps4_torot_color)
                    hardware_interface.set_led_blue(True)
                    #print("Robot start torot")
                else:
                    joystick_interface.set_color(config.ps4_activated_color)
                    hardware_interface.set_led_blue(False)
                    #print("Robot stop torot")

            # Read imu data. Orientation will be None if no data was available
            if use_imu:
                # mpu6050から傾き取得
                pitch, roll = mpu_kalman.get_pitch_roll()
                print("PITCH(deg): %7.2f ROLL(deg): %7.2f" % (pitch, roll))
                state.imu_pitch = pitch
                state.imu_roll = roll

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)

            # cycle tune
            t_time = time.time()
            dt = (float)(t_time - d_time)
            dt_dt = (float)(config.dt - dt)
            dt_dt = round(dt_dt, 3)
            if (dt_dt > 0) and (dt_dt >= config.dt_min_sleep):
                time.sleep(dt_dt)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_imu', action='store_true', help='Use IMU (mpu6050)')
    args = parser.parse_args()
    main(use_imu=args.use_imu)
