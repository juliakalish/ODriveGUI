import json
import time
import odrive
from odrive.enums import *
#from art import *
from tkinter import *
import configurationWalkthrough

# Set the user-input axis variable as a global variable

class configurationWalkthrough:
    def __init__(self, axis_num):
        self.curr_volt = 1

        self.axis_num = axis_num

        print("Looking for ODrive...")
        self.find_odrive()

    def find_odrive(self):
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        if (self.odrv):
            print("ODrive Found.")

    def configure(self):
        """
        Configures the odrive device for maxon motor.
        """
        print("Let's start with configurating the actuator settings.")
        while(True):
            actuator = input("\nPlease indicate which actuator you are using (FAT, TINY, MEGA): ")
            if (actuator == "FAT"):
                # Motor and encoder configurations
                self.odrv_axis.motor.config.current_lim = 2
                self.odrv_axis.motor.config.calibration_current = 2
                self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
                self.odrv_axis.motor.config.pole_pairs = 1
                self.odrv_axis.motor.config.torque_constant = 0.05
                self.odrv_axis.encoder.config.cpr = 2000

                # Control gains
                self.odrv_axis.controller.config.pos_gain = 16
                self.odrv_axis.controller.config.vel_gain = 0.045
                self.odrv_axis.controller.config.vel_integrator_gain = 0
                self.odrv_axis.controller.config.vel_limit = 115
                self.odrv_axis.controller.config.frc_gain = 0.0
                self.odrv_axis.controller.config.frc_integrator_gain = 0.0
                self.odrv_axis.controller.config.frc_derivative_gain = 0.0

            elif (actuator == "TINY"):
                # Motor and encoder configurations
                self.odrv_axis.motor.config.current_lim = 2
                self.odrv_axis.motor.config.calibration_current = 2
                self.odrv_axis.motor.config.resistance_calib_max_voltage = 2
                self.odrv_axis.motor.config.pole_pairs = 2
                self.odrv_axis.motor.config.torque_constant = 0.0211
                self.odrv_axis.encoder.config.cpr = 2000

                # Control gains
                self.odrv_axis.controller.config.pos_gain = 1.2
                self.odrv_axis.controller.config.vel_gain = 0.005
                self.odrv_axis.controller.config.vel_integrator_gain = 0
                self.odrv_axis.controller.config.vel_limit = 150
                self.odrv_axis.controller.config.frc_gain = 0.0
                self.odrv_axis.controller.config.frc_integrator_gain = 0.0
                self.odrv_axis.controller.config.frc_derivative_gain = 0.0

            elif (actuator == "MEGA"):
                # Motor and encoder configurations
                self.odrv_axis.motor.config.current_lim = 2
                self.odrv_axis.motor.config.calibration_current = 2
                self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
                self.odrv_axis.motor.config.pole_pairs = 1
                self.odrv_axis.motor.config.torque_constant = 0.427
                self.odrv_axis.encoder.config.cpr = 2000

                # Control gains
                self.odrv_axis.controller.config.pos_gain = 1.5
                self.odrv_axis.controller.config.vel_gain = 1.985
                self.odrv_axis.controller.config.vel_integrator_gain = 0
                self.odrv_axis.controller.config.vel_limit = 135
                self.odrv_axis.controller.config.frc_gain = 0.0
                self.odrv_axis.controller.config.frc_integrator_gain = 0.0
                self.odrv_axis.controller.config.frc_derivative_gain = 0.0

            if actuator in ("FAT", "TINY", "MEGA"):
                # System Settings
                self.odrv_axis.encoder.config.frc_calib_diff = -50875
                self.odrv_axis.encoder.config.frc_calib_factor = -179.87
                if self.axis_num == 1:
                    self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = 1
                else:
                    self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = 2
                self.odrv_axis.encoder.config.frc_calib_factor = -179.87
                self.odrv.config.enable_brake_resistor = True
                self.odrv.config.brake_resistance = 0.05
                self.odrv.config.dc_max_negative_current = -0.5
                self.odrv_axis.motor.config.current_control_bandwidth = 100
                self.odrv_axis.controller.config.enable_full_control = True
                self.odrv_axis.controller.config.full_control_setting = 3
                self.odrv.config.enable_uart_a = False
                self.odrv.config.gpio1_mode = GPIO_MODE_DIGITAL
                self.odrv.config.gpio2_mode = GPIO_MODE_DIGITAL
                self.odrv.config.enable_uart_b = True
                self.odrv.config.gpio3_mode = GPIO_MODE_UART_B
                self.odrv.config.gpio4_mode = GPIO_MODE_UART_B

                # Zero out commands
                self.odrv_axis.controller.input_vel = 0.0
                self.odrv_axis.controller.input_pos = 0.0
                self.odrv_axis.controller.input_frc = 0.0
                print("\nODrive configuration set.")
                break
            else:
                print("\nInvalid response. Try again.")
                continue

    def get_metrics(self):
        data = {}
        data["pos"] = self.odrv_axis.encoder.pos_estimate
        data["vel"] = self.odrv_axis.encoder.vel_estimate
        data["frc"] = self.odrv_axis.encoder.spi_val
        return data

    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        print("ODrive set to idle.")
    
    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """
        print("\nClosed loop control initiated.")
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def check_errors(self, printed):
        if (self.odrv_axis.error + self.odrv_axis.encoder.error + self.odrv_axis.motor.error != 0):
            if (printed):
                print("An error was found.")
                print("\nAxis Error: " + str(self.odrv_axis.error))
                print("Encoder Error: " + str(self.odrv_axis.encoder.error))
                print("Motor Error: " + str(self.odrv_axis.motor.error))
            return False
        else:
            if (printed):
                print("No errors found.")
            return True

    def mode_full_calibration_sequence(self):
        """
        Puts the motor in full calibration sequence.
        """
        print("\nCalibration sequence initiated.")
        self.odrv_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(10)
        print("Calibration sequence complete.")
        self.check_errors(printed=True)
        
    def set_control_setting(self, setting):
        self.odrv_axis.controller.config.full_control_setting = setting

    def clear_errs(self):
        self.odrv.clear_errors()
        print("Errors cleared.")


class Application(Frame):

    def createWidgets(self):

        boldFont = Font(family='Helvetica', size=10, weight='bold')

        self.button_axis0 = Button(self, text = "Axis 0", command=self.axis0Select)
        self.button_axis0.grid(row=0,column=0)
        self.button_axis1 = Button(self, text = "Axis 1", command=self.axis1Select)
        self.button_axis1.grid(row=1,column=0)
        #self.button_begin = Button(self, text = "Begin Configuration", command=)
        self.button_begin.grid(row=2,column=0)

        # Axis State Frame
        status_frame = LabelFrame(self, relief="groove", borderwidth=2, text = "Axis State")
        Label(status_frame, text="Axis 0", font=boldFont).grid(row=0, column=1, padx=4)
        Label(status_frame, text="Axis 1", font=boldFont).grid(row=0, column=2, padx=4)

        def axis0Select(self):
            if __name__ == "__main__":
                ax = 0
                motor_config = configurationWalkthrough(axis_num = ax)
                time.sleep(1)

                motor_config.configure()
        
        def axis1Select(self):
            if __name__ == "__main__":
                ax = 1
                motor_config = configurationWalkthrough(axis_num = ax)
                time.sleep(1)

                motor_config.configure()
"""
# Create an event handler to start the odrive configuration
def handle_click_config(event):
    if __name__ == "__main__":
        #ax = input("Which Axis? (0, 1):")
        motor_config = configurationWalkthrough(axis_num = ax)
        time.sleep(1)

        motor_config.configure()
        while(True):
            motor_config.mode_idle()
            motor_config.clear_errs()
            while(True):
                calib = input("\nWould you like to run the full calibration sequence? (YES or NO): ")
                if (calib == "YES"):
                    motor_config.mode_full_calibration_sequence()
                    break
                elif (calib == "NO"):
                    print("\nSomeone's feeling confident. Let's move on then.")
                    time.sleep(1)
                    break
                else:
                    print("\nInvalid response. Try again.")
                    continue
            quit()
"""
root = Tk()

app = Application(master=root)

app.mainloop()
