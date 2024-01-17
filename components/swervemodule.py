import math

import wpilib
import wpimath
from ctre.sensors import CANCoder
from rev import CANSparkMax

#from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

# Create the structure of the config: SmartDashboard prefix, Encoder's zero point, Drive motor inverted, Allow reverse
ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

MAX_VOLTAGE = 5 # Absolute encoder measures from 0V to 5V

class SwerveModule:
    # Get the motors, encoder and config from injection
    driveMotor: CANSparkMax
    rotateMotor: CANSparkMax
        
    encoder: CANCoder

    cfg: ModuleConfig

    def setup(self):
        """
        Called after injection
        """
        # Config
        self.sd_prefix = self.cfg.sd_prefix or 'Module'
        self.encoder_zero = self.cfg.zero or 0
        self.inverted = self.cfg.inverted or False
        self.allow_reverse = self.cfg.allow_reverse or True

        # SmartDashboard
        #self.sd = NetworkTables.getTable('SmartDashboard')
        #self.debugging = self.sd.getEntry('drive/drive/debugging')

        # Motor
        self.driveMotor.setInverted(self.inverted)

        self._requested_angle = 0
        self._requested_speed = 0

        # PID Controller
        #kP = 1.5, kI = 0.0, kD = 0.0
        # enable pid continues input (0.0, 5.0)
        # set tolerance (0.05, 0.05)
        self._pid_controller = PIDController(1.5, 0.0, 0.0, 0.05)
        self._pid_controller.enableContinuousInput(0.0, 5.0) # Will set the 0 and 5 as the same point
        self._pid_controller.setTolerance(1, 1) # Tolerance where the PID will be accpeted aligned

    def get_degree(self):
        """
        :returns: the degree from the encoder, return degree
        """
        return self.encoder.getAbsolutePosition() - self.encoder_zero
        
        ##return self.encoder.getBusVoltage() - self.encoder_zero
        

    def flush(self):
        """
        Flush the modules requested speed and degree.
        Resets the PID controller.
        """
        self._requested_angle = 0 # Check
        self._requested_speed = 0
        #self._pid_controller.reset()

    @staticmethod
    def degree_to_rad(degree):
        """
        Convert a given voltage value to rad.

        :param voltage: a voltage value between 0 and 5
        :returns: the radian value betwen 0 and 2pi
        """
        return degree * math.pi / 180
    
    @staticmethod
    def degree_to_voltage(degree):
        """
        Convert a given degree to voltage.

        :param degree: a degree value between 0 and 360
        :returns" the voltage value between 0 and 5
        """
       
        return degree / 360 * 5

    def move(self, speed, deg):
        """
        Set the requested speed and rotation of passed.

        :param speed: requested speed of wheel from -1 to 1
        :param deg: requested angle of wheel from 0 to 359 (Will wrap if over or under)
        """
        #deg %= 360 # Prevent values past 360

        ## NEED to review
         
        if self.allow_reverse:
            """
            If the difference between the requested degree and the current degree is
            more than 90 degrees, don't turn the wheel 180 degrees. Instead reverse the speed.
            """
            if abs(deg - self.get_degree() > 90):
                speed *= -1
                deg += 180
                deg %= 360
        

        self._requested_speed = speed
        self._requested_angle = deg

    def debug(self):
        """
        Print debugging information about the module to the log.
        """
        print(self.sd_prefix, '; requested_speed: ', self._requested_speed, ' requested_voltage: ', self._requested_voltage)

    
    def execute(self):
        """
        Use the PID controller to get closer to the requested position.
        Set the speed requested of the drive motor.

        Called every robot iteration/loop.
        """
        # Calculate the error using the current voltage and the requested voltage.
        # DO NOT use the #self.get_voltage function here. It has to be the raw voltage.
        Angle1_0 = self.get_degree()
        Angle1 = self.degree_to_voltage(Angle1_0)
        Angle2 = self.degree_to_voltage(self._requested_angle)
        error = self._pid_controller.calculate(Angle1, Angle2)
        ## Does the first value need to be set to subtract the zero position

        # Set the output 0 as the default value
        output = 0
        # If the error is not tolerable, set the output to the error.
        # Else, the output will stay at zero.
        if not self._pid_controller.atSetpoint():
            # Use max-min to clamped the output between -1 and 1.
            output = max(min(error, 1), -1)

        #rotate_speed = (output / 360) * 5
        # Put the output to the dashboard
        #print(output)
        #self.sd.putNumber('drive/%s/output' % self.sd_prefix, output)
        #print("position = ", self.encoder.getAbsolutePosition())
        #print("requestedAngle = ", self._requested_angle)
        #print(output)

        # Set the output as the rotateMotor's voltage / also move the rotational motors
        ## Convert output to voltage value
        self.rotateMotor.set(output)

        # Set the requested speed as the driveMotor's voltage
        self.driveMotor.set(self._requested_speed)

        #self.update_smartdash()
'''
    def update_smartdash(self):
        """
        Output a bunch on internal variables for debugging purposes.
        """
        self.sd.putNumber('drive/%s/degrees' % self.sd_prefix, self.voltage_to_degrees(self.get_voltage()))

        if self.debugging.getBoolean(False):
            self.sd.putNumber('drive/%s/requested_voltage' % self.sd_prefix, self._requested_voltage)
            self.sd.putNumber('drive/%s/requested_speed' % self.sd_prefix, self._requested_speed)
            self.sd.putNumber('drive/%s/raw voltage' % self.sd_prefix, self.encoder.getVoltage())  # DO NOT USE self.get_voltage() here
            self.sd.putNumber('drive/%s/average voltage' % self.sd_prefix, self.encoder.getAverageVoltage())
            self.sd.putNumber('drive/%s/encoder_zero' % self.sd_prefix, self.encoder_zero)

            self.sd.putNumber('drive/%s/PID Setpoint' % self.sd_prefix, self._pid_controller.getSetpoint())
            self.sd.putNumber('drive/%s/PID Error' % self.sd_prefix, self._pid_controller.getPositionError())
            self.sd.putBoolean('drive/%s/PID isAligned' % self.sd_prefix, self._pid_controller.atSetpoint())

            self.sd.putBoolean('drive/%s/allow_reverse' % self.sd_prefix, self.allow_reverse)
'''
