from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
FULL_BRAKE_SPD = 0.1


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.8 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel


        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        linear_accel = self.throttle_controller.step(vel_error, sample_time)

         # Full brake if target velocity is almost 0
        if linear_vel < FULL_BRAKE_SPD:
            throttle = 0.0
            brake = self.acceleration_to_torque(abs(self.decel_limit))
        else:
            if linear_accel > 0.0: # Tipin - Vehicle moving/accelerating
                throttle = linear_accel
                brake = 0.0
            else:
                throttle = 0.0 # Tipout - vehicle slowing down
                decel = -linear_accel

                # For small decel, do not brake
                if decel < self.brake_deadband:
                    decel = 0.0

                # Compute brake torque, in Nm
                brake = self.acceleration_to_tq(decel)

        return throttle, brake, steering

    def acceleration_to_tq(self, acceleration):
        """
        Convert acceleration to torque
        Input: acceleration (float) , in m/s^2
        Output: torque (float) , in Nm
        """
        return acceleration * self.total_mass * self.wheel_radius



