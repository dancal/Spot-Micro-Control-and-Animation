
from math import pi, sin, cos, atan, atan2, sqrt

class Utils:

    """ Angle measurement Complementary filter """
    def comp_filter(angle, t, T):
        # Complementary filter calculates body angles around xt and y axis from IMU data
        ## acc = mpu.acceleration
        ## gyr = mpu.gyro
        acc = [0, 0, 0]
        gyr = [0, 0]

        denb = sqrt(acc[1]**2+acc[2]**2)
        dena = sqrt(acc[2]**2+acc[0]**2)

        if (dena == 0):
            alpha = 0
        else:
            alpha = atan(acc[1]/dena)

        if (denb == 0):
            beta = 0
        else:
            beta = atan(acc[0]/denb)

        A = T/(T+t)

        anglex = A*(angle[0]+t*gyr[0]/180*pi)+(1-A)*alpha
        angley = A*(angle[1]+t*gyr[1]/180*pi)+(1-A)*beta

        return [anglex, angley]
