class ServoController:
    Spot            = None

    """Servo trimming """
    xtlf            = 14
    ytlf            = 0
    ztlf            = 0

    xtrf            = 14
    ytrf            = 0
    ztrf            = 3

    xtrr            = 14
    ytrr            = 0
    ztrr            = 0

    xtlr            = 14
    ytlr            = 0
    ztlr            = 0

    orgthetalf      = [0,0,0]
    orgthetarf      = [0,0,0]
    orgthetarr      = [0,0,0]
    orgthetalr      = [0,0,0]

    def __init__(self, _Spot):
        self.Spot   = _Spot

    def servo_moving(self, pos, move):

        thetalf_reply   = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, pos[0]+self.xtlf, pos[1]+self.ytlf, pos[2]+self.ztlf, 1)
        thetarf_reply   = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, pos[3]+self.xtrf, pos[4]+self.ytrf, pos[5]+self.ztrf, -1)
        thetarr_reply   = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, pos[6]+self.xtrr, pos[7]+self.ytrr, pos[8]+self.ztrr, -1)
        thetalr_reply   = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, pos[9]+self.xtlr, pos[10]+self.ytlr, pos[11]+self.ztlr, 1)

        thetalf         = thetalf_reply[0]
        thetarf         = thetarf_reply[0]
        thetarr         = thetarr_reply[0]
        thetalr         = thetalr_reply[0]

        if move == True:
            if (thetalf_reply[1] == False):
                try:
                    ShoulderAngle   = int(thetalf[0]/pi*180 * self.Spot.angle_scale_factor_lf1*self.Spot.dir01+self.Spot.zero01)
                    LegAngle        = int(thetalf[1]/pi*180 * self.Spot.angle_scale_factor_lf2*self.Spot.dir02+self.Spot.zero02)
                    FeetAngle       = int(thetalf[2]/pi*180 * self.Spot.angle_scale_factor_lf3*self.Spot.dir03+self.Spot.zero03)
                    if (self.orgthetalf[0] == ShoulderAngle and self.orgthetalf[1] == LegAngle and self.orgthetalf[2] == FeetAngle):
                        return
                    #else:
                    #    print('Front_LEFT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                    self.orgthetalf      = [ShoulderAngle, LegAngle, FeetAngle]
                except ValueError:
                    print('Angle out of Range')

            if (thetarf_reply[1] == False):
                try:
                    ShoulderAngle   = int(thetarf[0]/pi*180 * self.Spot.angle_scale_factor_rf1*self.Spot.dir04+self.Spot.zero04)
                    LegAngle        = int(thetarf[1]/pi*180 * self.Spot.angle_scale_factor_rf2*self.Spot.dir05+self.Spot.zero05)
                    FeetAngle       = int(thetarf[2]/pi*180 * self.Spot.angle_scale_factor_rf3*self.Spot.dir06+self.Spot.zero06)
                    if (self.orgthetarf[0] == ShoulderAngle and self.orgthetarf[1] == LegAngle and self.orgthetarf[2] == FeetAngle):
                        return
                    #else:
                    #    print('Front_RIGHT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                    self.orgthetarf      = [ShoulderAngle, LegAngle, FeetAngle]
                except ValueError:
                    print('Angle out of Range')

            if (thetarr_reply[1] == False):
                try:
                    ShoulderAngle   = int(thetarr[0]/pi*180 * self.Spot.angle_scale_factor_rr1*self.Spot.dir07+self.Spot.zero07)
                    LegAngle        = int(thetarr[1]/pi*180 * self.Spot.angle_scale_factor_rr2*self.Spot.dir07+self.Spot.zero08)
                    FeetAngle       = int(thetarr[2]/pi*180 * self.Spot.angle_scale_factor_rr3*self.Spot.dir09+self.Spot.zero09)
                    
                    if (self.orgthetarr[0] == ShoulderAngle and self.orgthetarr[1] == LegAngle and self.orgthetarr[2] == FeetAngle):
                        return
                    #else:
                    #    print('REAR_RIGHT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                    self.orgthetarr      = [ShoulderAngle, LegAngle, FeetAngle]
                except ValueError:
                    print('Angle out of Range')

            if (thetalr_reply[1] == False):
                try:
                    ShoulderAngle   = int(thetalr[0]/pi*180 * self.Spot.angle_scale_factor_lr1*self.Spot.dir10+self.Spot.zero10)
                    LegAngle        = int(thetalr[1]/pi*180 * self.Spot.angle_scale_factor_lr2*self.Spot.dir11+self.Spot.zero11)
                    FeetAngle       = int(thetalr[2]/pi*180 * self.Spot.angle_scale_factor_lr3*self.Spot.dir12+self.Spot.zero12)
                    
                    if (self.orgthetalr[0] == ShoulderAngle and self.orgthetalr[1] == LegAngle and self.orgthetalr[2] == FeetAngle):
                        return
                    #else:
                    #    print('REAR_LEFT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                    self.orgthetalr      = [ShoulderAngle, LegAngle, FeetAngle]
                except ValueError:
                    print('Angle out of Range')

            print(self.orgthetalf, self.orgthetarf, self.orgthetarr, self.orgthetalr)

