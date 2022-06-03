# """ 
# Winch winch controller base class, implemented as described in the PhD thesis of Uwe Fechner. 
# """
# # pylint: disable=E1101
# # TODO: implement the calculation of v_set for reeling out to a curtain lenght
# # TODO: send v_set messages
# from Components import CalcVSetIn, SpeedController, LowerForceController, UpperForceController, Mixer_3CH, \
#                        calcV_ro, setVRiMax
# from flufl.enum import Enum, IntEnum
# import numpy as np
# from Settings import pro as PRO

# T_BLEND =  0.25 # blending time of the mixers in seconds
# STARTUP_TIME = 0.5 # during this time the force controller is disabled

# def form(number):
#     """ Convert a number to a string with two digits after the decimal point. """
#     return "{:.2f}".format(number)

# class SystemState(Enum):
#     """ Class for encoding and decoding the field system_state.
#         See: http://www.kitepower.eu/wiki/index.php/CentralControl. """
#     ssManualOperation          = 0
#     ssParking                  = 1
#     ssPower                    = 2
#     ssKiteReelOut              = 3
#     ssWaitUntilPointedAtZenith = 4
#     ssDepower                  = 5
#     ssIntermediate             = 6 # after ssPower, before ssKiteReelOut
#     ssLaunching                = 7
#     ssEmergencyLanding         = 8
#     ssLanding                  = 9
#     ssReelIn                   = 10
#     ssTouchdown                = 11

# class WinchControlState(IntEnum):
#     wcsLowerForceLimit = 0 
#     wcsSpeedControl    = 1 
#     wcsUpperForceLimit = 2 
    
        
# class WinchController(object):
#     """ 
#     Basic winch controller. Works in one of the three modes wcsLowerForceLimit, wcsSpeedControl and
#     wcsUpperForceLimit.
#     """
#     def __init__(self, pro):
#         self.k_v = 0.02      # multiplicator for calculating the set speed from the sqare root of the force
#         self.v_set_pc = None # last set value from the position controller (can be None)
#         self.v_set_in = 0.0  # input of the speed controller
#         self.v_set_out = 0.0 # output of the speed controller
#         self.v_set_ufc = 0.0 # output of the upper force controller
#         self.v_set_lfc = 0.0 # output of the lower force controller
#         self.v_set = 0.0     # output of the winchcontroller, going to the motor-controller/ model
#         self.v_act = 0.0     # actual, measured speed
#         self.force = 0.0     # actual, measured force
#         setVRiMax(pro._winch_control.operational['v_ri_max'])
#         self.f_high = pro._winch.f_high
#         # print "--->>> f_high: ", self.f_high
#         self.f_low = pro._winch.f_low
#         self.calc = CalcVSetIn(self.f_high, self.f_low)
#         self.pid1 = SpeedController()
#         self.pid1.setTracking(0.0)
#         self.pid1.setInactive(True)
#         self.pid1.setVSetIn(0.0)
#         # create and initialize lower force controller
#         self.pid2 = LowerForceController(pro._winch_control.f_low['P'], pro._winch_control.f_low['I'], \
#              pro._winch_control.f_low['K_b'], pro._winch_control.f_low['K_t'])
#         self.pid2.setFSet(pro._winch.f_min)
#         self.pid2.setTracking(0.0)
#         self.pid2.setReset(True)
#         self.pid2.setV_SW(-1.0) # TODO: Is this a good choice? It should not be zero, though.
#         # create and initialize upper force controller
#         self.pid3 = UpperForceController(pro._winch_control.f_high['P'], pro._winch_control.f_high['I'], \
#         pro._winch_control.f_high['D'], pro._winch_control.f_high['N'], pro._winch_control.f_high['K_b'], \
#         pro._winch_control.f_high['K_t'])
#         f_upper = pro._winch_control.operational["f_ro_max"]
#         # print "------->>>> f_upper: ", form(f_upper)
#         self.pid3.setFSet(f_upper)
#         self.pid3.setTracking(0.0)   
#         self.pid3.setV_SW(2 * f_upper) # TODO: Is this a good choice? It should not be zero, though.  
#         self.pid3.setReset(True)
#         self.pid3.setReset(False)        
#         # create the mixer for the output of the two controllers
#         self.mix3 = Mixer_3CH()
#         self.last_force = 0.0  
#         self.time = 0.0
     
        
#     def calcVSet(self, v_set_pc, v_act, force, f_low):
#         self.pid2.setFSet(f_low)
#         self.v_set_pc = v_set_pc
#         self.v_act = v_act
#         self.force = force
#         # calc v_set_in
#         self.calc.setVSetPc_Force(v_set_pc, force)
#         v_set_in = self.calc.getVSetIn()
#         # set the inputs of pid1
#         self.pid1.setVSetIn(v_set_in)       
#         self.pid1.setVAct(v_act)
#         if self.time <= STARTUP_TIME:
#             reset = True
#         else:
#             reset = False
#         # set the inputs of pid2 and pid3    
#         self.pid2.setReset(reset)
#         self.pid2.setV_SW(calcV_ro(self.pid2._f_set, self.f_high, self.f_low) * 1.05)  
#         self.pid3.setV_SW(calcV_ro(self.pid3._f_set, self.f_high, self.f_low) * 0.95)
#         self.pid2.setVAct(v_act)
#         self.pid3.setVAct(v_act)
#         # set tracking and force for all controllers
#         self.pid1.setTracking(self.mix3.getDirectOutput())
#         self.pid2.setTracking(self.mix3.getDirectOutput())
#         self.pid3.setTracking(self.mix3.getDirectOutput())            
#         self.pid2.setForce(force)
#         self.pid3.setForce(force)        
#         # activate or deactivate the speed controller
#         self.pid1.setInactive((self.pid2._active or self.pid3._active))           
#         # calculate the output, using the mixer
#         self.mix3.setInputA(self.pid1.getVSetOut()    ) # from speed controller
#         self.mix3.setInputB(self.pid2.getVSetOut())     # from lower force controller
#         self.mix3.setInputC(self.pid3.getVSetOut())     # from upper force controller
#         self.mix3.selectB(self.pid2._active)  # 
#         self.mix3.selectC(self.pid3._active)  # 
#         self.pid1.setInactive((self.pid2._active or self.pid3._active))        
#         self.v_set_out = self.mix3.getOutput()        
        
#         self.last_force = force
#         return self.v_set_out
        
#     def onTimer(self, period_time):
#         self.time += period_time
#         self.pid1.onTimer()   
#         self.pid2.onTimer()
#         self.pid3.onTimer()
#         self.mix3.onTimer()
#         self.calc.onTimer()    

#     def getWinchControlState(self):
#         return self.mix3.getControllerState()
        
#     def getSetForce(self):
#         state = self.mix3.getControllerState()        
#         if state == 0:
#             return self.pid2._f_set
#         elif state == 2:
#             return self.pid3._f_set
#         return None            
    
#     def getStatus(self):
#         result = np.zeros(8)
#         result[0] = self.f_high
#         result[1] = self.f_low
#         result[2] = self.k_v
#         if self.v_set_pc is None:
#             result[3] = np.nan
#         else:
#             result[3] = self.v_set_pc
#         result[4] = self.v_set_in
#         result[5] = self.v_set_out
#         result[6] = self.v_set_ufc
#         result[7] = self.v_set_lfc
#         return result


# if __name__ == "__main__":
#     wc = WinchController(PRO)

