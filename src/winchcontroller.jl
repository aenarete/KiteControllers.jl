# Winch winch controller component, implemented as described in the PhD thesis of Uwe Fechner. 

@enum WinchControllerState wcsLowerForceLimit wcsSpeedControl wcsUpperForceLimit
    
        
# Basic winch controller. Works in one of the three modes wcsLowerForceLimit, wcsSpeedControl and
# wcsUpperForceLimit.
@with_kw mutable struct WinchController @deftype Float64
    wcs::WCSettings
    time = 0
    last_force = 0
    v_set_pc::Union{Float64, Nothing} = nothing # last set value from the position controller (can be nothing)
    v_set_in = 0.0  # input of the speed controller
    v_set_out = 0.0 # output of the speed controller
    v_set_ufc = 0.0 # output of the upper force controller
    v_set_lfc = 0.0 # output of the lower force controller
    v_set = 0.0     # output of the winchcontroller, going to the motor-controller/ model
    v_act = 0.0     # actual, measured speed
    force = 0.0     # actual, measured force
    calc::CalcVSetIn = CalcVSetIn(wcs)
    mix3::Mixer_3CH  = Mixer_3CH(wcs.dt, wcs.t_blend)
    pid1::SpeedController = SpeedController(wcs)
    pid2::LowerForceController = LowerForceController(wcs)
    pid3::UpperForceController = UpperForceController(wcs)
end

function WinchController(wcs::WCSettings)
    wc = WinchController(wcs=wcs)
    set_f_set(wc.pid2, wcs.f_low)
    set_reset(wc.pid2, true)
    set_v_sw(wc.pid2, -1.0)
    set_f_set(wc.pid3, wcs.f_high)
    set_v_sw(wc.pid3, calc_vro(wcs, wc.pid3.f_set))
    set_reset(wc.pid3, true)
    set_reset(wc.pid3, false)
    wc
end

function calc_v_set(wc::WinchController, v_act, force, f_low, v_set_pc=nothing)
    set_f_set(wc.pid2, f_low)
    wc.v_act = v_act
    wc.force = force
end

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
