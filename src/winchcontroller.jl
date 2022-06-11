# Winch winch controller component, implemented as described in the PhD thesis of Uwe Fechner. 

@enum WinchControllerState wcsLowerForceLimit wcsSpeedControl wcsUpperForceLimit
    
        
# Basic winch controller. Works in one of the three modes wcsLowerForceLimit, wcsSpeedControl and
# wcsUpperForceLimit.
@with_kw mutable struct WinchController @deftype Float64
    wcs::WCSettings
    time = 0
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
    set_vset_pc(wc.calc, v_set_pc, wc.force)
    v_set_in = calc_output(wc.calc)
    # set the inputs of pid1
    set_v_set_in(wc.pid1, v_set_in)
    set_v_act(wc.pid1, v_act)
    if wc.time <= wc.wcs.t_startup
        reset = true
    else
        reset = false
    end
    # set the inputs of pid2 and pid3    
    set_reset(wc.pid2, reset)
    set_v_sw(wc.pid2, calc_vro(wc.wcs, wc.pid2.f_set) * 1.05)
    set_v_sw(wc.pid3, calc_vro(wc.wcs, wc.pid3.f_set) * 0.95)
    set_v_act(wc.pid2, v_act)
    set_v_act(wc.pid3, v_act)
    # set tracking and force for all controllers
    set_tracking(wc.pid1, wc.v_set)
    set_tracking(wc.pid2, wc.v_set)
    set_tracking(wc.pid3, wc.v_set)
    set_force(wc.pid2, force)
    set_force(wc.pid3, force)
    # activate or deactivate the speed controller
    set_inactive(wc.pid1, wc.pid2.active || wc.pid3.active)    
    # calculate the output, using the mixer
    select_b(wc.mix3, wc.pid2.active)
    select_c(wc.mix3, wc.pid3.active)
    v_set_out_A = get_v_set_out(wc.pid1)
    v_set_out_B = get_v_set_out(wc.pid2)
    v_set_out_C = get_v_set_out(wc.pid3)
    wc.v_set = calc_output(wc.mix3, v_set_out_A, v_set_out_B, v_set_out_C)
    wc.v_set_out = v_set_out_A # for logging, store the output of the speed controller
end
        
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
