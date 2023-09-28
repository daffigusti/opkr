from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS,CruiseButtons, CarControllerParams
import cereal.messaging as messaging
from common.params import Params

from openpilot.selfdrive.car.wuling.navicontrol  import NaviControl
from openpilot.common.params import Params
import openpilot.common.log as trace1
from random import randint
from decimal import Decimal

VisualAlert = car.CarControl.HUDControl.VisualAlert
NetworkLocation = car.CarParams.NetworkLocation
LongCtrlState = car.CarControl.Actuators.LongControlState

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0
    self.frame = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.brake_counter = 0
    self.c_params = Params()

    self.cancel_counter = 0

    self.lka_steering_cmd_counter = 0
    self.lka_steering_cmd_counter_last = -1

    self.lka_icon_status_last = (False, False)

    self.params = CarControllerParams(self.CP)
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])
    
    
    self.aq_value = 0
    self.aq_value_raw = 0

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.last_resume_frame = 0
    self.accel = 0

    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above = False
    self.driver_steering_torque_above_timer = 150
    
    self.mode_change_timer = 0

    self.acc_standstill_timer = 0
    self.acc_standstill = False

    self.need_brake = False
    self.need_brake_timer = 0

    self.cancel_counter = 0

    self.v_cruise_kph_auto_res = 0
    
    self.param_s = Params()

    self.mode_change_switch = int(self.c_params.get("CruiseStatemodeSelInit", encoding="utf8"))
    self.opkr_variablecruise = self.c_params.get_bool("OpkrVariableCruise")
    self.opkr_autoresume = self.c_params.get_bool("OpkrAutoResume")
    self.opkr_cruisegap_auto_adj = self.c_params.get_bool("CruiseGapAdjust")
    self.opkr_cruise_auto_res = self.c_params.get_bool("CruiseAutoRes")
    self.opkr_cruise_auto_res_option = int(self.c_params.get("AutoResOption", encoding="utf8"))
    self.opkr_cruise_auto_res_condition = int(self.c_params.get("AutoResCondition", encoding="utf8"))
    
    self.stopping_dist_adj_enabled = self.c_params.get_bool("StoppingDistAdj")
    self.standstill_resume_alt = self.c_params.get_bool("StandstillResumeAlt")
    self.auto_res_delay = int(self.c_params.get("AutoRESDelay", encoding="utf8")) * 100
    self.auto_res_delay_timer = 0
    self.stopped = False
    self.stoppingdist = float(Decimal(self.c_params.get("StoppingDist", encoding="utf8"))*Decimal('0.1'))

    self.sm = messaging.SubMaster(['longitudinalPlan'])
    self.is_metric = self.param_s.get_bool("IsMetric")
    self.speed_limit_control_enabled = False
    self.last_speed_limit_sign_tap = False
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    
    self.longcontrol = self.CP.openpilotLongitudinalControl
    #self.scc_live is true because CP.radarUnavailable is False
    self.scc_live = not self.CP.radarUnavailable
    
    self.NC = NaviControl()

    
    self.dRel = 0
    self.vRel = 0
    self.yRel = 0

    self.cruise_gap_prev = 0
    self.cruise_gap_set_init = False
    self.cruise_gap_adjusting = False
    self.standstill_fault_reduce_timer = 0
    self.standstill_res_button = False
    self.standstill_res_count = int(self.c_params.get("RESCountatStandstill", encoding="utf8"))
    
    self.variable_steer_max = self.c_params.get_bool("OpkrVariableSteerMax")
    self.variable_steer_delta = self.c_params.get_bool("OpkrVariableSteerDelta")
    self.osm_spdlimit_enabled = self.c_params.get_bool("OSMSpeedLimitEnable")
    self.stock_safety_decel_enabled = self.c_params.get_bool("UseStockDecelOnSS")
    self.joystick_debug_mode = self.c_params.get_bool("JoystickDebugMode")
    #self.stopsign_enabled = self.c_params.get_bool("StopAtStopSign")

    self.smooth_start = False

    self.cc_timer = 0
    self.on_speed_control = False
    self.on_speed_bump_control = False
    self.curv_speed_control = False
    self.cut_in_control = False
    self.driver_scc_set_control = False
    self.vFuture = 0
    self.vFutureA = 0
    self.cruise_init = False
    self.change_accel_fast = False
    
    self.standstill_status = 0
    self.standstill_status_timer = 0
    self.switch_timer = 0
    self.switch_timer2 = 0
    self.auto_res_timer = 0
    self.auto_res_limit_timer = 0
    self.auto_res_limit_sec = int(self.c_params.get("AutoResLimitTime", encoding="utf8")) * 100
    self.auto_res_starting = False
    self.res_speed = 0
    self.res_speed_timer = 0
    self.autohold_popup_timer = 0
    self.autohold_popup_switch = False
    
    self.gap_by_spd_on = self.c_params.get_bool("CruiseGapBySpdOn")
    self.gap_by_spd_spd = list(map(int, Params().get("CruiseGapBySpdSpd", encoding="utf8").split(',')))
    self.gap_by_spd_gap = list(map(int, Params().get("CruiseGapBySpdGap", encoding="utf8").split(',')))
    self.gap_by_spd_on_buffer1 = 0
    self.gap_by_spd_on_buffer2 = 0
    self.gap_by_spd_on_buffer3 = 0
    self.gap_by_spd_gap1 = False
    self.gap_by_spd_gap2 = False
    self.gap_by_spd_gap3 = False
    self.gap_by_spd_gap4 = False
    self.gap_by_spd_on_sw = False
    self.gap_by_spd_on_sw_trg = True
    self.gap_by_spd_on_sw_cnt = 0
    self.gap_by_spd_on_sw_cnt2 = 0


    self.vrel_delta = 0
    self.vrel_delta_prev = 0
    self.vrel_delta_timer = 0
    self.vrel_delta_timer2 = 0
    self.vrel_delta_timer3 = 0

    self.e2e_standstill_enable = self.c_params.get_bool("DepartChimeAtResume")
    self.e2e_standstill = False
    self.e2e_standstill_stat = False
    self.e2e_standstill_timer = 0
    self.e2e_standstill_timer_buf = 0
    
    self.prev_cruiseButton = 0
    self.gapsettingdance = 4
    self.lead_visible = False
    self.lead_debounce = 0
    self.radarDisableOverlapTimer = 0
    self.objdiststat = 0
    self.fca11supcnt = self.fca11inc = self.fca11alivecnt = self.fca11cnt13 = 0
    self.fca11maxcnt = 0xD
    
    self.lkas_onoff_counter = 0
    self.lkas_temp_disabled = False
    self.lkas_temp_disabled_timer = 0
    self.model_speed = 255.0
    self.model_speed = 255.0
    self.model_speed_range = [30, 100, 255]
    
    self.experimental_long_enabled = self.c_params.get_bool("ExperimentalLongitudinalEnabled")
    self.experimental_mode = self.c_params.get_bool("ExperimentalMode")
    self.live_torque_params = self.c_params.get_bool("OpkrLiveTorque")
    
    self.opkr_long_alt = True if int(self.c_params.get("OPKRLongAlt", encoding="utf8")) in (1, 2) else False

    self.str_log2 = 'MultiLateral'
    if CP.lateralTuning.which() == 'pid':
      self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.5f}/{:0.2f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kf, CP.lateralTuning.pid.kd)
    elif CP.lateralTuning.which() == 'indi':
      self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGainV[0], CP.lateralTuning.indi.outerLoopGainV[0], \
       CP.lateralTuning.indi.timeConstantV[0], CP.lateralTuning.indi.actuatorEffectivenessV[0])
    elif CP.lateralTuning.which() == 'lqr':
      self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)
    elif CP.lateralTuning.which() == 'torque':
      self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.2f}/{:0.3f}'.format(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.kf, CP.lateralTuning.torque.ki, CP.lateralTuning.torque.friction)

    self.sm = messaging.SubMaster(['controlsState', 'radarState', 'lateralPlan', 'longitudinalPlan', 'liveTorqueParameters', 'liveENaviData'])

    
  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    self.vFuture = hud_control.vFuture
    self.vFutureA = hud_control.vFutureA
    new_steer = 0
    apply_steer = 0
    
    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    if self.frame % 10 == 0:
      self.model_speed = self.sm['lateralPlan'].modelSpeed

    self.dRel = self.sm['radarState'].leadOne.dRel #Vision Lead
    self.vRel = self.sm['radarState'].leadOne.vRel #Vision Lead
    self.yRel = self.sm['radarState'].leadOne.yRel #Vision Lead
    
    # if not self.CP.pcmCruiseSpeed:
    #   self.sm.update(0)

    #   if self.sm.updated['longitudinalPlan']:
    #     self.v_tsc_state = self.sm['longitudinalPlan'].visionTurnControllerState
    #     self.slc_state = self.sm['longitudinalPlan'].speedLimitControlState
    #     self.m_tsc_state = self.sm['longitudinalPlan'].turnSpeedControlState
    #     self.speed_limit = self.sm['longitudinalPlan'].speedLimit
    #     self.speed_limit_offset = self.sm['longitudinalPlan'].speedLimitOffset
    #     self.v_tsc = self.sm['longitudinalPlan'].visionTurnSpeed
    #     self.m_tsc = self.sm['longitudinalPlan'].turnSpeed

    #   if self.frame % 200 == 0:
    #     self.speed_limit_control_enabled = self.param_s.get_bool("SpeedLimitControl")
    #     self.is_metric = self.param_s.get_bool("IsMetric")
    #   self.last_speed_limit_sign_tap = self.param_s.get_bool("LastSpeedLimitSignTap")
    #   self.v_cruise_min = MAZDA_V_CRUISE_MIN[self.is_metric] * (CV.KPH_TO_MPH if not self.is_metric else 1)

    # Send CAN commands.
    can_sends = []

    # if not self.CP.pcmCruiseSpeed:
    #       if not self.last_speed_limit_sign_tap_prev and self.last_speed_limit_sign_tap:
    #         self.sl_force_active_timer = self.frame
    #         put_bool_nonblocking("LastSpeedLimitSignTap", False)
    #       self.last_speed_limit_sign_tap_prev = self.last_speed_limit_sign_tap

    #       sl_force_active = self.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
    #       sl_inactive = not sl_force_active and (not self.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
    #       sl_temp_inactive = not sl_force_active and (self.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
    #       slc_active = not sl_inactive and not sl_temp_inactive

    #       self.slc_active_stock = slc_active

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      # if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
      #   # Cancel Stock ACC if it's enabled while OP is disengaged
      #   # Send at a rate of 10hz until we sync with stock ACC state
      #   can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0

      # if CC.cruiseControl.resume and self.frame % 2 == 0:
      if CS.resume_alert == 1 and self.frame % 2 == 0:
        if CS.resume_alert == 1:
          print("Cruize button %s " % CC.cruiseControl.resume)
          print("Resule Alert %s " % CS.resume_alert)
        # Send Resume button when planner wants car to move
          can_sends.append(wulingcan.create_buttons(self.packer_pt, CS.crz_btns_counter+1, CruiseButtons.RES_ACCEL))
          print("Send Resume 2 %d" % (CS.crz_btns_counter+1))
          self.last_button_frame = self.frame

    # if CS.steeringPressed:
    #     can_sends.append(wulingcan.create_resume_button())
    #     print("Send Resume")
    # Steering (Active: 50Hz
    steer_step = self.params.STEER_STEP
    lat_active = CC.latActive
    
    self.lka_steering_cmd_counter += 1 if CS.loopback_lka_steering_cmd_updated else 0


    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we
    # received the ASCMLKASteeringCmd loopback confirmation too recently
    last_lka_steer_msg_ms = (now_nanos - CS.loopback_lka_steering_cmd_ts_nanos) * 1e-6
    if  (self.frame  % self.params.STEER_STEP) == 0:
      if CC.latActive:
        new_steer = int(round(actuators.steer * self.params.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
      else:
        apply_steer = 0
      
      self.apply_steer_last = apply_steer
      self.last_steer_frame = self.frame
      
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_steer, self.frame, CC.latActive))
    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
    
   
    # **** HUD Controls ***************************************************** #
    # if self.frame % self.params.LKAS_HUD_STEP == 0:
    #   can_sends.append(wulingcan.create_lkas_hud(self.packer_pt, CanBus.POWERTRAIN, CS.lkas_hud, steer_alert))
      
    if CS.acc_active:
      btn_signal = self.NC.update(CS)
    # # SW_GMLAN not yet on cam harness, no HUD alerts
    # if self.CP.networkLocation != NetworkLocation.fwdCamera and (self.frame % self.params.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last):
    #   steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
    #   can_sends.append(wulingcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
    #   self.lka_icon_status_last = lka_icon_status
    str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}/{:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
    int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.mdpsBus, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_steer), abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
    # trace1.printf2( '{}'.format( str_log2 ) )
    stopping = actuators.longControlState == LongCtrlState.stopping

    if self.opkr_variablecruise and CS.acc_active:
      btn_signal = self.NC.update(CS)
      self.btnsignal = btn_signal
      
      if self.frame % 20 == 0:
        print("Buttn signato")
        print(btn_signal)
        
      self.on_speed_control = self.NC.onSpeedControl
      self.on_speed_bump_control = self.NC.onSpeedBumpControl
      self.curv_speed_control = self.NC.curvSpeedControl
      self.cut_in_control = self.NC.cutInControl
      self.driver_scc_set_control = self.NC.driverSccSetControl
    else:
      self.on_speed_control = False
      self.on_speed_bump_control = False
      self.curv_speed_control = False
      self.cut_in_control = False
      self.driver_scc_set_control = False
      self.cruise_gap_adjusting = False
      self.standstill_res_button = False
      self.auto_res_starting = False
      self.gap_by_spd_gap1 = False
      self.gap_by_spd_gap2 = False
      self.gap_by_spd_gap3 = False
      self.gap_by_spd_gap4 = False
        
    if not CC.enabled:
      self.cruise_init = False
      self.lkas_temp_disabled = False
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer_buf = 0
    elif CS.cruise_active:
      self.cruise_init = True
      self.cancel_counter = 0
      self.auto_res_limit_timer = 0
      self.auto_res_delay_timer = 0
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer_buf = 0
      if self.res_speed_timer > 0:
        self.res_speed_timer -= 1
        self.auto_res_starting = False
      else:
        self.auto_res_starting = False
        self.v_cruise_kph_auto_res = 0
        self.res_speed = 0
    else:
      if CS.out.brakeLights:
        self.auto_res_limit_timer = 0
        self.auto_res_delay_timer = 0
      else:
        if self.auto_res_limit_timer < self.auto_res_limit_sec:
          self.auto_res_limit_timer += 1
        if self.auto_res_delay_timer < self.auto_res_delay:
          self.auto_res_delay_timer += 1
      
      if self.e2e_standstill_enable:
        try:
          if self.e2e_standstill:
            self.e2e_standstill_timer += 1
            if self.e2e_standstill_timer > 100:
              self.e2e_standstill = False
              self.e2e_standstill_timer = 0
          elif CS.clu_Vanz > 0:
            self.e2e_standstill = False
            self.e2e_standstill_stat = False
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf = 0
          elif self.e2e_standstill_stat and self.sm['longitudinalPlan'].e2eX[12] > 30 and CS.clu_Vanz == 0:
            self.e2e_standstill = True
            self.e2e_standstill_stat = False
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf += 300
          elif 0 < self.sm['longitudinalPlan'].e2eX[12] < 10 and CS.clu_Vanz == 0:
            self.e2e_standstill_timer += 1
            if self.e2e_standstill_timer > (300 + self.e2e_standstill_timer_buf):
              self.e2e_standstill_timer = 101
              self.e2e_standstill_stat = True
          else:
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf = 0
        except:
          pass
    
    if CS.out.autoHold and not self.autohold_popup_switch:
      self.autohold_popup_timer = 100
      self.autohold_popup_switch = True
    elif CS.out.autoHold and self.autohold_popup_switch and self.autohold_popup_timer:
      self.autohold_popup_timer -= 1
    elif not CS.out.autoHold and self.autohold_popup_switch:
      self.autohold_popup_switch = False
      self.autohold_popup_timer = 0
    
    opkr_cruise_auto_res_condition = False
    opkr_cruise_auto_res_condition = not self.opkr_cruise_auto_res_condition or CS.out.gasPressed
    t_speed = 30
    if self.auto_res_timer > 0:
      self.auto_res_timer -= 1
    elif self.model_speed > 95 and self.cancel_counter == 0 and not CS.cruise_active and not CS.out.brakeLights and round(CS.VSetDis) >= t_speed and \
    (1 < CS.lead_distance < 149 or round(CS.VSetDis) > t_speed) and round(CS.VSetDis) >= 3 and self.cruise_init and \
    self.opkr_cruise_auto_res and opkr_cruise_auto_res_condition and (self.auto_res_limit_sec == 0 or self.auto_res_limit_timer < self.auto_res_limit_sec) and \
    (self.auto_res_delay == 0 or self.auto_res_delay_timer >= self.auto_res_delay):
      if self.opkr_cruise_auto_res_option == 0:
        can_sends.append(wulingcan.create_buttons(self.packer_pt, CS.crz_btns_counter+1, CruiseButtons.RES_ACCEL))
        self.auto_res_starting = True
        self.res_speed = round(CS.VSetDis)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.auto_res_timer = int(randint(20, 25) * 2)
      elif self.opkr_cruise_auto_res_option == 1:
        can_sends.append(wulingcan.create_buttons(self.packer_pt, CS.crz_btns_counter+1, CruiseButtons.DECEL_SET))
        self.auto_res_starting = True
        self.v_cruise_kph_auto_res = round(CS.VSetDis)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.auto_res_timer = int(randint(20, 25) * 2)
            
    if CS.out.brakeLights and CS.out.vEgo == 0 and not CS.out.cruiseState.standstill:
      self.standstill_status_timer += 1
      if self.standstill_status_timer > 200:
        self.standstill_status = 1
        self.standstill_status_timer = 0
    if self.standstill_status == 1 and CS.out.vEgo > 1:
      self.standstill_status = 0
      self.standstill_fault_reduce_timer = 0
      self.last_resume_frame = self.frame
      self.res_switch_timer = 0
      self.resume_cnt = 0

    if CS.out.vEgo <= 1:
      if stopping and CS.out.vEgo < 0.1 and not CS.out.gasPressed:
        self.acc_standstill_timer += 1
        if self.acc_standstill_timer >= 200:
          self.acc_standstill_timer = 200
          self.acc_standstill = True
      else:
        self.acc_standstill_timer = 0
        self.acc_standstill = False
    elif CS.out.gasPressed or CS.out.vEgo > 1:
      self.acc_standstill = False
      self.acc_standstill_timer = 0      
    else:
      self.acc_standstill = False
      self.acc_standstill_timer = 0
      
    if self.c_params.get_bool("OpkrLiveTunePanelEnable"):
      if self.CP.lateralTuning.which() == 'pid':
        self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.1f}/{:0.5f}'.format(float(Decimal(self.c_params.get("PidKp", encoding="utf8"))*Decimal('0.01')), \
        float(Decimal(self.c_params.get("PidKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.c_params.get("PidKd", encoding="utf8"))*Decimal('0.01')), \
        float(Decimal(self.c_params.get("PidKf", encoding="utf8"))*Decimal('0.00001')))
      elif self.CP.lateralTuning.which() == 'indi':
        self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(float(Decimal(self.c_params.get("InnerLoopGain", encoding="utf8"))*Decimal('0.1')), \
        float(Decimal(self.c_params.get("OuterLoopGain", encoding="utf8"))*Decimal('0.1')), float(Decimal(self.c_params.get("TimeConstant", encoding="utf8"))*Decimal('0.1')), \
        float(Decimal(self.c_params.get("ActuatorEffectiveness", encoding="utf8"))*Decimal('0.1')))
      elif self.CP.lateralTuning.which() == 'lqr':
        self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(float(Decimal(self.c_params.get("Scale", encoding="utf8"))*Decimal('1.0')), \
        float(Decimal(self.c_params.get("LqrKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.c_params.get("DcGain", encoding="utf8"))*Decimal('0.00001')))
      elif self.CP.lateralTuning.which() == 'torque':
        self.str_log2 = 'T={:0.1f}/{:0.1f}/{:0.1f}/{:0.1f}/{:0.3f}'.format(float(Decimal(self.c_params.get("TorqueMaxLatAccel", encoding="utf8"))*Decimal('0.1')), \
        float(Decimal(self.c_params.get("TorqueKp", encoding="utf8"))*Decimal('0.1')), \
        float(Decimal(self.c_params.get("TorqueKf", encoding="utf8"))*Decimal('0.1')), float(Decimal(self.c_params.get("TorqueKi", encoding="utf8"))*Decimal('0.1')), \
        float(Decimal(self.c_params.get("TorqueFriction", encoding="utf8")) * Decimal('0.001')))
    elif self.CP.lateralTuning.which() == 'torque' and self.live_torque_params:
      torque_params = self.sm['liveTorqueParameters']
      self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.3f}'.format(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered, torque_params.frictionCoefficientFiltered)
    
    trace1.printf1('{}  {}'.format(str_log1, self.str_log2))

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake
    
    # print("Output %d", new_actuators.steerOutputCan)

    self.frame += 1
    return new_actuators, can_sends

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)
