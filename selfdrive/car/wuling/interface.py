#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV
from panda.python import uds

from selfdrive.car import STD_CARGO_KG,scale_tire_stiffness,create_button_events, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.wuling.values import CAR, CruiseButtons, PREGLOBAL_CARS, CarControllerParams, CanBus
from common.params import Params
from common.op_params import opParams
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.common.params import Params
from decimal import Decimal

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}

CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.dp_cruise_speed = 0. # km/h
    self.dp_override_speed_last = 0. # km/h
    self.dp_override_speed = 0. # m/s

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "wuling"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]
    ret.radarUnavailable = True
    ret.dashcamOnly = candidate in PREGLOBAL_CARS
    
    params = Params()

    ret.experimentalLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = experimental_long
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    
    op_params = opParams("wuling car_interface.py for lateral override")
    tire_stiffness_factor = 0.444
    
    ret.steerLimitTimer = 0.4
    # ret.steerActuatorDelay = 0.2
    ret.steerActuatorDelay = float(Decimal(params.get("SteerActuatorDelayAdj", encoding="utf8")) * Decimal('0.01'))

    ret.mass = 1950.
    ret.wheelbase = 2.75
    ret.steerRatio = op_params.get('steer_ratio', force_update=True)
    ret.centerToFront = ret.wheelbase * 0.4
    ret.tireStiffnessFactor = 0.82

    ret.transmissionType = TransmissionType.automatic

    ret.minEnableSpeed = -1
    ret.minSteerSpeed = -1
    
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    return ret
  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disabled = disable_ecu(logcan, sendcan, bus=0, addr=0x728, sub_addr=0xf, com_cont_req=communication_control)
      print(f"Radar disabled: {disabled}")

  # returns a car.CarState
  def _update(self, c):

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
    # self.CS = self.sp_update_params(self.CS)
    
    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.CS.cruise_buttons != CruiseButtons.UNPRESS or self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      ret.buttonEvents = create_button_events(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT,
                                              unpressed_btn=CruiseButtons.UNPRESS)

    ret.engineRpm = self.CS.engineRPM
    
    # if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
    #   buttonEvents.append(create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))
    #   # Handle ACCButtons changing buttons mid-press
    #   if self.CS.cruise_buttons != CruiseButtons.UNPRESS and self.CS.prev_cruise_buttons != CruiseButtons.UNPRESS:
    #     buttonEvents.append(create_button_event(CruiseButtons.UNPRESS, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))

    # # self.CS.mads_enabled = self.get_sp_cruise_main_state(ret, self.CS)

    # if not self.CP.pcmCruise:
    #   if any(b.type == ButtonType.accelCruise and b.pressed for b in buttonEvents):
    #     self.CS.accEnabled = True

    # self.CS.accEnabled, buttonEvents = self.get_sp_v_cruise_non_pcm_state(ret, self.CS.accEnabled,
    #                                                                       buttonEvents, c.vCruise)

    # if ret.cruiseState.available:
    #   if self.enable_mads:
    #     if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
    #       self.CS.madsEnabled = True
    #     if self.CS.prev_lkas_enabled != 1 and self.CS.lkas_enabled == 1:
    #       self.CS.madsEnabled = not self.CS.madsEnabled
    #     self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
    #   self.toggle_gac(ret, self.CS, bool(self.CS.gap_dist_button), 1, 3, 3, "-")
    # else:
    #   self.CS.madsEnabled = False

    # if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0):
    #   if any(b.type == ButtonType.cancel for b in buttonEvents):
    #     self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    # if self.get_sp_pedal_disengage(ret):
    #   self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    #   ret.cruiseState.enabled = False if self.CP.pcmCruise else self.CS.accEnabled

    # if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
    #   if ret.gasPressed and not ret.cruiseState.enabled:
    #     self.CS.accEnabled = False
    #   self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    # ret, self.CS = self.get_sp_common_state(ret, self.CS, gap_button=bool(self.CS.gap_dist_button))

    # MADS BUTTON
    # if self.CS.out.madsEnabled != self.CS.madsEnabled:
    #   if self.mads_event_lock:
    #     buttonEvents.append(create_mads_event(self.mads_event_lock))
    #     self.mads_event_lock = False
    # else:
    #   if not self.mads_event_lock:
    #     buttonEvents.append(create_mads_event(self.mads_event_lock))
    #     self.mads_event_lock = True

    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=self.CP.pcmCruise, enable_buttons=(ButtonType.decelCruise,))
    
    if not self.CP.pcmCruise:
      if any(b.type == ButtonType.accelCruise and b.pressed for b in ret.buttonEvents):
        events.add(EventName.buttonEnable)

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20):
      events.add(EventName.belowEngageSpeed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(EventName.belowSteerSpeed)

    if self.CC.standstill_res_button:
        events.add(EventName.standstillResButton)
    if self.CC.cruise_gap_adjusting:
      events.add(EventName.gapAdjusting)
    if self.CC.on_speed_bump_control and ret.vEgo > 8.3:
      events.add(EventName.speedBump)
    if self.CC.on_speed_control and ret.vEgo > 0.3:
      events.add(EventName.camSpeedDown)
    if self.CC.curv_speed_control and ret.vEgo > 8.3:
      events.add(EventName.curvSpeedDown)
    if self.CC.cut_in_control and ret.vEgo > 8.3:
      events.add(EventName.cutinDetection)
    if self.CC.driver_scc_set_control:
      events.add(EventName.sccDriverOverride)        
    if self.CC.autohold_popup_timer:
      events.add(EventName.autoHold)
    if self.CC.auto_res_starting:
      events.add(EventName.resCruise)
    if self.CC.e2e_standstill:
      events.add(EventName.chimeAtResume)
      
    if self.CS.cruiseState_standstill or self.CC.standstill_status == 1:
      #events.add(EventName.standStill)
      self.CP.standStill = True
    else:
      self.CP.standStill = False
    if self.CC.v_cruise_kph_auto_res > (20 if self.CS.is_set_speed_in_mph else 30):
      self.CP.vCruisekph = self.CC.v_cruise_kph_auto_res
    else:
      self.CP.vCruisekph = 0
    if self.CC.res_speed != 0:
      self.CP.resSpeed = self.CC.res_speed
    else:
      self.CP.resSpeed = 0
    if self.CC.vFuture >= 1:
      self.CP.vFuture = self.CC.vFuture
    else:
      self.CP.vFuture = 0
    if self.CC.vFutureA >= 1:
      self.CP.vFutureA = self.CC.vFutureA
    else:
      self.CP.vFutureA = 0
    self.CP.aqValue = self.CC.aq_value
    self.CP.aqValueRaw = self.CC.aq_value_raw

    if self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 0:
      events.add(EventName.modeChangeOpenpilot)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 1:
      events.add(EventName.modeChangeDistcurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 2:
      events.add(EventName.modeChangeDistance)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 3:
      events.add(EventName.modeChangeCurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 4:
      events.add(EventName.modeChangeOneway)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 5:
      events.add(EventName.modeChangeMaponly)

    if self.CC.lkas_temp_disabled:
      events.add(EventName.lkasDisabled)
    elif self.CC.lkas_temp_disabled_timer:
      events.add(EventName.lkasEnabled)
      
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
