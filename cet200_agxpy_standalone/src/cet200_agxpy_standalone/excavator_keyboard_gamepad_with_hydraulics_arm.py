# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
import agx
import agxSDK
import agxHydraulics
import agxPowerLine

from agxPythonModules.utils.callbacks import (
    KeyboardCallback as Input,
    GamepadCallback as Gamepad,
)
from agxPythonModules.utils.environment import simulation

class CollectorParams:
    """データ収集パラメータ"""
    fluid_density: float = 850.0
    pipe_diameter: float = 0.032
    pipe_length: float = 0.1
    
    relief_cracking_pressure: float = 35.0e6
    relief_fully_open_pressure: float = 37.3e6
    relief_diameter: float = 0.032
    
    piston_bore_diameter: float = 0.135
    piston_rod_diameter: float = 0.095
    
    # シリンダ範囲
    piston_stroke: float = 1.49  # m
    initial_position: float = 0.125  # m
    

    pump_flow_rate: float = 300.0 / 60000.0  # [m^3/s], 300 L/min

    target_delta_pressure: float = 6e6  # Pa
    
    # データ収集設定
    input_change_period: float = 0.5  # レバー入力変更周期 [s]
    simulation_time: float = 3600  # シミュレーション時間 [s]
    valve_transition_time: float = 0.05  # 弁開閉の遷移時間 [s]
    
    time_step: float = 1 / 10000
    ramp_up_time: float = 0.1
    
class FourWayThreePositionSpoolValve:
    """4ウェイ/3ポジションのスプールバルブ"""
    
    def __init__(self, 
                 in1: agxHydraulics.FlowUnit, in1Side,
                 in2: agxHydraulics.FlowUnit, in2Side,
                 out1: agxHydraulics.FlowUnit, out1Side,
                 out2: agxHydraulics.FlowUnit, out2Side):
        
        self.m_in1 = in1
        self.m_in2 = in2
        self.m_out1 = out1
        self.m_out2 = out2
        
        self.m_spool = agxHydraulics.SpoolValve()

        all_connected = all([
            self.m_spool.connect(agxPowerLine.INPUT, in1Side, in1),
            self.m_spool.connect(agxPowerLine.INPUT, in2Side, in2),
            self.m_spool.connect(agxPowerLine.OUTPUT, out1Side, out1),
            self.m_spool.connect(agxPowerLine.OUTPUT, out2Side, out2)
        ])

        assert all_connected, "SpoolValveの接続に失敗しました"
        self.linkNone()

    def linkNone(self):
        """ニュートラル (全閉)"""
        self.m_spool.unlink(self.m_in1)
        self.m_spool.unlink(self.m_in2)
        assert self.m_spool.getNumLinks() == 0

    def linkParallel(self):
        """平行接続 (伸び方向)"""
        self.linkNone()
        self.m_spool.link(self.m_in1, self.m_out1)
        self.m_spool.link(self.m_in2, self.m_out2)
        assert self.m_spool.getNumLinks() == 2

    def linkCross(self):
        """交差接続 (縮み方向)"""
        self.linkNone()
        self.m_spool.link(self.m_in1, self.m_out2)
        self.m_spool.link(self.m_in2, self.m_out1)
        assert self.m_spool.getNumLinks() == 2

class HydraulisCircuit:
    def __init__(self,joint: agx.Prismatic,sim: agxSDK.Simulation,params:CollectorParams):
        self.params = params
        OUTPUT = agxPowerLine.OUTPUT
        INPUT = agxPowerLine.INPUT
        pipe_area = agxHydraulics.diameterToArea(params.pipe_diameter)
        barrel_area = agxHydraulics.diameterToArea(params.piston_bore_diameter)
        rod_area = agxHydraulics.diameterToArea(params.piston_rod_diameter)
        piston_area = barrel_area - rod_area

        self._ensure_joint_range(joint)
        joint.getMotor1D().setEnable(False)

        self.pump = agxHydraulics.ConstantFlowValve(
            params.pipe_length, pipe_area, params.fluid_density, 0.0
        )
        self.pump.setAllowPumping(True)
        self.pump.setEnable(True)
        self.p_1 = self.create_pipe(pipe_area,params.fluid_density,params.pipe_length)
                
        self.p_2 = self.create_pipe(pipe_area,params.fluid_density,params.pipe_length)

        self.p_3 = self.create_pipe(pipe_area,params.fluid_density,params.pipe_length)

        self.p_4 = self.create_pipe(pipe_area,params.fluid_density,params.pipe_length)

        self.p_5 = self.create_pipe(pipe_area,params.fluid_density,params.pipe_length)


        self.relief_valve = agxHydraulics.ReliefValve(
            params.relief_cracking_pressure,
            params.relief_fully_open_pressure,
            agxHydraulics.diameterToArea(params.relief_diameter)
        )

        self.piston = agxHydraulics.PistonActuator(
            joint, barrel_area, piston_area, params.fluid_density
        )

        self.spool = FourWayThreePositionSpoolValve(
            self.p_1, OUTPUT,
            self.p_4, INPUT,
            self.p_2, INPUT,
            self.p_3, INPUT,
        )

        self.pump.connect(self.p_5)
        self.p_5.connect(self.relief_valve)
        self.relief_valve.connect(self.p_1)
        self.p_2.connect(self.piston)
        self.piston.connect(OUTPUT,OUTPUT,self.p_3)
        self._setup_connection(sim)

    def _ensure_joint_range(self, joint: agx.Prismatic):
        range_1d = joint.getRange1D()
        range_value = range_1d.getRange()
        range_width = range_value.upper() - range_value.lower()
        if range_width <= 0.0:
            current_position = joint.getAngle()
            range_1d.setRange(agx.RangeReal(
                current_position - self.params.initial_position,
                current_position - self.params.initial_position + self.params.piston_stroke,
            ))
        range_1d.setEnable(True)

    def create_pipe(self,area,fluid_density,length=1.0):
        return agxHydraulics.Pipe(length,area,fluid_density)
    
    def _setup_connection(self, sim: agxSDK.Simulation):
        self.powerline = agxPowerLine.PowerLine()
        sim.add(self.powerline)
        self.powerline.add(self.pump)
    
    def control_input(self,u):
        if abs(u) <= 0.0:
            self.pump.setTargetFlowRate(0.0)
            self.spool.linkNone()
            return

        self.pump.setTargetFlowRate(abs(u) * self.params.pump_flow_rate)

        if u > 0.0:
            self.spool.linkParallel()   # 伸び
        else:
            self.spool.linkCross()      # 縮み


# 各関節に与える最大速度。
# キーボードやゲームパッドの入力値 throttle [-1, 1] にこの値を掛けて Motor1D の目標速度にする。
max_speeds = {
    'Hinge_Slew': 1.256637,
    'Prismatic_Boom': 0.648409027 * 0.5,
    'Prismatic_Bucket': 0.706018147,
    'Hinge_Sprocket': 4.074104494,
}

# 旋回停止時だけ Motor1D の許容トルクを大きくして、上部旋回体が流れにくいようにする。
HINGE_SLEW_BRAKE_MULTIPLIER = 4
HINGE_SLEW_MOTOR_TORQUE: agx.RangeReal
HINGE_SLEW_BRAKE_TORQUE: agx.RangeReal


def _set_speed(joint: agx.Constraint1DOF, throttle: float):
    # ブーム/アーム/バケット/旋回などの 1 自由度拘束へ速度指令を出す。
    # throttle は -1.0 から 1.0 程度を想定し、関節ごとの max_speeds で実速度へ変換する。
    if joint.getName() == "Prismatic_Arm":
        raise RuntimeError("Prismatic_Arm is driven by hydraulics, not Motor1D.")

    target_speed = throttle * max_speeds[joint.getName()]
    motor1d: agx.Motor1D = joint.getMotor1D()
    motor1d.setSpeed(target_speed)

    if joint.getName() == "Hinge_Slew":
        # 旋回中は通常の力範囲、停止指令時は強めのブレーキ力範囲へ切り替える。
        motor1d.setForceRange(HINGE_SLEW_MOTOR_TORQUE)
        if target_speed == 0.0:
            motor1d.setForceRange(HINGE_SLEW_BRAKE_TORQUE)


def _set_sprocket_speed(joint: agx.Hinge, throttle: float):
    # 左右履帯のスプロケットへ速度指令を出す。
    # 左右を同じ向きに回すと前後進、反対向きに回すとその場旋回になる。
    speed = throttle * max_speeds['Hinge_Sprocket']
    joint.getMotor1D().setSpeed(speed)


def handle_stick_dead_zone(raw_value):
    # アナログスティックの中心付近の微小入力を 0 にする。
    # dead_zone を超えた分だけ 0..1 に再スケールし、操作開始時の急なジャンプを抑える。
    dead_zone = 0.3
    abs_raw_value = abs(raw_value)
    if abs_raw_value <= dead_zone:
        return 0
    sign = 1.0 if raw_value > 0.0 else -1.0
    return sign * (abs_raw_value - dead_zone) / (1 - dead_zone)


def is_stick_moved(value, delta_value):
    # 値が 0 でも delta があれば「中心へ戻った」イベントとして扱う。
    # これを無視すると、スティックを離したときに最後の速度指令が残る場合がある。
    if abs(value) > 0.0:
        return True
    if abs(delta_value) > 0.0:
        return True
    return False


def _set_speed_by_gamepad(joint: agx.Constraint1DOF, throttle: float, delta_throttle):
    # ゲームパッド軸入力をデッドゾーン処理してから 1 自由度拘束へ渡す。
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    _set_speed(joint, throttle)


def _set_hydraulic_by_gamepad(circuit: HydraulisCircuit, throttle: float, delta_throttle):
    # ゲームパッド軸入力をデッドゾーン処理してから油圧回路へ渡す。
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    circuit.control_input(throttle)


def _set_sprocket_speed_by_gamepad(joint: agx.Hinge, throttle: float, delta_throttle: float):
    # ゲームパッド軸/ボタン入力を左右履帯スプロケットの速度指令へ変換する。
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    _set_sprocket_speed(joint, throttle)


class ExcavatorKeyboardControl(agxSDK.GuiEventListener):
    def __init__(self, excavator, arm_hydraulic_circuit: HydraulisCircuit):
        # AGX Viewer のキーボードイベントを受け取る GuiEventListener。
        # 操作対象になる関節を初期化時に名前で取得しておく。
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)
        self.slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
        self.boom_joint = excavator.getConstraint1DOF("Prismatic_Boom")
        self.arm_hydraulic_circuit = arm_hydraulic_circuit
        self.bucket_joint = excavator.getConstraint1DOF("Prismatic_Bucket")
        self.sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
        self.sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    def keyboard(self, key, x, y, alt, down):
        # down=True はキー押下、down=False はキー解放。
        # 押下中だけ throttle=0.6 を与え、解放時は 0 にして停止させる。
        handled = False
        throttle = 0.6 if down else 0

        # 旋回: a/s で上部旋回体を左右へ回す。
        if key == ord('a'):
            _set_speed(self.slew_joint, throttle)
            handled = True
        if key == ord('s'):
            _set_speed(self.slew_joint, -throttle)
            handled = True

        # アーム: z/x でアームシリンダ相当の Prismatic_Arm を伸縮させる。
        if key == ord('z'):
            self.arm_hydraulic_circuit.control_input(-throttle)
            handled = True
        if key == ord('x'):
            self.arm_hydraulic_circuit.control_input(throttle)
            handled = True

        # ブーム: m/, でブームシリンダ相当の Prismatic_Boom を伸縮させる。
        if key == ord('m'):
            _set_speed(self.boom_joint, throttle)
            handled = True
        if key == ord(','):
            _set_speed(self.boom_joint, -throttle)
            handled = True

        # バケット: j/k でバケットシリンダ相当の Prismatic_Bucket を伸縮させる。
        if key == ord('j'):
            _set_speed(self.bucket_joint, throttle)
            handled = True
        if key == ord('k'):
            _set_speed(self.bucket_joint, -throttle)
            handled = True

        # 前進: 左右スプロケットを同じ正方向へ回す。
        if key == Input.KEY_Up:
            _set_sprocket_speed(self.sprocket_joint_l, throttle)
            _set_sprocket_speed(self.sprocket_joint_r, throttle)
            handled = True
        # 後退: 左右スプロケットを同じ負方向へ回す。
        if key == Input.KEY_Down:
            _set_sprocket_speed(self.sprocket_joint_l, -throttle)
            _set_sprocket_speed(self.sprocket_joint_r, -throttle)
            handled = True
        # 左旋回: 左履帯を後退、右履帯を前進方向へ回す。
        if key == Input.KEY_Left:
            _set_sprocket_speed(self.sprocket_joint_l, -throttle)
            _set_sprocket_speed(self.sprocket_joint_r, throttle)
            handled = True
        # 右旋回: 左履帯を前進、右履帯を後退方向へ回す。
        if key == Input.KEY_Right:
            _set_sprocket_speed(self.sprocket_joint_l, throttle)
            _set_sprocket_speed(self.sprocket_joint_r, -throttle)
            handled = True

        return handled


def _setup_keyboard(excavator: agxSDK.Assembly, arm_hydraulic_circuit: HydraulisCircuit):
    # キーボードリスナをシミュレーションへ登録する。
    simulation().addEventListener(ExcavatorKeyboardControl(excavator, arm_hydraulic_circuit))


def _setup_gamepad(excavator: agxSDK.Assembly, arm_hydraulic_circuit: HydraulisCircuit):
    # GamepadCallback は AGX Python Modules 側のユーティリティ。
    # 接続されていない場合もキーボード操作は使えるため、警告だけ出して処理は続ける。
    try:
        gamepad = Gamepad.instance()
    except AttributeError:
        gamepad = None
    if gamepad is None:
        print("WARNING: Gamepad controls deactivated.")
        return

    # ゲームパッド操作対象の関節を取得する。
    slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
    boom_joint = excavator.getConstraint1DOF("Prismatic_Boom")
    bucket_joint = excavator.getConstraint1DOF("Prismatic_Bucket")
    sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
    sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    # 軸/ボタンごとのバインド名を作り、GamepadCallback へ登録するための小さなヘルパ。
    def bind_gamepad_axis(axis: Gamepad.Axis, callback):
        name = f"Axis.{axis.name}"
        Gamepad.bind(name=name, axis=axis, callback=callback)

    def bind_gamepad_button(button: Gamepad.Button, callback):
        name = f"Button.{button.name}"
        Gamepad.bind(name=name, button=button, callback=callback)

    # 左スティック左右: 旋回。入力方向とモデルの正方向を合わせるため符号を反転している。
    bind_gamepad_axis(Gamepad.Axis.LeftHorizontal,
                      lambda axis_data: _set_speed_by_gamepad(slew_joint, -axis_data.value, axis_data.delta))
    # 左スティック上下: アーム。
    bind_gamepad_axis(Gamepad.Axis.LeftVertical,
                      lambda axis_data: _set_hydraulic_by_gamepad(arm_hydraulic_circuit,
                                                                  axis_data.value,
                                                                  axis_data.delta))
    # 右スティック左右: バケット。入力方向とモデルの正方向を合わせるため符号を反転している。
    bind_gamepad_axis(Gamepad.Axis.RightHorizontal,
                      lambda axis_data: _set_speed_by_gamepad(bucket_joint, -axis_data.value, axis_data.delta))
    # 右スティック上下: ブーム。
    bind_gamepad_axis(Gamepad.Axis.RightVertical,
                      lambda axis_data: _set_speed_by_gamepad(boom_joint, axis_data.value, axis_data.delta))

    # LB: 左履帯を前進方向へ回す。
    bind_gamepad_button(Gamepad.Button.LeftBumper,
                        lambda button_data: _set_sprocket_speed_by_gamepad(sprocket_joint_l,
                                                                           1.0 if button_data.down else 0.0, 1.0))
    # RB: 右履帯を前進方向へ回す。
    bind_gamepad_button(Gamepad.Button.RightBumper,
                        lambda button_data: _set_sprocket_speed_by_gamepad(sprocket_joint_r,
                                                                           1.0 if button_data.down else 0.0, 1.0))
    # LT: 左履帯を後退方向へ回す。トリガーは軸入力なので押し込み量が速度比になる。
    bind_gamepad_axis(Gamepad.Axis.LeftTrigger,
                      lambda axis_data: _set_sprocket_speed_by_gamepad(sprocket_joint_l, -axis_data.value,
                                                                       axis_data.delta))
    # RT: 右履帯を後退方向へ回す。
    bind_gamepad_axis(Gamepad.Axis.RightTrigger,
                      lambda axis_data: _set_sprocket_speed_by_gamepad(sprocket_joint_r, -axis_data.value,
                                                                       axis_data.delta))


def setup_keyboard_gamepad_speed_control(excavator: agxSDK.Assembly):
    # キーボード/ゲームパッドで操作する全関節を取得する。
    slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
    boom_joint = excavator.getConstraint1DOF("Prismatic_Boom")
    arm_joint = excavator.getConstraint1DOF("Prismatic_Arm")
    bucket_joint = excavator.getConstraint1DOF("Prismatic_Bucket")
    sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
    sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    global HINGE_SLEW_MOTOR_TORQUE
    global HINGE_SLEW_BRAKE_TORQUE
    slew_joint_motor1d: agx.Motor1D = slew_joint.getMotor1D()

    # 旋回 Motor1D の元の力範囲を保存し、停止保持用に倍率をかけた力範囲も用意する。
    HINGE_SLEW_MOTOR_TORQUE = slew_joint_motor1d.getForceRange()
    HINGE_SLEW_BRAKE_TORQUE = agx.RangeReal(HINGE_SLEW_MOTOR_TORQUE.lower() * HINGE_SLEW_BRAKE_MULTIPLIER,
                                            HINGE_SLEW_MOTOR_TORQUE.upper() * HINGE_SLEW_BRAKE_MULTIPLIER)

    # Motor1D は有効化しないと速度指令を受けても動かない。
    # arm は油圧回路で駆動するため、Motor1D は有効化しない。
    for joint in [slew_joint, boom_joint, bucket_joint, sprocket_joint_r, sprocket_joint_l]:
        joint.getMotor1D().setEnable(True)

    arm_hydraulic_circuit = HydraulisCircuit(arm_joint.asPrismatic(), simulation(), CollectorParams())

    # 入力デバイスごとのイベント設定を登録する。
    _setup_keyboard(excavator, arm_hydraulic_circuit)
    _setup_gamepad(excavator, arm_hydraulic_circuit)
