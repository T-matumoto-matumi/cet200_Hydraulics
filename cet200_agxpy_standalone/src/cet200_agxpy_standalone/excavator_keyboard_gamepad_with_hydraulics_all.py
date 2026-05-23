# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
import agx
import agxSDK
import agxHydraulics
import agxPowerLine

from dataclasses import dataclass

from agxPythonModules.utils.callbacks import (
    KeyboardCallback as Input,
    GamepadCallback as Gamepad,
)
from agxPythonModules.utils.environment import simulation


@dataclass
class CylinderHydraulicParams:
    """1つのPrismaticに対応する油圧シリンダ設定。

    n_cylinders:
        実機で同じ作業機を並列に駆動しているシリンダ本数。
        AGXモデル側が1つのPrismaticで代表されている場合は、
        受圧面積を n_cylinders 倍して等価1本として扱う。
    """
    name: str
    fluid_density: float = 850.0

    pipe_diameter: float = 0.032
    pipe_length: float = 0.1

    relief_cracking_pressure: float = 35.0e6
    relief_fully_open_pressure: float = 37.3e6
    relief_diameter: float = 0.032

    piston_bore_diameter: float = 0.135
    piston_rod_diameter: float = 0.095
    piston_stroke: float = 1.49
    initial_position: float = 0.125

    n_cylinders: int = 1

    # この回路1つに与える最大流量 [m^3/s]
    # 注意: 現状は各シリンダが独立したConstantFlowValveを持つため、
    # 同時操作時は実機の総ポンプ流量439 L/minを超え得る。
    pump_flow_rate: float = 300.0 / 60000.0


class PC200HydraulicParams:
    """Komatsu PC200LC-8相当の作業機シリンダパラメータ。

    参考値:
      Boom   : 2本, 130 mm x 1334 mm x 90 mm
      Arm    : 1本, 135 mm x 1490 mm x 95 mm
      Bucket : 2本, 115 mm x 1120 mm x 80 mm
      Implement relief: 37.3 MPa
      Main pump max flow: 439 L/min

    AGXモデルでは各作業機が1つのPrismaticで表現されている前提で、
    BoomとBucketは n_cylinders=2 として等価面積にする。
    """

    @staticmethod
    def boom() -> CylinderHydraulicParams:
        return CylinderHydraulicParams(
            name="Boom",
            piston_bore_diameter=0.130,
            piston_rod_diameter=0.090,
            piston_stroke=1.334,
            initial_position=0.125,
            n_cylinders=2,
            pipe_diameter=0.035,
            pipe_length=0.1,
            relief_cracking_pressure=35.0e6,
            relief_fully_open_pressure=37.3e6,
            relief_diameter=0.035,
            # ブームは2本分の面積なので、同じ速度には大きめの流量が必要。
            # 独立回路モデルとしては350 L/min程度から開始。
            pump_flow_rate=350.0 / 60000.0,
        )

    @staticmethod
    def arm() -> CylinderHydraulicParams:
        return CylinderHydraulicParams(
            name="Arm",
            piston_bore_diameter=0.135,
            piston_rod_diameter=0.095,
            piston_stroke=1.490,
            initial_position=0.125,
            n_cylinders=1,
            pipe_diameter=0.032,
            pipe_length=0.1,
            relief_cracking_pressure=35.0e6,
            relief_fully_open_pressure=37.3e6,
            relief_diameter=0.032,
            pump_flow_rate=300.0 / 60000.0,
        )

    @staticmethod
    def bucket() -> CylinderHydraulicParams:
        return CylinderHydraulicParams(
            name="Bucket",
            piston_bore_diameter=0.115,
            piston_rod_diameter=0.080,
            piston_stroke=1.120,
            initial_position=0.100,
            n_cylinders=1,
            pipe_diameter=0.025,
            pipe_length=0.1,
            relief_cracking_pressure=35.0e6,
            relief_fully_open_pressure=37.3e6,
            relief_diameter=0.025,
            # バケットは小径なので300 L/minだとかなり速い。
            # まずは200 L/min程度に抑える。
            pump_flow_rate=200.0 / 60000.0,
        )


class FourWayThreePositionSpoolValve:
    """4ウェイ/3ポジションのスプールバルブ"""

    def __init__(
        self,
        in1: agxHydraulics.FlowUnit, in1Side,
        in2: agxHydraulics.FlowUnit, in2Side,
        out1: agxHydraulics.FlowUnit, out1Side,
        out2: agxHydraulics.FlowUnit, out2Side
    ):
        self.m_in1 = in1
        self.m_in2 = in2
        self.m_out1 = out1
        self.m_out2 = out2

        self.m_spool = agxHydraulics.SpoolValve()

        all_connected = all([
            self.m_spool.connect(agxPowerLine.INPUT, in1Side, in1),
            self.m_spool.connect(agxPowerLine.INPUT, in2Side, in2),
            self.m_spool.connect(agxPowerLine.OUTPUT, out1Side, out1),
            self.m_spool.connect(agxPowerLine.OUTPUT, out2Side, out2),
        ])

        assert all_connected, "SpoolValveの接続に失敗しました"
        self.linkNone()

    def linkNone(self):
        """ニュートラル: 全閉"""
        self.m_spool.unlink(self.m_in1)
        self.m_spool.unlink(self.m_in2)
        assert self.m_spool.getNumLinks() == 0

    def linkParallel(self):
        """平行接続: P->A, T->B"""
        self.linkNone()
        self.m_spool.link(self.m_in1, self.m_out1)
        self.m_spool.link(self.m_in2, self.m_out2)
        assert self.m_spool.getNumLinks() == 2

    def linkCross(self):
        """交差接続: P->B, T->A"""
        self.linkNone()
        self.m_spool.link(self.m_in1, self.m_out2)
        self.m_spool.link(self.m_in2, self.m_out1)
        assert self.m_spool.getNumLinks() == 2


class HydraulicCircuit:
    """1つのPrismaticを油圧シリンダで駆動する回路"""

    def __init__(
        self,
        joint: agx.Prismatic,
        sim: agxSDK.Simulation,
        params: CylinderHydraulicParams
    ):
        self.params = params

        OUTPUT = agxPowerLine.OUTPUT
        INPUT = agxPowerLine.INPUT

        pipe_area = agxHydraulics.diameterToArea(params.pipe_diameter)
        barrel_area_single = agxHydraulics.diameterToArea(params.piston_bore_diameter)
        rod_area_single = agxHydraulics.diameterToArea(params.piston_rod_diameter)
        annulus_area_single = barrel_area_single - rod_area_single

        # AGX上では1つのPrismaticに接続するので、実機で複数本ある場合は面積を合算する。
        barrel_area = barrel_area_single * params.n_cylinders
        annulus_area = annulus_area_single * params.n_cylinders

        self._ensure_joint_range(joint)
        joint.getMotor1D().setEnable(False)

        self.pump = agxHydraulics.ConstantFlowValve(
            params.pipe_length,
            pipe_area,
            params.fluid_density,
            0.0
        )
        self.pump.setName(f"{params.name}_constant_flow_pump")
        self.pump.setAllowPumping(True)
        self.pump.setEnable(True)

        self.p_1 = self.create_pipe(pipe_area, params.fluid_density, params.pipe_length)
        self.p_2 = self.create_pipe(pipe_area, params.fluid_density, params.pipe_length)
        self.p_3 = self.create_pipe(pipe_area, params.fluid_density, params.pipe_length)
        self.p_4 = self.create_pipe(pipe_area, params.fluid_density, params.pipe_length)
        self.p_5 = self.create_pipe(pipe_area, params.fluid_density, params.pipe_length)

        for i, p in enumerate([self.p_1, self.p_2, self.p_3, self.p_4, self.p_5], start=1):
            p.setName(f"{params.name}_pipe_{i}")

        self.relief_valve = agxHydraulics.ReliefValve(
            params.relief_cracking_pressure,
            params.relief_fully_open_pressure,
            agxHydraulics.diameterToArea(params.relief_diameter)
        )
        self.relief_valve.setName(f"{params.name}_relief_valve")

        self.piston = agxHydraulics.PistonActuator(
            joint,
            barrel_area,
            annulus_area,
            params.fluid_density
        )
        self.piston.setName(f"{params.name}_piston_actuator")

        # p_1: P line, p_4: Tank line, p_2: A line, p_3: B line
        self.spool = FourWayThreePositionSpoolValve(
            self.p_1, OUTPUT,
            self.p_4, INPUT,
            self.p_2, INPUT,
            self.p_3, INPUT,
        )

        self.pump.connect(self.p_5)
        self.p_5.connect(self.relief_valve)
        self.relief_valve.connect(self.p_1)

        # A line -> piston input chamber
        self.p_2.connect(self.piston)

        # piston output chamber -> B line
        self.piston.connect(OUTPUT, OUTPUT, self.p_3)

        self._setup_connection(sim)

        print(
            f"[HydraulicCircuit] {params.name}: "
            f"bore={params.piston_bore_diameter*1000:.0f} mm, "
            f"rod={params.piston_rod_diameter*1000:.0f} mm, "
            f"stroke={params.piston_stroke:.3f} m, "
            f"n={params.n_cylinders}, "
            f"Qmax={params.pump_flow_rate*60000:.1f} L/min"
        )

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

    def create_pipe(self, area, fluid_density, length=1.0):
        return agxHydraulics.Pipe(length, area, fluid_density)

    def _setup_connection(self, sim: agxSDK.Simulation):
        self.powerline = agxPowerLine.PowerLine()
        self.powerline.setName(f"{self.params.name}_hydraulic_powerline")
        sim.add(self.powerline)
        self.powerline.add(self.pump)

    def control_input(self, u):
        if abs(u) <= 0.0:
            self.pump.setTargetFlowRate(0.0)
            self.spool.linkNone()
            return

        self.pump.setTargetFlowRate(abs(u) * self.params.pump_flow_rate)

        if u > 0.0:
            self.spool.linkParallel()
        else:
            self.spool.linkCross()


# 後方互換: 既存コードで HydraulisCircuit と書いていても動くようにする
HydraulisCircuit = HydraulicCircuit
CollectorParams = CylinderHydraulicParams


# 各関節に与える最大速度。
# 油圧化した作業機は Motor1D では駆動しない。
max_speeds = {
    'Hinge_Slew': 1.256637,
    'Hinge_Sprocket': 4.074104494,
}

HINGE_SLEW_BRAKE_MULTIPLIER = 4
HINGE_SLEW_MOTOR_TORQUE: agx.RangeReal
HINGE_SLEW_BRAKE_TORQUE: agx.RangeReal


def _set_speed(joint: agx.Constraint1DOF, throttle: float):
    if joint.getName() in ["Prismatic_Boom", "Prismatic_Arm", "Prismatic_Bucket"]:
        raise RuntimeError(f"{joint.getName()} is driven by hydraulics, not Motor1D.")

    target_speed = throttle * max_speeds[joint.getName()]
    motor1d: agx.Motor1D = joint.getMotor1D()
    motor1d.setSpeed(target_speed)

    if joint.getName() == "Hinge_Slew":
        motor1d.setForceRange(HINGE_SLEW_MOTOR_TORQUE)
        if target_speed == 0.0:
            motor1d.setForceRange(HINGE_SLEW_BRAKE_TORQUE)


def _set_sprocket_speed(joint: agx.Hinge, throttle: float):
    speed = throttle * max_speeds['Hinge_Sprocket']
    joint.getMotor1D().setSpeed(speed)


def handle_stick_dead_zone(raw_value):
    dead_zone = 0.3
    abs_raw_value = abs(raw_value)
    if abs_raw_value <= dead_zone:
        return 0
    sign = 1.0 if raw_value > 0.0 else -1.0
    return sign * (abs_raw_value - dead_zone) / (1 - dead_zone)


def is_stick_moved(value, delta_value):
    if abs(value) > 0.0:
        return True
    if abs(delta_value) > 0.0:
        return True
    return False


def _set_speed_by_gamepad(joint: agx.Constraint1DOF, throttle: float, delta_throttle):
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    _set_speed(joint, throttle)


def _set_hydraulic_by_gamepad(circuit: HydraulicCircuit, throttle: float, delta_throttle):
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    circuit.control_input(throttle)


def _set_sprocket_speed_by_gamepad(joint: agx.Hinge, throttle: float, delta_throttle: float):
    throttle = handle_stick_dead_zone(throttle)
    if not is_stick_moved(throttle, delta_throttle):
        return
    _set_sprocket_speed(joint, throttle)


class ExcavatorKeyboardControl(agxSDK.GuiEventListener):
    def __init__(self, excavator, hydraulic_circuits: dict):
        super().__init__(agxSDK.GuiEventListener.KEYBOARD)

        self.slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
        self.sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
        self.sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

        self.boom_hydraulic_circuit = hydraulic_circuits["Prismatic_Boom"]
        self.arm_hydraulic_circuit = hydraulic_circuits["Prismatic_Arm"]
        self.bucket_hydraulic_circuit = hydraulic_circuits["Prismatic_Bucket"]

    def keyboard(self, key, x, y, alt, down):
        handled = False
        throttle = 0.6 if down else 0

        # 旋回
        if key == ord('a'):
            _set_speed(self.slew_joint, throttle)
            handled = True
        if key == ord('s'):
            _set_speed(self.slew_joint, -throttle)
            handled = True

        # アーム
        if key == ord('z'):
            self.arm_hydraulic_circuit.control_input(-throttle)
            handled = True
        if key == ord('x'):
            self.arm_hydraulic_circuit.control_input(throttle)
            handled = True

        # ブーム
        if key == ord('m'):
            self.boom_hydraulic_circuit.control_input(throttle)
            handled = True
        if key == ord(','):
            self.boom_hydraulic_circuit.control_input(-throttle)
            handled = True

        # バケット
        if key == ord('j'):
            self.bucket_hydraulic_circuit.control_input(throttle)
            handled = True
        if key == ord('k'):
            self.bucket_hydraulic_circuit.control_input(-throttle)
            handled = True

        # 履帯
        if key == Input.KEY_Up:
            _set_sprocket_speed(self.sprocket_joint_l, throttle)
            _set_sprocket_speed(self.sprocket_joint_r, throttle)
            handled = True
        if key == Input.KEY_Down:
            _set_sprocket_speed(self.sprocket_joint_l, -throttle)
            _set_sprocket_speed(self.sprocket_joint_r, -throttle)
            handled = True
        if key == Input.KEY_Left:
            _set_sprocket_speed(self.sprocket_joint_l, -throttle)
            _set_sprocket_speed(self.sprocket_joint_r, throttle)
            handled = True
        if key == Input.KEY_Right:
            _set_sprocket_speed(self.sprocket_joint_l, throttle)
            _set_sprocket_speed(self.sprocket_joint_r, -throttle)
            handled = True

        return handled


def _setup_keyboard(excavator: agxSDK.Assembly, hydraulic_circuits: dict):
    simulation().addEventListener(ExcavatorKeyboardControl(excavator, hydraulic_circuits))


def _setup_gamepad(excavator: agxSDK.Assembly, hydraulic_circuits: dict):
    try:
        gamepad = Gamepad.instance()
    except AttributeError:
        gamepad = None
    if gamepad is None:
        print("WARNING: Gamepad controls deactivated.")
        return

    slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
    sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
    sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    boom_circuit = hydraulic_circuits["Prismatic_Boom"]
    arm_circuit = hydraulic_circuits["Prismatic_Arm"]
    bucket_circuit = hydraulic_circuits["Prismatic_Bucket"]

    def bind_gamepad_axis(axis: Gamepad.Axis, callback):
        name = f"Axis.{axis.name}"
        Gamepad.bind(name=name, axis=axis, callback=callback)

    def bind_gamepad_button(button: Gamepad.Button, callback):
        name = f"Button.{button.name}"
        Gamepad.bind(name=name, button=button, callback=callback)

    # 左スティック左右: 旋回
    bind_gamepad_axis(
        Gamepad.Axis.LeftHorizontal,
        lambda axis_data: _set_speed_by_gamepad(slew_joint, -axis_data.value, axis_data.delta)
    )

    # 左スティック上下: アーム
    bind_gamepad_axis(
        Gamepad.Axis.LeftVertical,
        lambda axis_data: _set_hydraulic_by_gamepad(
            arm_circuit,
            axis_data.value,
            axis_data.delta
        )
    )

    # 右スティック左右: バケット
    bind_gamepad_axis(
        Gamepad.Axis.RightHorizontal,
        lambda axis_data: _set_hydraulic_by_gamepad(
            bucket_circuit,
            -axis_data.value,
            axis_data.delta
        )
    )

    # 右スティック上下: ブーム
    bind_gamepad_axis(
        Gamepad.Axis.RightVertical,
        lambda axis_data: _set_hydraulic_by_gamepad(
            boom_circuit,
            axis_data.value,
            axis_data.delta
        )
    )

    # LB/RB/LT/RT: 履帯
    bind_gamepad_button(
        Gamepad.Button.LeftBumper,
        lambda button_data: _set_sprocket_speed_by_gamepad(
            sprocket_joint_l,
            1.0 if button_data.down else 0.0,
            1.0
        )
    )

    bind_gamepad_button(
        Gamepad.Button.RightBumper,
        lambda button_data: _set_sprocket_speed_by_gamepad(
            sprocket_joint_r,
            1.0 if button_data.down else 0.0,
            1.0
        )
    )

    bind_gamepad_axis(
        Gamepad.Axis.LeftTrigger,
        lambda axis_data: _set_sprocket_speed_by_gamepad(
            sprocket_joint_l,
            -axis_data.value,
            axis_data.delta
        )
    )

    bind_gamepad_axis(
        Gamepad.Axis.RightTrigger,
        lambda axis_data: _set_sprocket_speed_by_gamepad(
            sprocket_joint_r,
            -axis_data.value,
            axis_data.delta
        )
    )


def setup_keyboard_gamepad_speed_control(excavator: agxSDK.Assembly):
    slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
    boom_joint = excavator.getConstraint1DOF("Prismatic_Boom")
    arm_joint = excavator.getConstraint1DOF("Prismatic_Arm")
    bucket_joint = excavator.getConstraint1DOF("Prismatic_Bucket")
    sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
    sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    global HINGE_SLEW_MOTOR_TORQUE
    global HINGE_SLEW_BRAKE_TORQUE

    slew_joint_motor1d: agx.Motor1D = slew_joint.getMotor1D()
    HINGE_SLEW_MOTOR_TORQUE = slew_joint_motor1d.getForceRange()
    HINGE_SLEW_BRAKE_TORQUE = agx.RangeReal(
        HINGE_SLEW_MOTOR_TORQUE.lower() * HINGE_SLEW_BRAKE_MULTIPLIER,
        HINGE_SLEW_MOTOR_TORQUE.upper() * HINGE_SLEW_BRAKE_MULTIPLIER
    )

    # 作業機3軸は油圧化するのでMotor1Dは無効。
    for joint in [boom_joint, arm_joint, bucket_joint]:
        joint.getMotor1D().setEnable(False)

    # 旋回と履帯はMotor1Dのまま。
    for joint in [slew_joint, sprocket_joint_r, sprocket_joint_l]:
        joint.getMotor1D().setEnable(True)

    hydraulic_circuits = {
        "Prismatic_Boom": HydraulicCircuit(
            boom_joint.asPrismatic(),
            simulation(),
            PC200HydraulicParams.boom()
        ),
        "Prismatic_Arm": HydraulicCircuit(
            arm_joint.asPrismatic(),
            simulation(),
            PC200HydraulicParams.arm()
        ),
        "Prismatic_Bucket": HydraulicCircuit(
            bucket_joint.asPrismatic(),
            simulation(),
            PC200HydraulicParams.bucket()
        ),
    }

    _setup_keyboard(excavator, hydraulic_circuits)
    _setup_gamepad(excavator, hydraulic_circuits)

    return hydraulic_circuits
