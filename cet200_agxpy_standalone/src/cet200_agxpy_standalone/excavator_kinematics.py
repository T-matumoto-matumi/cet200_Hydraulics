# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

"""CET200 作業機の順運動学・逆運動学を扱う純 Python モジュール。

AGX モデルではブーム・アーム・バケットを ``Prismatic_*`` 拘束で駆動するため、
公開 API はシリンダ変位を中心にしている。内部では
「シリンダ変位 <-> ヒンジ角」と「ヒンジ角 <-> 刃先姿勢」を分けて計算する。
この分け方にしておくと、CAD 寸法、AGX の関節角、刃先 ObserverFrame を
それぞれ個別に照合しやすい。

基本的な使い方:

    cylinders = CylinderDisplacements(slew=0.0, boom=0.0, arm=0.0, bucket=0.0)
    pose = forward_kinematics_from_cylinders(cylinders)

    target = pose.position
    solved = inverse_kinematics_to_cylinders(target, pose.pitch, slew=pose.yaw)
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CylinderDisplacements:
    """AGX の ``Prismatic_*`` 変位 [m] と旋回角 [rad]。

    ``boom``、``arm``、``bucket`` は絶対シリンダ長ではなく、AGX 拘束の
    初期位置を 0 とした伸縮量として扱う。
    """

    slew: float
    boom: float
    arm: float
    bucket: float


@dataclass(frozen=True)
class JointAngles:
    """作業機の主ヒンジ角 [rad]。"""

    slew: float
    boom: float
    arm: float
    bucket: float


@dataclass(frozen=True)
class TipPose:
    """バケット刃先中央の姿勢。

    ``position`` は base/TF_Origin 座標系での 3D 位置 [m]。
    ``yaw`` は旋回角、``pitch`` はブーム・アーム・バケット角の合計。
    """

    position: Vector3
    yaw: float
    pitch: float


@dataclass(frozen=True)
class CylinderGeometry:
    """2 ピン式シリンダの平面幾何。

    ``parent_pin`` は親リンク側ピン、``child_pin`` は子リンク側ピン。
    どちらもヒンジ角 0 の姿勢で、各リンクの基準座標に対する x-z 平面座標。
    """

    parent_pin: Vector2
    child_pin: Vector2
    initial_length: float
    min_displacement: float
    max_displacement: float


@dataclass(frozen=True)
class BucketLinkageGeometry:
    """バケット用 4 節リンク機構の平面幾何。

    各点はヒンジ角 0 の姿勢で、アームまたはバケットの基準座標に対する
    x-z 平面座標として表す。``joint_branch`` は円同士の交点が 2 つあるとき、
    CAD/AGX の組み付けに合う側を選ぶための符号。
    """

    cylinder_base_on_arm: Vector2
    ilink_arm_pin: Vector2
    bucket_pivot_on_arm: Vector2
    hlink_bucket_pin: Vector2
    bucket_cylinder_rod_pin: Vector2
    ilink_length: float
    hlink_length: float
    initial_length: float
    min_displacement: float
    max_displacement: float
    joint_branch: int = 1


@dataclass(frozen=True)
class ExcavatorDimensions:
    """運動学計算に必要な寸法セット。CAD 寸法はここへ集約する。"""

    slew_origin: Vector3
    boom_pivot: Vector2
    boom_to_arm: Vector2
    arm_to_bucket: Vector2
    bucket_to_tip: Vector2
    boom_cylinder: CylinderGeometry
    arm_cylinder: CylinderGeometry
    bucket_linkage: BucketLinkageGeometry
    boom_angle_limits: Vector2
    arm_angle_limits: Vector2
    bucket_angle_limits: Vector2


ELBOW_DOWN = "elbow_down"
ELBOW_UP = "elbow_up"


# 現在の既定値は AGX 初期姿勢から測った仮寸法。
# CAD から最終寸法を取得したら、この DEFAULT_DIMENSIONS を置き換える。
DEFAULT_DIMENSIONS = ExcavatorDimensions(
    slew_origin=(0.0, 0.0, 0.0),
    boom_pivot=(0.180000, 1.829998),
    boom_to_arm=(4.713966, 3.194455),
    arm_to_bucket=(-0.000596, -2.900000),
    bucket_to_tip=(-0.684132, -1.116041),
    boom_cylinder=CylinderGeometry(
        parent_pin=(0.697430, -0.625301),
        child_pin=(1.396308, 2.164142),
        initial_length=2.8756603953062676,
        min_displacement=-0.9956592,
        max_displacement=0.3243408,
    ),
    arm_cylinder=CylinderGeometry(
        parent_pin=(-3.123586, -0.285432),
        child_pin=(0.064728, 0.704660),
        initial_length=3.338506901454601,
        min_displacement=-0.853,
        max_displacement=0.3514924,
    ),
    bucket_linkage=BucketLinkageGeometry(
        cylinder_base_on_arm=(0.557406, -0.263705),
        ilink_arm_pin=(-0.000507, -2.470000),
        bucket_pivot_on_arm=(-0.000596, -2.900000),
        hlink_bucket_pin=(0.423647, -0.203528),
        bucket_cylinder_rod_pin=(0.613541, -2.555701),
        ilink_length=0.620000,
        hlink_length=0.580000,
        initial_length=2.292684180149734,
        min_displacement=-0.55847,
        max_displacement=0.5,
        joint_branch=1,
    ),
    boom_angle_limits=(-0.443, 1.127),
    arm_angle_limits=(-1.40, 0.648),
    bucket_angle_limits=(-1.236, 1.77),
)


def forward_kinematics_from_joints(
    angles: JointAngles,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
) -> TipPose:
    """ヒンジ角から刃先中央の姿勢を求める順運動学。

    シリンダ機構は考慮せず、``JointAngles`` をそのままリンク角として使う。
    CAD のリンク寸法が正しいか確認する最初の関数として使う。
    """

    pitch = angles.boom + angles.arm + angles.bucket
    planar = dimensions.boom_pivot
    planar = _add2(planar, _rotate2(dimensions.boom_to_arm, angles.boom))
    planar = _add2(planar, _rotate2(dimensions.arm_to_bucket, angles.boom + angles.arm))
    planar = _add2(planar, _rotate2(dimensions.bucket_to_tip, pitch))

    x, y = _rotate_xy(planar[0], 0.0, angles.slew)
    return TipPose(
        position=(
            dimensions.slew_origin[0] + x,
            dimensions.slew_origin[1] + y,
            dimensions.slew_origin[2] + planar[1],
        ),
        yaw=angles.slew,
        pitch=pitch,
    )


def forward_kinematics_from_cylinders(
    cylinders: CylinderDisplacements,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
) -> TipPose:
    """AGX のシリンダ変位から刃先中央の姿勢を求める順運動学。

    内部では ``cylinders_to_joint_angles`` でヒンジ角へ変換してから、
    ``forward_kinematics_from_joints`` を呼ぶ。
    """

    return forward_kinematics_from_joints(
        cylinders_to_joint_angles(cylinders, dimensions),
        dimensions,
    )


def cylinders_to_joint_angles(
    cylinders: CylinderDisplacements,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
) -> JointAngles:
    """AGX のシリンダ変位を主ヒンジ角へ変換する。

    ブーム・アームは 2 ピンの三角形、バケットは I-Link/H-Link を含む
    4 節リンクとして解く。
    """

    _check_displacement("boom", cylinders.boom, dimensions.boom_cylinder)
    _check_displacement("arm", cylinders.arm, dimensions.arm_cylinder)
    _check_displacement("bucket", cylinders.bucket, dimensions.bucket_linkage)

    boom = _solve_angle_for_length(
        lambda angle: _two_pin_length(dimensions.boom_cylinder.parent_pin, dimensions.boom_cylinder.child_pin, angle),
        dimensions.boom_cylinder.initial_length + cylinders.boom,
        dimensions.boom_angle_limits,
        initial=0.0,
    )
    arm = _solve_angle_for_length(
        lambda angle: _two_pin_length(dimensions.arm_cylinder.parent_pin, dimensions.arm_cylinder.child_pin, angle),
        dimensions.arm_cylinder.initial_length + cylinders.arm,
        dimensions.arm_angle_limits,
        initial=0.0,
    )
    bucket = _solve_angle_for_length(
        lambda angle: _bucket_cylinder_length(angle, dimensions.bucket_linkage),
        dimensions.bucket_linkage.initial_length + cylinders.bucket,
        dimensions.bucket_angle_limits,
        initial=0.0,
    )
    return JointAngles(slew=cylinders.slew, boom=boom, arm=arm, bucket=bucket)


def joint_angles_to_cylinders(
    angles: JointAngles,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
) -> CylinderDisplacements:
    """主ヒンジ角を AGX のシリンダ変位へ変換する。"""

    _check_angle("boom", angles.boom, dimensions.boom_angle_limits)
    _check_angle("arm", angles.arm, dimensions.arm_angle_limits)
    _check_angle("bucket", angles.bucket, dimensions.bucket_angle_limits)

    boom = (
        _two_pin_length(dimensions.boom_cylinder.parent_pin, dimensions.boom_cylinder.child_pin, angles.boom)
        - dimensions.boom_cylinder.initial_length
    )
    arm = (
        _two_pin_length(dimensions.arm_cylinder.parent_pin, dimensions.arm_cylinder.child_pin, angles.arm)
        - dimensions.arm_cylinder.initial_length
    )
    bucket = _bucket_cylinder_length(angles.bucket, dimensions.bucket_linkage) - dimensions.bucket_linkage.initial_length

    return CylinderDisplacements(slew=angles.slew, boom=boom, arm=arm, bucket=bucket)


def inverse_kinematics_to_joint_angles(
    position: Vector3,
    pitch: float,
    slew: Optional[float] = None,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
    prefer: str = ELBOW_DOWN,
) -> JointAngles:
    """目標刃先姿勢を満たす主ヒンジ角を求める逆運動学。

    ``position`` は base/TF_Origin 座標系の刃先中央位置 [m]。
    ``pitch`` はバケット姿勢角で、ブーム・アーム・バケット角の合計。
    ``slew`` を省略した場合は、目標位置の x-y 方向から旋回角を推定する。
    """

    if prefer not in (ELBOW_DOWN, ELBOW_UP):
        raise ValueError("prefer は 'elbow_down' または 'elbow_up' を指定してください")

    if slew is None:
        slew = math.atan2(position[1] - dimensions.slew_origin[1], position[0] - dimensions.slew_origin[0])

    dx = position[0] - dimensions.slew_origin[0]
    dy = position[1] - dimensions.slew_origin[1]
    local_x, _local_y = _rotate_xy(dx, dy, -slew)
    local_z = position[2] - dimensions.slew_origin[2]

    tip_offset = _rotate2(dimensions.bucket_to_tip, pitch)
    wrist = (
        local_x - dimensions.boom_pivot[0] - tip_offset[0],
        local_z - dimensions.boom_pivot[1] - tip_offset[1],
    )

    boom, arm = _solve_two_link(dimensions.boom_to_arm, dimensions.arm_to_bucket, wrist, prefer)
    bucket = _normalize_angle(pitch - boom - arm)
    result = JointAngles(slew=slew, boom=boom, arm=arm, bucket=bucket)
    _check_angle("boom", result.boom, dimensions.boom_angle_limits)
    _check_angle("arm", result.arm, dimensions.arm_angle_limits)
    _check_angle("bucket", result.bucket, dimensions.bucket_angle_limits)
    return result


def inverse_kinematics_to_cylinders(
    position: Vector3,
    pitch: float,
    slew: Optional[float] = None,
    dimensions: ExcavatorDimensions = DEFAULT_DIMENSIONS,
    prefer: str = ELBOW_DOWN,
) -> CylinderDisplacements:
    """目標刃先姿勢を満たす AGX シリンダ変位を求める逆運動学。"""

    return joint_angles_to_cylinders(
        inverse_kinematics_to_joint_angles(position, pitch, slew=slew, dimensions=dimensions, prefer=prefer),
        dimensions,
    )


def _two_pin_length(parent_pin: Vector2, child_pin: Vector2, child_angle: float) -> float:
    return _norm2(_sub2(_rotate2(child_pin, child_angle), parent_pin))


def _bucket_cylinder_length(bucket_angle: float, geometry: BucketLinkageGeometry) -> float:
    rod_pin = _bucket_rod_pin(bucket_angle, geometry)
    return _norm2(_sub2(rod_pin, geometry.cylinder_base_on_arm))


def _bucket_rod_pin(bucket_angle: float, geometry: BucketLinkageGeometry) -> Vector2:
    hlink_pin = _add2(geometry.bucket_pivot_on_arm, _rotate2(geometry.hlink_bucket_pin, bucket_angle))
    return _circle_intersection(
        geometry.ilink_arm_pin,
        geometry.ilink_length,
        hlink_pin,
        geometry.hlink_length,
        geometry.joint_branch,
    )


def _solve_two_link(link1: Vector2, link2: Vector2, target: Vector2, prefer: str) -> Tuple[float, float]:
    l1 = _norm2(link1)
    l2 = _norm2(link2)
    distance = _norm2(target)
    if distance > l1 + l2 + 1e-9 or distance < abs(l1 - l2) - 1e-9:
        raise ValueError("目標位置が作業機の到達可能範囲外です")

    dot_base = link1[0] * link2[0] + link1[1] * link2[1]
    cross_base = link1[0] * link2[1] - link1[1] * link2[0]
    radius = math.hypot(dot_base, cross_base)
    rhs = (distance * distance - l1 * l1 - l2 * l2) / (2.0 * radius)
    if rhs > 1.0 + 1e-9 or rhs < -1.0 - 1e-9:
        raise ValueError("目標位置が作業機の到達可能範囲外です")
    rhs = _clamp(rhs, -1.0, 1.0)

    phase = math.atan2(cross_base, dot_base)
    delta = math.acos(rhs)
    candidates = (phase + delta, phase - delta)
    if prefer == ELBOW_UP:
        candidates = (candidates[1], candidates[0])

    for arm in candidates:
        combined = _add2(link1, _rotate2(link2, arm))
        boom = _normalize_angle(_angle2(combined) - _angle2(target))
        error = _norm2(_sub2(_add2(_rotate2(link1, boom), _rotate2(link2, boom + arm)), target))
        if error <= 1e-7:
            return (_normalize_angle(boom), _normalize_angle(arm))

    raise ValueError("2 リンクの運動学を解けませんでした")


def _solve_angle_for_length(func, target_length: float, limits: Vector2, initial: float) -> float:
    lower, upper = limits
    samples = 240
    brackets = []
    last_angle = lower
    last_value = func(last_angle) - target_length
    for index in range(1, samples + 1):
        angle = lower + (upper - lower) * index / samples
        value = func(angle) - target_length
        if abs(value) < 1e-9:
            brackets.append((angle, angle))
        elif last_value * value < 0.0:
            brackets.append((last_angle, angle))
        last_angle = angle
        last_value = value

    roots = []
    for left, right in brackets:
        if left == right:
            roots.append(left)
            continue
        f_left = func(left) - target_length
        for _ in range(80):
            middle = 0.5 * (left + right)
            f_middle = func(middle) - target_length
            if abs(f_middle) < 1e-12:
                left = right = middle
                break
            if f_left * f_middle <= 0.0:
                right = middle
            else:
                left = middle
                f_left = f_middle
        roots.append(0.5 * (left + right))

    if not roots:
        raise ValueError(f"目標シリンダ長 {target_length:.9f} はリンク機構の到達範囲外です")

    return min(roots, key=lambda angle: abs(_normalize_angle(angle - initial)))


def _circle_intersection(center1: Vector2, radius1: float, center2: Vector2, radius2: float, branch: int) -> Vector2:
    delta = _sub2(center2, center1)
    distance = _norm2(delta)
    if distance <= 0.0:
        raise ValueError("リンク機構の円中心が一致しているため交点を計算できません")
    if distance > radius1 + radius2 + 1e-9 or distance < abs(radius1 - radius2) - 1e-9:
        raise ValueError("バケットリンク機構が到達可能範囲外です")

    a = (radius1 * radius1 - radius2 * radius2 + distance * distance) / (2.0 * distance)
    h_sq = radius1 * radius1 - a * a
    if h_sq < -1e-9:
        raise ValueError("バケットリンク機構が到達可能範囲外です")
    h = math.sqrt(max(0.0, h_sq))
    unit = (delta[0] / distance, delta[1] / distance)
    base = (center1[0] + a * unit[0], center1[1] + a * unit[1])
    normal = (-unit[1], unit[0])
    sign = 1.0 if branch >= 0 else -1.0
    return (base[0] + sign * h * normal[0], base[1] + sign * h * normal[1])


def _check_displacement(name: str, displacement: float, geometry) -> None:
    if displacement < geometry.min_displacement - 1e-9 or displacement > geometry.max_displacement + 1e-9:
        raise ValueError(f"{name} シリンダ変位が AGX の可動範囲外です")


def _check_angle(name: str, angle: float, limits: Vector2) -> None:
    if angle < limits[0] - 1e-9 or angle > limits[1] + 1e-9:
        raise ValueError(f"{name} ヒンジ角が設定された可動範囲外です")


def _rotate2(vector: Vector2, angle: float) -> Vector2:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * vector[0] + s * vector[1], -s * vector[0] + c * vector[1])


def _rotate_xy(x: float, y: float, angle: float) -> Vector2:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * y, s * x + c * y)


def _add2(left: Vector2, right: Vector2) -> Vector2:
    return (left[0] + right[0], left[1] + right[1])


def _sub2(left: Vector2, right: Vector2) -> Vector2:
    return (left[0] - right[0], left[1] - right[1])


def _norm2(vector: Vector2) -> float:
    return math.hypot(vector[0], vector[1])


def _angle2(vector: Vector2) -> float:
    return math.atan2(vector[1], vector[0])


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
