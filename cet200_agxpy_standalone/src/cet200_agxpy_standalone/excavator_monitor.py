# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
from typing import Optional, Any

import agx
import agxPython
import agxCollide
import agxIO
import agxOSG
import agxSDK
import agxTerrain
import agxUtil
import osg
import agxVehicle
from agx import AGX_GET_VERSION, agxGetPythonVersion, agxGetVersion, AGX_CONVERT_VERSION, AGX_CALC_VERSION

from agxPythonModules.utils.callbacks import (
    StepEventCallback,
    KeyboardCallback as Input,
    GamepadCallback as Gamepad,
    ContactEventCallback as ContactEvent,
)

# Python imports
import sys

from agxPythonModules.utils.environment import application, simulation


def setup_excavator_monitor(excavator: agxSDK.Assembly):
    # AGX Viewer の画面上にテキストを描画するための SceneDecorator を取得する。
    # フォントサイズはワールド座標系に近いスケールなので、このサンプルでは小さめに設定している。
    sd: agxOSG.SceneDecorator = application().getSceneDecorator()
    sd.setFontSize(0.01)

    # モニタ表示に使う 1 自由度拘束を excavator Assembly から名前で取得する。
    # Prismatic_* は油圧シリンダ相当、Hinge_* は旋回・履帯スプロケット・リンク関節相当。
    joints: list[agx.Constraint1DOF] = list()
    slew_joint = excavator.getConstraint1DOF("Hinge_Slew")
    boom_joint = excavator.getConstraint1DOF("Prismatic_Boom")
    arm_joint = excavator.getConstraint1DOF("Prismatic_Arm")
    bucket_joint = excavator.getConstraint1DOF("Prismatic_Bucket")
    sprocket_joint_l = excavator.getConstraint1DOF("Hinge_Sprocket_L")
    sprocket_joint_r = excavator.getConstraint1DOF("Hinge_Sprocket_R")

    hinge_boom = excavator.getConstraint1DOF("Hinge_Boom")
    hinge_arm = excavator.getConstraint1DOF("Hinge_Arm")
    hinge_bucket = excavator.getConstraint1DOF("Hinge_Bucket")

    # Terrain がある場合は、シミュレーション内の Shovel を取得する。
    # Shovel はバケットと地形の相互作用を AGX Terrain に知らせるためのオブジェクトで、
    # 掘削抵抗やバケット内土量の取得にも使われる。
    terrain = simulation().getTerrain(0)
    shovel = None
    if terrain:
        shovels = agxTerrain.Shovel.findAll(simulation())
        shovel = shovels[0] if len(shovels) > 0 else None

    # 表示用の補助関数。
    # 浮動小数点の微小な揺れを 0 として扱うことで、停止中の値を読みやすくする。
    def clamp_to_zero(value: float, threshold=1e-6):
        return value if abs(value) > threshold else 0

    def get_penetration_force() -> agx.Vec3:
        # バケット刃先が土に入り込むときの貫入抵抗。
        # AGX API は force と torque を参照引数に書き込むため、Vec3 を用意してから呼び出す。
        penetration_force = agx.Vec3()
        penetration_torque = agx.Vec3()
        shovel.getPenetrationForce(penetration_force, penetration_torque)
        return penetration_force

    def get_separation_force():
        # 土塊をバケットから分離・持ち上げるときに発生する接触力。
        return shovel.getSeparationContactForce()

    def get_contact_force():
        # Shovel と Terrain の通常接触による力。
        return shovel.getContactForce()

    def get_deformation_contact_force():
        # 地形変形を伴う接触で発生する力。
        return shovel.getDeformationContactForce()

    def get_bucket_soil_volume() -> float:
        # AGX 2.40.1.5 より新しい版では getInnerSoilBulkVolume が利用できる。
        # 古い版では互換性のため getSoilVolume を使う。
        agx_version = AGX_GET_VERSION()
        if agx_version > AGX_CALC_VERSION(2, 40, 1, 5):
            return shovel.getInnerSoilBulkVolume()
        else:
            return shovel.getSoilVolume()

    def get_bucket_soil_mass() -> float:
        # バケット内に保持されている動的な土の質量。
        return shovel.getDynamicMass()

    # 物理ステップ後に毎回呼ばれる表示更新コールバック。
    # time_stamp は AGX コールバックから渡される現在時刻だが、
    # ここでは simulation().getTimeStamp() から表示用の時刻を取得している。
    def update_monitor(time_stamp):
        # 文字列の桁幅を固定して、値が変化しても表の列が揃うようにする。
        width1 = 30
        width2 = 10
        width3 = 13

        text_list: list[str] = list()

        # 操作キーと経過時間を先頭に表示する。
        str_keyboard = "Key: {Slew: a, s, Arm: z, x, Bucket: j, k, Boom: m, ','}"
        str_elapsed_time = f"Elapsed time: {simulation().getTimeStamp():>.2f}"

        text_list.append(str_keyboard)
        text_list.append(str_elapsed_time)
        text_list.append(f"{'Joint':>{width1}}: {'Angle':>{width2}} {'Speed':>{width2}} {'Force(k)':>{width3}}")

        def add_joint_to_text_list(_joint):
            # モデルや AGX ファイルの構成が変わって対象関節が無い場合は、その行だけ表示しない。
            if _joint is None:
                return

            # Angle は関節位置、Speed は現在速度、Force は Motor1D が発生している力・トルク。
            # Prismatic では力 [N]、Hinge ではトルク [N m] 相当の値だが、
            # 表示では 1e-3 倍して k 単位として読みやすくしている。
            name = _joint.getName()
            angle = clamp_to_zero(_joint.getAngle())
            speed = clamp_to_zero(_joint.getCurrentSpeed())
            motor_force = clamp_to_zero(_joint.getMotor1D().getCurrentForce(), 100)
            motor_force = _joint.getMotor1D().getCurrentForce() * 1e-3
            text_list.append(f"{name:>{width1}}: {angle:>{width2}.6f} {speed:>{width2}.3f} {motor_force:>{width3}.3f}")

        add_joint_to_text_list(slew_joint)
        add_joint_to_text_list(boom_joint)
        add_joint_to_text_list(arm_joint)
        add_joint_to_text_list(bucket_joint)
        add_joint_to_text_list(sprocket_joint_l)
        add_joint_to_text_list(sprocket_joint_r)
        add_joint_to_text_list(hinge_boom)
        add_joint_to_text_list(hinge_arm)
        add_joint_to_text_list(hinge_bucket)

        if shovel:
            # Shovel 由来の力は N 単位で返るため kN に変換する。
            # TotalExcavationForce は個別成分を足し合わせた、このサンプル内での参考合力。
            pf = get_penetration_force() * 1e-3
            sf = get_separation_force() * 1e-3
            cf = get_contact_force() * 1e-3
            dcf = get_deformation_contact_force() * 1e-3
            tf = pf + sf + cf + dcf

            text_list.append(
                f"{'Force':>{width1}}: {'Magnitude':>{width2}} {'x':>{width2}} {'y':>{width2}} {'z':>{width2}}")

            def add_force_to_text_list(_name: str, f: agx.Vec3):
                # length は力ベクトルの大きさ、x/y/z はワールド座標系の各成分。
                text_list.append(
                    f"{_name + '(kN)':>{width1}}: {f.length():>{width2}.3f} {f.x():>{width2}.3f} {f.y():>{width2}.3f} {f.z():>{width2}.3f}")

            add_force_to_text_list("PenetrationForce", pf)
            add_force_to_text_list("SeparationContactForce", sf)
            add_force_to_text_list("ContactForce", cf)
            add_force_to_text_list("DeformationContactForce", dcf)
            add_force_to_text_list("TotalExcavationForce", tf)

            text_list.append(f"{'BucketSoilVolume(m3)':>{width1}}: {get_bucket_soil_volume():{width2}.3f}")
            text_list.append(f"{'BucketSoilMass(kg)':>{width1}}: {get_bucket_soil_mass():{width2}.3f}")

        # SceneDecorator へ 1 行ずつ登録し、AGX Viewer の画面上に描画する。
        # index が行番号として扱われるため、text_list の順序がそのまま表示順になる。
        for index, line in enumerate(text_list):
            sd.setText(index, line)

    # update_monitor を postCallback として登録する。
    # postCallback は物理計算ステップの後に呼ばれるので、表示値は直近ステップの状態になる。
    StepEventCallback.postCallback(update_monitor)
