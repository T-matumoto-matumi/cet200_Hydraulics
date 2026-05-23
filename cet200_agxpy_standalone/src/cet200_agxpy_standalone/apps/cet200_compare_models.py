# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
import agx
import agxPython
import agxCollide
import agxIO
import agxOSG
import agxSDK
import agxUtil
import agxModel
from agxPythonModules.utils.environment import init_app, simulation, root, application

# Python imports
import sys
from pathlib import Path
import importlib

from cet200_agxpy_standalone import cet200


def set_camera_pose():
    # AGX Viewer のカメラ設定。
    # URDF 由来モデルと AGX ファイル由来モデルを同時に見比べるため、少し引いた視点を登録する。
    app = application()
    camera_data = app.getCameraData()
    camera_data.nearClippingPlane = 0.1
    camera_data.farClippingPlane = 500
    app.applyCameraData(camera_data)

    # eye: カメラ位置、center: 注視点、up: 画面上方向。
    eye = agx.Vec3(7.7989083905525947E-02, 1.3738740061511153E+01, 2.0896597500178338E+00)
    center = agx.Vec3(9.6601861454994287E-01, 3.1072512744819203E-01, 2.4927204246851895E+00)
    up = agx.Vec3(-0.0085, 0.0294, 0.9995)
    app.setCameraHome(eye, center, up)


def add_ground():
    # モデル比較用の単純な静的地面。
    # 掘削や履帯摩擦の検証ではなく、モデル形状・リンク配置の比較が主目的。
    rb_ground = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(20, 20, 1)))
    rb_ground.setPosition(agx.Vec3(0, 0, -1))
    rb_ground.setMotionControl(agx.RigidBody.STATIC)
    simulation().add(rb_ground)
    agxOSG.createVisual(rb_ground, root())
    return rb_ground


def add_urdf_excavator():
    # cet200_description の URDF から CET200 を読み込む。
    # AGX ファイル版 cet200.agx と並べて、リンク配置や可視形状を比較するための関数。
    base_dir = Path(__file__).resolve().parent
    urdf_file_path = (base_dir / "../../../../cet200_description/urdf/cet200.urdf").resolve().as_posix()
    package_path = (base_dir / "../../../../").resolve().as_posix()

    # URDF 読み込み設定。
    # fixToWorld_=False なのでモデルは世界固定されない。
    # disableLinkedBodies_=True は URDF のリンク結合を AGX 側で扱いやすい形にする設定。
    # mergeKinematicLinks_=False でキネマティックリンクを自動マージしない。
    urdf_settings = agxModel.UrdfReaderSettings(
        fixToWorld_=False,
        disableLinkedBodies_=True,
        mergeKinematicLinks_=True
    )
    assembly_ref_excavator = agxModel.UrdfReader.read(urdf_file_path, package_path, None, urdf_settings)

    if assembly_ref_excavator.get() is None:
        print("Error reading the URDF file.")
        sys.exit(2)

    # URDF から生成された Assembly をシミュレーションへ追加し、可視化ノードも作る。
    excavator: agxSDK.Assembly = assembly_ref_excavator.get()
    simulation().add(excavator)
    agxOSG.createVisual(excavator, root())

    # URDF では TF 用の dummy/tf リンクを使って座標系を表している。
    # AGX シミュレーションでは軽量な ObserverFrame として使いたいため、
    # dummy Constraint から ObserverFrame を作成し、不要になった dummy/tf RigidBody は削除する。
    def is_match_prefix(text: str, prefix: str):
        return text.lower().startswith(prefix.lower())

    def is_match_string(text: str, query: str):
        return text.lower() == query.lower()

    joint_removables = list()
    for joint in excavator.getConstraints():
        joint: agx.Constraint
        if is_match_prefix(joint.getName(), "dummy"):
            # dummy Joint は TF 表現用なので、ObserverFrame 作成後に削除する候補として記録する。
            joint_removables.append(joint)
            rb0: agx.RigidBody = joint.getBodyAt(0)
            rb1: agx.RigidBody = joint.getBodyAt(1)

            # tf RigidBody が rb0 側にあれば、rb1 にぶら下がる ObserverFrame として置き換える。
            if is_match_prefix(rb0.getName(), "tf"):
                of = agx.ObserverFrame(rb1)
                of.setTransform(rb0.getTransform())
                of.setName(rb0.getName())
                simulation().add(of)
                print(f"Add ObserverFrame: {rb0.getName()}")
            # tf RigidBody が rb1 側にある場合も同様に置き換える。
            if is_match_prefix(rb1.getName(), "tf"):
                of = agx.ObserverFrame(rb0)
                of.setTransform(rb1.getTransform())
                of.setName(rb1.getName())
                simulation().add(of)
                print(f"Add ObserverFrame: {rb1.getName()}")

    # ObserverFrame 化が終わった dummy Joint をシミュレーションから外す。
    for joint in joint_removables:
        print(f"Remove joint: {joint.getName()}")
        simulation().remove(joint)

    # base_link 自体にも ObserverFrame を追加し、残った dummy/tf RigidBody を削除する。
    rbs = excavator.getRigidBodies()
    for rb in rbs:
        rb: agx.RigidBody
        if is_match_string(rb.getName(), "base_link"):
            of = agx.ObserverFrame(rb)
            of.setName(rb.getName())
            simulation().add(of)
            print(f"Add ObserverFrame: {rb.getName()}")
        if is_match_prefix(rb.getName(), "dummy"):
            print(f"Remove RigidBody: {rb.getName()}")
            simulation().remove(rb)
        if is_match_prefix(rb.getName(), "tf"):
            print(f"Remove RigidBody: {rb.getName()}")
            simulation().remove(rb)

    return excavator


def buildScene1():
    # URDF 版と AGX ファイル版の CET200 を同じシーンに読み込み、形状や座標系を比較する。
    sim = simulation()

    # 利用可能スレッド数を取得した後、半分に落として計算負荷を抑える。
    agx.setNumThreads(0)
    agx.setNumThreads(int(agx.getNumThreads() / 2))
    set_camera_pose()

    # 比較用の基準地面を追加する。
    rb_ground = add_ground()

    # URDF 版モデルを読み込む。
    urdf_excavator = add_urdf_excavator()

    # デバッグ用: 下部走行体を固定したい場合に使う。
    # urdf_excavator.getRigidBody("RB_TrackFrame").setMotionControl(agx.RigidBody.KINEMATICS)

    # URDF 版の 1 自由度拘束モータを有効化し、関節状態を操作・保持できるようにする。
    for joint in urdf_excavator.getConstraints():
        j1: agx.Constraint1DOF = joint.asConstraint1DOF()
        if j1:
            j1.getMotor1D().setEnable(True)

    # AGX ファイル版モデルを読み込む。
    # cet200.add_excavator は履帯 Track と旋回ロックも追加設定する。
    excavator = cet200.add_excavator()

    # デバッグ用: 下部走行体を固定したい場合に使う。
    # excavator.getRigidBody("RB_TrackFrame").setMotionControl(agx.RigidBody.KINEMATICS)

    # AGX ファイル版の 1 自由度拘束モータも有効化する。
    for joint in excavator.getConstraints():
        j1: agx.Constraint1DOF = joint.asConstraint1DOF()
        if j1:
            j1.getMotor1D().setEnable(True)

    # 2 つのモデルが重ならないように、AGX ファイル版を y 方向へずらす。
    excavator.setPosition(agx.Vec3(0, -5, 0))

    # 比較表示が目的なので、URDF 版と AGX ファイル版の相互衝突を無効化する。
    # これにより、近くに置いてもモデル同士が押し合わない。
    agxUtil.setEnableCollisions(urdf_excavator, excavator, False)


# agxViewer からこのスクリプトを scene として読み込む場合の入口。
def buildScene():
    # scene ファイルと buildScene1 を AGX アプリケーションに登録する。
    scene_file = application().getArguments().getArgumentName(1)
    application().addScene(scene_file, "buildScene1", ord('1'))

    # 比較用途では勝手に動かず静止状態で見たいので、自動ステップを無効にする。
    application().setAutoStepping(False)
    buildScene1()


# ros2 run など、console_scripts 経由で起動される場合の入口。
def main():
    # Colcon は sys.argv[0] に console_scripts のコマンド名を入れる。
    # AGX 側は Python ファイルパスを使って scene を読み込むため、実ファイルパスへ差し替える。
    sys.argv[0] = str(Path(__file__).resolve())
    init = init_app(name="__main__", scenes=[(buildScene, '1')])


# python cet200_compare_models.py で直接実行された場合の入口。
if __name__ == "__main__":
    main()
