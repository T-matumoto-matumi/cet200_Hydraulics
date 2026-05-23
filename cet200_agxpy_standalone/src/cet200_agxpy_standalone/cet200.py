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
import osg
import agxVehicle
from agxPythonModules.utils.callbacks import StepEventCallback
from agxPythonModules.utils.environment import init_app, simulation, root, application

# Python imports
import sys
import os
import math
from typing import Optional
from pathlib import Path

base_dir = Path(__file__).resolve().parent

# このモジュールから参照する AGX/OBJ/MTL ファイルのパス。
# __file__ からの相対パスで解決しているため、実行時のカレントディレクトリに依存しにくい。
g_agx_file_path = (base_dir / "../../../agx_file/cet200.agx").resolve().as_posix()
g_bucket_agx_file_path = (base_dir / "../../../agx_file/cet200_bucket.agx").resolve().as_posix()
g_shoe_visual_path = (base_dir / "../../../agx_file/track_shoelink.obj").resolve().as_posix()
g_texture_file = (base_dir / "../../../agx_file/track_shoelink.mtl").resolve().as_posix()


def dump_object_names(assembly: agxSDK.Assembly):
    # デバッグ用: Assembly 内の RigidBody、Geometry、Constraint 名を一覧出力する。
    # AGX ファイル内の名前を確認して、getRigidBody/getConstraint1DOF の指定名を調べるときに使う。
    rbs: list[agx.RigidBody] = assembly.getRigidBodies()
    for rb in rbs:
        text_list = [rb.getName()]
        geometries: list[agxCollide.Geometry] = rb.getGeometries()
        for geometry in geometries:
            text_list.append(f"{geometry.getName()}")
        row = ",".join(text_list)
        print(row)
    constraints: list[agx.Constraint] = assembly.getConstraints()
    for joint in constraints:
        text_list = [joint.getName(), joint.getBodyAt(0).getName(), joint.getBodyAt(1).getName()]
        row = ",".join(text_list)
        print(row)


def create_track(track_name, rb_sprocket, rb_idler, rb_rollers):
    # AGX Vehicle の Track を作成する。
    # 引数は順に「履帯ノード数、履帯幅、履帯厚み、ノード間ヒンジの初期コンプライアンス相当」。
    # スプロケット、アイドラ、ローラの RigidBody を TrackWheel として登録すると、
    # AGX が履帯ノードをそれらの周りへ巻き付けて初期化する。
    track = agxVehicle.Track(46, 0.6, 0.135, 0.000499592)
    track.setName(track_name)

    def find_cylinder_shape(rb: agx.RigidBody) -> Optional[agxCollide.Cylinder]:
        # 車輪半径を得るため、Collider 名を含む Geometry の円柱 Shape を探す。
        # 見た目用メッシュではなく接触用 Collider を使うことで、物理形状と一致した半径になる。
        for g in rb.getGeometries():
            if not "Collider" in g.getName():
                continue
            for s in g.getShapes():
                if s.getType() == agxCollide.Shape.CYLINDER:
                    return s.asCylinder()
        print("Warning: Collider not found in ", rb.getName())
        return None

    def create_wheel(wheel_type, rb_wheel: agx.RigidBody):
        # TrackWheel は車輪 RigidBody と、車輪中心を表すローカルフレームを必要とする。
        # 円柱 Collider の位置からワールド寄りの wheel_frame を作り、
        # RigidBody ローカルへ変換して TrackWheel に渡す。
        wheel_shape = find_cylinder_shape(rb_wheel)
        wheel_radius = wheel_shape.getRadius()
        wheel_frame = agx.AffineMatrix4x4()
        # print(wheelGeometry.getPosition())
        wheel_frame.setTranslate(wheel_shape.getTransform().getTranslate())
        local_wheel_frame = wheel_frame * rb_wheel.getTransform().inverse()
        wheel = agxVehicle.TrackWheel(wheel_type, wheel_radius, rb_wheel, local_wheel_frame)
        return wheel

    # スプロケットとアイドラを 1 個ずつ、複数ローラをすべて登録する。
    # initialize 後に TrackNode が生成され、履帯としてシミュレーションできる状態になる。
    track.add(create_wheel(agxVehicle.TrackWheel.SPROCKET, rb_sprocket))
    track.add(create_wheel(agxVehicle.TrackWheel.IDLER, rb_idler))
    for roller in rb_rollers:
        track.add(create_wheel(agxVehicle.TrackWheel.ROLLER, roller))
    track.initialize()
    return track


def set_track_shoe_visual(track: agxVehicle.Track):
    # AGX Vehicle の標準 Track 可視化ではなく、履帯シューの OBJ を各 TrackNode に貼り付ける。
    # 物理形状は TrackNode の Geometry、表示形状は track_shoelink.obj という役割分担。
    shoe_visual_data = agxOSG.readNodeFile(g_shoe_visual_path, False)

    # if True:
    #     # agxOSG.setTexture(shoe_visual_data, g_texture_file, True, agxOSG.DIFFUSE_TEXTURE, 0.4, 1.8)
    #     agxOSG.setTexture(shoe_visual_data, g_texture_file, True, agxOSG.DIFFUSE_TEXTURE, 1, 1)
    # if True:
    #     color = agx.Vec4f(0.10, 0.11, 0.12, 1.0)
    #     agxOSG.setDiffuseColor(shoe_visual_data, color)
    #     agxOSG.setAmbientColor(shoe_visual_data, color * 0.2)
    #     # agxOSG.setSpecularColor(shoe_visual_data, vagx.saturateVec4f(desc.color * 2.0))
    #     agxOSG.setShininess(shoe_visual_data, 128)

    # def find_track_node_size() -> agx.Vec3:
    #     rb = track.nodes()[0].getRigidBody()
    #     geometry = rb.getGeometries()[0]
    #     box = geometry.getShapes()[0].asBox()
    #     return box.getHalfExtents()

    # print(find_track_node_size())

    rotation = agx.AffineMatrix4x4()
    rotation.setRotate(agx.EulerAngles(0, -agx.PI_2, 0))

    # OBJ の向きと TrackNode のローカル軸が合うように回転し、
    # ノード中心から少し上へオフセットして見た目の位置を合わせる。
    shoe_visual_transform = agxOSG.Transform()
    shoe_visual_transform.setMatrix(rotation)
    shoe_visual_transform.setTranslate(agx.Vec3(0, 0, 0.095))
    shoe_visual_transform.addChild(shoe_visual_data)

    # 各 TrackNode の GeometryNode に履帯シュー表示をぶら下げる。
    # これにより、履帯ノードの運動に合わせて OBJ も追従する。
    track_nodes = track.nodes()
    it = track_nodes.begin()
    while it != track_nodes.end():
        track_node: agxVehicle.TrackNode = it.get()
        geometry_node = agxOSG.GeometryNode(track_node.getRigidBody().getGeometries()[0])
        geometry_node.addChild(shoe_visual_transform)
        root().addChild(geometry_node)
        it.inc()


def attach_tracks(excavator: agxSDK.Assembly):
    # AGX ファイルから読み込んだ車輪 RigidBody を使って、左右の履帯を後付けする。
    # cet200.agx 側にはスプロケット/アイドラ/ローラが含まれ、ここで Track オブジェクトを構成する。
    rbs: list[agx.RigidBody] = excavator.getRigidBodies()

    def _create_track(track_name, sprocket_name, idler_name, roller_name):
        # 名前で対象車輪を取り出し、roller_name を含む RigidBody をローラ群として集める。
        sprocket = excavator.getRigidBody(sprocket_name)
        idler = excavator.getRigidBody(idler_name)
        rollers = [rb for rb in rbs if roller_name in rb.getName()]
        _track = create_track(track_name, sprocket, idler, rollers)
        excavator.add(_track)
        simulation().add(_track)

        # createVisual は標準の履帯表示を作るが、このサンプルでは OBJ 表示を使うため一度削除する。
        # 透明度を設定しているのは、デバッグ時に root へ残した場合でも目立ちすぎないようにするため。
        track_visual_node: osg.Node = agxOSG.createVisual(_track, root())
        agxOSG.setAlpha(track_visual_node, 0.3)
        root().removeChild(track_visual_node)
        return _track

    track_l = _create_track("Track_L", "RB_Sprocket_L", "RB_Idler_L", "Roller_L")
    track_r = _create_track("Track_R", "RB_Sprocket_R", "RB_Idler_R", "Roller_R")

    for track in [track_l, track_r]:
        for node in track.nodes():  # type: agxVehicle.TrackNode
            # TrackNode の RigidBody 名と質量を設定する。
            # setup_track_material ではこの名前を使って履帯ノードへ材料を割り当てる。
            track_rb: agx.RigidBody = node.getRigidBody()
            track_rb.setName("RB_TrackNode")
            track_rb.getMassProperties().setMass(34.0)

        # 履帯ノード間ヒンジや車輪との結合・分離条件を調整する。
        # Merge は接触数と自由度を減らして計算を安定化・高速化するための設定。
        tp: agxVehicle.TrackProperties = track.getProperties()
        tp.setStabilizingHingeFrictionParameter(1)
        tp.setHingeCompliance(1e-9)
        tp.setHingeDamping(2 / 60)
        tp.setMinStabilizingHingeNormalForce(9e4)
        tp.setNodesToWheelsMergeThreshold(-0.1)
        tp.setNodesToWheelsSplitThreshold(-0.05)
        tp.setNumNodesIncludedInAverageDirection(3)
        tp.setEnableHingeRange(True)
        tp.setHingeRangeRange(math.radians(-75), math.radians(20))
        tp.setEnableOnInitializeTransformNodesToWheels(True)
        tp.setTransformNodesToWheelsOverlap(1e-3)

        imp: agxVehicle.TrackInternalMergeProperties = track.getInternalMergeProperties()
        imp.setEnableMerge(True)
        imp.setEnableLockToReachMergeCondition(True)
        imp.setNumNodesPerMergeSegment(2)
        imp.setMaxAngleMergeCondition(1e-5)
        imp.setContactReduction(agxVehicle.TrackInternalMergeProperties.MODERATE)
        imp.setLockToReachMergeConditionCompliance(1e-11)
        imp.setLockToReachMergeConditionDamping(0.05)

    # 物理履帯ができた後で、左右の TrackNode に履帯シューの表示モデルを取り付ける。
    set_track_shoe_visual(track_l)
    set_track_shoe_visual(track_r)


def setup_slew_lock_system(excavator: agxSDK.Assembly):
    # 旋回軸は Motor1D だけだと目標速度 0 でもわずかに流れることがあるため、
    # 停止中だけ LockController を有効化して現在角度に保持する。
    hinge_slew = excavator.getConstraint1DOF("Hinge_Slew")
    hinge_slew_motor1d: agx.Motor1D = hinge_slew.getMotor1D()
    hinge_slew_lock1d: agx.LockController = hinge_slew.getLock1D()

    def handle_brake(timestamp):
        def is_zero(value):
            return math.isclose(value, 0.0, abs_tol=0.001)

        if not hinge_slew_motor1d.getEnable():
            return
        target_speed = hinge_slew_motor1d.getSpeed()
        current_speed = hinge_slew.getCurrentSpeed()

        # 入力がある間はロックを外す。現在速度と目標速度が十分 0 に近いときだけ、
        # その時点の角度をロック位置として保持する。
        hinge_slew_lock1d.setEnable(False)
        if is_zero(current_speed) and is_zero(target_speed):
            hinge_slew_lock1d.setEnable(True)
            hinge_slew_lock1d.setPosition(hinge_slew.getAngle())

    # preCallback で物理計算前にロック状態を更新し、そのステップの拘束計算へ反映させる。
    StepEventCallback.preCallback(handle_brake)


def setup_wheel_material(excavator: agxSDK.Assembly, mt_wheel):
    # Roller を名前に含む RigidBody を転輪系として扱い、車輪用 Material を割り当てる。
    for rb in excavator.getRigidBodies():
        if "Roller" in rb.getName():
            agxUtil.setBodyMaterial(rb, mt_wheel)


def setup_track_material(excavator: agxSDK.Assembly, mat_track, cmat: agx.ContactMaterial):
    # TrackNode に履帯用 Material を割り当てる。
    # attach_tracks で TrackNode の RigidBody 名を RB_TrackNode にそろえている。
    track_l: agxSDK.Assembly = excavator.getAssembly("Track_L")
    track_r: agxSDK.Assembly = excavator.getAssembly("Track_R")
    for track in [track_l, track_r]:
        for rb in track.getRigidBodies():
            if "RB_TrackNode" in rb.getName():
                agxUtil.setBodyMaterial(rb, mat_track)

    # 履帯と地面の摩擦モデルを設定する。
    # OrientedBoxFrictionModel は指定フレームの向きに沿って主摩擦方向を定義できるため、
    # 履帯の進行方向と横方向で違う摩擦係数を持たせる用途に合う。
    # body_track_frame = excavator.getRigidBody("TrackFrame")
    observer_frame: agx.ObserverFrame = excavator.getObserverFrame("TF_Origin_Model")

    # normalForceMagnitude = 機械質量 * 重力加速度 / 接地している履帯ノード数、と仮定する。
    # 接地ノード 1 個あたりの代表法線力を与えることで、安定した履帯摩擦を得る。
    machine_mass = 20186
    gravity = math.fabs(simulation().getUniformGravity().z())
    num_ground_contact_track_nodes = 40
    normal_force = machine_mass * gravity / num_ground_contact_track_nodes
    cmat.setFrictionModel(
        agx.ConstantNormalForceOrientedBoxFrictionModel(normal_force,
                                                        observer_frame.getFrame(),
                                                        agx.Vec3.X_AXIS(),
                                                        agx.FrictionModel.DIRECT,
                                                        False))


def setup_terrain_shovel(excavator: agxSDK.Assembly):
    # AGX Terrain で掘削を行うため、バケットを Shovel として登録する。
    # cet200.agx 内の ObserverFrame から上端・刃先・切削方向を取得し、
    # Terrain がバケット形状と土の取り込み方向を理解できるようにする。
    import agxTerrain
    rb_bucket = excavator.getRigidBody("RB_Bucket")
    tf_top_edge_begin = excavator.getObserverFrame("TF_TopEdgeBegin")
    tf_top_edge_end = excavator.getObserverFrame("TF_TopEdgeEnd")
    tf_cutting_edge_begin = excavator.getObserverFrame("TF_CuttingEdgeBegin")
    tf_cutting_edge_end = excavator.getObserverFrame("TF_CuttingEdgeEnd")
    tf_cutting_direction = excavator.getObserverFrame("TF_CuttingDirection")

    top_edge = agx.Line(tf_top_edge_begin.getLocalPosition(), tf_top_edge_end.getLocalPosition())
    cutting_edge = agx.Line(tf_cutting_edge_begin.getLocalPosition(), tf_cutting_edge_end.getLocalPosition())

    # 切削方向は ObserverFrame から一度ワールド方向へ変換し、
    # 最終的にバケット RigidBody のローカル方向へ戻して Shovel に渡す。
    cutting_direction_world = tf_cutting_direction.transformVectorToWorld(agx.Vec3.X_AXIS())
    cutting_direction = rb_bucket.getFrame().transformVectorToLocal(cutting_direction_world)

    shovel = agxTerrain.Shovel(rb_bucket, top_edge, cutting_edge, cutting_direction)
    shovel_settings: agxTerrain.ShovelSettings = shovel.getSettings()

    # バケット歯の本数・長さ・先端半径を設定する。
    # 半径は歯先面積から円相当半径として計算している。
    shovel_settings.setNumberOfTeeth(5)
    shovel_settings.setToothLength(201.5687 * 1e-3)
    shovel_settings.setToothMaximumRadius(math.sqrt(9141.9103 * 1e-6 / agx.PI))
    shovel_settings.setToothMinimumRadius(math.sqrt(2419.9957 * 1e-6 / agx.PI))

    simulation().add(shovel)
    return shovel


def add_excavator():
    # cet200.agx から油圧ショベル本体を読み込む。
    # 読み込み先 Assembly を渡すことで、後続処理でモデル全体を 1 つの単位として扱える。
    excavator = agxSDK.Assembly()
    if not agxOSG.readFile(g_agx_file_path, simulation(), root(), excavator):
        raise Exception("Unable to load model: " + g_agx_file_path)

    # AGX ファイルから読み込んだ本体に、旋回停止保持と履帯 Track を追加設定する。
    setup_slew_lock_system(excavator)
    attach_tracks(excavator)
    return excavator


def add_bucket() -> agxSDK.Assembly:
    # バケット単体モデルを読み込む補助関数。
    # モデル確認や比較表示など、本体とは別にバケットだけ表示したい場合に使う。
    assembly_bucket = agxSDK.Assembly()
    if not agxOSG.readFile(g_bucket_agx_file_path, simulation(), root(), assembly_bucket):
        raise Exception("Unable to load model: " + g_bucket_agx_file_path)

    return assembly_bucket


def buildScene1():
    # cet200.py 単体で実行したときの簡易表示シーン。
    # 地形や入力制御は追加せず、AGX ファイルからモデルが読めるか確認する用途。
    add_excavator()

    application().setAutoStepping(False)

    # モデル全体を見やすい位置から見るための初期カメラ。
    eye = agx.Vec3(2.8976305330906644E+00, 1.6718488143031252E+01, 3.8321300958064648E+00)
    center = agx.Vec3(6.4849247247395725E-01, 1.3679887760401846E-01, 2.7364818354149665E+00)
    up = agx.Vec3(0.0008, -0.0660, 0.9978)
    application().setCameraHome(eye, center, up)


# agxViewer からこのスクリプトを scene として読み込む場合の入口。
def buildScene():
    scene_file = application().getArguments().getArgumentName(1)
    application().addScene(scene_file, "buildScene1", ord('1'))

    buildScene1()


# ros2 run など、console_scripts 経由で起動される場合の入口。
def main():
    # AGX の scene 読み込みがこの Python ファイルを参照できるように、実ファイルパスを渡す。
    sys.argv[0] = Path(__file__).resolve()
    init = init_app(name="__main__", scenes=[(buildScene, '1')])


# python cet200.py で直接実行された場合の入口。
if __name__ == "__main__":
    main()
