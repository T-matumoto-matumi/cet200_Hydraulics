# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
import agx
import agxPython
import agxCollide
import agxIO
import agxOSG
import agxUtil
from agxPythonModules.utils.callbacks import StepEventCallback
from agxPythonModules.utils.environment import init_app, simulation, root, application


# Python imports
import math
import sys
from pathlib import Path
import importlib

from cet200_agxpy_standalone import excavator_keyboard_gamepad_with_hydraulics as excavator_keyboard_gamepad, excavator_monitor, cet200, lidar_sensor

# AGX Viewer で scene を再読み込みしながら調整する場合に備えて、
# 補助モジュールを reload して最新の編集内容を反映しやすくする。
importlib.reload(cet200)
importlib.reload(excavator_monitor)
importlib.reload(excavator_keyboard_gamepad)
importlib.reload(lidar_sensor)

g_lidar_sensor = None
g_latest_ground_points = None
g_lidar_point_cloud_plotter = None

LIDAR_PLOT_EVERY_N_STEPS = 10


def get_latest_ground_point_cloud():
    return g_latest_ground_points


class LidarPointCloudPlotter:
    def __init__(self):
        self._enabled = False
        self._scatter = None
        self._colorbar = None

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("Warning: matplotlib is not installed. 2D lidar height plot window was not opened.")
            return

        self._plt = plt
        self._plt.ion()
        self._fig = self._plt.figure("CET200 Lidar Ground Height Map")
        self._ax = self._fig.add_subplot(111)
        self._ax.set_title("Lidar Ground Height Map")
        self._ax.set_xlabel("X [m]")
        self._ax.set_ylabel("Y [m]")
        self._ax.set_aspect("equal", adjustable="box")
        self._ax.grid(True, alpha=0.3)
        self._enabled = True

    def update(self, points):
        if not self._enabled:
            return
        if points is None or len(points) == 0:
            return

        if self._scatter is not None:
            self._scatter.remove()

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        self._scatter = self._ax.scatter(x, y, c=z, cmap="terrain", s=8)
        if self._colorbar is None:
            self._colorbar = self._fig.colorbar(self._scatter, ax=self._ax)
            self._colorbar.set_label("Height Z [m]")

        self._set_equal_xy_axes(points)
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        self._plt.pause(0.001)

    def _set_equal_xy_axes(self, points):
        mins = points[:, :2].min(axis=0)
        maxs = points[:, :2].max(axis=0)
        center = 0.5 * (mins + maxs)
        radius = max((maxs - mins).max() * 0.55, 0.5)

        self._ax.set_xlim(center[0] - radius, center[0] + radius)
        self._ax.set_ylim(center[1] - radius, center[1] + radius)


def set_camera_pose():
    # AGX Viewer のカメラ設定。
    # near/far clipping plane を明示して、地面とショベル全体がクリップされにくいようにする。
    app = application()
    camera_data = app.getCameraData()
    camera_data.nearClippingPlane = 0.1
    camera_data.farClippingPlane = 500
    app.applyCameraData(camera_data)

    # eye: カメラ位置、center: 注視点、up: 画面上方向。
    # Home 視点として登録して、Viewer 操作中にこの視点へ戻れるようにする。
    eye = agx.Vec3(7.7989083905525947E-02, 1.3738740061511153E+01, 2.0896597500178338E+00)
    center = agx.Vec3(9.6601861454994287E-01, 3.1072512744819203E-01, 2.4927204246851895E+00)
    up = agx.Vec3(-0.0085, 0.0294, 0.9995)
    app.setCameraHome(eye, center, up)


def add_contact_material_ground_vs_track(mat_ground, mat_track):
    # 平面地面と履帯の接触材を作成する。
    # 履帯進行方向と横方向で摩擦係数・表面粘性を分け、クローラらしい走行に近づける。
    cmat: agx.ContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(mat_ground, mat_track)
    cmat.setRestitution(0)
    cmat.setFrictionCoefficient(1, agx.ContactMaterial.PRIMARY_DIRECTION)
    cmat.setSurfaceViscosity(1.0E-6, agx.ContactMaterial.PRIMARY_DIRECTION)
    cmat.setFrictionCoefficient(0.25, agx.ContactMaterial.SECONDARY_DIRECTION)
    cmat.setSurfaceViscosity(6.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION)
    return cmat


def add_contact_material_wheel_vs_track(mat_wheel, mat_track):
    # 転輪・スプロケットと履帯の接触材。
    # 反発を 0 にし、接触を硬め・粘性ありにして履帯が車輪上で暴れにくい設定にする。
    cmat: agx.ContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(mat_wheel, mat_track)
    cmat.setYoungsModulus(1e10)
    cmat.setRestitution(0)
    cmat.setSurfaceViscosity(1)
    return cmat


def add_ground():
    # 掘削 Terrain ではなく、単純な静的 Box を地面として置く。
    # cet200_on_terrain.py より軽く、走行・関節操作・履帯挙動を確認しやすい。
    rb_ground = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(10, 10, 1)))
    rb_ground.setPosition(agx.Vec3(0, 0, -1))
    rb_ground.setMotionControl(agx.RigidBody.STATIC)
    simulation().add(rb_ground)
    agxOSG.createVisual(rb_ground, root())
    return rb_ground


def add_labeled_axes(rb: agx.RigidBody, frame, label: str, scale: float = 0.35, width: float = 2.0):
    # createAxes が返す MatrixTransform に Text を追加すると、軸とラベルが同じ姿勢で追従する。
    axes_node = agxOSG.createAxes(rb, frame, root(), scale, width)
    label_node = agxOSG.createText(label, agx.Vec3(scale * 0.2, scale * 0.2, scale * 0.2),
                                   agx.Vec4f(1, 1, 1, 1), scale * 0.25)
    axes_node.addChild(label_node)
    return axes_node


def add_observer_frame_axes(assembly):
    # Assembly 配下の ObserverFrame を取得し、名前付きの座標軸として表示する。
    # 子 Assembly に含まれる ObserverFrame も拾う。
    for observer_frame in assembly.getObserverFrames():
        observer_frame: agx.ObserverFrame
        rb = observer_frame.getRigidBody()
        add_labeled_axes(rb, observer_frame.getFrame(), observer_frame.getName())
        print(f"Show ObserverFrame: {observer_frame.getName()} on {rb.getName()}")

    for child_assembly in assembly.getAssemblies():
        add_observer_frame_axes(child_assembly)


def find_observer_frame(assembly, name: str):
    observer_frame = assembly.getObserverFrame(name)
    if observer_frame:
        return observer_frame

    for child_assembly in assembly.getAssemblies():
        observer_frame = find_observer_frame(child_assembly, name)
        if observer_frame:
            return observer_frame

    return None


def get_observer_frame_world_position(observer_frame: agx.ObserverFrame):
    rb = observer_frame.getRigidBody()
    return rb.getFrame().transformPointToWorld(observer_frame.getLocalPosition())


def setup_lidar_ground_scan(excavator, ground_body):
    global g_lidar_sensor
    global g_latest_ground_points
    global g_lidar_point_cloud_plotter

    tf_lidar_base = find_observer_frame(excavator, "TF_LidarBase")
    if not tf_lidar_base:
        print("Warning: TF_LidarBase was not found. 3D lidar was not added.")
        return None

    rb_lidar_base = tf_lidar_base.getRigidBody()
    lidar_position = get_observer_frame_world_position(tf_lidar_base)
    lidar_forward = tf_lidar_base.transformVectorToWorld(agx.Vec3.X_AXIS()).normal()

    ground_lidar = lidar_sensor.LidarSensor3DNoGPU(
        sim=simulation(),
        world_position=lidar_position,
        world_direction=lidar_forward,
        horizontal_rays=81,
        vertical_rays=24,
        horizontal_fov=math.radians(120.0),
        vertical_fov=math.radians(45.0),
        max_length=10.0,
        rb_origin=rb_lidar_base,
        rb_origin_frame=tf_lidar_base.getLocalTransform(),
        draw_lines=True,
    )

    # 地形形状を取りたいので、車体・アーム・履帯などショベル自身はLiDAR接触から外す。
    agxUtil.setEnableCollisions(ground_lidar.rigid_body, excavator, False)
    agxUtil.setEnableCollisions(ground_lidar.rigid_body, ground_body, True)
    g_lidar_sensor = ground_lidar
    g_latest_ground_points = ground_lidar.get_point_cloud(world_coordinates=True, include_misses=False)
    g_lidar_point_cloud_plotter = LidarPointCloudPlotter()

    sample_index = {"value": 0}

    def update_ground_points(_timestamp):
        global g_latest_ground_points

        sample_index["value"] += 1
        points = ground_lidar.get_point_cloud(world_coordinates=True, include_misses=False)
        g_latest_ground_points = points

        if sample_index["value"] % LIDAR_PLOT_EVERY_N_STEPS == 0:
            g_lidar_point_cloud_plotter.update(points)

        if sample_index["value"] % 30 != 0:
            return

        if len(points) == 0:
            print("3D Lidar: no ground hits")
            return

        min_xyz = points.min(axis=0)
        max_xyz = points.max(axis=0)
        print(
            "3D Lidar ground points: "
            f"{len(points)} hits, "
            f"x[{min_xyz[0]:.2f}, {max_xyz[0]:.2f}] "
            f"y[{min_xyz[1]:.2f}, {max_xyz[1]:.2f}] "
            f"z[{min_xyz[2]:.2f}, {max_xyz[2]:.2f}]"
        )

    StepEventCallback.postCallback(update_ground_points)
    print(f"3D Lidar attached to TF_LidarBase on {rb_lidar_base.getName()}")
    return ground_lidar


def buildScene1():
    # 平面地面上で CET200 を動かすサンプルシーン。
    # Terrain 掘削は使わず、走行と上部作業機の基本操作を確認する構成。
    sim = simulation()

    # 利用可能スレッド数を取得した後、半分に落として計算負荷を抑える。
    agx.setNumThreads(0)
    agx.setNumThreads(int(agx.getNumThreads() / 2))

    # 接触判定に使う Material と ContactMaterial を作成する。
    # Material は物体側へ、ContactMaterial は物体ペアごとの接触特性へ使う。
    mt_ground = agx.Material("MT_Ground")
    mt_wheel = agx.Material("MT_Wheel")
    mt_track = agx.Material("MT_Track")
    sim.add(mt_ground)
    sim.add(mt_wheel)
    sim.add(mt_track)
    cm_ground_vs_track = add_contact_material_ground_vs_track(mt_ground, mt_track)
    cm_wheel_vs_track = add_contact_material_wheel_vs_track(mt_wheel, mt_track)

    # 静的な地面を追加し、モデルが見やすいように少し x 方向へずらす。
    rb_ground = add_ground()
    rb_ground.setPosition(rb_ground.getPosition() + agx.Vec3(-7, 0, 0))
    agxUtil.setBodyMaterial(rb_ground, mt_ground)

    # CET200 の AGX モデルを読み込み、履帯や旋回ロックをセットアップする。
    excavator = cet200.add_excavator()

    # デバッグ用: 下部走行体をキネマティック固定したい場合に使う。
    # excavator.getRigidBody("RB_TrackFrame").setMotionControl(agx.RigidBody.KINEMATICS)

    # 転輪と履帯へ材料を割り当て、履帯と地面の摩擦モデルを設定する。
    cet200.setup_wheel_material(excavator, mt_wheel)
    cet200.setup_track_material(excavator, mt_track, cm_ground_vs_track)

    # キーボード・ゲームパッド入力を Motor1D 速度指令へ接続する。
    excavator_keyboard_gamepad.setup_keyboard_gamepad_speed_control(excavator)

    # 画面上に関節角度・速度・モータ力などを表示する。
    excavator_monitor.setup_excavator_monitor(excavator)

    setup_lidar_ground_scan(excavator, rb_ground)

    # デバッグ用: ショベルに付いている ObserverFrame の座標軸と名前を可視化する。
    add_observer_frame_axes(excavator)
    set_camera_pose()


# agxViewer からこのスクリプトを scene として読み込む場合の入口。
def buildScene():
    # scene ファイルと buildScene1 を AGX アプリケーションに登録する。
    scene_file = application().getArguments().getArgumentName(1)
    application().addScene(scene_file, "buildScene1", ord('1'))

    # 起動後に物理ステップが自動で進むようにする。
    application().setAutoStepping(True)
    buildScene1()


# ros2 run など、console_scripts 経由で起動される場合の入口。
def main():
    # Colcon は sys.argv[0] に console_scripts のコマンド名を入れる。
    # AGX 側は Python ファイルパスを使って scene を読み込むため、実ファイルパスへ差し替える。
    sys.argv[0] = str(Path(__file__).resolve())
    init = init_app(name="__main__", scenes=[(buildScene, '1')])


# python cet200_on_ground.py で直接実行された場合の入口。
if __name__ == "__main__":
    main()
