# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

# AGX Dynamics imports
import agx
import agxPython
import agxCollide
import agxIO
import agxOSG
import agxUtil
import agxTerrain
from agxPythonModules.utils.environment import init_app, simulation, root, application

# Python imports
import sys
from pathlib import Path
import importlib

from cet200_agxpy_standalone import (
    cet200,
    excavator_monitor,
    #excavator_keyboard_gamepad_with_hydraulics as excavator_keyboard_gamepad,
    excavator_keyboard_gamepad_with_hydraulics_arm as excavator_keyboard_gamepad,

)

# AGX Viewer 上で何度も scene を読み直して試す場合、Python の import キャッシュに
# 古いモジュールが残ることがある。reload しておくと、編集した補助モジュールの内容が
# 次回のシーン構築に反映されやすくなる。
importlib.reload(cet200)
importlib.reload(excavator_monitor)
importlib.reload(excavator_keyboard_gamepad)


def set_camera_pose():
    # AGX Viewer のカメラ設定。
    # near/far clipping plane を明示して、ショベル全体と遠方の地形が欠けずに見えるようにする。
    app = application()
    camera_data = app.getCameraData()
    camera_data.nearClippingPlane = 0.1
    camera_data.farClippingPlane = 500
    app.applyCameraData(camera_data)

    # eye: カメラ位置、center: 注視点、up: 画面上方向。
    # app.setCameraHome に登録しておくと、Viewer の Home 操作でこの視点に戻れる。
    eye = agx.Vec3(-2.4283494181646127E+01, 1.2931941843364848E+01, 1.0132067363302802E+01)
    center = agx.Vec3(-1.2156663185500085E+00, 2.2800663641677626E-01, -1.9087158882887673E-01)
    up = agx.Vec3(0.3019, -0.2078, 0.9304)
    app.setCameraHome(eye, center, up)


def add_contact_material_ground_vs_track(mat_ground, mat_track):
    # 地面と履帯の接触材を作成する。
    # 履帯は進行方向と横方向で摩擦特性を変える必要があるため、
    # PRIMARY_DIRECTION と SECONDARY_DIRECTION に別々の摩擦・粘性を設定している。
    cmat: agx.ContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(mat_ground, mat_track)
    cmat.setRestitution(0)
    cmat.setFrictionCoefficient(1, agx.ContactMaterial.PRIMARY_DIRECTION)
    cmat.setSurfaceViscosity(1.0E-6, agx.ContactMaterial.PRIMARY_DIRECTION)
    cmat.setFrictionCoefficient(0.25, agx.ContactMaterial.SECONDARY_DIRECTION)
    cmat.setSurfaceViscosity(6.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION)
    return cmat


def add_contact_material_wheel_vs_track(mat_wheel, mat_track):
    # 転輪・スプロケット類と履帯の接触材。
    # 高い Young 率で接触を硬めにし、反発を 0 にして履帯が車輪上で跳ねにくい設定にする。
    cmat: agx.ContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(mat_wheel, mat_track)
    cmat.setYoungsModulus(1e10)
    cmat.setRestitution(0)
    cmat.setSurfaceViscosity(1)
    return cmat


def add_terrain(material: agx.Material):
    # 掘削可能な Terrain を作成する。
    # resolution_x/y はグリッド数、element_size は 1 セルの大きさ [m]、
    # maximum_depth は地形が変形・掘削できる深さの上限を表す。
    resolution_x = 200
    resolution_y = 200
    element_size = 0.15
    maximum_depth = 2
    terrain = agxTerrain.Terrain(resolution_x, resolution_y, element_size, maximum_depth)

    # AGX Terrain に用意されている土質プリセットを読み込み、
    # このシーンで使う地面用 Material を Terrain 本体へ割り当てる。
    terrain.loadLibraryMaterial("dirt_1")
    terrain.setMaterial(material, agxTerrain.Terrain.MaterialType_TERRAIN)

    # 掘削接触の硬さを調整する。値を小さくすると土砂が比較的やわらかく反応する。
    ecp: agxTerrain.ExcavationContactProperties = terrain.getTerrainMaterial().getExcavationContactProperties()
    ecp.setAggregateStiffnessMultiplier(0.1)

    # 範囲外へ飛んだ土粒子を削除して、長時間実行時の粒子数増加を抑える。
    terrain.getProperties().setDeleteSoilParticlesOutsideBounds(True)

    simulation().add(terrain)

    # Terrain の可視化設定。
    # HeightField は地形表面、SoilParticlesMesh は掘削で生成された土粒子、
    # Compaction は締固め状態を色として表示するためのレンダラ設定。
    renderer = agxOSG.TerrainVoxelRenderer(terrain, root())
    renderer.setRenderHeightField(True)
    renderer.setRenderSoilParticlesMesh(True)
    renderer.setRenderCompaction(True, agx.RangeReal(0.85, 1.15))
    simulation().add(renderer)

    return terrain


def buildScene1():
    # 実際に excavator シーンを組み立てる本体関数。
    # buildScene から呼ばれるほか、AGX Viewer の scene 登録先としても使われる。
    sim = simulation()

    # いったん AGX に利用可能スレッド数を判定させた後、半分のスレッド数に落とす。
    # Terrain や履帯は計算負荷が高いため、環境によっては全スレッド使用より安定することがある。
    agx.setNumThreads(0)
    agx.setNumThreads(int(agx.getNumThreads() / 2))

    # 接触判定に使う材料を作成し、シミュレーションへ登録する。
    # 物体側の Material と ContactMaterial を分けることで、
    # 「どの物体同士が触れたか」に応じた摩擦・粘性を設定できる。
    mt_ground = agx.Material("MT_Ground")
    mt_wheel = agx.Material("MT_Wheel")
    mt_track = agx.Material("MT_Track")
    sim.add(mt_ground)
    sim.add(mt_wheel)
    sim.add(mt_track)
    cm_ground_vs_track = add_contact_material_ground_vs_track(mt_ground, mt_track)
    cm_wheel_vs_track = add_contact_material_wheel_vs_track(mt_wheel, mt_track)

    # 掘削対象となる土の地形を追加する。
    terrain = add_terrain(mt_ground)

    # CET200 の AGX モデルを読み込み、初期位置を terrain の上に置く。
    # setup_terrain_shovel はバケットを Terrain 用 Shovel として登録し、
    # 土の掘削量・掘削抵抗・バケット内土量などを計算できるようにする。
    excavator = cet200.add_excavator()
    excavator.setPosition(-10, 0, 0)
    cet200.setup_terrain_shovel(excavator)

    # 車輪側と履帯側へ材料を割り当てる。
    # setup_track_material では履帯と地面の摩擦モデルも設定され、走行挙動に影響する。
    cet200.setup_wheel_material(excavator, mt_wheel)
    cet200.setup_track_material(excavator, mt_track, cm_ground_vs_track)
    excavator_keyboard_gamepad.setup_keyboard_gamepad_speed_control(excavator)


    # 画面左上に関節角度、速度、モータ力、掘削力、バケット内土量などを表示する。
    excavator_monitor.setup_excavator_monitor(excavator)

    set_camera_pose()


# agxViewer からこのスクリプトを scene として読み込む場合の入口。
def buildScene():
    # コマンドライン引数の scene ファイル名を AGX アプリケーションに登録し、
    # キー '1' で buildScene1 を呼び出せるようにする。
    scene_file = application().getArguments().getArgumentName(1)
    application().addScene(scene_file, "buildScene1", ord('1'))

    # 自動ステップを有効にして、Viewer 起動後すぐ物理シミュレーションが進むようにする。
    application().setAutoStepping(True)
    buildScene1()


# ros2 run など、console_scripts 経由で起動される場合の入口。
def main():
    # Colcon は sys.argv[0] に console_scripts のコマンド名を入れる。
    # AGX 側は「実行中の Python ファイルのパス」を使って scene を読み込むため、
    # ここで実ファイルパスへ差し替えてから init_app を呼ぶ。
    sys.argv[0] = str(Path(__file__).resolve())
    init = init_app(name="__main__", scenes=[(buildScene, '1')])


# python cet200_on_terrain.py で直接実行された場合の入口。
if __name__ == "__main__":
    main()
