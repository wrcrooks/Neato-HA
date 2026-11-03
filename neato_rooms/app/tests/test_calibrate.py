import dataclasses

from neato_rooms.calibrate import calibrate
from neato_rooms.config import Config
from neato_rooms.robot.sim import SimRobot


async def _noop(msg, progress):
    pass


async def test_detects_wheel_base_and_lidar_direction():
    true_cfg = Config(sim_time_scale=0, lidar_ccw=False)  # how the "real" robot behaves
    robot = SimRobot(true_cfg, true_wheel_base_mm=252.0, odom_noise=0.003)
    robot.pose = type(robot.pose)(3.0, 1.5, 0.3)  # open floor in the living room
    await robot.set_manual(True)
    # Our (wrong) beliefs: ccw LIDAR and 241 mm wheel base.
    believed = Config(sim_time_scale=0, lidar_ccw=True, wheel_base_mm=241.0)
    robot.cfg = dataclasses.replace(believed, lidar_ccw=False)  # sim renders the true convention
    res = await calibrate(believed, robot, _noop)
    print(res)
    assert res["lidar_ccw"] is False
    assert abs(res["wheel_base_mm_suggested"] - 252.0) < 2.5
