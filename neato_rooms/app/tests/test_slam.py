import math

from neato_rooms.config import Config
from neato_rooms.robot.sim import SimRobot
from neato_rooms.slam.mapper import Mapper


async def _drive(robot, mapper, cmds):
    errs = []
    for kind, val in cmds:
        r = await (robot.forward(val, 200) if kind == "f" else robot.rotate(math.radians(val), 150))
        scan = await robot.scan()
        info = mapper.update(await robot.odometry(), scan.to_points(robot.cfg))
        truth = robot.world_to_map(robot.pose)
        errs.append((info.pose.distance_to(truth.x, truth.y), abs(info.pose.theta - truth.theta)))
    return errs


async def test_slam_tracks_ground_truth_better_than_odometry():
    cfg = Config(sim_time_scale=0)
    robot = SimRobot(cfg, odom_noise=0.03, true_wheel_base_mm=cfg.wheel_base_mm * 1.04)
    await robot.set_manual(True)
    mapper = Mapper(cfg)
    mapper.reset_odometry(await robot.odometry())
    mapper.update(await robot.odometry(), (await robot.scan()).to_points(cfg))
    # Loop around the living room.
    cmds = [("f", 0.3)] * 12 + [("r", 45), ("r", 45)] + [("f", 0.3)] * 5 + [("r", 90)] + [("f", 0.3)] * 10 + [("r", 90)] + [("f", 0.3)] * 4
    errs = await _drive(robot, mapper, cmds)
    max_pos = max(e[0] for e in errs)
    max_ang = max(e[1] for e in errs)
    print("max pos err", max_pos, "max ang err deg", math.degrees(max_ang))
    assert max_pos < 0.05
    assert math.degrees(max_ang) < 2.5
