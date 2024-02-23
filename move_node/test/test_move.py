import pytest
import numpy as np
from move_node.move import SimpleRobotMover

# Test case: When the target pose is not explicitly set and the current pose is [0, 0, 0],
# get_velocity_command() should return None.
def test_get_velocity_command_no_target_pose_0_0_0_return_none():
    mover = SimpleRobotMover()
    assert mover.get_velocity_command(0, 0, 0) is None

# Test case: When the target pose is not explicitly set and the current pose is [1, 2, 3],
# get_velocity_command() should return None.
def test_get_velocity_command_no_target_pose_1_2_3_return_none():
    mover = SimpleRobotMover()
    assert mover.get_velocity_command(1, 1, 0) is None

# Test case: When the target pose is set and then cancelled,
# get_velocity_command() should return None.
def test_get_velocity_command_target_pose_set_and_cancelled_return_none():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 2, 3)
    mover.cancel_target()
    assert mover.get_velocity_command(4, 5, 6) is None

# Test case: When the target pose is set to [1, 1, 0] and the current pose is [0, 0, 0],
# get_velocity_command() should return a command to turn left.
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_1_0_return_turn_left():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 1, 0)
    assert mover.get_velocity_command(0, 0, 0) == [0.0, SimpleRobotMover._omega]

# Test case: When the target pose is set to [1, -1, 0] and the current pose is [0, 0, 0],
# get_velocity_command() should return a command to turn right.
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_minus_1_0_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, -1, 0)
    assert mover.get_velocity_command(0, 0, 0) == [0.0, -SimpleRobotMover._omega]

# Test case: When the target pose is set to [1, 0, 0] and the current pose is [0, 0, 0],
# get_velocity_command() should return a command to move forward.
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_0_0_return_move_forward():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 0, 0)
    assert mover.get_velocity_command(0, 0, 0) == [SimpleRobotMover._v, 0.0]

# Test case: When the target pose is set to [2, 0, 0] and the current pose is [1, 0, 0],
# get_velocity_command() should return a command to move forward.
def test_get_velocity_command_cur_pose_1_0_0_target_pose_2_0_0_return_move_forward():
    mover = SimpleRobotMover()
    mover.set_target_pose(2, 0, 0)
    assert mover.get_velocity_command(1, 0, 0) == [SimpleRobotMover._v, 0.0]

# Test case: When the target pose is set to [42, 1, 0] and the current pose is [1, 2, pi/4],
# get_velocity_command() should return a command to turn right.
def test_get_velocity_command_cur_pose_1_2_pi4_target_pose_42_1_0_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(42, 1, 0)
    assert mover.get_velocity_command(1, 2, np.pi/4) == [0.0, -SimpleRobotMover._omega]

# Test case: When the target pose is set to [1, 2, pi/4] and the current pose is [1, 2, pi/4],
# get_velocity_command() should return None.
def test_get_velocity_command_cur_pose_1_2_pi4_target_pose_1_2_pi4_return_none():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 2, np.pi/4)
    assert mover.get_velocity_command(1, 2, np.pi/4) is None

# Test case: When the target pose is set to [1.01, 2.01, -pi/4] and the current pose is [1, 2, 0],
# get_velocity_command() should return a command to turn right.
def test_get_velocity_command_cur_pose_1_2_0_target_pose_101_201_minus_pi4_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(1.01, 2.01, -np.pi/4)
    assert mover.get_velocity_command(1, 2, 0) == [0.0, -SimpleRobotMover._omega]


# Test case: When the target pose is set to [1, 2, -pi/8] and the current pose is [1, 2, -pi/4],
# get_velocity_command() should return_turn_left .
def test_get_velocity_command_cur_pose_1_2_mpi4_target_pose_1_2_mpi8_return_turn_left():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 2, -np.pi/8)
    assert mover.get_velocity_command(1, 2, -np.pi/4) == [0.0, SimpleRobotMover._omega ]