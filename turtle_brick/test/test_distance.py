from turtle_brick import turtle_robot


def test_distance_helper():
    pos1 = turtle_robot.Position(1.0, 3.0, 0.0)
    pos2 = turtle_robot.Position(3.0, 3.0, 0.0)

    assert turtle_robot.distance_helper(pos1, pos2) == 2.0
