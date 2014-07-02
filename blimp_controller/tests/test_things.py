from blimp_controller.autopilot.blimp import BlimpAutopilot
from blimp_controller.constants import Instructions


def test_yaw():
    autopilot = BlimpAutopilot()
    direction_to_count = {
        Instructions.YAW_LEFT: 0,
        Instructions.YAW_RIGHT: 0
    }

    for target_azimuth in range(0, 360):
        for current_azimuth in range(0, 360):
            autopilot.current_azimuth = current_azimuth
            autopilot.target_azimuth = target_azimuth
            instruction, intensity = autopilot._get_yaw_instruction()
            if current_azimuth == target_azimuth:
                assert(intensity == 0)
            else:
                direction_to_count[instruction] += 1

            if current_azimuth % 30 == 0 and target_azimuth % 30 == 0:
                print current_azimuth, target_azimuth
    print direction_to_count

if __name__ == "__main__":
    test_yaw()
