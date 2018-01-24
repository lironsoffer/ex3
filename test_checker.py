import time
import ex3_checker

def state_to_agent():
    stuff = []
    stuff.append(ships)
    stuff.append(locations)
    stuff.append(lasers)
    stuff.append(working_instrument)
    calibrated_as_output = {}
    for ship, instrument in calibrated.items():
        if instrument is not None:
            calibrated_as_output[ship] = True
        else:
            calibrated_as_output[ship] = False
    stuff.append(calibrated_as_output)
    stuff.append(reward)
    return tuple(stuff)

if __name__ == '__main__':
    start = time.time()
    problem = (
        5,
        ["Sidonia", "Enterprise"],
        ("rangefinder", "thermal_camera"),
        {"Sidonia": ("rangefinder", ), "Enterprise": ("thermal_camera", "rangefinder")},
        {"rangefinder": (0, 3, 2), "thermal_camera": (1, 4, 2)},
        {(1, 4, 1): ("thermal_camera", "rangefinder"), (4, 3, 4): ("thermal_camera", "rangefinder")},
        {"Sidonia": (4, 1, 1), "Enterprise": (3, 0, 3)},
        ((-1, 2, 2), (4, 4, -1)),
        20,
        180
    )
    try:
        calibrated = {x: None for x in problem[1]}
        calibrated_as_output = {}
        for ship, instrument in calibrated.items():
            if instrument is not None:
                calibrated_as_output[ship] = True
        ex3_checker.check_solution(problem, (problem[1], problem[6], {x: False for x in problem[7]}, {x: None for x in problem[1]}, calibrated_as_output, 0))

    except Exception as e:
        print(e)
    finish = time.time()
    print("Overall time elapsed:", finish-start)


