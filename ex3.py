import time
import random
from collections import namedtuple
import our_ex1_utils_for_ex3
import search

ids = ["000000000", "111111111"]
Tuple_state = namedtuple('Tuple_state', 'ships, positions, lasers, working_instruments, calibrated_instruments, reward')
Target = namedtuple('Target', 'pos, req_instrument')

def convert_dictionary_to_string_keys(d):
    newd = {}
    for k in d:
        newd[repr(k)] = d[k]
    return newd

def convert_dictionary_from_string_keys(d):
    newd = {}
    for k in d:
        newd[tuple(eval(k))] = d[k]
    return newd

class SpaceshipController:
    "This class is a controller for a spaceship problem."

    def __init__(self, problem, initial_state):
        """Initialize controller for given the initial setting.
        This method MUST terminate within the specified timeout."""
        self.timeout = problem[9]
        #todo: make timeout
        self.max_steps=problem[8]
        self.problem = problem
        self.saved_initial = initial_state

        self.reward = 0
        self.steps = 0

        self.dim = problem[0]
        # self.ships = problem[1]
        self.all_instruments = problem[2]
        self.instruments_on_ships = problem[3]
        self.calibration_targets = problem[4]
        self.all_calibration_targets_positions = problem[4].values()
        self.all_targets = problem[5]
        # self.initial_positions = problem[6]
        self.all_real_targets_positions = problem[5].keys()
        self.all_lasers_positions = problem[7]

        self.exploded_ships = set()
        self.open_targets = make_set_targets_from_dict(problem[5])
        self.closed_targets = set()

        self.last_state = Tuple_state(*initial_state)
        self.last_action = None

        self.action_seq = []
        self.active_problem = (None,None)
        self.obstacles = self.get_obstacles(Tuple_state(*initial_state))

        print('COMPLETE init ')

    def reward(self, state):
        pass

    def h(self, state, action):
        """This is the heuristic. It evaluates the action"""
        # print(state)
        # state = Tuple_state(*state)

        h=self.find_closest_distances_to_targets(state)
        h-=len(self.closed_targets)+self.count_working_and_calibrated(state)

        instruments_needed = []  # amount of needed ships. if ship is not needed ---> append to useless ships
        for target in self.open_targets:
            weapon = target.req_instrument
            if weapon not in instruments_needed:
                instruments_needed.append(weapon)
        useless_ships = []
        for ship in state.ships:
            useless = True
            for weapon in self.instruments_on_ships[ship]:
                if weapon in instruments_needed:
                    useless = False
                    break
            if useless:
                useless_ships.append(ship)

        safe_percentage = 1
        behaviour_penalty = 1
        if action[0]=="use":
            return -10000
        elif action[0] == "turn_on":
            if useless_ships:
                if action[1] in useless_ships:
                    return 100000
            behaviour_penalty = 0.85
        elif action[0] == "calibrate":
            if useless_ships:
                if action[1] in useless_ships:
                    return 100000
            behaviour_penalty = 0.85
        elif action[0] == "move":
            if useless_ships:
                if action[1] in useless_ships:
                    return 100000
            safe_percentage = (self.neighbor_safe_percentage(state, action[2]))
            if safe_percentage < 0.6:
                h+=100*(1-safe_percentage)
            behaviour_penalty = 1.05 + (1-safe_percentage)
        #print("action = ", action, "->", safe_percentage,"->", h * behaviour_penalty)
        return (h * behaviour_penalty)

    def find_closest_distances_to_targets(self,state):
        count = 0
        for target in self.open_targets:  # distance from the closest ship to this target
            inst_needed = target.req_instrument
            distance_dict = {}
            for ship in state.ships:
                for weapon_on_ship in self.instruments_on_ships[ship]:
                    on = state.working_instruments[ship]==inst_needed
                    calibrated = state.calibrated_instruments[ship]
                    ship_pos = state.positions[ship]
                    if inst_needed == weapon_on_ship:
                        if on and calibrated:
                            straight_line = self.is_target_in_straight_line(ship_pos,target.pos)
                            obstacles = self.is_obstacles_on_the_way(state,ship,target.pos)
                            distance_from_target = axis_distance(ship_pos,target.pos)
                            distance_from_target+= 4 if straight_line and obstacles else 0
                            distance_dict[ship] = distance_from_target
                        else:
                            distance_from_target = axis_distance(ship_pos, self.calibration_targets[inst_needed])
                            distance_from_target+= axis_distance(self.calibration_targets[inst_needed], target.pos)
                            distance_dict[ship] = distance_from_target + 1 if on else 0

            if distance_dict:
                # print("closest")
                closest = min(distance_dict, key=distance_dict.get)
                count += distance_dict[closest]
                if not state.working_instruments[closest]==inst_needed:
                    count += 1
            else:
                count=1000
        return count

    def count_working_and_calibrated(self,state):
        add = lambda k,v: 1 if v else 0
        count = 0
        for (k,v) in state.calibrated_instruments.items():
            count += add(k,v)
        for (k,v) in state.working_instruments.items():
            count += add(k,v)
        return count

    def actions(self, state):
        actions = []
        for spaceship in state.ships:
            if state.calibrated_instruments[spaceship]:
                use_actions = self.get_use_actions_for_ship(state, spaceship)
                if use_actions:
                    return tuple(use_actions)

            elif state.working_instruments[spaceship]:
                calibrate_actions = self.get_calibrate_actions_for_ship(state, spaceship)
                if calibrate_actions:
                    actions.extend(calibrate_actions)

            turn_on_actions = self.get_turn_on_actions_for_ship(state, spaceship)
            if turn_on_actions:
                # print("turn_on_actions=",turn_on_actions)
                actions.extend(turn_on_actions)

            move_actions = self.get_move_actions_for_ship(state, spaceship)
            if move_actions:
                actions.extend(move_actions)
        if not actions:
            actions.append(("reset",))
        return tuple(actions)

    def get_use_actions_for_ship(self, state, ship):
        actions = []
        for target in self.open_targets:
            condition1 = target.req_instrument == state.working_instruments[ship]
            if condition1:
                condition3 = self.is_target_in_straight_line(state.positions[ship], target.pos)
                condition4 = not self.is_obstacles_on_the_way(state, ship, target.pos)
                if condition3 and condition4:
                    actions.append(("use", ship, target.req_instrument, target.pos))
        return actions

    def get_calibrate_actions_for_ship(self,state, ship):
        weapon = state.working_instruments[ship]
        target_pos = self.calibration_targets[weapon]
        condition3 = self.is_target_in_straight_line(state.positions[ship], target_pos)
        condition4 = not self.is_obstacles_on_the_way(state, ship, target_pos)
        if condition3 and condition4:
            return [("calibrate", ship, weapon, target_pos)]

    def get_turn_on_actions_for_ship(self,state,ship):
        actions = []
        for weapon in self.instruments_on_ships[ship]:
            if weapon==state.working_instruments[ship]:
                continue
            target_pos = self.calibration_targets[weapon]
            condition3 = self.is_target_in_straight_line(state.positions[ship], target_pos)
            condition4 = not self.is_obstacles_on_the_way(state, ship, target_pos)
            if condition3 and condition4:
                actions.append(("turn_on", ship, weapon))
        return actions

    def get_move_actions_for_ship(self, state, ship):
        move_actions = []
        ship_pos = tuple(state.positions[ship])
        neighbors = self.get_neighbors(ship_pos)
        # obstacles = self.get_obstacles(state)
        for neighbor in neighbors:
            if (neighbor not in self.obstacles):
                move_actions.append(("move", ship, ship_pos, neighbor))
        return move_actions

    def get_obstacles(self,state):
        r_targets = list(self.all_real_targets_positions)
        c_targets = list(self.calibration_targets.values())
        positions = [tuple(v) for v in state.positions.values()]

        obstacles = r_targets + c_targets + positions
        return obstacles

    def neighbor_safe_percentage(self,state, neighbor):
        #todo: consider lasers
        percentage = 1

        percentage -= 0.8 * self.check_laser_on_chances(state,neighbor)

        neighbors = set(self.get_neighbors(neighbor))
        illegal_neighbors = set(self.get_all_neighbors(neighbor)).difference(neighbors)
        if illegal_neighbors:
            percentage -= 0.1 * len(illegal_neighbors)

        obstacles = set(self.obstacles)
        obstacles_neighbors = neighbors.intersection(obstacles)
        if obstacles_neighbors:
            percentage -= 0.1 * len(obstacles_neighbors)

        for possible_neighbor in neighbors:
            percentage -= 0.1 * self.check_laser_on_chances(state, possible_neighbor)

        if percentage<0:
            return 0
        return percentage

    def check_laser_on_chances(self, state,pos):
        #Calculates the chances a laser will be on in the position in the next turn
        (a, b, c) = pos
        lasers = ((-1, b, c), (a, -1, c), (a, b, -1))
        count = 0
        for laser in lasers:
            if laser in state.lasers:
                if state.lasers[laser]:
                    count+= 1
                else:
                    count+= 0.3
        return count

    def get_all_neighbors(self,pos):
        neighbors = []
        (x, y, z) = pos
        neighbors.append((x + 1, y, z))
        neighbors.append((x - 1, y, z))
        neighbors.append((x, y + 1, z))
        neighbors.append((x, y - 1, z))
        neighbors.append((x, y, z + 1))
        neighbors.append((x, y, z - 1))
        return neighbors

    def is_target_in_straight_line(self, spaceship_pos, target_pos):
        f1 = lambda a, b: not a - b
        (a, b, c) = spaceship_pos
        (x, y, z) = target_pos
        result = (f1(x, a), f1(y, b), f1(z, c))
        if sum(result) > 1:
            return True
        return False

    def is_obstacles_on_the_way(self, state, ship, target_pos):
        """Returns False for no obstacles"""
        #TODO: change obstacles to include targets that have been taken down
        # obstacles = self.get_obstacles(state)
        (a, b, c) = state.positions[ship]
        (x, y, z) = target_pos
        for obstacle in self.obstacles:
            if obstacle != target_pos:
                (k, l, m) = obstacle
                if x != a and y == b == l and z == c == m:
                    if ((a < k) and (k < x)) or ((x < k) and (k < a)):
                        return True
                elif x == a == k and y != b and z == c == m:
                    if ((b < l) and (l < y)) or ((y < l) and (l < b)):
                        return True
                elif x == a == k and y == b == l and z != c:
                    if ((c < m) and (m < z)) or ((z < m) and (m < c)):
                        return True
        return False

    def get_neighbors(self, pos):
        neighbors = []
        (x, y, z) = pos
        if x + 1 < self.dim:
            neighbors.append((x + 1, y, z))
        if x > 0:
            neighbors.append((x - 1, y, z))
        if y + 1 < self.dim:
            neighbors.append((x, y + 1, z))
        if y > 0:
            neighbors.append((x, y - 1, z))
        if z + 1 < self.dim:
            neighbors.append((x, y, z + 1))
        if z > 0:
            neighbors.append((x, y, z - 1))
        return neighbors

    def choose_next_action(self, state):
        """Choose next action for driverlog controller given the current state.
        Action should be returned in the format described previous parts of the project"""
        state = Tuple_state(*state)
        self.print_state(state)

        if self.steps>=self.max_steps:
            return None
        else:
            self.steps+=1

        if self.last_action:
            self.update_world(state)

        """Find child to open"""
        f = self.h
        actions = self.actions(state)
        h_actions_dict = {}
        for action in actions:
            h_action = f(state,action)
            h_actions_dict[tuple(action)] = h_action

        min_value = min(h_actions_dict.values())
        min_keys = [k for k in h_actions_dict if h_actions_dict[k] == min_value]
        chosen_action = random.choice(min_keys)

        print("---Chosen action: ", chosen_action, "=>", min_value,"---")
        self.last_state = state
        self.last_action = chosen_action
        return chosen_action

        # if self.check_action(chosen_action):
        #     self.last_action = chosen_action
        # todo: choose something to do if it's not valid either
        # todo: deal with the problem that there is no solution or the solution is bad
        #     print('COMPLETE choose_next_action', chosen_action)
        #     return chosen_action
        # else:
        #     print('PROBLEM choose_next_action', chosen_action)
        #     return None

    def update_world(self, state):
        show_changes = True

        #check what changed and update reward
        reward = state.reward-self.reward
        if show_changes:
            print("Reward: ", reward)
            print("Steps: ", self.steps)
        #todo: check other options for reward
        exploded = set()
        if reward==-100:
            pass
        #todo
        elif reward < 0:
            exploded = self.get_newly_exploded_ships(state.ships)
            self.exploded_ships.update(exploded)
            if show_changes and exploded:
                print("Exploded: ", exploded)
        elif reward == 50:
            self.update_targets()
        self.reward=state.reward

        self.obstacles = self.get_obstacles(state)

        if self.last_action[0]=="move":
            # self.positions = state.positions #update the positions of the ships
            (command,ship,pos1,pos2) = self.last_action
            if ship not in exploded:
                pos2 = tuple(pos2)
                real_pos2 = tuple(state.positions[ship])
                if real_pos2 != pos2:
                    print("******Ship ", ship, "in pos", real_pos2, "instead of", pos2,"******")

    # def find_ship_and_target(self, ships):
    #     open_targets = list(self.open_targets)
    #     for target in open_targets:
    #         for ship in ships:
    #             if target.req_instruments in self.instruments_on_ships[ship]:
    #                 return (ship, target)
    #     return (-1,-1)

    # def check_action(self,action):
    #     """Takes an action and returns True if the action is valid"""
    #     #todo: understand how to decide, plus what to do with lasers
    #     #todo:add probability function
    #     #todo: make sure all the other options are true
    #     check = {"move": self.check_move_action, "turn_on": True, "calibrate": True, "use": True}
    #     return check[action[0]](action)

    # def check_move_action(self,action):
    #     (command, ship, pos1, pos2) = action
    #     condition1  = tuple(self.positions[ship]) == pos1       #is_ship_in_pos
    #     (x,y,z) = pos2
    #     laser_representation = [(x,-1,z),(x,y,-1),(-1,y,z)]
    #     condition2 = lambda x: x in self.all_lasers_pos and self.lasers_status[x]
    #
    #     for x in laser_representation:
    #         if condition2(x):
    #             print("LASER")
    #             return False
    #
    #     return condition1

    def get_newly_exploded_ships(self, new_ships):
        ships = set(self.last_state.ships)
        exploded = ships.difference(new_ships)
        return exploded

    def update_targets(self):
        target = Target(self.last_action[3], self.last_action[2])
        self.closed_targets.add(target)
        self.open_targets.remove(target)

    def get_spaceship_problem(self,state):
        """Returns a representation of spaceship problem"""
        def get_ex1_problem_targets_rep():
            targets = {}
            for key in self.all_real_targets_positions:
                targets.setdefault(key, [])
            for k, v in self.open_targets:
                targets.setdefault(k, []).append(v)
            return dict_list_to_dict_tuple(targets)
        targets = get_ex1_problem_targets_rep()
        positions = dict_list_to_dict_tuple(state.positions)

        problem = (self.dim, state.ships, self.all_instruments, self.instruments_on_ships,
                   self.calibration_targets, targets , positions, state.lasers)
        return problem

    def print_state(self, state):
        print("------------------------------------------------")
        for item in state:
            print(item)
        print("------------------------------------------------")

def dict_list_to_dict_tuple(d):
    final_dict = {}
    for k, v in d.items():
        final_dict[k] = tuple(d[k])
    return final_dict

def make_set_targets_from_dict(targets):
    targets_set = set()
    for pos, instruments in targets.items():
        for instrument in instruments:
            targets_set.add(Target(pos,instrument))
    return targets_set

#_______________________________________________________________
def timeout_exec(func, args=(), kwargs={}, timeout_duration=10, default=None):
    """This function will spawn a thread and run the given function
    using the args, kwargs and return the given default value if the
    timeout_duration is exceeded.
    """
    import threading
    class InterruptableThread(threading.Thread):
        def __init__(self):
            threading.Thread.__init__(self)
            self.result = default

        def run(self):
            try:
                self.result = func(*args, **kwargs)
            except Exception as e:
                self.result = (-3, -3, e)

    it = InterruptableThread()
    it.start()
    it.join(timeout_duration)
    if it.isAlive():
        return default
    else:
        return it.result

def check_spaceship_problem(p, search_method, timeout):
    """ Constructs a problem using ex1.create_poisonserver_problem,
    and solves it using the given search_method with the given timeout.
    Returns a tuple of (solution length, solution time, solution)
    (-2, -2, None) means there was a timeout
    (-3, -3, ERR) means there was some error ERR during search"""

    t1 = time.time()
    s = timeout_exec(search_method, args=[p], timeout_duration=timeout)
    t2 = time.time()

    if isinstance(s, search.Node):
        solve = s
        solution = list(map(lambda n: n.action, solve.path()))[1:]
        return (len(solution), t2 - t1, solution)
    elif s is None:
        return (-2, -2, None)
    else:
        return s

def solve_problems(problems,ship,target,goal):
    solved = 0
    print("Solving ex1")
    try:
        p = our_ex1_utils_for_ex3.create_spaceship_problem(problems,ship,target,goal)
    except Exception as e:
        print("Error creating problem: ", e)
        return None
    timeout = 60
    result = check_spaceship_problem(p, (lambda p: search.best_first_graph_search(p, p.h)), timeout)
    # print("GBFS ", result)
    if result[2] != None:
        if result[0] != -3:
            solved += 1
    print("GBFS Solved ", solved)
    if solved:
        return result[2]
    else:
        print("Couldn't solve problem->Returned reset")
        return [("reset",)]
    #todo: what to do?
#_______________________________________________________________
def check_problem(problem, initial_state):
    controller = SpaceshipController(problem, initial_state)
    # print("got state", state)
    action = controller.choose_next_action(initial_state)
    print("returning action", action)
    return action
    # try:
    #     action = controller.choose_next_action(initial_state)
    #     print("returning action", action)
    #     return action
    # except:
    #     print("Error check_solution")

def axis_distance(ship_pos, target_pos):
    (xA, yA, zA) = ship_pos
    (xB, yB, zB) = target_pos
    distance = 0
    if xA != xB:
        distance += abs(xA - xB)
    if yA != yB:
        distance += abs(yA - yB)
    if zA != zB:
        distance += abs(zA - zB)
    if (xA == xB) and (yA == yB):
        return 0
    if (xA == xB) and (zA == zB):
        return 0
    if (yA == yB) and (zA == zB):
        return 0
    return distance

# if __name__ == '__main__':
#     start = time.time()
#     problem = (
#         5, #0 Dimensions
#         ["Sidonia", "Enterprise"], #1 Ships
#         ("rangefinder", "thermal_camera"), #2 Instruments
#         {"Sidonia": ("rangefinder", ), "Enterprise": ("thermal_camera", "rangefinder")}, #3  Instruments on ships
#         {"rangefinder": (0, 3, 2), "thermal_camera": (1, 4, 2)}, #4 Calibration targets
#         {(1, 4, 1): ("thermal_camera", "rangefinder"), (4, 3, 4): ("thermal_camera", "rangefinder")}, #5 Targets
#         {"Sidonia": (4, 1, 1), "Enterprise": (3, 0, 3)}, #6 Positions
#         ((-1, 2, 2), (4, 4, -1)), #7 Lasers
#         20, #8 Steps
#         180 #9 Time
#     )
# # try:
#     calibrated = {x: None for x in problem[1]}
#     calibrated_as_output = {}
#     for ship, instrument in calibrated.items():
#         if instrument is not None:
#             calibrated_as_output[ship] = True
#     check_solution(problem, (problem[1], problem[6], {x: False for x in problem[7]}, {x: None for x in problem[1]}, calibrated_as_output, 0))
#
#     # except Exception as e:
#         # print(e)
#     finish = time.time()
#     print("Overall time elapsed:", finish-start)

#_____________________________________________________________