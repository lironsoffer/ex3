import time
from threading import Timer
import random
from collections import namedtuple
# import search

ids = ["307915462", "204858823"]
Tuple_state = namedtuple('Tuple_state', 'ships, positions, lasers, working_instruments, calibrated_instruments, reward')
Target = namedtuple('Target', 'pos, req_instrument')

LASER_TURNING_ON_CHANCES = 0.3
CHOSEN_ACTION_MIN_SAFETY = 0.7 # When lower makes the agent take more risk
# USE_behaviour_penalty = 1
TURN_ON_behaviour_penalty = 0.85
MOVE_behaviour_penalty = 1.05
CALIBRATE_behaviour_penalty = 0.8
MOVE_safe_percentage_to_effect_h = 0.74

LOG = True

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

def end_run():
    return None

def create_last_moves(ships):
    dic={}
    for ship in ships:
        dic[ship]=None
    return dic

class SpaceshipController:
    "This class is a controller for a spaceship problem."

    def __init__(self, problem, initial_state):
        """Initialize controller for given the initial setting.
        This method MUST terminate within the specified timeout."""
        self.timeout = problem[9]
        # self.timer = Timer(self.timeout, end_run)
        # self.timer.start()  # after self.timeout seconds, returns None (illegal action)
        self.max_steps=problem[8]
        self.problem = problem
        self.saved_initial = initial_state

        self.reward = 0
        self.steps = 0

        self.dim = problem[0]
        self.ships = problem[1]
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

        state = Tuple_state(*initial_state)
        self.last_state = state
        self.last_action = None

        self.obstacles = self.get_obstacles(state)
        self.less_obstacles = self.get_less_obstacles()
        self.update_instruments_needed_and_useless_ships(state)
        self.last_moves = create_last_moves(self.ships)

        print('COMPLETE init ')
        print("Req Reward = {}".format(self.dim*20))

    def reward(self, state):
        pass

    def h(self, state, action):
        """This is the heuristic. It evaluates the action"""
        if action==("reset",):
            return 0

        action_ship = action[0]
        h=self.find_closest_distance_to_targets(state, action_ship)
        if self.open_targets:
            targets_factor = (len(self.open_targets)/len(self.all_targets))
        else:
            targets_factor=0
        h*=targets_factor
        # h-=len(self.closed_targets)
        # h-=self.count_working_and_calibrated(state)

        self.update_instruments_needed_and_useless_ships(state)
        useless_ships = self.useless_ships
        safe_percentage = 1
        behaviour_penalty = 1
        if action[0]=="use":
            return -10000
        elif action[0] == "turn_on":
            if useless_ships:
                if action[1] in useless_ships:
                    return 100000
            behaviour_penalty = TURN_ON_behaviour_penalty
        elif action[0] == "calibrate":
            if useless_ships:
                if action[1] in useless_ships:
                    return 100000
            behaviour_penalty = CALIBRATE_behaviour_penalty
        elif action[0] == "move":
            (command,ship,pos1,pos2) = action
            if useless_ships:
                if ship in useless_ships:
                    return 100000
            safe_percentage = (self.neighbor_safe_percentage(state, pos2,pos1))
            if safe_percentage < MOVE_safe_percentage_to_effect_h:
                h+=100*(1-safe_percentage)
            behaviour_penalty = MOVE_behaviour_penalty #+ (1-safe_percentage)
        if LOG: print("action = ", action, "->", safe_percentage,"->", h * behaviour_penalty)
        return (h * behaviour_penalty)

    def update_instruments_needed_and_useless_ships(self, state):
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

        self.instruments_needed = instruments_needed
        self.useless_ships = useless_ships

    def find_closest_distance_to_targets(self, state, action_ship):
        count = 0
        distance_dict = {}
        min_distance = self.dim * 3
        for target in self.open_targets:  # distance from the closest ship to this target
            inst_needed = target.req_instrument
            if action_ship in state.ships and inst_needed in self.instruments_on_ships[action_ship]:
                min_distance = self.find_closest_distance_from_ship_to_target(state,action_ship,target)
                distance_dict[target] = min_distance

        if distance_dict:
            closest = min(distance_dict, key=distance_dict.get)
            min_distance = distance_dict[closest]
        return min_distance

    def find_closest_distance_from_ship_to_target(self, state, ship, target):
        inst_needed = target.req_instrument
        distance_dict = {}
        for weapon_on_ship in self.instruments_on_ships[ship]:
            on = state.working_instruments[ship] == inst_needed
            calibrated = state.calibrated_instruments[ship]
            ship_pos = state.positions[ship]
            if inst_needed == weapon_on_ship:
                if on and calibrated:
                    # straight_line = self.is_target_in_straight_line(ship_pos,target.pos)
                    distance_from_target = distance(ship_pos, target.pos)
                    if distance_from_target==0:
                        obstacle = self.is_obstacles_on_the_way(state, ship, target.pos)
                        if obstacle:
                            distance, newP = calculate_projection_distance(ship_pos, obstacle)
                            distance_from_target = distance + distance(newP, target.pos)
                    # distance_from_target+= 4 if distance_from_target==0 and obstacle else 0
                    distance_dict[target] = distance_from_target
                else:
                    distance, newP = calculate_projection_distance(ship_pos, self.calibration_targets[inst_needed])
                    distance_from_target = distance(ship_pos, newP)
                    distance_from_target += distance(newP, target.pos)
                    distance_dict[target] = distance_from_target + 1 if on else 0

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
            # if weapon != state.working_instruments[ship]:
            #     actions.append(("turn_on", ship, weapon))
            if weapon==state.working_instruments[ship]:
                continue
            target_pos = self.calibration_targets[weapon]
            condition3 = self.is_target_in_straight_line(state.positions[ship], target_pos)
            condition4 = not self.is_obstacles_on_the_way(state, ship, target_pos)
            if condition3 and condition4:
                actions.append(("turn_on", ship, weapon))
        return actions

    def get_safe_turn_on_action(self,state):
        chances = {}
        for ship in state.ships:
            for weapon in self.instruments_on_ships[ship]:
                if weapon != state.working_instruments[ship]:
                    laser_on_chances = self.check_laser_on_chances(state,state.positions[ship])
                    if laser_on_chances<0.6:
                        return ("turn_on", ship, weapon)
                    chances[("turn_on", ship, weapon)] = laser_on_chances
        if chances:
            return min(chances, key=chances.get)
        else:
            return None

    def get_move_actions_for_ship(self, state, ship):
        move_actions = []
        ship_pos = tuple(state.positions[ship])
        neighbors = self.get_neighbors(ship_pos)
        # obstacles = self.get_obstacles(state)
        for neighbor in neighbors:
            # if (neighbor not in self.obstacles) and neighbor!=self.last_moves[ship]:
            if (neighbor not in self.obstacles):
                move_actions.append(("move", ship, ship_pos, neighbor))
        return move_actions

    def get_obstacles(self,state):
        r_targets = list(self.all_real_targets_positions)
        c_targets = list(self.calibration_targets.values())
        positions = [tuple(v) for v in state.positions.values()]

        obstacles = r_targets + c_targets + positions
        return obstacles

    def get_less_obstacles(self):
        r_targets = list(self.all_real_targets_positions)
        c_targets = list(self.calibration_targets.values())

        obstacles = r_targets + c_targets
        return obstacles

    def neighbor_safe_percentage(self,state, neighbor, pos1):
        #todo: consider lasers
        percentage = 1

        percentage -= 0.8 * self.check_laser_on_chances(state,neighbor)

        illegal = lambda a,b,c: (a<0) or (a>=self.dim) or (b<0) or (b>=self.dim) or (c<0) or (c>=self.dim)
        neighbors = self.get_all_neighbors(neighbor,pos1)
        illegal_neighbors = [neighbor for neighbor in neighbors if illegal(*neighbor)]
        if illegal_neighbors:
            percentage -= 0.1 * len(illegal_neighbors)

        obstacles = set(self.obstacles)
        obstacles_neighbors = obstacles.intersection(neighbors)
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
            if laser in state.lasers.keys():
                if state.lasers[laser]:
                    count+= 1
                else:
                    count+= LASER_TURNING_ON_CHANCES
        return count

    def get_all_neighbors(self,pos,last_pos):
        (x, y, z) = pos
        direction = (x-last_pos[0],y-last_pos[1],z-last_pos[2])
        if direction[0]:
            neighbors = [(x,y+1,z),(x,y+1,z+1),(x,y+1,z-1),
                     (x,y-1,z),(x,y-1,z+1),(x,y-1,z-1),
                     (x,y,z+1),(x,y,z-1)]
        elif direction[1]:
            neighbors = [(x + 1, y, z), (x + 1, y, z + 1), (x + 1, y, z - 1),
                         (x - 1, y, z), (x - 1, y, z + 1), (x - 1, y, z - 1),
                         (x, y, z + 1), (x, y, z - 1)]
        elif direction[2]:
            neighbors = [(x + 1, y, z), (x + 1, y + 1, z), (x + 1, y - 1, z),
                         (x - 1, y, z), (x - 1, y + 1, z), (x - 1, y - 1, z),
                         (x, y + 1, z), (x, y - 1, z)]
        else:
            neighbors = []
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
                        return (k,l,m)
                elif x == a == k and y != b and z == c == m:
                    if ((b < l) and (l < y)) or ((y < l) and (l < b)):
                        return (k,l,m)
                elif x == a == k and y == b == l and z != c:
                    if ((c < m) and (m < z)) or ((z < m) and (m < c)):
                        return (k,l,m)
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

        if self.steps>=self.max_steps:
            # self.timer.cancel()
            return None

        if self.last_action:
            self.update_world(state)

        """Find child to open"""
        f = self.h
        actions = self.actions(state)
        h_actions_dict = {}
        safe_percentage_dict = {}
        for action in actions:
            safe_percentage = 1
            #todo: consider changing for other actions
            h_action = f(state,action)
            if action[0]=="move":
                (command,ship,pos1,pos2) = action
                safe_percentage = self.neighbor_safe_percentage(state, pos2, pos1)
            safe_percentage_dict[action] = safe_percentage
            if LOG: ("{} %={}, h={}".format(action,safe_percentage,h_action))
            if safe_percentage!=0:
                h_action*=1/safe_percentage
                h_actions_dict[tuple(action)] = h_action

        if not h_actions_dict:
            min_value = 0
            chosen_action = self.get_safe_turn_on_action(state)
            if not chosen_action:
                print("NO SAFE ACTIONS!! returning reset")
                chosen_action = ("reset",)
        else:
            min_value = min(h_actions_dict.values())
            min_keys = [k for k in h_actions_dict if h_actions_dict[k] == min_value]
            chosen_action = random.choice(min_keys)

            steps_factor = self.steps/self.max_steps
            not_safe = safe_percentage_dict[chosen_action] < (CHOSEN_ACTION_MIN_SAFETY * steps_factor)
            if not_safe and min_value>0:
                if LOG: print("---Chosen action: {} => {} not safe enough---".format(chosen_action,min_value))
                chosen_action_try = self.get_safe_turn_on_action(state)
                if chosen_action_try:
                    chosen_action = chosen_action_try
        if LOG: print("---Chosen action: ", chosen_action, "=>", min_value,"---")
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
        #check what changed and update reward
        reward = state.reward-self.reward
        self.steps += 1
        if LOG:
            print("New Reward: ", reward)
            print("Total Reward: ", state.reward)
            print("Steps: ", self.steps)

        #todo: check other options for reward
        exploded = set()
        if reward==-100:
            self.open_targets.update(self.closed_targets)
            self.closed_targets.clear()
        elif reward < 0:
            exploded = self.get_newly_exploded_ships(state.ships)
            self.exploded_ships.update(exploded)
        elif reward == 50:
            self.update_targets()
        self.reward=state.reward

        self.obstacles = self.get_obstacles(state)
        self.update_instruments_needed_and_useless_ships(state)

        if self.last_action[0]=="move":
            # self.positions = state.positions #update the positions of the ships
            (command,ship,pos1,pos2) = self.last_action
            if ship not in exploded:
                pos2 = tuple(pos2)
                real_pos2 = tuple(state.positions[ship])
                if LOG and (real_pos2 != pos2):
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
        if LOG: print("{} is down!".format(target))
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
def distance(A,B):
    return two_dim_distance(A,B)

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

def two_dim_distance(ship_pos, target_pos):
    (a,b,c) = (abs(target_pos[0]-ship_pos[0]),abs(target_pos[1]-ship_pos[1]),abs(target_pos[2]-ship_pos[2]))
    distance = a + b + c - max(a,b,c)
    return distance

def calculate_projection_distance(self, p1, p2):
    # reutrns a tuple of (smallest_distance, new P)
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    distance, newP = min((abs(x1 - x2) + abs(y1 - y2), (x2, y2, z1)),
                         (abs(x1 - x2) + abs(z1 - z2), (x2, y1, z2)),
                         (abs(y1 - y2) + abs(z1 - z2), (x1, y2, z2))
                         )
    return (distance, newP)
# _______________________________________________________________
