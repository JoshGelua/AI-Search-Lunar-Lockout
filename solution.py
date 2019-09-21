#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
import random
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    return 0

def heur_manhattan_distance(state):
#OPTIONAL
    '''Manhattan distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses Manhattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the Manhattan distances between each xanadu and the escape hatch.
    center = int((state.width-1)/2)
    distance = 0
    if (isinstance(state.xanadus[0], int)):
        distance += abs(state.xanadus[0] - center)
        distance += abs(state.xanadus[1] - center)
    else:
        for robot in state.xanadus:
            test_center = list(robot)
            distance += abs(test_center[0] - center)
            distance += abs(test_center[1] - center)
    return distance

def heur_L_distance(state):
    #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses mahnattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the L distances between each xanadu and the escape hatch.
    center = int((state.width-1)/2)
    distance = 0
    if (isinstance(state.xanadus[0], int)):
        if state.xanadus[0] != center:
            distance += 1
        if state.xanadus[1] != center:
            distance += 1
    else:
        for robot in state.xanadus:
            test_center = list(robot)
            if test_center[0] != center:
                distance += 1
            if test_center[1] != center:
                distance += 1
    return distance

def heur_alternate(state):
    #IMPLEMENT
    '''a better lunar lockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Your function should return a numeric value for the estimate of the distance to the goal.
    center = int((state.width-1)/2)
    multiplier = 0
    ''' first and last are h-values associated with the steps explained below.
    4 and 3 were values that worked the fastest upon observation of combinations
    of 1-10 each. Values where last > first were noticeably slower than any
    other combination. '''
    first = 4 #STEP 1 H-Value
    last = 3 #STEP 2 H-Value
    if (isinstance(state.xanadus[0], int)):

        '''Here, dead states are set to a heuristic of infinity.
        Particularly, states in which the lone xanadu is
        isolated in the corner are dead, since any movement will result in
        unsolved successions. This was given as a tip by Randy on Piazza.
        '''
        dead_up_left = 1
        dead_up_right = 1
        dead_down_left = 1
        dead_down_right = 1
        no_adj = 1
        for robots in state.robots:
            if state.xanadus[0] > robots[0]:
                dead_up_right = 0
                dead_down_right = 0
            if state.xanadus[0] < robots[0]:
                dead_up_left = 0
                dead_down_left = 0
            if state.xanadus[1] > robots[1]:
                dead_up_left = 0
                dead_up_right = 0
            if state.xanadus[1] < robots[1]:
                dead_down_left = 0
                dead_down_right = 0
            if robots[0] == state.xanadus[0] or robots[1] == state.xanadus[1]:
                no_adj = 0
            if robots[0] == center and robots[1] == center:
                multiplier += 2
        if (dead_up_left or dead_down_right or dead_up_left or dead_up_right):
            return 10000000000000
        if no_adj == 1:
            multiplier += 2


        '''Here, successive states are separated into two steps:
        1) Xanadu is not aligned with either x or y center coordinates **
        ** This may or may not be skipped depending if the starting state places
        xanadu in one of the center coordinates.
        2) Xanadu that is aligned with ONE of either x or y center coordinates

        Additionally, for each position of xanadu, the idea is that it will
        only be of the same coordinate as the center if there is a robot that
        blocks xanadu so that it will align with the x or y center coordinate.

        This will happen with two parts, in the following order:
        1) xanadu that is not on the same coordinate as center-x or center-y
        ultimately will be aligned with the center with either one of the x or y
        center coordinate with the help of a robot in exactly one position.
        A lack thereof of that particular robot will result in an added heuristic
        value of 4.
        2) xanadu that is aligned atleast one of the center coordinates will
        be aligned with the second coordinate with the help of another robot in
        exactly one position. A lack thereof of that particular robot will result
        in an added heuristic value of 3.

        NOTE: Since these occur in steps 1-2, it is still admissible, as
        Step 1 h-value > Step 2 h-value, respectively.
         '''

        # X-Coordinate Alignment: Step 1
        if state.xanadus[0] != center:
            if state.xanadus[1] < center:
                for robot in state.robots:
                    if robot[0] == center:
                        if robot[1] != center + 1:
                            multiplier += first
            elif (state.xanadus[1] > center):
                for robot in state.robots:
                    if (robot[0] == center):
                        if (robot[1] != (center - 1)):
                            multiplier += first

        else: # Y-Coordinate Alignment: Step 2
            if state.xanadus[1] < center:
                for robot in state.robots:
                    if robot[0] == center:
                        if robot[1] != center + 1:
                            multiplier += last
            elif (state.xanadus[1] > center):
                for robot in state.robots:
                    if (robot[0] == center):
                        if (robot[1] != (center - 1)):
                            multiplier += last

        # Y-Coordinate Alignment: Step 1
        if (state.xanadus[1] != center):
            if (state.xanadus[0] < center):
                for robot in state.robots:
                    if (robot[1] == center):
                        if (robot[0] != (center + 1)):
                            multiplier += first
            elif (state.xanadus[0] < center):
                for robot in state.robots:
                    if (robot[1] == center):
                        if (robot[0] != (center - 1)):
                            multiplier += first

        else: # X-coordinate Alignment: Step 2
            if (state.xanadus[0] < center):
                for robot in state.robots:
                    if (robot[1] == center):
                        if (robot[0] != (center + 1)):
                            multiplier += last
            elif (state.xanadus[0] < center):
                for robot in state.robots:
                    if (robot[1] == center):
                        if (robot[0] != (center - 1)):
                            multiplier += last
    else:
        ''' Here, code is repeated to account for multiple xanadus.'''
        #DEAD_STATES
        no_adj = 1
        for xanadus in state.xanadus:
            test_center = list(xanadus)
            dead_up_left = 1
            dead_up_right = 1
            dead_down_left = 1
            dead_down_right = 1

            for robots in state.robots:
                if test_center[0] > robots[0]:
                    dead_up_right = 0
                    dead_down_right = 0
                if test_center[0] < robots[0]:
                    dead_up_left = 0
                    dead_down_left = 0
                if test_center[1] > robots[1]:
                    dead_up_left = 0
                    dead_up_right = 0
                if test_center[1] < robots[1]:
                    dead_down_left = 0
                    dead_down_right = 0
                if robots[0] == test_center[0] or robots[1] == test_center[1]:
                    no_adj = 0
                if robots[0] == center and robots[1] == center:
                    multiplier += 1
            if (dead_up_left or dead_down_right or dead_up_left or dead_up_right):
                return 10000000000000



            # STEP 1-2 Alignment
            if (test_center[0] != center):
                if (test_center[1] < center):
                    for robot in state.robots:
                        if (robot[0] == center):
                            if (robot[1] != (center + 1)):
                                multiplier += first
                elif (test_center[1] > center):
                    for robot in state.robots:
                        if (robot[0] == center):
                            if (robot[1] != (center - 1)):
                                multiplier += first
            else:
                if (test_center[1] < center):
                    for robot in state.robots:
                        if (robot[0] == center):
                            if (robot[1] != (center + 1)):
                                multiplier += last
                elif (test_center[1] > center):
                    for robot in state.robots:
                        if (robot[0] == center):
                            if (robot[1] != (center - 1)):
                                multiplier += last
            if (test_center[1] != center):
                if (test_center[0] < center):
                    for robot in state.robots:
                        if (robot[1] == center):
                            if (robot[0] != (center + 1)):
                                multiplier += first
                elif (test_center[0] < center):
                    for robot in state.robots:
                        if (robot[1] == center):
                            if (robot[0] != (center - 1)):
                                multiplier += first
            else:
                if (test_center[0] < center):
                    for robot in state.robots:
                        if (robot[1] == center):
                            if (robot[0] != (center + 1)):
                                multiplier += last
                elif (test_center[0] < center):
                    for robot in state.robots:
                        if (robot[1] == center):
                            if (robot[0] != (center - 1)):
                                multiplier += last
    if no_adj == 1:
        multiplier += 3
    total_distance = multiplier
    return total_distance


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a LunarLockoutState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    f_value = sN.gval + weight * sN.hval
    return f_value

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 2):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
 # Initiate the search engine.
  se = SearchEngine('custom', 'full')
  se.init_search(initial_state,
  goal_fn = lockout_goal_state,
  heur_fn = heur_alternate,
  fval_function = (lambda sN: fval_function(sN, weight)))

  search_start_time = os.times()[0]
  suboptimal = se.search(timebound)
  if suboptimal is not False:
      total_search_time = timebound - (os.times()[0] - search_start_time)
      best_yet = suboptimal
      weight = weight * 0.66
      while total_search_time > 0:
          start_time = os.times()[0]
          more_optimal = se.init_search(initial_state,
          goal_fn = lockout_goal_state,
          heur_fn = heur_alternate,
          fval_function = (lambda sN: fval_function(sN, weight)))
          total_search_time = total_search_time - (os.times()[0] - start_time) # Update remaining time.
          if more_optimal:
              weight = weight * 0.66
              best_yet = more_optimal
      return best_yet
  else:
      return False



def anytime_gbfs(initial_state, heur_fn, timebound = 2):
#OPTIONAL
  '''Provides an implementation of anytime greedy best-first search.  This iteratively uses greedy best first search,'''
  '''At each iteration, however, a cost bound is enforced.  At each iteration the cost of the current "best" solution'''
  '''is used to set the cost bound for the next iteration.  Only paths within the cost bound are considered at each iteration.'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  return 0

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3),)),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 9, ((2,2), (3,4), (4,5), (5,5), (5,0), (6,1), (6,4), (7,0), (8,1), (8,3)), ((0,0), (0,8))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")
  print("Running A-star")

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******")
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_alternate)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0;
  print("Running Anytime Weighted A-star")

  for i in range(len(PROBLEMS)):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    weight = 2
    final = anytime_weighted_astar(s0, heur_alternate, weight, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0;
  print("Running Anytime GBFS")

  for i in range(len(PROBLEMS)):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    final = anytime_gbfs(s0, heur_alternate, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")
