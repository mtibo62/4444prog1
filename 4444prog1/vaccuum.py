import Astar

class Problem(object):
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return Astar.is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

class Vacuum(Problem):
    """ The problem of a 3x3 board with dirty tiles. The top 3 tiles are dirty
    and the vaccuum starts in the middle tile We aim to have all the tiles cleaned by the end"""

    def __init__(self, initial, goal  = ()):
        """ Define goal state and initialize a problem """
        #print(initial)
        self.goal = goal
        Problem.__init__(self, initial, goal)


    def find_agent(self, state):
        """Return the index of the vaccuum in a given state"""
        return state.index(2)

    def find_dirty_tile(self, state):
        """The desired purpose of this method would be to locate the closest dirty
        tile to the vacuum and then base the actions and result of the next move off of that.
        However,I was unable to complete this method or figure out how it will be incorporated
        into the final solution of result"""
        return state.index(1)


    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only five possible actions
        in any given state of the environment """

        #print(state[0])
        #print(state[1])

        #conversion of the tuple into a list so we can get values from it
        tupleList = list(state)

        vacuumx = state[0][0]
        vacuumy = state[0][1]

        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT', 'SUCK']


        vacuumLocation = 3*vacuumy +vacuumx

        if vacuumx == 0:
            possible_actions.remove('LEFT')
        if vacuumy  == 0:
            possible_actions.remove('UP')
        if vacuumx == 2:
            possible_actions.remove('RIGHT')
        if vacuumy == 2:
            possible_actions.remove('DOWN')
        if vacuumLocation not in  tupleList[1]:
            possible_actions.remove('SUCK')
        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """

        new_state = list(state)
        vaccCoord = list(new_state[0])
        dirtyLocs = list(new_state[1])
        tempDirtyList = dirtyLocs
        vacuumx = vaccCoord[0]
        vacuumy = vaccCoord[1]
        #print(vaccCoord)
        #print(dirtyLocs)

        if action == 'LEFT':
            vacuumx = vacuumx - 1
        if action == 'UP':
            vacuumy = vacuumy - 1
        if action == 'RIGHT':
            vacuumx = vacuumx + 1
        if action == 'DOWN':
            vacuumy = vacuumy + 1
        elif action == 'SUCK':
            index = 3 * vacuumy + vacuumx
            dirtyLocs.remove(index)

        return ((vacuumx,vacuumy),tuple(dirtyLocs))

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """

        return state[1] == self.goal

    def check_solvability(self, state):
        """ Checks if the given state is solvable """

        inversion = 0
        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if (state[i] > state[j]) and state[i] != 0 and state[j] != 0:
                    inversion += 1

        return inversion % 2 == 0

    def h(self, node):
        """ Return the heuristic value for a given state. Default heuristic function used is
        h(n) = number of dirty tiles """
        return 0



############
#This si the problem thast has been presented
###########
h = 1
problem = Vacuum(((1,1),(0,1,2)))

aStar = Astar.astar_search(problem)

print(aStar.solution())
print(aStar.path())
print(aStar)