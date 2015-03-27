import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *
from planning.fsm import *

class Planner:    

    def __init__(self, world, robot, role, fsmSpecFilePaths):

        print "Using the following finite state machine spec files:"
        print
        for path in fsmSpecFilePaths:
            print path
        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []

        for pathStr in fsmSpecFilePaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()
                parseRes = grammar.parseString(inputText)
                self._fsmList.append(createFSM(parseRes, pathStr))

        if False in self._fsmList:
            print "Terminating due to parse errors detailed above..."
            print
            quit()

        # inFile = open(fsmSpecFilePath, 'r')
        # inputText = inFile.read()

        # parseRes = grammar.parseString(inputText)    # Using PyParsing, initiate a parse run of the input config file
    
        # inFile.close()      # We're done reading files, so close them

        # self._fsm = createFSM(parseRes) # Create the finite state machine object which the planner will use
        self._world = world
        self._robot=robot
        self._role = role
        # self._fsm.show()
        for fsm in self._fsmList:
            fsm.show()

    def checkTrueConditions(self, fsm):
        """An FSM accepts a list of letters from it's alphabet which reflect conditions in the world
        that are currently true. To build this list, we iterate through the lambdas corresponding to 
        the letters. For each lambda that results in 'True', we know that the associated letter 
        represents a true condition in the world. We proceed and add this letter to the accumulating 
        list"""
        trueCnds = []
        lambdas = fsm.getLambdaDic()
        for letter in fsm.getAlpha():
            if letter in lambdas.keys() and lambdas[letter](self) == True:
                trueCnds.append(letter)
        return trueCnds

    def plan(self):
        """The heart of the planner. For every call, we determing a list of FSM letters which hold
        in the current world state, and provide this to the machine for consumption"""
        print "------------ plan step ------------"
        print
        for fsm in self._fsmList:
            truths=self.checkTrueConditions(fsm)    
            fsm.consumeInput(truths, self._world, self._robot, self._role)
        print "---------- end plan step ----------"
        print

    @property
    def current_task(self):
        return self._fsmList[0].currentTask

    @property
    def current_state(self):
        return self._fsmList[0].currentState

    @property
    def world(self):
        '''Returns the world instance held by the Planner.'''
        return self._world

    @property
    def our_robot(self):
        '''Returns the Robot model pertaining to our role.'''
        return self.world.our_attacker if self._role == 'attacker' else self.world.our_defender
