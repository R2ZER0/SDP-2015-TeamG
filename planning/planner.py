import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *

class Planner:    

    def checkTrueConditions(self, world):
        """An FSM accepts a list of letters from it's alphabet which reflect conditions in the world
        that are currently true. To build this list, we iterate through the lambdas corresponding to 
        the letters. For each lambda that results in 'True', we know that the associated letter 
        represents a true condition in the world. We proceed and add this letter to the accumulating 
        list"""
        trueCnds = []
        lambdas = self._fsm.getLambdaDic()
        for letter in self._fsm.getAlpha():
            if letter in lambdas.keys() and lambdas[letter](world) == True:
                trueCnds.append(letter)
        return trueCnds

    def plan(self, world):
        """The heart of the planner. For every call, we determing a list of FSM letters which hold
        in the current world state, and provide this to the machine for consumption"""
        truths=self.checkTrueConditions(world)
        self._fsm.consumeInput(truths, self._world, self._robot, self._role)

    def __init__(self, world, robot, role, fsmSpecFilePaths):

        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []

        # filelist = []
        for pathStr in fsmSpecFilePaths:
            fileObj.append(open(pathStr, 'r'))
            inputText = fileObj.read()
            parseRes = grammar.parseString(inputText)
            inFile.close()      # We're done reading files, so close them
            self._fsmList.append(createFSM(parseRes, pathStr))

        if False in fsmList:
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