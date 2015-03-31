import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *
from planning.fsm import *
import math

class Planner:    

    def __init__(self, world, robot, role, fsmSpecFilePaths):

        logger = createLogger("PlannerLogging")
        logger.info("Using the following finite state machine spec files:")
        logger.newline()
        for path in fsmSpecFilePaths:
            logger.info(path)
        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []

        logger.info("-------- READING AND PARSING FSM SPEC FILES --------")
        for pathStr in fsmSpecFilePaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()
                if len(inputText) == 0:
                    logger.info("FSM FILE INPUT ERROR: Spec file '" + str(filePath) + "' is empty.")
                    fsmList.append(False)
                else: 
                    parseRes = grammar.parseString(inputText)
                    self._fsmList.append(createFSM(parseRes, pathStr, logger))


        if False in self._fsmList:
            logger.newline()
            logger.info("-------- PARSING FAILURE - Terminating due to the parse errors detailed above --------")
            logger.newline()
            quit()
        else:
            logger.newline()
            logger.info("-------- SPEC FILE INPUT AND PARSING COMPLETED SUCCESSFULLY --------")
            logger.newline()

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
        logger.info("------------ plan step ------------")
        logger.newline()
        for fsm in self._fsmList:
            truths=self.checkTrueConditions(fsm)    
            fsm.consumeInput(truths, self._world, self._robot, self._role)
        logger.info("---------- end plan step ----------")
        logger.newline()

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
