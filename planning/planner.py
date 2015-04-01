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

        logger = createLogger("Planner") # Create a logger object to use

        logger.info("Using the following finite state machine spec files to begin execution with:")
        logger.newline()
        for path in fsmSpecFilePaths:
            logger.info(path)

        logger.newline()
        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []
        self._readTexts=[]


        logger.info("-------- READING AND PARSING FSM SPEC FILES --------")
        for pathStr in fsmSpecFilePaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()
                
                if inputText in self._readTexts:
                    logger.warning("ERROR: Identified that two input spec files are identical.")
                self._readTexts.append(inputText)

                if len(inputText) == 0:
                    logger.error("FSM FILE INPUT ERROR: Spec file '" + str(filePath) + "' is empty.")
                    self._fsmList.append(False)
                else: 
                    parseRes = grammar.parseString(inputText)
                    self._fsmList.append(createFSM(parseRes, pathStr, logger))

        # If False is within the list of fsms, then there has been a parse error
        if False in self._fsmList:
            logger.newline()
            logger.error("-------- PARSING FAILURE - Terminating due to the parse errors detailed above --------")
            logger.newline()
            quit()
        else:
            logger.newline()
            logger.info("-------- SPEC FILE INPUT AND PARSING COMPLETED SUCCESSFULLY --------")
            logger.newline()

        self._world = world
        self._robot = robot
        self._role  = role
        
        showFsms()

        logger.info("-------- COMMENCING PLANNING OPERATIONS --------")
        logger.newline()
        logger.newline()

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
        in the current world state for each machine, and provide that machine with the letters for consumption

        If the machine responds with something other than None, this means that particular one has finished
        running and entered it's final state. As such we look up the next machine to begin running,
        and set it active so that it will run in future plan() calls"""

        logger.info("------------ plan step ------------")
        logger.newline()
        for fsm in self._fsmList:
            truths=self.checkTrueConditions(fsm)    
            machineResponse = fsm.consumeInput(truths, self._world, self._robot, self._role)
            if machineResponse != None:
                for fsm in self._fsmList:
                    if fsm.definingFile == machineResponse:
                        fsm.setActive()
        logger.info("---------- end plan step ----------")
        logger.newline()

    def showFsms(self):
        logger.newline()
        logger.info("-------- Planning FSMs --------")

        for fsm in self._fsms:
            fsm.show()
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

   

