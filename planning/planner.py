import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *
from planning.fsm import *
import math

class Planner:    

    def __init__(self, world, robot, role, fsmSpecFilePaths, dynPlansPaths):

        logger = createLogger("Planner")
        logger.info("Using the following finite state machine spec files to begin execution with:")
        logger.newline()
        for path in fsmSpecFilePaths:
            logger.info(path)

        logger.newline()

        logger.info("The following finite state machine spec files will be available to dynamically switch to:")
        logger.newline()
        for dynPath in dynPlansPaths:
            logger.info(path)
        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []
        self._dynFsmList = []
        self._readTexts=[]


        logger.info("-------- READING AND PARSING FSM SPEC FILES --------")
        for pathStr in fsmSpecFilePaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()
                
                if inputText in self._readTexts:
                    logger.warning("WARNING: Identified that two input spec files are identical. This could result in VERY undesirable behaviour.")
                self._readTexts.append(inputText)

                if len(inputText) == 0:
                    logger.error("FSM FILE INPUT ERROR: Spec file '" + str(filePath) + "' is empty.")
                    self._fsmList.append(False)
                else: 
                    parseRes = grammar.parseString(inputText)
                    self._fsmList.append(createFSM(parseRes, pathStr, logger))

        for pathStr in dynPlansPaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()

                if inputText in self._readTexts:
                    logger.warning("WARNING: Identified that two input spec files are identical. This could result in VERY undesirable behaviour.")
                self._readTexts.append(inputText)
                
                if len(inputText) == 0:
                    logger.error("FSM FILE INPUT ERROR: Spec file '" + str(filePath) + "' is empty.")
                    self._dynFsmList.append(False)
                else: 
                    parseRes = grammar.parseString(inputText)
                    self._dynFsmList.append(createFSM(parseRes, pathStr, logger))


        if False in self._fsmList or False in self._dynFsmList:
            logger.newline()
            logger.error("-------- PARSING FAILURE - Terminating due to the parse errors detailed above --------")
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
        logger.info("-------- Planning FSMs --------")
        logger.newline()
        logger.newline()
        for fsm in self._fsmList:
            fsm.show()

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
        in the current world state, and provide this to the machine for consumption"""
        logger.info("------------ plan step ------------")
        logger.newline()
        for fsm in self._fsmList:
            truths=self.checkTrueConditions(fsm)    
            machineResponse = fsm.consumeInput(truths, self._world, self._robot, self._role)
            if machineResponse != None:
                for fsm in self._dynFsmList:
                    if fsm.definingFile == machineResponse:
                        self._fsmList.append(fsm)
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

    def showFsms(self):

        logger.newline()
        logger.info("FSMs currently in use:")

        for fsm in self._fsms:
        fsm.show()
        logger.newline()

        logger.info("FSMs that began execution with the planner:")
        for fsm in self._fsmList:
        fsm.show()
        logger.newline()

        logger.info("FSMs available to be used dynamically:")
        for fsm in self._dynFsmList:
        fsm.show()
        logger.newline()
