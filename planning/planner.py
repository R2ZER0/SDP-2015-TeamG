import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *
from planning.fsm import *
import math

def atLeastOneInitiallyActiveFsm(fsms):
    """Ensure there is at least one fsm that starts active - otherwise we'd get no activity!"""
    machinesThatStartActive = 0
    for fsm in fsms:
        if fsm.startingActiveState:
            return True
    return False

def checkFileReferenceErrors(fsms, logger):
    fileNames = []
    errorFound = False
    for fsm in fsms:
        fileNames.append(fsm.definingFile)

    for fsm in fsms:
        if not [fsm.nextPlanToInvoke] == ["NA"] and not set([fsm.nextPlanToInvoke]) <= set(fileNames):
            logger.info("PARSE ERROR: Found a reference to an unknown file. File '" + str(fsm.definingFile) + "' referenced a file '" + str(fsm.nextPlanToInvoke) + "', and this is unrecognised.")
            errorFound = True
    return errorFound

class Planner:    

    def __init__(self, world, robot, role, fsmSpecFilePaths):

        self._logger = createLogger("Planner") # Create a logger object to use for tracking operations

        self._logger.info(">>> Using the following finite state machine spec files to begin execution with:")
        self._logger.newline()
        for path in fsmSpecFilePaths:
            self._logger.info(path)

        self._logger.newline()
        
        grammar = createConfigGrammar()  # Build the grammar used to parse input config files
        self._fsmList = []               # List to store the FSMs the planner is to work with
        self._readTexts=[]               # Stores the text of each file that's been read - used to detect when we read in duplicates


        # We read each file specified by the 
        self._logger.info(">>> READING AND PARSING FSM SPEC FILES")
        for pathStr in fsmSpecFilePaths:
            with open(pathStr, 'r') as fileObj:
                inputText = fileObj.read()
                
                if inputText in self._readTexts:
                    # If we read something we've already seen, fail
                    self._logger.error("ERROR: Identified that two input spec files are identical.")
                    self._fsmList.append(False)
                self._readTexts.append(inputText)

                if len(inputText) == 0:
                    # Empty input config files are errors
                    self._logger.error("FSM FILE INPUT ERROR: Spec file '" + str(filePath) + "' is empty.")
                    self._fsmList.append(False)
                else: 
                    parseRes = grammar.parseString(inputText)
                    self._fsmList.append(createFSM(parseRes, pathStr, self._logger))

        # If False is within the list of fsms, then there has been a parse error
        if False in self._fsmList:
            self._logger.newline()
            self._logger.error(">>> PARSING FAILURE - Terminating due to the parse errors detailed above")
            self._logger.newline()
            quit()
        else:
            self._logger.newline()
            self._logger.info(">>> SPEC FILE INPUT AND PARSING COMPLETED SUCCESSFULLY")
            self._logger.newline()

        if not atLeastOneInitiallyActiveFsm(self._fsmList):
            self._logger.error(">>> PLANNER ERROR: There are no planner fsms that start active. This is useless - it means nothing will ever happen.")
            self._logger.newline()
            quit()
        elif checkFileReferenceErrors(self._fsmList, self._logger):
            quit()

        self._world = world
        self._robot = robot
        self._role  = role
        
        # Output what we're about to work with
        showFsms()

        self._logger.info(">>> COMMENCING PLANNING OPERATIONS")
        self._logger.newline()
        self._logger.newline()

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

        self._logger.info("------------ plan step ------------")
        self._logger.newline()
        for fsm in self._fsmList:
            truths=self.checkTrueConditions(fsm)    
            machineResponse = fsm.consumeInput(truths, self._world, self._robot, self._role)
            if machineResponse != None:
                for fsm in self._fsmList:
                    if fsm.definingFile == machineResponse:
                        fsm.setActive()
                        
        self._logger.newline()
        self._logger.info("---------- end plan step ----------")
        self._logger.newline()
        self._logger.newline()
        self._logger.newline()
        self._logger.newline()
        self._logger.newline()
        self._logger.newline()


    def showFsms(self):
        self._logger.newline()
        self._logger.info("-------- Planning FSMs --------")

        for fsm in self._fsmList:
            fsm.show()
        self._logger.newline()


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

   

