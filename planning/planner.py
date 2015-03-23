import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparsing import *

class Planner:    

    def checkTrueConditions(self, world):
        trueCnds = []
        lambdas = self._fsm.getLambdaDic()
        for letter in self._fsm.getAlpha():
            if letter in lambdas.keys() and lambdas[letter](world) == True:
                trueCnds.append(letter)
        return trueCnds

    def plan(self, world):
        truths=self.checkTrueConditions(world)
        self._fsm.consumeInput(truths, self._world, self._robot, self._role)

    def __init__(self, world, robot, role, fsmSpecFilePath):

        grammar = createConfigGrammar()
        inFile = open(fsmSpecFilePath, 'r')
        inputText = inFile.read()
        parseRes = grammar.parseString(inputText)
    
        # We're done reading files, so close them
        inFile.close()

        self._fsm = createFSM(parseRes)
        self._world = world
        self._robot=robot
        self._role = role
        self._fsm.show()