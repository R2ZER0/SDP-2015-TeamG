import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *
from pyparse import *

# Thomas, still to document

class Planner:

    def __init__(self, world, robot, role, fsmConfigFilepath):
        self._conditionLambdaBindings = consumeLambdas(lambdaConfigPath)
        self._grammar = createConfigGrammar()
        self._configFile = open(fsmConfigFilepath, 'r').read()

        self._parseResults =  grammar.parseString(configFile)
        self._fsm = createFSM(parseResults)

        checkLamdas(conditionLambdaBindings)

        self._fsm.show()

    def plan():
        for alph in fsm.getAlpha():
            if self._conditionLambdaBindings[a]:
                fsm.consumeInput(a)
        
    def checkLambdas(self, conditionLambdaBindings):
        for key in conditionLambdaBindings:
            if key not in self._fsm.getStates():
                print "Planner Error"
                quit()

    def consumeLambdas(self, path):
        lambdaConfig = open(path, 'r').readlines()
        strippedNewlines = [line.strip("\n") for line in lambdaConfig]
    

        code = ""
        dictionary={}

        for line in strippedNewlines:
            code = code + "dictionary.update({ " + line + " })\n"
    
        code_obj = compile(code, '<string>', 'exec')
        exec(code_obj)
        return dictionary

    def createConfigGrammar(self):
        # FSM grammar
        inAlphKWD     = Literal("inAlph")
        finalSKWD     = Literal("finalState")
        statesKWD     = Literal("states")
        initSKWD      = Literal("initialState")
        nameKWD       = Literal("name")

        separator     = Literal(",")
        leftBrkt      = Literal("<")
        rightBrkt     = Literal(">")
        assign        = Literal("=")

        state         = Word(alphanums)
        taskName      = Word(alphas)
        conditionName = Word(alphas)
        name          = Word(alphanums)

        nameDef       = Group(nameKWD + name)
        inAlphabetDef = Group(inAlphKWD + Group(Word(alphanums) + ZeroOrMore(separator + Word(alphanums))))
        statesDef     = Group(statesKWD + Group(state + ZeroOrMore(separator + state)))
        initialSDef   = Group(initSKWD + state)
        finalSDef     = Group(finalSKWD + state)

        taskBindKWD   = Literal("taskBindings")
        machineSecKWD = Literal("machineParams")
        transitionsKWD= Literal("transitions")

        taskBinding   = Group(state + separator + taskName)
        transition    = Group(leftBrkt + state + separator + OneOrMore(conditionName) + separator + state + rightBrkt)

        machineParamSec = machineSecKWD + nameDef + inAlphabetDef + statesDef + initialSDef + finalSDef
        taskBindingsSec = taskBindKWD + OneOrMore(taskBinding)
        transitionSec   = transitionsKWD + OneOrMore(transition)

        config          = Group(machineParamSec) + Group(taskBindingsSec) + Group(transitionSec)

        return config

    def createFSM(parsedInput):
        alphabet = [x for x in parsedInput[0][2][1] if x != separator]
        name = parsedInput[0][1][1]
        startState = parsedInput[0][4][1]
        finalState = parsedInput[0][5][1]
        transitions = []
        for trans in parsedInput[2][1:]:
            t = tuple([x for x in trans if x != leftBrkt and x != rightBrkt and x != separator])
            transitions.append(t)

        states = [x for x in parsedInput[0][3][1] if x != separator]

        bindings = []
        for binding in parsedInput[1][1:]:
            t = tuple([x for x in binding if x != separator])
            bindings.append(t)

        fsm = FSM(name, alphabet, states, startState, finalState, transitions, bindings)
        return fsm
