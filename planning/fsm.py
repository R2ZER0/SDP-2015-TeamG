
import pdb
from pyparsing import *
from planning.models import Robot
from planning.tasks import *

def createFSM(parsedInput, sourceFilePath, logger):
    """Takes the results of a successful parse and builds and returns an FSM object adhering to 
    the specs in the given config file."""

    # parsedInput[0][2][1] contains the string read which represents the machine alphabet
    # We create a list, with useless characters like "," stripped
    alphabet = [x for x in parsedInput[0][2][1] if x != ","]

    # parsedInput[0][1][1] contains the string read which represents the machine name
    name = parsedInput[0][1][1]

    # parsedInput[0][4][1] contains the string read which represents the machine start state
    startState = parsedInput[0][4][1]

    # parsedInput[0][5][1] contains the string read which represents the machine final state
    finalState = "".join(c for c in parsedInput[0][5][1:] if c != "[" and c != "]")

    # parsedInput[0][6][1:] contains the string read which represents the next plan spec file to execute (if there is one)
    nextPlanToInvoke = "".join(c for c in parsedInput[0][6][1:] if c != "[" and c != "]")

    # parsedInput[0][7][1] contains the string read which represents whether the machine is initially active
    initiallyActive = parsedInput[0][7][1]

    # Create the list which we'll populate with transition tuples of the form
    # (currState, letter1, letter2,..., [TaskName, arg1,arg2,..], newState)
    transitions = []

    # parsedInput[1][1:] contains the raw transitions. "x.asList() if type(x) == ParseResults else x" is
    # necessary because of the way PyParsing returns parse results. We want things as a list, so we call
    # asList()
    for trans in parsedInput[1][1:]:
        t = tuple([(x.asList() if type(x) == ParseResults else x) for x in trans if x != "<" and x != ">" and x != ","])
        transitions.append(t)
    
    # parsedInput[0][3][1] contains the raw state list
    # We create a tuple of the form (state1, state2,...), filtering out useless chars like
    # ","
    states = tuple([x for x in parsedInput[0][3][1] if x != ","])

    # Create the string we'll use to contain the lambdas found during the parse
    lambdas = []
    for lambdaStmt in parsedInput[2][1:]:
        t = ''.join(map(str, lambdaStmt))
        lambdas.append(t)

    # Build the actual letter:lambda dictionary
    dic = consumeLambdas(lambdas)

    checkAlphabet(alphabet, dic, sourceFilePath, logger)
    transitionParseError = checkTransitions(alphabet, transitions, sourceFilePath, states, logger)
    lambdaParseError = checkLambdas(alphabet, dic, sourceFilePath, logger)
    finalStateConsistencyError = checkFinalStateConsistencies(finalState, nextPlanToInvoke, sourceFilePath, logger)

    if transitionParseError or lambdaParseError or finalStateConsistencyError:
        return False
    else:
        # If we're here, everything is valid and we can go ahead and create the FSM
        logger.newline()
        logger.info("Parse of file '" + str(sourceFilePath) + "' successful.")
        return FSM(name, alphabet, states, startState, finalState, initiallyActive, sourceFilePath, nextPlanToInvoke, transitions, dic, lambdas, logger)

def createConfigGrammar():
    """Here, we define the PyParsing grammar to use when executing a parse run of an FSM
    input specification script"""

    # Define the keywords
    inAlphKWD     = Literal("inAlph")
    finalSKWD     = Literal("finalState")
    statesKWD     = Literal("states")
    initSKWD      = Literal("initialState")
    nameKWD       = Literal("name")
    lambdaSecKWD  = Literal("lambdaConditions")
    machineSecKWD = Literal("machineParams")
    transitionsKWD= Literal("transitions")
    lambdaKWD     = Literal("lambda ")
    worldKWD      = Literal("world")
    exisitingKWD  = Literal("EXISTING")
    plannerKWD    = Literal("planner")
    nextPlanKWD   = Literal("nextPlanOnCompletion")
    unusedKWD     = Literal("UNUSED")
    naKWD         = Literal("NA")
    yesKWD        = Literal("YES")
    noKWD         = Literal("NO")
    activeStartKWD= Literal("activeAtStart")

    # Define the various single-character tokens we wish to recognise
    separator     = Literal(",")
    leftAngBrkt   = Literal("<")
    rightAngBrkt  = Literal(">")
    assign        = Literal("=")
    sMark         = Literal("\"")
    colon         = Literal(":")
    leftSqBrkt    = Literal("[")
    rightSqBrkt   = Literal("]")
    anyState      = Literal("*")

    # A 'state' is an alphanumeric 'word'
    state         = Word(alphanums)

    # A task invocation is either of the form '[EXISTING]' or '[TaskName, arg1, arg2, ...]']
    taskInvocation = Group(leftSqBrkt + exisitingKWD + rightSqBrkt) ^ Group(leftSqBrkt + Word(alphas) + ZeroOrMore(separator + ZeroOrMore(Word(alphanums+" +-_()='.\"")^nestedExpr(opener='[', closer=']', content=Word(alphanums+" _+-=()'.\"")))) + rightSqBrkt)

    # An FSM name is an alphanumeric 'word'
    name          = Word(alphanums) 

    # A letter in an FSM alphabet is an alphanumeric 'word'
    letter        = Word(alphanums)

    # A name definition is of the form 'name identifier'
    nameDef       = Group(nameKWD + name)

    # Starting active specification is of form activeAtStart followed by either YES or NO
    startActiveDef = Group(activeStartKWD + (yesKWD ^ noKWD))

    # A filename is an alphanumeric word, including a period character
    fileName      = Word(alphanums+".")

    # An alphabet definition is of the form  'inAlph letter1,...'
    inAlphabetDef = Group(inAlphKWD + Group(letter + ZeroOrMore(separator + letter)))

    # A states definition list is of the form  'states state1,...'
    statesDef     = Group(statesKWD + Group(state + ZeroOrMore(separator + state)))

    # If there is no next plan to invoke, it is of the form [NA]
    noNextPlan    = leftSqBrkt + naKWD + rightSqBrkt

    # A new plan is either nextPlanOnCompletion [NA] or nextPlanOnCompletion filename.txt
    nextPlanDef   = Group(nextPlanKWD + noNextPlan) ^ Group(nextPlanKWD + fileName)

    # An initial state definition is of the form  'initialState state1,...'
    initialSDef   = Group(initSKWD + state)

    # A final state definition is of the form  'finalState stateName'
    finalSDef     = Group(finalSKWD + state)

    # A lambda statement is of the form '"alphabetLetter" : lambda planner : code', where code is an alphanumeric 'word', with >< .+-(),\t\n characters
    lambdaStmt    = Group(sMark + letter + sMark + colon + lambdaKWD + plannerKWD + colon + Word(alphanums + "/>< .+-[]_!'=(),\t\n"))

    # A transition is of the form '<stateName, letter1, letter2,..., [TaskName, arg1, arg2,...], newState>'
    transition    = Group(leftAngBrkt + (anyState ^ state) + separator + OneOrMore(letter) + separator + taskInvocation + separator + state + rightAngBrkt)

    machineParamSec= machineSecKWD + nameDef + inAlphabetDef + statesDef + initialSDef + finalSDef + nextPlanDef + startActiveDef
    transitionSec  = transitionsKWD + OneOrMore(transition)
    lambdaSec      = lambdaSecKWD + OneOrMore(lambdaStmt)

    # Config is the 'root' of the parse
    config         = Group(machineParamSec) + Group(transitionSec) + Group(lambdaSec)

    return config

def consumeLambdas(text):
    """This function takes python code representing key:value pairs in string form,
    and for each pair, packages them up into code that will update the dictionay letter-condition
    map.

    Once we have the dictionary update code for each lambda, we cause Python to execute this code_obj
    using exec and return the resulting dictionary
    """
    strippedNewlines = [line.strip("\n") for line in text]

    code = ""
    dictionary={}

    for line in strippedNewlines:
        code = code + "dictionary.update({ " + line + " })\n"
   
    code_obj = compile(code, '<string>', 'exec')
    exec(code_obj)
    return dictionary

def checkTransitions(alphabet, transitions, sourceFilePath, states, logger):
    """Ensures there are no transitions which reference unrecognised letters, that is,
    those that have not been speficied in the machine specification. If we enounter an error, we 
    inform the user and terminate."""
    errorDiscovered=False                                                        
    if transitions[0][len(transitions[0]) - 2 : len(transitions[0]) - 1] == (['[', 'EXISTING', ']'],):
        logger.newline()                                                       # HACKY, CLEAN UP^^
        logger.error("PARSE ERROR: Attempted invocation of existing task in very first transition in '" + str(sourceFilePath) + "'. Invalid since there won't be an existing task to execute.")
        errorDiscovered = True

    for transition in transitions:
        lettersAppearingInTrans = transition[1 : len(transition)- 2]   # Extract the letters appearing in a transition table entry
        for candidateLetter in lettersAppearingInTrans:
            """Check each letter valid"""
            if not candidateLetter in alphabet:
                logger.newline()
                logger.error("PARSE ERROR: Encountered a transition statement which refers to unrecognised alphabet letters - namely: '" + str(candidateLetter) + "' in transition\n'" + str(transition) + "'\nin file '" + str(sourceFilePath) + "'.")
                errorDiscovered = True
        if transition[0] not in list(states) + ["*"]:
            logger.newline()
            logger.error("PARSE ERORR: Encountered an unrecognised state '" + str(transition[0]) + "' in transition\n'" + str(transition) + "'\nin file '" + str(sourceFilePath) + "'.")
            errorDiscovered = True
        if transition[len(transition) - 1] not in list(states) + ["*"]:
            logger.newline()
            logger.error("PARSE ERROR: Encountered an unrecognised state '" + str(transition[len(transition) - 1]) + "' in transition\n'" + str(transition) + "'\nin file '" + str(sourceFilePath) + "'.")
            errorDiscovered = True
    return errorDiscovered

def checkLambdas(alphabet, lambdas, sourceFilePath, logger):
    """Ensures the letter:lambda pairs do not refer to a non-existent FSM letter. If we encounter an
    error, we inform the user and then terminate"""
    errorDiscovered = False
    for key, code in lambdas.iteritems():
        if not key in alphabet:
            logger.newline()
            logger.error("PARSE ERROR: Found a lambda statement whose key is inconsistent with the FSM alphabet definition, the offending key being '" + str(key) + "' in file '" + str(sourceFilePath) + "'.")
            errorDiscovered = True
    return errorDiscovered

def checkAlphabet(alphabet, lambdas, sourceFilePath, logger):
    for letter in alphabet:
        if letter not in lambdas.keys():
            logger.newline()
            logger.warning("PARSE WARNING: The FSM alphabet letter '" + str(letter) + "' has no corresponding lambda condition trigger in spec file '" + str(sourceFilePath) + "'.")

def checkFinalStateConsistencies(finalState, nextPlanSpec, sourceFilePath, logger):
    if finalState == 'UNUSED' and nextPlanSpec != "NA":
        logger.error("PARSE ERROR: Machine specified in file '" + str(sourceFilePath) + "' wants to execute a new plan ('" + str(nextPlanSpec) + "') upon it's own completion - however, this can never happen since there is no final state defined.")
        return True
    elif finalState == 'UNUSED' and nextPlanSpec != "NA":
        logger.error("PARSE ERROR: Machine specified in file '" + str(sourceFilePath) + "' defines a final state, but no new plan to invoke after settling in this state.")
        return True


class FSM:
    """Class representing planner finite state machines"""
    def __init__(self, name, inAlph, states, initState, finalState, initiallyActive, sourceFilePath, nextPlanToInvoke, transTable, lambdaDict, lambdaDescs, logger):
        self._alph = inAlph
        self._states = states
        self._initState = initState
        self._definingFile = sourceFilePath

        if initiallyActive == "YES":
            self._active = self._InitiallyActive = True
        else:
            self._active = self._InitiallyActive = False

        self._nextPlan = nextPlanToInvoke
        self._finalState = finalState
        self._lambdas = lambdaDict      # The {letter : lambda} dictionary used in actual computation
        self._lambdaDescs = lambdaDescs # This is a print-friendly form of self._lambdas

        self._logger = logger           # Used for recording output

        self._transTable = transTable   # Table representing transition function
        self._name = name

        self._currentState = self._initState
        self._currentTask = None


    def executeExistingTask(self):
        """tran[len(tran) - 2][1] contains the name of the task the input file
        specifies to execute when leaving the current state. If EXISITING has 
        been given, it is desired the current task continues executing."""
        logger.info("FSM '"+ self._name + "' executing exisiting task")
        self._currentTask.execute()
        self._logger.newline()
        return

    def executeNewTask(self, tran, world, robot, role):
        # Handle list elements that are nested [] brackets
        t = []
        for index, element in enumerate(tran[len(tran)-2]):

            # Ignore separator characters
            if element in [",", "[", "]"]:
                continue

            # Concatenate nested square bracket elements onto last
            elif type(element) == list:
                t[-1] = t[-1] + '[%s]' % ''.join(element)

            elif element[0] == '.':
                t[-1] = ''.join([t[-1],element])

            # Add normal elements into the list
            else:
                t.append(element)

        code = t[0] + "(" + ','.join(t[1:]) + ")"
        self._logger.info("FSM '" + self._name + "' changing to execute new task - " + str(t[0]) + " with args " + "(" + ','.join(t[1:]) + ")")
        self._currentTask = eval(code)
        self._currentTask.execute()
        self._logger.newline()
 
        return

    def consumeInput(self, inp, world, robot, role): 
        """Takes a set of FSM letters and checks to see what transition should be executed by consulting
        the transition table. The code considers the current state, the letters that are to be consumed and
        the task invocation. 

        Once the appropriate transition is identified, we change the current state, and execute the 
        specified task, or continue executing the existing one if EXISTING is given"""

        # To ensure the set of letters we've been supplied with is in fact valid, we check the given set is 
        # a subset of the alphabet the machine recognises
        assert set(inp) <= set(self._alph) 

        # If this FSM is not active, it should not do anything - so we return 
        if not self._active return

        # Consider each possible transition
        for tran in sorted(self._transTable, key=lambda t : 1 if t[0] == "*" else 10):

            # Pull list of required true conditions from transition
            required_truths = list(tran[1:len(tran)-2])
            missing_truths = list([x for x in required_truths if x not in inp])

            if tran[0] == "*" and len(missing_truths) == 0:
                if tran[len(tran) - 2][1] == "EXISTING":
                    self.executeExistingTask()
                    self._currentState = tran[len(tran) - 1] 
                    return
                else:
                    self.executeNewTask(tran, world, robot, role)
                    self._currentState = tran[len(tran) - 1] 
                    return

            if tran[0] == self._currentState and len(missing_truths) == 0:
                """Transitions are of the form (currState, listOfLetters, [taskName, taskArgs...], newState)
                So, we check to see if the current state matches the first entry in the transition we're 
                currently examining, as well as list of letters supplied to the function matches. 
                If this is the case, we've found the right transition to invoke."""

                # If we've found the right transition update the current state to the new one
                # specified in the transition tuple.
                self._currentState = tran[len(tran) - 1] 
                if tran[len(tran) - 2][1] == "EXISTING":
                    self.executeExistingTask()
                    self._currentState = tran[len(tran) - 1] 
                    return 
                else:
                    """If we have something other than EXISTING, a task name with arguments has
                    been given. So, we create a list of relevant information, filtering out detritus 
                    characters such as ',' and '[', and then extract the task name ( t[0] ), and set of task
                    args ( ','.join(t[1:]) - this builds a string of form ['a1,a2,a3,...']).

                    Finally, we package these pieces of data into valid code representing
                    the creation of a task object with supplied contructor arguments, and cause 
                    Python to evaulate this text as code using eval()."""
                    self.executeNewTask(tran, world, robot, role)
                    self._currentState = tran[len(tran) - 1] 
                    return
                    
        if self._finalState != "UNUSED" and self._finalState == self.currentState:
            # Here, the current FSM has entered it's final state and is done executing. We inform the users and 
            # return the next plan to execute, if there is one
            self._logger.info("FSM '" + self._name + " has entered it's final state and ceased running. ")

            # Machine has finished it's run, so should become inactive
            self._active = False

            # Upon entering it's final state, the machine should make this known to the planner. It does this by
            # returning the filename of the next fsm spec to make active (as opposed to None as usual).
            return self._nextPlan
        else:
            # If we get here, we looped through the entire set of transitions and didn't find any that applied
            # in the current situation.
            self._logger.info("FSM '" + self._name + " reports there is no transition entry for current state " + self._currentState + " and input '" + str(inp) + "'")

    def show(self):
        self._logger.newline()
        self._logger.info("------------------------------------------------------------------------------------------------------------------------------")
        self._logger.info("FSM '" + str(self._name + "'"))
        self._logger.info("Defining file: " + str(self._definingFile))
        self._logger.newline()
        self._logger.info("Recognised alphabet: " + str(self._alph))
        self._logger.info("States : " + str(self._states))
        self._logger.info("Initial State: " + str(self._initState))
        self._logger.info("Final State: " + str(self._finalState))
        self._logger.info("Next plan to invoke: " + str(self._nextPlan))
        self._logger.info("Currently active: " + str(self._active))
        self._logger.info("Initially active: " + str(self._InitiallyActive))
        self._logger.info("Current State: " + str(self._currentState))
        self._logger.info("Current Task: " + str(self._currentTask))
        self._logger.newline()
        self._logger.info("Transitions and associated tasks bindings: ")
        for trans in self._transTable:
            self._logger.info(trans)
        self._logger.newline()
        self._logger.info("Lambda condition triggers")
        for lambdaDesc in self._lambdaDescs:
            self._logger.info(lambdaDesc)
        self._logger.info("------------------------------------------------------------------------------------------------------------------------------")
        self._logger.newline()


    @property
    def currentTask(self):
        '''Returns the current task instance run by the FSM.'''
        return self._currentTask

    @property
    def definingFile(self):
        return self._definingFile

    @property
    def currentState(self):
        return self._currentState

    def setActive():
        self._active = True

    def getStates(self):
        return self._states

    def getAlpha(self):
        return self._alph

    def getLambdaDic(self):
        return self._lambdas

