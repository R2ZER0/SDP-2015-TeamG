
import pdb
from pyparsing import *
from planning.models import Robot
from planning.tasks import *

def createFSM(parsedInput, sourceFilePath):
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
    finalState = parsedInput[0][5][1]

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

    # Perform sanity checks on what we've read 
    checkAlphabet(alphabet, lambdas, sourceFilePath)
    checkTransitions(alphabet, transitions, sourceFilePath)
    checkLambdas(alphabet, dic, sourceFilePath)

    parseError = False
    parseError = checkAlphabet(alphabet, lambdas, sourceFilePath)
    parseError = checkTransitions(alphabet, transitions, sourceFilePath)
    parseError = checkLambdas(alphabet, dic, sourceFilePath)

    if parseError:
        return False
    else:
        return FSM(name, alphabet, states, startState, finalState, transitions, dic, lambdas)

    # If we're here, everything is valid and we can go ahead and create the FSM
    # fsm = FSM(name, alphabet, states, startState, finalState, transitions, dic, lambdas)
    # return fsm

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

    # Define the various single-character tokens we wish to recognise
    separator     = Literal(",")
    leftAngBrkt   = Literal("<")
    rightAngBrkt  = Literal(">")
    assign        = Literal("=")
    sMark         = Literal("\"")
    colon         = Literal(":")
    leftSqBrkt    = Literal("[")
    rightSqBrkt   = Literal("]")

    # A 'state' is an alphanumeric 'word'
    state         = Word(alphanums)

    # A task invocation is either of the form '[EXISTING]' or '[TaskName, arg1, arg2, ...]']
    taskInvocation = Group(leftSqBrkt + exisitingKWD + rightSqBrkt) ^ Group(leftSqBrkt + Word(alphas) + ZeroOrMore(separator + ZeroOrMore(Word(alphanums+" _()'.\"")^nestedExpr(opener='[', closer=']', content=Word(alphanums+" _()'.\"")))) + rightSqBrkt)

    # An FSM name is an alphanumeric 'word'
    name          = Word(alphanums) 

    # A letter in an FSM alphabet is an alphanumeric 'word'
    letter        = Word(alphanums)

    # A name definition is of the form 'name identifier'
    nameDef       = Group(nameKWD + name)

    # An alphabet definition is of the form  'inAlph letter1,...'
    inAlphabetDef = Group(inAlphKWD + Group(letter + ZeroOrMore(separator + letter)))

    # A states definition list is of the form  'states state1,...'
    statesDef     = Group(statesKWD + Group(state + ZeroOrMore(separator + state)))

    # An initial state definition is of the form  'initialState state1,...'
    initialSDef   = Group(initSKWD + state)

    # A final state definition is of the form  'finalState stateName'
    finalSDef     = Group(finalSKWD + state)

    # A lambda statement is of the form '"alphabetLetter" : lambda planner : code', where code is an alphanumeric 'word', with >< .+-(),\t\n characters
    lambdaStmt    = Group(sMark + letter + sMark + colon + lambdaKWD + plannerKWD + colon + Word(alphanums + ">< .+-[]_!=(),\t\n"))

    # A transition is of the form '<stateName, letter1, letter2,..., [TaskName, arg1, arg2,...], newState>'
    transition    = Group(leftAngBrkt + state + separator + OneOrMore(letter) + separator + taskInvocation + separator + state + rightAngBrkt)

    machineParamSec= machineSecKWD + nameDef + inAlphabetDef + statesDef + initialSDef + finalSDef
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

def checkTransitions(alphabet, transitions, sourceFilePath):
    """Ensures there are no transitions which reference unrecognised letters, that is,
    those that have not been speficied in the machine specification. If we enounter an error, we 
    inform the user and terminate."""
    for transition in transitions:
        lettersAppearingInTrans = transition[1:len(transition)-2]   # Extract the letters appearing in a transition table entry
        for candidateLetter in lettersAppearingInTrans:
            """Check each letter valid"""
            if not candidateLetter in alphabet:
                print
                print "Parse error: Found a transition statement which is inconsistent with the FSM alphabet definition - namely:\n'" + str(transition) + "'\nin " + str(sourceFilePath)
                print
                return True

def checkLambdas(alphabet, lambdas, sourceFilePath):
    """Ensures the letter:lambda pairs do not refer to a non-existent FSM letter. If we encounter an
    error, we inform the user and then terminate"""
    for lambdaStmt, code in lambdas.iteritems():
        if not lambdaStmt in alphabet:
            print "Parse error: Found a lambda statement whose key is inconsistent with the FSM alphabet definition, the offending key being '" + lambdaStmt + "' in " + str(sourceFilePath)
            print
            return True

def checkAlphabet(alphabet, lambdas, sourceFilePath):
    if not len(alphabet) == len(lambdas):
        print "Parse warning: There are some FSM alphabet letters which do not have corresponding lambda condition triggers in spec file " + str(sourceFilePath)
        print


class FSM:
    """Class representing planner finite state machines"""
    def __init__(self, name, inAlph, states, initState, finalState, transTable, lambdaDict, lambdaDescs):
        self._alph = inAlph
        self._states = states
        self._initState = initState
        self._finalState = finalState
        self._lambdas = lambdaDict      # The {letter : lambda} dictionary used in actual computation
        self._lambdaDescs = lambdaDescs # This is a print-friendly form of self._lambdas

        self._transTable = transTable   # Table representing transition function
        self._name = name

        self._currentState = self._initState
        # self._currentTask = AcquireBall(world,robot,"attacker")  #HACK
        self._currentTask = None

    @property
    def currentTask(self):
        '''Returns the current task instance run by the FSM.'''
        return self._currentTask

    @property
    def currentState(self):
        return self._currentState

    def consumeInput(self, inp, world, robot, role):
        """Takes a set of FSM letters and checks to see what transition should be executed by consulting
        the transition table. The code considers the current state, the letters that are to be consumed and
        the task invocation. 

        Once the appropriate transition is identified, we change the current state, and execute the 
        specified task, or continue executing the existing one if EXISTING is given"""

        # To ensure the set of letters we've been supplied with is in fact valid, we check the given set is 
        # a subset of the alphabet the machine recognises
        assert set(inp) <= set(self._alph) 

        # Consider each possible transition
        for tran in self._transTable:

            # Pull list of required true conditions from transition
            required_truths = list(tran[1:len(tran)-2])
            missing_truths = list([x for x in required_truths if x not in inp])

            if tran[0] == self._currentState and len(missing_truths) == 0:
                """Transitions are of the form (currState, listOfLetters, [taskName, taskArgs...], newState)
                So, we check to see if the current state matches the first entry in the transition we're 
                currently examining, as well as list of letters supplied to the function matches. 
                If this is the case, we've found the right transition to invoke."""

                # If we've found the right transition update the current state to the new one
                # specified in the transition tuple.
                self._currentState = tran[len(tran) - 1] 
                if tran[len(tran) - 2][1] == "EXISTING":
                    """tran[len(tran) - 2][1] contains the name of the task the input file
                    specifies to execute when leaving the current state. If EXISITING has 
                    been given, it is desired the current task continues executing."""
                    print "FSM '"+ self._name + "' executing exisiting task"
                    self._currentTask.execute()
                    print
                    return
                else:
                    """If we have something other than EXISTING, a task name with arguments has
                    been given. So, we create a list of relevant information, filtering out detritus 
                    characters such as ',' and '[', and then extract the task name ( t[0] ), and set of task
                    args ( ','.join(t[1:]) - this builds a string of form ['a1,a2,a3,...']).

                    Finally, we package these pieces of data into valid code representing
                    the creation of a task object with supplied contructor arguments, and cause 
                    Python to evaulate this text as code using eval()."""

                    # Handle list elements that are nested [] brackets
                    t = []
                    for index, element in enumerate(tran[len(tran)-2]):

                        # Ignore separator characters
                        if element in [",", "[", "]"]:
                            continue

                        # Concatenate nested square bracket elements onto last
                        elif type(element) == list:
                            t[-1] = t[-1] + '[%s]' % ''.join(element)

                        # Add normal elements into the list
                        else:
                            t.append(element)

                    code = t[0] + "(" + ','.join(t[1:]) + ")"

                    print "FSM '" + self._name + "' changing to execute new task - " + str(t[0]) + " with args " + "(" + ','.join(t[1:]) + ")"
                    self._currentTask = eval(code)
                    self._currentTask.execute()
                    print

                    return
                    
        # If we get here, we looped through the entire set of transitions and didn't find any that applied
        # in the current situation.
        print "FSM '" + self._name + " reports there is no transition entry for current state " + self._currentState + " and input '" + str(inp) + "'"

    def getStates(self):
        return self._states

    def getAlpha(self):
        return self._alph

    def getLambdaDic(self):
        return self._lambdas

    def show(self):
        print
        print "------------------------------------------------------------------------------------------------------------------------------"
        print "FSM '" + self._name + "'"
        print
        print "Recognised alphabet: " + str(self._alph)
        print "States : " + str(self._states)
        print "Initial State: " + str(self._initState)
        print "Final State: " + str(self._finalState)
        print "Current State: " + str(self._currentState)
        print "Current Task: " + str(self._currentTask)
        print
        print "Transitions and associated tasks bindings: "
        for trans in self._transTable:
            print trans
        print
        print "Lambda condition triggers"
        for lambdaDesc in self._lambdaDescs:
            print lambdaDesc
        print "------------------------------------------------------------------------------------------------------------------------------"
        print
