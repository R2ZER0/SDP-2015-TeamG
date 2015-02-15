import unittest2
from control.command import Command
from control.commander import Commander
from control import action


class TestCommand(unittest2.TestCase):
    """Tests for the Command class"""
    
    def test_correct_construction(self):
        """Normal instantiation of a valid command"""
        cmd = 'RUN'
        args = ['0', '50', '100']
        
        command = Command(cmd, args)
        
        self.assertEquals(command.command, cmd)
        self.assertEquals(command.args, args)
        
    def test_correct_from_string(self):
        """Correct instantiation by Command.fromCommandString"""
        cmd = 'RUN'
        args = ['0', '100', '20']
        cmdstr = 'RUN 0 100 20'
        
        command = Command.fromCommandString(cmdstr)
        
        self.assertEquals(command.command, cmd)
        self.assertEquals(command.args, args)
        self.assertEquals(command, Command(cmd, args))
        
    def test_produces_correct_string(self):
        """Does Command.getCommandString work"""
        cmd = 'RUN'
        args = ['13','10','90']
        cmdstr = 'RUN 13 10 90'
        
        command = Command(cmd, args)
        
        self.assertEquals(command.getCommandString(), cmdstr)
        
    def test_invalid_command(self):
        """I shouldn't be allowed to construct an invalid command"""
        with self.assertRaises(Exception):
            command = Command("NOT_A_REAL_COMMAND")
            

class TestCommander(unittest2.TestCase):
    """Tests for the Commander class"""
    
    class MockComm():
        """A mock Serial class"""
        def __init__(self):
            self.written = []
            
        def write(self, something):
            self.written.append(something)
            
        def flush(self):
            pass
        
    def setUp(self):
        self.comm = self.MockComm()
        
    def test_constructor(self):
        """Creating a Commander instance"""
        commander = Commander(self.comm)
        
    #edef test_run_
        
if __name__ == '__main__':
    unittest2.main()