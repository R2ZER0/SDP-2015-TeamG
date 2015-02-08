import unittest2
from action import Command, Commander, Action


class TestCommand(unittest2.TestCase):
    
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
        
    def test_produces_correct_string(self):
        """Does Command.getCommandString work"""
        cmd = 'RUN'
        args = ['13','10','90']
        cmdstr = 'RUN 13 10 90'
        
        command = Command(cmd, args)
        
        self.assertEquals(command.getCommandString(), cmdstr)
        
if __name__ == '__main__':
    unittest2.main()