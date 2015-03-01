import unittest2
from action import Action
from serial import Serial

class TestAction(unittest2.TestCase):
    
    def test_create_action(self):
        self.serial = Serial('/dev/ttyACM3', 115200)
        self.action = Action(self.serial)
        
if __name__ == '__main__':
    unittest2.main()