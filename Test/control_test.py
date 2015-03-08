import unittest2
from action import Action
from serial import Serial
import time

class TestAction(unittest2.TestCase):
    
    def test_create_action(self):
        self.serial = Serial('/dev/ttyACM3', 115200)
        self.action = Action(self.serial)
        time.sleep(0.1)
        self.action.exit()
        
if __name__ == '__main__':
    unittest2.main()