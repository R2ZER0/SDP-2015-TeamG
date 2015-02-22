##System Design Project - Group 9##

*Thomas Cumming, Rikki Guy, Joseph Kennelly, Ayrton Massey, Robert Oyler, Sebastian Schulze, Ronan Turner*

![alt text](http://i.imgur.com/Y6Z5NtE.png "I think he just says ""bloop bloop bloop"")
------

###Future SDP teams

The planner and vision are very good.

We used an Arduino powered robot but the vision and planning can be effectively used even with the default NXT Brick. All it takes is to modify the Controller class in `controller.py`. 

The vision is very accurate with the correct calibration and achieves around 28 FPS (faster than the camera).
Planner is based on a reactive system. The defender strategy was one of the best in 2014 and the latest attacker strategy worked very well. In the end, it all depends on the build of the robot.

###Running the system

In the root of the project, execute `python controller.py <pitch_number> <our_side> <our_color>` where *pitch_number* is either 0 for the main pitch and 1 for the secondary pitch. Colors are regular yellow and blue. Side can be either left or right.

####Vision controls
Press the following keys to switch the thresholding scheme:
*r* - red
*p* - plate
*b* - black dots

Note that the colors of the plates themselves are ignored - you don't need them.

------
###Installation

This project uses an Arduino. We connect to the Arduino via an RF Stick which we write to over a serial port.

#### Linux/DICE

The project has the following dependencies:
- ArgParse
- Polygon2
- PySerial
- OpenCV 2.4.5
- Numpy
- pygame 	(For `xbox360controller/`)
- pymunk 	(For Simulation)
- Flask 	(For `ControlApp/`)
- Pyzmq 	(For the RF Console)
- Sphinx-Docs 	(Generating documentation)

##### Core Dependencies 

The run with `scripts/` as the working directory, `scripts/install_venv.sh` will install ArgParse, Polygon2, PySerial, OpenCV 2.4.5 and Numpy to a virtual envrionment in a directory named `venv` to the left of the repository folder. This script requires ~2.8GB of space to compile OpenCV but the files will be deleted afterward.

##### pygame

To install pygame to this virtual environment, download the [1.9.1 release](http://www.pygame.org/ftp/pygame-1.9.1release.tar.gz). Unfortunately debug statements were left in the source code, so to prevent the console spam this causes extract the files, navigate to the src/ folder and edit `joystick.c` to remove all `printf` statements. Compress the file again, activate the virtual environment and then `pip install` pygame from the new archive:

```
curl -O http://www.pygame.org/ftp/pygame-1.9.1release.tar.gz
tar -zxvf pygame-1.9.1release.tar.gz
//edit src/joystick.c to remove printf statements
tar -zcvf pygame-1.9.1release.tar.gz pygame-1.9.1release
source venv/bin/activate
pip install pygame-1.9.1release.tar.gz
```

*Note: The XBOX 360 controller drivers are not present on DICE. It is not possible to install them because DICE does not have the dependencies required for the drivers.*

------
### Vision

* At the moment OpenCV + Python are being used. A [book](http://programmingcomputervision.com/downloads/ProgrammingComputerVision_CCdraft.pdf) on Computer Vision with OpenCV in Python is a decent starting point about what OpenCV can do.
* A detailed tutorial with examples and use cases can be found [here](https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_tutorials.html) - going through it can be handy to understand the code *
