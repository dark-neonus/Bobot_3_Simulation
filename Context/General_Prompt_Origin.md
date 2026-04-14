Good, now lets together write prompt for my ai agent in vs code to create this simulation. In prompt you msut mention all needed parameters(even those, which I havent mentioned), their short description and how they should be used in simulation. Decsribe tools to use and for what, python libraries, venv, urdf files, etc.
Describe project file structure. Mention about example project and enlist its "git clone" received version in project structure. Give usefull advices and if there are some rough or non intuitive moments mention them and explain how to fix/implement/bahave with them.
Dont forget to disribe simulation not only of physical parameters, but also of electrical(current use, other parameters, motors operating voltage, deriving motor speed, acceleration and speed from its constants). In addition, there should be some sort of code+electronics simulation. There should be code that represent reading of gyroscope+accelerometer values(also simulated) and position of wheels, those values are passed to pid regulator and it produce command to something that controll wheels similar to SimpleFOC controller and special foc library for it. This controll loop must be runned with specific frequency corresponding to simulation time(to simulate robot controll). And yet again, dont forget about simulating backslash. Also my robot shoudl consist of next physical elements: wheels system(and something they connectedt to body, I guess), body, batteries. I need this separation, because I want to change position of my batteries(up to putting them under wheels axis) and see how robot will behave. All those components should have their mass and be one single physical system(not fall apart), but behave properly. Also there should be ability to somehow simulate next cases: motor, motor+reductor, motor+reducting belt, motor+reductor+reducting belt. All parameters should be easy to change, maybe even in gui of simulation, all parameters(including physica dimensions, mass, constants, reduction ration, battery voltage and etc. Also there should be a way to record simulation state, meaning current usage, positions of robot/each element, robot tilt, robot inertia, motor speed, gyroscope and accelerometer simulated values, etc(all meaningfull for simulation) and further should be able to be plotted and examinated fir debugging and monitoring system for decision making. Things like arms, display and other fun features should not be simulated. Dont add styling or role or behaviour specification for ai agent, as for latest researches it is bad practice. Just state very clear what is need to be done and how. Just high quality technical request.
This is very complex task, so there should be step by step plan on implementing this project. Make prompt not less detailed as me writing to you right now. Give up some styiling and well build sentences for more details and explanations. This prompt mustbe well understood by both: middle-seniour engineer and by ai agent.

Here is robot context(I will provide it to ai agent as separate file in Context diretory, dont worry about it, just take as context for yourself for better prompt generating):
```
# Bobot 3 MainContext

## Vision
Balancing robot with with display, speakers and two arms.
Robot can balance, move, rotate, move arms, produce audio.
Robot contain some sort of very simple AI(neural network or something similar) system to help mimic real thinking, experience and learning.
Some modules give robot ability to perceive real world and human. It could be dedicated modules to recognize humans, human faces, human presence, distance to objects, voice commands, etc.

## Physical parameters
Height aroung 50cm, maybe a little bit lower
Width would be mainly decided by motors system, for now counted as 20-30cm without arms.
Length(from back side to front) around 10cm, mainly dicated by motors diameter.
Wheels: 10-12cm diameter, 3-7cm width, to be able to ride one non idea suface and outside.
Mass: around 3kg, maximal referenced 5kg, depending on parts.
Center of mass: on height at around 25cm fron ground, around the center for other dimensions.
Desired max recovery angle: 35-45 degrees frfom ground normal

## Power supply
5S1P(in future possibly 5S2P) li-ion battery.
Model: 21700 li-ion Eve INR21700 50e 5000mAh 15A
Voltage range: 15-21V
Voltage rating chosen to be as high as possinble, to lower the current for same power output, and limited at the top by limits of SimpleFOC Shield to 24V.

## Microcontroller
ESP32-S3.
One core will be dedicated for balancing logic and other for self hosted small neural network reinforcment learning. Other tasks, like updating display, controlling speakers, moving arms servos, et, would be dispatchered on core, which would be the msot suitable for them, preferably the one with neural network processing.

## Arms
Two 3DOF arms with 3 mg90s servos on each. One servo for shoulder, one fo elbow and one for claw-like(gripper) mechanism on the end of arm.
Arms are for entertaining purpose mainly.

## Legs(Wheels)
There are two BLDC motors spinning wheels. They will be controlled by SimpleFOC Shield MKS DUAL FOC V3.2 BLDC.
Motors will controll wheels directly or through reductor. Prefered is option of direct drive, but if reductor need to be choosed, it must have low reduction ration, up to just 1:10, to remain speed of spin direction change low enough to balance well and fast. If reductor ration would be too high, it will have spinning inertial, wheels from change spinning direction fast enought.
On each motor there will be AS5600 12bit magnetic encoder, added by me manually for close-loop controll.
Wheels a nd motors would face invard.
If motors would be too long and cause robot width to be longer than 25-30 cm width wheels, additional belt would be used to put motors on top of wheels and reduce robot width by placing motots facing outwards on top of wheels, and whells facing invard under motors, but this can increse time to chagne wheels rotation speed. 
Since low speed and high torque is required, bldc motors with more poles are prefered.
Budget per single wheels system(motor or motor+reductor, belt, encoder and wheels not included in this limitation) is 40USD.
```