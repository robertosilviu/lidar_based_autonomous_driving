# Documentation
Simulation of a real time system that learns to drive a car on track, via rudimental lidar beams, using reinforcement learning and pthread library. This application has been created for educational purpose only  as a project for an  university exam. 
<br>
The main idea behind this project is to simulate a car having a lidar sensor in order to "sense" the track margins, driven by an intelligent agent. The sensor has 3 beams with an angle of 45 degrees between each of them and detects the track margins via pixels color analysis. 
<br>
The agent uses a simple implementation of Epsilon-greedy Q-Learning algorithm in order to learn to drive the car on track. There are two learning modes:
1. only steering: the agent learns the steering input while the velocity is set constant
2. steering + acceleration: the agent needs to learn both steering and acceleration

The second learning mode is implemented using two quality functions (*Q* and *Q_vel*) in order to be able to train the agent for two different control inputs.  
<br>
#### Reward function structure
- big negative reward when crash occures
- small negative reward for being alive 
- reward based on the distance driven on track
- reward based on the steering variance
- negative reward for bad acceleration input (negative when car's velocity is 0)

### Multi task vs single task
The application can be executed in 2 different modalities:
#### Multi task
This mode simulates a real time system with multiple tasks.  
The executed tasks are:
- *display_task*: handles the graphics
- *agent_task*: updates car model and handles the agent controls
- *sensors_task*: updates sensors readings
- *comms_task*: handles the keyboard inputs
#### Single task
In this mode all the previous tasks are executed as one task which name is *learning_task*.

## Project structure

- libs/**ptask**: rudimental library for a multi task application using pthread and access to shared resources via mutex semaphors
- libs/**qlearn**: implementation of qlearn algorithm
- libs/**tlib**: rudimental library for time handling 
- src: main application files
- scripts: contains a python script used to analyze the learning algorithm
- **q_matrix.txt**/**q_vel_matrix.txt**: contains examples of trained Q matrix
- **run.sh**: simple script to execute the application
- **project_report_italian.pdf**: report in italian language containing more informations about the project.

## Requirements
### Allegro library
This is a library used to handle the graphics of the application. More information can be found [here](http://alleg.sourceforge.net).
In order to install the library on ubuntu: 
```sh
sudo apt-get install liballegro4.2 liballegro4.2-dev
```
### Python script
The script requires `numpy` and `matplotlib`.

## Gui example
### Steering only
![example](img/steer_only.gif)

### Steering + aceleration
![example](img/steer_acc.gif)