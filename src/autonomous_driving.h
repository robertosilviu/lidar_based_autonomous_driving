#include <stdio.h>
#include <allegro.h>
#include <pthread.h>

/*-----------------------------------------------------------------------------*/
/*								CONSTANTS									   */
/*-----------------------------------------------------------------------------*/
#define PRINTLINE printf("LINE: %d\n", __LINE__)
#define LEN 65 // length of array of chars

//-------------------------------REINFORCEMENT LEARNING-------------------------

// r. on crash
#define RWD_CRASH -100 
// r. for being alive
#define RWD_ALIVE -1
// r. for keep the car near the centre of track
#define RWD_ON_CENTRE 1
// r. for driving for a certain amount of meters on track
#define RWD_DISTANCE 5
// r. for using wrong acceleration based on car velocity
#define RWD_BAD_ACC -50
// r. for steering variation
#define RWD_STEER_DELTA_GAP -2
// distance threshold for reward based on distance driven on track
#define DIST_THRESHOLD_RWD 20.0
// initial car velocity on mode STEER + ACC
#define TRAIN_VEL 0.0

#define LIDAR_ANGLE_POS 45.0
#define LIDAR_ANGLE_NEG -45.0
// number of states to use in Q matrix
#define MAX_STATES_LIDAR 7
// resolution to use stering angle quantization 
#define ACTIONS_STEP 5 // 5 degree resolution
// max steering angle
#define MAX_THETA 40
// max acceleration to use as command in mode STEER + ACC
#define MAX_A (0.2*G)
#define MIN_A (-0.2*G)
#define ACC_STEP 0.1
// hard upper limit on max velocity
// user can set velocity=min(MAX_VELOCITY_ALLOWED, MAX_V)
#define MAX_V 15.0
// file name where to save episodes statistics
#define EPISODES_STATS_FILE_NAME "episodes_stats.txt"
// file name of the Q matrix
#define Q_MAT_FILE_NAME "q_matrix.txt"
// file name of the Q_vel matrix
#define Q_VEL_MAT_FILE_NAME "q_vel_matrix.txt"
// max number of episodes which 
// can be used for statistics related to Q-Learning
#define MAX_EPISODES 50000
//-------------------------------GRAPHICS---------------------------------------
#define WIN_X 1024
#define WIN_Y 1024
#define BITS_COL 24 // color depth
// black color
#define BKG 0
#define BLACK 0
// asphalt color
#define ASPHALT 7303283
//#define WHITE 15

// height of region for the deadline misses gui
#define DMISS_H 250
// dimensions of track gui
#define SIM_X 800
#define SIM_Y 800 //800 before
#define SCALE 0.16 // 1px = 0.16m
// time scale to increase or decrease simulation velocity
#define T_SCALE 1.0

#define INIT_CAR_X 570
#define INIT_CAR_Y 650

#define BTM_X INIT_CAR_X // (x,y) coordinates with origin on bottomo left
#define BTM_Y SIM_Y-INIT_CAR_Y

#define LF 3.0					// car length to front from centre of gravity
#define LR 2.0					// car length to back from centre of gravity
#define WD 2.0					// car width
#define L (LF+LR)				// car length
#define W_PX (L/SCALE)
#define H_PX (WD/SCALE)
#define G 9.8					// g constant

// keeps track of maximul number of initial position of car
// that can be set on track
#define POOL_DIM 13
// used as dimension for array that keeps
// the values of actions from Q matrix, inside the RL graph gui
#define BUF_LEN ((MAX_THETA*2)/ACTIONS_STEP)+1

//-------------------------------SENSOR--------------------------------------
#define SMAX 100 // lidar beam max distance
#define STEP 1 // lidar resolution(m) = 1px

//-------------------------------TASKS---------------------------------------
#define MAX_TASKS 10
// handles command interpreter
#define COM_INTERP_ID 1
#define COM_INTERP_PRIO 40
#define COM_INTERP_PER 20	// ms
#define COM_INTERP_DLR 20

// handles agent state update 
#define AGENT_ID 2
#define AGENT_PRIO 10
#define AGENT_PER 20	// ms
#define AGENT_DLR 20
// handles learning process 
#define LEARNING_ID 4
#define LEARNING_PRIO 10
#define LEARNING_PER 20	// ms
#define LEARNING_DLR 20
// handles sensors reading
#define SENSORS_ID 3
#define SENSORS_PRIO 10
#define SENSORS_PER 20	// ms
#define SENSORS_DLR 20
// handles neural network

// handles graphics 
#define GRAPHICS_ID 0
#define GRAPHICS_PRIO 5
#define GRAPHICS_PER 25	// ms
#define GRAPHICS_DLR 25

/*-----------------------------------------------------------------------------*/
/*								CUSTOM STRUCTURES							   */
/*-----------------------------------------------------------------------------*/
// Integer coordinates inside a virtual screen or bitmap
struct ViewPoint {
	int x;
	int y;
};

// structure used to simulate a lidar beam
// (x,y) are pixel coordinates
// alpha is the inclination angle of the beam
// d is the measured distance
struct Lidar {
	int x;	
	int y;
	float alpha;
	int d;
};

// (x,y) are coordinates in meters
// v is the car velocity in m/s
// theta is the car orientation in (0,2pi) radians 
struct Car {
	float x;
	float y;
	float v;
	float theta;							// heading angle
};

// structure of the input that the agent 
// can use to control the car
// a = acceleration in m/s2
// delta = steering cmd in rad
struct Controls {
	float a;
	float delta;						// steering angle
};

// simulate an intelligent agent that drives the car on track
struct Agent {
	struct Car car; // car to control
	int alive;	// 1 = alive, 0 = dead due to collision with track borders
	float distance;	// distance raced on track 
	struct Controls action;	// actions performed on last step 
	int state;	// current state from Q matrix
	struct Actions_ID a_id;	// action IDs of last step
	struct EpisodeStats ep_stats; // statistics of current episode
};

/*-----------------------------------------------------------------------------*/
/*								FUNCTION PROTOTYPES							   */
/*-----------------------------------------------------------------------------*/

//----------------------------- GUI ----------------------------------------
void init();
void init_agent();
void init_scene();
void draw_track();
void draw_car();
void draw_sensors();
void update_scene();
void reset_scene();
void show_dmiss();
void show_rl_graph();
void show_gui_interaction_instructions();
//------------------------------- Tasks --------------------------------------
void* comms_task(void* arg);
void* display_task(void* arg);
void* agent_task(void* arg);
void* learning_task(void* arg);
void* sensors_task(void* arg);

//-------------------------------- Sensors -----------------------------------
int read_sensor(int x0, int y0, float alpha);
void refresh_sensors();
void get_updated_lidars_distance(struct Car car, struct Lidar car_sensors[]);
//-------------------------------- Commands Interpreter -----------------------------------
char interpreter();
//-------------------------------- Reinforcement Learning --------------------

void init_qlearn_training_mode();
void init_qlearn_inference_mode();
struct Car update_car_model(struct Agent agent);
void crash_check();
int is_car_offtrack(struct Car car);
float action_to_steering(int action_k);
float action_to_acc(int action_a);
int decode_lidar_to_state(int d_left, int d_right, int d_front);
float get_reward(struct Agent agent, int d_left, int d_front, int d_right);
struct Agent learn_to_drive(struct Agent old_agent);
void single_thread_learning();

void save_episodes_stats_to_file();
void save_Q_matrix_to_file();
void read_Q_matrix_from_file();
void save_Q_vel_matrix_to_file();
void read_Q_vel_matrix_from_file();
void init_pool_poses();

//-------------------------------- UTILS --------------------------------------

void find_rect_vertices(struct ViewPoint vertices[], int size, struct Car car);
int check_color_px_in_line(int x1, int y1, int x0, int y0, int color);
float deg_to_rad(float deg_angle);
float rad_to_deg(float rad_angle);
char get_scancode();
fixed deg_to_fixed(float deg);
void write_debug();
