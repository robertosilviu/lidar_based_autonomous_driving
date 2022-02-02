#include <stdio.h>
#include <allegro.h>
#include <pthread.h>

/*-----------------------------------------------------------------------------*/
/*								CONSTANTS									   */
/*-----------------------------------------------------------------------------*/
#define PRINTLINE printf("LINE: %d\n", __LINE__)
#define LEN 50 // length of array of chars
#define BUF_LEN 10
#define GRAPH_H 300
#define GRAPH_W 500
//-------------------------------REINFORCEMENT LEARNING-------------------------
#define RWD_CRASH -100
#define RWD_ALIVE 1
#define RWD_CORRECT_TURN 5
#define RWD_WRONG_TURN -5
#define RWD_STRAIGHT 6
#define RWD_TURN_STRAIGHT -6
#define MAX_STATES_LIDAR 20
//-------------------------------GRAPHICS---------------------------------------
#define WIN_X 1024
#define WIN_Y 1024
#define BITS_COL 24
#define BKG 0
#define BLACK 0
#define ASPHALT 7303283
//#define WHITE 15

#define DMISS_H 200
#define SIM_X 800
#define SIM_Y 800
#define XCEN_SIM (SIM_X/2)
#define YCEN_SIM (SIM_Y/2)
#define SCALE 0.16 // 1px = 0.16m
#define T_SCALE 0.4

#define INIT_CAR_X 500
#define INIT_CAR_Y 350
#define BTM_X INIT_CAR_X // (x,y) coordinates with origin on bottomo left
#define BTM_Y SIM_Y-INIT_CAR_Y
#define MAX_THETA 15
#define MIN_THETA -15
#define LF 3.0					// car length to front from centre of gravity
#define LR 2.0					// car length to back from centre of gravity
#define WD 2.0					// car width
#define L (LF+LR)
#define W_PX (L/SCALE)
#define H_PX (WD/SCALE)
#define WHEELBASE 4.0
#define C_R 0.8 				// friction coefficient
#define C_A 2.0					// aerodynamic coefficient
#define G 9.8
#define MAX_A (0.5*G)
#define MIN_A (-0.5*G)
#define MAX_V 300.0
#define MIN_V 0.0
#define MAX_AGENTS 2
#define CRASH_DIST 3
#define INFERENCE 1
#define TRAINING 0
#define TRAIN_VEL 5.0
//-------------------------------SENSOR--------------------------------------
#define SMAX 100 // lidar beam max distance
#define STEP 1 // lidar resolution(m) = 1px

//-------------------------------TASKS---------------------------------------
#define MAX_TASKS 10
// handles command interpreter
#define COM_INTERP_ID 1
#define COM_INTERP_PRIO 40
#define COM_INTERP_PER 40	// ms
#define COM_INTERP_DLR 40

// handles agent state update 
#define AGENT_ID 2
#define AGENT_PRIO 10
#define AGENT_PER 10	// ms
#define AGENT_DLR 10
// handles sensors reading
#define SENSORS_ID 3
#define SENSORS_PRIO 10
#define SENSORS_PER 10	// ms
#define SENSORS_DLR 10
// handles neural network

// handles graphics 
#define GRAPHICS_ID 0
#define GRAPHICS_PRIO 20
#define GRAPHICS_PER 20	// ms
#define GRAPHICS_DLR 20
/*-----------------------------------------------------------------------------*/
/*								CUSTOM STRUCTURES							   */
/*-----------------------------------------------------------------------------*/
// Integer coordinates inside a virtual screen or bitmap
struct ViewPoint {
	int x;
	int y;
};

struct Lidar {
	int x;
	int y;
	float alpha;
	int d;
};

struct Car {
	float x;
	float y;
	float v;
	//float vx;
	//float vy;
	//float a;
	//float delta;						// steering angle
	float theta;							// heading angle
};

struct Controls {
	float a;
	float delta;						// steering angle
};

struct Agent {
	struct Car car;
	int alive; // 1 = alive, 0 = dead due to collision with track borders
	float distance;	// distance raced on track 
	struct Controls action;
	int state;
	float error;
	// da aggiungere il reward
};

struct cbuf {
	int head;
	int tail;
	int x[BUF_LEN];
	float y[BUF_LEN];
};
/*-----------------------------------------------------------------------------*/
/*								GLOBAL VARIABLES							   */
/*-----------------------------------------------------------------------------*/
static const char *TRACK_FILE = "img/track_4.tga";
BITMAP *track_bmp = NULL;
BITMAP *scene_bmp = NULL;
BITMAP *debug_bmp = NULL;
BITMAP *deadline_bmp = NULL;
BITMAP *graph_bmp = NULL;

struct Agent agent; // to be removed
struct Agent agents[MAX_AGENTS];
int end = 0;
char debug[LEN];
int mode = TRAINING;
struct Lidar sensors[MAX_AGENTS][3];

struct cbuf graph_buff;
int episode = 1;
int graph_index = 0;
//---------------------------------------------------------------------------
// GLOBAL SEMAPHORES
//---------------------------------------------------------------------------
pthread_mutex_t			mux_agent, mux_sensors, mux_cbuffer; // define 3 mutex

pthread_mutexattr_t 	matt;			// define mutex attributes


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
//------------------------------- Tasks --------------------------------------
void* comms_task(void* arg);
void* display_task(void* arg);
void* agent_task(void* arg);
void* sensors_task(void* arg);

//-------------------------------- Sensors -----------------------------------
int read_sensor(int x0, int y0, float alpha);
void refresh_sensors();
void get_updated_lidars_distance(struct Car car, struct Lidar car_sensors[]);
//-------------------------------- Reinforcement Learning --------------------

void init_qlearn_params();
struct Car update_car_model(struct Agent agent);
void crash_check();
float action_to_steering(int action_k);
int decode_lidar_to_state(int d_left, int d_right, int d_front);
int get_reward(struct Agent agent, int d_left, int d_front, int d_right);
//int next_state(int a, int agent_id);
float learn_to_drive();

//-------------------------------- UTILS --------------------------------------
void find_rect_vertices(struct ViewPoint vertices[], int size, int id);
int check_color_px_in_line(int x1, int y1, int x0, int y0, int color);
float deg_to_rad(float deg_angle);
float rad_to_deg(float rad_angle);
char get_scancode();
fixed deg_to_fixed(float deg);
void push_to_cbuf(int x, float y);
int is_cbuff_empty();
void write_debug();
