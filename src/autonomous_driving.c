#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>

#include "../libs/ptask/ptask.h"
#include "../libs/tlib/tlib.h"
/*-----------------------------------------------------------------------------*/
/*								CONSTANTS									   */
/*-----------------------------------------------------------------------------*/

#define LEN 50 // length of array of chars
//-------------------------------GRAPHICS---------------------------------------
#define WIN_X 1024
#define WIN_Y 1024
#define BITS_COL 24
#define BKG 0
#define BLACK 0
#define ASPHALT 7303283
//#define WHITE 15

#define SIM_X 800
#define SIM_Y 800
#define XCEN_SIM SIM_X/2
#define YCEN_SIM SIM_Y/2
#define SCALE 0.16 // 1px = 0.16m
#define T_SCALE 0.4
#define PI 3.14

static const char *TRACK_FILE = "img/track_4.tga";
static const char *CAR_FILE = "img/small_car.tga";

#define INIT_CAR_X 500
#define INIT_CAR_Y 350
#define MAX_THETA 5
#define MIN_THETA -5
#define LF 3					// car length to front from centre of gravity
#define LR 2					// car length to back from centre of gravity
#define L LF+LR
#define G 9.8
#define MAX_A 1.0*G
#define MIN_A -1.0*G
#define MAX_V 300.0
#define MIN_V 0.0
//-------------------------------SENSOR--------------------------------------
#define SMAX 50 // lidar beam max distance
#define STEP 1 // lidar resolution

//-------------------------------TASKS---------------------------------------
#define MAX_TASKS 10
// handles command interpreter
#define COM_INTERP_ID 1
#define COM_INTERP_PRIO 10
#define COM_INTERP_PER 40	// ms
#define COM_INTERP_DLR 40

// handles agent state update 
#define AGENT_ID 2
#define AGENT_PRIO 1
#define AGENT_PER 20	// ms
#define AGENT_DLR 20
// handles sensors reading

// handles neural network

// handles graphics 
#define GRAPHICS_ID 0
#define GRAPHICS_PRIO 30
#define GRAPHICS_PER 20	// ms
#define GRAPHICS_DLR 20
/*-----------------------------------------------------------------------------*/
/*								CUSTOM STRUCTURES							   */
/*-----------------------------------------------------------------------------*/
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
	struct Lidar left_lidar;
	struct Lidar front_lidar;
	struct Lidar right_lidar;
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
	// da aggiungere il reward
};

/*-----------------------------------------------------------------------------*/
/*								GLOBAL VARIABLES							   */
/*-----------------------------------------------------------------------------*/
BITMAP *track_bmp = NULL;
BITMAP *car_bmp = NULL;
BITMAP *scene_bmp = NULL;

struct Agent agent;
int end = 0;

char debug[LEN];

//---------------------------------------------------------------------------
// GLOBAL SEMAPHORES
//---------------------------------------------------------------------------
pthread_mutex_t			mux_agent; // define 3 mutex

pthread_mutexattr_t 	matt;			// define mutex attributes
/*-----------------------------------------------------------------------------*/
/*								FUNCTION PROTOTYPES							   */
/*-----------------------------------------------------------------------------*/
void init();
void init_agent();
void init_scene();
int read_sensor(int x0, int y0, float alpha);
void get_track_bbox(float *length, float *height);
void draw_track();
void draw_car();
void draw_sensors();
void update_scene();

void update_car_model();
void update_sensors();
void* comms_task(void* arg);
void* display_task(void* arg);
void* agent_task(void* arg);
int crash_check();
void reset_scene();
//--------------------------------UTILS---------------------------------------
float deg_to_rad(float deg_angle);
float rad_to_deg(float rad_angle);
char get_scancode();
fixed deg_to_fixed(float deg);

void write_debug();

int main() {
	int ret;
	printf("Starting app...\n");

	init();
	//printf("here app...\n");
	ret = wait_for_task(GRAPHICS_ID);
	ret = wait_for_task(COM_INTERP_ID);
	if(ret != 0) {
		printf("ERROR: error while waiting for thread\n");
	}
	//readkey();
	destroy_bitmap(track_bmp);
	destroy_bitmap(scene_bmp);
	allegro_exit();

	return 0;
}

void init() {
	allegro_init();
	install_keyboard();
	///install_mouse();
	//show_mouse(screen);

	set_color_depth(BITS_COL);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, WIN_X, WIN_Y, 0, 0);
	clear_to_color(screen, BKG);

	// MUTEX PROTOCOLS
	pthread_mutexattr_init(&matt);
	pthread_mutexattr_setprotocol(&matt, PTHREAD_PRIO_INHERIT);
	// create 3 semaphores with Priority inheritance protocol
	pthread_mutex_init(&mux_agent, &matt);

	pthread_mutexattr_destroy(&matt); 	//destroy attributes

	// organize app windows
	//int white = makecol(255, 255, 255);
	//rect(screen,0, WIN_Y-1, SIM_X, WIN_Y-SIM_Y, white); // track area
	init_scene();
	init_agent();
	draw_sensors();
	//update_scene();

	task_create(display_task, GRAPHICS_ID, GRAPHICS_PER, GRAPHICS_DLR, GRAPHICS_PRIO);
	task_create(comms_task, COM_INTERP_ID, COM_INTERP_PER, COM_INTERP_DLR, COM_INTERP_PRIO);
//
}

void init_scene() {
	fixed rot;
	//set_color_conversion(COLORCONV_32_TO_24);
	track_bmp = load_bitmap(TRACK_FILE, NULL);
	if (track_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}

	scene_bmp = load_bitmap(TRACK_FILE, NULL);
	if (scene_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}

	car_bmp = load_bitmap(CAR_FILE, NULL);
	if (car_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}
	
	rot = deg_to_fixed(0.0);

	rotate_sprite(scene_bmp, car_bmp, INIT_CAR_X, INIT_CAR_Y, rot);
	//draw_sprite(scene_bmp, car_bmp, INIT_CAR_X, INIT_CAR_Y);
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void update_scene() {
	int crash_flag = 0;

	draw_track();
	draw_car();
	draw_sensors();
	//printf("inside update_scene\n");
	// should be a for cycle for reach agent
	crash_flag = crash_check();
	if (crash_flag == 1) {
		printf(" crash occured\n");
		init_agent();
		draw_track();
		draw_car();
	}
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void draw_track() {
	//set_color_conversion(COLORCONV_32_TO_24);

	blit(track_bmp, scene_bmp, 0, 0, 0, 0, track_bmp->w, track_bmp->h);
}

// rotate car sprite inside the scene using fixed points convention
void draw_car() {
	float theta_deg;
	int x, y;
	fixed rot; // allegro type to express fixed point number

	theta_deg = rad_to_deg(agent.car.theta);
	rot = deg_to_fixed(theta_deg);

	x = INIT_CAR_X + agent.car.x/SCALE;
	y = INIT_CAR_Y - agent.car.y/SCALE; 
	//rotate_sprite(scene_bmp, car_bmp, agent.car.x/SCALE, agent.car.y/SCALE, rot);
	rotate_sprite(scene_bmp, car_bmp, x, y, rot);
	//draw_sprite(scene_bmp, car_bmp, INIT_CAR_X, INIT_CAR_Y);
}

// draw the lidar beams on the TRACK sprite
void draw_sensors() {
	int x, y;
	int col;
	struct Lidar lidar;
	col = makecol(255,255,255);

	lidar = agent.car.left_lidar;
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);
	//printf(" left_lidar: (%d,%d) \n", x, y);

	lidar = agent.car.front_lidar;
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);
	//printf(" front_lidar: (%d,%d) \n", x, y);

	lidar = agent.car.right_lidar;
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);
	//printf(" right_lidar: (%d,%d) \n", x, y);
}

// checks if car has any collision with the track borders
// if there are collisions return 1, otherwise returns 0
int crash_check() {
	int color, x, y;
	int h, w;
	
	h = (int)car_bmp->w; // the real image is vertical so h and w are inverted
	w = (int)car_bmp->h;
	// upper left corner
	x = INIT_CAR_X + (int)agent.car.x;
	y = INIT_CAR_Y + (int)agent.car.y;
	color = getpixel(track_bmp,x,y);
	if (color == BLACK)
		return 1;
	// upper right corner
	x += w;
	color = getpixel(track_bmp,x,y);
	if (color == BLACK)
		return 1;
	// down right corner
	y += h;
	color = getpixel(track_bmp,x,y);
	if (color == BLACK)
		return 1;
	// down left corner
	x -= w;
	color = getpixel(track_bmp,x,y);
	if (color == BLACK)
		return 1;

	// no collision
	return 0;
}

void* display_task(void* arg) {
	struct Controls action;
	int i;
	
	i = get_task_index(arg);
	set_activation(i);

	while(!end) {
		//action.a = 0.0;
		//action.delta = 0;

		update_car_model();
		update_scene();
		//write_debug();

		wait_for_activation(i);
	}

	return NULL;
}

void* comms_task(void* arg) {
	int i;
	float delta;
	char scan;
	struct Controls act;
	struct Car car;

	i = get_task_index(arg);
	set_activation(i);

	do {
		if (keypressed()) {
			pthread_mutex_lock(&mux_agent);
			act = agent.action;
			car = agent.car;
			delta = rad_to_deg(act.delta);
			scan = get_scancode();
			switch (scan) {
				case KEY_UP:
					act.a += (0.1 * G);
					if (act.a > MAX_A)
						act.a = MAX_A;
					//car.v += 1;
					//if (car.v > MAX_V)
					//	car.v = MAX_V;
					break;
				case KEY_DOWN:
					act.a -= (0.1 * G);
					if (act.a < MIN_A)
						act.a = MIN_A;
					//car.v -= 1;
					break;
				case KEY_LEFT:
					delta += 1.0;
					if (delta > MAX_THETA)
						delta = MAX_THETA;
					act.delta = deg_to_rad(delta);
					break;
				case KEY_RIGHT:
					delta -= 1.0;
					if (delta < MIN_THETA)
						delta = MIN_THETA;
					act.delta = deg_to_rad(delta);
					break;
				default:
					break;
			}

			//printf("delta: %f, d_rad: %f\n", delta, act.delta);
			agent.action = act;
			//printf("a: %f, delta: %f \n", agent.action.a, agent.action.delta);
			//agent.car = car;
			pthread_mutex_unlock(&mux_agent);
		}
		wait_for_activation(i);
	} while (scan != KEY_ESC);

	end = 1;

	return NULL;
}

float deg_to_rad(float deg_angle) {
	return deg_angle*PI/180;
}

float rad_to_deg(float rad_angle) {
	return rad_angle*180/PI;
}

fixed deg_to_fixed(float deg) {
	float angle = 0.0;

	angle = 64 - deg*(float)256/360;
	//printf("angle is %f\n", angle);
	return ftofix(angle);
}

char get_scancode() {
	if (keypressed())
		return readkey() >> 8;
	else
		return 0;
}

void init_agent() {
	struct Car vehicle;
	
	printf("Initializing agent...\n");

	//vehicle.x = INIT_CAR_X;
	//vehicle.y = INIT_CAR_Y;
	vehicle.x = 0;
	vehicle.y = 0;
	vehicle.theta = 0.0;
	vehicle.v = 0.0;
	//vehicle.a = 0.0;

	vehicle.left_lidar.x = INIT_CAR_X + (int)vehicle.x + car_bmp->w;
	vehicle.left_lidar.y = INIT_CAR_Y + (int)vehicle.y;
	vehicle.left_lidar.alpha = deg_to_rad(45.0);
	vehicle.left_lidar.d = read_sensor(
							vehicle.left_lidar.x, 
							vehicle.left_lidar.y, 
							vehicle.theta + vehicle.left_lidar.alpha);

	vehicle.front_lidar.x = INIT_CAR_X + (int)vehicle.x + car_bmp->w;
	vehicle.front_lidar.y = INIT_CAR_Y + vehicle.y + (car_bmp->h/2);
	vehicle.front_lidar.alpha = 0.0;
	vehicle.front_lidar.d = read_sensor(
							vehicle.front_lidar.x, 
							vehicle.front_lidar.y, 
							vehicle.theta + vehicle.front_lidar.alpha);

	vehicle.right_lidar.x = INIT_CAR_X + vehicle.x + car_bmp->w;
	vehicle.right_lidar.y = INIT_CAR_Y + vehicle.y + car_bmp->h;
	vehicle.right_lidar.alpha = deg_to_rad(-45.0);
	vehicle.right_lidar.d = read_sensor(
							vehicle.right_lidar.x, 
							vehicle.right_lidar.y, 
							vehicle.theta + vehicle.right_lidar.alpha);
	
	agent.car = vehicle;
	agent.alive = 1;
	agent.distance = 0.0;
}

int read_sensor(int x0, int y0, float alpha) {
	int col;
	int x, y;
	int d = 1;

	do {
		x = x0 + d*cos(alpha);
		y = y0 - d*sin(alpha);
		col = getpixel(track_bmp, x, y);
		d = d + STEP;
	} while ((d <= SMAX) && (col == ASPHALT));

	return d;
}

void update_car_model() {
	struct Car old_state, new_state;
	struct Controls act;
	float vx, vy, dt;
	float beta, r, omega;

	pthread_mutex_lock(&mux_agent);
	//dt = T_SCALE * AGENT_PER;
	dt = T_SCALE * (float)AGENT_PER/1000;
	//dt = (float)AGENT_PER/1000;
	old_state = agent.car;
	act = agent.action;

	beta = atan2(LR*tan(act.delta), L);
	r = L/( (tan(act.delta)*cos(beta)) );
	omega = old_state.v/r;

	vx = old_state.v*cos(old_state.theta+beta);
	vy = old_state.v*sin(old_state.theta+beta);
	new_state.x = old_state.x + (vx*dt);
	//printf("vx is: %f\n", vx*dt);
	new_state.y = old_state.y + (vy*dt);
	new_state.theta = old_state.theta + (omega*dt);
	//printf("new theta is %f \n", new_state.theta);
	new_state.v = old_state.v + act.a*dt; // to be checked
	//new_state.v = old_state.v;
	//new_state.a = old_state.a;
	// assuming constant velocity 
	// if acceleration is present, the velocity must be updated too
	agent.car = new_state;
	write_debug();

	//printf(" new v: %f, theta: %f, delta: %d\n", new_state.v, new_state.theta, act.delta);
	pthread_mutex_unlock(&mux_agent);
}

void write_debug() {
	int white;
	int y, y_max;

	// refresh data on screen
	y_max = 70;
	for(y = 20; y <= y_max; y+=10) {
		textout_ex(screen, font, debug, 10, y, BLACK, BLACK);
	}

	white = makecol(255,255,255);

	sprintf(debug,"x: %f", agent.car.x);
	textout_ex(screen, font, debug, 10, 20, white, -1);

	sprintf(debug,"y: %f", agent.car.y);
	textout_ex(screen, font, debug, 10, 30, white, -1);

	sprintf(debug,"v: %f", agent.car.v);
	textout_ex(screen, font, debug, 10, 40, white, -1);

	sprintf(debug,"theta: %f", agent.car.theta);
	textout_ex(screen, font, debug, 10, 50, white, -1);

	sprintf(debug,"act.a: %f", agent.action.a);
	textout_ex(screen, font, debug, 10, 60, white, -1);

	sprintf(debug,"act.delta: %f", agent.action.delta);
	textout_ex(screen, font, debug, 10, 70, white, -1);
}