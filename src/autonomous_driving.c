#include <stdlib.h>
#include <math.h>

#include "autonomous_driving.h"
#include "../libs/ptask/ptask.h"
#include "../libs/tlib/tlib.h"
//#include "../libs/qlearn/qlearn.h"


int main() {
	int ret;
	printf("Starting app...\n");

	init();
	//printf("here app...\n");
	ret = wait_for_task(GRAPHICS_ID);
	ret = wait_for_task(COM_INTERP_ID);
	ret = wait_for_task(AGENT_ID);
	ret = wait_for_task(SENSORS_ID);
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
	int w, h;
	allegro_init();
	install_keyboard();

	set_color_depth(BITS_COL);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, WIN_X, WIN_Y, 0, 0);
	clear_to_color(screen, BKG);

	// MUTEX PROTOCOLS
	pthread_mutexattr_init(&matt);
	pthread_mutexattr_setprotocol(&matt, PTHREAD_PRIO_INHERIT);
	// create 3 semaphores with Priority inheritance protocol
	pthread_mutex_init(&mux_agent, &matt);
	pthread_mutex_init(&mux_sensors, &matt);

	pthread_mutexattr_destroy(&matt); 	//destroy attributes

	// organize app windows
	// graph window
	w = WIN_X;
	h = WIN_Y - SIM_Y;
	graph_bmp = create_bitmap(w, h);
	// deadline window
	w = WIN_X - SIM_X;
	h = DMISS_H;
	deadline_bmp = create_bitmap(w, h);
	// debug window
	w = WIN_X - SIM_X;
	h = SIM_Y - DMISS_H;
	debug_bmp = create_bitmap(w, h);
	//rect(screen,0, WIN_Y-1, SIM_X, WIN_Y-SIM_Y, white); // track area
	// track window
	init_agent();
	init_scene();
	refresh_sensors();
	update_scene();

	task_create(display_task, GRAPHICS_ID, GRAPHICS_PER, GRAPHICS_DLR, GRAPHICS_PRIO);
	task_create(comms_task, COM_INTERP_ID, COM_INTERP_PER, COM_INTERP_DLR, COM_INTERP_PRIO);
	task_create(agent_task, AGENT_ID, AGENT_PER, AGENT_DLR, AGENT_PRIO);
	task_create(sensors_task,SENSORS_ID,SENSORS_PER,SENSORS_DLR,SENSORS_PRIO);

}

void init_scene() {
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

	draw_car();

	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void update_scene() {
	// should be a for cycle for reach agent
	crash_check();
	draw_track();
	draw_car();
	draw_sensors();
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void draw_track() {
	blit(track_bmp, scene_bmp, 0, 0, 0, 0, track_bmp->w, track_bmp->h);
}

// rotate car sprite inside the scene using fixed points convention
void draw_car() {
	int points[8];
	int i;
	struct ViewPoint vertices[4];
	
	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		find_rect_vertices(vertices, 4, i);
		points[0] = vertices[0].x;
		points[1] = vertices[0].y;
		points[2] = vertices[1].x;
		points[3] = vertices[1].y;
		points[4] = vertices[2].x;
		points[5] = vertices[2].y;
		points[6] = vertices[3].x;
		points[7] = vertices[3].y;
		polygon(scene_bmp, 4, points, makecol(255,255,255));
	}
	pthread_mutex_unlock(&mux_agent);
}

void find_rect_vertices(struct ViewPoint vertices[], int size, int id) {
	float ca, sa;
	float x1, y1, x2, y2, x3, y3, x4, y4;
	struct Car car;
	if(size != 4) {
		printf("ERROR: find_rect_vertices requires array with dimension = 4!\n");
		exit(1);
	}
	car = agents[id].car;
	ca = cos(car.theta);
	sa = sin(car.theta);
	// p1 = (0, /WD/2)
	x1 = car.x + WD/2*sa;
	y1 = car.y - WD/2*ca;
	// p2 = (0, WD/2)
	x2 = car.x - WD/2*sa;
	y2 = car.y + WD/2*ca;
	// p3 = (L, WD/2)
	x3 = car.x + L*ca - WD/2*sa;
	y3 = car.y + L*sa + (WD/2)*ca;
	// p4 = (L, -WD/2)
	x4 = car.x + L*ca + WD/2*sa;
	y4 = car.y + L*sa - (WD/2)*ca;

	vertices[0].x = BTM_X + x1/SCALE;
	vertices[0].y = SIM_Y - (BTM_Y + y1/SCALE);
	vertices[1].x = BTM_X + x2/SCALE;
	vertices[1].y = SIM_Y - (BTM_Y + y2/SCALE);
	vertices[2].x = BTM_X + x3/SCALE;
	vertices[2].y = SIM_Y - (BTM_Y + y3/SCALE);
	vertices[3].x = BTM_X + x4/SCALE;
	vertices[3].y = SIM_Y - (BTM_Y + y4/SCALE);
}

// draw the lidar beams on the TRACK sprite
void draw_sensors() {
	int x, y, i;
	int col;
	struct Lidar lidar;

	col = makecol(255,255,255);

	pthread_mutex_lock(&mux_sensors);
	for(i = 0; i < MAX_AGENTS; i++) {
		lidar = sensors[i][0];
		x = lidar.x + lidar.d*cos(lidar.alpha);
		y = lidar.y - lidar.d*sin(lidar.alpha);
		line(scene_bmp, lidar.x, lidar.y, x, y, col);

		lidar = sensors[i][1];
		x = lidar.x + lidar.d*cos(lidar.alpha);
		y = lidar.y - lidar.d*sin(lidar.alpha);
		line(scene_bmp, lidar.x, lidar.y, x, y, col);

		lidar = sensors[i][2];		
		x = lidar.x + lidar.d*cos(lidar.alpha);
		y = lidar.y - lidar.d*sin(lidar.alpha);
		line(scene_bmp, lidar.x, lidar.y, x, y, col);
	}
	pthread_mutex_unlock(&mux_sensors);
}

// checks if car has any collision with the track borders
// if there are collisions return 1, otherwise returns 0
void crash_check() {
	int found;
	int i, k; // used for for cycles
	int dead_agents[MAX_AGENTS];
	struct Car new_car;
	struct ViewPoint vertices[MAX_AGENTS][4];
	// initialize empty car to reset dead agent's car
	new_car.v = 0.0;
	new_car.x = 0.0;
	new_car.y = 0.0;
	new_car.theta = 0.0;

	for(i = 0; i< MAX_AGENTS; i++) {
		dead_agents[MAX_AGENTS] = 0;
	}
	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		find_rect_vertices(vertices[i], 4, i);
		// use y=mx +b
		for(k = 0; k < 4; k++) {
			found = check_color_px_in_line(
						vertices[i][(k+1)%4].x,
						vertices[i][(k+1)%4].y,
						vertices[i][k%4].x,
						vertices[i][k%4].y,
						0);
			if(found)
				dead_agents[i] = 1;
		}
		
		if( dead_agents[i] == 1) {
			agents[i].alive = 0;
			agents[i].car = new_car;
			// at each restart the agent should change some parameters for the next simulation
		}
	}
	pthread_mutex_unlock(&mux_agent);
}

int check_color_px_in_line(int x1, int y1, int x0, int y0, int color) {
	// use y=mx +b
	int col, x, y, min_x, max_x, min_y, max_y, j;
	float m, b;
	if( (x1 - x0) == 0) { // m goes to infinity
		max_y = (y1 > y0) ? y1 : y0;
		min_y = (y1 < y0) ? y1 : y0;
		for(j = min_y; j < max_y; j++) {
			col = getpixel(scene_bmp, x0, j);
			if(col == color)
				return 1;
		}
	}else {
		m = (y1 - y0)/(x1 - x0);
		b = y0 - (m * x0);
		max_x = (x1 > x0) ? x1 : x0;
		min_x = (x1 < x0) ? x1 : x0;
		for(j = min_x; j < max_x; j++) {
			x = j;
			y = (int)(m * x) + b;
			col = getpixel(scene_bmp, x, y);
			if(col == color)
				return 1;
		}
	}
	// pixel of provided color not found
	return 0;
}

// needs mutex
void push_to_cbuf(float x, float y) {
	int curr_id;

	curr_id = graph_buff.top;
	curr_id = (curr_id + 1) % BUF_LEN;
	// save the data
	graph_buff.x[curr_id] = curr_id;
	graph_buff.y[curr_id] = y;
	graph_buff.top = curr_id;
}

void show_rl_graph() {
	int px_h, px_w, white;
	struct ViewPoint p1, p2;
	int i;	// used for for cycle
	float scale_y, scale_x, g_h;
	// it needs a mutex for buffer access
	px_h = GRAPH_H;
	px_w = GRAPH_W;
	white = makecol(255,255,255);
	clear_to_color(graph_bmp, 0);

	for(i =0; i < BUF_LEN; i++) {
		if(graph_buff.y[i] > g_h)
			g_h = graph_buff.y[i];
	}

	// find scale based on max value stored in buffer
	scale_y = (float)g_h/px_h;
	scale_x = px_w/BUF_LEN; // amount of pixel for 1 unit of buffer

	for(i = 0; i < (BUF_LEN - 1); i++) {
		p1.y = floor(scale_y * graph_buff.y[i]);
		p1.x = floor(scale_x * i);
		
		p2.y = floor(scale_y * graph_buff.y[i+1]);
		p2.x = floor(scale_x * (i+1));
		
		line(graph_bmp, p1.x, p1.y, p2.x, p2.y, white);
	}

	blit(graph_bmp, screen, 0, 0, 10, 10, graph_bmp->w, graph_bmp->h);
}

void* display_task(void* arg) {
	//struct Controls action;
	int i;
	
	i = get_task_index(arg);
	set_activation(i);

	while(!end) {
		//action.a = 0.0;
		//action.delta = 0;

		update_scene();

		clear_to_color(debug_bmp, 0);
		write_debug();
		show_dmiss();

		wait_for_activation(i);
	}

	return NULL;
}

void* agent_task(void* arg) {
	int i;

	i = get_task_index(arg);
	set_activation(i);

	do {	
		// need to update agent when crash occured 

		update_car_model();
	
		// push error from rl optimization to cbuf
		// should have also the time of pushing
		deadline_miss(AGENT_ID);
		wait_for_activation(i);
	} while (!end);

	return NULL;
}

void* sensors_task(void* arg) {
	int i;

	i = get_task_index(arg);
	set_activation(i);

	do {
		refresh_sensors();

		deadline_miss(SENSORS_ID);
		wait_for_activation(i);
	} while (!end);

	return NULL;
}

void* comms_task(void* arg) {
	int i;
	float delta;
	char scan;
	struct Controls act;
	//struct Car car;

	i = get_task_index(arg);
	set_activation(i);

	do {
		if (keypressed()) {
			pthread_mutex_lock(&mux_agent);
			act = agents[0].action;
			//car = agents[0].car;
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
				case KEY_M:
					if( mode == TRAINING) {
						mode = INFERENCE;
					}else if( mode == INFERENCE) {
						mode = TRAINING;
					}else {
						printf("ERROR: wrong mode! Should be 0 or 1\n");
						exit(1);
					}
					break;
				default:
					break;
			}

			//printf("delta: %f, d_rad: %f\n", delta, act.delta);
			agents[0].action = act;
			//printf("a: %f, delta: %f \n", agents[0].action.a, agents[0].action.delta);
			//agents[0].car = car;
			pthread_mutex_unlock(&mux_agent);
		}

		deadline_miss(COM_INTERP_ID);
		wait_for_activation(i);
	} while (scan != KEY_ESC);

	end = 1;

	return NULL;
}

float deg_to_rad(float deg_angle) {
	return deg_angle*M_PI/180;
}

float rad_to_deg(float rad_angle) {
	return rad_angle*180/M_PI;
}

fixed deg_to_fixed(float deg) {
	float angle = 0.0;

	angle = 128 - deg*(float)256/360;
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
	int i;
	
	printf("Initializing agent...\n");

	vehicle.x = 0;
	vehicle.y = 0;
	vehicle.theta = 0.0;
	vehicle.v = 0;
	//vehicle.a = 0.0;
	
	for(i = 0; i < MAX_AGENTS; i++) {
		agents[i].alive = 1;
		agents[i].distance = 0.0;
		agents[i].car = vehicle;
	}
}

void show_dmiss() {
	char debug[LEN];
	int white, red, dmiss;
	int x, y;

	dmiss = 0;
	white = makecol(255, 255, 255);
	red = makecol(255, 0 , 0);
	clear_to_color(deadline_bmp, 0);

	sprintf(debug,"Tasks deadline misses");
	textout_ex(deadline_bmp, font, debug, 10, 10, red, -1);

	// flush array of chars
	memset(debug, 0, sizeof debug);
	// get deadline miss of task
	dmiss = param[COM_INTERP_ID].dmiss;
	// add the content in the bitmap
	sprintf(debug,"comms_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 30, white, -1);

	memset(debug, 0, sizeof debug);
	deadline_miss(GRAPHICS_ID);
	dmiss = param[GRAPHICS_ID].dmiss;
	sprintf(debug,"display_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 50, white, -1);

	memset(debug, 0, sizeof debug);
	dmiss = param[SENSORS_ID].dmiss;
	sprintf(debug,"sensors_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 70, white, -1);

	memset(debug, 0, sizeof debug);
	dmiss = param[AGENT_ID].dmiss;
	sprintf(debug,"agent_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 90, white, -1);

	x = SIM_X;
	y = (WIN_Y - DMISS_H);
	blit(deadline_bmp, screen, 0, 0, x, y, deadline_bmp->w, deadline_bmp->h);
}

void refresh_sensors() {
	int i;
	struct Lidar lidar;
	struct Car car;
	struct Lidar tmp_sensors[MAX_AGENTS][3];
	float x_p, y_p;
	pthread_mutex_lock(&mux_agent);

	for(i = 0; i < MAX_AGENTS; i++) {
		car = agents[i].car;
		// left lidar
		lidar.alpha = car.theta + deg_to_rad(45.0);
		x_p = car.x + L*cos(car.theta);	// global frame in m from bottom left
		y_p = car.y + L*sin(car.theta);
		//printf("x_p: %f, y_p: %f\n", x_p, y_p);
		lidar.x = BTM_X + (x_p/SCALE);
		lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
		lidar.d = read_sensor(
						lidar.x, 
						lidar.y, 
						lidar.alpha);
		tmp_sensors[i][0] = lidar;
		// front
		lidar.alpha = car.theta + deg_to_rad(0.0);
		lidar.x = BTM_X + (x_p/SCALE);
		lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
		lidar.d = read_sensor(
						lidar.x, 
						lidar.y, 
						lidar.alpha);
		tmp_sensors[i][1] = lidar;
		// right
		lidar.alpha = car.theta + deg_to_rad(-45.0);
		lidar.x = BTM_X + (x_p/SCALE);
		lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
		lidar.d = read_sensor(
						lidar.x, 
						lidar.y, 
						lidar.alpha);
		tmp_sensors[i][2] = lidar;
	}
	pthread_mutex_unlock(&mux_agent);

	// save the updated sensors measurements to global variable
	pthread_mutex_lock(&mux_sensors);
	for(i = 0; i< MAX_AGENTS; i++) {
		sensors[i][0] = tmp_sensors[i][0];
		sensors[i][1] = tmp_sensors[i][1];
		sensors[i][2] = tmp_sensors[i][2];
	}
	pthread_mutex_unlock(&mux_sensors);
}
// consider the scale 1m = 1px
int read_sensor(int x0, int y0, float alpha) {
	int col;
	int x, y;
	int d = 1;

	do {
		x = x0 + d*cos(alpha);
		y = y0 - d*sin(alpha);

		col = getpixel(track_bmp, x, y);
		d = d + STEP;
	} while ((d < SMAX) && (col == ASPHALT));

	return d;
}

void update_car_model() {
	struct Car old_state, new_state;
	struct Controls act;
	float vx, vy, dt;
	float omega;
	// /float friction;
	float norm_theta;
	int i;

	pthread_mutex_lock(&mux_agent);
	//dt = T_SCALE * (float)AGENT_PER/1000;
	dt = (float)AGENT_PER/1000;
	for(i = 0; i < MAX_AGENTS; i++) {
		old_state = agents[i].car;
		act = agents[i].action;

		// CENTRE OF MASS
		/*
		old_state.v = old_state.v + act.a*dt;
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
		//new_state.v = old_state.v + act.a*dt; // to be checked
		new_state.v = old_state.v;
		//new_state.a = old_state.a;
		// assuming constant velocity 
		// if acceleration is present, the velocity must be updated too
		agent.car = new_state;
		*/

		//printf(" new v: %f, theta: %f, delta: %d\n", new_state.v, new_state.theta, act.delta);

		// REAR AXEL 
		//friction = old_state.v * (C_R + C_A * old_state.v);
		new_state.v = old_state.v + dt * (act.a); // - friction);
		norm_theta = atan2(sin(old_state.theta), cos(old_state.theta));
		vx = old_state.v*cos(norm_theta);
		vy = old_state.v*sin(norm_theta);
		omega = old_state.v * tan(act.delta) / WHEELBASE;
		new_state.x = old_state.x + (vx * dt);
		new_state.y = old_state.y + (vy * dt);
		new_state.theta = norm_theta + (omega * dt);

		agents[i].car = new_state;
	}
	pthread_mutex_unlock(&mux_agent);
}

void write_debug() {
	int white;
	int x, y;
	struct Agent agent;
	
	x = 0;
	white = makecol(255,255,255);
	clear_to_color(debug_bmp, 0);

	pthread_mutex_lock(&mux_agent);
	agent = agents[0];
	pthread_mutex_unlock(&mux_agent);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"x: %f", agent.car.x);
	textout_ex(debug_bmp, font, debug, x, 20, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"y: %f", agent.car.y);
	textout_ex(debug_bmp, font, debug, x, 30, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"v: %f", agent.car.v);
	textout_ex(debug_bmp, font, debug, x, 40, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"theta: %f", rad_to_deg(agent.car.theta));
	textout_ex(debug_bmp, font, debug, x, 50, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.a: %f", agent.action.a);
	textout_ex(debug_bmp, font, debug, x, 60, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.delta: %f", agent.action.delta);
	textout_ex(debug_bmp, font, debug, x, 70, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"alive: %d", agent.alive);
	textout_ex(debug_bmp, font, debug, x, 80, white, -1);

	//pthread_mutex_unlock(&mux_agent);

	x = SIM_X;
	y = (WIN_Y - SIM_Y + 50);
	blit(debug_bmp, screen, 0, 0, x, y, debug_bmp->w, debug_bmp->h);
}

float action_to_steering(int action_k) {
	float x, y, z;
	int n_actions = (MAX_THETA*2)-1;

	if ((action_k < 0) || (action_k > n_actions)) {
		printf("ERROR: AGENT action should be an index from 0 to %d\n", n_actions);
		exit(1);
	}

	x = (float)action_k/n_actions;
	y = 2*x -1;
	z = MAX_THETA*y*M_PI/180;

	return z;
}

// combine left and right lidar by getting d_left - d_right
// without Kohonen network
int decode_lidar_to_state(int d_left, int d_right, int d_front) {
	float delta, front, y;
	int s1, s2, s;

	delta = (d_left - d_right)/(SMAX+1);
	front = d_front/SMAX;

	y = (delta+1)/2;
	s1 = floor(SMAX * y); // to be checked
	s2 = floor(SMAX * front);
	s = s2*SMAX +s1;

	return s;
}
