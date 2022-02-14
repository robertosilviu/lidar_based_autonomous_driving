#include <stdlib.h>
#include <math.h>

#include "../libs/ptask/ptask.h"
#include "../libs/tlib/tlib.h"
#include "../libs/qlearn/qlearn.h"
#include "autonomous_driving.h"


int main() {
	int ret;
	printf("Starting app...\n");

	init();
	//printf("here app...\n");
	if (mode == TRAINING) {
		ret = wait_for_task(LEARNING_ID);
	} else {
		ret = wait_for_task(GRAPHICS_ID);
		ret = wait_for_task(COM_INTERP_ID);
		ret = wait_for_task(AGENT_ID);
		ret = wait_for_task(SENSORS_ID);
	}
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
	int w, h, i;
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
	pthread_mutex_init(&mux_cbuffer, &matt);
	pthread_mutex_init(&mux_q_matrix, &matt);

	pthread_mutexattr_destroy(&matt); 	//destroy attributes

	// initialzie circular buffer
	graph_buff.head = -1;
	graph_buff.tail = -1;
	for(i = 0; i < BUF_LEN; i++) {
		graph_buff.x[i] = 0;
		graph_buff.y[i] = 0.0;
	}
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
	init_pool_poses();
	init_agent();
	init_qlearn_params();
	init_scene();
	refresh_sensors();
	update_scene();
	
	if (mode == TRAINING) {
		task_create(learning_task, LEARNING_ID, LEARNING_PER, LEARNING_DLR, LEARNING_PRIO);
	} else {
		task_create(display_task, GRAPHICS_ID, GRAPHICS_PER, GRAPHICS_DLR, GRAPHICS_PRIO);
		task_create(comms_task, COM_INTERP_ID, COM_INTERP_PER, COM_INTERP_DLR, COM_INTERP_PRIO);
		task_create(agent_task, AGENT_ID, AGENT_PER, AGENT_DLR, AGENT_PRIO);
		task_create(sensors_task,SENSORS_ID,SENSORS_PER,SENSORS_DLR,SENSORS_PRIO);
	}
	
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
	draw_track();
	draw_car();
	if(!disable_sensors)
		draw_sensors();
	//crash_check();
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
		find_rect_vertices(vertices, 4, agents[i].car);
		points[0] = vertices[0].x;
		points[1] = vertices[0].y;
		points[2] = vertices[1].x;
		points[3] = vertices[1].y;
		points[4] = vertices[2].x;
		points[5] = vertices[2].y;
		points[6] = vertices[3].x;
		points[7] = vertices[3].y;
		polygon(scene_bmp, 4, points, makecol(51,153,255));
	}
	pthread_mutex_unlock(&mux_agent);
}

void find_rect_vertices(struct ViewPoint vertices[], int size, struct Car car) {
	float ca, sa;
	float x1, y1, x2, y2, x3, y3, x4, y4;
	//struct Car car;
	if(size != 4) {
		printf("ERROR: find_rect_vertices requires array with dimension = 4!\n");
		exit(1);
	}
	//car = agents[id].car;
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
	new_car.x = pose_pool[pool_index][0];
	new_car.y = pose_pool[pool_index][1];
	new_car.theta = pose_pool[pool_index][2];

	for(i = 0; i < MAX_AGENTS; i++) {
		dead_agents[i] = 0;
	}
	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		find_rect_vertices(vertices[i], 4, agents[i].car);
		// use y=mx +b
		found = 0;
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

		if (dead_agents[i] == 1) {
			agents[i].alive = 0;
			//agents[i].car.v = 0.0;
			agents[i].car = new_car;
			//printf("dead agent %d\n", i);
			// at each restart the agent should change some parameters for the next simulation
		}
	}
	pthread_mutex_unlock(&mux_agent);
}

int is_car_offtrack(struct Car car) {
	int off_track;
	int k;
	struct ViewPoint vertices[4];

	off_track = 0;
	find_rect_vertices(vertices, 4, car);
	for(k = 0; k < 4; k++) {
		off_track = check_color_px_in_line(
					vertices[(k+1)%4].x,
					vertices[(k+1)%4].y,
					vertices[k%4].x,
					vertices[k%4].y,
					0);
	}

	return off_track;
}

int check_color_px_in_line(int x1, int y1, int x0, int y0, int color) {
	// use y=mx +b
	int col;
	//int x, y, min_x, max_x, min_y, max_y, j;
	//float m, b;

	col = getpixel(track_bmp, x0, y0);
	if(col == color)
		return 1;
	col = getpixel(track_bmp, x1, y1);
	if(col == color)
		return 1;
	/*
	if( (x1 - x0) == 0) { // m goes to infinity
		max_y = (y1 > y0) ? y1 : y0;
		min_y = (y1 < y0) ? y1 : y0;
		for(j = min_y; j < max_y; j++) {
			col = getpixel(track_bmp, x0, j);
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
			col = getpixel(track_bmp, x, y);
			if(col == color)
				return 1;
		}
	}
	*/
	// pixel of provided color not found
	return 0;
}

int is_cbuff_empty() {
	if (graph_buff.tail == graph_buff.head)
		return 1;
	return 0;
}

void push_to_cbuf(int x, int y, int id) {

	//pthread_mutex_lock(&mux_cbuffer);
	
	// save the data
	graph_buff.x[id] = x;
	graph_buff.y[id] = y;
	//printf("adding: %d, %d \n", x, y);
	//pthread_mutex_unlock(&mux_cbuffer);
}

void show_rl_graph() {
	int px_h, px_w, white, orange;
	int shift_y_axis = 50;
	int shift_x_axis = 300;
	int x_offset = 50;
	int r = 3;
	struct ViewPoint p1;
	int i, index;	// used for for cycle
	float scale_y, scale_x;
	int g_h;
	char debug[LEN];
	// it needs a mutex for buffer access
	px_h = graph_bmp->h - shift_y_axis;
	px_w = graph_bmp->w - shift_x_axis - x_offset;

	white = makecol(255,255,255);
	orange = makecol(255,99,71);
	clear_to_color(graph_bmp, 0);
	// draw axes
	line(graph_bmp, x_offset, px_h/2, px_w, px_h/2, white);	// x
	line(graph_bmp, x_offset, px_h, x_offset, 0, white);			// y

	pthread_mutex_lock(&mux_cbuffer);
	// get max elem of buffer
	for(i = 0; i < BUF_LEN; i++) {
		if(abs(graph_buff.y[i]) > g_h)
			g_h = abs(graph_buff.y[i]);
	}

	// find scale based on max value stored in buffer
	scale_y = (float)g_h/(px_h/2);
	//printf("scale_y: %f, g_h: %d\n", scale_y, g_h);
	scale_x = px_w/(BUF_LEN+1); // amount of pixel for 1 unit of buffer

	//i = 1;
	for(i = 0; i < BUF_LEN; i++) {
		index = i;
		p1.y = px_h/2 - floor(graph_buff.y[index]/scale_y);
		//memset(debug, 0, sizeof debug);
		//sprintf(debug,"%d", graph_buff.y[index]);
		//textout_ex(graph_bmp, font, debug, 0, p1.y, white, -1);

		p1.x = (x_offset + scale_x + (scale_x * index));
		memset(debug, 0, sizeof debug);
		//sprintf(debug,"ep: %d", graph_buff.x[index]);
		sprintf(debug,"%d", (i*ACTIONS_STEP) - MAX_THETA);
		textout_ex(graph_bmp, font, debug, p1.x, px_h+5, white, -1);
		
		circlefill(graph_bmp, p1.x, p1.y, r, orange);
		line(graph_bmp, p1.x, px_h, p1.x, px_h -8, white);
		//line(graph_bmp, x_offset, p1.y, x_offset + 8, p1.y, white);
	}

	pthread_mutex_unlock(&mux_cbuffer);
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
		show_rl_graph();

		deadline_miss(GRAPHICS_ID);
		wait_for_activation(i);
	}

	return NULL;
}

void* agent_task(void* arg) {
	int i, j, alive_flag;
	//float progress = 0.0;
	struct Car new_car;

	i = get_task_index(arg);
	set_activation(i);

	// initialize empty car to reset dead agent's car
	//new_car.x = 0.0;
	//new_car.y = 0.0;
	//new_car.theta = deg_to_rad(INIT_THETA);
	

	do {	
		crash_check();

		alive_flag = 0;
		// need to update agent when crash occured
		//for(i = 0; i < MAX_AGENTS; i++) {
			if (mode == TRAINING)
				conv_delta += learn_to_drive();
			else if ( mode == INFERENCE)
				printf("Should do inference here\n");
			
			// push error from rl optimization to cbuf
			// should have also the time of pushing
			//push_to_cbuf(episode, progress);
			// reset dead agent
			pthread_mutex_lock(&mux_agent);
			for(j = 0; j < MAX_AGENTS; j++) {
				if (agents[j].alive == 1) {
					alive_flag = 1;
				}
			}
			// no agent alive, start new episode
			if(alive_flag == 0) {
				episode++;

				if((episode%100) == 0) {
					//ql_reduce_expl();
					// change initial position to improve training
					pool_index = (pool_index + 1) % POOL_DIM;
					printf("Changing initial pose!\n");
				}

				for(j = 0; j < MAX_AGENTS; j++) {
						agents[j].alive = 1;
						new_car.v = TRAIN_VEL;
						new_car.x = pose_pool[pool_index][0];
						new_car.y = pose_pool[pool_index][1];
						new_car.theta = pose_pool[pool_index][2];
						agents[j].car = new_car;
						//printf("Reset agent %d!\n", j);
				}
			}
			pthread_mutex_unlock(&mux_agent);
		//}

		deadline_miss(AGENT_ID);
		wait_for_activation(i);
	} while (!end);
	// save learning Q matrix on file for further use
	save_Q_matrix_to_file();

	return NULL;
}

void* learning_task(void* arg) {
	int i, j, alive_flag;
	//float progress = 0.0;
	struct Car new_car;
	char scan;
	int s, a; // agent data
	int d_l, d_r, d_f; // lidars distance

	i = get_task_index(arg);
	set_activation(i);

	pthread_mutex_lock(&mux_sensors);
	for(j = 0; j < MAX_AGENTS; j++) {
		d_l = sensors[j][0].d;
		d_f = sensors[j][1].d;
		d_r = sensors[j][2].d;
		s = decode_lidar_to_state(d_l, d_r, d_f);
		a = ql_egreedy_policy(s);
		// correct only in single thread. should have mux otherwise
		agents[j].state = s;
		// sarsa
		agents[j].a_id = a;
		//printf("action for agent %d is %d\n", i, a[i]);
	}
	pthread_mutex_unlock(&mux_sensors);
	do {
		crash_check();
		alive_flag = 0;
		// need to update agent when crash occured
		//for(i = 0; i < MAX_AGENTS; i++) {
		conv_delta += single_thread_learning();
		// push error from rl optimization to cbuf
		// should have also the time of pushing
		//push_to_cbuf(episode, progress);
		// reset dead agent
		//pthread_mutex_lock(&mux_agent);
		for(j = 0; j < MAX_AGENTS; j++) {
			if (agents[j].alive == 1) {
				alive_flag = 1;
			}
		}
		// no agent alive, start new episode
		if(alive_flag == 0) {
			episode++;
			
			if((episode%100) == 0) {
				//ql_reduce_expl();
				// change initial position to improve training
				pool_index = (pool_index + 1) % POOL_DIM;
				printf("Changing initial pose!\n");
			}

			for(j = 0; j < MAX_AGENTS; j++) {
					agents[j].alive = 1;
					new_car.v = TRAIN_VEL;
					new_car.x = pose_pool[pool_index][0];
					new_car.y = pose_pool[pool_index][1];
					new_car.theta = pose_pool[pool_index][2];
					agents[j].car = new_car;
					//printf("Reset agent %d!\n", j);
			}
			// refresh sensors with initial pose
			refresh_sensors();
			// update initial state for learning
			for(j = 0; j < MAX_AGENTS; j++) {
				d_l = sensors[j][0].d;
				d_f = sensors[j][1].d;
				d_r = sensors[j][2].d;
				s = decode_lidar_to_state(d_l, d_r, d_f);
				a = ql_egreedy_policy(s);
				agents[j].state = s;
				agents[j].a_id = a;
			}
		}
		//pthread_mutex_unlock(&mux_agent);
	//}
		// sensors task
		refresh_sensors();
		// display task
		update_scene();
		clear_to_color(debug_bmp, 0);
		write_debug();
		show_dmiss();
		show_rl_graph();
		// command interpreter task
		scan = interpreter();
		if (scan == KEY_ESC)
			end = 1;
		
		deadline_miss(LEARNING_ID);
		wait_for_activation(i);
	} while (!end);
	// save learning Q matrix on file for further use
	save_Q_matrix_to_file();

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
	char scan;

	i = get_task_index(arg);
	set_activation(i);

	do {
		scan = interpreter();

		deadline_miss(COM_INTERP_ID);
		wait_for_activation(i);
	} while (scan != KEY_ESC);

	end = 1;

	return NULL;
}

char interpreter() {
	int i;
	float delta;
	char scan;
	struct Controls act;

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
				delta += 5.0;
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
			case KEY_L:
				printf("Loading Q matrix from file %s\n", Q_MAT_FILE_NAME);
				pthread_mutex_lock(&mux_q_matrix);
				read_Q_matrix_from_file();
				pthread_mutex_unlock(&mux_q_matrix);
				break;
			case KEY_D:
				if (disable_sensors) {
					printf("Sensors drawing enabled! \n");
					disable_sensors = 0;
				} else {
					printf("Sensors drawing disabled! \n");
					disable_sensors = 1;
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

	return scan;
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

	vehicle.x = pose_pool[pool_index][0];
	vehicle.y = pose_pool[pool_index][1];
	vehicle.theta = pose_pool[pool_index][2];
	vehicle.v = TRAIN_VEL;
	//vehicle.a = 0.0;
	
	for(i = 0; i < MAX_AGENTS; i++) {
		agents[i].alive = 1;
		agents[i].distance = 0.0;
		agents[i].car = vehicle;
		agents[i].error = 0.0;
		agents[i].state = 0; // should be checked if it is ok to use a wrong state
	}
}

void init_pool_poses() {
	pose_pool[0][0] = 0.0;
	pose_pool[0][1] = 0.0;
	pose_pool[0][2] = deg_to_rad(90.0);

	pose_pool[1][0] = -50.0;
	pose_pool[1][1] = 0.0;
	pose_pool[1][2] = deg_to_rad(90.0);

	pose_pool[2][0] = -75.0;
	pose_pool[2][1] = 50.0;
	pose_pool[2][2] = deg_to_rad(90.0);

	pose_pool[3][0] = -60.0;
	pose_pool[3][1] = 70.0;
	pose_pool[3][2] = deg_to_rad(270.0);

	pose_pool[4][0] = 10.0;
	pose_pool[4][1] = 57.0;
	pose_pool[4][2] = deg_to_rad(180.0);

	pose_pool[5][0] = 20.0;
	pose_pool[5][1] = 20.0;
	pose_pool[5][2] = deg_to_rad(270.0);

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
	
	dmiss = param[GRAPHICS_ID].dmiss;
	sprintf(debug,"display_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 50, white, -1);

	memset(debug, 0, sizeof debug);
	dmiss = param[SENSORS_ID].dmiss;
	sprintf(debug,"sensors_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 110, white, -1);

	memset(debug, 0, sizeof debug);
	dmiss = param[AGENT_ID].dmiss;
	sprintf(debug,"agent_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 90, white, -1);

	memset(debug, 0, sizeof debug);
	dmiss = param[LEARNING_ID].dmiss;
	sprintf(debug,"learning_task: %d", dmiss);
	textout_ex(deadline_bmp, font, debug, 10, 70, white, -1);

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
void get_updated_lidars_distance(struct Car car, struct Lidar car_sensors[]) {
	struct Lidar lidar;
	float x_p, y_p;
	
	if (car_sensors == NULL) {
		printf("ERROR: Invalid array! \n");
		exit(1);
	}

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
	car_sensors[0] = lidar;
	// front
	lidar.alpha = car.theta + deg_to_rad(0.0);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	car_sensors[1] = lidar;
	// right
	lidar.alpha = car.theta + deg_to_rad(-45.0);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	car_sensors[2] = lidar;
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

struct Car update_car_model(struct Agent agent) {
	struct Car old_state, new_state;
	struct Controls act;
	float vx, vy, dt;
	float omega;
	// /float friction;
	float norm_theta, theta_deg;

	//pthread_mutex_lock(&mux_agent);
	dt = T_SCALE * (float)AGENT_PER/1000;
	//dt = (float)AGENT_PER/1000;
	//for(i = 0; i < MAX_AGENTS; i++) {
	old_state = agent.car;
	act = agent.action;

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
	// normalize theta to 0 - 360
	//theta_deg = rad_to_deg(old_state.theta);
	//norm_theta = theta_deg - (floor(theta_deg/360.0) * 360.0);
	//norm_theta = deg_to_rad(norm_theta);
	//norm_theta = atan2(sin(old_state.theta), cos(old_state.theta));
	
	//theta_deg = rad_to_deg(norm_theta) + 180;
	//norm_theta = deg_to_rad(theta_deg);

	vx = old_state.v*cos(old_state.theta);
	vy = old_state.v*sin(old_state.theta);

	new_state.x = old_state.x + (vx * dt);
	new_state.y = old_state.y + (vy * dt);

	omega = old_state.v * tan(act.delta) / WHEELBASE;
	norm_theta = old_state.theta + (omega * dt);
	norm_theta = atan2(-sin(norm_theta), -cos(norm_theta)) + M_PI;
	//new_state.theta = norm_theta + (omega * dt);
	new_state.theta = norm_theta;
	//printf("y: %f, theta: %f\n", new_state.y, rad_to_deg(norm_theta));
	return new_state;
	//}
	//pthread_mutex_unlock(&mux_agent);
}

void write_debug() {
	int white;
	int x, y;
	int i;
	struct Agent agent;
	
	x = 0;
	white = makecol(255,255,255);
	clear_to_color(debug_bmp, 0);

	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		if(agents[i].alive)
			agent = agents[i];
	}
	//agent = agents[0];
	pthread_mutex_unlock(&mux_agent);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"x: %f m", agent.car.x);
	textout_ex(debug_bmp, font, debug, x, 20, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"y: %f m", agent.car.y);
	textout_ex(debug_bmp, font, debug, x, 30, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"v: %f m/s", agent.car.v);
	textout_ex(debug_bmp, font, debug, x, 40, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"theta: %f deg", rad_to_deg(agent.car.theta));
	textout_ex(debug_bmp, font, debug, x, 50, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.a: %f m/s^2", agent.action.a);
	textout_ex(debug_bmp, font, debug, x, 60, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.steer: %f rad", agent.action.delta);
	textout_ex(debug_bmp, font, debug, x, 70, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"alive: %d", agent.alive);
	textout_ex(debug_bmp, font, debug, x, 80, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"gamma: %f", ql_get_discount_factor());
	textout_ex(debug_bmp, font, debug, x, 90, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"alpha: %f", ql_get_learning_rate());
	textout_ex(debug_bmp, font, debug, x, 100, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"max steer: %f rad", deg_to_rad(MAX_THETA));
	textout_ex(debug_bmp, font, debug, x, 110, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"episode: %d", episode);
	textout_ex(debug_bmp, font, debug, x, 120, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"epsilon: %f", ql_get_epsilon());
	textout_ex(debug_bmp, font, debug, x, 130, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"max reward: %f", max_reward);
	textout_ex(debug_bmp, font, debug, x, 140, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"convergence delta: %.1f", (conv_delta/episode));
	textout_ex(debug_bmp, font, debug, x, 150, white, -1);

	//pthread_mutex_unlock(&mux_agent);

	x = SIM_X;
	y = (WIN_Y - SIM_Y + 50);
	blit(debug_bmp, screen, 0, 0, x, y, debug_bmp->w, debug_bmp->h);
}

float action_to_steering(int action_k) {
	float x, y, z;
	int n_actions = ql_get_nactions() -1;

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

	delta = (float)(d_left - d_right)/(SMAX+1);
	front = (float)d_front/(SMAX+1);

	y = (delta+1)/2;
	s1 = floor(MAX_STATES_LIDAR * y); // to be checked
	s2 = floor(MAX_STATES_LIDAR * front);
	s = s2*MAX_STATES_LIDAR +s1;

	//printf("s: %d, y: %f, delta: %f\n", s, y, delta);
	return s;
}

float get_reward(struct Agent agent, int d_left, int d_front, int d_right) {
	float r = 0;
	//int d_left, d_front, d_right;
	//struct Agent agent;
	float track_pos; // distance between car's (x,y) and centre of track
	float x, y, alpha; // vx, vy, ca, sa;
	int d_l, d_r;

	track_pos = 0.0;
	
	// compute left distance perpendicular to car's (x,y) 
	alpha = deg_to_rad(+90.0);
	x = BTM_X + (agent.car.x/SCALE);
	y = SIM_Y - (BTM_Y + agent.car.y/SCALE);
	d_l = read_sensor(x, y, alpha);
	// compute right distance
	alpha = deg_to_rad(-90.0);
	x = BTM_X + (agent.car.x/SCALE);
	y = SIM_Y - (BTM_Y + agent.car.y/SCALE);
	d_r = read_sensor(x, y, alpha);
	
	// compute distance from track centre
	/*
	if (d_r > d_l)
		track_pos = d_r - (d_r + d_l)/2;
	else 
		track_pos = d_l - (d_r + d_l)/2;
	*/
	track_pos = d_l - d_r;
	//printf("d_l: %d, d_r: %d, t_p: %f\n", d_l, d_r, track_pos);
	// compute vx and vy
	//ca = cos(agent.car.theta);
	//sa = sin(agent.car.theta);
	//vx = agent.car.v * ca;
	//vy = agent.car.v * sa;
	
	// if car has crashed return bad reward
	if(is_car_offtrack(agent.car)) {
		return RWD_CRASH;
	} else {
		//r = ALPHA_REWARD * (vx + vy - agent.car.v*fabs(track_pos));
		// reward for staying alive
		return RWD_ALIVE;
		//r += RWD_ALIVE;
		// reward for correct turn
		
		if (d_right > d_left) {
			if (agent.action.delta < 0)
				r += RWD_CORRECT_TURN;
			else
				r += RWD_WRONG_TURN;
		}
		if (d_left > d_right) {
			if (agent.action.delta > 0)
				r += RWD_CORRECT_TURN;
			else
				r += RWD_WRONG_TURN;
		}
		
		// reward for no turn on straight
		if ((d_front > d_left) && (d_front > d_right)) {
			if (fabs(agent.action.delta) < deg_to_rad(5))
				r += RWD_STRAIGHT;
			else
				r += RWD_TURN_STRAIGHT;
		}
		// reward for keeping the car near the center
		// normalize to track length
		track_pos = track_pos/12;
		//r -= fabs(track_pos);
	}
	//printf("r: %f\n", r);
	//printf("r: %f, vx: %f, vy: %f, track_pos: %f\n", r, vx, vy, agent.car.v*fabs(track_pos));

	return r;
}

float learn_to_drive() {
	int a[MAX_AGENTS], s[MAX_AGENTS], s_new[MAX_AGENTS];
	float max_err, err;
	float r[MAX_AGENTS];
	int i, curr_s;
	struct Agent agent;
	struct Lidar car_sensors[3];
	int d_l[MAX_AGENTS], d_f[MAX_AGENTS], d_r[MAX_AGENTS]; // lidar distances
	int act;

	max_err = 0.0;
	err = 0.0;

	pthread_mutex_lock(&mux_sensors);
	for(i = 0; i < MAX_AGENTS; i++) {
		d_l[i] = sensors[i][0].d;
		d_f[i] = sensors[i][1].d;
		d_r[i] = sensors[i][2].d;
		s[i] = decode_lidar_to_state(d_l[i], d_r[i], d_f[i]);
		a[i] = ql_egreedy_policy(s[i]);
		//printf("action for agent %d is %d\n", i, a[i]);
	}
	pthread_mutex_unlock(&mux_sensors);

	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		agent = agents[i];
		agent.action.delta = action_to_steering(a[i]);
		agent.action.a = 0.0;
		agent.car = update_car_model(agent);
		

		get_updated_lidars_distance(agent.car, car_sensors);
		d_l[i] = car_sensors[0].d;
		d_f[i] = car_sensors[1].d;
		d_r[i] = car_sensors[2].d;

		s_new[i] = decode_lidar_to_state(d_l[i], d_r[i], d_f[i]);
		r[i] = get_reward(agent, d_l[i], d_f[i], d_r[i]);
		// update max reward for debug
		if (r[i] > max_reward)
			max_reward = r[i];
		//printf(" reward: %f, action %d \n", max_reward, a[i]);
		//err = ql_updateQ(s[i], a[i], r[i], s_new[i]);
		err = ql_updateQ(s[i], a[i], r[i], s_new[i]);
		// update agent state
		agent.state = s_new[i];
		agents[i] = agent;
		// update max error
		// needs better handling of error
		if (err > max_err)
			max_err = err;
	}
	pthread_mutex_unlock(&mux_agent);

	pthread_mutex_lock(&mux_cbuffer);
	for(i = 0; i < BUF_LEN; i++) {
		curr_s = s_new[0]; // save to graph the action of first agent only
		act = ql_get_Q(curr_s, i);
		push_to_cbuf(i, act, i);
		//printf("s: %d, Q: %d\n", curr_s, act);
	}
	pthread_mutex_unlock(&mux_cbuffer);
	// error handling is wrong !!  needs to be changed
	return max_err;
}

void init_qlearn_params() {
	int n_states, n_actions;

	n_states = MAX_STATES_LIDAR*MAX_STATES_LIDAR;
	//n_actions = (MAX_THETA * 2) - 1;
	n_actions = (int)((MAX_THETA * 2)/ACTIONS_STEP) + 1;
	printf("n_states: %d\n",n_states);
	ql_init(n_states, n_actions);
	// modify specific params by calling related function
	ql_set_learning_rate(0.5);
	ql_set_discount_factor(1.0);
	ql_set_expl_range(0.1, 0.01);
	ql_set_expl_decay(0.95);
}
void save_Q_matrix_to_file() {
	FILE *fp;
	int n_states, n_actions;
	int i, j;
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	float Q_tmp[n_states][n_actions];
	
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			Q_tmp[i][j] = ql_get_Q(i, j);
		}
	}

	fp = fopen(Q_MAT_FILE_NAME, "w");
	if (fp == NULL) {
		printf("ERROR: could not open file to write Q matrix\n");
		exit(1);
	}
	// save the number of states and actions
	fprintf(fp, "%d %d\n", n_states, n_actions);
	// save values of Q Matrix
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			fprintf(fp, "%.2f ", Q_tmp[i][j]);
		}
		fprintf(fp, "\n");
	}
	
	fclose(fp);
	printf("Q matrix saved on file %s!\n", Q_MAT_FILE_NAME);
}

void read_Q_matrix_from_file() {
	FILE *fp;
	int dim_states_f, dim_actions_f, n_states, n_actions;
	char buf[50];
	char q_buff[1024];
	int size;
	char *ptr;
	float val;
	int i, j; // indexes to use for matrix

	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	i = 0;
	j = 0;
	fp = fopen(Q_MAT_FILE_NAME, "r");

	size = 50;
	if (fp == NULL) {
		printf("ERROR: could not open file to read Q matrix\n");
		exit(1);
	}
	// check saved Q matrix dimensions from file
	if (fgets(buf, size, fp) != NULL) {
		sscanf(buf, " %d %d", &dim_states_f, &dim_actions_f);
		if (dim_states_f != n_states) {
			printf("ERROR: STATES dimension different from current Q matrix configuration!\n");
			exit(1);
		}
		if (dim_actions_f != n_actions) {
			printf("ERROR: ACTIONS dimension different from current Q matrix configuration!\n");
			exit(1);
		}
	}

	size = 1024;
	while (fgets(q_buff, size, fp) != NULL) {
		ptr = q_buff;
		j = 0;
		while (val = strtof(ptr, &ptr)) {
			ql_set_Q_matrix(i, j, val);
			j++;
		}
		i++;
	}
	
	printf("Q Matrix restored!\n");
}

float single_thread_learning() {
	int a[MAX_AGENTS], s[MAX_AGENTS], s_new[MAX_AGENTS];
	float max_err, err;
	float r[MAX_AGENTS];
	int i, curr_s;
	struct Agent agent;
	struct Lidar car_sensors[3];
	int d_l[MAX_AGENTS], d_f[MAX_AGENTS], d_r[MAX_AGENTS]; // lidar distances
	int act;

	max_err = 0.0;
	err = 0.0;
	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		agent = agents[i];
		// learn only if the agent is still alive
		if (agent.alive == 0)
			continue;

		agent.action.delta = action_to_steering(agent.a_id);
		agent.action.a = 0.0;
		agent.car = update_car_model(agent);
		
		get_updated_lidars_distance(agent.car, car_sensors);
		d_l[i] = car_sensors[0].d;
		d_f[i] = car_sensors[1].d;
		d_r[i] = car_sensors[2].d;

		s[i] = agent.state;
		a[i] = agent.a_id;
		s_new[i] = decode_lidar_to_state(d_l[i], d_r[i], d_f[i]);

		r[i] = get_reward(agent, d_l[i], d_f[i], d_r[i]);

		// update max reward for debug
		if (r[i] > max_reward)
			max_reward = r[i];
		// Q Learning
		//err = ql_updateQ(s[i], a[i], r[i], s_new[i]);
		int a_new = ql_egreedy_policy(s_new[i]);
		//printf("s_new: %d, a: %d, s: %d, r: %f \n", s[i], a[i], s[i], r[i]);
		// Sarsa algorithm
		err = updateQ_sarsa(s[i], a[i], r[i], s_new[i], a_new);

		// update agent state
		agent.state = s_new[i];
		agent.a_id = a_new;
		agents[i] = agent;
		// update max error
		// needs better handling of error
		if (err > max_err)
			max_err = err;
	}
	pthread_mutex_unlock(&mux_agent);
	pthread_mutex_lock(&mux_cbuffer);
	for(i = 0; i < BUF_LEN; i++) {
		curr_s = s_new[0]; // save to graph the action of first agent only
		act = ql_get_Q(curr_s, i);
		push_to_cbuf(i, act, i);
		//printf("s: %d, Q: %d\n", curr_s, act);
	}
	pthread_mutex_unlock(&mux_cbuffer);
	// error handling is wrong !!  needs to be changed
	return max_err;
}

