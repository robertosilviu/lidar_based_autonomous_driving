#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "../libs/ptask/ptask.h"
#include "../libs/tlib/tlib.h"
#include "../libs/qlearn/qlearn.h"
#include "autonomous_driving.h"


int main() {
	int ret, mode;
	printf("\n*** Starting app\n");

	init();
	//printf("here app...\n");
	mode = ql_get_rl_mode();

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
	int w, h, i, mode;
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
	pthread_mutex_init(&mux_q_matrix, &matt);

	pthread_mutexattr_destroy(&matt); 	//destroy attributes

	// organize app windows
	// graph window
	w = WIN_X;
	h = (WIN_Y - SIM_Y);
	instructions_bmp = create_bitmap(w, h);

	w = WIN_X;
	h = (WIN_Y - SIM_Y);
	graph_bmp = create_bitmap(w, h);
	// keyboard instructions window
	// deadline window
	w = WIN_X - SIM_X;
	h = DMISS_H;
	deadline_bmp = create_bitmap(w, h);
	// debug window
	w = WIN_X - SIM_X;
	h = SIM_Y - DMISS_H;
	debug_bmp = create_bitmap(w, h);
	//rect(screen,0, WIN_Y-1, SIM_X, WIN_Y-SIM_Y, white); // track area
	init_pool_poses();
	// Q learning related
	init_agent();
	init_qlearn_training_mode();
	// track window
	init_scene();
	refresh_sensors();
	update_scene();
	
	mode = ql_get_rl_mode();

	if (mode == TRAINING) {
		task_create(learning_task, LEARNING_ID, LEARNING_PER, LEARNING_DLR, LEARNING_PRIO);
	} else {
		task_create(display_task, GRAPHICS_ID, GRAPHICS_PER, GRAPHICS_DLR, GRAPHICS_PRIO);
		task_create(comms_task, COM_INTERP_ID, COM_INTERP_PER, COM_INTERP_DLR, COM_INTERP_PRIO);
		task_create(agent_task, AGENT_ID, AGENT_PER, AGENT_DLR, AGENT_PRIO);
		task_create(sensors_task,SENSORS_ID,SENSORS_PER,SENSORS_DLR,SENSORS_PRIO);
	}
	
}

// initialize track side of GUI
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

// update car movement on track on display
void update_scene() {
	// should be a for cycle for reach agent
	draw_track();
	draw_car();
	if(!disable_sensors)
		draw_sensors();
	//crash_check();
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

// used to reset track visuals at each timestamp 
void draw_track() {
	blit(track_bmp, scene_bmp, 0, 0, 0, 0, track_bmp->w, track_bmp->h);
}

// rotate car sprite and display inside the scene 
void draw_car() {
	int points[8];
	int i;
	struct ViewPoint vertices[4];
	
	pthread_mutex_lock(&mux_agent);
	
	find_rect_vertices(vertices, 4, rl_agent.car);
	points[0] = vertices[0].x;
	points[1] = vertices[0].y;
	points[2] = vertices[1].x;
	points[3] = vertices[1].y;
	points[4] = vertices[2].x;
	points[5] = vertices[2].y;
	points[6] = vertices[3].x;
	points[7] = vertices[3].y;
	polygon(scene_bmp, 4, points, makecol(51,153,255));

	pthread_mutex_unlock(&mux_agent);
}

// find pixel coordinates of car rectangle 
// and save them inside and array of type ViewPoint
// size is the dimension of the array
void find_rect_vertices(struct ViewPoint vertices[], int size, struct Car car) {
	float ca, sa;
	float x1, y1, x2, y2, x3, y3, x4, y4;
	//struct Car car;
	if(size != 4) {
		printf("ERROR: find_rect_vertices requires array with dimension = 4!\n");
		exit(1);
	}

	ca = cos(car.theta);
	sa = sin(car.theta);
	// p1 = (0, -WD/2) // check error -wd or wd
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

	lidar = sensors[0];
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);

	lidar = sensors[1];
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);

	lidar = sensors[2];		
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);

	pthread_mutex_unlock(&mux_sensors);
}

// checks if all cars have any collision with the track borders
// if there are collisions return 1, otherwise returns 0
void crash_check() {
	int found;
	int i, k; // used for for cycles
	int dead_agent;
	struct Car new_car;
	struct ViewPoint vertices[4];

	// initialize empty car to reset dead agent's car
	new_car.v = 0.0;
	new_car.x = pose_pool[pool_index][0];
	new_car.y = pose_pool[pool_index][1];
	new_car.theta = pose_pool[pool_index][2];

	dead_agent = 0;

	pthread_mutex_lock(&mux_agent);

	find_rect_vertices(vertices, 4, rl_agent.car);
	// use y=mx +b
	found = 0;
	for(k = 0; k < 4; k++) {
		found = check_color_px_in_line(
					vertices[(k+1)%4].x,
					vertices[(k+1)%4].y,
					vertices[k%4].x,
					vertices[k%4].y,
					0);
		if(found)
			dead_agent = 1;
	}

	if (dead_agent == 1) {
		rl_agent.alive = 0;
		//rl_agent.car.v = 0.0;
		rl_agent.car = new_car;
		//printf("dead agent %d\n", i);
		// at each restart the agent should change some parameters for the next simulation
	}

	pthread_mutex_unlock(&mux_agent);
}

// check if the car passed as argument has any collisions with track boundaries
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

// check if the points provided as argument
// are on pixels of the color passed to the function
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

// display on screen a bitmap with the values of Q Matrix 
// for each action that the agent can choose at each timestamp
// the values are scaled based on the greatest absolute value among all actions
void show_rl_graph() {
	int px_h, px_w;
	int white, orange, blue, green;
	int shift_y_axis = 100;
	int shift_x_axis = 100;
	// move graph 10px from left margin
	int x_offset = 10;
	// circle radius
	int r = 3;
	struct Agent agent;
	struct Actions_ID best_act, agent_act;
	float act_values[BUF_LEN];

	struct ViewPoint p1;
	int i, index;	// used for for cycle
	float scale_y, scale_x;
	// max value from Q matrix 
	float g_h;
	char debug[LEN];
	// it needs a mutex for buffer access
	// area where points are being drawn
	px_h = graph_bmp->h - shift_y_axis;
	px_w = graph_bmp->w - shift_x_axis - x_offset;

	white = makecol(255,255,255);
	orange = makecol(255,99,71);
	green = makecol(0,255,0);
	blue = makecol(100,149,237);

	clear_to_color(graph_bmp, 0);
	// draw axes
	line(graph_bmp, x_offset, px_h/2, px_w, px_h/2, white);	// x
	line(graph_bmp, x_offset, px_h, x_offset, 0, white);	// y

	pthread_mutex_lock(&mux_agent);
	agent = rl_agent;
	pthread_mutex_unlock(&mux_agent);

	best_act = ql_best_action(agent.state);
	// i should use buffer for Q matrix access
	assert(BUF_LEN == ql_get_nactions());
	// save actions row from Q matrix on state s
	pthread_mutex_lock(&mux_q_matrix);
	for(i = 0; i < BUF_LEN; i++) {
		act_values[i] = ql_get_Q(agent.state, i);
	}
	pthread_mutex_unlock(&mux_q_matrix);
	// get max elem of buffer
	for(i = 0; i < BUF_LEN; i++) {
		if(fabs(act_values[i]) > g_h)
			g_h = fabs(act_values[i]);
	}

	// find scale based on max value stored in buffer
	scale_y = (float)g_h/(px_h/2);
	//printf("scale_y: %f, g_h: %d\n", scale_y, g_h);
	// amount of pixel for 1 unit of buffer
	scale_x = px_w/(BUF_LEN+1); 

	//i = 1;
	for(i = 0; i < BUF_LEN; i++) {
		index = i;
		p1.y = px_h/2 - floor(act_values[index]/scale_y);
		//memset(debug, 0, sizeof debug);
		//sprintf(debug,"%d", graph_buff.y[index]);
		//textout_ex(graph_bmp, font, debug, 0, p1.y, white, -1);

		p1.x = (x_offset + scale_x + (scale_x * index));
		memset(debug, 0, sizeof debug);
		//sprintf(debug,"ep: %d", graph_buff.x[index]);
		sprintf(debug,"%d", (i*ACTIONS_STEP) - MAX_THETA);
		textout_ex(graph_bmp, font, debug, p1.x, px_h+5, white, -1);

		memset(debug, 0, sizeof debug);
		//sprintf(debug,"ep: %d", graph_buff.x[index]);
		sprintf(debug,"%.2f", act_values[index]);
		textout_ex(graph_bmp, font, debug, p1.x, px_h+25, white, -1);
		if(index == best_act.steer_act_id)
			circlefill(graph_bmp, p1.x, p1.y, r, green);
		else if (index == agent.a_id.steer_act_id)
			circlefill(graph_bmp, p1.x, p1.y, r, blue);
		else
			circlefill(graph_bmp, p1.x, p1.y, r, orange);
		line(graph_bmp, p1.x, px_h, p1.x, px_h-8, white);
		//line(graph_bmp, x_offset, p1.y, x_offset + 8, p1.y, white);
	}

	blit(graph_bmp, screen, 0, 0, 10, 20, graph_bmp->w, graph_bmp->h);
}

// handles the display thread
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
		//show_rl_graph();
		//show_gui_interaction_instructions();

		deadline_miss(GRAPHICS_ID);
		wait_for_activation(i);
	}

	return NULL;
}

// handles the agent thread
void* agent_task(void* arg) {
	int i, j, alive_flag, mode;
	//float progress = 0.0;
	struct Car new_car;

	i = get_task_index(arg);
	set_activation(i);

	// initialize empty car to reset dead agent's car
	//new_car.x = 0.0;
	//new_car.y = 0.0;
	//new_car.theta = deg_to_rad(INIT_THETA);
	mode = ql_get_rl_mode();

	do {	
		crash_check();

		alive_flag = 0;
		// need to update agent when crash occured
		//for(i = 0; i < MAX_AGENTS; i++) {
			if (mode == TRAINING)
				conv_delta += learn_to_drive();
			else if ( mode == INFERENCE)
				printf("Should do inference here\n");
			
			// reset dead agent
			pthread_mutex_lock(&mux_agent);
			if (rl_agent.alive == 1) {
				alive_flag = 1;
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

				rl_agent.alive = 1;
				new_car.v = TRAIN_VEL;
				new_car.x = pose_pool[pool_index][0];
				new_car.y = pose_pool[pool_index][1];
				new_car.theta = pose_pool[pool_index][2];
				rl_agent.car = new_car;
				//printf("Reset agent %d!\n", j);
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

// handles the learning thread
void* learning_task(void* arg) {
	int i, j, alive_flag, index;
	//float progress = 0.0;
	struct Car new_car;
	char scan;
	// agent data
	int s;
	struct Actions_ID a;
	// lidars distance
	int d_l, d_r, d_f; 
	int train_only_steering = ql_get_train_mode();

	i = get_task_index(arg);
	set_activation(i);

	pthread_mutex_lock(&mux_sensors);
	d_l = sensors[0].d;
	d_f = sensors[1].d;
	d_r = sensors[2].d;
	pthread_mutex_unlock(&mux_sensors);

	s = decode_lidar_to_state(d_l, d_r, d_f);

	pthread_mutex_lock(&mux_q_matrix);
	a = ql_egreedy_policy(s);
	pthread_mutex_unlock(&mux_q_matrix);

	// correct only in single thread. should have mux otherwise
	rl_agent.state = s;
	rl_agent.a_id = a;
	
	do {
		crash_check();
		// interrupt training after driving on whole track
		// for certain amount of meters
		if (rl_agent.distance > 1200)
			rl_agent.alive = 0;

		alive_flag = 0;
		// need to update agent when crash occured
		//for(i = 0; i < MAX_AGENTS; i++) {
		single_thread_learning();
	
		index = episode-1;
		statistics[index] = rl_agent.ep_stats;
		// reset dead agent
		//pthread_mutex_lock(&mux_agent);
		if (rl_agent.alive == 1) {
			alive_flag = 1;
		}

		// no agent alive, start new episode
		if(alive_flag == 0) {
			episode++;
			
			//if(((episode%200) == 0) && (episode > 100))
			//	ql_reduce_expl();

			if((episode%100) == 0) {
				// q-learn
				ql_reduce_expl();
				// change initial position to improve training
				pool_index = (pool_index + 1) % POOL_DIM;
				printf("Changing initial pose!\n");
			}

			rl_agent.alive = 1;
			rl_agent.distance = 0.0;
			rwd_distance_counter = 0.0;
			
			// train_mode=1 -> train only steering
			if (train_only_steering)
				new_car.v = MAX_V_ALLOWED;
			else
				new_car.v = TRAIN_VEL;
			new_car.x = pose_pool[pool_index][0];
			new_car.y = pose_pool[pool_index][1];
			new_car.theta = pose_pool[pool_index][2];
			
			rl_agent.car = new_car;

			// reset episode stats
			struct EpisodeStats stats;
			stats.steps = 0;
			stats.total_reward = 0.0;
			stats.total_td_error = 0.0;

			rl_agent.ep_stats = stats;
			//printf("Reset agent %d!\n", j);

			// refresh sensors with initial pose
			refresh_sensors();
			// update initial state for learning
			d_l = sensors[0].d;
			d_f = sensors[1].d;
			d_r = sensors[2].d;
			s = decode_lidar_to_state(d_l, d_r, d_f);
			a = ql_egreedy_policy(s);
			rl_agent.state = s;
			rl_agent.a_id = a;
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
		//show_rl_graph();
		//show_Q_matrix();
		show_gui_interaction_instructions();

		// command interpreter task
		scan = interpreter();
		if (scan == KEY_ESC)
			end = 1;
		
		deadline_miss(LEARNING_ID);
		wait_for_activation(i);
	} while (!end);
	// save Q learning matrix on file for further use
	save_Q_matrix_to_file();
	save_Q_vel_matrix_to_file();
	save_Tr_matrix_to_file();
	// save episodes statistics on file for further analysis 
	save_episodes_stats_to_file();

	return NULL;
}

// handles the thread that updates lidars sensors at each timestamp 
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

// handles the thread of commands  provided by the user via keyboard
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

// defines the commands available via keyboard
char interpreter() {
	//int i;
	int mode;
	float delta;
	char scan;
	struct Controls act;

	if (keypressed()) {
		pthread_mutex_lock(&mux_agent);
		act = rl_agent.action;
		pthread_mutex_unlock(&mux_agent);
		delta = rad_to_deg(act.delta);
		scan = get_scancode();
		switch (scan) {
			case KEY_A:
				act.a += (0.1 * G);
				if (act.a > MAX_A)
					act.a = MAX_A;
				//car.v += 1;
				//if (car.v > MAX_V)
				//	car.v = MAX_V;
				break;
			case KEY_Q:
				act.a -= (0.1 * G);
				if (act.a < MIN_A)
					act.a = MIN_A;
				//car.v -= 1;
				break;
			case KEY_UP:
				MAX_V_ALLOWED += 1.0;
				MAX_V_ALLOWED = (MAX_V_ALLOWED >= MAX_V) ? MAX_V : MAX_V_ALLOWED;
				break;
			case KEY_DOWN:
				MAX_V_ALLOWED -= 1.0;
				MAX_V_ALLOWED = (MAX_V_ALLOWED <= 0.0) ? 0.0 : MAX_V_ALLOWED;
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
				mode = ql_get_rl_mode();
				pthread_mutex_lock(&mux_q_matrix);
				if( mode == TRAINING) {
					ql_set_rl_mode(INFERENCE);
					init_qlearn_inference_mode();
					printf("Changing rl mode: TRAINING -> INFERENCE \n");
				}else if( mode == INFERENCE) {
					ql_set_rl_mode(TRAINING);
					init_qlearn_training_mode();
					printf("Changing rl mode: INFERENCE -> TRAINING \n");
				}else {
					printf("ERROR: wrong mode! Should be %d or %d\n", TRAINING, INFERENCE);
					exit(1);
				}
				pthread_mutex_unlock(&mux_q_matrix);
				break;
			case KEY_L:
				printf("Loading Q matrix from file %s\n", Q_MAT_FILE_NAME);
				//pthread_mutex_lock(&mux_q_matrix);
				// Q Learning
				read_Q_matrix_from_file();
				read_Q_vel_matrix_from_file();
				read_Tr_matrix_from_file();
				//pthread_mutex_unlock(&mux_q_matrix);
				break;
			case KEY_S:
				printf("Saving Q matrix to file %s\n", Q_MAT_FILE_NAME);
				//pthread_mutex_lock(&mux_q_matrix);
				// Q Learning
				save_Q_matrix_to_file();
				save_Q_vel_matrix_to_file();
				save_Tr_matrix_to_file();
				// save episodes statistics on file for further analysis 
				save_episodes_stats_to_file();
				//pthread_mutex_unlock(&mux_q_matrix);
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
		pthread_mutex_lock(&mux_agent);
		rl_agent.action = act;
		//printf("a: %f, delta: %f \n", rl_agent.action.a, rl_agent.action.delta);
		pthread_mutex_unlock(&mux_agent);
	}

	return scan;
}

// conversion from degrees to radians
float deg_to_rad(float deg_angle) {
	return deg_angle*M_PI/180;
}

// conversion from radians to degrees
float rad_to_deg(float rad_angle) {
	return rad_angle*180/M_PI;
}

// converts from degrees to fixed Allegro angles unit
fixed deg_to_fixed(float deg) {
	float angle = 0.0;

	angle = 128 - deg*(float)256/360;
	//printf("angle is %f\n", angle);
	return ftofix(angle);
}

// reads input from keyboards using Allegro API
char get_scancode() {
	if (keypressed())
		return readkey() >> 8;
	else
		return 0;
}

// initialize agent structure with the initial conditions on track
void init_agent() {
	struct Car vehicle;
	int i;
	int train_only_steering = ql_get_train_mode();
	
	printf("*** Initializing agent\n");

	if (train_only_steering)
		printf("Training mode is: only STEERING \n");
	else
		printf("Training mode is: STEERING + ACCELERATION \n");
	
	vehicle.x = pose_pool[pool_index][0];
	vehicle.y = pose_pool[pool_index][1];
	vehicle.theta = pose_pool[pool_index][2];
	if (train_only_steering)
		vehicle.v = MAX_V_ALLOWED;
	else
		vehicle.v = TRAIN_VEL;
	//vehicle.a = 0.0;
	
	pthread_mutex_lock(&mux_agent);
	rl_agent.alive = 1;
	rl_agent.distance = 0.0;
	rl_agent.car = vehicle;
	rl_agent.error = 0.0;
	rl_agent.state = 0; // should be checked if it is ok to use a wrong state

	// reset episode stats
	struct EpisodeStats stats;
	stats.steps = 0;
	stats.total_reward = 0.0;
	stats.total_td_error = 0.0;
	rl_agent.ep_stats = stats;
	pthread_mutex_unlock(&mux_agent);
}

// defines an array of different initial poses on track 
void init_pool_poses() {
	pose_pool[0][0] = 0.0;
	pose_pool[0][1] = 0.0;
	pose_pool[0][2] = deg_to_rad(90.0);

	pose_pool[1][0] = -50.0;
	pose_pool[1][1] = 0.0;
	pose_pool[1][2] = deg_to_rad(90.0);

	pose_pool[2][0] = -73.0;
	pose_pool[2][1] = 52.0;
	pose_pool[2][2] = deg_to_rad(90.0);

	pose_pool[3][0] = -60.0;
	pose_pool[3][1] = 70.0;
	pose_pool[3][2] = deg_to_rad(270.0);

	pose_pool[4][0] = -25.0;
	pose_pool[4][1] = 55.0;
	pose_pool[4][2] = deg_to_rad(270.0);

	pose_pool[5][0] = 20.0;
	pose_pool[5][1] = 20.0;
	pose_pool[5][2] = deg_to_rad(270.0);

	pose_pool[6][0] = -45.0;
	pose_pool[6][1] = 60.0;
	pose_pool[6][2] = deg_to_rad(90.0);

	pose_pool[7][0] = -5.0;
	pose_pool[7][1] = 85.0;
	pose_pool[7][2] = deg_to_rad(0.0);

	pose_pool[8][0] = -15.0;
	pose_pool[8][1] = 45.0;
	pose_pool[8][2] = deg_to_rad(0.0);

	pose_pool[9][0] = 19.0;
	pose_pool[9][1] = 75.0;
	pose_pool[9][2] = deg_to_rad(270.0);

	pose_pool[10][0] = 10.0;
	pose_pool[10][1] = 60.0;
	pose_pool[10][2] = deg_to_rad(180.0);

	pose_pool[11][0] = -58.0;
	pose_pool[11][1] = 40.0;
	pose_pool[11][2] = deg_to_rad(0.0);

	pose_pool[12][0] = -75.0;
	pose_pool[12][1] = 82.0;
	pose_pool[12][2] = deg_to_rad(15.0);


}

// update a bitmap with the deadline misses of the various tasks
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

void show_gui_interaction_instructions() {
	char buff[100];
	int x, y;
	int white, yellow, blue;
	int x_offset;

	white = makecol(255,255,255);
	blue = makecol(100,149,237);
	yellow = makecol(255,255,0);

	x = 10;
	y = 10;
	x_offset = 50;

	clear_to_color(instructions_bmp, 0);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"Keyboard shortcuts");
	textout_ex(instructions_bmp, font, buff, x, y, blue, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key A ");
	textout_ex(instructions_bmp, font, buff, x, y + 20, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"increase acceleration input");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 20, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key Q ");
	textout_ex(instructions_bmp, font, buff, x, y + 40, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"decrease acceleration input");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 40, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"ARROW UP/DOWN" );
	textout_ex(instructions_bmp, font, buff, x, y + 60, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"change car's max velocity allowed");
	textout_ex(instructions_bmp, font, buff, x + 115, y + 60, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key M ");
	textout_ex(instructions_bmp, font, buff, x, y + 80, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"switch between INFERENCE and TRAINING mode");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 80, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key L ");
	textout_ex(instructions_bmp, font, buff, x, y + 100, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"load Q matrix from file");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 100, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key S ");
	textout_ex(instructions_bmp, font, buff, x, y + 120, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"save Q matrix to file");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 120, white, -1);

	memset(buff, 0, sizeof buff);
	sprintf(buff,"Key D ");
	textout_ex(instructions_bmp, font, buff, x, y + 140, yellow, -1);
	memset(buff, 0, sizeof buff);
	sprintf(buff,"enable/disable lidar beams drawing");
	textout_ex(instructions_bmp, font, buff, x + x_offset, y + 140, white, -1);

	rect(instructions_bmp, 5, 5, x + 400, y + 160, blue);
	blit(instructions_bmp, screen, 0, 0, 10, 10, instructions_bmp->w, instructions_bmp->h);
}

// visualize current state "S" and Q values of each action on state "S" 
void show_Q_matrix() {
	struct Agent agent;
	float action_val;
	int x, y, i;
	int MAX_FLOAT_SIZE =6;
	int DEBUG_SIZE = 200;
	int N_ACTIONS_STEER, N_ACTIONS_VEL;
	struct Actions_ID best_act, agent_act;
	char debug[DEBUG_SIZE];
	char action_buff[12];
	int white, green, yellow, blue;

	white = makecol(255,255,255);
	green = makecol(0,255,0);
	yellow = makecol(255,255,0);
	blue = makecol(100,149,237);

	// find number of actions
	N_ACTIONS_STEER = (int)((MAX_THETA * 2)/ACTIONS_STEP) + 1;
	N_ACTIONS_VEL = (int)((MAX_A/G * 2)/ACC_STEP) + 1;
	// check for buffer overflow
	assert((N_ACTIONS_STEER * MAX_FLOAT_SIZE) < (DEBUG_SIZE - 10));
	assert((N_ACTIONS_VEL * MAX_FLOAT_SIZE) < (DEBUG_SIZE - 10));
	// (x,y) where start drawing 
	x = 0;
	y = instructions_bmp->h - 50;

	// get agent related info
	pthread_mutex_lock(&mux_agent);
	agent = rl_agent;
	pthread_mutex_unlock(&mux_agent);

	// get Q_learn data
	pthread_mutex_lock(&mux_q_matrix);
	best_act = ql_best_action(agent.state);
	pthread_mutex_unlock(&mux_q_matrix);

	clear_to_color(graph_bmp, 0);
	// write data on GUI
	memset(debug, 0, sizeof debug);
	sprintf(debug,"state: %d", agent.state);
	textout_ex(graph_bmp, font, debug, x, (y + 10), white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"best IDs: steer = %d, vel = %d", best_act.steer_act_id, best_act.vel_act_id);
	textout_ex(graph_bmp, font, debug, x, (y + 20), green, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"used IDs: steer = %d, vel = %d", agent.a_id.steer_act_id, agent.a_id.vel_act_id);
	textout_ex(graph_bmp, font, debug, x, (y + 30), blue, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"Q[s] actions row: ");
	textout_ex(graph_bmp, font, debug, x, (y + 40), white, -1);
	memset(debug, 0, sizeof debug);

	pthread_mutex_lock(&mux_q_matrix);
	for(i = 0; i < N_ACTIONS_STEER; i++) {
		action_val = ql_get_Q(agent.state, i);
		memset(action_buff, 0, sizeof action_buff);
		sprintf(action_buff, "%.2f | ", action_val);

		strcat(debug, action_buff);
	}
	pthread_mutex_unlock(&mux_q_matrix);

	textout_ex(graph_bmp, font, debug, x, (y + 50), white, -1);

	blit(graph_bmp, instructions_bmp, 0, 0, 10, 10, graph_bmp->w, graph_bmp->h);
}

// update the distance measured by lidars at each timestamp
void refresh_sensors() {
	int i;
	struct Lidar lidar;
	struct Car car;
	struct Lidar tmp_sensors[3];
	float x_p, y_p;

	pthread_mutex_lock(&mux_agent);
	car = rl_agent.car;
	// left lidar
	//lidar.alpha = car.theta + deg_to_rad(45.0);
	lidar.alpha = car.theta + deg_to_rad(LIDAR_ANGLE_POS);
	x_p = car.x + L*cos(car.theta);	// global frame in m from bottom left
	y_p = car.y + L*sin(car.theta);
	//printf("x_p: %f, y_p: %f\n", x_p, y_p);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	tmp_sensors[0] = lidar;
	// front
	lidar.alpha = car.theta + deg_to_rad(0.0);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	tmp_sensors[1] = lidar;
	// right
	//lidar.alpha = car.theta + deg_to_rad(-45.0);
	lidar.alpha = car.theta + deg_to_rad(LIDAR_ANGLE_NEG);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	tmp_sensors[2] = lidar;
	pthread_mutex_unlock(&mux_agent);

	// save the updated sensors measurements to global variable
	pthread_mutex_lock(&mux_sensors);
	sensors[0] = tmp_sensors[0];
	sensors[1] = tmp_sensors[1];
	sensors[2] = tmp_sensors[2];
	pthread_mutex_unlock(&mux_sensors);
}

// updates the lidar distance of the car passed as argument and 
// return it to the array passed as argument
void get_updated_lidars_distance(struct Car car, struct Lidar car_sensors[]) {
	struct Lidar lidar;
	float x_p, y_p;
	
	if (car_sensors == NULL) {
		printf("ERROR: Invalid array! \n");
		exit(1);
	}

	// left lidar
	//lidar.alpha = car.theta + deg_to_rad(45.0);
	lidar.alpha = car.theta + deg_to_rad(LIDAR_ANGLE_POS);
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
	//lidar.alpha = car.theta + deg_to_rad(-45.0);
	lidar.alpha = car.theta + deg_to_rad(LIDAR_ANGLE_NEG);
	lidar.x = BTM_X + (x_p/SCALE);
	lidar.y = SIM_Y - (BTM_Y + y_p/SCALE);
	lidar.d = read_sensor(
					lidar.x, 
					lidar.y, 
					lidar.alpha);
	car_sensors[2] = lidar;
}

// consider the scale 1m = 1px
// find the distance measured by the lidar until the first 
// balck pixel is reached
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

// update the mathematical model of the car (in m) at each timestamp
// returns the updated state of car passed as argument
struct Car update_car_model(struct Agent agent) {
	struct Car old_state, new_state;
	struct Controls act;
	float vx, vy, dt;
	float omega;
	// /float friction;
	float norm_theta, theta_deg;
	float max_v;

	(MAX_V > MAX_V_ALLOWED) ? (max_v = MAX_V_ALLOWED) : (max_v = MAX_V);
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
	
	//theta_deg = rad_to_deg(old_state.theta);
	//norm_theta = theta_deg - (floor(theta_deg/360.0) * 360.0);
	//norm_theta = deg_to_rad(norm_theta);
	//norm_theta = atan2(sin(old_state.theta), cos(old_state.theta));
	
	//theta_deg = rad_to_deg(norm_theta) + 180;
	//norm_theta = deg_to_rad(theta_deg);

	new_state.v = old_state.v + dt * (act.a); // - friction);
	// clamp velocity to max velocity feasible
	if (new_state.v > max_v)
		new_state.v = max_v;
	// don't allow reverse
	if (new_state.v < 0.0)
		new_state.v = 0.0;

	vx = old_state.v*cos(old_state.theta);
	vy = old_state.v*sin(old_state.theta);

	new_state.x = old_state.x + (vx * dt);
	new_state.y = old_state.y + (vy * dt);

	omega = old_state.v * tan(act.delta) / WHEELBASE;
	norm_theta = old_state.theta + (omega * dt);
	// normalize theta to 0 - 360
	norm_theta = atan2(-sin(norm_theta), -cos(norm_theta)) + M_PI;
	//new_state.theta = norm_theta + (omega * dt);
	new_state.theta = norm_theta;	
	//printf("y: %f, theta: %f\n", new_state.y, rad_to_deg(norm_theta));
	return new_state;
	//}
	//pthread_mutex_unlock(&mux_agent);
}

// update a bitmap with debug information about car and agent
void write_debug() {
	int white, yellow;
	int x, y;
	int i;
	int index;
	int mode;
	struct Agent agent;
	
	x = 0;
	white = makecol(255,255,255);
	yellow = makecol(255,255,0);
	clear_to_color(debug_bmp, 0);

	pthread_mutex_lock(&mux_agent);
	agent = rl_agent;
	pthread_mutex_unlock(&mux_agent);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"Car Telemetry");
	textout_ex(debug_bmp, font, debug, x, 10, yellow, -1);

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
	sprintf(debug,"Controls Input");
	textout_ex(debug_bmp, font, debug, x, 60, yellow, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.a: %f m/s^2", agent.action.a);
	textout_ex(debug_bmp, font, debug, x, 70, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"act.steer: %f rad", agent.action.delta);
	textout_ex(debug_bmp, font, debug, x, 80, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"max steer: %f rad", deg_to_rad(MAX_THETA));
	textout_ex(debug_bmp, font, debug, x, 90, white, -1);
	
	memset(debug, 0, sizeof debug);
	sprintf(debug,"Reinforcement Learning Data");
	textout_ex(debug_bmp, font, debug, x, 100, yellow, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"gamma: %f", ql_get_discount_factor());
	textout_ex(debug_bmp, font, debug, x, 110, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"alpha: %f", ql_get_learning_rate());
	textout_ex(debug_bmp, font, debug, x, 120, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"epsilon: %f", ql_get_epsilon());
	textout_ex(debug_bmp, font, debug, x, 130, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"episode: %d", episode);
	textout_ex(debug_bmp, font, debug, x, 140, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"driven distance: %.3f m", agent.distance);
	textout_ex(debug_bmp, font, debug, x, 150, white, -1);

	memset(debug, 0, sizeof debug);
	if (ql_get_rl_mode() == INFERENCE) {
		sprintf(debug,"RL mode: inference");
	}
	else if (ql_get_rl_mode() == TRAINING) {
		sprintf(debug,"RL mode: training");
	}
	textout_ex(debug_bmp, font, debug, x, 160, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"max v allowed: %f m/s", MAX_V_ALLOWED);
	textout_ex(debug_bmp, font, debug, x, 170, white, -1);

	index = episode - 1;
	memset(debug, 0, sizeof debug);
	//printf(" td_error: %f \n \n", statistics[index].total_td_error);
	sprintf(debug,"ep. TD error: %.3f", statistics[index].total_td_error);
	textout_ex(debug_bmp, font, debug, x, 190, white, -1);

	memset(debug, 0, sizeof debug);
	sprintf(debug,"ep. reward: %.3f", statistics[index].total_reward);
	textout_ex(debug_bmp, font, debug, x, 200, white, -1);
	//pthread_mutex_unlock(&mux_agent);

	x = SIM_X;
	y = (WIN_Y - SIM_Y + 50);
	blit(debug_bmp, screen, 0, 0, x, y, debug_bmp->w, debug_bmp->h);
}

// encode action id to steering input
float action_to_steering(int action_k) {
	float x, y, z;
	int n_actions = ql_get_nactions() -1;

	if ((action_k < 0) || (action_k > n_actions)) {
		printf("ERROR: AGENT steering action should be an index from 0 to %d\n", n_actions);
		exit(1);
	}

	x = (float)action_k/n_actions;
	y = 2*x -1;
	z = MAX_THETA*y*M_PI/180;

	return z;
}

// encode action id to acceleration input
float action_to_acc(int action_a) {
	float x, y, z;
	int n_actions_vel = ql_get_nactions_vel() - 1;

	if ((action_a < 0) || (action_a > n_actions_vel)) {
		printf("ERROR: AGENT acceleration action should be an index from 0 to %d\n", n_actions_vel);
		exit(1);
	}

	x = (float)action_a/n_actions_vel; // [0;1]
	y = 2*x - 1; // [-1;1]
	z = MAX_A*y; //m/s^2 * scale

	return z;
}
// combine left and right lidar by getting d_left - d_right
// without Kohonen network
// decode lidars distances to a Reinforcement Learning State
// used inside the Q-Learing algorithm
int decode_lidar_to_state(int d_left, int d_right, int d_front) {
	float delta, front, y;
	int s1, s2, s;

	delta = (float)(d_left - d_right)/(SMAX+1);
	front = (float)d_front/(SMAX+1);

	y = (delta+1)/2;
	s1 = floor(MAX_STATES_LIDAR * y); // to be checked
	s2 = floor(MAX_STATES_LIDAR * front);
	s = (s2 * MAX_STATES_LIDAR) +s1;

	//printf("s: %d, s1: %d, s2: %d, y: %f, front: %f\n", s, s1, s2, y, front);
	return s;
}

// returns the reward from environment based on current agent inputs
// and lidars sensors measurements
float get_reward(struct Agent agent, int d_left, int d_front, int d_right) {
	float r = 0;
	//int d_left, d_front, d_right;
	//struct Agent agent;
	float track_pos, track_center; // distance between car's (x,y) and centre of track
	float x, y, alpha;
	//float vx, vy, ca, sa;
	int d_l, d_r;
	float dist;
	int train_only_steering = ql_get_train_mode();
	float old_steer_delta, new_steer_delta;

	track_pos = 0.0;
	dist = 0.0;
	
	// compute left distance perpendicular to car's (x,y) 
	alpha = agent.car.theta + deg_to_rad(90.0);
	x = BTM_X + (agent.car.x/SCALE);
	y = SIM_Y - (BTM_Y + agent.car.y/SCALE);
	d_l = read_sensor(x, y, alpha);
	// compute right distance
	alpha = agent.car.theta + deg_to_rad(-90.0);
	x = BTM_X + (agent.car.x/SCALE);
	y = SIM_Y - (BTM_Y + agent.car.y/SCALE);
	d_r = read_sensor(x, y, alpha);
	
	track_center = (d_l + d_r)/2;
	track_pos = fabs(d_l - d_r);
	//printf("d_l: %d, d_r: %d \n", d_l, d_r);
	// compute distance from track centre
	/*
	if (d_r > d_l)
		track_pos = d_r - (d_r + d_l)/2;
	else 
		track_pos = d_l - (d_r + d_l)/2;
	*/
	//track_pos = fabs(d_left - d_right)/100;
	//printf("d_l: %d, d_r: %d, d_f: %d\n", d_l, d_r, d_front);
	// compute vx and vy
	//ca = cos(agent.car.theta);
	//sa = sin(agent.car.theta);
	//vx = agent.car.v * ca;
	//vy = agent.car.v * sa;
	
	// if car has crashed return bad reward
	if (is_car_offtrack(agent.car)) {
		return RWD_CRASH;
	} else {
		// TO-DO
		// 1) add reward based of distance from track centre
		// 2) change distance reward based on counter that gives positive reward every x meters on track and not relative to total distance of car
		// 3) add saturation of velocity instead of behavior with deccelerion if > max_v_allowed
		//r = ALPHA_REWARD * (vx + vy - agent.car.v*fabs(track_pos));
		// reward for staying alive
		//return RWD_ALIVE;
		r += RWD_ALIVE;
		// increase reward when front distance increases
		//r += (d_front/100);
		// decrese reward when car is off-center
		//r -= track_pos;
		//dist = (float)agent.distance/20.0;

		//dist = floor(agent.distance/20.0);
		//r += dist * RWD_DISTANCE;
		if (agent.distance >= (rwd_distance_counter + DIST_THRESHOLD_RWD)) {
			r += RWD_DISTANCE;
			//printf("Adding reward %.2f for distance on track \n", DIST_THRESHOLD_RWD);
			rwd_distance_counter += DIST_THRESHOLD_RWD;
		}

		// compute reward in relation with distance from track center
		if ((track_pos < 10) && (d_front > 30)) {
			r += RWD_ON_CENTRE;
			//printf("near the center of track: %.3f \n", track_pos);
		}
		// compute reward in relation with the variation of delta steering
		old_steer_delta = rad_to_deg(rwd_previous_delta);
		new_steer_delta = rad_to_deg(agent.action.delta);
		float variance = fabs((old_steer_delta - new_steer_delta)/MAX_THETA);
		//printf("variance: %f, reward: %f\n", variance, (variance*RWD_STEER_DELTA_GAP));
		//printf("old steer: %f, new steer: %f \n", old_steer_delta, new_steer_delta);
		//r += RWD_STEER_DELTA_GAP * variance;
		//printf("r: %f, dist: %f\n", r, dist);

		// don't consider the acceleration related reward change
		if (train_only_steering)
			return r;
		// -------- acceleration ---------
		//if ((agent.action.a > 0) && (agent.car.v >= MAX_V_ALLOWED)) {
		//	r = RWD_BAD_ACC;
		//}
		//else if ((agent.action.a <= 0) && (agent.car.v == 0)) {
		//	r = RWD_BAD_ACC;
		//}
		//else if (agent.car.v > (MAX_V_ALLOWED+1.0)) {
		//	r = RWD_BAD_ACC;
		//}
		//else if (agent.car.v >= MAX_V_ALLOWED){
		//	r+= RWD_CORRECT_ACC*(agent.car.v/MAX_V_ALLOWED);
		//}
		//else if (agent.car.v < MAX_V_ALLOWED){
		//	r+= (agent.car.v/MAX_V_ALLOWED);
		//}
		
		//printf("r: %f\n", r);
		return r;
		// check if it should be deleted
		// reward for correct turn
		if (d_right > d_left) {
			if (agent.action.delta < 0)
				r += RWD_CORRECT_TURN * fabs(agent.action.delta);
			else
				r += RWD_WRONG_TURN;
		}
		if (d_right < d_left) {
			if (agent.action.delta > 0)
				r += RWD_CORRECT_TURN * fabs(agent.action.delta);
			else
				r += RWD_WRONG_TURN;
		}
		
		// reward for no turn on straight
		if ((d_front > d_left) && (d_front > d_right)) {
			if (fabs(agent.action.delta) < deg_to_rad(5))
				r += RWD_STRAIGHT/(1 - fabs(agent.action.delta));
			else
				r += RWD_TURN_STRAIGHT/(1 - fabs(agent.action.delta));
		}
		// reward for keeping the car near the center
		// off centre decreses reward
		//r -= track_pos;
		//r += RWD_OFF_CENTRE * fabs(track_pos);
	}
	//printf("r: %f\n", r);
	//printf("r: %f, vx: %f, vy: %f, track_pos: %f\n", r, vx, vy, agent.car.v*fabs(track_pos));

	return r;
}

// multi thread learning algorithm
float learn_to_drive() {
	struct Actions_ID a;
	int s, s_new;
	float max_err, err;
	float r;
	int i, curr_s;
	struct Agent agent;
	struct Lidar car_sensors[3];
	int d_l, d_f, d_r; // lidar distances
	int act;

	max_err = 0.0;
	err = 0.0;

	pthread_mutex_lock(&mux_sensors);
	d_l = sensors[0].d;
	d_f = sensors[1].d;
	d_r = sensors[2].d;
	s = decode_lidar_to_state(d_l, d_r, d_f);
	a = ql_egreedy_policy(s);
	//printf("action for agent %d is steer: %d, vel: %d\n", i, a.steer_act_id, a[i]..vel_act_id);
	pthread_mutex_unlock(&mux_sensors);

	pthread_mutex_lock(&mux_agent);
	for(i = 0; i < MAX_AGENTS; i++) {
		agent = rl_agent;
		agent.action.delta = action_to_steering(a.steer_act_id);
		agent.action.a = action_to_acc(a.vel_act_id);
		agent.car = update_car_model(agent);
		

		get_updated_lidars_distance(agent.car, car_sensors);
		d_l = car_sensors[0].d;
		d_f = car_sensors[1].d;
		d_r = car_sensors[2].d;

		s_new = decode_lidar_to_state(d_l, d_r, d_f);
		r = get_reward(agent, d_l, d_f, d_r);
		// update max reward for debug
		if (r > max_reward)
			max_reward = r;
		//printf(" reward: %f, action %d \n", max_reward, a);
		//err = ql_updateQ(s, a, r, s_new);
		err = ql_updateQ(s, a, r, s_new);
		// update agent state
		agent.state = s_new;
		rl_agent = agent;
		// update max error
		// needs better handling of error
		if (err > max_err)
			max_err = err;
	}
	pthread_mutex_unlock(&mux_agent);

	// error handling is wrong !!  needs to be changed
	return max_err;
}

// initialize hyper parameters used by the Q-Learning algorithm
void init_qlearn_training_mode() {
	int n_states, n_actions_steer, n_actions_vel;

	printf("*** Initializing epsilon-greedy params: \n");
	n_states = MAX_STATES_LIDAR*MAX_STATES_LIDAR;
	//n_actions = (MAX_THETA * 2) - 1;
	n_actions_steer = (int)((MAX_THETA * 2)/ACTIONS_STEP) + 1;
	n_actions_vel = (int)((MAX_A/G * 2)/ACC_STEP) + 1;
	printf("n_states: %d, n_actions steer: %d, n_actions_velocity: %d\n",n_states, n_actions_steer, n_actions_vel);
	
	pthread_mutex_lock(&mux_q_matrix);
	ql_init(n_states, n_actions_steer, n_actions_vel);
	// modify specific params by calling related function
	ql_set_learning_rate(0.6);
	ql_set_discount_factor(0.95);
	ql_set_expl_factor(0.4);
	ql_set_expl_range(0.4, 0.1);
	ql_set_expl_decay(0.99);
	pthread_mutex_unlock(&mux_q_matrix);
	printf("---------\n");
}

void init_qlearn_inference_mode() {
	int n_states, n_actions_steer, n_actions_vel;

	printf("*** Initializing epsilon-greedy params: \n");
	n_states = MAX_STATES_LIDAR*MAX_STATES_LIDAR;
	//n_actions = (MAX_THETA * 2) - 1;
	n_actions_steer = (int)((MAX_THETA * 2)/ACTIONS_STEP) + 1;
	n_actions_vel = (int)((MAX_A/G * 2)/ACC_STEP) + 1;
	printf("n_states: %d, n_actions steer: %d, n_actions_velocity: %d\n",n_states, n_actions_steer, n_actions_vel);
	
	pthread_mutex_lock(&mux_q_matrix);
	ql_init(n_states, n_actions_steer, n_actions_vel);
	// modify specific params by calling related function
	ql_set_learning_rate(0.6);
	ql_set_discount_factor(0.95);
	ql_set_expl_factor(0.0);
	ql_set_expl_range(0.0, 0.0);
	ql_set_expl_decay(0.99);
	pthread_mutex_unlock(&mux_q_matrix);
	printf("---------\n");
}

void save_episodes_stats_to_file() {
	FILE *fp;
	int i;
	
	fp = fopen(EPISODES_STATS_FILE_NAME, "w");
	if (fp == NULL) {
		printf("ERROR: could not open file to write episodes statistics array\n");
		exit(1);
	}
	// save the number of states and actions
	fprintf(fp, "%d\n", episode);
	// save values of Q Matrix
	for(i = 0; i < episode; i++) {
		fprintf(fp, "%d %.2f %.2f", statistics[i].steps, statistics[i].total_td_error, statistics[i].total_reward);
		fprintf(fp, "\n");
	}
	
	fclose(fp);
	printf("Episodes statistics array saved on file %s!\n", EPISODES_STATS_FILE_NAME);
}

// save Q Matrix to file
void save_Q_matrix_to_file() {
	FILE *fp;
	int n_states, n_actions;
	int i, j;

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	float Q_tmp[n_states][n_actions];
	
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			Q_tmp[i][j] = ql_get_Q(i, j);
		}
	}
	pthread_mutex_unlock(&mux_q_matrix);

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

void save_Q_vel_matrix_to_file() {
	FILE *fp;
	int n_states, n_actions;
	int i, j;

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions_vel();
	float Q_tmp[n_states][n_actions];
	
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			Q_tmp[i][j] = ql_get_Q_vel(i, j);
		}
	}
	pthread_mutex_unlock(&mux_q_matrix);

	fp = fopen(Q_VEL_MAT_FILE_NAME, "w");
	if (fp == NULL) {
		printf("ERROR: could not open file to write Q_vel matrix\n");
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
	printf("Q_vel matrix saved on file %s!\n", Q_VEL_MAT_FILE_NAME);
}

// save to filer T_r Matrix used by the Q(lambda) algorithm
void save_Tr_matrix_to_file() {
	FILE *fp;
	int n_states, n_actions;
	int i, j;

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	float Tr_tmp[n_states][n_actions];
	
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			Tr_tmp[i][j] = ql_get_Tr(i, j);
		}
	}
	pthread_mutex_unlock(&mux_q_matrix);

	fp = fopen(Tr_MAT_FILE_NAME, "w");
	if (fp == NULL) {
		printf("ERROR: could not open file to write T_r matrix\n");
		exit(1);
	}
	// save the number of states and actions
	fprintf(fp, "%d %d\n", n_states, n_actions);
	// save values of Q Matrix
	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			fprintf(fp, "%.2f ", Tr_tmp[i][j]);
		}
		fprintf(fp, "\n");
	}
	
	fclose(fp);
	printf("T_r matrix saved on file %s!\n", Tr_MAT_FILE_NAME);
}

// restore from file the values of the Q Matrix
void read_Q_matrix_from_file() {

	FILE *fp;
	int dim_states_f, dim_actions_f, n_states, n_actions;
	char buf[50];
	char q_buff[1024];
	int size;
	char *ptr;
	float val;
	int i, j; // indexes to use for matrix

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	pthread_mutex_unlock(&mux_q_matrix);

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
	pthread_mutex_lock(&mux_q_matrix);
	while (fgets(q_buff, size, fp) != NULL) {
		ptr = q_buff;
		j = 0;
		
		while ((val = strtof(ptr, &ptr))) {
			ql_set_Q_matrix(i, j, val);
			j++;
		}
		i++;
	}
	pthread_mutex_unlock(&mux_q_matrix);
	printf("Q matrix restored!\n");
}

void read_Q_vel_matrix_from_file() {

	FILE *fp;
	int dim_states_f, dim_actions_f, n_states, n_actions;
	char buf[50];
	char q_buff[1024];
	int size;
	char *ptr;
	float val;
	int i, j; // indexes to use for matrix

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions_vel();
	pthread_mutex_unlock(&mux_q_matrix);

	i = 0;
	j = 0;
	fp = fopen(Q_VEL_MAT_FILE_NAME, "r");

	size = 50;
	if (fp == NULL) {
		printf("ERROR: could not open file to read Q_vel matrix\n");
		exit(1);
	}
	// check saved Q matrix dimensions from file
	if (fgets(buf, size, fp) != NULL) {
		sscanf(buf, " %d %d", &dim_states_f, &dim_actions_f);
		if (dim_states_f != n_states) {
			printf("ERROR: STATES dimension different from current Q_vel matrix configuration!\n");
			exit(1);
		}
		if (dim_actions_f != n_actions) {
			printf("ERROR: ACTIONS dimension different from current Q_vel matrix configuration!\n");
			exit(1);
		}
	}

	//printf(" previous Q_vel val: %f \n", ql_get_Q_vel(27,0));
	size = 1024;
	pthread_mutex_lock(&mux_q_matrix);
	while (fgets(q_buff, size, fp) != NULL) {
		ptr = q_buff;
		j = 0;
		
		while ((val = strtof(ptr, &ptr))) {
			ql_set_Q_vel_matrix(i, j, val);
			j++;
		}
		i++;
	}
	pthread_mutex_unlock(&mux_q_matrix);
	//printf(" new Q_vel val: %f \n", ql_get_Q_vel(27,0));
	printf("Q_vel matrix restored!\n");
}

void read_Tr_matrix_from_file() {
	FILE *fp;
	int dim_states_f, dim_actions_f, n_states, n_actions;
	char buf[50];
	char tr_buff[1024];
	int size;
	char *ptr;
	float val;
	int i, j; // indexes to use for matrix

	pthread_mutex_lock(&mux_q_matrix);
	n_states = ql_get_nstates();
	n_actions = ql_get_nactions();
	pthread_mutex_unlock(&mux_q_matrix);

	i = 0;
	j = 0;
	fp = fopen(Tr_MAT_FILE_NAME, "r");

	size = 50;
	if (fp == NULL) {
		printf("ERROR: could not open file to read T_r matrix\n");
		exit(1);
	}
	// check saved Q matrix dimensions from file
	if (fgets(buf, size, fp) != NULL) {
		sscanf(buf, " %d %d", &dim_states_f, &dim_actions_f);
		if (dim_states_f != n_states) {
			printf("ERROR: STATES dimension different from current T_r matrix configuration!\n");
			exit(1);
		}
		if (dim_actions_f != n_actions) {
			printf("ERROR: ACTIONS dimension different from current T_r matrix configuration!\n");
			exit(1);
		}
	}

	size = 1024;
	pthread_mutex_lock(&mux_q_matrix);
	while (fgets(tr_buff, size, fp) != NULL) {
		ptr = tr_buff;
		j = 0;
		while ((val = strtof(ptr, &ptr))) {
			ql_set_Tr_matrix(i, j, val);
			j++;
		}
		i++;
	}
	pthread_mutex_unlock(&mux_q_matrix);

	printf("T_r Matrix restored!\n");
}

// handles learning under the single thread mode
void single_thread_learning() {
	struct Actions_ID a;
	int s, s_new;
	float err;
	float r;
	int i;
	float act;
	struct Agent agent;
	struct Lidar car_sensors[3];
	int d_l, d_f, d_r; // lidar distances
	float updated_distance;
	int train_only_steering = ql_get_train_mode();

	err = 0.0;
	pthread_mutex_lock(&mux_agent);
	agent = rl_agent;
	pthread_mutex_unlock(&mux_agent);
	// learn only if the agent is still alive
	if (agent.alive == 0)
		return;

	a = ql_egreedy_policy(agent.state);
	// backup last iteration delta steering
	rwd_previous_delta = agent.action.delta;
	agent.action.delta = action_to_steering(a.steer_act_id);
	if (train_only_steering)
		agent.action.a = 0.0;
	else
		agent.action.a = action_to_acc(a.vel_act_id);
	// Sarsa
	//agent.action.delta = action_to_steering(agent.a_id);
	agent.car = update_car_model(agent);
	// find distance of agent on track
	updated_distance = agent.distance + sqrt(pow((rl_agent.car.x - agent.car.x), 2) + pow((rl_agent.car.y - agent.car.y), 2));
	agent.distance = updated_distance;
	get_updated_lidars_distance(agent.car, car_sensors);
	d_l = car_sensors[0].d;
	d_f = car_sensors[1].d;
	d_r = car_sensors[2].d;

	s = agent.state;
	// Sarsa
	//a[i] = agent.a_id;
	s_new = decode_lidar_to_state(d_l, d_r, d_f);

	r = get_reward(agent, d_l, d_f, d_r);
	//printf("r: %f \n", r);
	// update max reward for debug
	if (r > max_reward)
		max_reward = r;
	// Q Learning
	err = ql_updateQ(s, a, r, s_new);
	// Q(lambda) Learning
	//err = ql_lambda_updateQ(s, a, r, s_new);
	//printf("s_new: %d, a: %d, s: %d, r: %f \n", s, a, s, r);
	// Sarsa algorithm
	//int a_new = ql_egreedy_policy(s_new);
	//err = updateQ_sarsa(s, a, r, s_new, a_new);

	// update agent state
	agent.state = s_new;
	agent.a_id = a;
	agent.ep_stats.steps++;
	agent.ep_stats.total_reward += r;
	agent.ep_stats.total_td_error += err;

	pthread_mutex_lock(&mux_agent);
	rl_agent = agent;
	pthread_mutex_unlock(&mux_agent);
}

