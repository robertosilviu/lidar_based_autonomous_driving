#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>
#include <math.h>

//----------------------------------------------------------------------------
// ------------------------------CONSTANTS------------------------------------
//----------------------------------------------------------------------------
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
#define SCALE 10

#define PI 3.14

static const char *TRACK_FILE = "img/track_4.tga";
static const char *CAR_FILE = "img/orange_car.tga";

#define INIT_CAR_X 500
#define INIT_CAR_Y 350
//-------------------------------SENSOR---------------------------------------
#define SMAX 100 // lidar beam max distance
#define STEP 1 // lidar resolution

//-------------------------------CUSTOM STRUCTURES----------------------------
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
	float a;
	float theta;
	struct Lidar left_lidar;
	struct Lidar front_lidar;
	struct Lidar right_lidar;
};

struct Agent {
	struct Car car;
	int alive; // 1 = alive, 0 = dead due to collision with track borders
	float distance;	// distance raced on track 
};

//----------------------------------------------------------------------------
// ------------------------------GLOBAL VARIABLES-----------------------------
//----------------------------------------------------------------------------
BITMAP *track_bmp;
BITMAP *car_bmp;
BITMAP *scene_bmp;

struct Agent agent;

void init();
void init_agent();
void init_scene();
int read_sensor(int x0, int y0, float alpha);
void get_track_bbox(float *length, float *height);
void draw_track();
void draw_car();
void draw_sensors();
void update_scene();

//--------------------------------UTILS---------------------------------------
float deg_to_rad(float deg_angle);
float rad_to_deg(float rad_angle);

int main() {
	printf("Starting app...\n");

	init();
	
	readkey();
	destroy_bitmap(track_bmp);
	destroy_bitmap(scene_bmp);
	allegro_exit();

	return 0;
}

void init() {
	allegro_init();
	install_keyboard();
	install_mouse();
	show_mouse(screen);

	set_color_depth(BITS_COL);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, WIN_X, WIN_Y, 0, 0);
	clear_to_color(screen, BKG);

	// organize app windows
	//int white = makecol(255, 255, 255);
	//rect(screen,0, WIN_Y-1, SIM_X, WIN_Y-SIM_Y, white); // track area
	init_scene();
	init_agent();
	//draw_sensors();
	update_scene();
}

void init_scene() {
	//set_color_conversion(COLORCONV_32_TO_24);
	track_bmp = load_bitmap(TRACK_FILE, NULL);
	if (track_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}

	scene_bmp = track_bmp;	// update scene bitmap to initial track state

	car_bmp = load_bitmap(CAR_FILE, NULL);
	if (car_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}
	draw_sprite(scene_bmp, car_bmp, INIT_CAR_X, INIT_CAR_Y);
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void update_scene() {
	draw_sensors();
	blit(scene_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, scene_bmp->w, scene_bmp->h);
}

void draw_track() {
	//set_color_conversion(COLORCONV_32_TO_24);
	track_bmp = load_bitmap(TRACK_FILE, NULL);

	if (track_bmp == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}

	blit(track_bmp, screen, 0, 0, 0, WIN_Y-SIM_Y, track_bmp->w, track_bmp->h);
}

// rotate car sprite inside the scene using fixed points convention
void draw_car() {
	float angle, theta_deg;
	fixed rot; // allegro type to express fixed point number

	theta_deg = rad_to_deg(agent.car.theta);
	angle = 64 - theta_deg*(float)256/360;
	rot = ftofix(angle);
	rotate_sprite(scene_bmp, car_bmp, agent.car.x, agent.car.y, rot);
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
	printf(" left_lidar: (%d,%d) \n", x, y);

	lidar = agent.car.front_lidar;
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);
	printf(" front_lidar: (%d,%d) \n", x, y);

	lidar = agent.car.right_lidar;
	x = lidar.x + lidar.d*cos(lidar.alpha);
	y = lidar.y - lidar.d*sin(lidar.alpha);
	line(scene_bmp, lidar.x, lidar.y, x, y, col);
	printf(" right_lidar: (%d,%d) \n", x, y);
}

float deg_to_rad(float deg_angle) {
	return deg_angle*PI/180;
}

float rad_to_deg(float rad_angle) {
	return rad_angle*180/PI;
}

void init_agent() {
	struct Car vehicle;
	
	printf("Initializing agent...\n");

	vehicle.x = INIT_CAR_X;
	vehicle.y = INIT_CAR_Y;
	vehicle.theta = 0.0;
	vehicle.v = 0.0;
	vehicle.a = 0.0;

	vehicle.left_lidar.x = vehicle.x + car_bmp->w;
	vehicle.left_lidar.y = (int)vehicle.y;
	vehicle.left_lidar.alpha = deg_to_rad(45.0);
	vehicle.left_lidar.d = read_sensor(
							vehicle.left_lidar.x, 
							vehicle.left_lidar.y, 
							vehicle.theta + vehicle.left_lidar.alpha);

	vehicle.front_lidar.x = vehicle.x + car_bmp->w;
	vehicle.front_lidar.y = vehicle.y + (car_bmp->h/2);
	vehicle.front_lidar.alpha = 0.0;
	vehicle.front_lidar.d = read_sensor(
							vehicle.front_lidar.x, 
							vehicle.front_lidar.y, 
							vehicle.theta + vehicle.front_lidar.alpha);

	vehicle.right_lidar.x = vehicle.x + car_bmp->w;
	vehicle.right_lidar.y = vehicle.y + car_bmp->h;
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
/*
void get_track_bbox(float *length, float *height) {
	float x_int = 0, y_int = 0, x_ext = 0, y_ext = 0;
	float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
	int err = 0; // file closing error handler
	FILE *reader;
	char line[1024]; // array of chars to save each line of file

	reader = fopen(TRACK_FILE, "r");
	if (reader == NULL) {
		printf("ERROR: could not access specified track file! \n");
		exit(1);
	}

	// get min and max value of x and y in order to scale
	// the coordinates to the area dedicated to the track on screen
	while ( fgets(line, sizeof line, reader) != NULL) {
		sscanf(line, "%f %f %f %f", &x_int, &y_int, &x_ext, &y_ext);
		// internal bounds
		x_min = (x_int < x_min) ? x_int : x_min;
		x_max = (x_int > x_max) ? x_int : x_max;
		y_min = (y_int < y_min) ? y_int : y_min;
		y_max = (y_int > y_max) ? y_int : y_max;
		// external bounds
		x_min = (x_ext < x_min) ? x_ext : x_min;
		x_max = (x_ext > x_max) ? x_ext : x_max;
		y_min = (y_ext < y_min) ? y_ext : y_min;
		y_max = (y_ext > y_max) ? y_ext : y_max;

	}

	err = fclose(reader);
	if (err == EOF) {
		printf("ERROR: an error occured while trying to close the file! \n");
		exit(1);
	}

	// compute bounding boxes
	*length = (x_max - x_min)/2;
	*height = (y_max - y_min)/2;
}

void tmp_track() {
	float x_int = 0.0, y_int = 0.0, x_ext = 0.0, y_ext = 0.0;
	float track_length = 0.0, track_height = 0.0;
	int err = 0; // file closing error handler
	FILE *reader;
	char line[1024];

	get_track_bbox(&track_length, &track_height);

	reader = fopen(TRACK_FILE, "r");
	if (reader == NULL) {
		printf("ERROR: could not access specified track file! \n");
		exit(1);
	}

	// read (x,y) coordinates of internal and external track boundaries
	// the points format inside the txt file is 
	// x_internal y_internal x_external y_external
	while ( fgets(line, sizeof line, reader) != NULL) {
		sscanf(line, "%f %f %f %f", &x_int, &y_int, &x_ext, &y_ext);
		x_int += track_length/2;
		y_int += track_height/2;
		x_ext += track_length/2;
		y_ext += track_height/2;
		putpixel(screen, XCEN_SIM+x_int/SCALE, SIM_Y-YCEN_SIM-y_int/SCALE, WHITE);
		putpixel(screen, XCEN_SIM+x_ext/SCALE, SIM_Y-YCEN_SIM-y_ext/SCALE, WHITE);

	}

	err = fclose(reader);
	if (err == EOF) {
		printf("ERROR: an error occured while trying to close the file! \n");
		exit(1);
	}

}
*/