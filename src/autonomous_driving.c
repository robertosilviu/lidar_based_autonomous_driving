#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>

//----------------------------------------------------------------------------
// ------------------------------CONSTANTS------------------------------------
//----------------------------------------------------------------------------
#define WIN_X 1024
#define WIN_Y 1024
#define BITS_COL 24
#define BKG 0
#define WHITE 15

#define SIM_X 800
#define SIM_Y 800
#define XCEN_SIM SIM_X/2
#define YCEN_SIM SIM_Y/2
#define SCALE 10

#define FE 180 // false eastaning
#define PI 3.14

static const char *TRACK_FILE = "img/track_4.tga";

//----------------------------------------------------------------------------
// ------------------------------GLOBALS------------------------------------
//----------------------------------------------------------------------------
BITMAP *track;

void init();
void get_track_bbox(float *length, float *height);
void draw_track();
float deg_to_rad(float deg_angle);

int main() {
	printf("Starting app...\n");

	init();
	
	readkey();
	destroy_bitmap(track);
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
	int white = makecol(255, 255, 255);
	//rect(screen,0, WIN_Y-1, SIM_X, WIN_Y-SIM_Y, white); // track area

	draw_track();
}

void draw_track() {
	//set_color_conversion(COLORCONV_32_TO_24);
	track = load_bitmap(TRACK_FILE, NULL);
	if (track == NULL) {
		printf("ERROR: file not found\n");
		exit(1);
	}

	blit(track, screen, 0, 0, 0, WIN_Y-SIM_Y, track->w, track->h);
}

float deg_to_rad(float deg_angle) {
	return deg_angle/PI * 180;
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