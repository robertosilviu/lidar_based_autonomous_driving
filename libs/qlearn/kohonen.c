#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kohonen.h"

// structures
static float ts[EXE_MAX][INP_MAX]; // training examples
static float kw[OUT_MAX][INP_MAX]; // output weights
static float ki[INP_MAX]; // input vector
static float ko[OUT_MAX]; // output vector

static int k_inputs;		// number of input units
static int k_units;		// number of output units
static int topology;		// output map topology
static int nex;			// number of examples

static float k_radius;		// neighborhood radius
static float k_r_ini, k_r_fin; // initial & final radius
static float k_r_norm;		// normalized radius in [0;1]

static float k_alpha;		// learning rate
static float k_a_ini, k_a_fin;	//initial & final learning rate
static float k_a_norm;		// normalized learning rate in [0;1]

static float learn_decay;	// learning rate decay
static float radius_decay;	// radius decay
static float w_min, w_max;	// initial weight range

// neural net functions
void init_net(int ni, int no, int topo) {
	int i, j;

	k_inputs = ni;
	k_units = no;
	topology = topo;

	if ((topology == GRID) || (topology == MESH)) {
		k_r_ini = sqrt((double)k_units);
	} else {
		k_r_ini = (k_units/2) + 1;
	}

	k_alpha = k_a_ini;
	k_radius = k_r_ini;
	k_a_norm = 1.0;
	k_r_norm = 1.0;

	srand(time(NULL));

	for (j=0; j < k_units; j++) {
		for (i=0; i < k_inputs; i++)
			kw[j][i] = k_frand(w_min, w_max);
	}
}

void set_input(float *v) {
	int i;
	for (i = 0; i< k_inputs; i++) {
		ki[i] = v[i];
	}
}

void compute_output() {
	int i, j;
	for (j=0; j < k_units; j++) {
		ko[j] = 0.0;
		for (i=0; i < k_inputs; i++) {
			ko[j] = ko[j] + kw[j][i] * ki[i];
		}
	}
}

int select_winner() {
	int i, winner;
	float min, d;

	min = distance(ki, kw[0]);
	winner = 0;

	for (i=1; i < k_units; i++) {
		d = distance(ki, kw[i]);
		if (d <= min) {
			min = d;
			winner = i;
		}
	}

	return winner;
}

// to adapt to the kind of topo used
void update_weights(int win) {
	if (topology == LINE)
		update_weights_line(win);
}
//line
void update_weights_line(int win) {
	int i, j, d2, r2;
	int j_min, j_max;
	float phi;

	j_min = win - (int)(k_radius + 0.5);
	if (j_min < 0)
		j_min = 0;
	j_max = win + (int)(k_radius + 0.5);
	if (j_max > k_units - 1)
		j_max = k_units - 1;

	r2 = pow(k_r_ini, 2);
	for (j=j_min; j <= j_max; j++) {
		d2 = pow((j - win), 2);
		phi = 1.0 - (float)(d2/r2);
		for (i=0; i < k_inputs; i++)
			kw[j][i] += k_alpha*phi*(ki[i] - kw[j][i]);
	}

}

void reduce_radius() {
	k_r_norm = k_r_norm * radius_decay;
	k_radius = k_r_fin + (k_r_norm * (k_r_ini - k_r_fin));
}

void reduce_learning() {
	k_a_norm = k_a_norm * learn_decay;
	k_alpha = k_a_fin + (k_a_norm * (k_a_ini - k_a_fin));
}

void learn_example(float *v) {
	int win;

	set_input(v);
	compute_output();
	win = select_winner();
	update_weights(win);
}

void learn_ts(int max_epoch) {
	int k, epoch = 0;
	float *p;

	do {
		for (k=0; k < nex; k++) {
			p = get_example(k);
			learn_example(p);
		}
		epoch++;
		reduce_radius();
		reduce_learning();
	} while (epoch < max_epoch);
}

void load_ts_from_file() {
	FILE *fp;
	int dim_inputs_f, dim_units_f, n_inputs, n_units;
	char buf[50];
	char tr_buff[1024];
	int size;
	char *ptr;
	float val;
	int i, j; // indexes to use for matrix

	n_inputs = get_input_dim();
	n_units = get_output_dim();
	i = 0;
	j = 0;
	fp = fopen(TS_MAT_FILE_NAME, "r");

	size = 50;
	if (fp == NULL) {
		printf("ERROR: could not open file to read ts matrix\n");
		exit(1);
	}
	// check saved Q matrix dimensions from file
	if (fgets(buf, size, fp) != NULL) {
		sscanf(buf, " %d %d", &dim_inputs_f, &dim_units_f);
		if (dim_inputs_f != n_inputs) {
			printf("ERROR: INPUTS dimension different from current ts matrix configuration!\n");
			exit(1);
		}
		if (dim_units_f != n_units) {
			printf("ERROR: OUTPUT dimension different from current ts matrix configuration!\n");
			exit(1);
		}
	}

	size = 1024;
	while (fgets(tr_buff, size, fp) != NULL) {
		ptr = tr_buff;
		j = 0;
		val = strtof(ptr, &ptr);
		while (val) {
			set_ts_matrix(i, j, val);
			j++;
		}
		i++;
	}
	
	printf("ts Matrix restored!\n");
}

void save_ts_to_file() {
	FILE *fp;
	int n_inputs, n_units;
	int i, j;
	n_inputs = get_input_dim();
	n_units = get_output_dim();
	float ts_tmp[n_inputs][n_units];
	
	for(i = 0; i < n_inputs; i++) {
		for(j = 0; j < n_units; j++) {
			ts_tmp[i][j] = get_ts_val(i, j);
		}
	}

	fp = fopen(TS_MAT_FILE_NAME, "w");
	if (fp == NULL) {
		printf("ERROR: could not open file to write ts matrix\n");
		exit(1);
	}
	// save the number of states and actions
	fprintf(fp, "%d %d\n", n_inputs, n_units);
	// save values of Q Matrix
	for(i = 0; i < n_inputs; i++) {
		for(j = 0; j < n_units; j++) {
			fprintf(fp, "%.2f ", ts_tmp[i][j]);
		}
		fprintf(fp, "\n");
	}
	
	fclose(fp);
	printf("ts matrix saved on file %s!\n", TS_MAT_FILE_NAME);
}

// auxiliary functions
float k_frand(int xmin, int xmax) {
	float range = 0.0;

	range = (xmax - xmin);
	return (xmin + range * rand()/(float)RAND_MAX);
}

float distance(float *x, float *w) {
	int i;
	float d, sum = 0.0;
	for (i=0; i < k_inputs; i++) {
		d = w[i] - x[i];
		sum = sum + pow(d,2);
	}

	return sum;
}

void set_weight_range(float min, float max) {
	w_min = min;
	w_max = max;
}

void set_learning_range(float ini, float fin) {
	k_a_ini = ini;
	k_a_fin = fin;
}

void set_radius_range(float ini, float fin) {
	k_r_ini = ini;
	k_r_fin = fin;
}

void set_learn_decay(float v) {
	learn_decay = v;
}

void set_radius_decay(float v) {
	radius_decay = v;
}

void set_ts_matrix(int i, int j, float val) {
	ts[i][j] = val;
}

void set_kw_matrix(int i, int j, float val) {
	kw[i][j] = val;
}

float get_radius() {
	return k_radius;
}

float get_learning_rate() {
	return k_alpha;
}

float *get_example(int k) {
	return ts[k];
}

float get_ts_val(int i, int j) {
	return ts[i][j];
}

float get_kw_val(int i, int j) {
	return kw[i][j];
}

float get_input(int i) {
	return ki[i];
}

float get_output(int j) {
	return ko[j];
}

float get_weight(int j, int i) {
	return kw[j][i];
}

int get_input_dim() {
	return k_inputs;
}

int get_output_dim() {
	return k_units;
}