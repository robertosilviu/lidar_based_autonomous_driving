#ifndef KOHONEN_H_
#define KOHONEN_H_

#define PRINTLINE printf("LINE: %d\n", __LINE__)

#define EXE_MAX		100
#define INP_MAX		100
#define OUT_MAX		36	
//topology types
#define LINE 1
#define RING 2
#define GRID 3
#define MESH 4

#define TS_MAT_FILE_NAME "ts_matrix.txt"

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
void init_net(int ni, int no, int topo);
void set_input(float *v);
void compute_output();
int select_winner();
void update_weights(int win); // to adapt to the kind of topo used
void reduce_radius();
void reduce_learning();
void learn_example(float *v);
void learn_ts(int max_epoch);
void train_kohonen_live(float d_l, float d_r, float d_f);
void load_ts_from_file();
void save_ts_to_file();

// auxiliary functions
float frand(int xmin, int xmax);
float distance(float *x, float *w);

void set_weight_range(float w_min, float w_max);
void set_learning_range(float ini, float fin);
void set_radius_range(float ini, float fin);
void set_learn_decay(float v);
void set_radius_decay(float v);
void set_ts_matrix(int i, int j, float val);

float get_radius();
float get_learning_rate();
float *get_example(int k);
float get_ts_val(int i, int j);
float get_input(int i);
float get_output(int j);
float get_weight(int i, int j);
int get_input_dim();
int get_output_dim();
#endif