#ifndef KOHONEN_H_
#define KOHONEN_H_

#define PRINTLINE printf("LINE: %d\n", __LINE__)

#define EXE_MAX		100
#define INP_MAX		5
#define OUT_MAX		36	
//topology types
#define LINE 1
#define RING 2
#define GRID 3
#define MESH 4

#define TS_MAT_FILE_NAME "ts_matrix.txt"

// neural net functions
void init_net(int ni, int no, int topo);
void set_input(float *v);
void compute_output();
int select_winner();
void update_weights(int win); // to adapt to the kind of topo used
void update_weights_line(int win);
void reduce_radius();
void reduce_learning();
void learn_example(float *v);
void learn_ts(int max_epoch);
void train_kohonen_live(float d_l, float d_r, float d_f);
void load_ts_from_file();
void save_ts_to_file();

// auxiliary functions
float k_frand(int xmin, int xmax);
float distance(float *x, float *w);

void set_weight_range(float w_min, float w_max);
void set_learning_range(float ini, float fin);
void set_radius_range(float ini, float fin);
void set_learn_decay(float v);
void set_radius_decay(float v);
void set_ts_matrix(int i, int j, float val);
void set_kw_matrix(int i, int j, float val);

float get_radius();
float get_learning_rate();
float *get_example(int k);
float get_ts_val(int i, int j);
float get_kw_val(int i, int j);
float get_input(int i);
float get_output(int j);
float get_weight(int i, int j);
int get_input_dim();
int get_output_dim();
#endif