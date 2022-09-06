#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "qlearn.h"

static int n_states;
static int n_actions;
static int n_actions_vel;
//static int goal_state;

static float alpha;						// learning rate
static float gam;						// discount factor
static float decay;						// decay rate for epsilon
static float norm_eps = 1.0;			// normal exploration probability
static float ini_eps;					// initial exploration probability
static float fin_eps;					// final exploration probability
static float epsilon;					// actual exploration probability
// if mode is INFERENCE return always best actions
static int mode = TRAINING;
static int train_only_steering = 0;
//----------------------------
//	QL matrixes
//----------------------------

// Q matrix related to steering
static float Q[MAX_STATES][MAX_ACTIONS];

// Q matrix related to velocity
static float Q_vel[MAX_STATES][MAX_ACTIONS];

float frand(float xmin, float xmax) {
	float range;

	assert(xmax >= xmin);
	
	range = (xmax - xmin);

	return (xmin + range*(float)rand()/RAND_MAX);
}

void ql_init(int ns, int na, int na_vel) {
	int s, a;
	n_states = ns;
	n_actions = na;
	n_actions_vel = na_vel;

	if(n_states > MAX_STATES) {
		printf("Number of states exceeds max limit!\n");
		exit(1);
	}

	if(n_actions > MAX_ACTIONS) {
		printf("Number of actions exceeds max limit!\n");
		exit(1);
	}

	if(n_actions_vel > MAX_ACTIONS) {
		printf("Number of actions exceeds max limit!\n");
		exit(1);
	}

	alpha = ALPHA0;
	gam = GAMMA0;
	ini_eps = EPSINI;
	fin_eps = EPSFIN;
	decay = DECAY0;
	epsilon = EPSILON0;

	for(s = 0; s < n_states; s++) {
		for(a = 0; a < n_actions; a++) {
			//printf("s: %d, a: %d\n", s, a);
			// steer
			Q[s][a] = 0.0;
		}
	}

	for(s = 0; s < n_states; s++) {
		for(a = 0; a < n_actions_vel; a++) {
			//printf("s: %d, a: %d\n", s, a);
			// steer
			Q_vel[s][a] = 0.0;
		}
	}
}

void ql_set_learning_rate(float lr) {
	alpha = lr;
	printf("Learning rate: alpha = %f\n", alpha);
}

void ql_set_discount_factor(float df) {
	gam = df;
	printf("Discount factor: gamma = %f\n", gam);
}

void ql_set_expl_range(float ini_e, float fin_e) {
	ini_eps = ini_e;
	fin_eps = fin_e;
	printf("Eploration probability: ini_eps = %f, fin_eps = %f\n", ini_e, fin_e);
}

void ql_set_expl_factor(float e) {
	epsilon = e;
	printf("Exploration factor: epsilon = %f\n", epsilon);
}

void ql_set_expl_decay(float d) {
	decay = d;
	printf("Exploration decay: decay = %f\n", decay);
}

void ql_set_Q_matrix(int s, int a, float val) {

	if (s >= n_states) {
		printf("ERROR Q matrix: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions) {
		printf("ERROR Q matrix: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	Q[s][a] = val;
}

void ql_set_Q_vel_matrix(int s, int a, float val) {

	if (s >= n_states) {
		printf("ERROR Q_vel matrix: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions_vel) {
		printf("ERROR Q_vel matrix: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	Q_vel[s][a] = val;
}

void ql_set_rl_mode(int val) {
	if ((val != INFERENCE) && (val != TRAINING)) {
		printf(" INVALID mode for qlearn library -> %d \n", val);
		exit(1);
	}

	mode = val;
}

void ql_set_train_mode(int val) {
	if ((val != 1) && (val != 0)) {
		printf(" INVALID train mode for qlearn library -> %d \n", val);
		exit(1);
	}

	if (val == 1)
		printf("train_mode is: only STEERING \n");
	else if (val == 0)
		printf("train_mode is: STEERING + ACCELERATION\n");
	
	train_only_steering = val;
}

float ql_get_learning_rate() {
	return alpha;
}

float ql_get_discount_factor() {
	return gam;
}

float ql_get_expl_decay() {
	return decay;
}

float ql_get_epsilon() {
	return epsilon;
}

float ql_get_Q(int s, int a) {
	if (s >= n_states) {
		printf("ERROR Q matrix: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions) {
		printf("ERROR Q matrix: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	return Q[s][a];
}

float ql_get_Q_vel(int s, int a) {
	if (s >= n_states) {
		printf("ERROR Q_vel matrix: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions_vel) {
		printf("ERROR Q_vel matrix: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	return Q_vel[s][a];
}

int ql_get_nstates() {
	return n_states;
}

int ql_get_nactions() {
	return n_actions;
}

int ql_get_nactions_vel() {
	return n_actions_vel;
}

int ql_get_rl_mode() {
	//assert((mode == 0) || (mode == 1));
	return mode;
}

int ql_get_train_mode() {
	return train_only_steering;
}

void ql_reduce_expl() {
	norm_eps = decay*norm_eps;
	epsilon = fin_eps + norm_eps*(ini_eps - fin_eps);

	printf("Reducing exploration rate -> norm_eps: %f, epsilon: %f \n", norm_eps, epsilon);
	
	assert(epsilon <= ini_eps);
	assert(epsilon >= fin_eps);

}

float ql_maxQ(int s, int flag) {
	int a;
	float m;

	if (flag == 0) {
		m = Q[s][0];

		for(a = 1; a < n_actions; a++) {
			if (Q[s][a] > m) {
				m = Q[s][a];
				//printf("q: %d\n", Q[s][a]);
			}

		}
	}
	else if (flag == 1) {
		m = Q_vel[s][0];

		for(a = 1; a < n_actions_vel; a++) {
			if (Q_vel[s][a] > m) {
				m = Q_vel[s][a];
				//printf("q: %d\n", Q_vel[s][a]);
			}

		}
	}
	else {
		printf("ql_maxQ() error: not a valid input for -flag- argumment! \n");
		exit(1);
	}

	//printf("m: %d\n", m);
	return m;
}

// TO-DO: add condition for inference mode
struct Actions_ID ql_best_action(int s) {
	int a, ba;
	float m;
	struct Actions_ID ql_act;

	// steer
	m = Q[s][0];
	ba = 0;

	for(a = 1; a < n_actions; a++) {
		if (Q[s][a] > m) {
			m = Q[s][a];
			ba = a;
		}
	}
	// when value is zero return casual action
	if (Q[s][ba] == 0)
		ba = rand()%n_actions;
	
	ql_act.steer_act_id = ba;

	if(train_only_steering)
		return ql_act;
	
	// velocity
	m = Q_vel[s][0];
	ba = 0;

	for(a = 1; a < n_actions_vel; a++) {
		if (Q_vel[s][a] > m) {
			m = Q_vel[s][a];
			ba = a;
		}
	}
	// when value is zero return casual action
	if (Q_vel[s][ba] == 0)
		ba = rand()%n_actions_vel;
	
	ql_act.vel_act_id = ba;

	return ql_act;
}

struct Actions_ID ql_egreedy_policy(int s) {
	int ra;
	float x;
	struct Actions_ID best_actions, new_actions;
	
	// get best actions id for state s
	best_actions = ql_best_action(s);
	// update steer action
	ra = rand()%n_actions;
	assert((ra <= n_actions) && (ra >= 0));

	x = frand(0.0, 1.0);
	assert(x >= 0.0);
	assert(x <= 1.0);
	
	//printf("ra: %d, x: %f, ba: %d\n", ra, x, best_actions.steer_act_id);
	if (x < epsilon)
		new_actions.steer_act_id = ra;
	else 
		new_actions.steer_act_id = best_actions.steer_act_id;

	if(train_only_steering)
		return new_actions;

	// update velocity action
	ra = rand()%n_actions_vel;
	assert((ra <= n_actions_vel) && (ra >= 0));

	x = frand(0.0, 1.0);
	assert(x >= 0.0);
	assert(x <= 1.0);
	
	//printf("ra: %d, x: %f, ba: %d\n", ra, x, best_actions.vel_act_id);
	if (x < epsilon)
		new_actions.vel_act_id = ra;
	else 
		new_actions.vel_act_id = best_actions.vel_act_id;

	return new_actions;
}

float ql_updateQ(int s, struct Actions_ID a, float r, int snew) {
	float td_err;	// TD error

	td_err = ql_updateQ_steer(s, a.steer_act_id, r, snew);
	if (!train_only_steering)
		ql_updateQ_vel(s, a.vel_act_id, r, snew);

	return td_err;
}

float ql_updateQ_steer(int s, int a, float r, int snew) {
	float q_target;	// target Q value
	float td_err;	// TD error
	// flag to discern between steer and velocity q-learn update
	int flag = 0; 
	
	//if (r == RWD_CRASH)
	//	q_target = r + gam*ql_maxQ(s, flag);
	//else
	//	q_target = r + gam*ql_maxQ(snew, flag);
	q_target = r + gam*ql_maxQ(snew, flag);

	//q_target = r + gam*ql_maxQ(snew);
	td_err = q_target - Q[s][a];
	//old_q = Q[s][a];
	//Q[s][a] = (1 - alpha) * Q[s][a] + alpha * (q_target);
	Q[s][a] = Q[s][a] + alpha * (q_target - Q[s][a]);
	//printf("Q: %f, td_err: %f \n", Q[s][a], td_err);


	return fabs(td_err);
}

float ql_updateQ_vel(int s, int a, float r, int snew) {
	float q_target;	// target Q value
	float td_err;	// TD error
	// flag to choose max value from Q_vel 
	int flag = 1;
	
	//if (r == RWD_CRASH)
	//	q_target = r + gam*ql_maxQ(s, flag);
	//else
	//	q_target = r + gam*ql_maxQ(snew, flag);
	q_target = r + gam*ql_maxQ(snew, flag);

	//q_target = r + gam*ql_maxQ(snew);
	td_err = q_target - Q_vel[s][a];
	//old_q = Q_vel[s][a];
	//Q_vel[s][a] = (1 - alpha) * Q_vel[s][a] + alpha * (q_target);
	Q_vel[s][a] = Q_vel[s][a] + alpha * (q_target - Q_vel[s][a]);
	//printf("Q: %f, td_err: %f \n", Q_vel[s][a], td_err);

	return fabs(td_err);
}