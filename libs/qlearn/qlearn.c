#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
static float lambda = 0.3;
//----------------------------
//	QL matrixes
//----------------------------

// Q matrix related to steering
static float Q[MAX_STATES][MAX_ACTIONS];

// Q matrix related to velocity
static float Q_vel[MAX_STATES][MAX_ACTIONS];
static float T_r[MAX_STATES][MAX_ACTIONS];

float frand(float xmin, float xmax) {
	float range;

	range = (xmax - xmin);

	return (xmin + range*(float)rand()/RAND_MAX);
}

void ql_init(int ns, int na, int na_vel) {
	int s, a;
	n_states = ns;
	n_actions = na;
	n_actions_vel = na;

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

	// Q(lambda) learning
	for(s = 0; s < n_states; s++) {
		for(a = 0; a < n_actions; a++) {
			//printf("s: %d, a: %d\n", s, a);
			T_r[s][a] = 0.0;
			// TO-DO see if some changes are required for Q_vel
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

void ql_set_Tr_matrix(int s, int a, float val) {

	if (s >= n_states) {
		printf("ERROR: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions) {
		printf("ERROR: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	T_r[s][a] = val;
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

float ql_get_Tr(int s, int a) {
	if (s >= n_states) {
		printf("ERROR: current state index greater than STATES dimension: %d > %d\n", s, n_states);
		exit(1);
	}

	if (a >= n_actions) {
		printf("ERROR: current action index greater than ACTIONS dimension: %d > %d\n", a, n_actions);
		exit(1);
	}

	return T_r[s][a];
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

void ql_reduce_expl() {
	norm_eps = decay*norm_eps;
	epsilon = fin_eps + norm_eps*(ini_eps - fin_eps);

	printf("Reducing exploration rate -> norm_eps: %f, epsilon: %f \n", norm_eps, epsilon);
}

float ql_maxQ(int s) {
	int a;
	float m;

	m = Q[s][0];

	for(a = 1; a < n_actions; a++) {
		if (Q[s][a] > m) {
			m = Q[s][a];
			//printf("q: %d\n", Q[s][a]);
		}
			
	}
	//printf("m: %d\n", m);
	return m;
}

float ql_maxQ_vel(int s) {
	int a;
	float m;

	m = Q_vel[s][0];

	for(a = 1; a < n_actions; a++) {
		if (Q_vel[s][a] > m) {
			m = Q_vel[s][a];
			//printf("q: %d\n", Q[s][a]);
		}
			
	}
	//printf("m: %d\n", m);
	return m;
}

int ql_best_action_steer(int s) {
	int a, ba;
	float m;

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
	
	return ba;
}

int ql_best_action_vel(int s) {
	int a, ba;
	float m;

	m = Q_vel[s][0];
	ba = 0;

	for(a = 1; a < n_actions; a++) {
		if (Q_vel[s][a] > m) {
			m = Q_vel[s][a];
			ba = a;
		}
	}
	// when value is zero return casual action
	if (Q_vel[s][ba] == 0)
		ba = rand()%n_actions;
	
	return ba;
}

// change to ql_egreedy_policy_steer
int ql_egreedy_policy_steer(int s) {
	int ra, ba;
	float x;
	
	ba = ql_best_action_steer(s);
	ra = rand()%n_actions;
	x = frand(0.0, 1.0);
	//printf("ra: %d, x: %f, ba: %d\n", ra, x, ba);
	if (x < epsilon)
		return ra;
	else 
		return ba;
}

int ql_egreedy_policy_vel(int s) {
	int ra, ba;
	float x;
	
	ba = ql_best_action_vel(s);
	ra = rand()%n_actions;
	x = frand(0.0, 1.0);
	//printf("ra: %d, x: %f, ba: %d\n", ra, x, ba);
	if (x < epsilon)
		return ra;
	else 
		return ba;
}

float ql_updateQ(int s, int a, float r, int snew) {
	float q_target;	// target Q value
	float td_err;	// TD error
	//float old_q;
	//q_target = r + gam*ql_maxQ(snew);
	//td_err = q_target - Q[s][a];
	//Q[s][a] = Q[s][a] + alpha*td_err;
	//printf("r: %f, s: %d, s_new: %d, a: %d \n", r, s, snew, a);

	
	if (r == RWD_CRASH)
		q_target = r + gam*ql_maxQ(s);
	else
		q_target = r + gam*ql_maxQ(snew);
	

	//q_target = r + gam*ql_maxQ(snew);
	td_err = q_target - Q[s][a];
	//old_q = Q[s][a];
	Q[s][a] = (1 - alpha) * Q[s][a] + alpha * (q_target);
	//printf("Q: %f, td_err: %f \n", Q[s][a], td_err);


	return fabs(td_err);
}

float ql_updateQ_vel(int s, int a, float r, int snew) {
	float q_target;	// target Q value
	float td_err;	// TD error
	//float old_q;
	//q_target = r + gam*ql_maxQ(snew);
	//td_err = q_target - Q[s][a];
	//Q[s][a] = Q[s][a] + alpha*td_err;
	//printf("r: %f, s: %d, s_new: %d, a: %d \n", r, s, snew, a);

	
	if (r == RWD_CRASH)
		q_target = r + gam*ql_maxQ_vel(s);
	else
		q_target = r + gam*ql_maxQ_vel(snew);
	

	//q_target = r + gam*ql_maxQ(snew);
	td_err = q_target - Q_vel[s][a];
	//old_q = Q_vel[s][a];
	Q_vel[s][a] = (1 - alpha) * Q_vel[s][a] + alpha * (q_target);
	//printf("Q: %f, td_err: %f \n", Q_vel[s][a], td_err);


	return fabs(td_err);
}

float ql_lambda_updateQ(int s, int a, float r, int snew) {
	float max_s, max_snew;
	float e, e_dot;
	int i, j;
	float old_q;

	old_q = Q[s][a];
	max_s = ql_maxQ(s);
	max_snew = ql_maxQ(snew);

	e_dot = r + gam*max_snew - Q[s][a];
	e = r + gam*max_snew - max_s;

	for(i = 0; i < n_states; i++) {
		for(j = 0; j < n_actions; j++) {
			//printf("s: %d, a: %d\n", s, a);
			T_r[i][j] = gam*lambda*T_r[i][j];
			Q[i][j] = Q[i][j] + alpha*T_r[i][j]*e;
		}
	}

	Q[s][a] = Q[s][a] + alpha*e_dot;
	//T_r[s][a] = T_r[s][a] + 1;
	T_r[s][a] = 1;

	return fabs(Q[s][a] - old_q);
}


float updateQ_sarsa(int s, int a, float r, int snew, int anew) {
	//float td_err;
	float old_q;
	// get action from new state based on e_greedy policy
	//a_new = ql_egreedy_policy(snew);
	//printf("s: %d, a: %d \n", s, a);
	//printf("q: %f \n", Q[s][a]);
	old_q = Q[s][a];
	Q[s][a] = Q[s][a] + alpha * (r + gam*Q[snew][anew] - Q[s][a]);

	return fabs(Q[s][a] - old_q);
}
/*
float evaluate_convergence(float prev_errr, float curr_err) {
	printf("to do\n");
}
*/