#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "qlearn.h"

float frand(float xmin, float xmax) {
	float range;

	range = (xmax - xmin);

	return (xmin + range*(float)rand()/RAND_MAX);
}

void ql_init(int ns, int na) {
	int s, a;
	n_states = ns;
	n_actions = na;

	if(n_states > MAX_STATES) {
		printf("Number of states exceeds max limit!\n");
		exit(1);
	}

	if(n_actions > MAX_ACTIONS) {
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
		for(a = 0; a < n_states; a++) {
			Q[s][a] = 0;
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

void ql_set_expl_decay(float d) {
	decay = d;
	printf("Exploration decay: decay = %f\n", decay);
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

void ql_reduce_expl() {
	norm_eps = decay*norm_eps;
	epsilon = fin_eps + norm_eps*(ini_eps - fin_eps);

	printf("Reducing exploration rate -> norm_eps: %f, epsilon: %f \n", norm_eps, epsilon);
}

float ql_maxQ(int s) {
	int a;
	float m;

	m = Q[s][0];

	for(a = 1; a < n_states; a++) {
		if (Q[s][a] > m)
			m = Q[s][a];
	}

	return m;
}

int ql_best_action(int s) {
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

	return ba;
}

int ql_egreedy_policy(int s) {
	int ra, ba;
	float x;
	
	ba = ql_best_action(s);
	ra = rand()%n_actions;
	x = frand(0, 1);
	//printf("ra is %d\n", ra);
	if (x < epsilon)
		return ra;
	else 
		return ba;
}

float ql_updateQ(int s, int a, float r, int snew) {
	float q_target;	// target Q value
	float td_err;	// TD error

	q_target = r + gam*ql_maxQ(snew);
	td_err = q_target - Q[s][a];
	Q[s][a] = Q[s][a] + alpha*td_err;

	return fabs(td_err);
}