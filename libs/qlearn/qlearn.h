#ifndef QLEARN_H_
#define QLEARN_H_

#define PRINTLINE printf("LINE: %d\n", __LINE__)

#define MAX_STATES 400
#define MAX_ACTIONS 20
#define ALPHA0	1.0						// default learning rate
#define EPSINI 0.7						// initial exploration factor
#define EPSFIN 0.01						// final exploration factor
#define GAMMA0 0.9						// default discount factor
#define DECAY0 0.95						// default epsilon decay rate
#define EPSILON0 0.2
#define ALPHA_REWARD 1.0

#define RWD_CRASH -100
#define RWD_ALIVE -1
#define RWD_CORRECT_TURN 2
#define RWD_WRONG_TURN -4
#define RWD_STRAIGHT 2
#define RWD_TURN_STRAIGHT -4
#define RWD_OFF_CENTRE -1
#define RWD_DISTANCE 1

static int n_states;
static int n_actions;
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
//	QL matrice
//----------------------------
static float Q[MAX_STATES][MAX_ACTIONS];
static float T_r[MAX_STATES][MAX_ACTIONS];

void ql_init(int ns, int na);
void ql_set_learning_rate(float lr);
void ql_set_discount_factor(float df);
void ql_set_expl_range(float ini_e, float fin_e);
void ql_set_expl_decay(float d);
void ql_set_expl_factor(float e);
void ql_set_Q_matrix(int s, int a, float val);
void ql_set_Tr_matrix(int s, int a, float val);

float ql_get_learning_rate();
float ql_get_discount_factor();
float ql_get_expl_decay();
float ql_get_epsilon();
float ql_get_Q(int s, int a);
float ql_get_Tr(int s, int a);
int ql_get_nstates();
int ql_get_nactions();

void ql_reduce_expl();
float ql_maxQ(int s);
int ql_best_action(int s);
int ql_egreedy_policy(int s);
float ql_updateQ(int s, int a, float r, int snew);
float ql_lambda_updateQ(int s, int a, float r, int snew);

float updateQ_sarsa(int s, int a, float r, int snew, int anew);

float frand(float xmin, float xmax);
//float evaluate_convergence(float prev_errr, float curr_err);
#endif