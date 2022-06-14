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
#define RWD_DISTANCE 0.1
#define RWD_BAD_ACC -20
#define RWD_CORRECT_ACC -1

struct Actions_ID {
	int steer_act_id;
	int vel_act_id;
};

void ql_init(int ns, int na, int na_vel);
void ql_set_learning_rate(float lr);
void ql_set_discount_factor(float df);
void ql_set_expl_range(float ini_e, float fin_e);
void ql_set_expl_decay(float d);
void ql_set_expl_factor(float e);
void ql_set_Q_matrix(int s, int a, float val);
void ql_set_Q_vel_matrix(int s, int a, float val);
void ql_set_Tr_matrix(int s, int a, float val);

float ql_get_learning_rate();
float ql_get_discount_factor();
float ql_get_expl_decay();
float ql_get_epsilon();
float ql_get_Q(int s, int a);
float ql_get_Q_vel(int s, int a);
float ql_get_Tr(int s, int a);
int ql_get_nstates();
int ql_get_nactions();
int ql_get_nactions_vel();

void ql_reduce_expl();
float ql_maxQ(int s, int flag);
//float ql_maxQ(int s);
float ql_maxQ_vel(int s);

struct Actions_ID ql_best_action(int s);
int ql_best_action_steer(int s);
int ql_best_action_vel(int s);

struct Actions_ID ql_egreedy_policy(int s);
int ql_egreedy_policy_steer(int s);
int ql_egreedy_policy_vel(int s);

float ql_updateQ(int s, struct Actions_ID a, float r, int snew);
float ql_updateQ_steer(int s, int a, float r, int snew);
float ql_updateQ_vel(int s, int a, float r, int snew);

float ql_lambda_updateQ(int s, int a, float r, int snew);
float updateQ_sarsa(int s, int a, float r, int snew, int anew);

float frand(float xmin, float xmax);
//float evaluate_convergence(float prev_errr, float curr_err);
#endif