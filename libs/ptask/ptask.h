#include <pthread.h>

#ifndef PTASK_H_
#define PTASK_H_

#define NT 10 						// max number of tasks

// Custom structure to handle a real time task
struct Task {
	int		arg;					// task id
	long	wcet;					// worst case execution time
	int 	period; 				// task period in millisecods
	int 	deadline;				// relative deadline (ms)
	int 	priority;				// priority (from 0 to 99)
	int		dmiss;					// deadline miss counter
	struct 	timespec activ_time; 	// next activation time
	struct 	timespec dead_time;		// absolute deadline
};

struct Task param[NT];				// keeps record of active tasks
	
pthread_t t_id[NT];	 				// pthread IDs (task)

// Function prototypes

int 	task_create (void* (*task)(void *), int i, int period, int drel, int prio);
void 	set_activation(int i);
void	wait_for_activation(int i);
void	wait_for_task(int i);
int		deadline_miss(int i);
int		get_task_index(void* arg);
int		get_task_period(void* arg);

#endif /* PTASK_H_ */