#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>	

#include "ptask.h"
#include "../tlib/tlib.h"

// It creates a custom Task, adds it to the global variables that
// keeps trace of the created tasks and creates a pthread 
// with the parameters provided
int task_create (void* (*task)(void *), int i, int period, int drel, int prio) {
	pthread_attr_t attr;	//thread attributes
	
	struct sched_param prio_task;	//handles thread priority
	int t_ret;
	
	if (i > NT){
		printf("Number of task exceeds NT\n");
		return -1;
	}

	param[i].arg = i;
	param[i].period = period;
	param[i].deadline = drel;
	param[i].priority = prio;
	param[i].dmiss = 0;

	// Initialize Attributes
	pthread_attr_init(&attr);

	// Setting the scheduler
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	// Thread priority
	prio_task.sched_priority = param[i].priority;
	pthread_attr_setschedparam(&attr, &prio_task);

	t_ret = pthread_create(&t_id[i], &attr, task, (void*)(&param[i]));
	if (t_ret != 0) printf("Unable to create thread %d\n", i);

	// Destroy attributes
	pthread_attr_destroy(&attr);
	return t_ret;	
}

// calculates the nex activation time and absolute deadline
void set_activation(int i)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	time_copy(&(param[i].activ_time), t);
	time_copy(&(param[i].dead_time), t);
	time_add_ms(&(param[i].activ_time), param[i].period);
	time_add_ms(&(param[i].dead_time), param[i].period);
}

// suspends the pthread until the next activation time 
// and updates the new absolute deadline and activation time
void wait_for_activation(int i) {
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(param[i].activ_time), NULL);
	 
	time_add_ms(&(param[i].activ_time), param[i].period);
	time_add_ms(&(param[i].dead_time), param[i].period);
}

// waits for the thread specified by the ID to terminate
int wait_for_task(int i) {
	int ret;

	ret = pthread_join(t_id[i], NULL);
	return ret;
}

// Counter for deadline misses
int deadline_miss(int i) {
	struct	timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	if (time_cmp(now, param[i].dead_time) > 0) {
		param[i].dmiss++;
		return 1;
	}

	return 0;
}

// Returns the task's id stored in tp->arg
int get_task_index(void* arg) {
	struct	Task *tpar;	

	tpar = (struct Task *)arg;
	return tpar->arg;
}

// Returns the period of the task stored in tp->arg
int get_task_period(void* arg) {
	struct	Task *tpar;
	tpar = (struct Task *)arg;
	return tpar->period;
}

int get_task_dmiss(int id) {
	return param[id].dmiss;
}