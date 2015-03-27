/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 * 
 * @author Example User <mail@example.com>
 */

#define MW_RTOS_DEBUG 1
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

/* Added headers */
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

typedef struct {
    int sigNo;
    sigset_t sigMask;
    double period;
} baseRateInfo_t;

void myWaitForThisEvent(int sigNo);
void myAddBlockForThisEvent(int sigNo);
void myAddHandlerForThisEvent(int sigNo, int sigToBlock[], int numSigToBlock, void *sigHandler);
void myRestoreDefaultHandlerForThisEvent(int sigNo);
void myRTOSInit(double baseRatePeriod, int baseRatePriority, int numSubrates);
void setTaskPeriod(double periodInSeconds, int sigNo);
void schedulerTask(void* arg);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/***********************************************
  Added for Simulink Threads
************************************************/
pthread_attr_t attr;
baseRateInfo_t info;
struct sched_param sp;

#define CHECK_STATUS(status, expStatus, fcn) printf("Call to %s returned status (%d)\n", fcn, status);if (status != expStatus) {printf("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}
#define CHECK_STATUS_NOT(status, errStatus, fcn) printf("Call to %s returned status (%d)\n", fcn, status);if (status == errStatus) {printf("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}

sem_t stopSem;
sem_t termSem;
sem_t baserateTaskSem;
static int baseRateLife = 0;
__EXPORT int px4_simulink_app_main(int argc, char *argv[]);
pthread_t terminateThread;
pthread_t schedulerThread;
pthread_t baseRateThread;
int subratePriority[0];

/* ---------------------------- */
/* Internally visible functions */
/* ---------------------------- */
void setTaskPeriod(double periodInSeconds, int sigNo)
{
  timer_t timerId;
  struct sigevent sev;
  struct itimerspec its;
  long stNanoSec;
  int status;

  /* Create a timer */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = sigNo;
  sev.sigev_value.sival_ptr = &timerId;
  /* only CLOCK_REALTIME is supported in NuttX */
  status = timer_create(CLOCK_REALTIME, &sev, &timerId);
  CHECK_STATUS(status, 0,"timer_create");

  /* Arm real-time scheduling timer */
  stNanoSec = (long)(periodInSeconds * 1e9);
  its.it_value.tv_sec = stNanoSec / 1000000000;
  its.it_value.tv_nsec = stNanoSec % 1000000000;
  its.it_interval.tv_sec = its.it_value.tv_sec;
  its.it_interval.tv_nsec = its.it_value.tv_nsec;
  status = timer_settime(timerId, 0, &its, NULL);
  CHECK_STATUS(status, 0,"timer_settime");
}

void schedulerTask(void* arg)
{
    baseRateInfo_t taskinfo = *((baseRateInfo_t *)arg);
    setTaskPeriod(taskinfo.period, taskinfo.sigNo);    
    while(1) {
			myWaitForThisEvent(taskinfo.sigNo);
      sem_post(&baserateTaskSem);    
    }
}

/* ---------------------------- */
/* Externally visible functions */
/* ---------------------------- */
void myWaitForThisEvent(int sigNo)
{
	int status;
  sigset_t sigMask;
  sigemptyset(&sigMask);
  sigaddset(&sigMask, sigNo);
	status = sigwaitinfo(&sigMask, NULL);
	CHECK_STATUS_NOT(status, -1, "sigwaitinfo");
}

void myAddBlockForThisEvent(int sigNo)
{
		int status;
    sigset_t sigMask;
    sigemptyset(&sigMask);
    sigaddset(&sigMask, sigNo);
    status = pthread_sigmask(SIG_BLOCK, &sigMask, NULL);
    CHECK_STATUS(status, 0, "pthread_sigmask");
}

void myAddHandlerForThisEvent(int sigNo, int sigToBlock[], int numSigToBlock, void *sigHandler)
{
}

void myRestoreDefaultHandlerForThisEvent(int sigNo)
{
}

/**
 * daemon management function.
 */
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/****************************************************************
*****************************************************************/
/* Define the Base Rate Task here */
void baseRateTask(void *arg)
{
  printf("*** baseRateTask initial call ***\n");
  volatile bool noErr;
  noErr = true;
  while (noErr ) {
    printf("*** baseRateTask blocking on baserateTaskSem ***\n");
    sem_wait(&baserateTaskSem);

    /* Get model outputs here */
    noErr = (baseRateLife == 1);
  }                                    /* while */
  sem_post(&termSem);
}

/* Define the exit task here */
void exitTask(void *arg)
{
  sem_post(&stopSem);
}

/* Define the terminate task here */
void terminateTask(void *arg)
{
  printf("**blocking on termSem semaphore in terminateTask**\n");
  sem_wait(&termSem);
  printf("**terminating the model**\n");
  fflush(stdout);
  /* Terminate model */
   sem_post(&stopSem);
}

void myRTOSInit(double baseRatePeriod, int baseRatePriority, int numSubrates)
{
	int i;
	int status;
// 	uid_t euid;
	size_t stackSize;
	unsigned long cpuMask = 0x1;
	unsigned int len = sizeof(cpuMask);
    
	status = sem_init(&termSem, 0, 0);
	CHECK_STATUS(status, 0,"sem_init:termSem");

    // or you can use/check: _POSIX_PRIORITY_SCHEDULING
    // _POSIX_THREAD_PRIORITY_SCHEDULING
 #if !defined (_POSIX_PRIORITY_SCHEDULING)
 	printf("Priority scheduling is NOT supported by your system.\n");
 	printf("The generated code will not run correctly because your\n");
 	printf("model contains multiple rates and uses multi-tasking\n");
 	printf("code generation mode. You can only run the generated code\n");
 	printf("in single-tasking mode in your system. Open\n");
 	printf("Simulation -> Configuration Parameters -> Solver dialog\n");
 	printf("and set \"Tasking mode for periodic sample times\" parameter to SingleTasking.\n");
 	printf("Re-build the Simulink model with the new settings and try executing the generated code again.\n");
 	exit(-1);
 #endif

	/* Need root privileges for real-time scheduling */
    // we are assuming here that we have "Root" privledges on NuttX
// 	euid = geteuid();
// 	if (euid != 0) {
// 	    printf("You must have root privileges to run the generated code because\n");
// 	    printf("generated code requires SCHED_FIFO scheduling class to run correctly.\n");
// 	    printf("Try running the executable with the following command: sudo ./<executable name>\n");
// 	    exit(EXIT_FAILURE);
// 	}

	/* Set scheduling policy of the main thread to SCHED_FIFO */
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	status = sched_setscheduler(0, SCHED_FIFO, &sp);
	CHECK_STATUS(status, 0,"sched_setscheduler");

	/* Create threads executing the Simulink model */
	pthread_attr_init(&attr);
	status = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	CHECK_STATUS(status, 0,"pthread_attr_setinheritsched");
	status = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	CHECK_STATUS(status, 0,"pthread_attr_setschedpolicy");
	//status = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	//CHECK_STATUS(status, 0,"pthread_attr_setdetachstate");

	/* PTHREAD_STACK_MIN is the minimum stack size required to start a thread */
	stackSize = 64 + PTHREAD_STACK_MIN;
	status = pthread_attr_setstacksize(&attr, stackSize);
	CHECK_STATUS(status, 0,"pthread_attr_setstacksize");

#ifdef MW_RTOS_DEBUG
	printf("   stackSize = %d sched_priority = %d\n", stackSize, sp.sched_priority);
    fflush(stdout);
#endif    

	/* Block signal used for timer notification */
	info.period = baseRatePeriod;
// 	info.sigNo = SIGRTMIN;
	info.sigNo = SIGRTMIN;
#ifdef MW_RTOS_DEBUG
	printf("   MW_BASERATE_PERIOD = %8.5f MW_BASERATE_PRIORITY = %d SIGRTMIN = 0x%08X\n", (double)baseRatePeriod, (int)baseRatePriority, SIGRTMIN);
	printf("   Init info.period = %f sigNo = 0x%08X\n", info.period, info.sigNo);
    fflush(stdout);
#endif    
	sigemptyset(&info.sigMask);
    sigaddset(&info.sigMask, info.sigNo);
	myAddBlockForThisEvent(info.sigNo);

#ifdef MW_RTOS_DEBUG
	printf("**creating the base rate task thread**\n");
    fflush(stdout);
#endif    
 	sp.sched_priority = baseRatePriority;
 	status = pthread_attr_setschedparam(&attr, &sp);
 	CHECK_STATUS(status, 0,"pthread_attr_setschedparam");
 #ifdef MW_RTOS_DEBUG
 	printf("**creating the baserate thread before calling pthread_create**\n");
 	printf("   FcnPtr = 0x%08X Info.period %lf sigNo = 0x%08X\n", (void*)baseRateTask, info.period, info.sigNo);
     fflush(stdout);
 #endif    
	baseRateThread = task_create("px4SimBaseTask", baseRatePriority, stackSize, (void *) baseRateTask, (void *) &info);
 	/* status = pthread_create(&baseRateThread, &attr, (void *) baseRateTask, (void *) &info); */
 	// CHECK_STATUS(status, 0,"pthread_create");

#ifdef MW_RTOS_DEBUG
	printf("** Base Rate Task ID = %d : 0x%08X\n", baseRateThread, baseRateThread);
	printf("**creating the terminate thread**\n");
    fflush(stdout);
#endif    
	sp.sched_priority = baseRatePriority;
	status = pthread_attr_setschedparam(&attr, &sp);
	CHECK_STATUS(status, 0,"pthread_attr_setschedparam");
#ifdef MW_RTOS_DEBUG
	printf("**creating the terminate thread before calling pthread_create**\n");
	printf("   FcnPtr = 0x%08X Info.period %8.5f sigNo = 0x%08X\n", (void*)terminateTask, info.period, info.sigNo);
    fflush(stdout);
#endif    
	terminateThread = task_create("px4SimTermTask", baseRatePriority, stackSize, (void *) terminateTask, (void *) &info);
	/* status = pthread_create(&terminateThread, &attr, (void *) terminateTask, (void *) &info); */
	// CHECK_STATUS(status, 0,"pthread_create");

#ifdef MW_RTOS_DEBUG
	printf("** Terminate Task ID = %d : 0x%08X\n", terminateThread, terminateThread);
	printf("**creating the scheduler thread**\n");
    fflush(stdout);
#endif    
 	sp.sched_priority = baseRatePriority;
 	status = pthread_attr_setschedparam(&attr, &sp);
 	CHECK_STATUS(status, 0,"pthread_attr_setschedparam");
 #ifdef MW_RTOS_DEBUG
 	printf("**creating the scheduler thread before calling pthread_create**\n");
 	printf("   FcnPtr = 0x%08X Info.period %lf sigNo = 0x%08X\n", (void*)schedulerTask, info.period, info.sigNo);
     fflush(stdout);
 #endif    
	schedulerThread = task_create("px4SimSchedTask", baseRatePriority, stackSize, (void *) schedulerTask, (void *) &info);
 	/* status = pthread_create(&schedulerThread, &attr, (void *) schedulerTask, (void *) &info); */
 	// CHECK_STATUS(status, 0,"pthread_create");

#ifdef MW_RTOS_DEBUG
	printf("** Scheduler Task ID = %d : 0x%08X\n", schedulerThread, schedulerThread);
	printf("**DONE! creating simulink task threads**\n");   
#endif
	pthread_attr_destroy(&attr);
}

/****************************************************************
*****************************************************************/

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_app_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");
	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("daemon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 px4_daemon_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}
	usage("unrecognized command");
	exit(1);
}

int px4_daemon_thread_main(int argc, char *argv[]) {
	warnx("[daemon] starting\n");
	thread_running = true;
  baseRateLife = 1;
	myRTOSInit(2,40,0);
	while (!thread_should_exit) {
		warnx("Hello daemon!\n");
		sleep(10);
	}
	warnx("[daemon] exiting.\n");
  baseRateLife = 0;
	thread_running = false;
	return 0;
}
