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
 * Added for Simulink Threads
 ************************************************/
baseRateInfo_t info;
struct sched_param sp;

#define CHECK_STATUS(status, expStatus, fcn) warnx("Call to %s returned status (%d)\n", fcn, status);if (status != expStatus) {warnx("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}
#define CHECK_STATUS_NOT(status, errStatus, fcn) warnx("Call to %s returned status (%d)\n", fcn, status);if (status == errStatus) {warnx("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}

static sem_t stopSem;
static sem_t termSem;
static sem_t baserateTaskSem;
static bool baseRateLife = false;
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
    while(baseRateLife) {
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
    int count = 0;
    warnx("*** baseRateTask initial call ***\n");
    volatile bool noErr;
    noErr = true;
    while (noErr) {
        sem_wait(&baserateTaskSem);
        
        /*  This will call the Model's step function */
        warnx("*** baseRateTask blocking on baserateTaskSem [ %d ] ***\n", count++);
        usleep(5000);
        
        /* Get model outputs here */
        noErr = baseRateLife;
    } /* while No Error */
    warnx("+++ baseRateTask Exiting [ %d ] +++\n", count++);
    /* terminate base rate thread - post Terminate Semaphore */
    sem_post(&termSem);
}

/* Define the terminate task here */
void terminateTask(void *arg)
{
    warnx("**blocking on termSem semaphore in terminateTask**\n");
    sem_wait(&termSem);
    warnx("**terminating the model**\n");
    /* Terminate model */
    sem_post(&stopSem);
}

void myRTOSInit(double baseRatePeriod, int baseRatePriority, int numSubrates)
{
    int i;
    int status;
    size_t stackSize;
    
    sched_lock();
    status = sem_init(&termSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:termSem");
    status = sem_init(&stopSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:stopSem");
    status = sem_init(&baserateTaskSem, 0, 0);
    CHECK_STATUS(status, 0,"sem_init:baserateTaskSem");
    
    // or you can use/check: _POSIX_PRIORITY_SCHEDULING
    // _POSIX_THREAD_PRIORITY_SCHEDULING
#if !defined (_POSIX_PRIORITY_SCHEDULING)
    warnx("Priority scheduling is NOT supported by your system.\n");
    warnx("The generated code will not run correctly because your\n");
    warnx("model contains multiple rates and uses multi-tasking\n");
    warnx("code generation mode. You can only run the generated code\n");
    warnx("in single-tasking mode in your system. Open\n");
    warnx("Simulation -> Configuration Parameters -> Solver dialog\n");
    warnx("and set \"Tasking mode for periodic sample times\" parameter to SingleTasking.\n");
    warnx("Re-build the Simulink model with the new settings and try executing the generated code again.\n");
    exit(-1);
#endif
    
    /* Set scheduling policy of the main thread to SCHED_FIFO */
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    status = sched_setscheduler(0, SCHED_FIFO, &sp);
    CHECK_STATUS(status, 0,"sched_setscheduler");
    
    /* PTHREAD_STACK_MIN is the minimum stack size required to start a thread */
    stackSize = 2048 + PTHREAD_STACK_MIN;
    
#ifdef MW_RTOS_DEBUG
    warnx("   stackSize = %d sched_priority = %d\n", stackSize, sp.sched_priority);
#endif
    
    /* Block signal used for timer notification */
    info.period = baseRatePeriod;
    info.sigNo = SIGRTMIN;
#ifdef MW_RTOS_DEBUG
    warnx("   MW_BASERATE_PERIOD = %8.5f MW_BASERATE_PRIORITY = %d SIGRTMIN = 0x%08X\n", (double)baseRatePeriod, (int)baseRatePriority, SIGRTMIN);
    warnx("   Init info.period = %f sigNo = 0x%08X\n", info.period, info.sigNo);
#endif
    sigemptyset(&info.sigMask);
    sigaddset(&info.sigMask, info.sigNo);
    myAddBlockForThisEvent(info.sigNo);
    
    /* Create the Base Rate Task here */
#ifdef MW_RTOS_DEBUG
    warnx("**creating the baserate thread before calling task_create**\n");
#endif
    baseRateThread = task_create("px4SimBaseTask", baseRatePriority, stackSize, (void *) baseRateTask, (void *) &info);
//     baseRateThread = task_spawn_cmd("px4SimBaseTask", SCHED_FIFO, baseRatePriority, stackSize, (void *) baseRateTask, (void *) &info);
    if (baseRateThread > 0) {
        /* configure the scheduler */
        struct sched_param param;
        param.sched_priority = baseRatePriority;
        sched_setscheduler(baseRateThread, SCHED_FIFO, &param);
    }
#ifdef MW_RTOS_DEBUG
    warnx("** Base Rate Task ID = %d : 0x%08X\n", baseRateThread, baseRateThread);
#endif
    
    /* Create the Terminate Task here */
#ifdef MW_RTOS_DEBUG
    warnx("**creating the terminate thread before calling task_create**\n");
#endif
    terminateThread = task_create("px4SimTermTask", baseRatePriority, stackSize, (void *) terminateTask, (void *) &info);
//     terminateThread = task_spawn_cmd("px4SimTermTask", SCHED_FIFO, baseRatePriority, stackSize, (void *) terminateTask, (void *) &info);
    if (terminateThread > 0) {
        /* configure the scheduler */
        struct sched_param param;
        param.sched_priority = baseRatePriority;
        sched_setscheduler(terminateThread, SCHED_FIFO, &param);
    }
#ifdef MW_RTOS_DEBUG
    warnx("** Terminate Task ID = %d : 0x%08X\n", terminateThread, terminateThread);
#endif
    
    /* Create the Scheduler Task here */
#ifdef MW_RTOS_DEBUG
    warnx("**creating the scheduler thread before calling pthread_create**\n");
#endif
    schedulerThread = task_create("px4SimSchedTask", baseRatePriority, stackSize, (void *) schedulerTask, (void *) &info);
//     schedulerThread = task_spawn_cmd("px4SimSchedTask", SCHED_FIFO, baseRatePriority, stackSize, (void *) schedulerTask, (void *) &info);
    if (schedulerThread > 0) {
        /* configure the scheduler */
        struct sched_param param;
        param.sched_priority = baseRatePriority;
        sched_setscheduler(schedulerThread, SCHED_FIFO, &param);
    }
#ifdef MW_RTOS_DEBUG
    warnx("** Scheduler Task ID = %d : 0x%08X\n", schedulerThread, schedulerThread);
    warnx("**DONE! creating simulink task threads**\n");
#endif
    sched_unlock();
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
        daemon_task = task_spawn_cmd("daemon_app_main",
                SCHED_DEFAULT,
                SCHED_PRIORITY_DEFAULT,
                4096,
                px4_daemon_thread_main,
                (argv) ? (const char **)&argv[2] : (const char **)NULL);
                exit(0);
    }
    if (!strcmp(argv[1], "stop")) {
        warnx("[daemon] thread_should_exit is set\n");
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
    baseRateLife = true;
    myRTOSInit(5,30,0);
    while (!thread_should_exit) {
        warnx("Hello daemon!\n");
        sleep(10);
    }
    warnx("[daemon] exiting.\n");
    baseRateLife = false;
    thread_running = false;
    /* wait until the tasks completely finish */
    warnx("[daemon] waiting on stopSem...\n");
    sem_wait(&stopSem);
    return 0;
}
