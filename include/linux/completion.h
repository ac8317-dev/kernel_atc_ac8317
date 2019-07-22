#ifndef __LINUX_COMPLETION_H
#define __LINUX_COMPLETION_H

/*
 * (C) Copyright 2001 Linus Torvalds
 *
 * Atomic wait-for-completion handler data structures.
 * See kernel/sched.c for details.
 */

#include <linux/wait.h>

/*
 * struct completion - structure used to maintain state for a "completion"
 *
 * This is the opaque structure used to maintain the state for a "completion".
 * Completions currently use a FIFO to queue threads that have to wait for
 * the "completion" event.
 *
 * See also:  complete(), wait_for_completion() (and friends _timeout,
 * _interruptible, _interruptible_timeout, and _killable), init_completion(),
 * and macros DECLARE_COMPLETION(), DECLARE_COMPLETION_ONSTACK(), and
 * INIT_COMPLETION().
 由一个计数值和一个等待队列组成completion是类似于信号量的东西，用completion.done来表示资源是否可用，
 获取不到的线程会阻塞在completion.wait的等待队列上，直到其它线程释放completion
 这样理解在实现上不错，但我认为completion不是与具体的资源绑定，而是单纯作为一种线程间同步的机制，
 它在概念上要比信号量清晰得多。以后会逐渐看到，线程间事件的同步大多靠completion，而资源临界区的保护大多靠信号量。
 所以说，completion是一种线程间的约会

 Completion是一种轻量级的机制，他允许一个线程告诉另一个线程某个工作已经完成
 */
struct completion {
	unsigned int done;        /*指示等待的事件是否完成。初始化时为0   如果为0,则表示等待的事件未完成。大于0表示等待的事件已经完成  */
	wait_queue_head_t wait;   /* 存放等待该事件完成的进程队列 */ 
};

// 和信号量一样，初始化分为静态初始化和动态初始化两种情况
// 静态初始化：
#define COMPLETION_INITIALIZER(work) \
	{ 0, __WAIT_QUEUE_HEAD_INITIALIZER((work).wait) }

// 动态初始化：
#define COMPLETION_INITIALIZER_ONSTACK(work) \
	({ init_completion(&work); work; })

//可见，两种初始化都将用于同步的done原子量置位了0，后面我们会看到，该变量在wait相关函数中减一，在complete系列函数中加一。


/**
 * DECLARE_COMPLETION - declare and initialize a completion structure
 * @work:  identifier for the completion structure
 *
 * This macro declares and initializes a completion structure. Generally used
 * for static declarations. You should use the _ONSTACK variant for automatic
 * variables.
 */
#define DECLARE_COMPLETION(work) \
	struct completion work = COMPLETION_INITIALIZER(work)

/*
 * Lockdep needs to run a non-constant initializer for on-stack
 * completions - so we use the _ONSTACK() variant for those that
 * are on the kernel stack:
 */
/**
 * DECLARE_COMPLETION_ONSTACK - declare and initialize a completion structure
 * @work:  identifier for the completion structure
 *
 * This macro declares and initializes a completion structure on the kernel
 * stack.
 */
#ifdef CONFIG_LOCKDEP
# define DECLARE_COMPLETION_ONSTACK(work) \
	struct completion work = COMPLETION_INITIALIZER_ONSTACK(work)
#else
# define DECLARE_COMPLETION_ONSTACK(work) DECLARE_COMPLETION(work)
#endif

/**
 * init_completion - Initialize a dynamically allocated completion
 * @x:  completion structure that is to be initialized
 *
 * This inline function will initialize a dynamically created completion
 * structure.
 */
static inline void init_completion(struct completion *x)    //初始化完成量my_completion
{
	x->done = 0;
	init_waitqueue_head(&x->wait);
}

// 该函数等待一个完成量被唤醒;该函数会阻塞调用进程,如果所等待的完成量没有被唤醒,那就一直阻塞下去,而且不会被信号打断;
extern void wait_for_completion(struct completion *);

// 该函数等待一个完成量被唤醒;但是它可以被外部信号打断;
extern int wait_for_completion_interruptible(struct completion *x);

//  该函数等待一个完成量被唤醒;但是它可以被kill信号打断;
extern int wait_for_completion_killable(struct completion *x);

/*****************
该函数等待一个完成量被唤醒;该函数会阻塞调用进程,如果所等待的完成量没有被唤醒,调用进程也不会一直阻塞下去,
而是等待一个指定的超时时间timeout,当超时时间到达时,如果所等待的完成量仍然没有被唤醒,那就返回;超时时间timeout以系统的时钟滴答次数jiffies计算;
*********************/
extern unsigned long wait_for_completion_timeout(struct completion *x,
						   unsigned long timeout);

extern long wait_for_completion_interruptible_timeout(
	struct completion *x, unsigned long timeout);
extern long wait_for_completion_killable_timeout(
	struct completion *x, unsigned long timeout);

//  该函数尝试等待一个完成量被唤醒;不管所等待的完成量是否被唤醒,该函数都会立即返回;
extern bool try_wait_for_completion(struct completion *x);
// 该函数用于检查是否有执行单元阻塞在完成量comp上(是否已经完成),返回0,表示有执行单元被完成量comp阻塞
extern bool completion_done(struct completion *x);

// 该函数只唤醒一个正在等待完成量comp的执行单元  
extern void complete(struct completion *);

// 该函数唤醒所有正在等待同一个完成量comp的执行单元  
extern void complete_all(struct completion *);

/**
 * INIT_COMPLETION - reinitialize a completion structure
 * @x:  completion structure to be reinitialized
 *
 * This macro should be used to reinitialize a completion structure so it can
 * be reused. This is especially important after complete_all() is used.
 */
#define INIT_COMPLETION(x)	((x).done = 0)


#endif
