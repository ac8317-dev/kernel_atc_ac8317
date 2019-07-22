/* Kernel thread helper functions.
 *   Copyright (C) 2004 IBM Corporation, Rusty Russell.
 *
 * Creation is done via kthreadd, so that we get a clean environment
 * even if we're invoked from userspace (think modprobe, hotplug cpu,
 * etc.).
 */
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/cpuset.h>
#include <linux/unistd.h>
#include <linux/file.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <trace/events/sched.h>

static DEFINE_SPINLOCK(kthread_create_lock);
static LIST_HEAD(kthread_create_list);
struct task_struct *kthreadd_task;

struct kthread_create_info
{
	/* Information passed to kthread() from kthreadd. */
	int (*threadfn)(void *data);
	void *data;
	int node;

	/* Result passed back to kthread_create() from kthreadd. */
	struct task_struct *result;
	struct completion done;

	struct list_head list;
};

struct kthread {
	int should_stop;
	void *data;
	struct completion exited;
};

#define to_kthread(tsk)	\
	container_of((tsk)->vfork_done, struct kthread, exited)

/**
 * kthread_should_stop - should this kthread return now?
 *
 * When someone calls kthread_stop() on your kthread, it will be woken
 * and this will return true.  You should then return, and your return
 * value will be passed through to kthread_stop().
 kthread_should_stop()返回should_stop标志 它用于创建的线程检查结束标志，并决定是否退出
 */
int kthread_should_stop(void)
{
	return to_kthread(current)->should_stop;
}
EXPORT_SYMBOL(kthread_should_stop);

/**
 * kthread_freezable_should_stop - should this freezable kthread return now?
 * @was_frozen: optional out parameter, indicates whether %current was frozen
 *
 * kthread_should_stop() for freezable kthreads, which will enter
 * refrigerator if necessary.  This function is safe from kthread_stop() /
 * freezer deadlock and freezable kthreads should use this function instead
 * of calling try_to_freeze() directly.
 */
bool kthread_freezable_should_stop(bool *was_frozen)
{
	bool frozen = false;

	might_sleep();

	if (unlikely(freezing(current)))
		frozen = __refrigerator(true);

	if (was_frozen)
		*was_frozen = frozen;

	return kthread_should_stop();
}
EXPORT_SYMBOL_GPL(kthread_freezable_should_stop);

/**
 * kthread_data - return data value specified on kthread creation
 * @task: kthread task in question
 *
 * Return the data value specified when kthread @task was created.
 * The caller is responsible for ensuring the validity of @task when
 * calling this function.
 */
void *kthread_data(struct task_struct *task)
{
	return to_kthread(task)->data;
}

/*********
kthreadd进程由idle通过kernel_thread创建，并始终运行在内核空间, 负责所有内核线程的调度和管理，
它的任务就是管理和调度其他内核线程kernel_thread, 会循环执行一个kthreadd的函数，
该函数的作用就是运行kthread_create_list全局链表中维护的kthread, 当我们调用kernel_thread创建的内核线程会被加入到此链表中，
因此所有的内核线程都是直接或者间接的以kthreadd为父进程我们在内核中通过kernel_create或者其他方式创建一个内核线程, 
然后kthreadd内核线程被唤醒,　来执行内核线程创建的真正工作，新的线程将执行kthread函数, 完成创建工作，创建完毕后让出CPU，
因此新的内核线程不会立刻运行．需要手工 wake up, 被唤醒后将执行自己的真正工作函数
1.任何一个内核线程入口都是 kthread()
2.通过 kthread_create() 创建的内核线程不会立刻运行．需要手工 wake up.
3.通过 kthread_create() 创建的内核线程有可能不会执行相应线程函数threadfn而直接退出

*********/
static int kthread(void *_create)
{
	/* Copy data: it's on kthread's stack
	create 指向 kthread_create_info 中的 kthread_create_info
	*/

	 /*  新的线程创建完毕后执行的函数 */
	struct kthread_create_info *create = _create;
	int (*threadfn)(void *data) = create->threadfn;
	 /*  新的线程执行的参数  */
	void *data = create->data;
	struct kthread self;
	int ret;

	self.should_stop = 0;  //  kthread设置标志should_stop，
	self.data = data;
	init_completion(&self.exited);
	current->vfork_done = &self.exited;

	/* OK, tell user we're spawned, wait for stop or wakeup  设置运行状态为 TASK_UNINTERRUPTIBLE  */
	__set_current_state(TASK_UNINTERRUPTIBLE);
	 /*  current 表示当前新创建的 thread 的 task_struct 结构  */
	create->result = current;
	complete(&create->done);
	/*  至此线程创建完毕 ,  执行任务切换，让出 CPU  */
	schedule();

/*******
线程创建完毕:
创建新 thread 的进程恢复运行 kthread_create() 并且返回新创建线程的任务描述符 
新创建的线程由于执行了 schedule() 调度，此时并没有执行.
直到我们使用wake_up_process(p);唤醒新创建的线程线程被唤醒后, 会接着执行threadfn(data)
********/
	ret = -EINTR;
	if (!self.should_stop)
		ret = threadfn(data);

	/* we can't just return, we must preserve "self" on stack */
	do_exit(ret);
}

/* called from do_fork() to get node information for about to be created task */
int tsk_fork_get_node(struct task_struct *tsk)
{
#ifdef CONFIG_NUMA
	if (tsk == kthreadd_task)
		return tsk->pref_node_fork;
#endif
	return numa_node_id();
}

static void create_kthread(struct kthread_create_info *create)
{
	int pid;

#ifdef CONFIG_NUMA
	current->pref_node_fork = create->node;
#endif
	/* We want our own signal handler (we take no signals by default). 
其实就是调用首先构造一个假的上下文执行环境，最后调用 do_fork()
    返回进程 id, 创建后的线程执行 kthread 函数
*/
	pid = kernel_thread(kthread, create, CLONE_FS | CLONE_FILES | SIGCHLD);  //调用kernel_thread来生成一个新的进程，该进程的内核函数为kthread 创建的内核线程执行的事件kthread
		create->result = ERR_PTR(pid);
		complete(&create->done);
		
/**********
此时回到 kthreadd thread,它在完成了进程的创建后继续循环，检查 kthread_create_list 链表，
如果为空，则 kthreadd 内核线程昏睡过去那么我们现在回想我们的操作 我们在内核中通过kernel_create或者其他方式创建一个内核线程, 
然后kthreadd内核线程被唤醒,　来执行内核线程创建的真正工作，于是这里有三个线程
1.kthreadd已经光荣完成使命(接手执行真正的创建工作)，睡眠
2.唤醒kthreadd的线程由于新创建的线程还没有创建完毕而继续睡眠 (在 kthread_create函数中)
3.新创建的线程已经正在运行kthread，但是由于还有其它工作没有做所以还没有最终创建完成.
***************/
	}
}

/**
 * kthread_create_on_node - create a kthread.
 * @threadfn: the function to run until signal_pending(current).
 * @data: data ptr for @threadfn.
 * @node: memory node number.
 * @namefmt: printf-style name for the thread.
 *
 * Description: This helper function creates and names a kernel
 * thread.  The thread will be stopped: use wake_up_process() to start
 * it.  See also kthread_run().
 *
 * If thread is going to be bound on a particular cpu, give its node
 * in @node, to get NUMA affinity for kthread stack, or else give -1.
 * When woken, the thread will run @threadfn() with @data as its
 * argument. @threadfn() can either call do_exit() directly if it is a
 * standalone thread for which no one will call kthread_stop(), or
 * return when 'kthread_should_stop()' is true (which means
 * kthread_stop() has been called).  The return value should be zero
 * or a negative error number; it will be passed to kthread_stop().
 *
 这个函数会创建一个名为namefmt的内核线程，这个线程刚创建时不会马上执行，
 要等到它将kthread_create() 返回的task_struct指针传给wake_up_process()，
 然后通过此函数运行线程。我们看到creat结构体，我们将传入的参数付给了它，而threadfn这个函数就是创建的运行函数。
 在使用中我们可以在此函数中调用kthread_should_stop（）或者kthread_stop()函数来结束线程
 * Returns a task_struct or ERR_PTR(-ENOMEM).
 */
struct task_struct *kthread_create_on_node(int (*threadfn)(void *data),
					   void *data,
					   int node,
					   const char namefmt[],
					   ...)
{
	struct kthread_create_info create;

	create.threadfn = threadfn; //线程执行函数  
	create.data = data;  //线程执行函数的参数  
	create.node = node;  //NUMA系统上会用到,这里不介绍
	init_completion(&create.done); //completion结构,一会儿会通过它判断该线程的创建是否完成

	spin_lock(&kthread_create_lock);
	list_add_tail(&create.list, &kthread_create_list);
	spin_unlock(&kthread_create_lock);

	wake_up_process(kthreadd_task);     //唤醒kthreadd_task内核线程
	wait_for_completion(&create.done);  //进入睡眠,等待线程创建的完成.  

	if (!IS_ERR(create.result)) {
		static const struct sched_param param = { .sched_priority = 0 };
		va_list args;

		va_start(args, namefmt);
		vsnprintf(create.result->comm, sizeof(create.result->comm),    //设置线程名字  
			  namefmt, args);
		va_end(args);
		/*
		 * root may have changed our (kthreadd's) priority or CPU mask.
		 * The kernel thread should not inherit these properties.
		 */
		sched_setscheduler_nocheck(create.result, SCHED_NORMAL, &param);
		set_cpus_allowed_ptr(create.result, cpu_all_mask);
	}
	return create.result;
}
EXPORT_SYMBOL(kthread_create_on_node);

/**
 * kthread_bind - bind a just-created kthread to a cpu.
 * @p: thread created by kthread_create().
 * @cpu: cpu (might not be online, must be possible) for @k to run on.
 *
 * Description: This function is equivalent to set_cpus_allowed(),
 * except that @cpu doesn't need to be online, and the thread must be
 * stopped (i.e., just returned from kthread_create()).
 */
void kthread_bind(struct task_struct *p, unsigned int cpu)
{
	/* Must have done schedule() in kthread() before we set_task_cpu */
	if (!wait_task_inactive(p, TASK_UNINTERRUPTIBLE)) {
		WARN_ON(1);
		return;
	}

	/* It's safe because the task is inactive. */
	do_set_cpus_allowed(p, cpumask_of(cpu));
	p->flags |= PF_THREAD_BOUND;
}
EXPORT_SYMBOL(kthread_bind);

/**
 * kthread_stop - stop a thread created by kthread_create().
 * @k: thread created by kthread_create().
 *
 * Sets kthread_should_stop() for @k to return true, wakes it, and
 * waits for it to exit. This can also be called after kthread_create()
 * instead of calling wake_up_process(): the thread will exit without
 * calling threadfn().
 *
 * If threadfn() may call do_exit() itself, the caller must ensure
 * task_struct can't go away.
 *
 * Returns the result of threadfn(), or %-EINTR if wake_up_process()
 * was never called.
  线程一旦启动起来后，会一直运行，除非该线程主动调用do_exit函数，或者其他的进程调用kthread_stop函数，结束线程的运行
  如果直接使用do_exit直接退出线程那么kthread_stop不会收到完成信号将一直等待下去
 */
int kthread_stop(struct task_struct *k)
{
	struct kthread *kthread;
	int ret;

	trace_sched_kthread_stop(k);
	get_task_struct(k);

	kthread = to_kthread(k);
	barrier(); /* it might have exited */
	if (k->vfork_done != NULL) {
		kthread->should_stop = 1;    
		wake_up_process(k);
		wait_for_completion(&kthread->exited);   //  
	}
	ret = k->exit_code;

	put_task_struct(k);
	trace_sched_kthread_stop_ret(ret);

	return ret;
}
EXPORT_SYMBOL(kthread_stop);




/*********
在for循环中，如果发现kthread_create_list是一空链表，则调用schedule调度函数，
因为此前已经将该进程的状态设置为TASK_INTERRUPTIBLE，所以schedule的调用将会使当前进程进入睡眠。
如果kthread_create_list不为空，则进入while循环，在该循环体中会遍历该kthread_create_list列表，
对于该列表上的每一个entry，都会得到对应的类型为struct kthread_create_info的节点的指针create.
然后函数在kthread_create_list中删除create对应的列表entry，接下来以create指针为参数调用create_kthread(create)
************/
int kthreadd(void *unused)
{
	struct task_struct *tsk = current;

	/* Setup a clean context for our children to inherit. */
	set_task_comm(tsk, "kthreadd");
	ignore_signals(tsk);
	set_cpus_allowed_ptr(tsk, cpu_all_mask);         //  允许kthreadd在任意CPU上运行
	set_mems_allowed(node_states[N_HIGH_MEMORY]);

	current->flags |= PF_NOFREEZE;

	for (;;) {

		/*  首先将线程状态设置为 TASK_INTERRUPTIBLE, 如果当前
            没有要创建的线程则主动放弃 CPU 完成调度.此进程变为阻塞态*/
		set_current_state(TASK_INTERRUPTIBLE);
		if (list_empty(&kthread_create_list))    //  没有需要创建的内核线程
			schedule();                            //   什么也不做, 执行一次调度, 让出CPU

			/*  运行到此表示 kthreadd 线程被唤醒(就是我们当前)
            设置进程运行状态为 TASK_RUNNING */
		__set_current_state(TASK_RUNNING);

		spin_lock(&kthread_create_lock);  //  加锁,
		while (!list_empty(&kthread_create_list)) {
			struct kthread_create_info *create;
			
			  /*  从链表中取得 kthread_create_info 结构的地址，在上文中已经完成插入操作(将
			  kthread_create_info 结构中的 list 成员加到链表中，此时根据成员 list 的偏移
			  获得 create)  */
			create = list_entry(kthread_create_list.next,
					    struct kthread_create_info, list);
			    /* 完成穿件后将其从链表中删除 */
			list_del_init(&create->list);
			 /* 完成真正线程的创建 */
			spin_unlock(&kthread_create_lock);

			create_kthread(create);

			spin_lock(&kthread_create_lock);
		}
		spin_unlock(&kthread_create_lock);
	}

	return 0;
}

void __init_kthread_worker(struct kthread_worker *worker,
				const char *name,
				struct lock_class_key *key)
{
	spin_lock_init(&worker->lock);
	lockdep_set_class_and_name(&worker->lock, key, name);
	INIT_LIST_HEAD(&worker->work_list);
	worker->task = NULL;
}
EXPORT_SYMBOL_GPL(__init_kthread_worker);

/**
 * kthread_worker_fn - kthread function to process kthread_worker
 * @worker_ptr: pointer to initialized kthread_worker
 *
 * This function can be used as @threadfn to kthread_create() or
 * kthread_run() with @worker_ptr argument pointing to an initialized
 * kthread_worker.  The started kthread will process work_list until
 * the it is stopped with kthread_stop().  A kthread can also call
 * this function directly after extra initialization.
 *
 * Different kthreads can be used for the same kthread_worker as long
 * as there's only one kthread attached to it at any given time.  A
 * kthread_worker without an attached kthread simply collects queued
 * kthread_works.
 */
int kthread_worker_fn(void *worker_ptr)
{
	struct kthread_worker *worker = worker_ptr;
	struct kthread_work *work;

	WARN_ON(worker->task);
	worker->task = current;
repeat:
	set_current_state(TASK_INTERRUPTIBLE);	/* mb paired w/ kthread_stop */

	if (kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
		spin_lock_irq(&worker->lock);
		worker->task = NULL;
		spin_unlock_irq(&worker->lock);
		return 0;
	}

	work = NULL;
	spin_lock_irq(&worker->lock);
	if (!list_empty(&worker->work_list)) {
		work = list_first_entry(&worker->work_list,
					struct kthread_work, node);
		list_del_init(&work->node);
	}
	worker->current_work = work;
	spin_unlock_irq(&worker->lock);

	if (work) {
		__set_current_state(TASK_RUNNING);
		work->func(work);
	} else if (!freezing(current))
		schedule();

	try_to_freeze();
	goto repeat;
}
EXPORT_SYMBOL_GPL(kthread_worker_fn);

/* insert @work before @pos in @worker */
static void insert_kthread_work(struct kthread_worker *worker,
			       struct kthread_work *work,
			       struct list_head *pos)
{
	lockdep_assert_held(&worker->lock);

	list_add_tail(&work->node, pos);
	work->worker = worker;
	if (likely(worker->task))
		wake_up_process(worker->task);
}

/**
 * queue_kthread_work - queue a kthread_work
 * @worker: target kthread_worker
 * @work: kthread_work to queue
 *
 * Queue @work to work processor @task for async execution.  @task
 * must have been created with kthread_worker_create().  Returns %true
 * if @work was successfully queued, %false if it was already pending.
 */
bool queue_kthread_work(struct kthread_worker *worker,
			struct kthread_work *work)
{
	bool ret = false;
	unsigned long flags;

	spin_lock_irqsave(&worker->lock, flags);
	if (list_empty(&work->node)) {
		insert_kthread_work(worker, work, &worker->work_list);
		ret = true;
	}
	spin_unlock_irqrestore(&worker->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(queue_kthread_work);

struct kthread_flush_work {
	struct kthread_work	work;
	struct completion	done;
};

static void kthread_flush_work_fn(struct kthread_work *work)
{
	struct kthread_flush_work *fwork =
		container_of(work, struct kthread_flush_work, work);
	complete(&fwork->done);
}

/**
 * flush_kthread_work - flush a kthread_work
 * @work: work to flush
 *
 * If @work is queued or executing, wait for it to finish execution.
 */
void flush_kthread_work(struct kthread_work *work)
{
	struct kthread_flush_work fwork = {
		KTHREAD_WORK_INIT(fwork.work, kthread_flush_work_fn),
		COMPLETION_INITIALIZER_ONSTACK(fwork.done),
	};
	struct kthread_worker *worker;
	bool noop = false;

retry:
	worker = work->worker;
	if (!worker)
		return;

	spin_lock_irq(&worker->lock);
	if (work->worker != worker) {
		spin_unlock_irq(&worker->lock);
		goto retry;
	}

	if (!list_empty(&work->node))
		insert_kthread_work(worker, &fwork.work, work->node.next);
	else if (worker->current_work == work)
		insert_kthread_work(worker, &fwork.work, worker->work_list.next);
	else
		noop = true;

	spin_unlock_irq(&worker->lock);

	if (!noop)
		wait_for_completion(&fwork.done);
}
EXPORT_SYMBOL_GPL(flush_kthread_work);

/**
 * flush_kthread_worker - flush all current works on a kthread_worker
 * @worker: worker to flush
 *
 * Wait until all currently executing or pending works on @worker are
 * finished.
 */
void flush_kthread_worker(struct kthread_worker *worker)
{
	struct kthread_flush_work fwork = {
		KTHREAD_WORK_INIT(fwork.work, kthread_flush_work_fn),
		COMPLETION_INITIALIZER_ONSTACK(fwork.done),
	};

	queue_kthread_work(worker, &fwork.work);
	wait_for_completion(&fwork.done);
}
EXPORT_SYMBOL_GPL(flush_kthread_worker);
