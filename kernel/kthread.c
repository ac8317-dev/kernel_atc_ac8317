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
 kthread_should_stop()����should_stop��־ �����ڴ������̼߳�������־���������Ƿ��˳�
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
kthreadd������idleͨ��kernel_thread��������ʼ���������ں˿ռ�, ���������ں��̵߳ĵ��Ⱥ͹���
����������ǹ���͵��������ں��߳�kernel_thread, ��ѭ��ִ��һ��kthreadd�ĺ�����
�ú��������þ�������kthread_create_listȫ��������ά����kthread, �����ǵ���kernel_thread�������ں��̻߳ᱻ���뵽�������У�
������е��ں��̶߳���ֱ�ӻ��߼�ӵ���kthreaddΪ�������������ں���ͨ��kernel_create����������ʽ����һ���ں��߳�, 
Ȼ��kthreadd�ں��̱߳�����,����ִ���ں��̴߳����������������µ��߳̽�ִ��kthread����, ��ɴ���������������Ϻ��ó�CPU��
����µ��ں��̲߳����������У���Ҫ�ֹ� wake up, �����Ѻ�ִ���Լ���������������
1.�κ�һ���ں��߳���ڶ��� kthread()
2.ͨ�� kthread_create() �������ں��̲߳����������У���Ҫ�ֹ� wake up.
3.ͨ�� kthread_create() �������ں��߳��п��ܲ���ִ����Ӧ�̺߳���threadfn��ֱ���˳�

*********/
static int kthread(void *_create)
{
	/* Copy data: it's on kthread's stack
	create ָ�� kthread_create_info �е� kthread_create_info
	*/

	 /*  �µ��̴߳�����Ϻ�ִ�еĺ��� */
	struct kthread_create_info *create = _create;
	int (*threadfn)(void *data) = create->threadfn;
	 /*  �µ��߳�ִ�еĲ���  */
	void *data = create->data;
	struct kthread self;
	int ret;

	self.should_stop = 0;  //  kthread���ñ�־should_stop��
	self.data = data;
	init_completion(&self.exited);
	current->vfork_done = &self.exited;

	/* OK, tell user we're spawned, wait for stop or wakeup  ��������״̬Ϊ TASK_UNINTERRUPTIBLE  */
	__set_current_state(TASK_UNINTERRUPTIBLE);
	 /*  current ��ʾ��ǰ�´����� thread �� task_struct �ṹ  */
	create->result = current;
	complete(&create->done);
	/*  �����̴߳������ ,  ִ�������л����ó� CPU  */
	schedule();

/*******
�̴߳������:
������ thread �Ľ��ָ̻����� kthread_create() ���ҷ����´����̵߳����������� 
�´������߳�����ִ���� schedule() ���ȣ���ʱ��û��ִ��.
ֱ������ʹ��wake_up_process(p);�����´������߳��̱߳����Ѻ�, �����ִ��threadfn(data)
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
��ʵ���ǵ������ȹ���һ���ٵ�������ִ�л����������� do_fork()
    ���ؽ��� id, ��������߳�ִ�� kthread ����
*/
	pid = kernel_thread(kthread, create, CLONE_FS | CLONE_FILES | SIGCHLD);  //����kernel_thread������һ���µĽ��̣��ý��̵��ں˺���Ϊkthread �������ں��߳�ִ�е��¼�kthread
		create->result = ERR_PTR(pid);
		complete(&create->done);
		
/**********
��ʱ�ص� kthreadd thread,��������˽��̵Ĵ��������ѭ������� kthread_create_list ����
���Ϊ�գ��� kthreadd �ں��̻߳�˯��ȥ��ô�������ڻ������ǵĲ��� �������ں���ͨ��kernel_create����������ʽ����һ���ں��߳�, 
Ȼ��kthreadd�ں��̱߳�����,����ִ���ں��̴߳������������������������������߳�
1.kthreadd�Ѿ��������ʹ��(����ִ�������Ĵ�������)��˯��
2.����kthreadd���߳������´������̻߳�û�д�����϶�����˯�� (�� kthread_create������)
3.�´������߳��Ѿ���������kthread���������ڻ�����������û�������Ի�û�����մ������.
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
 ��������ᴴ��һ����Ϊnamefmt���ں��̣߳�����̸߳մ���ʱ��������ִ�У�
 Ҫ�ȵ�����kthread_create() ���ص�task_structָ�봫��wake_up_process()��
 Ȼ��ͨ���˺��������̡߳����ǿ���creat�ṹ�壬���ǽ�����Ĳ���������������threadfn����������Ǵ��������к�����
 ��ʹ�������ǿ����ڴ˺����е���kthread_should_stop��������kthread_stop()�����������߳�
 * Returns a task_struct or ERR_PTR(-ENOMEM).
 */
struct task_struct *kthread_create_on_node(int (*threadfn)(void *data),
					   void *data,
					   int node,
					   const char namefmt[],
					   ...)
{
	struct kthread_create_info create;

	create.threadfn = threadfn; //�߳�ִ�к���  
	create.data = data;  //�߳�ִ�к����Ĳ���  
	create.node = node;  //NUMAϵͳ�ϻ��õ�,���ﲻ����
	init_completion(&create.done); //completion�ṹ,һ�����ͨ�����жϸ��̵߳Ĵ����Ƿ����

	spin_lock(&kthread_create_lock);
	list_add_tail(&create.list, &kthread_create_list);
	spin_unlock(&kthread_create_lock);

	wake_up_process(kthreadd_task);     //����kthreadd_task�ں��߳�
	wait_for_completion(&create.done);  //����˯��,�ȴ��̴߳��������.  

	if (!IS_ERR(create.result)) {
		static const struct sched_param param = { .sched_priority = 0 };
		va_list args;

		va_start(args, namefmt);
		vsnprintf(create.result->comm, sizeof(create.result->comm),    //�����߳�����  
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
  �߳�һ�����������󣬻�һֱ���У����Ǹ��߳���������do_exit���������������Ľ��̵���kthread_stop�����������̵߳�����
  ���ֱ��ʹ��do_exitֱ���˳��߳���ôkthread_stop�����յ�����źŽ�һֱ�ȴ���ȥ
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
��forѭ���У��������kthread_create_list��һ�����������schedule���Ⱥ�����
��Ϊ��ǰ�Ѿ����ý��̵�״̬����ΪTASK_INTERRUPTIBLE������schedule�ĵ��ý���ʹ��ǰ���̽���˯�ߡ�
���kthread_create_list��Ϊ�գ������whileѭ�����ڸ�ѭ�����л������kthread_create_list�б�
���ڸ��б��ϵ�ÿһ��entry������õ���Ӧ������Ϊstruct kthread_create_info�Ľڵ��ָ��create.
Ȼ������kthread_create_list��ɾ��create��Ӧ���б�entry����������createָ��Ϊ��������create_kthread(create)
************/
int kthreadd(void *unused)
{
	struct task_struct *tsk = current;

	/* Setup a clean context for our children to inherit. */
	set_task_comm(tsk, "kthreadd");
	ignore_signals(tsk);
	set_cpus_allowed_ptr(tsk, cpu_all_mask);         //  ����kthreadd������CPU������
	set_mems_allowed(node_states[N_HIGH_MEMORY]);

	current->flags |= PF_NOFREEZE;

	for (;;) {

		/*  ���Ƚ��߳�״̬����Ϊ TASK_INTERRUPTIBLE, �����ǰ
            û��Ҫ�������߳����������� CPU ��ɵ���.�˽��̱�Ϊ����̬*/
		set_current_state(TASK_INTERRUPTIBLE);
		if (list_empty(&kthread_create_list))    //  û����Ҫ�������ں��߳�
			schedule();                            //   ʲôҲ����, ִ��һ�ε���, �ó�CPU

			/*  ���е��˱�ʾ kthreadd �̱߳�����(�������ǵ�ǰ)
            ���ý�������״̬Ϊ TASK_RUNNING */
		__set_current_state(TASK_RUNNING);

		spin_lock(&kthread_create_lock);  //  ����,
		while (!list_empty(&kthread_create_list)) {
			struct kthread_create_info *create;
			
			  /*  ��������ȡ�� kthread_create_info �ṹ�ĵ�ַ�����������Ѿ���ɲ������(��
			  kthread_create_info �ṹ�е� list ��Ա�ӵ������У���ʱ���ݳ�Ա list ��ƫ��
			  ��� create)  */
			create = list_entry(kthread_create_list.next,
					    struct kthread_create_info, list);
			    /* ��ɴ��������������ɾ�� */
			list_del_init(&create->list);
			 /* ��������̵߳Ĵ��� */
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
