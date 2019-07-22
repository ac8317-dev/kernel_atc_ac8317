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
 ��һ������ֵ��һ���ȴ��������completion���������ź����Ķ�������completion.done����ʾ��Դ�Ƿ���ã�
 ��ȡ�������̻߳�������completion.wait�ĵȴ������ϣ�ֱ�������߳��ͷ�completion
 ���������ʵ���ϲ���������Ϊcompletion������������Դ�󶨣����ǵ�����Ϊһ���̼߳�ͬ���Ļ��ƣ�
 ���ڸ�����Ҫ���ź��������öࡣ�Ժ���𽥿������̼߳��¼���ͬ����࿿completion������Դ�ٽ����ı�����࿿�ź�����
 ����˵��completion��һ���̼߳��Լ��

 Completion��һ���������Ļ��ƣ�������һ���̸߳�����һ���߳�ĳ�������Ѿ����
 */
struct completion {
	unsigned int done;        /*ָʾ�ȴ����¼��Ƿ���ɡ���ʼ��ʱΪ0   ���Ϊ0,���ʾ�ȴ����¼�δ��ɡ�����0��ʾ�ȴ����¼��Ѿ����  */
	wait_queue_head_t wait;   /* ��ŵȴ����¼���ɵĽ��̶��� */ 
};

// ���ź���һ������ʼ����Ϊ��̬��ʼ���Ͷ�̬��ʼ���������
// ��̬��ʼ����
#define COMPLETION_INITIALIZER(work) \
	{ 0, __WAIT_QUEUE_HEAD_INITIALIZER((work).wait) }

// ��̬��ʼ����
#define COMPLETION_INITIALIZER_ONSTACK(work) \
	({ init_completion(&work); work; })

//�ɼ������ֳ�ʼ����������ͬ����doneԭ������λ��0���������ǻῴ�����ñ�����wait��غ����м�һ����completeϵ�к����м�һ��


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
static inline void init_completion(struct completion *x)    //��ʼ�������my_completion
{
	x->done = 0;
	init_waitqueue_head(&x->wait);
}

// �ú����ȴ�һ�������������;�ú������������ý���,������ȴ��������û�б�����,�Ǿ�һֱ������ȥ,���Ҳ��ᱻ�źŴ��;
extern void wait_for_completion(struct completion *);

// �ú����ȴ�һ�������������;���������Ա��ⲿ�źŴ��;
extern int wait_for_completion_interruptible(struct completion *x);

//  �ú����ȴ�һ�������������;���������Ա�kill�źŴ��;
extern int wait_for_completion_killable(struct completion *x);

/*****************
�ú����ȴ�һ�������������;�ú������������ý���,������ȴ��������û�б�����,���ý���Ҳ����һֱ������ȥ,
���ǵȴ�һ��ָ���ĳ�ʱʱ��timeout,����ʱʱ�䵽��ʱ,������ȴ����������Ȼû�б�����,�Ǿͷ���;��ʱʱ��timeout��ϵͳ��ʱ�ӵδ����jiffies����;
*********************/
extern unsigned long wait_for_completion_timeout(struct completion *x,
						   unsigned long timeout);

extern long wait_for_completion_interruptible_timeout(
	struct completion *x, unsigned long timeout);
extern long wait_for_completion_killable_timeout(
	struct completion *x, unsigned long timeout);

//  �ú������Եȴ�һ�������������;�������ȴ���������Ƿ񱻻���,�ú���������������;
extern bool try_wait_for_completion(struct completion *x);
// �ú������ڼ���Ƿ���ִ�е�Ԫ�����������comp��(�Ƿ��Ѿ����),����0,��ʾ��ִ�е�Ԫ�������comp����
extern bool completion_done(struct completion *x);

// �ú���ֻ����һ�����ڵȴ������comp��ִ�е�Ԫ  
extern void complete(struct completion *);

// �ú��������������ڵȴ�ͬһ�������comp��ִ�е�Ԫ  
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
