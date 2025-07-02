#ifndef __DCM_NOTIFIER_H__
#define __DCM_NOTIFIER_H__

#include <linux/notifier.h>

#define EVENT_OF_DOCK_ONLINE	0x01U	// dock在线
#define EVENT_OF_DOCK_OFFLINE	0x02U	// dock离线

/*-------------------------------------------------------------------------*/
/*                           pad event function                            */
// 发送通知
int notify_dock_call_chain(unsigned long val);

// 注册通知处理函数
int register_dock_notifier(struct notifier_block *nb);

// 注销通知处理函数
int unregister_dock_notifier(struct notifier_block *nb);

#endif // __DCM_NOTIFIER_H__

