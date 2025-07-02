/*
 * 特别声明：本技术材料受《中华人民共和国著作权法》、《计算机软件保护条例》等法律、法规、行政
 *  规章以及有关国际条约的保护，赞同科技股份有限公司享有知识产权、保留一切权利并视其为技术秘密。未经本公司书
 *  面许可，任何人不得擅自（包括但不限于：以非法的方式复制、传播、展示、镜像、上载、下载）使用，
 *  不得向第三方泄露、透露、披露。否则，本公司将依法追究侵权者的法律责任。特此声明！
 *
 *  Special Declaration: These technical material reserved as the technical secrets by AGREE
 *  TECHNOLOGY have been protected by the "Copyright Law" "ordinances on Protection of Computer
 *  Software" and other relevant administrative regulations and international treaties. Without
 *  the written permission of the Company, no person may use (including but not limited to the
 *  illegal copy, distribute, display, image, upload, and download) and disclose the above
 *  technical documents to any third party. Otherwise, any infringer shall afford the legal
 *  liability to the company.
 *
 *  Copyright 1993-2023 Agree Tech. All rights reserved.
 *
 */

/**
 * Copyright (C), 1993-2023, 赞同科技股份有限公司
 * FileName: dcm_notifier.c
 * Author:   ouyangyunsheng
 * Reviewer: ouyangyunsheng
 * Date:     2023/5/9 下午4:38
 * Description: DCM Notifier
 * Since:    3.2
 * History:
 * <author>          <time>          <version>          <desc>
 * 作者姓名           修改时间           版本号              描述
 */

#include "dcm_notifier.h"

/*-------------------------------------------------------------------------*/
/*                            DCM notifier                                 */

BLOCKING_NOTIFIER_HEAD(dock_nofifier_list);

/*-------------------------------------------------------------------------*/
/*                           pad event function                            */
// 发送通知
int notify_dock_call_chain(unsigned long val)
{
	return blocking_notifier_call_chain(&dock_nofifier_list, val, NULL);
}
EXPORT_SYMBOL(notify_dock_call_chain);

// 注册通知处理函数
int register_dock_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_nofifier_list, nb);
}
EXPORT_SYMBOL(register_dock_notifier);

// 注销通知处理函数
int unregister_dock_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_nofifier_list, nb);
}
EXPORT_SYMBOL(unregister_dock_notifier);

