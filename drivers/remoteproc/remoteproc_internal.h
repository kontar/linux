/*
 * Remote processor framework
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef REMOTEPROC_INTERNAL_H
#define REMOTEPROC_INTERNAL_H

#include <linux/irqreturn.h>

struct rproc;

/* from remoteproc_core.c */
void rproc_release(struct kref *kref);

#ifdef CONFIG_RPMSG
/* from remoteproc_rpmsg.c */
int rproc_add_rpmsg_vdev(struct rproc *rp);
void rproc_remove_rpmsg_vdev(struct rproc *rp);
irqreturn_t rproc_vq_interrupt(struct rproc *rp, int vqid);
#else
static inline int rproc_add_rpmsg_vdev(struct rproc *rp)
{
	return 0;
}

static inline void rproc_remove_rpmsg_vdev(struct rproc *rp)
{
}

static inline irqreturn_t rproc_vq_interrupt(struct rproc *rp, int vqid)
{
	return IRQ_NONE;
}
#endif

#ifdef CONFIG_DEBUG_FS
/* from remoteproc_debugfs.c */
void rproc_remove_trace_file(struct dentry *tfile);
struct dentry *rproc_create_trace_file(const char *name, struct rproc *rproc,
					struct rproc_mem_entry *trace);
void rproc_delete_debug_dir(struct rproc *rproc);
void rproc_create_debug_dir(struct rproc *rproc);
void rproc_init_debugfs(void);
void rproc_exit_debugfs(void);
#else
static inline void rproc_remove_trace_file(struct dentry *tfile)
{
}

static inline struct dentry *
rproc_create_trace_file(const char *name, struct rproc *rproc,
			struct rproc_mem_entry *trace)
{
	return NULL;
}

static inline void rproc_delete_debug_dir(struct rproc *rproc)
{
}

static inline void rproc_create_debug_dir(struct rproc *rproc)
{
}

static inline void rproc_init_debugfs(void)
{
}

static inline void rproc_exit_debugfs(void)
{
}

#endif

#endif /* REMOTEPROC_INTERNAL_H */
