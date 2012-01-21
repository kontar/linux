/*
 * Remote Processor sysfs controls
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Cyril Chemparathy <a0875269@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/remoteproc.h>
#include <linux/stat.h>

#include "remoteproc_internal.h"

#define attr_state_to_rproc(attr)	\
	container_of(attr, struct rproc, attr_state)

static ssize_t show_state(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct rproc *rproc = attr_state_to_rproc(attr);
	size_t count = 0;

	sprintf(buf + count, "%s\n", rproc_get_state_string(rproc->state));

	return count;
}

static ssize_t store_state(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct rproc *rproc = attr_state_to_rproc(attr);
	int len = count, state;

	if (len && (buf[len - 1] == '\n' || buf[len - 1] == '\0'))
		len--;

	if (!len)
		return count;

	for (state = 0; state < RPROC_LAST; state++) {
		if (!strncmp(rproc_get_state_string(state), buf, len))
			break;
	}

	if (state == RPROC_RUNNING && rproc->state != RPROC_RUNNING)
		rproc_boot(rproc);
	else if (state == RPROC_OFFLINE && rproc->state != RPROC_OFFLINE)
		rproc_shutdown(rproc);

	return count;
}

void rproc_create_sysfs(struct rproc *rproc)
{
	struct device_attribute *attr = &rproc->attr_state;

	attr->attr.name = "state";
	attr->attr.mode = S_IRUGO | S_IWUSR;
	attr->show = show_state;
	attr->store = store_state;

	WARN_ON(device_create_file(rproc->dev, attr));
}

void rproc_remove_sysfs(struct rproc *rproc)
{
	device_remove_file(rproc->dev, &rproc->attr_state);
}
