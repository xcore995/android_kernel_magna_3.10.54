/*
 * Example events provider
 *
 * Copyright (C) ARM Limited 2010-2014. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Similar entries to those below must be present in the events.xml file.
 * To add them to the events.xml, create an events-mmap.xml with the
 * following contents and rebuild gatord:
 *
 * <category name="MT6732Load">
 *   	<event counter="load_cnt0" title="MT6732 Load" name="GPU Load" display="maximum" class="absolute" percentage="yes" description="Frequency setting of GPU Load"/>
 * </category>
 *
 * When adding custom events, be sure to do the following:
 * - add any needed .c files to the gator driver Makefile
 * - call gator_events_install in the events init function
 * - add the init function to GATOR_EVENTS_LIST in gator_main.c
 * - add a new events-*.xml file to the gator daemon and rebuild
 *
 * Troubleshooting:
 * - verify the new events are part of events.xml, which is created when building the daemon
 * - verify the new events exist at /dev/gator/events/ once gatord is launched
 * - verify the counter name in the XML matches the name at /dev/gator/events
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/ratelimit.h>

#include "gator.h"

enum {
	GPU_LOAD = 0,
	LOAD_COUNTERS_NUM,
};

static int load_global_enabled;

static struct {
	unsigned long enabled;
	unsigned long key;
} load_counters[LOAD_COUNTERS_NUM];

static int load_buffer[LOAD_COUNTERS_NUM * 2];

/* Adds clock_cntX directories and enabled, event, and key files to /dev/gator/events */
static int gator_events_load_create_files(struct super_block *sb,
					     struct dentry *root)
{
	int i;

	for (i = 0; i < LOAD_COUNTERS_NUM; i++) {
		char buf[16];
		struct dentry *dir;

		snprintf(buf, sizeof(buf), "load_cnt%d", i);
		dir = gatorfs_mkdir(sb, root, buf);
		if (WARN_ON(!dir))
			return -1;
		gatorfs_create_ulong(sb, dir, "enabled",
				     &load_counters[i].enabled);
		gatorfs_create_ro_ulong(sb, dir, "key",
					&load_counters[i].key);
	}

	return 0;
}

static int gator_events_load_start(void)
{
	int i;

	load_global_enabled = 0;
	for (i = 0; i < LOAD_COUNTERS_NUM; i++) {
		if (load_counters[i].enabled) {
			load_global_enabled = 1;
			break;
		}
	}

	return 0;
}

static void gator_events_load_stop(void)
{
}

/* This function is purpose to mali profiling  */
//extern unsigned int kbasep_get_gl_utilization(void);
extern int g_current_sample_gl_utilization;


static int get_mali_gpu_load(void)
{
//    return kbasep_get_gl_utilization();
      return g_current_sample_gl_utilization;
}

/* This function clock setting value, generating values of fancy */

static int get_load_value(int load_type)
{
	int result = 0;

	switch (load_type) {
	case GPU_LOAD:		/* GPU LOAD Information */
		{
			result = get_mali_gpu_load();
		}
		break;		
	default:		/* None Clock Information */
		{
			result = 0;
		}
		break;
	}

	return result;
}

static int gator_events_load_read(int **buffer, bool sched_switch)
{
	int i;
	int len = 0;

	/* System wide counters - read from one core only */
	if (!on_primary_core() || !load_global_enabled)
		return 0;

	for (i = 0; i < LOAD_COUNTERS_NUM; i++) {
		if (load_counters[i].enabled) {
			load_buffer[len++] = load_counters[i].key;
			load_buffer[len++] =
			    get_load_value(i);
		}
	}

	if (buffer)
		*buffer = load_buffer;

	return len;
}

static struct gator_interface gator_events_load_interface = {
	.create_files = gator_events_load_create_files,
	.start = gator_events_load_start,
	.stop = gator_events_load_stop,
	.read = gator_events_load_read,
};

/* Must not be static! */
int __init gator_events_load_init(void)
{
	int i;

	for (i = 0; i < LOAD_COUNTERS_NUM; i++) {
		load_counters[i].enabled = 0;
		load_counters[i].key = gator_events_get_key();
	}

	return gator_events_install(&gator_events_load_interface);
}
