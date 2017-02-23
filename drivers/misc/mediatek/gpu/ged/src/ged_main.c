/*
 * (C) Copyright 2010
 * MediaTek <www.MediaTek.com>
 *
 * MTK GPU Extension Device
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/disp_assert_layer.h>
#include <mach/system.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/aee.h>

#include "ged_debugFS.h"
#include "ged_log.h"
#include "ged_bridge.h"
#include "ged_profile_dvfs.h"
#include "ged_monitor_3D_fence.h"

#define GED_DRIVER_DEVICE_NAME "ged"

#define GED_IOCTL_PARAM_BUF_SIZE 0x3000 //12KB

#ifdef GED_DEBUG
#define GED_LOG_BUF_COMMON_GLES "GLES"
static GED_LOG_BUF_HANDLE ghLogBuf_GLES = 0;
GED_LOG_BUF_HANDLE ghLogBuf_GED = 0;
#endif

static void* gvIOCTLParamBuf = NULL;

/******************************************************************************
 * GED File operations
 *****************************************************************************/
static int ged_open(struct inode *inode, struct file *filp)
{
    GED_LOGE("%s:%d:%d\n", __func__, MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
    return 0;
}

static int ged_release(struct inode *inode, struct file *filp)
{
    GED_LOGE("%s:%d:%d\n", __func__, MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
    return 0;
}

static unsigned int ged_poll(struct file *file, struct poll_table_struct *ptable)
{
    return 0;
}

static ssize_t ged_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

static ssize_t ged_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

DEFINE_SEMAPHORE(ged_dal_sem);
static long ged_ioctl(struct file *pFile, unsigned int ioctlCmd, unsigned long arg)
{
    int ret = -EFAULT;
	GED_BRIDGE_PACKAGE *psBridgePackageKM, *psBridgePackageUM = (GED_BRIDGE_PACKAGE *)arg;
	GED_BRIDGE_PACKAGE sBridgePackageKM;
    void *pvInt, *pvOut;

    if (down_interruptible(&ged_dal_sem) < 0) 
    {
        GED_LOGE("Fail to down ged_dal_sem\n");
        return -ERESTARTSYS;
    }

	psBridgePackageKM = &sBridgePackageKM;
    if (0 != ged_copy_from_user(psBridgePackageKM, psBridgePackageUM, sizeof(GED_BRIDGE_PACKAGE)))
    {
        GED_LOGE("Fail to ged_copy_from_user\n");
        goto ERROR;
    }

    if (psBridgePackageKM->i32InBufferSize + psBridgePackageKM->i32OutBufferSize < GED_IOCTL_PARAM_BUF_SIZE)
    {
        typedef int (ged_bridge_func_type)(void*, void*);
        ged_bridge_func_type* pFunc = NULL;

        pvInt = gvIOCTLParamBuf;
        pvOut = (void*)((char*)pvInt + (uintptr_t)psBridgePackageKM->i32InBufferSize);
        if (psBridgePackageKM->i32InBufferSize > 0)
        {
            if (0 != ged_copy_from_user(pvInt, psBridgePackageKM->pvParamIn, psBridgePackageKM->i32InBufferSize))
            {
                goto ERROR;
            }
        }

        // we will change the below switch into a function pointer mapping table in the future
        switch(psBridgePackageKM->ui32FunctionID)
        {
        case GED_BRIDGE_IO_LOG_BUF_GET:
            pFunc = (ged_bridge_func_type*)ged_bridge_log_buf_get;
            break;
        case GED_BRIDGE_IO_LOG_BUF_WRITE:
            pFunc = (ged_bridge_func_type*)ged_bridge_log_buf_write;
            break;
        case GED_BRIDGE_IO_LOG_BUF_RESET:
            pFunc = (ged_bridge_func_type*)ged_bridge_log_buf_reset;
            break;            
        case GED_BRIDGE_IO_BOOST_GPU_FREQ:
            pFunc = (ged_bridge_func_type*)ged_bridge_boost_gpu_freq;
            break;
        case GED_BRIDGE_IO_MONITOR_3D_FENCE:
            pFunc = (ged_bridge_func_type*)ged_bridge_monitor_3D_fence;
            break;
        default:
            GED_LOGE("Unknown Command ID: %u\n", psBridgePackageKM->ui32FunctionID);
            break;
        }

        if (pFunc)
        {
            ret = pFunc(pvInt, pvOut);
        }
        else
        {
            ret = GED_ERROR_FAIL;
        }

        if (psBridgePackageKM->i32OutBufferSize > 0)
        {
            if (0 != ged_copy_to_user(psBridgePackageKM->pvParamOut, pvOut, psBridgePackageKM->i32OutBufferSize))
            {
                goto ERROR;
            }
        }
    }

ERROR:
    up(&ged_dal_sem);

    return ret;
}

/******************************************************************************
 * Module related
 *****************************************************************************/

static struct file_operations ged_fops = {
    .owner = THIS_MODULE,
    .open = ged_open,
    .release = ged_release,
    .poll = ged_poll,
    .read = ged_read,
    .write = ged_write,
    .unlocked_ioctl = ged_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = ged_ioctl,
#endif
};

#if 0
static struct miscdevice ged_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ged",
    .fops = &ged_fops,
};
#endif

static void ged_exit(void)
{
#ifdef GED_DEBUG
    ged_log_buf_free(ghLogBuf_GED);
    ghLogBuf_GED = 0;

    ged_log_buf_free(ghLogBuf_GLES);
    ghLogBuf_GLES = 0;
#endif

    ged_profile_dvfs_exit();

    ged_log_system_exit();

    ged_debugFS_exit();

    remove_proc_entry(GED_DRIVER_DEVICE_NAME, NULL);

    if (gvIOCTLParamBuf)
    {
        vfree(gvIOCTLParamBuf);
        gvIOCTLParamBuf = NULL;
    }
}

static int ged_init(void)
{
    GED_ERROR err = GED_ERROR_FAIL;

    gvIOCTLParamBuf = vmalloc(GED_IOCTL_PARAM_BUF_SIZE);
    if (NULL == gvIOCTLParamBuf)
    {
        err = GED_ERROR_OOM;
        goto ERROR;
    }

    if (NULL == proc_create(GED_DRIVER_DEVICE_NAME, 0644, NULL, &ged_fops))
    {
        err = GED_ERROR_FAIL;
        GED_LOGE("ged: failed to register ged proc entry!\n");
        goto ERROR;
    }

    err = ged_debugFS_init();
    if (unlikely(err != GED_OK))
    {
        GED_LOGE("ged: failed to init debug FS!\n");
        goto ERROR;
    }

    err = ged_log_system_init();
    if (unlikely(err != GED_OK))
    {
        GED_LOGE("ged: failed to create gedlog entry!\n");
        goto ERROR;
    }

    err = ged_profile_dvfs_init();
    if (unlikely(err != GED_OK))
    {
        GED_LOGE("ged: failed to init profile dvfs!\n");
        goto ERROR;
    }

#ifdef GED_DEBUG
    ghLogBuf_GLES = ged_log_buf_alloc(160, 128, GED_LOG_BUF_TYPE_RINGBUFFER, GED_LOG_BUF_COMMON_GLES, NULL);
    ghLogBuf_GED = ged_log_buf_alloc(32, 64, GED_LOG_BUF_TYPE_RINGBUFFER, "GED internal", NULL);
#endif

    return 0;

ERROR:
    ged_exit();

    return -EFAULT;
}

module_init(ged_init);
module_exit(ged_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek GED Driver");
MODULE_AUTHOR("MediaTek Inc.");
