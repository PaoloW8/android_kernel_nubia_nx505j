/*
 * cyttsp5_mt_common.c
 * Cypress TrueTouch(TM) Standard Product V5 Multi-Touch Reports Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_mt_common.h"
#define ZTEMT_CYPRESS_PALM_SLEEP    1  //Added by luochangyang, for palm sleep  2013/12/16

/*ZTEMT Added by luochangyang, 2014/02/26*/
static ssize_t cyttsp5_hall_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&md->mt_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", md->hall_mode);
	mutex_unlock(&md->mt_lock);

	return ret;
}

static ssize_t cyttsp5_hall_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 0xFF && value < 0)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	mutex_lock(&md->mt_lock);
	md->hall_mode = (u8)value;
	mutex_unlock(&md->mt_lock);

	pm_runtime_put(dev);

	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR(hall_mode, 0664, cyttsp5_hall_mode_show, cyttsp5_hall_mode_store);
/*ZTEMT END*/

static void cyttsp5_mt_lift_all(struct cyttsp5_mt_data *md)
{
	int max = md->si->tch_abs[CY_TCH_T].max;

	if (md->num_prv_tch != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		md->num_prv_tch = 0;
	}
}

static void cyttsp5_get_touch_axis(struct cyttsp5_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		dev_vdbg(&md->ttsp->dev,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	dev_vdbg(&md->ttsp->dev,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

	dev_dbg(dev,
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[CY_TCH_TIME],
		touch->hdr[CY_TCH_NUM],
		touch->hdr[CY_TCH_LO],
		touch->hdr[CY_TCH_NOISE],
		touch->hdr[CY_TCH_COUNTER]);
}

static void cyttsp5_get_touch(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_abs abs;
	int tmp;
	bool flipped;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}

	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		tmp = touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_Y] = tmp;
		flipped = true;
	} else
		flipped = false;

	if (md->pdata->flags & CY_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_X];
		else
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_X];
	}
	if (md->pdata->flags & CY_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_Y];
		else
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_Y];
	}

	dev_vdbg(dev, "%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

static void cyttsp5_get_mt_touches(struct cyttsp5_mt_data *md,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	int sig;
	int i, j, t = 0;
    u16 fw_ver_ic_temp = 0;
	DECLARE_BITMAP(ids, MAX_TOUCH_NUMBER);
	int mt_sync_count = 0;

	bitmap_zero(ids, MAX_TOUCH_NUMBER);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		cyttsp5_get_touch(md, tch, si->xy_data +
			(i * si->desc.tch_record_size));

		/*  Discard proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			dev_vdbg(dev, "%s: Discarding proximity event\n",
					__func__);
			continue;
		} else if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
		/*** ZTEMT Added by luochangyang, For hover  2013/10/31 ***/
		    if (tch->abs[CY_TCH_Y] < md->pdata->vkeys_y) {
    		    dev_dbg(dev, "%s: HOVER: tch->abs[CY_TCH_O] = %d\n",
    					__func__, tch->abs[CY_TCH_O]);
                
    			tch->abs[CY_TCH_P] = 0;
		    } else {
		        continue;
		    }
        }

		/* Hall mode - filter some pointer */
		if (md->hall_mode == 1) {
            fw_ver_ic_temp = si->fw_ver_ic & 0xFFF0;
            if (fw_ver_ic_temp == 0x05D0){
                if (tch->abs[CY_TCH_Y] > 765) {
                    dev_info(dev, "%s: HALL: filter some pointer.\n", __func__);
                    continue;
                }
            }
            else if (fw_ver_ic_temp == 0x0900 || fw_ver_ic_temp == 0x0A00 ){
                if (tch->abs[CY_TCH_Y] > 1230) {
                    dev_info(dev, "%s: HALL: filter some pointer.\n", __func__);
                    continue;
                }
            }
            else{
                dev_info(dev, "%s: HALL: Don't filter this pointer.\n", __func__);
            }
        }
        /***ZTEMT END***/

		if ((tch->abs[CY_TCH_T] < md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST]) ||
			(tch->abs[CY_TCH_T] > md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MAX_OST])) {
			dev_err(dev, "%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, tch->abs[CY_TCH_T],
				md->pdata->frmwrk->abs[(CY_ABS_ID_OST *
				CY_NUM_ABS_SET) + CY_MAX_OST]);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* use 0 based track id's */
		sig = md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + 0];
		if (sig != CY_IGNORE_VALUE) {
			t = tch->abs[CY_TCH_T] - md->pdata->frmwrk->abs
				[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST];
			if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF) {
				dev_dbg(dev, "%s: t=%d e=%d lift-off\n",
					__func__, t, tch->abs[CY_TCH_E]);
				goto cyttsp5_get_mt_touches_pr_tch;
			}
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[CY_TCH_O]);
			__set_bit(t, ids);
		}

		/* all devices: position and pressure fields */
		for (j = 0; j <= CY_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs[((CY_ABS_X_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE)
				input_report_abs(md->input, sig,
					tch->abs[CY_TCH_X + j]);
		}

		/* Get the extended touch fields */
		for (j = 0; j < CY_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs
				[((CY_ABS_MAJ_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE)
				input_report_abs(md->input, sig,
					tch->abs[CY_TCH_MAJ + j]);
		}
        
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

cyttsp5_get_mt_touches_pr_tch:
		dev_dbg(dev,
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[CY_TCH_X],
			tch->abs[CY_TCH_Y],
			tch->abs[CY_TCH_P],
			tch->abs[CY_TCH_MAJ],
			tch->abs[CY_TCH_MIN],
			tch->abs[CY_TCH_OR],
			tch->abs[CY_TCH_E],
			tch->abs[CY_TCH_O],
			tch->abs[CY_TCH_TIP]);
	}

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, MAX_TOUCH_NUMBER,
				mt_sync_count, ids);

	md->num_prv_tch = num_cur_tch;

	return;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_mt_data *md)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;
#if ZTEMT_CYPRESS_PALM_SLEEP
	unsigned long ids = 0;  //Added by luochangyang, 2013/09/25
#endif
	cyttsp5_get_touch_hdr(md, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > MAX_TOUCH_NUMBER) {
		dev_err(dev, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = MAX_TOUCH_NUMBER;
	}

	if (tch.hdr[CY_TCH_LO]) {
		dev_dbg(dev, "%s: Large area detected\n", __func__);
#if ZTEMT_CYPRESS_PALM_SLEEP
        /*** ZTEMT Added by luochangyang, 2013/09/25 ***/
    	/* For large area event */
    	if (md->mt_function.input_report)
    		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
    			0, CY_OBJ_STANDARD_FINGER);

    	input_report_abs(md->input, ABS_MT_PRESSURE, 1000);

    	if (md->mt_function.input_sync)
    		md->mt_function.input_sync(md->input);
    	if (md->mt_function.final_sync)
    		md->mt_function.final_sync(md->input, 0, 1, &ids);
    	if (md->mt_function.report_slot_liftoff)
    		md->mt_function.report_slot_liftoff(md, 1);
    	if (md->mt_function.final_sync)
    		md->mt_function.final_sync(md->input, 1, 1, &ids);
        
		cyttsp5_mt_lift_all(md);
        /***ZTEMT END***/
#endif
		if (md->pdata->flags & CY_MT_FLAG_NO_TOUCH_ON_LO)
			num_cur_tch = 0;
	}

	/* extract xy_data for all currently reported touches */
	dev_vdbg(dev, "%s: extract data num_cur_tch=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_mt_touches(md, &tch, num_cur_tch);
	else
		cyttsp5_mt_lift_all(md);

	return 0;
}

//hyuc: for easy wakeup
static void cyttsp5_mt_send_dummy_event(struct cyttsp5_mt_data *md)
{
#if 1
    input_report_key(md->input, KEY_F10, 1);
    input_sync(md->input);

    input_report_key(md->input, KEY_F10, 0);
    input_sync(md->input);
#else
	unsigned long ids = 0;

	/* for easy wakeup */
	if (md->mt_function.input_report)
		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
			0, CY_OBJ_STANDARD_FINGER);
	if (md->mt_function.input_sync)
		md->mt_function.input_sync(md->input);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 0, 1, &ids);
	if (md->mt_function.report_slot_liftoff)
		md->mt_function.report_slot_liftoff(md, 1);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 1, 1, &ids);
#endif
}

static int cyttsp5_mt_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
	rc = cyttsp5_xy_worker(md);
	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

//hyuc: for easy wakeup
static int cyttsp5_mt_wake_attention(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(&ttsp->dev);

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_send_dummy_event(md);
	mutex_unlock(&md->mt_lock);
	return 0;
}

static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return rc;
}

static int cyttsp5_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	pm_runtime_get_sync(dev);

	dev_vdbg(dev, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	/* set up wakeup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_WAKE,
		cyttsp5_mt_wake_attention, 0); //hyuc: for easy wakeup

	return 0;
}

static void cyttsp5_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	pm_runtime_put(dev);
}

/* ZTEMT Added by luochangyang, 2013/09/06 */
#if defined(CONFIG_FB)
static int cyttsp5_mt_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct cyttsp5_mt_data *md =
		container_of(self, struct cyttsp5_mt_data, fb_notif);
	struct device *dev = &md->ttsp->dev;
	int *blank;
    
	if (evdata && evdata->data && event == FB_EVENT_BLANK && md && dev) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
            pm_runtime_get(dev);
            
            mutex_lock(&md->mt_lock);
            md->is_suspended = false;
            mutex_unlock(&md->mt_lock);
        } else if (*blank == FB_BLANK_POWERDOWN) {
		
        	if (md->si)
        		cyttsp5_mt_lift_all(md);
			
        	pm_runtime_put_sync(dev);

        	mutex_lock(&md->mt_lock);
        	md->is_suspended = true;
        	mutex_unlock(&md->mt_lock);    
		}
	}

	return 0;
}

static int cyttsp5_mt_fb_register(struct cyttsp5_mt_data *md)
{
    int retval = 0;

    md->fb_notif.notifier_call = cyttsp5_mt_fb_notifier_callback;

    retval = fb_register_client(&md->fb_notif);
    if (retval)
        dev_err(&md->ttsp->dev,
            "Unable to register fb_notifier: %d\n", retval);
    return retval;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cyttsp5_mt_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);
	struct device *dev = &md->ttsp->dev;

	pm_runtime_put_sync(dev);

	mutex_lock(&md->mt_lock);
	md->is_suspended = true;
	mutex_unlock(&md->mt_lock);
}

static void cyttsp5_mt_late_resume(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);
	struct device *dev = &md->ttsp->dev;

	pm_runtime_get(dev);

	mutex_lock(&md->mt_lock);
	md->is_suspended = false;
	mutex_unlock(&md->mt_lock);
}

static void cyttsp5_setup_early_suspend(struct cyttsp5_mt_data *md)
{
	md->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	md->es.suspend = cyttsp5_mt_early_suspend;
	md->es.resume = cyttsp5_mt_late_resume;

	register_early_suspend(&md->es);
}
#endif

#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_mt_rt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_rt_resume(struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_mt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_mt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_mt_suspend, cyttsp5_mt_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_mt_rt_suspend, cyttsp5_mt_rt_resume, NULL)
};

static int cyttsp5_setup_input_device(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int signal = CY_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	dev_vdbg(dev, "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);

/*** ZTEMT Added by luochangyang, 2013/09/11 ***/
	__set_bit(KEY_BACK, md->input->keybit);
	__set_bit(KEY_MENU, md->input->keybit);
	__set_bit(KEY_HOME, md->input->keybit);

    __set_bit(KEY_POWER, md->input->keybit);
    __set_bit(KEY_F10, md->input->keybit);
/***ZTEMT END***/

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif

	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & CY_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		max_x_tmp = md->si->sensing_conf_data.res_x;
		max_y_tmp = md->si->sensing_conf_data.res_y;
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}
	max_p = md->si->sensing_conf_data.max_z;

	/* set event signal capabilities */
	for (i = 0; i < (md->pdata->frmwrk->size / CY_NUM_ABS_SET); i++) {
		signal = md->pdata->frmwrk->abs
			[(i * CY_NUM_ABS_SET) + CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);
			min = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MIN_OST];
			max = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MAX_OST];
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			} else if (i == CY_ABS_X_OST)
				max = max_x;
			else if (i == CY_ABS_Y_OST)
				max = max_y;
			else if (i == CY_ABS_P_OST)
				max = max_p;
			input_set_abs_params(md->input, signal, min, max,
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FUZZ_OST],
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FLAT_OST]);
			dev_dbg(dev, "%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

	rc = md->mt_function.input_register_device(md->input,
			md->si->tch_abs[CY_TCH_T].max);
	if (rc < 0)
		dev_err(dev, "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	md->si = cyttsp5_request_sysinfo(ttsp);
	if (!md->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

static int cyttsp5_mt_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	/*
	 * This check is to prevent pm_runtime usage_count drop below zero
	 * because of removing the module while in suspended state
	 */
	if (md->is_suspended)
		pm_runtime_get_noresume(dev);
    
/*** ZTEMT Modify by luochangyang, 2014/1/3 ***/
#if defined(CONFIG_FB)
    if (fb_unregister_client(&md->fb_notif))
        dev_err(dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/***ZTEMT END***/
	unregister_early_suspend(&md->es);
#endif

	if (md->input_device_registered) {
		input_unregister_device(md->input);
	} else {
		input_free_device(md->input);
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	
	/*ZTEMT Added by luochangyang, 2014/02/26*/
	device_remove_file(dev, &dev_attr_hall_mode);
	/*ZTEMT END*/

	dev_set_drvdata(dev, NULL);
	kfree(md);
	return 0;
}

static int cyttsp5_mt_probe(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md;
	struct cyttsp5_mt_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	if (pdata == NULL) {
		dev_err(dev, "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	cyttsp5_init_function_ptrs(md);

	mutex_init(&md->mt_lock);
	md->ttsp = ttsp;
	md->pdata = pdata;
	dev_set_drvdata(dev, md);
	/* Create the input device and register it. */
	dev_vdbg(dev, "%s: Create the input device and register it\n",
		__func__);
	md->input = input_allocate_device();
	if (md->input == NULL) {
		dev_err(dev, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (pdata->inp_dev_name)
		md->input->name = pdata->inp_dev_name;
	else
		md->input->name = ttsp->name;
	scnprintf(md->phys, sizeof(md->phys)-1, "%s", dev_name(dev));
	md->input->phys = md->phys;
	md->input->dev.parent = &md->ttsp->dev;
	md->input->open = cyttsp5_mt_open;
	md->input->close = cyttsp5_mt_close;
	input_set_drvdata(md->input, md);

	pm_runtime_enable(dev);

	/* get sysinfo */
	md->si = cyttsp5_request_sysinfo(ttsp);

	if (md->si) {
		rc = cyttsp5_setup_input_device(ttsp);
		if (rc)
			goto error_init_input;
	} else {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, md->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}
    
#if defined(CONFIG_FB)
    cyttsp5_mt_fb_register(md);
#elif defined (CONFIG_HAS_EARLYSUSPEND)
	cyttsp5_setup_early_suspend(md);
#endif

	/*ZTEMT Added by luochangyang, 2014/02/26*/
	dev_err(dev, "%s: create hall_mode\n",
					__func__);

	rc = device_create_file(dev, &dev_attr_hall_mode);
	if (rc) {
		dev_err(dev, "%s: Error, could not create hall_mode\n",
				__func__);
		goto error_init_input;
	}
	/*ZTEMT END*/

	return 0;

error_init_input:
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	input_free_device(md->input);
error_alloc_failed:
	dev_set_drvdata(dev, NULL);
	kfree(md);
error_alloc_data_failed:
error_no_pdata:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

struct cyttsp5_driver cyttsp5_mt_driver = {
	.probe = cyttsp5_mt_probe,
	.remove = cyttsp5_mt_release,
	.driver = {
		.name = CYTTSP5_MT_NAME,
		.bus = &cyttsp5_bus_type,
		.pm = &cyttsp5_mt_pm_ops,
	},
};

