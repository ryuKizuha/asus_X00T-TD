/*
 * Universal Flash Storage Host controller driver Core
 *
 * This code is based on drivers/scsi/ufs/ufshcd.c
 * Copyright (C) 2011-2013 Samsung India Software Operations
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 *
 * Authors:
 *	Santosh Yaraganavi <santosh.sy@samsung.com>
 *	Vinayak Holikatti <h.vinayak@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#include <linux/async.h>
#include <scsi/ufs/ioctl.h>
#include <linux/devfreq.h>
#include <linux/nls.h>
#include <linux/of.h>
#include <linux/blkdev.h>
#include <asm/unaligned.h>

#include "ufshcd.h"
#include "ufshci.h"
#include "ufs_quirks.h"
#include "ufs-debugfs.h"
#include "ufs-qcom.h"

#define CREATE_TRACE_POINTS
#include <trace/events/ufs.h>

#ifdef CONFIG_DEBUG_FS

static int ufshcd_tag_req_type(struct request *rq)
{
	int rq_type = TS_WRITE;

	if (!rq || !(rq->cmd_type & REQ_TYPE_FS))
		rq_type = TS_NOT_SUPPORTED;
	else if (rq->cmd_flags & REQ_FLUSH)
		rq_type = TS_FLUSH;
	else if (rq_data_dir(rq) == READ)
		rq_type = (rq->cmd_flags & REQ_URGENT) ?
			TS_URGENT_READ : TS_READ;
	else if (rq->cmd_flags & REQ_URGENT)
		rq_type = TS_URGENT_WRITE;

	return rq_type;
}

static void ufshcd_update_error_stats(struct ufs_hba *hba, int type)
{
	ufsdbg_set_err_state(hba);
	if (type < UFS_ERR_MAX)
		hba->ufs_stats.err_stats[type]++;
}

static void ufshcd_update_tag_stats(struct ufs_hba *hba, int tag)
{
	struct request *rq =
		hba->lrb[tag].cmd ? hba->lrb[tag].cmd->request : NULL;
	u64 **tag_stats = hba->ufs_stats.tag_stats;
	int rq_type;

	if (!hba->ufs_stats.enabled)
		return;

	tag_stats[tag][TS_TAG]++;
	if (!rq || !(rq->cmd_type & REQ_TYPE_FS))
		return;

	WARN_ON(hba->ufs_stats.q_depth > hba->nutrs);
	rq_type = ufshcd_tag_req_type(rq);
	if (!(rq_type < 0 || rq_type > TS_NUM_STATS))
		tag_stats[hba->ufs_stats.q_depth++][rq_type]++;
}

static void ufshcd_update_tag_stats_completion(struct ufs_hba *hba,
		struct scsi_cmnd *cmd)
{
	struct request *rq = cmd ? cmd->request : NULL;

	if (rq && rq->cmd_type & REQ_TYPE_FS)
		hba->ufs_stats.q_depth--;
}

static void update_req_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	int rq_type;
	struct request *rq = lrbp->cmd ? lrbp->cmd->request : NULL;
	s64 delta = ktime_us_delta(lrbp->complete_time_stamp,
		lrbp->issue_time_stamp);

	/* update general request statistics */
	if (hba->ufs_stats.req_stats[TS_TAG].count == 0)
		hba->ufs_stats.req_stats[TS_TAG].min = delta;
	hba->ufs_stats.req_stats[TS_TAG].count++;
	hba->ufs_stats.req_stats[TS_TAG].sum += delta;
	if (delta > hba->ufs_stats.req_stats[TS_TAG].max)
		hba->ufs_stats.req_stats[TS_TAG].max = delta;
	if (delta < hba->ufs_stats.req_stats[TS_TAG].min)
			hba->ufs_stats.req_stats[TS_TAG].min = delta;

	rq_type = ufshcd_tag_req_type(rq);
	if (rq_type == TS_NOT_SUPPORTED)
		return;

	/* update request type specific statistics */
	if (hba->ufs_stats.req_stats[rq_type].count == 0)
		hba->ufs_stats.req_stats[rq_type].min = delta;
	hba->ufs_stats.req_stats[rq_type].count++;
	hba->ufs_stats.req_stats[rq_type].sum += delta;
	if (delta > hba->ufs_stats.req_stats[rq_type].max)
		hba->ufs_stats.req_stats[rq_type].max = delta;
	if (delta < hba->ufs_stats.req_stats[rq_type].min)
			hba->ufs_stats.req_stats[rq_type].min = delta;
}

static void
ufshcd_update_query_stats(struct ufs_hba *hba, enum query_opcode opcode, u8 idn)
{
	if (opcode < UPIU_QUERY_OPCODE_MAX && idn < MAX_QUERY_IDN)
		hba->ufs_stats.query_stats_arr[opcode][idn]++;
}

#else
static inline void ufshcd_update_tag_stats(struct ufs_hba *hba, int tag)
{
}

static inline void ufshcd_update_tag_stats_completion(struct ufs_hba *hba,
		struct scsi_cmnd *cmd)
{
}

static inline void ufshcd_update_error_stats(struct ufs_hba *hba, int type)
{
}

static inline
void update_req_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
}

static inline
void ufshcd_update_query_stats(struct ufs_hba *hba,
			       enum query_opcode opcode, u8 idn)
{
}
#endif

#define PWR_INFO_MASK	0xF
#define PWR_RX_OFFSET	4

#define UFSHCD_REQ_SENSE_SIZE	18

#define UFSHCD_ENABLE_INTRS	(UTP_TRANSFER_REQ_COMPL |\
				 UTP_TASK_REQ_COMPL |\
				 UFSHCD_ERROR_MASK)
/* UIC command timeout, unit: ms */
#define UIC_CMD_TIMEOUT	500

/* NOP OUT retries waiting for NOP IN response */
#define NOP_OUT_RETRIES    10
/* Timeout after 30 msecs if NOP OUT hangs without response */
#define NOP_OUT_TIMEOUT    30 /* msecs */

/* Query request retries */
#define QUERY_REQ_RETRIES 3
/* Query request timeout */
#define QUERY_REQ_TIMEOUT 1500 /* 1.5 seconds */

/* Task management command timeout */
#define TM_CMD_TIMEOUT	100 /* msecs */

/* maximum number of retries for a general UIC command  */
#define UFS_UIC_COMMAND_RETRIES 3

/* maximum number of link-startup retries */
#define DME_LINKSTARTUP_RETRIES 3

/* Maximum retries for Hibern8 enter */
#define UIC_HIBERN8_ENTER_RETRIES 3

/* maximum number of reset retries before giving up */
#define MAX_HOST_RESET_RETRIES 5

/* Expose the flag value from utp_upiu_query.value */
#define MASK_QUERY_UPIU_FLAG_LOC 0xFF

/* Interrupt aggregation default timeout, unit: 40us */
#define INT_AGGR_DEF_TO	0x02

/* default value of auto suspend is 3 seconds */
#define UFSHCD_AUTO_SUSPEND_DELAY_MS 3000 /* millisecs */

#define UFSHCD_CLK_GATING_DELAY_MS_PWR_SAVE	10
#define UFSHCD_CLK_GATING_DELAY_MS_PERF		50

/* IOCTL opcode for command - ufs set device read only */
#define UFS_IOCTL_BLKROSET      BLKROSET

#define UFSHCD_DEFAULT_LANES_PER_DIRECTION		2

#define ufshcd_toggle_vreg(_dev, _vreg, _on)				\
	({                                                              \
		int _ret;                                               \
		if (_on)                                                \
			_ret = ufshcd_enable_vreg(_dev, _vreg);         \
		else                                                    \
			_ret = ufshcd_disable_vreg(_dev, _vreg);        \
		_ret;                                                   \
	})

#define ufshcd_hex_dump(prefix_str, buf, len) \
print_hex_dump(KERN_ERR, prefix_str, DUMP_PREFIX_OFFSET, 16, 4, buf, len, false)

static u32 ufs_query_desc_max_size[] = {
	QUERY_DESC_DEVICE_MAX_SIZE,
	QUERY_DESC_CONFIGURAION_MAX_SIZE,
	QUERY_DESC_UNIT_MAX_SIZE,
	QUERY_DESC_RFU_MAX_SIZE,
	QUERY_DESC_INTERCONNECT_MAX_SIZE,
	QUERY_DESC_STRING_MAX_SIZE,
	QUERY_DESC_RFU_MAX_SIZE,
	QUERY_DESC_GEOMETRY_MAZ_SIZE,
	QUERY_DESC_POWER_MAX_SIZE,
	QUERY_DESC_HEALTH_MAX_SIZE,
	QUERY_DESC_RFU_MAX_SIZE,
};

enum {
	UFSHCD_MAX_CHANNEL	= 0,
	UFSHCD_MAX_ID		= 1,
	UFSHCD_CMD_PER_LUN	= 32,
	UFSHCD_CAN_QUEUE	= 32,
};

/* UFSHCD states */
enum {
	UFSHCD_STATE_RESET,
	UFSHCD_STATE_ERROR,
	UFSHCD_STATE_OPERATIONAL,
};

/* UFSHCD error handling flags */
enum {
	UFSHCD_EH_IN_PROGRESS = (1 << 0),
};

/* UFSHCD UIC layer error flags */
enum {
	UFSHCD_UIC_DL_PA_INIT_ERROR = (1 << 0), /* Data link layer error */
	UFSHCD_UIC_DL_NAC_RECEIVED_ERROR = (1 << 1), /* Data link layer error */
	UFSHCD_UIC_DL_TCx_REPLAY_ERROR = (1 << 2), /* Data link layer error */
	UFSHCD_UIC_NL_ERROR = (1 << 3), /* Network layer error */
	UFSHCD_UIC_TL_ERROR = (1 << 4), /* Transport Layer error */
	UFSHCD_UIC_DME_ERROR = (1 << 5), /* DME error */
};

/* Interrupt configuration options */
enum {
	UFSHCD_INT_DISABLE,
	UFSHCD_INT_ENABLE,
	UFSHCD_INT_CLEAR,
};

#define DEFAULT_UFSHCD_DBG_PRINT_EN	UFSHCD_DBG_PRINT_ALL

#define ufshcd_set_eh_in_progress(h) \
	(h->eh_flags |= UFSHCD_EH_IN_PROGRESS)
#define ufshcd_eh_in_progress(h) \
	(h->eh_flags & UFSHCD_EH_IN_PROGRESS)
#define ufshcd_clear_eh_in_progress(h) \
	(h->eh_flags &= ~UFSHCD_EH_IN_PROGRESS)

#define ufshcd_set_ufs_dev_active(h) \
	((h)->curr_dev_pwr_mode = UFS_ACTIVE_PWR_MODE)
#define ufshcd_set_ufs_dev_sleep(h) \
	((h)->curr_dev_pwr_mode = UFS_SLEEP_PWR_MODE)
#define ufshcd_set_ufs_dev_poweroff(h) \
	((h)->curr_dev_pwr_mode = UFS_POWERDOWN_PWR_MODE)
#define ufshcd_is_ufs_dev_active(h) \
	((h)->curr_dev_pwr_mode == UFS_ACTIVE_PWR_MODE)
#define ufshcd_is_ufs_dev_sleep(h) \
	((h)->curr_dev_pwr_mode == UFS_SLEEP_PWR_MODE)
#define ufshcd_is_ufs_dev_poweroff(h) \
	((h)->curr_dev_pwr_mode == UFS_POWERDOWN_PWR_MODE)

static struct ufs_pm_lvl_states ufs_pm_lvl_states[] = {
	{UFS_ACTIVE_PWR_MODE, UIC_LINK_ACTIVE_STATE},
	{UFS_ACTIVE_PWR_MODE, UIC_LINK_HIBERN8_STATE},
	{UFS_SLEEP_PWR_MODE, UIC_LINK_ACTIVE_STATE},
	{UFS_SLEEP_PWR_MODE, UIC_LINK_HIBERN8_STATE},
	{UFS_POWERDOWN_PWR_MODE, UIC_LINK_HIBERN8_STATE},
	{UFS_POWERDOWN_PWR_MODE, UIC_LINK_OFF_STATE},
};

static inline enum ufs_dev_pwr_mode
ufs_get_pm_lvl_to_dev_pwr_mode(enum ufs_pm_level lvl)
{
	return ufs_pm_lvl_states[lvl].dev_state;
}

static inline enum uic_link_state
ufs_get_pm_lvl_to_link_pwr_state(enum ufs_pm_level lvl)
{
	return ufs_pm_lvl_states[lvl].link_state;
}

static inline enum ufs_pm_level
ufs_get_desired_pm_lvl_for_dev_link_state(enum ufs_dev_pwr_mode dev_state,
					enum uic_link_state link_state)
{
	enum ufs_pm_level lvl;

	for (lvl = UFS_PM_LVL_0; lvl < UFS_PM_LVL_MAX; lvl++) {
		if ((ufs_pm_lvl_states[lvl].dev_state == dev_state) &&
			(ufs_pm_lvl_states[lvl].link_state == link_state))
			return lvl;
	}

	/* if no match found, return the level 0 */
	return UFS_PM_LVL_0;
}

static inline bool ufshcd_is_valid_pm_lvl(int lvl)
{
	if (lvl >= 0 && lvl < ARRAY_SIZE(ufs_pm_lvl_states))
		return true;
	else
		return false;
}

static irqreturn_t ufshcd_intr(int irq, void *__hba);
static irqreturn_t ufshcd_tmc_handler(struct ufs_hba *hba);
static void ufshcd_async_scan(void *data, async_cookie_t cookie);
static int ufshcd_reset_and_restore(struct ufs_hba *hba);
static int ufshcd_eh_host_reset_handler(struct scsi_cmnd *cmd);
static int ufshcd_clear_tm_cmd(struct ufs_hba *hba, int tag);
static void ufshcd_hba_exit(struct ufs_hba *hba);
static int ufshcd_probe_hba(struct ufs_hba *hba);
static int ufshcd_enable_clocks(struct ufs_hba *hba);
static int ufshcd_disable_clocks(struct ufs_hba *hba,
				 bool is_gating_context);
static int ufshcd_disable_clocks_skip_ref_clk(struct ufs_hba *hba,
					      bool is_gating_context);
static void ufshcd_hold_all(struct ufs_hba *hba);
static void ufshcd_release_all(struct ufs_hba *hba);
static int ufshcd_set_vccq_rail_unused(struct ufs_hba *hba, bool unused);
static inline void ufshcd_add_delay_before_dme_cmd(struct ufs_hba *hba);
static inline void ufshcd_save_tstamp_of_last_dme_cmd(struct ufs_hba *hba);
static int ufshcd_host_reset_and_restore(struct ufs_hba *hba);
static void ufshcd_resume_clkscaling(struct ufs_hba *hba);
static void ufshcd_suspend_clkscaling(struct ufs_hba *hba);
static void __ufshcd_suspend_clkscaling(struct ufs_hba *hba);
static void ufshcd_release_all(struct ufs_hba *hba);
static void ufshcd_hba_vreg_set_lpm(struct ufs_hba *hba);
static void ufshcd_hba_vreg_set_hpm(struct ufs_hba *hba);
static int ufshcd_devfreq_target(struct device *dev,
				unsigned long *freq, u32 flags);
static int ufshcd_devfreq_get_dev_status(struct device *dev,
		struct devfreq_dev_status *stat);

#if IS_ENABLED(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)
static struct devfreq_simple_ondemand_data ufshcd_ondemand_data = {
	.upthreshold = 35,
	.downdifferential = 30,
	.simple_scaling = 1,
};

static void *gov_data = &ufshcd_ondemand_data;
#else
static void *gov_data;
#endif

static struct devfreq_dev_profile ufs_devfreq_profile = {
	.polling_ms	= 40,
	.target		= ufshcd_devfreq_target,
	.get_dev_status	= ufshcd_devfreq_get_dev_status,
};

static inline bool ufshcd_valid_tag(struct ufs_hba *hba, int tag)
{
	return tag >= 0 && tag < hba->nutrs;
}

static inline void ufshcd_enable_irq(struct ufs_hba *hba)
{
	if (!hba->is_irq_enabled) {
		enable_irq(hba->irq);
		hba->is_irq_enabled = true;
	}
}

static inline void ufshcd_disable_irq(struct ufs_hba *hba)
{
	if (hba->is_irq_enabled) {
		disable_irq(hba->irq);
		hba->is_irq_enabled = false;
	}
}

void ufshcd_scsi_unblock_requests(struct ufs_hba *hba)
{
	unsigned long flags;
	bool unblock = false;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->scsi_block_reqs_cnt--;
	unblock = !hba->scsi_block_reqs_cnt;
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	if (unblock)
		scsi_unblock_requests(hba->host);
}
EXPORT_SYMBOL(ufshcd_scsi_unblock_requests);

static inline void __ufshcd_scsi_block_requests(struct ufs_hba *hba)
{
	if (!hba->scsi_block_reqs_cnt++)
		scsi_block_requests(hba->host);
}

void ufshcd_scsi_block_requests(struct ufs_hba *hba)
{
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	__ufshcd_scsi_block_requests(hba);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
}
EXPORT_SYMBOL(ufshcd_scsi_block_requests);

static int ufshcd_device_reset_ctrl(struct ufs_hba *hba, bool ctrl)
{
	int ret = 0;

	if (!hba->pctrl)
		return 0;

	/* Assert reset if ctrl == true */
	if (ctrl)
		ret = pinctrl_select_state(hba->pctrl,
			pinctrl_lookup_state(hba->pctrl, "dev-reset-assert"));
	else
		ret = pinctrl_select_state(hba->pctrl,
			pinctrl_lookup_state(hba->pctrl, "dev-reset-deassert"));

	if (ret < 0)
		dev_err(hba->dev, "� : �  failed with err �n",
			__func__, ctrl ? "Assert" : "Deassert", ret);

	return ret;
}

static inline int ufshcd_assert_device_reset(struct ufs_hba *hba)
{
	return ufshcd_device_reset_ctrl(hba, true);
}

static inline int ufshcd_deassert_device_reset(struct ufs_hba *hba)
{
	return ufshcd_device_reset_ctrl(hba, false);
}

static int ufshcd_reset_device(struct ufs_hba *hba)
{
	int ret;

	/* reset the connected UFS device */
	ret = ufshcd_assert_device_reset(hba);
	if (ret)
		goto out;
	/*
	 * The reset signal is active low.
	 * The UFS device shall detect more than or equal to 1us of positive
	 * or negative RST_n pulse width.
	 * To be on safe side, keep the reset low for atleast 10us.
	 */
	usleep_range(10, 15);

	ret = ufshcd_deassert_device_reset(hba);
	if (ret)
		goto out;
	/* same as assert, wait for atleast 10us after deassert */
	usleep_range(10, 15);
out:
	return ret;
}

/* replace non-printable or non-ASCII characters with spaces */
static inline void ufshcd_remove_non_printable(char *val)
{
	if (!val || !*val)
		return;

	if (*val < 0x20 || *val > 0x7e)
		*val = ' ';
}

#define UFSHCD_MAX_CMD_LOGGING	200

#ifdef CONFIG_TRACEPOINTS
static inline void ufshcd_add_command_trace(struct ufs_hba *hba,
			struct ufshcd_cmd_log_entry *entry, u8 opcode)
{
	if (trace_ufshcd_command_enabled()) {
		u32 intr = ufshcd_readl(hba, REG_INTERRUPT_STATUS);

		trace_ufshcd_command(dev_name(hba->dev), entry->str, entry->tag,
				     entry->doorbell, entry->transfer_len, intr,
				     entry->lba, opcode);
	}
}
#else
static inline void ufshcd_add_command_trace(struct ufs_hba *hba,
			struct ufshcd_cmd_log_entry *entry, u8 opcode)
{
}
#endif

#ifdef CONFIG_SCSI_UFSHCD_CMD_LOGGING
static void ufshcd_cmd_log_init(struct ufs_hba *hba)
{
	/* Allocate log entries */
	if (!hba->cmd_log.entries) {
		hba->cmd_log.entries = kzalloc(UFSHCD_MAX_CMD_LOGGING *
			sizeof(struct ufshcd_cmd_log_entry), GFP_KERNEL);
		if (!hba->cmd_log.entries)
			return;
		dev_dbg(hba->dev, "� : cmd_log.entries initialized\n",
				__func__);
	}
}

static void __ufshcd_cmd_log(struct ufs_hba *hba, char *str, char *cmd_type,
			     unsigned int tag, u8 cmd_id, u8 idn, u8 lun,
			     sector_t lba, int transfer_len, u8 opcode)
{
	struct ufshcd_cmd_log_entry *entry;

	if (!hba->cmd_log.entries)
		return;

	entry = &hba->cmd_log.entries[hba->cmd_log.pos];
	entry->lun = lun;
	entry->str = str;
	entry->cmd_type = cmd_type;
	entry->cmd_id = cmd_id;
	entry->lba = lba;
	entry->transfer_len = transfer_len;
	entry->idn = idn;
	entry->doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
	entry->tag = tag;
	entry->tstamp = ktime_get();
	entry->outstanding_reqs = hba->outstanding_reqs;
	entry->seq_num = hba->cmd_log.seq_num;
	hba->cmd_log.seq_num++;
	hba->cmd_log.pos =
			(hba->cmd_log.pos + 1) � UFSHCD_MAX_CMD_LOGGING;

	ufshcd_add_command_trace(hba, entry, opcode);
}

static void ufshcd_cmd_log(struct ufs_hba *hba, char *str, char *cmd_type,
	unsigned int tag, u8 cmd_id, u8 idn)
{
	__ufshcd_cmd_log(hba, str, cmd_type, tag, cmd_id, idn,
			 0xff, (sector_t)-1, -1, -1);
}

static void ufshcd_dme_cmd_log(struct ufs_hba *hba, char *str, u8 cmd_id)
{
	ufshcd_cmd_log(hba, str, "dme", 0xff, cmd_id, 0xff);
}

static void ufshcd_print_cmd_log(struct ufs_hba *hba)
{
	int i;
	int pos;
	struct ufshcd_cmd_log_entry *p;

	if (!hba->cmd_log.entries)
		return;

	pos = hba->cmd_log.pos;
	for (i = 0; i < UFSHCD_MAX_CMD_LOGGING; i++) {
		p = &hba->cmd_log.entries[pos];
		pos = (pos + 1) � UFSHCD_MAX_CMD_LOGGING;

		if (ktime_to_us(p->tstamp)) {
			pr_err("� : � : seq_no=�  lun=0x�  cmd_id=0xx lba=0x� lx txfer_len=�tag=� , doorbell=0x�  outstanding=0x�  idn=�time=� ld us\n",
				p->cmd_type, p->str, p->seq_num,
				p->lun, p->cmd_id, (unsigned long long)p->lba,
				p->transfer_len, p->tag, p->doorbell,
				p->outstanding_reqs, p->idn,
				ktime_to_us(p->tstamp));
				usleep_range(1000, 1100);
		}
	}
}
#else
static void ufshcd_cmd_log_init(struct ufs_hba *hba)
{
}

static void __ufshcd_cmd_log(struct ufs_hba *hba, char *str, char *cmd_type,
			     unsigned int tag, u8 cmd_id, u8 idn, u8 lun,
			     sector_t lba, int transfer_len, u8 opcode)
{
	struct ufshcd_cmd_log_entry entry;

	entry.str = str;
	entry.lba = lba;
	entry.cmd_id = cmd_id;
	entry.transfer_len = transfer_len;
	entry.doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
	entry.tag = tag;

	ufshcd_add_command_trace(hba, &entry, opcode);
}

static void ufshcd_dme_cmd_log(struct ufs_hba *hba, char *str, u8 cmd_id)
{
}

static void ufshcd_print_cmd_log(struct ufs_hba *hba)
{
}
#endif

#ifdef CONFIG_TRACEPOINTS
static inline void ufshcd_cond_add_cmd_trace(struct ufs_hba *hba,
					unsigned int tag, const char *str)
{
	struct ufshcd_lrb *lrbp;
	char *cmd_type = NULL;
	u8 opcode = 0;
	u8 cmd_id = 0, idn = 0;
	sector_t lba = -1;
	int transfer_len = -1;

	lrbp = &hba->lrb[tag];

	if (lrbp->cmd) { /* data phase exists */
		opcode = (u8)(*lrbp->cmd->cmnd);
		if ((opcode == READ_10) || (opcode == WRITE_10)) {
			/*
			 * Currently we only fully trace read(10) and write(10)
			 * commands
			 */
			if (lrbp->cmd->request && lrbp->cmd->request->bio)
				lba =
				lrbp->cmd->request->bio->bi_iter.bi_sector;
			transfer_len = be32_to_cpu(
				lrbp->ucd_req_ptr->sc.exp_data_transfer_len);
		}
	}

	if (lrbp->cmd && (lrbp->command_type == UTP_CMD_TYPE_SCSI)) {
		cmd_type = "scsi";
		cmd_id = (u8)(*lrbp->cmd->cmnd);
	} else if (lrbp->command_type == UTP_CMD_TYPE_DEV_MANAGE) {
		if (hba->dev_cmd.type == DEV_CMD_TYPE_NOP) {
			cmd_type = "nop";
			cmd_id = 0;
		} else if (hba->dev_cmd.type == DEV_CMD_TYPE_QUERY) {
			cmd_type = "query";
			cmd_id = hba->dev_cmd.query.request.upiu_req.opcode;
			idn = hba->dev_cmd.query.request.upiu_req.idn;
		}
	}

	__ufshcd_cmd_log(hba, (char *) str, cmd_type, tag, cmd_id, idn,
			 lrbp->lun, lba, transfer_len, opcode);
}
#else
static inline void ufshcd_cond_add_cmd_trace(struct ufs_hba *hba,
					unsigned int tag, const char *str)
{
}
#endif

static void ufshcd_print_clk_freqs(struct ufs_hba *hba)
{
	struct ufs_clk_info *clki;
	struct list_head *head = &hba->clk_list_head;

	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_CLK_FREQ_EN))
		return;

	if (!head || list_empty(head))
		return;

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR_OR_NULL(clki->clk) && clki->min_freq &&
				clki->max_freq)
			dev_err(hba->dev, "clk: � , rate: � \n",
					clki->name, clki->curr_freq);
	}
}

static void ufshcd_print_uic_err_hist(struct ufs_hba *hba,
		struct ufs_uic_err_reg_hist *err_hist, char *err_name)
{
	int i;

	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_UIC_ERR_HIST_EN))
		return;

	for (i = 0; i < UIC_ERR_REG_HIST_LENGTH; i++) {
		int p = (i + err_hist->pos - 1) � UIC_ERR_REG_HIST_LENGTH;

		if (err_hist->reg[p] == 0)
			continue;
		dev_err(hba->dev, "� [� = 0x�  at � ld us", err_name, i,
			err_hist->reg[p], ktime_to_us(err_hist->tstamp[p]));
	}
}

static inline void __ufshcd_print_host_regs(struct ufs_hba *hba, bool no_sleep)
{
	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_HOST_REGS_EN))
		return;

	/*
	 * hex_dump reads its data without the readl macro. This might
	 * cause inconsistency issues on some platform, as the printed
	 * values may be from cache and not the most recent value.
	 * To know whether you are looking at an un-cached version verify
	 * that IORESOURCE_MEM flag is on when xxx_get_resource() is invoked
	 * during platform/pci probe function.
	 */
	ufshcd_hex_dump("host regs: ", hba->mmio_base, UFSHCI_REG_SPACE_SIZE);
	dev_err(hba->dev, "hba->ufs_version = 0x� , hba->capabilities = 0x� ",
		hba->ufs_version, hba->capabilities);
	dev_err(hba->dev,
		"hba->outstanding_reqs = 0x� , hba->outstanding_tasks = 0x� ",
		(u32)hba->outstanding_reqs, (u32)hba->outstanding_tasks);
	dev_err(hba->dev,
		"last_hibern8_exit_tstamp at � ld us, hibern8_exit_cnt = �,
		ktime_to_us(hba->ufs_stats.last_hibern8_exit_tstamp),
		hba->ufs_stats.hibern8_exit_cnt);

	ufshcd_print_uic_err_hist(hba, &hba->ufs_stats.pa_err, "pa_err");
	ufshcd_print_uic_err_hist(hba, &hba->ufs_stats.dl_err, "dl_err");
	ufshcd_print_uic_err_hist(hba, &hba->ufs_stats.nl_err, "nl_err");
	ufshcd_print_uic_err_hist(hba, &hba->ufs_stats.tl_err, "tl_err");
	ufshcd_print_uic_err_hist(hba, &hba->ufs_stats.dme_err, "dme_err");

	ufshcd_print_clk_freqs(hba);

	ufshcd_vops_dbg_register_dump(hba, no_sleep);
}

static void ufshcd_print_host_regs(struct ufs_hba *hba)
{
	__ufshcd_print_host_regs(hba, false);
}

static
void ufshcd_print_trs(struct ufs_hba *hba, unsigned long bitmap, bool pr_prdt)
{
	struct ufshcd_lrb *lrbp;
	int prdt_length;
	int tag;

	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_TRS_EN))
		return;

	for_each_set_bit(tag, &bitmap, hba->nutrs) {
		lrbp = &hba->lrb[tag];

		dev_err(hba->dev, "UPIU[� - issue time � ld us",
				tag, ktime_to_us(lrbp->issue_time_stamp));
		dev_err(hba->dev,
			"UPIU[� - Transfer Request Descriptor phys@0x� lx",
			tag, (u64)lrbp->utrd_dma_addr);
		ufshcd_hex_dump("UPIU TRD: ", lrbp->utr_descriptor_ptr,
				sizeof(struct utp_transfer_req_desc));
		dev_err(hba->dev, "UPIU[� - Request UPIU phys@0x� lx", tag,
			(u64)lrbp->ucd_req_dma_addr);
		ufshcd_hex_dump("UPIU REQ: ", lrbp->ucd_req_ptr,
				sizeof(struct utp_upiu_req));
		dev_err(hba->dev, "UPIU[� - Response UPIU phys@0x� lx", tag,
			(u64)lrbp->ucd_rsp_dma_addr);
		ufshcd_hex_dump("UPIU RSP: ", lrbp->ucd_rsp_ptr,
				sizeof(struct utp_upiu_rsp));
		prdt_length =
			le16_to_cpu(lrbp->utr_descriptor_ptr->prd_table_length);
		dev_err(hba->dev, "UPIU[� - PRDT - �entries  phys@0x� lx",
			tag, prdt_length, (u64)lrbp->ucd_prdt_dma_addr);
		if (pr_prdt)
			ufshcd_hex_dump("UPIU PRDT: ", lrbp->ucd_prdt_ptr,
				sizeof(struct ufshcd_sg_entry) * prdt_length);
	}
}

static void ufshcd_print_tmrs(struct ufs_hba *hba, unsigned long bitmap)
{
	struct utp_task_req_desc *tmrdp;
	int tag;

	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_TMRS_EN))
		return;

	for_each_set_bit(tag, &bitmap, hba->nutmrs) {
		tmrdp = &hba->utmrdl_base_addr[tag];
		dev_err(hba->dev, "TM[� - Task Management Header", tag);
		ufshcd_hex_dump("TM TRD: ", &tmrdp->header,
				sizeof(struct request_desc_header));
		dev_err(hba->dev, "TM[� - Task Management Request UPIU",
				tag);
		ufshcd_hex_dump("TM REQ: ", tmrdp->task_req_upiu,
				sizeof(struct utp_upiu_req));
		dev_err(hba->dev, "TM[� - Task Management Response UPIU",
				tag);
		ufshcd_hex_dump("TM RSP: ", tmrdp->task_rsp_upiu,
				sizeof(struct utp_task_req_desc));
	}
}

static void ufshcd_print_fsm_state(struct ufs_hba *hba)
{
	int err = 0, tx_fsm_val = 0, rx_fsm_val = 0;

	err = ufshcd_dme_get(hba,
			UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE,
			UIC_ARG_MPHY_TX_GEN_SEL_INDEX(0)),
			&tx_fsm_val);
	dev_err(hba->dev, "� : TX_FSM_STATE = � , err = �n", __func__,
			tx_fsm_val, err);
	err = ufshcd_dme_get(hba,
			UIC_ARG_MIB_SEL(MPHY_RX_FSM_STATE,
			UIC_ARG_MPHY_RX_GEN_SEL_INDEX(0)),
			&rx_fsm_val);
	dev_err(hba->dev, "� : RX_FSM_STATE = � , err = �n", __func__,
			rx_fsm_val, err);
}

static void ufshcd_print_host_state(struct ufs_hba *hba)
{
	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_HOST_STATE_EN))
		return;

	dev_err(hba->dev, "UFS Host state=�n", hba->ufshcd_state);
	dev_err(hba->dev, "lrb in use=0x� x, outstanding reqs=0x� x tasks=0x� x\n",
		hba->lrb_in_use, hba->outstanding_tasks, hba->outstanding_reqs);
	dev_err(hba->dev, "saved_err=0x� , saved_uic_err=0x� , saved_ce_err=0x� \n",
		hba->saved_err, hba->saved_uic_err, hba->saved_ce_err);
	dev_err(hba->dev, "Device power mode=� UIC link state=�n",
		hba->curr_dev_pwr_mode, hba->uic_link_state);
	dev_err(hba->dev, "PM in progress=� sys. suspended=�n",
		hba->pm_op_in_progress, hba->is_sys_suspended);
	dev_err(hba->dev, "Auto BKOPS=� Host self-block=�n",
		hba->auto_bkops_enabled, hba->host->host_self_blocked);
	dev_err(hba->dev, "Clk gate=� hibern8 on idle=�n",
		hba->clk_gating.state, hba->hibern8_on_idle.state);
	dev_err(hba->dev, "error handling flags=0x� , req. abort count=�n",
		hba->eh_flags, hba->req_abort_count);
	dev_err(hba->dev, "Host capabilities=0x� , caps=0x� \n",
		hba->capabilities, hba->caps);
	dev_err(hba->dev, "quirks=0x� , dev. quirks=0x� \n", hba->quirks,
		hba->dev_quirks);
}

/**
 * ufshcd_print_pwr_info - print power params as saved in hba
 * power info
 * @hba: per-adapter instance
 */
static void ufshcd_print_pwr_info(struct ufs_hba *hba)
{
	char *names[] = {
		"INVALID MODE",
		"FAST MODE",
		"SLOW_MODE",
		"INVALID MODE",
		"FASTAUTO_MODE",
		"SLOWAUTO_MODE",
		"INVALID MODE",
	};

	if (!(hba->ufshcd_dbg_print & UFSHCD_DBG_PRINT_PWR_EN))
		return;

	dev_err(hba->dev, "� :[RX, TX]: gear=[� �, lane[� �, pwr[� , � ], rate = �n",
		 __func__,
		 hba->pwr_info.gear_rx, hba->pwr_info.gear_tx,
		 hba->pwr_info.lane_rx, hba->pwr_info.lane_tx,
		 names[hba->pwr_info.pwr_rx],
		 names[hba->pwr_info.pwr_tx],
		 hba->pwr_info.hs_rate);
}

/*
 * ufshcd_wait_for_register - wait for register value to change
 * @hba - per-adapter interface
 * @reg - mmio register offset
 * @mask - mask to apply to read register value
 * @val - wait condition
 * @interval_us - polling interval in microsecs
 * @timeout_ms - timeout in millisecs
 * @can_sleep - perform sleep or just spin
 * Returns -ETIMEDOUT on error, zero on success
 */
int ufshcd_wait_for_register(struct ufs_hba *hba, u32 reg, u32 mask,
				u32 val, unsigned long interval_us,
				unsigned long timeout_ms, bool can_sleep)
{
	int err = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	/* ignore bits that we don't intend to wait on */
	val = val & mask;

	while ((ufshcd_readl(hba, reg) & mask) != val) {
		if (can_sleep)
			usleep_range(interval_us, interval_us + 50);
		else
			udelay(interval_us);
		if (time_after(jiffies, timeout)) {
			if ((ufshcd_readl(hba, reg) & mask) != val)
				err = -ETIMEDOUT;
			break;
		}
	}

	return err;
}

/**
 * ufshcd_get_intr_mask - Get the interrupt bit mask
 * @hba - Pointer to adapter instance
 *
 * Returns interrupt bit mask per version
 */
static inline u32 ufshcd_get_intr_mask(struct ufs_hba *hba)
{
	u32 intr_mask = 0;

	switch (hba->ufs_version) {
	case UFSHCI_VERSION_10:
		intr_mask = INTERRUPT_MASK_ALL_VER_10;
		break;
	/* allow fall through */
	case UFSHCI_VERSION_11:
	case UFSHCI_VERSION_20:
		intr_mask = INTERRUPT_MASK_ALL_VER_11;
		break;
	/* allow fall through */
	case UFSHCI_VERSION_21:
	default:
		intr_mask = INTERRUPT_MASK_ALL_VER_21;
	}

	if (!ufshcd_is_crypto_supported(hba))
		intr_mask &= ~CRYPTO_ENGINE_FATAL_ERROR;

	return intr_mask;
}

/**
 * ufshcd_get_ufs_version - Get the UFS version supported by the HBA
 * @hba - Pointer to adapter instance
 *
 * Returns UFSHCI version supported by the controller
 */
static inline u32 ufshcd_get_ufs_version(struct ufs_hba *hba)
{
	if (hba->quirks & UFSHCD_QUIRK_BROKEN_UFS_HCI_VERSION)
		return ufshcd_vops_get_ufs_hci_version(hba);

	return ufshcd_readl(hba, REG_UFS_VERSION);
}

/**
 * ufshcd_is_device_present - Check if any device connected to
 *			      the host controller
 * @hba: pointer to adapter instance
 *
 * Returns 1 if device present, 0 if no device detected
 */
static inline int ufshcd_is_device_present(struct ufs_hba *hba)
{
	return (ufshcd_readl(hba, REG_CONTROLLER_STATUS) &
						DEVICE_PRESENT) ? 1 : 0;
}

/**
 * ufshcd_get_tr_ocs - Get the UTRD Overall Command Status
 * @lrb: pointer to local command reference block
 *
 * This function is used to get the OCS field from UTRD
 * Returns the OCS field in the UTRD
 */
static inline int ufshcd_get_tr_ocs(struct ufshcd_lrb *lrbp)
{
	return le32_to_cpu(lrbp->utr_descriptor_ptr->header.dword_2) & MASK_OCS;
}

/**
 * ufshcd_get_tmr_ocs - Get the UTMRD Overall Command Status
 * @task_req_descp: pointer to utp_task_req_desc structure
 *
 * This function is used to get the OCS field from UTMRD
 * Returns the OCS field in the UTMRD
 */
static inline int
ufshcd_get_tmr_ocs(struct utp_task_req_desc *task_req_descp)
{
	return le32_to_cpu(task_req_descp->header.dword_2) & MASK_OCS;
}

/**
 * ufshcd_get_tm_free_slot - get a free slot for task management request
 * @hba: per adapter instance
 * @free_slot: pointer to variable with available slot value
 *
 * Get a free tag and lock it until ufshcd_put_tm_slot() is called.
 * Returns 0 if free slot is not available, else return 1 with tag value
 * in @free_slot.
 */
static bool ufshcd_get_tm_free_slot(struct ufs_hba *hba, int *free_slot)
{
	int tag;
	bool ret = false;

	if (!free_slot)
		goto out;

	do {
		tag = find_first_zero_bit(&hba->tm_slots_in_use, hba->nutmrs);
		if (tag >= hba->nutmrs)
			goto out;
	} while (test_and_set_bit_lock(tag, &hba->tm_slots_in_use));

	*free_slot = tag;
	ret = true;
out:
	return ret;
}

static inline void ufshcd_put_tm_slot(struct ufs_hba *hba, int slot)
{
	clear_bit_unlock(slot, &hba->tm_slots_in_use);
}

/**
 * ufshcd_utrl_clear - Clear a bit in UTRLCLR register
 * @hba: per adapter instance
 * @pos: position of the bit to be cleared
 */
static inline void ufshcd_utrl_clear(struct ufs_hba *hba, u32 pos)
{
	ufshcd_writel(hba, ~(1 << pos), REG_UTP_TRANSFER_REQ_LIST_CLEAR);
}

/**
 * ufshcd_outstanding_req_clear - Clear a bit in outstanding request field
 * @hba: per adapter instance
 * @tag: position of the bit to be cleared
 */
static inline void ufshcd_outstanding_req_clear(struct ufs_hba *hba, int tag)
{
	__clear_bit(tag, &hba->outstanding_reqs);
}

/**
 * ufshcd_get_lists_status - Check UCRDY, UTRLRDY and UTMRLRDY
 * @reg: Register value of host controller status
 *
 * Returns integer, 0 on Success and positive value if failed
 */
static inline int ufshcd_get_lists_status(u32 reg)
{
	/*
	 * The mask 0xFF is for the following HCS register bits
	 * Bit		Description
	 *  0		Device Present
	 *  1		UTRLRDY
	 *  2		UTMRLRDY
	 *  3		UCRDY
	 * 4-7		reserved
	 */
	return ((reg & 0xFF) >> 1) ^ 0x07;
}

/**
 * ufshcd_get_uic_cmd_result - Get the UIC command result
 * @hba: Pointer to adapter instance
 *
 * This function gets the result of UIC command completion
 * Returns 0 on success, non zero value on error
 */
static inline int ufshcd_get_uic_cmd_result(struct ufs_hba *hba)
{
	return ufshcd_readl(hba, REG_UIC_COMMAND_ARG_2) &
	       MASK_UIC_COMMAND_RESULT;
}

/**
 * ufshcd_get_dme_attr_val - Get the value of attribute returned by UIC command
 * @hba: Pointer to adapter instance
 *
 * This function gets UIC command argument3
 * Returns 0 on success, non zero value on error
 */
static inline u32 ufshcd_get_dme_attr_val(struct ufs_hba *hba)
{
	return ufshcd_readl(hba, REG_UIC_COMMAND_ARG_3);
}

/**
 * ufshcd_get_req_rsp - returns the TR response transaction type
 * @ucd_rsp_ptr: pointer to response UPIU
 */
static inline int
ufshcd_get_req_rsp(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_0) >> 24;
}

/**
 * ufshcd_get_rsp_upiu_result - Get the result from response UPIU
 * @ucd_rsp_ptr: pointer to response UPIU
 *
 * This function gets the response status and scsi_status from response UPIU
 * Returns the response result code.
 */
static inline int
ufshcd_get_rsp_upiu_result(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_1) & MASK_RSP_UPIU_RESULT;
}

/*
 * ufshcd_get_rsp_upiu_data_seg_len - Get the data segment length
 *				from response UPIU
 * @ucd_rsp_ptr: pointer to response UPIU
 *
 * Return the data segment length.
 */
static inline unsigned int
ufshcd_get_rsp_upiu_data_seg_len(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_2) &
		MASK_RSP_UPIU_DATA_SEG_LEN;
}

/**
 * ufshcd_is_exception_event - Check if the device raised an exception event
 * @ucd_rsp_ptr: pointer to response UPIU
 *
 * The function checks if the device raised an exception event indicated in
 * the Device Information field of response UPIU.
 *
 * Returns true if exception is raised, false otherwise.
 */
static inline bool ufshcd_is_exception_event(struct utp_upiu_rsp *ucd_rsp_ptr)
{
	return be32_to_cpu(ucd_rsp_ptr->header.dword_2) &
			MASK_RSP_EXCEPTION_EVENT ? true : false;
}

/**
 * ufshcd_reset_intr_aggr - Reset interrupt aggregation values.
 * @hba: per adapter instance
 */
static inline void
ufshcd_reset_intr_aggr(struct ufs_hba *hba)
{
	ufshcd_writel(hba, INT_AGGR_ENABLE |
		      INT_AGGR_COUNTER_AND_TIMER_RESET,
		      REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL);
}

/**
 * ufshcd_config_intr_aggr - Configure interrupt aggregation values.
 * @hba: per adapter instance
 * @cnt: Interrupt aggregation counter threshold
 * @tmout: Interrupt aggregation timeout value
 */
static inline void
ufshcd_config_intr_aggr(struct ufs_hba *hba, u8 cnt, u8 tmout)
{
	ufshcd_writel(hba, INT_AGGR_ENABLE | INT_AGGR_PARAM_WRITE |
		      INT_AGGR_COUNTER_THLD_VAL(cnt) |
		      INT_AGGR_TIMEOUT_VAL(tmout),
		      REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL);
}

/**
 * ufshcd_disable_intr_aggr - Disables interrupt aggregation.
 * @hba: per adapter instance
 */
static inline void ufshcd_disable_intr_aggr(struct ufs_hba *hba)
{
	ufshcd_writel(hba, 0, REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL);
}

/**
 * ufshcd_enable_run_stop_reg - Enable run-stop registers,
 *			When run-stop registers are set to 1, it indicates the
 *			host controller that it can process the requests
 * @hba: per adapter instance
 */
static void ufshcd_enable_run_stop_reg(struct ufs_hba *hba)
{
	ufshcd_writel(hba, UTP_TASK_REQ_LIST_RUN_STOP_BIT,
		      REG_UTP_TASK_REQ_LIST_RUN_STOP);
	ufshcd_writel(hba, UTP_TRANSFER_REQ_LIST_RUN_STOP_BIT,
		      REG_UTP_TRANSFER_REQ_LIST_RUN_STOP);
}

/**
 * ufshcd_hba_start - Start controller initialization sequence
 * @hba: per adapter instance
 */
static inline void ufshcd_hba_start(struct ufs_hba *hba)
{
	u32 val = CONTROLLER_ENABLE;

	if (ufshcd_is_crypto_supported(hba))
		val |= CRYPTO_GENERAL_ENABLE;
	ufshcd_writel(hba, val, REG_CONTROLLER_ENABLE);
}

/**
 * ufshcd_is_hba_active - Get controller state
 * @hba: per adapter instance
 *
 * Returns zero if controller is active, 1 otherwise
 */
static inline int ufshcd_is_hba_active(struct ufs_hba *hba)
{
	return (ufshcd_readl(hba, REG_CONTROLLER_ENABLE) & 0x1) ? 0 : 1;
}

static const char *ufschd_uic_link_state_to_string(
			enum uic_link_state state)
{
	switch (state) {
	case UIC_LINK_OFF_STATE:	return "OFF";
	case UIC_LINK_ACTIVE_STATE:	return "ACTIVE";
	case UIC_LINK_HIBERN8_STATE:	return "HIBERN8";
	default:			return "UNKNOWN";
	}
}

static const char *ufschd_ufs_dev_pwr_mode_to_string(
			enum ufs_dev_pwr_mode state)
{
	switch (state) {
	case UFS_ACTIVE_PWR_MODE:	return "ACTIVE";
	case UFS_SLEEP_PWR_MODE:	return "SLEEP";
	case UFS_POWERDOWN_PWR_MODE:	return "POWERDOWN";
	default:			return "UNKNOWN";
	}
}

u32 ufshcd_get_local_unipro_ver(struct ufs_hba *hba)
{
	/* HCI version 1.0 and 1.1 supports UniPro 1.41 */
	if ((hba->ufs_version == UFSHCI_VERSION_10) ||
	    (hba->ufs_version == UFSHCI_VERSION_11))
		return UFS_UNIPRO_VER_1_41;
	else
		return UFS_UNIPRO_VER_1_6;
}
EXPORT_SYMBOL(ufshcd_get_local_unipro_ver);

static bool ufshcd_is_unipro_pa_params_tuning_req(struct ufs_hba *hba)
{
	/*
	 * If both host and device support UniPro ver1.6 or later, PA layer
	 * parameters tuning happens during link startup itself.
	 *
	 * We can manually tune PA layer parameters if either host or device
	 * doesn't support UniPro ver 1.6 or later. But to keep manual tuning
	 * logic simple, we will only do manual tuning if local unipro version
	 * doesn't support ver1.6 or later.
	 */
	if (ufshcd_get_local_unipro_ver(hba) < UFS_UNIPRO_VER_1_6)
		return true;
	else
		return false;
}

/**
 * ufshcd_set_clk_freq - set UFS controller clock frequencies
 * @hba: per adapter instance
 * @scale_up: If True, set max possible frequency othewise set low frequency
 *
 * Returns 0 if successful
 * Returns < 0 for any other errors
 */
static int ufshcd_set_clk_freq(struct ufs_hba *hba, bool scale_up)
{
	int ret = 0;
	struct ufs_clk_info *clki;
	struct list_head *head = &hba->clk_list_head;

	if (!head || list_empty(head))
		goto out;

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR_OR_NULL(clki->clk)) {
			if (scale_up && clki->max_freq) {
				if (clki->curr_freq == clki->max_freq)
					continue;

				ret = clk_set_rate(clki->clk, clki->max_freq);
				if (ret) {
					dev_err(hba->dev, "� : �  clk set rate(�z) failed, �n",
						__func__, clki->name,
						clki->max_freq, ret);
					break;
				}
				trace_ufshcd_clk_scaling(dev_name(hba->dev),
						"scaled up", clki->name,
						clki->curr_freq,
						clki->max_freq);
				clki->curr_freq = clki->max_freq;

			} else if (!scale_up && clki->min_freq) {
				if (clki->curr_freq == clki->min_freq)
					continue;

				ret = clk_set_rate(clki->clk, clki->min_freq);
				if (ret) {
					dev_err(hba->dev, "� : �  clk set rate(�z) failed, �n",
						__func__, clki->name,
						clki->min_freq, ret);
					break;
				}
				trace_ufshcd_clk_scaling(dev_name(hba->dev),
						"scaled down", clki->name,
						clki->curr_freq,
						clki->min_freq);
				clki->curr_freq = clki->min_freq;
			}
		}
		dev_dbg(hba->dev, "� : clk: � , rate: � u\n", __func__,
				clki->name, clk_get_rate(clki->clk));
	}

out:
	return ret;
}

/**
 * ufshcd_scale_clks - scale up or scale down UFS controller clocks
 * @hba: per adapter instance
 * @scale_up: True if scaling up and false if scaling down
 *
 * Returns 0 if successful
 * Returns < 0 for any other errors
 */
static int ufshcd_scale_clks(struct ufs_hba *hba, bool scale_up)
{
	int ret = 0;

	ret = ufshcd_vops_clk_scale_notify(hba, scale_up, PRE_CHANGE);
	if (ret)
		return ret;

	ret = ufshcd_set_clk_freq(hba, scale_up);
	if (ret)
		return ret;

	ret = ufshcd_vops_clk_scale_notify(hba, scale_up, POST_CHANGE);
	if (ret) {
		ufshcd_set_clk_freq(hba, !scale_up);
		return ret;
	}

	return ret;
}

static inline void ufshcd_cancel_gate_work(struct ufs_hba *hba)
{
	hrtimer_cancel(&hba->clk_gating.gate_hrtimer);
	cancel_work_sync(&hba->clk_gating.gate_work);
}

static void ufshcd_ungate_work(struct work_struct *work)
{
	int ret;
	unsigned long flags;
	struct ufs_hba *hba = container_of(work, struct ufs_hba,
			clk_gating.ungate_work);

	ufshcd_cancel_gate_work(hba);

	spin_lock_irqsave(hba->host->host_lock, flags);
	if (hba->clk_gating.state == CLKS_ON) {
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		goto unblock_reqs;
	}

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	ufshcd_hba_vreg_set_hpm(hba);
	ufshcd_enable_clocks(hba);

	/* Exit from hibern8 */
	if (ufshcd_can_hibern8_during_gating(hba)) {
		/* Prevent gating in this path */
		hba->clk_gating.is_suspended = true;
		if (ufshcd_is_link_hibern8(hba)) {
			ret = ufshcd_uic_hibern8_exit(hba);
			if (ret)
				dev_err(hba->dev, "� : hibern8 exit failed �n",
					__func__, ret);
			else
				ufshcd_set_link_active(hba);
		}
		hba->clk_gating.is_suspended = false;
	}
unblock_reqs:
	ufshcd_scsi_unblock_requests(hba);
}

/**
 * ufshcd_hold - Enable clocks that were gated earlier due to ufshcd_release.
 * Also, exit from hibern8 mode and set the link as active.
 * @hba: per adapter instance
 * @async: This indicates whether caller should ungate clocks asynchronously.
 */
int ufshcd_hold(struct ufs_hba *hba, bool async)
{
	int rc = 0;
	bool flush_result;
	unsigned long flags;

	if (!ufshcd_is_clkgating_allowed(hba))
		goto out;
	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->clk_gating.active_reqs++;

	if (ufshcd_eh_in_progress(hba)) {
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		return 0;
	}

start:
	switch (hba->clk_gating.state) {
	case CLKS_ON:
		/*
		 * Wait for the ungate work to complete if in progress.
		 * Though the clocks may be in ON state, the link could
		 * still be in hibner8 state if hibern8 is allowed
		 * during clock gating.
		 * Make sure we exit hibern8 state also in addition to
		 * clocks being ON.
		 */
		if (ufshcd_can_hibern8_during_gating(hba) &&
		    ufshcd_is_link_hibern8(hba)) {
			if (async) {
				rc = -EAGAIN;
				hba->clk_gating.active_reqs--;
				break;
			}
			spin_unlock_irqrestore(hba->host->host_lock, flags);
			flush_result = flush_work(&hba->clk_gating.ungate_work);
			if (hba->clk_gating.is_suspended && !flush_result)
				goto out;
			spin_lock_irqsave(hba->host->host_lock, flags);
			goto start;
		}
		break;
	case REQ_CLKS_OFF:
		/*
		 * If the timer was active but the callback was not running
		 * we have nothing to do, just change state and return.
		 */
		if (hrtimer_try_to_cancel(&hba->clk_gating.gate_hrtimer) == 1) {
			hba->clk_gating.state = CLKS_ON;
			trace_ufshcd_clk_gating(dev_name(hba->dev),
				hba->clk_gating.state);
			break;
		}
		/*
		 * If we are here, it means gating work is either done or
		 * currently running. Hence, fall through to cancel gating
		 * work and to enable clocks.
		 */
	case CLKS_OFF:
		__ufshcd_scsi_block_requests(hba);
		hba->clk_gating.state = REQ_CLKS_ON;
		trace_ufshcd_clk_gating(dev_name(hba->dev),
			hba->clk_gating.state);
		queue_work(hba->clk_gating.clk_gating_workq,
				&hba->clk_gating.ungate_work);
		/*
		 * fall through to check if we should wait for this
		 * work to be done or not.
		 */
	case REQ_CLKS_ON:
		if (async) {
			rc = -EAGAIN;
			hba->clk_gating.active_reqs--;
			break;
		}

		spin_unlock_irqrestore(hba->host->host_lock, flags);
		flush_work(&hba->clk_gating.ungate_work);
		/* Make sure state is CLKS_ON before returning */
		spin_lock_irqsave(hba->host->host_lock, flags);
		goto start;
	default:
		dev_err(hba->dev, "� : clk gating is in invalid state �n",
				__func__, hba->clk_gating.state);
		break;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	hba->ufs_stats.clk_hold.ts = ktime_get();
	return rc;
}
EXPORT_SYMBOL_GPL(ufshcd_hold);

static void ufshcd_gate_work(struct work_struct *work)
{
	struct ufs_hba *hba = container_of(work, struct ufs_hba,
						clk_gating.gate_work);
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	/*
	 * In case you are here to cancel this work the gating state
	 * would be marked as REQ_CLKS_ON. In this case save time by
	 * skipping the gating work and exit after changing the clock
	 * state to CLKS_ON.
	 */
	if (hba->clk_gating.is_suspended ||
		(hba->clk_gating.state != REQ_CLKS_OFF)) {
		hba->clk_gating.state = CLKS_ON;
		trace_ufshcd_clk_gating(dev_name(hba->dev),
			hba->clk_gating.state);
		goto rel_lock;
	}

	if (hba->clk_gating.active_reqs
		|| hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL
		|| hba->lrb_in_use || hba->outstanding_tasks
		|| hba->active_uic_cmd || hba->uic_async_done)
		goto rel_lock;

	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (ufshcd_is_hibern8_on_idle_allowed(hba) &&
	    hba->hibern8_on_idle.is_enabled)
		/*
		 * Hibern8 enter work (on Idle) needs clocks to be ON hence
		 * make sure that it is flushed before turning off the clocks.
		 */
		flush_delayed_work(&hba->hibern8_on_idle.enter_work);

	/* put the link into hibern8 mode before turning off clocks */
	if (ufshcd_can_hibern8_during_gating(hba)) {
		if (ufshcd_uic_hibern8_enter(hba)) {
			hba->clk_gating.state = CLKS_ON;
			trace_ufshcd_clk_gating(dev_name(hba->dev),
				hba->clk_gating.state);
			goto out;
		}
		ufshcd_set_link_hibern8(hba);
	}

	/*
	 * If auto hibern8 is supported then the link will already
	 * be in hibern8 state and the ref clock can be gated.
	 */
	if ((ufshcd_is_auto_hibern8_supported(hba) ||
	     !ufshcd_is_link_active(hba)) && !hba->no_ref_clk_gating)
		ufshcd_disable_clocks(hba, true);
	else
		/* If link is active, device ref_clk can't be switched off */
		ufshcd_disable_clocks_skip_ref_clk(hba, true);

	/* Put the host controller in low power mode if possible */
	ufshcd_hba_vreg_set_lpm(hba);

	/*
	 * In case you are here to cancel this work the gating state
	 * would be marked as REQ_CLKS_ON. In this case keep the state
	 * as REQ_CLKS_ON which would anyway imply that clocks are off
	 * and a request to turn them on is pending. By doing this way,
	 * we keep the state machine in tact and this would ultimately
	 * prevent from doing cancel work multiple times when there are
	 * new requests arriving before the current cancel work is done.
	 */
	spin_lock_irqsave(hba->host->host_lock, flags);
	if (hba->clk_gating.state == REQ_CLKS_OFF) {
		hba->clk_gating.state = CLKS_OFF;
		trace_ufshcd_clk_gating(dev_name(hba->dev),
			hba->clk_gating.state);
	}
rel_lock:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	return;
}

/* host lock must be held before calling this variant */
static void __ufshcd_release(struct ufs_hba *hba, bool no_sched)
{
	if (!ufshcd_is_clkgating_allowed(hba))
		return;

	hba->clk_gating.active_reqs--;

	if (hba->clk_gating.active_reqs || hba->clk_gating.is_suspended
		|| hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL
		|| hba->lrb_in_use || hba->outstanding_tasks
		|| hba->active_uic_cmd || hba->uic_async_done
		|| ufshcd_eh_in_progress(hba) || no_sched)
		return;

	hba->clk_gating.state = REQ_CLKS_OFF;
	trace_ufshcd_clk_gating(dev_name(hba->dev), hba->clk_gating.state);
	hba->ufs_stats.clk_rel.ts = ktime_get();

	hrtimer_start(&hba->clk_gating.gate_hrtimer,
			ms_to_ktime(hba->clk_gating.delay_ms),
			HRTIMER_MODE_REL);
}

void ufshcd_release(struct ufs_hba *hba, bool no_sched)
{
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	__ufshcd_release(hba, no_sched);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
}
EXPORT_SYMBOL_GPL(ufshcd_release);

static ssize_t ufshcd_clkgate_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "� u\n", hba->clk_gating.delay_ms);
}

static ssize_t ufshcd_clkgate_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->clk_gating.delay_ms = value;
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	return count;
}

static ssize_t ufshcd_clkgate_delay_pwr_save_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "� u\n",
			hba->clk_gating.delay_ms_pwr_save);
}

static ssize_t ufshcd_clkgate_delay_pwr_save_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);

	hba->clk_gating.delay_ms_pwr_save = value;
	if (ufshcd_is_clkscaling_supported(hba) &&
	    !hba->clk_scaling.is_scaled_up)
		hba->clk_gating.delay_ms = hba->clk_gating.delay_ms_pwr_save;

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	return count;
}

static ssize_t ufshcd_clkgate_delay_perf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "� u\n", hba->clk_gating.delay_ms_perf);
}

static ssize_t ufshcd_clkgate_delay_perf_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);

	hba->clk_gating.delay_ms_perf = value;
	if (ufshcd_is_clkscaling_supported(hba) &&
	    hba->clk_scaling.is_scaled_up)
		hba->clk_gating.delay_ms = hba->clk_gating.delay_ms_perf;

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	return count;
}

static ssize_t ufshcd_clkgate_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "�n", hba->clk_gating.is_enabled);
}

static ssize_t ufshcd_clkgate_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	u32 value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	value = !!value;
	if (value == hba->clk_gating.is_enabled)
		goto out;

	if (value) {
		ufshcd_release(hba, false);
	} else {
		spin_lock_irqsave(hba->host->host_lock, flags);
		hba->clk_gating.active_reqs++;
		spin_unlock_irqrestore(hba->host->host_lock, flags);
	}

	hba->clk_gating.is_enabled = value;
out:
	return count;
}

static enum hrtimer_restart ufshcd_clkgate_hrtimer_handler(
					struct hrtimer *timer)
{
	struct ufs_hba *hba = container_of(timer, struct ufs_hba,
					   clk_gating.gate_hrtimer);

	queue_work(hba->clk_gating.clk_gating_workq,
				&hba->clk_gating.gate_work);

	return HRTIMER_NORESTART;
}

static void ufshcd_init_clk_gating(struct ufs_hba *hba)
{
	struct ufs_clk_gating *gating = &hba->clk_gating;
	char wq_name[sizeof("ufs_clk_gating_00")];

	hba->clk_gating.state = CLKS_ON;

	if (!ufshcd_is_clkgating_allowed(hba))
		return;

	/*
	 * Disable hibern8 during clk gating if
	 * auto hibern8 is supported
	 */
	if (ufshcd_is_auto_hibern8_supported(hba))
		hba->caps &= ~UFSHCD_CAP_HIBERN8_WITH_CLK_GATING;

	INIT_WORK(&gating->gate_work, ufshcd_gate_work);
	INIT_WORK(&gating->ungate_work, ufshcd_ungate_work);
	/*
	 * Clock gating work must be executed only after auto hibern8
	 * timeout has expired in the hardware or after aggressive
	 * hibern8 on idle software timeout. Using jiffy based low
	 * resolution delayed work is not reliable to guarantee this,
	 * hence use a high resolution timer to make sure we schedule
	 * the gate work precisely more than hibern8 timeout.
	 *
	 * Always make sure gating->delay_ms > hibern8_on_idle->delay_ms
	 */
	hrtimer_init(&gating->gate_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gating->gate_hrtimer.function = ufshcd_clkgate_hrtimer_handler;

	snprintf(wq_name, ARRAY_SIZE(wq_name), "ufs_clk_gating_�,
			hba->host->host_no);
	hba->clk_gating.clk_gating_workq =
		create_singlethread_workqueue(wq_name);

	gating->is_enabled = true;

	gating->delay_ms_pwr_save = UFSHCD_CLK_GATING_DELAY_MS_PWR_SAVE;
	gating->delay_ms_perf = UFSHCD_CLK_GATING_DELAY_MS_PERF;

	/* start with performance mode */
	gating->delay_ms = gating->delay_ms_perf;

	if (!ufshcd_is_clkscaling_supported(hba))
		goto scaling_not_supported;

	gating->delay_pwr_save_attr.show = ufshcd_clkgate_delay_pwr_save_show;
	gating->delay_pwr_save_attr.store = ufshcd_clkgate_delay_pwr_save_store;
	sysfs_attr_init(&gating->delay_pwr_save_attr.attr);
	gating->delay_pwr_save_attr.attr.name = "clkgate_delay_ms_pwr_save";
	gating->delay_pwr_save_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &gating->delay_pwr_save_attr))
		dev_err(hba->dev, "Failed to create sysfs for clkgate_delay_ms_pwr_save\n");

	gating->delay_perf_attr.show = ufshcd_clkgate_delay_perf_show;
	gating->delay_perf_attr.store = ufshcd_clkgate_delay_perf_store;
	sysfs_attr_init(&gating->delay_perf_attr.attr);
	gating->delay_perf_attr.attr.name = "clkgate_delay_ms_perf";
	gating->delay_perf_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &gating->delay_perf_attr))
		dev_err(hba->dev, "Failed to create sysfs for clkgate_delay_ms_perf\n");

	goto add_clkgate_enable;

scaling_not_supported:
	hba->clk_gating.delay_attr.show = ufshcd_clkgate_delay_show;
	hba->clk_gating.delay_attr.store = ufshcd_clkgate_delay_store;
	sysfs_attr_init(&hba->clk_gating.delay_attr.attr);
	hba->clk_gating.delay_attr.attr.name = "clkgate_delay_ms";
	hba->clk_gating.delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &hba->clk_gating.delay_attr))
		dev_err(hba->dev, "Failed to create sysfs for clkgate_delay\n");

add_clkgate_enable:
	gating->enable_attr.show = ufshcd_clkgate_enable_show;
	gating->enable_attr.store = ufshcd_clkgate_enable_store;
	sysfs_attr_init(&gating->enable_attr.attr);
	gating->enable_attr.attr.name = "clkgate_enable";
	gating->enable_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &gating->enable_attr))
		dev_err(hba->dev, "Failed to create sysfs for clkgate_enable\n");
}

static void ufshcd_exit_clk_gating(struct ufs_hba *hba)
{
	if (!ufshcd_is_clkgating_allowed(hba))
		return;
	if (ufshcd_is_clkscaling_supported(hba)) {
		device_remove_file(hba->dev,
				   &hba->clk_gating.delay_pwr_save_attr);
		device_remove_file(hba->dev, &hba->clk_gating.delay_perf_attr);
	} else {
		device_remove_file(hba->dev, &hba->clk_gating.delay_attr);
	}
	device_remove_file(hba->dev, &hba->clk_gating.enable_attr);
	ufshcd_cancel_gate_work(hba);
	cancel_work_sync(&hba->clk_gating.ungate_work);
	destroy_workqueue(hba->clk_gating.clk_gating_workq);
}

static void ufshcd_set_auto_hibern8_timer(struct ufs_hba *hba, u32 delay)
{
	ufshcd_rmwl(hba, AUTO_HIBERN8_TIMER_SCALE_MASK |
			 AUTO_HIBERN8_IDLE_TIMER_MASK,
			AUTO_HIBERN8_TIMER_SCALE_1_MS | delay,
			REG_AUTO_HIBERN8_IDLE_TIMER);
	/* Make sure the timer gets applied before further operations */
	mb();
}

/**
 * ufshcd_hibern8_hold - Make sure that link is not in hibern8.
 *
 * @hba: per adapter instance
 * @async: This indicates whether caller wants to exit hibern8 asynchronously.
 *
 * Exit from hibern8 mode and set the link as active.
 *
 * Return 0 on success, non-zero on failure.
 */
static int ufshcd_hibern8_hold(struct ufs_hba *hba, bool async)
{
	int rc = 0;
	unsigned long flags;

	if (!ufshcd_is_hibern8_on_idle_allowed(hba))
		goto out;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->hibern8_on_idle.active_reqs++;

	if (ufshcd_eh_in_progress(hba)) {
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		return 0;
	}

start:
	switch (hba->hibern8_on_idle.state) {
	case HIBERN8_EXITED:
		break;
	case REQ_HIBERN8_ENTER:
		if (cancel_delayed_work(&hba->hibern8_on_idle.enter_work)) {
			hba->hibern8_on_idle.state = HIBERN8_EXITED;
			trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
				hba->hibern8_on_idle.state);
			break;
		}
		/*
		 * If we here, it means Hibern8 enter work is either done or
		 * currently running. Hence, fall through to cancel hibern8
		 * work and exit hibern8.
		 */
	case HIBERN8_ENTERED:
		__ufshcd_scsi_block_requests(hba);
		hba->hibern8_on_idle.state = REQ_HIBERN8_EXIT;
		trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
			hba->hibern8_on_idle.state);
		schedule_work(&hba->hibern8_on_idle.exit_work);
		/*
		 * fall through to check if we should wait for this
		 * work to be done or not.
		 */
	case REQ_HIBERN8_EXIT:
		if (async) {
			rc = -EAGAIN;
			hba->hibern8_on_idle.active_reqs--;
			break;
		} else {
			spin_unlock_irqrestore(hba->host->host_lock, flags);
			flush_work(&hba->hibern8_on_idle.exit_work);
			/* Make sure state is HIBERN8_EXITED before returning */
			spin_lock_irqsave(hba->host->host_lock, flags);
			goto start;
		}
	default:
		dev_err(hba->dev, "� : H8 is in invalid state �n",
				__func__, hba->hibern8_on_idle.state);
		break;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	return rc;
}

/* host lock must be held before calling this variant */
static void __ufshcd_hibern8_release(struct ufs_hba *hba, bool no_sched)
{
	unsigned long delay_in_jiffies;

	if (!ufshcd_is_hibern8_on_idle_allowed(hba))
		return;

	hba->hibern8_on_idle.active_reqs--;
	BUG_ON(hba->hibern8_on_idle.active_reqs < 0);

	if (hba->hibern8_on_idle.active_reqs
		|| hba->hibern8_on_idle.is_suspended
		|| hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL
		|| hba->lrb_in_use || hba->outstanding_tasks
		|| hba->active_uic_cmd || hba->uic_async_done
		|| ufshcd_eh_in_progress(hba) || no_sched)
		return;

	hba->hibern8_on_idle.state = REQ_HIBERN8_ENTER;
	trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
		hba->hibern8_on_idle.state);
	/*
	 * Scheduling the delayed work after 1 jiffies will make the work to
	 * get schedule any time from 0ms to 1000/HZ ms which is not desirable
	 * for hibern8 enter work as it may impact the performance if it gets
	 * scheduled almost immediately. Hence make sure that hibern8 enter
	 * work gets scheduled atleast after 2 jiffies (any time between
	 * 1000/HZ ms to 2000/HZ ms).
	 */
	delay_in_jiffies = msecs_to_jiffies(hba->hibern8_on_idle.delay_ms);
	if (delay_in_jiffies == 1)
		delay_in_jiffies++;

	schedule_delayed_work(&hba->hibern8_on_idle.enter_work,
			      delay_in_jiffies);
}

static void ufshcd_hibern8_release(struct ufs_hba *hba, bool no_sched)
{
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	__ufshcd_hibern8_release(hba, no_sched);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
}

static void ufshcd_hibern8_enter_work(struct work_struct *work)
{
	struct ufs_hba *hba = container_of(work, struct ufs_hba,
					   hibern8_on_idle.enter_work.work);
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	if (hba->hibern8_on_idle.is_suspended) {
		hba->hibern8_on_idle.state = HIBERN8_EXITED;
		trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
			hba->hibern8_on_idle.state);
		goto rel_lock;
	}

	if (hba->hibern8_on_idle.active_reqs
		|| hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL
		|| hba->lrb_in_use || hba->outstanding_tasks
		|| hba->active_uic_cmd || hba->uic_async_done)
		goto rel_lock;

	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (ufshcd_is_link_active(hba) && ufshcd_uic_hibern8_enter(hba)) {
		/* Enter failed */
		hba->hibern8_on_idle.state = HIBERN8_EXITED;
		trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
			hba->hibern8_on_idle.state);
		goto out;
	}
	ufshcd_set_link_hibern8(hba);

	/*
	 * In case you are here to cancel this work the hibern8_on_idle.state
	 * would be marked as REQ_HIBERN8_EXIT. In this case keep the state
	 * as REQ_HIBERN8_EXIT which would anyway imply that we are in hibern8
	 * and a request to exit from it is pending. By doing this way,
	 * we keep the state machine in tact and this would ultimately
	 * prevent from doing cancel work multiple times when there are
	 * new requests arriving before the current cancel work is done.
	 */
	spin_lock_irqsave(hba->host->host_lock, flags);
	if (hba->hibern8_on_idle.state == REQ_HIBERN8_ENTER) {
		hba->hibern8_on_idle.state = HIBERN8_ENTERED;
		trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
			hba->hibern8_on_idle.state);
	}
rel_lock:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	return;
}

static void __ufshcd_set_auto_hibern8_timer(struct ufs_hba *hba,
					    unsigned long delay_ms)
{
	pm_runtime_get_sync(hba->dev);
	ufshcd_hold_all(hba);
	ufshcd_scsi_block_requests(hba);
	down_write(&hba->lock);
	/* wait for all the outstanding requests to finish */
	ufshcd_wait_for_doorbell_clr(hba, U64_MAX);
	ufshcd_set_auto_hibern8_timer(hba, delay_ms);
	up_write(&hba->lock);
	ufshcd_scsi_unblock_requests(hba);
	ufshcd_release_all(hba);
	pm_runtime_put_sync(hba->dev);
}

static void ufshcd_hibern8_exit_work(struct work_struct *work)
{
	int ret;
	unsigned long flags;
	struct ufs_hba *hba = container_of(work, struct ufs_hba,
					   hibern8_on_idle.exit_work);

	cancel_delayed_work_sync(&hba->hibern8_on_idle.enter_work);

	spin_lock_irqsave(hba->host->host_lock, flags);
	if ((hba->hibern8_on_idle.state == HIBERN8_EXITED)
	     || ufshcd_is_link_active(hba)) {
		hba->hibern8_on_idle.state = HIBERN8_EXITED;
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		goto unblock_reqs;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	/* Exit from hibern8 */
	if (ufshcd_is_link_hibern8(hba)) {
		hba->ufs_stats.clk_hold.ctx = H8_EXIT_WORK;
		ufshcd_hold(hba, false);
		ret = ufshcd_uic_hibern8_exit(hba);
		hba->ufs_stats.clk_rel.ctx = H8_EXIT_WORK;
		ufshcd_release(hba, false);
		if (!ret) {
			spin_lock_irqsave(hba->host->host_lock, flags);
			ufshcd_set_link_active(hba);
			hba->hibern8_on_idle.state = HIBERN8_EXITED;
			trace_ufshcd_hibern8_on_idle(dev_name(hba->dev),
				hba->hibern8_on_idle.state);
			spin_unlock_irqrestore(hba->host->host_lock, flags);
		}
	}
unblock_reqs:
	ufshcd_scsi_unblock_requests(hba);
}

static ssize_t ufshcd_hibern8_on_idle_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "� u\n", hba->hibern8_on_idle.delay_ms);
}

static ssize_t ufshcd_hibern8_on_idle_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags, value;
	bool change = true;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);
	if (hba->hibern8_on_idle.delay_ms == value)
		change = false;

	if (value >= hba->clk_gating.delay_ms_pwr_save ||
	    value >= hba->clk_gating.delay_ms_perf) {
		dev_err(hba->dev, "hibern8_on_idle_delay (� u) can not be >= to clkgate_delay_ms_pwr_save (� u) and clkgate_delay_ms_perf (� u)\n",
			value, hba->clk_gating.delay_ms_pwr_save,
			hba->clk_gating.delay_ms_perf);
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		return -EINVAL;
	}

	hba->hibern8_on_idle.delay_ms = value;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	/* Update auto hibern8 timer value if supported */
	if (change && ufshcd_is_auto_hibern8_supported(hba) &&
	    hba->hibern8_on_idle.is_enabled)
		__ufshcd_set_auto_hibern8_timer(hba,
						hba->hibern8_on_idle.delay_ms);

	return count;
}

static ssize_t ufshcd_hibern8_on_idle_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "�n",
			hba->hibern8_on_idle.is_enabled);
}

static ssize_t ufshcd_hibern8_on_idle_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	u32 value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	value = !!value;
	if (value == hba->hibern8_on_idle.is_enabled)
		goto out;

	/* Update auto hibern8 timer value if supported */
	if (ufshcd_is_auto_hibern8_supported(hba)) {
		__ufshcd_set_auto_hibern8_timer(hba,
			value ? hba->hibern8_on_idle.delay_ms : value);
		goto update;
	}

	if (value) {
		/*
		 * As clock gating work would wait for the hibern8 enter work
		 * to finish, clocks would remain on during hibern8 enter work.
		 */
		ufshcd_hold(hba, false);
		ufshcd_release_all(hba);
	} else {
		spin_lock_irqsave(hba->host->host_lock, flags);
		hba->hibern8_on_idle.active_reqs++;
		spin_unlock_irqrestore(hba->host->host_lock, flags);
	}

update:
	hba->hibern8_on_idle.is_enabled = value;
out:
	return count;
}

static void ufshcd_init_hibern8_on_idle(struct ufs_hba *hba)
{
	/* initialize the state variable here */
	hba->hibern8_on_idle.state = HIBERN8_EXITED;

	if (!ufshcd_is_hibern8_on_idle_allowed(hba) &&
	    !ufshcd_is_auto_hibern8_supported(hba))
		return;

	if (ufshcd_is_auto_hibern8_supported(hba)) {
		hba->hibern8_on_idle.delay_ms = 1;
		hba->hibern8_on_idle.state = AUTO_HIBERN8;
		/*
		 * Disable SW hibern8 enter on idle in case
		 * auto hibern8 is supported
		 */
		hba->caps &= ~UFSHCD_CAP_HIBERN8_ENTER_ON_IDLE;
	} else {
		hba->hibern8_on_idle.delay_ms = 10;
		INIT_DELAYED_WORK(&hba->hibern8_on_idle.enter_work,
				  ufshcd_hibern8_enter_work);
		INIT_WORK(&hba->hibern8_on_idle.exit_work,
			  ufshcd_hibern8_exit_work);
	}

	hba->hibern8_on_idle.is_enabled = true;

	hba->hibern8_on_idle.delay_attr.show =
					ufshcd_hibern8_on_idle_delay_show;
	hba->hibern8_on_idle.delay_attr.store =
					ufshcd_hibern8_on_idle_delay_store;
	sysfs_attr_init(&hba->hibern8_on_idle.delay_attr.attr);
	hba->hibern8_on_idle.delay_attr.attr.name = "hibern8_on_idle_delay_ms";
	hba->hibern8_on_idle.delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &hba->hibern8_on_idle.delay_attr))
		dev_err(hba->dev, "Failed to create sysfs for hibern8_on_idle_delay\n");

	hba->hibern8_on_idle.enable_attr.show =
					ufshcd_hibern8_on_idle_enable_show;
	hba->hibern8_on_idle.enable_attr.store =
					ufshcd_hibern8_on_idle_enable_store;
	sysfs_attr_init(&hba->hibern8_on_idle.enable_attr.attr);
	hba->hibern8_on_idle.enable_attr.attr.name = "hibern8_on_idle_enable";
	hba->hibern8_on_idle.enable_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(hba->dev, &hba->hibern8_on_idle.enable_attr))
		dev_err(hba->dev, "Failed to create sysfs for hibern8_on_idle_enable\n");
}

static void ufshcd_exit_hibern8_on_idle(struct ufs_hba *hba)
{
	if (!ufshcd_is_hibern8_on_idle_allowed(hba) &&
	    !ufshcd_is_auto_hibern8_supported(hba))
		return;
	device_remove_file(hba->dev, &hba->hibern8_on_idle.delay_attr);
	device_remove_file(hba->dev, &hba->hibern8_on_idle.enable_attr);
}

static void ufshcd_hold_all(struct ufs_hba *hba)
{
	ufshcd_hold(hba, false);
	ufshcd_hibern8_hold(hba, false);
}

static void ufshcd_release_all(struct ufs_hba *hba)
{
	ufshcd_hibern8_release(hba, false);
	ufshcd_release(hba, false);
}

/* Must be called with host lock acquired */
static void ufshcd_clk_scaling_start_busy(struct ufs_hba *hba)
{
	bool queue_resume_work = false;

	if (!ufshcd_is_clkscaling_supported(hba))
		return;

	if (!hba->clk_scaling.active_reqs++)
		queue_resume_work = true;

	if (!hba->clk_scaling.is_allowed || hba->pm_op_in_progress)
		return;

	if (queue_resume_work)
		queue_work(hba->clk_scaling.workq,
			   &hba->clk_scaling.resume_work);

	if (!hba->clk_scaling.window_start_t) {
		hba->clk_scaling.window_start_t = jiffies;
		hba->clk_scaling.tot_busy_t = 0;
		hba->clk_scaling.is_busy_started = false;
	}

	if (!hba->clk_scaling.is_busy_started) {
		hba->clk_scaling.busy_start_t = ktime_get();
		hba->clk_scaling.is_busy_started = true;
	}
}

static void ufshcd_clk_scaling_update_busy(struct ufs_hba *hba)
{
	struct ufs_clk_scaling *scaling = &hba->clk_scaling;

	if (!ufshcd_is_clkscaling_supported(hba))
		return;

	if (!hba->outstanding_reqs && scaling->is_busy_started) {
		scaling->tot_busy_t += ktime_to_us(ktime_sub(ktime_get(),
					scaling->busy_start_t));
		scaling->busy_start_t = ktime_set(0, 0);
		scaling->is_busy_started = false;
	}
}

/**
 * ufshcd_send_command - Send SCSI or device management commands
 * @hba: per adapter instance
 * @task_tag: Task tag of the command
 */
static inline
int ufshcd_send_command(struct ufs_hba *hba, unsigned int task_tag)
{
	int ret = 0;

	hba->lrb[task_tag].issue_time_stamp = ktime_get();
	hba->lrb[task_tag].complete_time_stamp = ktime_set(0, 0);
	ufshcd_clk_scaling_start_busy(hba);
	__set_bit(task_tag, &hba->outstanding_reqs);
	ufshcd_writel(hba, 1 << task_tag, REG_UTP_TRANSFER_REQ_DOOR_BELL);
	/* Make sure that doorbell is committed immediately */
	wmb();
	ufshcd_cond_add_cmd_trace(hba, task_tag, "send");
	ufshcd_update_tag_stats(hba, task_tag);
	return ret;
}

/**
 * ufshcd_copy_sense_data - Copy sense data in case of check condition
 * @lrb - pointer to local reference block
 */
static inline void ufshcd_copy_sense_data(struct ufshcd_lrb *lrbp)
{
	int len;
	if (lrbp->sense_buffer &&
	    ufshcd_get_rsp_upiu_data_seg_len(lrbp->ucd_rsp_ptr)) {
		int len_to_copy;

		len = be16_to_cpu(lrbp->ucd_rsp_ptr->sr.sense_data_len);
		len_to_copy = min_t(int, RESPONSE_UPIU_SENSE_DATA_LENGTH, len);

		memcpy(lrbp->sense_buffer,
			lrbp->ucd_rsp_ptr->sr.sense_data,
			min_t(int, len_to_copy, UFSHCD_REQ_SENSE_SIZE));
	}
}

/**
 * ufshcd_copy_query_response() - Copy the Query Response and the data
 * descriptor
 * @hba: per adapter instance
 * @lrb - pointer to local reference block
 */
static
int ufshcd_copy_query_response(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct ufs_query_res *query_res = &hba->dev_cmd.query.response;

	memcpy(&query_res->upiu_res, &lrbp->ucd_rsp_ptr->qr, QUERY_OSF_SIZE);

	/* Get the descriptor */
	if (hba->dev_cmd.query.descriptor &&
	    lrbp->ucd_rsp_ptr->qr.opcode == UPIU_QUERY_OPCODE_READ_DESC) {
		u8 *descp = (u8 *)lrbp->ucd_rsp_ptr +
				GENERAL_UPIU_REQUEST_SIZE;
		u16 resp_len;
		u16 buf_len;

		/* data segment length */
		resp_len = be32_to_cpu(lrbp->ucd_rsp_ptr->header.dword_2) &
						MASK_QUERY_DATA_SEG_LEN;
		buf_len = be16_to_cpu(
				hba->dev_cmd.query.request.upiu_req.length);
		if (likely(buf_len >= resp_len)) {
			memcpy(hba->dev_cmd.query.descriptor, descp, resp_len);
		} else {
			dev_warn(hba->dev,
				"� : Response size is bigger than buffer",
				__func__);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * ufshcd_hba_capabilities - Read controller capabilities
 * @hba: per adapter instance
 */
static inline void ufshcd_hba_capabilities(struct ufs_hba *hba)
{
	hba->capabilities = ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES);

	/* nutrs and nutmrs are 0 based values */
	hba->nutrs = (hba->capabilities & MASK_TRANSFER_REQUESTS_SLOTS) + 1;
	hba->nutmrs =
	((hba->capabilities & MASK_TASK_MANAGEMENT_REQUEST_SLOTS) >> 16) + 1;
}

/**
 * ufshcd_ready_for_uic_cmd - Check if controller is ready
 *                            to accept UIC commands
 * @hba: per adapter instance
 * Return true on success, else false
 */
static inline bool ufshcd_ready_for_uic_cmd(struct ufs_hba *hba)
{
	if (ufshcd_readl(hba, REG_CONTROLLER_STATUS) & UIC_COMMAND_READY)
		return true;
	else
		return false;
}

/**
 * ufshcd_get_upmcrs - Get the power mode change request status
 * @hba: Pointer to adapter instance
 *
 * This function gets the UPMCRS field of HCS register
 * Returns value of UPMCRS field
 */
static inline u8 ufshcd_get_upmcrs(struct ufs_hba *hba)
{
	return (ufshcd_readl(hba, REG_CONTROLLER_STATUS) >> 8) & 0x7;
}

/**
 * ufshcd_dispatch_uic_cmd - Dispatch UIC commands to unipro layers
 * @hba: per adapter instance
 * @uic_cmd: UIC command
 *
 * Mutex must be held.
 */
static inline void
ufshcd_dispatch_uic_cmd(struct ufs_hba *hba, struct uic_command *uic_cmd)
{
	WARN_ON(hba->active_uic_cmd);

	hba->active_uic_cmd = uic_cmd;

	ufshcd_dme_cmd_log(hba, "send", hba->active_uic_cmd->command);
	/* Write Args */
	ufshcd_writel(hba, uic_cmd->argument1, REG_UIC_COMMAND_ARG_1);
	ufshcd_writel(hba, uic_cmd->argument2, REG_UIC_COMMAND_ARG_2);
	ufshcd_writel(hba, uic_cmd->argument3, REG_UIC_COMMAND_ARG_3);

	/* Write UIC Cmd */
	ufshcd_writel(hba, uic_cmd->command & COMMAND_OPCODE_MASK,
		      REG_UIC_COMMAND);
}

/**
 * ufshcd_wait_for_uic_cmd - Wait complectioin of UIC command
 * @hba: per adapter instance
 * @uic_command: UIC command
 *
 * Must be called with mutex held.
 * Returns 0 only if success.
 */
static int
ufshcd_wait_for_uic_cmd(struct ufs_hba *hba, struct uic_command *uic_cmd)
{
	int ret;
	unsigned long flags;

	if (wait_for_completion_timeout(&uic_cmd->done,
					msecs_to_jiffies(UIC_CMD_TIMEOUT)))
		ret = uic_cmd->argument2 & MASK_UIC_COMMAND_RESULT;
	else
		ret = -ETIMEDOUT;

	if (ret)
		ufsdbg_set_err_state(hba);

	ufshcd_dme_cmd_log(hba, "cmp1", hba->active_uic_cmd->command);

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->active_uic_cmd = NULL;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return ret;
}

/**
 * __ufshcd_send_uic_cmd - Send UIC commands and retrieve the result
 * @hba: per adapter instance
 * @uic_cmd: UIC command
 * @completion: initialize the completion only if this is set to true
 *
 * Identical to ufshcd_send_uic_cmd() expect mutex. Must be called
 * with mutex held and host_lock locked.
 * Returns 0 only if success.
 */
static int
__ufshcd_send_uic_cmd(struct ufs_hba *hba, struct uic_command *uic_cmd,
		      bool completion)
{
	if (!ufshcd_ready_for_uic_cmd(hba)) {
		dev_err(hba->dev,
			"Controller not ready to accept UIC commands\n");
		return -EIO;
	}

	if (completion)
		init_completion(&uic_cmd->done);

	ufshcd_dispatch_uic_cmd(hba, uic_cmd);

	return 0;
}

/**
 * ufshcd_send_uic_cmd - Send UIC commands and retrieve the result
 * @hba: per adapter instance
 * @uic_cmd: UIC command
 *
 * Returns 0 only if success.
 */
static int
ufshcd_send_uic_cmd(struct ufs_hba *hba, struct uic_command *uic_cmd)
{
	int ret;
	unsigned long flags;

	hba->ufs_stats.clk_hold.ctx = UIC_CMD_SEND;
	ufshcd_hold_all(hba);
	mutex_lock(&hba->uic_cmd_mutex);
	ufshcd_add_delay_before_dme_cmd(hba);

	spin_lock_irqsave(hba->host->host_lock, flags);
	ret = __ufshcd_send_uic_cmd(hba, uic_cmd, true);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	if (!ret)
		ret = ufshcd_wait_for_uic_cmd(hba, uic_cmd);

	ufshcd_save_tstamp_of_last_dme_cmd(hba);
	mutex_unlock(&hba->uic_cmd_mutex);
	ufshcd_release_all(hba);
	hba->ufs_stats.clk_rel.ctx = UIC_CMD_SEND;

	ufsdbg_error_inject_dispatcher(hba,
		ERR_INJECT_UIC, 0, &ret);

	return ret;
}

/**
 * ufshcd_map_sg - Map scatter-gather list to prdt
 * @lrbp - pointer to local reference block
 *
 * Returns 0 in case of success, non-zero value in case of failure
 */
static int ufshcd_map_sg(struct ufshcd_lrb *lrbp)
{
	struct ufshcd_sg_entry *prd_table;
	struct scatterlist *sg;
	struct scsi_cmnd *cmd;
	int sg_segments;
	int i;

	cmd = lrbp->cmd;
	sg_segments = scsi_dma_map(cmd);
	if (sg_segments < 0)
		return sg_segments;

	if (sg_segments) {
		lrbp->utr_descriptor_ptr->prd_table_length =
					cpu_to_le16((u16) (sg_segments));

		prd_table = (struct ufshcd_sg_entry *)lrbp->ucd_prdt_ptr;

		scsi_for_each_sg(cmd, sg, sg_segments, i) {
			prd_table[i].size  =
				cpu_to_le32(((u32) sg_dma_len(sg))-1);
			prd_table[i].base_addr =
				cpu_to_le32(lower_32_bits(sg->dma_address));
			prd_table[i].upper_addr =
				cpu_to_le32(upper_32_bits(sg->dma_address));
			prd_table[i].reserved = 0;
		}
	} else {
		lrbp->utr_descriptor_ptr->prd_table_length = 0;
	}

	return 0;
}

/**
 * ufshcd_enable_intr - enable interrupts
 * @hba: per adapter instance
 * @intrs: interrupt bits
 */
static void ufshcd_enable_intr(struct ufs_hba *hba, u32 intrs)
{
	u32 set = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);

	if (hba->ufs_version == UFSHCI_VERSION_10) {
		u32 rw;
		rw = set & INTERRUPT_MASK_RW_VER_10;
		set = rw | ((set ^ intrs) & intrs);
	} else {
		set |= intrs;
	}

	ufshcd_writel(hba, set, REG_INTERRUPT_ENABLE);
}

/**
 * ufshcd_disable_intr - disable interrupts
 * @hba: per adapter instance
 * @intrs: interrupt bits
 */
static void ufshcd_disable_intr(struct ufs_hba *hba, u32 intrs)
{
	u32 set = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);

	if (hba->ufs_version == UFSHCI_VERSION_10) {
		u32 rw;
		rw = (set & INTERRUPT_MASK_RW_VER_10) &
			~(intrs & INTERRUPT_MASK_RW_VER_10);
		set = rw | ((set & intrs) & ~INTERRUPT_MASK_RW_VER_10);

	} else {
		set &= ~intrs;
	}

	ufshcd_writel(hba, set, REG_INTERRUPT_ENABLE);
}

static int ufshcd_prepare_crypto_utrd(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp)
{
	struct utp_transfer_req_desc *req_desc = lrbp->utr_descriptor_ptr;
	u8 cc_index = 0;
	bool enable = false;
	u64 dun = 0;
	int ret;

	/*
	 * Call vendor specific code to get crypto info for this request:
	 * enable, crypto config. index, DUN.
	 * If bypass is set, don't bother setting the other fields.
	 */
	ret = ufshcd_vops_crypto_req_setup(hba, lrbp, &cc_index, &enable, &dun);
	if (ret) {
		if (ret != -EAGAIN) {
			dev_err(hba->dev,
				"� : failed to setup crypto request (�\n",
				__func__, ret);
		}

		return ret;
	}

	if (!enable)
		goto out;

	req_desc->header.dword_0 |= cc_index | UTRD_CRYPTO_ENABLE;
	req_desc->header.dword_1 = (u32)(dun & 0xFFFFFFFF);
	req_desc->header.dword_3 = (u32)((dun >> 32) & 0xFFFFFFFF);
out:
	return 0;
}

/**
 * ufshcd_prepare_req_desc_hdr() - Fills the requests header
 * descriptor according to request
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block
 * @upiu_flags: flags required in the header
 * @cmd_dir: requests data direction
 */
static int ufshcd_prepare_req_desc_hdr(struct ufs_hba *hba,
	struct ufshcd_lrb *lrbp, u32 *upiu_flags,
	enum dma_data_direction cmd_dir)
{
	struct utp_transfer_req_desc *req_desc = lrbp->utr_descriptor_ptr;
	u32 data_direction;
	u32 dword_0;

	if (cmd_dir == DMA_FROM_DEVICE) {
		data_direction = UTP_DEVICE_TO_HOST;
		*upiu_flags = UPIU_CMD_FLAGS_READ;
	} else if (cmd_dir == DMA_TO_DEVICE) {
		data_direction = UTP_HOST_TO_DEVICE;
		*upiu_flags = UPIU_CMD_FLAGS_WRITE;
	} else {
		data_direction = UTP_NO_DATA_TRANSFER;
		*upiu_flags = UPIU_CMD_FLAGS_NONE;
	}

	dword_0 = data_direction | (lrbp->command_type
				<< UPIU_COMMAND_TYPE_OFFSET);
	if (lrbp->intr_cmd)
		dword_0 |= UTP_REQ_DESC_INT_CMD;

	/* Transfer request descriptor header fields */
	req_desc->header.dword_0 = cpu_to_le32(dword_0);
	/* dword_1 is reserved, hence it is set to 0 */
	req_desc->header.dword_1 = 0;
	/*
	 * assigning invalid value for command status. Controller
	 * updates OCS on command completion, with the command
	 * status
	 */
	req_desc->header.dword_2 =
		cpu_to_le32(OCS_INVALID_COMMAND_STATUS);
	/* dword_3 is reserved, hence it is set to 0 */
	req_desc->header.dword_3 = 0;

	req_desc->prd_table_length = 0;

	if (ufshcd_is_crypto_supported(hba))
		return ufshcd_prepare_crypto_utrd(hba, lrbp);

	return 0;
}

/**
 * ufshcd_prepare_utp_scsi_cmd_upiu() - fills the utp_transfer_req_desc,
 * for scsi commands
 * @lrbp - local reference block pointer
 * @upiu_flags - flags
 */
static
void ufshcd_prepare_utp_scsi_cmd_upiu(struct ufshcd_lrb *lrbp, u32 upiu_flags)
{
	struct utp_upiu_req *ucd_req_ptr = lrbp->ucd_req_ptr;
	unsigned short cdb_len;

	/* command descriptor fields */
	ucd_req_ptr->header.dword_0 = UPIU_HEADER_DWORD(
				UPIU_TRANSACTION_COMMAND, upiu_flags,
				lrbp->lun, lrbp->task_tag);
	ucd_req_ptr->header.dword_1 = UPIU_HEADER_DWORD(
				UPIU_COMMAND_SET_TYPE_SCSI, 0, 0, 0);

	/* Total EHS length and Data segment length will be zero */
	ucd_req_ptr->header.dword_2 = 0;

	ucd_req_ptr->sc.exp_data_transfer_len =
		cpu_to_be32(lrbp->cmd->sdb.length);

	cdb_len = min_t(unsigned short, lrbp->cmd->cmd_len, MAX_CDB_SIZE);
	memcpy(ucd_req_ptr->sc.cdb, lrbp->cmd->cmnd, cdb_len);
	if (cdb_len < MAX_CDB_SIZE)
		memset(ucd_req_ptr->sc.cdb + cdb_len, 0,
		       (MAX_CDB_SIZE - cdb_len));
	memset(lrbp->ucd_rsp_ptr, 0, sizeof(struct utp_upiu_rsp));
}

/**
 * ufshcd_prepare_utp_query_req_upiu() - fills the utp_transfer_req_desc,
 * for query requsts
 * @hba: UFS hba
 * @lrbp: local reference block pointer
 * @upiu_flags: flags
 */
static void ufshcd_prepare_utp_query_req_upiu(struct ufs_hba *hba,
				struct ufshcd_lrb *lrbp, u32 upiu_flags)
{
	struct utp_upiu_req *ucd_req_ptr = lrbp->ucd_req_ptr;
	struct ufs_query *query = &hba->dev_cmd.query;
	u16 len = be16_to_cpu(query->request.upiu_req.length);
	u8 *descp = (u8 *)lrbp->ucd_req_ptr + GENERAL_UPIU_REQUEST_SIZE;

	/* Query request header */
	ucd_req_ptr->header.dword_0 = UPIU_HEADER_DWORD(
			UPIU_TRANSACTION_QUERY_REQ, upiu_flags,
			lrbp->lun, lrbp->task_tag);
	ucd_req_ptr->header.dword_1 = UPIU_HEADER_DWORD(
			0, query->request.query_func, 0, 0);

	/* Data segment length */
	ucd_req_ptr->header.dword_2 = UPIU_HEADER_DWORD(
			0, 0, len >> 8, (u8)len);

	/* Copy the Query Request buffer as is */
	memcpy(&ucd_req_ptr->qr, &query->request.upiu_req,
			QUERY_OSF_SIZE);

	/* Copy the Descriptor */
	if (query->request.upiu_req.opcode == UPIU_QUERY_OPCODE_WRITE_DESC)
		memcpy(descp, query->descriptor, len);

	memset(lrbp->ucd_rsp_ptr, 0, sizeof(struct utp_upiu_rsp));
}

static inline void ufshcd_prepare_utp_nop_upiu(struct ufshcd_lrb *lrbp)
{
	struct utp_upiu_req *ucd_req_ptr = lrbp->ucd_req_ptr;

	memset(ucd_req_ptr, 0, sizeof(struct utp_upiu_req));

	/* command descriptor fields */
	ucd_req_ptr->header.dword_0 =
		UPIU_HEADER_DWORD(
			UPIU_TRANSACTION_NOP_OUT, 0, 0, lrbp->task_tag);
	/* clear rest of the fields of basic header */
	ucd_req_ptr->header.dword_1 = 0;
	ucd_req_ptr->header.dword_2 = 0;

	memset(lrbp->ucd_rsp_ptr, 0, sizeof(struct utp_upiu_rsp));
}

/**
 * ufshcd_compose_upiu - form UFS Protocol Information Unit(UPIU)
 * @hba - per adapter instance
 * @lrb - pointer to local reference block
 */
static int ufshcd_compose_upiu(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	u32 upiu_flags;
	int ret = 0;

	switch (lrbp->command_type) {
	case UTP_CMD_TYPE_SCSI:
		if (likely(lrbp->cmd)) {
			ret = ufshcd_prepare_req_desc_hdr(hba, lrbp,
				&upiu_flags, lrbp->cmd->sc_data_direction);
			ufshcd_prepare_utp_scsi_cmd_upiu(lrbp, upiu_flags);
		} else {
			ret = -EINVAL;
		}
		break;
	case UTP_CMD_TYPE_DEV_MANAGE:
		ret = ufshcd_prepare_req_desc_hdr(hba, lrbp, &upiu_flags,
			DMA_NONE);
		if (hba->dev_cmd.type == DEV_CMD_TYPE_QUERY)
			ufshcd_prepare_utp_query_req_upiu(
					hba, lrbp, upiu_flags);
		else if (hba->dev_cmd.type == DEV_CMD_TYPE_NOP)
			ufshcd_prepare_utp_nop_upiu(lrbp);
		else
			ret = -EINVAL;
		break;
	case UTP_CMD_TYPE_UFS:
		/* For UFS native command implementation */
		ret = -ENOTSUPP;
		dev_err(hba->dev, "� : UFS native command are not supported\n",
			__func__);
		break;
	default:
		ret = -ENOTSUPP;
		dev_err(hba->dev, "� : unknown command type: 0x� \n",
				__func__, lrbp->command_type);
		break;
	} /* end of switch */

	return ret;
}

/*
 * ufshcd_scsi_to_upiu_lun - maps scsi LUN to UPIU LUN
 * @scsi_lun: scsi LUN id
 *
 * Returns UPIU LUN id
 */
static inline u8 ufshcd_scsi_to_upiu_lun(unsigned int scsi_lun)
{
	if (scsi_is_wlun(scsi_lun))
		return (scsi_lun & UFS_UPIU_MAX_UNIT_NUM_ID)
			| UFS_UPIU_WLUN_ID;
	else
		return scsi_lun & UFS_UPIU_MAX_UNIT_NUM_ID;
}

/**
 * ufshcd_upiu_wlun_to_scsi_wlun - maps UPIU W-LUN id to SCSI W-LUN ID
 * @scsi_lun: UPIU W-LUN id
 *
 * Returns SCSI W-LUN id
 */
static inline u16 ufshcd_upiu_wlun_to_scsi_wlun(u8 upiu_wlun_id)
{
	return (upiu_wlun_id & ~UFS_UPIU_WLUN_ID) | SCSI_W_LUN_BASE;
}

/**
 * ufshcd_get_write_lock - synchronize between shutdown, scaling &
 * arrival of requests
 * @hba: ufs host
 *
 * Lock is predominantly held by shutdown context thus, ensuring
 * that no requests from any other context may sneak through.
 */
static inline void ufshcd_get_write_lock(struct ufs_hba *hba)
{
	down_write(&hba->lock);
}

/**
 * ufshcd_get_read_lock - synchronize between shutdown, scaling &
 * arrival of requests
 * @hba: ufs host
 *
 * Returns 1 if acquired, < 0 on contention
 *
 * After shutdown's initiated, allow requests only directed to the
 * well known device lun. The sync between scaling & issue is maintained
 * as is and this restructuring syncs shutdown with these too.
 */
static int ufshcd_get_read_lock(struct ufs_hba *hba, u64 lun)
{
	int err = 0;

	err = down_read_trylock(&hba->lock);
	if (err > 0)
		goto out;
	/* let requests for well known device lun to go through */
	if (ufshcd_scsi_to_upiu_lun(lun) == UFS_UPIU_UFS_DEVICE_WLUN)
		return 0;
	else if (!ufshcd_is_shutdown_ongoing(hba))
		return -EAGAIN;
	else
		return -EPERM;

out:
	return err;
}

/**
 * ufshcd_put_read_lock - synchronize between shutdown, scaling &
 * arrival of requests
 * @hba: ufs host
 *
 * Returns none
 */
static inline void ufshcd_put_read_lock(struct ufs_hba *hba)
{
	up_read(&hba->lock);
}

/**
 * ufshcd_queuecommand - main entry point for SCSI requests
 * @cmd: command from SCSI Midlayer
 * @done: call back function
 *
 * Returns 0 for success, non-zero in case of failure
 */
static int ufshcd_queuecommand(struct Scsi_Host *host, struct scsi_cmnd *cmd)
{
	struct ufshcd_lrb *lrbp;
	struct ufs_hba *hba;
	unsigned long flags;
	int tag;
	int err = 0;
	bool has_read_lock = false;

	hba = shost_priv(host);

	if (!cmd || !cmd->request || !hba)
		return -EINVAL;

	tag = cmd->request->tag;
	if (!ufshcd_valid_tag(hba, tag)) {
		dev_err(hba->dev,
			"� : invalid command tag � cmd=0x� , cmd->request=0x� ",
			__func__, tag, cmd, cmd->request);
		BUG();
	}

	err = ufshcd_get_read_lock(hba, cmd->device->lun);
	if (unlikely(err < 0)) {
		if (err == -EPERM) {
			set_host_byte(cmd, DID_ERROR);
			cmd->scsi_done(cmd);
			return 0;
		}
		if (err == -EAGAIN)
			return SCSI_MLQUEUE_HOST_BUSY;
	} else if (err == 1) {
		has_read_lock = true;
	}

	spin_lock_irqsave(hba->host->host_lock, flags);

	/* if error handling is in progress, return host busy */
	if (ufshcd_eh_in_progress(hba)) {
		err = SCSI_MLQUEUE_HOST_BUSY;
		goto out_unlock;
	}

	switch (hba->ufshcd_state) {
	case UFSHCD_STATE_OPERATIONAL:
		break;
	case UFSHCD_STATE_RESET:
		err = SCSI_MLQUEUE_HOST_BUSY;
		goto out_unlock;
	case UFSHCD_STATE_ERROR:
		set_host_byte(cmd, DID_ERROR);
		cmd->scsi_done(cmd);
		goto out_unlock;
	default:
		dev_WARN_ONCE(hba->dev, 1, "� : invalid state �n",
				__func__, hba->ufshcd_state);
		set_host_byte(cmd, DID_BAD_TARGET);
		cmd->scsi_done(cmd);
		goto out_unlock;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	hba->req_abort_count = 0;

	/* acquire the tag to make sure device cmds don't use it */
	if (test_and_set_bit_lock(tag, &hba->lrb_in_use)) {
		/*
		 * Dev manage command in progress, requeue the command.
		 * Requeuing the command helps in cases where the request *may*
		 * find different tag instead of waiting for dev manage command
		 * completion.
		 */
		err = SCSI_MLQUEUE_HOST_BUSY;
		goto out;
	}

	hba->ufs_stats.clk_hold.ctx = QUEUE_CMD;
	err = ufshcd_hold(hba, true);
	if (err) {
		err = SCSI_MLQUEUE_HOST_BUSY;
		clear_bit_unlock(tag, &hba->lrb_in_use);
		goto out;
	}
	if (ufshcd_is_clkgating_allowed(hba))
		WARN_ON(hba->clk_gating.state != CLKS_ON);

	err = ufshcd_hibern8_hold(hba, true);
	if (err) {
		clear_bit_unlock(tag, &hba->lrb_in_use);
		err = SCSI_MLQUEUE_HOST_BUSY;
		hba->ufs_stats.clk_rel.ctx = QUEUE_CMD;
		ufshcd_release(hba, true);
		goto out;
	}
	if (ufshcd_is_hibern8_on_idle_allowed(hba))
		WARN_ON(hba->hibern8_on_idle.state != HIBERN8_EXITED);

	/* Vote PM QoS for the request */
	ufshcd_vops_pm_qos_req_start(hba, cmd->request);

	/* IO svc time latency histogram */
	if (hba->latency_hist_enabled &&
	    (cmd->request->cmd_type == REQ_TYPE_FS)) {
		cmd->request->lat_hist_io_start = ktime_get();
		cmd->request->lat_hist_enabled = 1;
	} else {
		cmd->request->lat_hist_enabled = 0;
	}

	WARN_ON(hba->clk_gating.state != CLKS_ON);

	lrbp = &hba->lrb[tag];

	WARN_ON(lrbp->cmd);
	lrbp->cmd = cmd;
	lrbp->sense_bufflen = UFSHCD_REQ_SENSE_SIZE;
	lrbp->sense_buffer = cmd->sense_buffer;
	lrbp->task_tag = tag;
	lrbp->lun = ufshcd_scsi_to_upiu_lun(cmd->device->lun);
	lrbp->intr_cmd = !ufshcd_is_intr_aggr_allowed(hba) ? true : false;
	lrbp->command_type = UTP_CMD_TYPE_SCSI;
	lrbp->req_abort_skip = false;

	/* form UPIU before issuing the command */
	err = ufshcd_compose_upiu(hba, lrbp);
	if (err) {
		if (err != -EAGAIN)
			dev_err(hba->dev,
				"� : failed to compose upiu �n",
				__func__, err);

		lrbp->cmd = NULL;
		clear_bit_unlock(tag, &hba->lrb_in_use);
		ufshcd_release_all(hba);
		ufshcd_vops_pm_qos_req_end(hba, cmd->request, true);
		goto out;
	}

	err = ufshcd_map_sg(lrbp);
	if (err) {
		ufshcd_release(hba, false);
		lrbp->cmd = NULL;
		clear_bit_unlock(tag, &hba->lrb_in_use);
		ufshcd_release_all(hba);
		ufshcd_vops_pm_qos_req_end(hba, cmd->request, true);
		goto out;
	}

	err = ufshcd_vops_crypto_engine_cfg_start(hba, tag);
	if (err) {
		if (err != -EAGAIN)
			dev_err(hba->dev,
				"� : failed to configure crypto engine �n",
				__func__, err);

		scsi_dma_unmap(lrbp->cmd);
		lrbp->cmd = NULL;
		clear_bit_unlock(tag, &hba->lrb_in_use);
		ufshcd_release_all(hba);
		ufshcd_vops_pm_qos_req_end(hba, cmd->request, true);

		goto out;
	}

	/* Make sure descriptors are ready before ringing the doorbell */
	wmb();
	/* issue command to the controller */
	spin_lock_irqsave(hba->host->host_lock, flags);

	err = ufshcd_send_command(hba, tag);
	if (err) {
		spin_unlock_irqrestore(hba->host->host_lock, flags);
		scsi_dma_unmap(lrbp->cmd);
		lrbp->cmd = NULL;
		clear_bit_unlock(tag, &hba->lrb_in_use);
		ufshcd_release_all(hba);
		ufshcd_vops_pm_qos_req_end(hba, cmd->request, true);
		ufshcd_vops_crypto_engine_cfg_end(hba, lrbp, cmd->request);
		dev_err(hba->dev, "� : failed sending command, �n",
							__func__, err);
		err = DID_ERROR;
		goto out;
	}

out_unlock:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	if (has_read_lock)
		ufshcd_put_read_lock(hba);
	return err;
}

static int ufshcd_compose_dev_cmd(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp, enum dev_cmd_type cmd_type, int tag)
{
	lrbp->cmd = NULL;
	lrbp->sense_bufflen = 0;
	lrbp->sense_buffer = NULL;
	lrbp->task_tag = tag;
	lrbp->lun = 0; /* device management cmd is not specific to any LUN */
	lrbp->command_type = UTP_CMD_TYPE_DEV_MANAGE;
	lrbp->intr_cmd = true; /* No interrupt aggregation */
	hba->dev_cmd.type = cmd_type;

	return ufshcd_compose_upiu(hba, lrbp);
}

static int
ufshcd_clear_cmd(struct ufs_hba *hba, int tag)
{
	int err = 0;
	unsigned long flags;
	u32 mask = 1 << tag;

	/* clear outstanding transaction before retry */
	spin_lock_irqsave(hba->host->host_lock, flags);
	ufshcd_utrl_clear(hba, tag);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	/*
	 * wait for for h/w to clear corresponding bit in door-bell.
	 * max. wait is 1 sec.
	 */
	err = ufshcd_wait_for_register(hba,
			REG_UTP_TRANSFER_REQ_DOOR_BELL,
			mask, ~mask, 1000, 1000, true);

	return err;
}

static int
ufshcd_check_query_response(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct ufs_query_res *query_res = &hba->dev_cmd.query.response;

	/* Get the UPIU response */
	query_res->response = ufshcd_get_rsp_upiu_result(lrbp->ucd_rsp_ptr) >>
				UPIU_RSP_CODE_OFFSET;
	return query_res->response;
}

/**
 * ufshcd_dev_cmd_completion() - handles device management command responses
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block
 */
static int
ufshcd_dev_cmd_completion(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	int resp;
	int err = 0;

	hba->ufs_stats.last_hibern8_exit_tstamp = ktime_set(0, 0);
	resp = ufshcd_get_req_rsp(lrbp->ucd_rsp_ptr);

	switch (resp) {
	case UPIU_TRANSACTION_NOP_IN:
		if (hba->dev_cmd.type != DEV_CMD_TYPE_NOP) {
			err = -EINVAL;
			dev_err(hba->dev, "� : unexpected response � \n",
					__func__, resp);
		}
		break;
	case UPIU_TRANSACTION_QUERY_RSP:
		err = ufshcd_check_query_response(hba, lrbp);
		if (!err)
			err = ufshcd_copy_query_response(hba, lrbp);
		break;
	case UPIU_TRANSACTION_REJECT_UPIU:
		/* TODO: handle Reject UPIU Response */
		err = -EPERM;
		dev_err(hba->dev, "� : Reject UPIU not fully implemented\n",
				__func__);
		break;
	default:
		err = -EINVAL;
		dev_err(hba->dev, "� : Invalid device management cmd response: � \n",
				__func__, resp);
		break;
	}

	return err;
}

static int ufshcd_wait_for_dev_cmd(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp, int max_timeout)
{
	int err = 0;
	unsigned long time_left;
	unsigned long flags;

	time_left = wait_for_completion_timeout(hba->dev_cmd.complete,
			msecs_to_jiffies(max_timeout));

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->dev_cmd.complete = NULL;
	if (likely(time_left)) {
		err = ufshcd_get_tr_ocs(lrbp);
		if (!err)
			err = ufshcd_dev_cmd_completion(hba, lrbp);
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (!time_left) {
		err = -ETIMEDOUT;
		dev_dbg(hba->dev, "� : dev_cmd request timedout, tag �n",
			__func__, lrbp->task_tag);
		if (!ufshcd_clear_cmd(hba, lrbp->task_tag))
			/* successfully cleared the command, retry if needed */
			err = -EAGAIN;
		/*
		 * in case of an error, after clearing the doorbell,
		 * we also need to clear the outstanding_request
		 * field in hba
		 */
		ufshcd_outstanding_req_clear(hba, lrbp->task_tag);
	}

	if (err)
		ufsdbg_set_err_state(hba);

	return err;
}

/**
 * ufshcd_get_dev_cmd_tag - Get device management command tag
 * @hba: per-adapter instance
 * @tag: pointer to variable with available slot value
 *
 * Get a free slot and lock it until device management command
 * completes.
 *
 * Returns false if free slot is unavailable for locking, else
 * return true with tag value in @tag.
 */
static bool ufshcd_get_dev_cmd_tag(struct ufs_hba *hba, int *tag_out)
{
	int tag;
	bool ret = false;
	unsigned long tmp;

	if (!tag_out)
		goto out;

	do {
		tmp = ~hba->lrb_in_use;
		tag = find_last_bit(&tmp, hba->nutrs);
		if (tag >= hba->nutrs)
			goto out;
	} while (test_and_set_bit_lock(tag, &hba->lrb_in_use));

	*tag_out = tag;
	ret = true;
out:
	return ret;
}

static inline void ufshcd_put_dev_cmd_tag(struct ufs_hba *hba, int tag)
{
	clear_bit_unlock(tag, &hba->lrb_in_use);
}

/**
 * ufshcd_exec_dev_cmd - API for sending device management requests
 * @hba - UFS hba
 * @cmd_type - specifies the type (NOP, Query...)
 * @timeout - time in seconds
 *
 * NOTE: Since there is only one available tag for device management commands,
 * it is expected you hold the hba->dev_cmd.lock mutex.
 */
static int ufshcd_exec_dev_cmd(struct ufs_hba *hba,
		enum dev_cmd_type cmd_type, int timeout)
{
	struct ufshcd_lrb *lrbp;
	int err;
	int tag;
	struct completion wait;
	unsigned long flags;

	/*
	 * May get invoked from shutdown and IOCTL contexts.
	 * In shutdown context, it comes in with lock acquired.
	 * In error recovery context, it may come with lock acquired.
	 */

	if (!ufshcd_is_shutdown_ongoing(hba) && !ufshcd_eh_in_progress(hba))
		down_read(&hba->lock);

	/*
	 * Get free slot, sleep if slots are unavailable.
	 * Even though we use wait_event() which sleeps indefinitely,
	 * the maximum wait time is bounded by SCSI request timeout.
	 */
	wait_event(hba->dev_cmd.tag_wq, ufshcd_get_dev_cmd_tag(hba, &tag));

	init_completion(&wait);
	lrbp = &hba->lrb[tag];
	WARN_ON(lrbp->cmd);
	err = ufshcd_compose_dev_cmd(hba, lrbp, cmd_type, tag);
	if (unlikely(err))
		goto out_put_tag;

	hba->dev_cmd.complete = &wait;

	/* Make sure descriptors are ready before ringing the doorbell */
	wmb();
	spin_lock_irqsave(hba->host->host_lock, flags);
	err = ufshcd_send_command(hba, tag);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	if (err) {
		dev_err(hba->dev, "� : failed sending command, �n",
							__func__, err);
		goto out_put_tag;
	}
	err = ufshcd_wait_for_dev_cmd(hba, lrbp, timeout);

out_put_tag:
	ufshcd_put_dev_cmd_tag(hba, tag);
	wake_up(&hba->dev_cmd.tag_wq);
	if (!ufshcd_is_shutdown_ongoing(hba) && !ufshcd_eh_in_progress(hba))
		up_read(&hba->lock);
	return err;
}

/**
 * ufshcd_init_query() - init the query response and request parameters
 * @hba: per-adapter instance
 * @request: address of the request pointer to be initialized
 * @response: address of the response pointer to be initialized
 * @opcode: operation to perform
 * @idn: flag idn to access
 * @index: LU number to access
 * @selector: query/flag/descriptor further identification
 */
static inline void ufshcd_init_query(struct ufs_hba *hba,
		struct ufs_query_req **request, struct ufs_query_res **response,
		enum query_opcode opcode, u8 idn, u8 index, u8 selector)
{
	int idn_t = (int)idn;

	ufsdbg_error_inject_dispatcher(hba,
		ERR_INJECT_QUERY, idn_t, (int *)&idn_t);
	idn = idn_t;

	*request = &hba->dev_cmd.query.request;
	*response = &hba->dev_cmd.query.response;
	memset(*request, 0, sizeof(struct ufs_query_req));
	memset(*response, 0, sizeof(struct ufs_query_res));
	(*request)->upiu_req.opcode = opcode;
	(*request)->upiu_req.idn = idn;
	(*request)->upiu_req.index = index;
	(*request)->upiu_req.selector = selector;

	ufshcd_update_query_stats(hba, opcode, idn);
}

static int ufshcd_query_flag_retry(struct ufs_hba *hba,
	enum query_opcode opcode, enum flag_idn idn, bool *flag_res)
{
	int ret;
	int retries;

	for (retries = 0; retries < QUERY_REQ_RETRIES; retries++) {
		ret = ufshcd_query_flag(hba, opcode, idn, flag_res);
		if (ret)
			dev_dbg(hba->dev,
				"� : failed with error � retries �n",
				__func__, ret, retries);
		else
			break;
	}

	if (ret)
		dev_err(hba->dev,
			"� : query attribute, opcode � idn � failed with error �after �retires\n",
			__func__, opcode, idn, ret, retries);
	return ret;
}

/**
 * ufshcd_query_flag() - API function for sending flag query requests
 * hba: per-adapter instance
 * query_opcode: flag query to perform
 * idn: flag idn to access
 * flag_res: the flag value after the query request completes
 *
 * Returns 0 for success, non-zero in case of failure
 */
int ufshcd_query_flag(struct ufs_hba *hba, enum query_opcode opcode,
			enum flag_idn idn, bool *flag_res)
{
	struct ufs_query_req *request = NULL;
	struct ufs_query_res *response = NULL;
	int err, index = 0, selector = 0;
	int timeout = QUERY_REQ_TIMEOUT;

	BUG_ON(!hba);

	ufshcd_hold_all(hba);
	mutex_lock(&hba->dev_cmd.lock);
	ufshcd_init_query(hba, &request, &response, opcode, idn, index,
			selector);

	switch (opcode) {
	case UPIU_QUERY_OPCODE_SET_FLAG:
	case UPIU_QUERY_OPCODE_CLEAR_FLAG:
	case UPIU_QUERY_OPCODE_TOGGLE_FLAG:
		request->query_func = UPIU_QUERY_FUNC_STANDARD_WRITE_REQUEST;
		break;
	case UPIU_QUERY_OPCODE_READ_FLAG:
		request->query_func = UPIU_QUERY_FUNC_STANDARD_READ_REQUEST;
		if (!flag_res) {
			/* No dummy reads */
			dev_err(hba->dev, "� : Invalid argument for read request\n",
					__func__);
			err = -EINVAL;
			goto out_unlock;
		}
		break;
	default:
		dev_err(hba->dev,
			"� : Expected query flag opcode but got = �n",
			__func__, opcode);
		err = -EINVAL;
		goto out_unlock;
	}

	err = ufshcd_exec_dev_cmd(hba, DEV_CMD_TYPE_QUERY, timeout);

	if (err) {
		dev_err(hba->dev,
			"� : Sending flag query for idn �failed, err = �n",
			__func__, request->upiu_req.idn, err);
		goto out_unlock;
	}

	if (flag_res)
		*flag_res = (be32_to_cpu(response->upiu_res.value) &
				MASK_QUERY_UPIU_FLAG_LOC) & 0x1;

out_unlock:
	mutex_unlock(&hba->dev_cmd.lock);
	ufshcd_release_all(hba);
	return err;
}
EXPORT_SYMBOL(ufshcd_query_flag);

/**
 * ufshcd_query_attr - API function for sending attribute requests
 * hba: per-adapter instance
 * opcode: attribute opcode
 * idn: attribute idn to access
 * index: index field
 * selector: selector field
 * attr_val: the attribute value after the query request completes
 *
 * Returns 0 for success, non-zero in case of failure
*/
int ufshcd_query_attr(struct ufs_hba *hba, enum query_opcode opcode,
			enum attr_idn idn, u8 index, u8 selector, u32 *attr_val)
{
	struct ufs_query_req *request = NULL;
	str…