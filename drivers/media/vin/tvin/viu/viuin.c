// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/module.h>

#include <linux/amlogic/media/utils/amstream.h>
#include <linux/amlogic/media/frame_sync/ptsserv.h>
#include <linux/amlogic/media/vfm/vframe.h>
#include <linux/amlogic/media/vfm/vframe_provider.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/media/frame_provider/tvin/tvin_v4l2.h>
#include <linux/amlogic/media/rdma/rdma_mgr.h>

#include "../tvin_global.h"
#include "../tvin_frontend.h"
#include "../tvin_format_table.h"

#define DEVICE_NAME "viuin"
#define MODULE_NAME "viuin"

#define AMVIUIN_DEC_START       1
#define AMVIUIN_DEC_STOP        0

/* register */
#define VIU_MISC_CTRL1 0x1a07
#define VPU_VIU_VENC_MUX_CTRL 0x271a
#define ENCI_INFO_READ 0x271c
#define ENCP_INFO_READ 0x271d
#define ENCT_INFO_READ 0x271e
#define ENCL_INFO_READ 0x271f
#define VPU_VIU2VDIN_HDN_CTRL 0x2780
/* t7 enable hsync reg */
#define VIU_FRM_CTRL   0x1a51
#define VIU1_FRM_CTRL   0x1a8d
#define VIU2_FRM_CTRL   0x1a8e

/* ENCL/ENCT polarity, positive:begin > end, negative: begin < end */
#define ENCL_VIDEO_VSO_BLINE	0x1cb9
#define ENCL_VIDEO_VSO_ELINE	0x1cba

/* ENCP polarity, positive:begin > end, negative: begin < end */
#define ENCP_VIDEO_VSO_BLINE	0x1bab
#define ENCP_VIDEO_VSO_ELINE	0x1bac

/* ENCI polarity, always negative, no need control dynamically */

/*g12a new add*/
#define VPU_VIU_ASYNC_MASK 0x2781
#define VDIN_MISC_CTRL 0x2782
#define VPU_VIU_VDIN_IF_MUX_CTRL 0x2783
#define VPU_VIU2VDIN1_HDN_CTRL 0x2784
#define VPU_VENCX_CLK_CTRL 0x2785
#define VPP_WR_BAK_CTRL 0x1df9
#define VPP1_WR_BAK_CTRL 0x5981
#define VPP2_WR_BAK_CTRL 0x59c1
#define VPU_422TO444_CTRL0 0x274b
#define VPU_422TO444_CTRL1 0x274c
#define VPU_422TO444_CTRL2 0x274d
#define WR_BACK_MISC_CTRL 0x1a0d

/* s5 */
#define VIU_WR_BAK_CTRL                             0x1a25
#define S5_VPP_POST_HOLD_CTRL                       0x1d1f
/* t3x */
#define VPP_POST_HOLD_CTRL_T3X		0x1d19
#define VPU_VIU_VDIN_IF_MUX_CTRL_T3X	0x2783
#define VPU_VIU2VDIN0_BUF_SIZE_T3X	0x2784
#define VDIN_LITE_CORE_MAX_PIXEL_CLOCK	(4096 * 2160 * 60UL)
/* txhd2 for keystone dump vdin1 */
#define VPP_MISC									0x1d26

static unsigned int vsync_enter_line_curr;
module_param(vsync_enter_line_curr, uint, 0664);
MODULE_PARM_DESC(vsync_enter_line_curr,
		 "\n encoder process line num when enter isr.\n");

static unsigned int vsync_enter_line_max;
module_param(vsync_enter_line_max, uint, 0664);
MODULE_PARM_DESC(vsync_enter_line_max,
		 "\n max encoder process line num when enter isr.\n");

static unsigned int vsync_enter_line_max_threshold = 10000;
module_param(vsync_enter_line_max_threshold, uint, 0664);
MODULE_PARM_DESC(vsync_enter_line_max_threshold,
		 "\n max encoder process line num over threshold drop the frame.\n");

static unsigned int vsync_enter_line_min_threshold = 10000;
module_param(vsync_enter_line_min_threshold, uint, 0664);
MODULE_PARM_DESC(vsync_enter_line_min_threshold,
		 "\n max encoder process line num less threshold drop the frame.\n");
static unsigned int vsync_enter_line_threshold_overflow_count;
module_param(vsync_enter_line_threshold_overflow_count, uint, 0664);
MODULE_PARM_DESC(vsync_enter_line_threshold_overflow_count,
		 "\ncnt overflow encoder process line no over threshold drop the frame\n");

static unsigned short v_cut_offset;
module_param(v_cut_offset, ushort, 0664);
MODULE_PARM_DESC(v_cut_offset, "the cut window vertical offset for viuin");

static unsigned short open_cnt;
module_param(open_cnt, ushort, 0664);
MODULE_PARM_DESC(open_cnt, "open_cnt for vdin0/1");

#define VIUIN_CHAN_MAX	2
/* one frontend per vdin, its index the vdin's */
#define VIUIN_FE_NUM	2

/* both write-back channels program the registers below; one lock orders them */
static DEFINE_SPINLOCK(viuin_lock);

/* the registers an open programs: the tap selects of the channels, then
 * the input word that both channels share, its width and port fields
 */
enum {
	VIUIN_WB_VPP,
	VIUIN_WB_VPP1,
	VIUIN_WB_VPP2,
	VIUIN_WB_VIU,
	VIUIN_WB_HDN,
	VIUIN_WB_NUM
};

static const u32 viuin_wb_reg[VIUIN_WB_NUM] = {
	VPP_WR_BAK_CTRL, VPP1_WR_BAK_CTRL, VPP2_WR_BAK_CTRL,
	VIU_WR_BAK_CTRL, VPU_VIU2VDIN_HDN_CTRL
};

struct viuin_wb_fields {
	u32 val[VIUIN_WB_NUM];
	u32 mask[VIUIN_WB_NUM];
};

struct viuin_s {
	unsigned int flag;
	struct vframe_prop_s *prop;
	/*add for tvin frontend*/
	struct tvin_frontend_s frontend;
	struct vdin_parm_s parm;
	unsigned int enc_info_addr;
	unsigned int viu_mux;
	/* the write-back channels this frontend reads */
	unsigned int chans;
	/* per bound input: its tap, its channel and the fields its port sets
	 * on each channel; base holds the input word fields every open sets,
	 * swc the channel whose tap field this frontend's table switches
	 */
	unsigned int bound;
	enum tvin_port_e tap[TVIN_TAP_MAX];
	unsigned int chan[TVIN_TAP_MAX];
	struct viuin_wb_fields wb[TVIN_TAP_MAX][VIUIN_CHAN_MAX];
	struct viuin_wb_fields base;
	unsigned int swc;
};

/* the value each register takes once every queued write lands; viuin is
 * their only writer. While a frontend switches a tap field only its rdma
 * table writes them, so a cpu write cannot land between its entries.
 */
static struct {
	u32 val[VIUIN_WB_NUM];
	u32 valid;
	struct viuin_s *writer;
} viuin_wb;

/* a channel carries one tap to every reader; the vdin1 histogram is one */
static struct {
	enum tvin_port_e tap;
	unsigned int readers;
} viuin_chan[VIUIN_CHAN_MAX];

/* the write-back channels the vpp offers, from the vdin dt */
static unsigned int viuin_chans = 1;
static bool viuin_hist;

static inline u32 rd_viu(u32 reg)
{
	return (u32)aml_read_vcbus(reg);
}

static inline void wr_viu(u32 reg,
			  const u32 val)
{
	aml_write_vcbus(reg, val);
}

static inline void wr_bits_viu(u32 reg,
			       const u32 value,
			       const u32 start,
			       const u32 len)
{
	aml_write_vcbus(reg, ((aml_read_vcbus(reg) &
			      ~(((1L << (len)) - 1) << (start))) |
			      (((value) & ((1L << (len)) - 1)) << (start))));
}

static inline u32 rd_bits_viu(u32 reg,
			      const u32 start,
			      const u32 len)
{
	u32 val;

	val = ((aml_read_vcbus(reg) >> (start)) & ((1L << (len)) - 1));

	return val;
}

static inline void wb_bits(struct viuin_wb_fields *wb, unsigned int i,
			   const u32 value, const u32 start, const u32 len)
{
	u32 mask = ((1L << len) - 1) << start;

	wb->val[i] = (wb->val[i] & ~mask) | ((value << start) & mask);
	wb->mask[i] |= mask;
}

/* under viuin_lock */
static u32 viuin_wb_get(unsigned int i)
{
	if (!(viuin_wb.valid & BIT(i))) {
		viuin_wb.val[i] = rd_viu(viuin_wb_reg[i]);
		viuin_wb.valid |= BIT(i);
	}
	return viuin_wb.val[i];
}

/* under viuin_lock; while a table writes the registers it carries the change */
static void viuin_wb_set(unsigned int i, u32 val)
{
	viuin_wb_get(i);
	viuin_wb.val[i] = val;
	if (!viuin_wb.writer)
		wr_viu(viuin_wb_reg[i], val);
}

static void viuin_wb_bits(unsigned int i, const u32 value, const u32 start,
			  const u32 len)
{
	u32 mask = ((1L << len) - 1) << start;

	viuin_wb_set(i, (viuin_wb_get(i) & ~mask) | ((value << start) & mask));
}

static bool viuin_is_wb1(enum tvin_port_e port)
{
	return port >= TVIN_PORT_VIU1_WB1_VDIN_BIST &&
	       port <= TVIN_PORT_VIU1_WB1_POST_BLEND;
}

/* a write-back tap is named by its channel 0 port */
static enum tvin_port_e viuin_tap(enum tvin_port_e port)
{
	if (port >= TVIN_PORT_VIU1_WB1_VIDEO && port <= TVIN_PORT_VIU1_WB1_POST_BLEND)
		return port - TVIN_PORT_VIU1_WB1_VIDEO + TVIN_PORT_VIU1_VIDEO;
	return port;
}

static bool viuin_is_tap(enum tvin_port_e tap)
{
	return tap >= TVIN_PORT_VIU1_VIDEO && tap <= TVIN_PORT_VIU1_WB0_POST_BLEND;
}

static enum tvin_port_e viuin_chan_port(enum tvin_port_e port, unsigned int c)
{
	enum tvin_port_e tap = viuin_tap(port);

	if (!viuin_is_tap(tap))
		return port;
	return c ? tap - TVIN_PORT_VIU1_VIDEO + TVIN_PORT_VIU1_WB1_VIDEO : tap;
}

void viuin_set_wb_channels(unsigned int num)
{
	viuin_chans = clamp(num, 1U, (unsigned int)VIUIN_CHAN_MAX);
}
EXPORT_SYMBOL(viuin_set_wb_channels);

static int viuin_support(struct tvin_frontend_s *fe, enum tvin_port_e port)
{
	if ((port >= TVIN_PORT_VIU1 && port < TVIN_PORT_VIU1_MAX) ||
	    (port >= TVIN_PORT_VIU2 && port < TVIN_PORT_VIU2_MAX) ||
	    (port >= TVIN_PORT_VIU3 && port < TVIN_PORT_VIU3_MAX) ||
	    (port >= TVIN_PORT_VENC && port < TVIN_PORT_VENC_MAX) ||
	    port == TVIN_PORT_MIPI || port == TVIN_PORT_ISP)
		return 0;
	else
		return -1;
}

void viuin_check_venc_line(struct viuin_s *devp_local)
{
	unsigned int vencv_line_cur, cnt;

	cnt = 0;
	do {
		vencv_line_cur =
			(rd_viu(devp_local->enc_info_addr) >> 16) & 0x1fff;
		udelay(9);
		cnt++;
		if (cnt > 100000)
			break;
	} while (vencv_line_cur != 1);

	if (vencv_line_cur != 1)
		pr_info("**************%s,vencv_line_cur:%d,cnt:%d***********\n",
			__func__, vencv_line_cur, cnt);
}

/* the vdin1 histogram reads the vpp output through write-back channel 0 */
int viuin_select_loopback_path(void)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&viuin_lock, flags);
	if (!viuin_hist && viuin_chan[0].readers &&
	    (viuin_chan[0].tap != TVIN_PORT_VIU1_WB0_VPP ||
	     (viuin_wb.writer && !viuin_wb.writer->swc)))
		ret = -EBUSY;
	if (viuin_hist || ret)
		goto out;
	viuin_hist = true;
	if (viuin_chan[0].readers++)
		goto out;
	viuin_chan[0].tap = TVIN_PORT_VIU1_WB0_VPP;
	/* set VPU_VIU_VDIN_IF_MUX_CTRL */
	wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL_T3X, 0x808, 0, 16);
	/* set VPP_WRBAK_CTRL */
	viuin_wb_bits(VIUIN_WB_VPP, 6, 0, 4);
	viuin_wb_bits(VIUIN_WB_VPP, 0xff, 16, 8);
	/* wrback hsync en */
	//if (port >= TVIN_PORT_VIU1_VIDEO &&
	//    port <= TVIN_PORT_VIU1_WB0_POST_BLEND) {
	wr_bits_viu(WR_BACK_MISC_CTRL, 1, 0, 1);/*vd0 hsync*/
	//} else {
	//	wr_bits_viu(WR_BACK_MISC_CTRL, 0xf, 0, 4);
	//}
	//if (cpu_after_eq(MESON_CPU_MAJOR_ID_T5M)) {
	wr_viu(VIU_FRM_CTRL, 0x3000004);
	//}
out:
	spin_unlock_irqrestore(&viuin_lock, flags);
	return ret;
}
EXPORT_SYMBOL(viuin_select_loopback_path);

void viuin_clear_loopback_path(void)
{
	unsigned long flags;

	spin_lock_irqsave(&viuin_lock, flags);
	if (viuin_hist) {
		viuin_hist = false;
		if (!--viuin_chan[0].readers) {
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, 0, 16);
			viuin_wb_bits(VIUIN_WB_VPP, 0, 0, 4);
		}
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
}
EXPORT_SYMBOL(viuin_clear_loopback_path);

/* mux0 (vdin source 7) is bits 15:0, mux1 (source 9) bits 31:16 */
static void viuin_set_vdin_if_mux_ctrl(enum tvin_port_e port)
{
	unsigned int viu_mux = 0;
	unsigned int viu_sel = 0;
	unsigned int base = viuin_is_wb1(port) ? 16 : 0;

	switch (port) {
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_VIDEO:
	case TVIN_PORT_VIU1_WB0_VD1:
	case TVIN_PORT_VIU1_WB0_VD2:
	case TVIN_PORT_VIU1_WB0_OSD1:
	case TVIN_PORT_VIU1_WB0_OSD2:
	case TVIN_PORT_VIU1_WB0_POST_BLEND:
	case TVIN_PORT_VIU1_WB0_VPP:
		viu_mux = VIU_MUX_SEL_WB0;
		break;
	case TVIN_PORT_VIU1_WB1_VDIN_BIST:
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_WB1_VIDEO:
	case TVIN_PORT_VIU1_WB1_VD1:
	case TVIN_PORT_VIU1_WB1_VD2:
	case TVIN_PORT_VIU1_WB1_OSD1:
	case TVIN_PORT_VIU1_WB1_OSD2:
	case TVIN_PORT_VIU1_WB1_POST_BLEND:
	case TVIN_PORT_VIU1_WB1_VPP:
		viu_mux = VIU_MUX_SEL_WB1;
		break;
	case TVIN_PORT_VIU2_ENCL:
	case TVIN_PORT_VENC1:
		viu_mux = VIU_MUX_SEL_ENCL;
		break;
	case TVIN_PORT_VIU2_ENCI:
	case TVIN_PORT_VENC2:
		viu_mux = VIU_MUX_SEL_ENCI;
		break;
	case TVIN_PORT_VIU2_ENCP:
	case TVIN_PORT_VENC0:
		viu_mux = VIU_MUX_SEL_ENCP;
		break;
	case TVIN_PORT_VIU2_VD1:
	case TVIN_PORT_VIU2_OSD1:
	case TVIN_PORT_VIU2_VPP:
		viu_mux = VIU_MUX_SEL_WB2;
		break;
	case TVIN_PORT_VIU3_VD1:
	case TVIN_PORT_VIU3_OSD1:
	case TVIN_PORT_VIU3_VPP:
		viu_mux = VIU_MUX_SEL_WB3;
		break;
	default:
		viu_mux = 0;
		break;
	}

	if (cpu_after_eq(MESON_CPU_MAJOR_ID_TM2)) {
		viu_sel = 1;
	} else {
		/*old chip*/
		switch (port & ~0xff) {
		case TVIN_PORT_VIU1:
			viu_sel = 1;
			break;
		case TVIN_PORT_VIU2:
			viu_sel = 2;
			break;
		default:
			break;
		}
	}

	if (viu_sel == 1) {
		if (is_meson_t3x_cpu()) {
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL_T3X, viu_mux, 0, 7);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL_T3X, viu_mux, 8, 7);
		} else if (cpu_after_eq(MESON_CPU_MAJOR_ID_T7)) {
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base, 7);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base, 7);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base + 8, 7);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base + 8, 7);
		} else {
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base, 5);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base, 5);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base + 8, 5);
			wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base + 8, 5);
		}
	} else if (viu_sel == 2) {
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base, 5);
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base, 5);
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base + 8, 5);
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, viu_mux, base + 8, 5);
	} else {
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, base, 16);
	}
}

#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
static void viuin_set_wr_bak_ctrl_s5(enum tvin_port_e port, struct viuin_wb_fields *wb)
{
	const struct vinfo_s *vinfo = NULL;

	vinfo = get_current_vinfo();
	if (!vinfo)
		pr_info("%s vinfo == NULL\n", __func__);
	else
		pr_info("cur_enc_ppc = %d\n", vinfo->cur_enc_ppc);

	/* wr_bak use 4ppc always on S5 */
	wb_bits(wb, VIUIN_WB_HDN, 3, 21, 2);
	wb_bits(wb, VIUIN_WB_HDN, 2, 30, 2);

	switch (port) {
	/* wr_bak_chan1_sel wb_chan_sel*/
	//wr_bak_chan0_sel : 0:vd1 1:vd2 2:vd3 3:osd1 4:osd2 5:post_blend_out;
	case TVIN_PORT_VIU1_WB0_VD1:
		wb_bits(wb, VIUIN_WB_VIU, 0, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VD2:
		wb_bits(wb, VIUIN_WB_VIU, 1, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD1:
		wb_bits(wb, VIUIN_WB_VIU, 3, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD2:
		wb_bits(wb, VIUIN_WB_VIU, 4, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VPP://s5 no vppout
	case TVIN_PORT_VIU1_WB0_POST_BLEND:
		wb_bits(wb, VIUIN_WB_VIU, 5, 0, 4);
		if (vinfo && vinfo->cur_enc_ppc == 4) { //4 slice
			wb_bits(wb, VIUIN_WB_HDN, 0, 26, 2);
			//wr_bits_viu(VPU_VIU2VDIN_HDN_CTRL, 2, 30, 2);
		} else if (vinfo && vinfo->cur_enc_ppc == 2) { //2 slice
			wb_bits(wb, VIUIN_WB_HDN, 1, 26, 2);
			//wr_bits_viu(VPU_VIU2VDIN_HDN_CTRL, 1, 30, 2);
		} else { //1 slice
			wb_bits(wb, VIUIN_WB_HDN, 0, 26, 2);
			//wr_bits_viu(VPU_VIU2VDIN_HDN_CTRL, 2, 30, 2);
		}
		break;
	case TVIN_PORT_VIU1_VIDEO:
		wb_bits(wb, VIUIN_WB_VIU, 7, 0, 4);
		break;
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_WB1_VD1:
	case TVIN_PORT_VIU1_WB1_VD2:
	case TVIN_PORT_VIU1_WB1_OSD1:
	case TVIN_PORT_VIU1_WB1_OSD2:
	case TVIN_PORT_VIU1_WB1_POST_BLEND:
	case TVIN_PORT_VIU1_WB1_VPP:
	case TVIN_PORT_VIU1_WB1_VIDEO:
		break;
	case TVIN_PORT_VIU2_ENCL:
	case TVIN_PORT_VIU2_ENCI:
	case TVIN_PORT_VIU2_ENCP:
		wb_bits(wb, VIUIN_WB_VIU, 6, 0, 4);

		/* increase h banking in case vdin afifo overflow
		 * pre chip has 8bits
		 * tm2_revb increased 4bits, all 12bit
		 */
		wb_bits(wb, VIUIN_WB_VIU, 0xff, 16, 8);
		break;
	case TVIN_PORT_VIU2_VD1:
		wb_bits(wb, VIUIN_WB_VPP1, 1, 0, 2);
		break;
	case TVIN_PORT_VIU2_OSD1:
		wb_bits(wb, VIUIN_WB_VPP1, 2, 0, 2);
		break;
	case TVIN_PORT_VIU2_VPP:
		wb_bits(wb, VIUIN_WB_VPP1, 3, 0, 2);
		break;
	case TVIN_PORT_VIU3_VD1:
		wb_bits(wb, VIUIN_WB_VPP2, 1, 0, 2);
		break;
	case TVIN_PORT_VIU3_OSD1:
		wb_bits(wb, VIUIN_WB_VPP2, 2, 0, 2);
		break;
	case TVIN_PORT_VIU3_VPP:
		wb_bits(wb, VIUIN_WB_VPP2, 3, 0, 2);
		break;
	case TVIN_PORT_VENC0:
	case TVIN_PORT_VENC1:
	case TVIN_PORT_VENC2:
		if (vinfo && vinfo->cur_enc_ppc == 4) { //4 slice
			wb_bits(wb, VIUIN_WB_HDN, 2, 26, 2);
			wb_bits(wb, VIUIN_WB_HDN, 2, 30, 2);
		} else if (vinfo && vinfo->cur_enc_ppc == 2) { //2 slice
			wb_bits(wb, VIUIN_WB_HDN, 1, 26, 2);
			wb_bits(wb, VIUIN_WB_HDN, 1, 30, 2);
		} else { //1 slice
			wb_bits(wb, VIUIN_WB_HDN, 0, 26, 2);
			wb_bits(wb, VIUIN_WB_HDN, 0, 30, 2);
		}
		break;
	default:
		wb_bits(wb, VIUIN_WB_VIU, 0, viuin_is_wb1(port) ? 4 : 0, 4);
		wb_bits(wb, VIUIN_WB_VPP1, 0, 0, 2);
		wb_bits(wb, VIUIN_WB_VPP2, 0, 0, 2);
		break;
	}
}

static void viuin_set_wr_bak_ctrl_t3x(enum tvin_port_e port, struct viuin_wb_fields *wb)
{
	bool fixed_din_mode = true;/* vpp or venc may have 1 or 2slice input */
	const struct vinfo_s *vinfo = NULL;

	vinfo = get_current_vinfo();
	if (!vinfo) {
		pr_info("%s vinfo == NULL\n", __func__);
	} else {
		pr_info("ppc:%d,w:%d,h:%d,dur:%d\n", vinfo->cur_enc_ppc,
			vinfo->width, vinfo->height, vinfo->std_duration);
		wb_bits(wb, VIUIN_WB_HDN, vinfo->width, 0, 32);
	}

	switch (port) {
	/* wr_bak_chan1_sel wb_chan_sel*/
	//wr_bak_chan0_sel : 0:vd1 1:vd2 2:vd3 3:osd1 4:osd2 5:post_blend_out;
	case TVIN_PORT_VIU1_WB0_VD1:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_VD1, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VD2:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_VD2, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VD3:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_VD3, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD1:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_OSD1, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD2:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_OSD2, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD3:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_OSD3, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VPP:
		fixed_din_mode = false;
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_OUT, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_POST_BLEND:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP0_POST_BLD_OUT, 0, 4);
		break;
	case TVIN_PORT_VIU1_VIDEO:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP_POST_VD1_BEFORE, 0, 4);
		break;
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_WB1_VD1:
	case TVIN_PORT_VIU1_WB1_VD2:
	case TVIN_PORT_VIU1_WB1_OSD1:
	case TVIN_PORT_VIU1_WB1_OSD2:
	case TVIN_PORT_VIU1_WB1_POST_BLEND:
	case TVIN_PORT_VIU1_WB1_VPP:
	case TVIN_PORT_VIU1_WB1_VIDEO:
		break;
	case TVIN_PORT_VIU2_ENCL:
	case TVIN_PORT_VIU2_ENCI:
	case TVIN_PORT_VIU2_ENCP:
	case TVIN_PORT_VIU2_VD1:
	case TVIN_PORT_VIU2_OSD1:
	case TVIN_PORT_VIU2_VPP:
		wb_bits(wb, VIUIN_WB_VIU, T3X_WRBACK_VPP1_POST_BLD_OUT, 0, 4);
		break;
	case TVIN_PORT_VIU3_VD1:
	case TVIN_PORT_VIU3_OSD1:
	case TVIN_PORT_VIU3_VPP:
		break;
	case TVIN_PORT_VENC0:
	case TVIN_PORT_VENC1:
	case TVIN_PORT_VENC2:
		fixed_din_mode = false;
		break;
	default:
		pr_info("do not support prot = %#x\n", port);
		wb_bits(wb, VIUIN_WB_VIU, 9, 0, 4);
		break;
	}

	if (vinfo && vinfo->width * vinfo->height * vinfo->std_duration >
		VDIN_LITE_CORE_MAX_PIXEL_CLOCK) { //need skip
		/* din_mode,2ppc */
		wb_bits(wb, VIUIN_WB_HDN, 1, 30, 2);
		/* vskip:1 */
		wb_bits(wb, VIUIN_WB_HDN, 1, 24, 1);
		/* dout_mode:2 */
		wb_bits(wb, VIUIN_WB_HDN, 2, 21, 2);
		/* dn_ratio:1 */
		wb_bits(wb, VIUIN_WB_HDN, 1, 18, 2);
	} else if (!fixed_din_mode) {
		if (vinfo->cur_enc_ppc == 2)
			wb_bits(wb, VIUIN_WB_HDN, 1, 30, 2);/* 2ppc */
		else
			wb_bits(wb, VIUIN_WB_HDN, 0, 30, 2);/* 1ppc */
		wb_bits(wb, VIUIN_WB_HDN, 3, 21, 2);/* 4ppc */
	} else { //din_mode fixed 2 slice input
		wb_bits(wb, VIUIN_WB_HDN, 1, 30, 2);/* 2ppc */
		wb_bits(wb, VIUIN_WB_HDN, 3, 21, 2);/* 4ppc */
	}

}
#endif

static void viuin_set_wr_bak_ctrl(enum tvin_port_e port, struct viuin_wb_fields *wb)
{
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
	if (is_meson_s5_cpu()) {
		viuin_set_wr_bak_ctrl_s5(port, wb);
		return;
	} else if (is_meson_t3x_cpu()) {
		viuin_set_wr_bak_ctrl_t3x(port, wb);
		return;
	}
#endif

	switch (port) {
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_WB0_VD1:
		wb_bits(wb, VIUIN_WB_VPP, 1, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_VD2:
		wb_bits(wb, VIUIN_WB_VPP, 2, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_OSD1:
		wb_bits(wb, VIUIN_WB_VPP, 3, 0, 4);
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		if (is_meson_txhd2_cpu() && rd_bits_viu(VPP_MISC, 27, 1))
			wb_bits(wb, VIUIN_WB_VPP, 1, 11, 1);
#endif
		break;
	case TVIN_PORT_VIU1_WB0_OSD2:
		wb_bits(wb, VIUIN_WB_VPP, 4, 0, 4);
		break;
	case TVIN_PORT_VIU1_WB0_POST_BLEND:
		wb_bits(wb, VIUIN_WB_VPP, 5, 0, 4);
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		if (is_meson_txhd2_cpu() && rd_bits_viu(VPP_MISC, 27, 1))
			wb_bits(wb, VIUIN_WB_VPP, 1, 11, 1);
#endif
		break;
	case TVIN_PORT_VIU1_WB0_VPP:
		wb_bits(wb, VIUIN_WB_VPP, 6, 0, 4);

		/* increase h banking in case vdin afifo overflow
		 * pre chip has 8bits
		 * tm2_revb increased 4bits, all 12bit
		 */
		wb_bits(wb, VIUIN_WB_VPP, 0xff, 16, 8);
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		if (is_meson_txhd2_cpu() && rd_bits_viu(VPP_MISC, 27, 1))
			wb_bits(wb, VIUIN_WB_VPP, 1, 11, 1);
#endif
		break;
	case TVIN_PORT_VIU1_VIDEO:
		wb_bits(wb, VIUIN_WB_VPP, 7, 0, 4);
		break;
	/* wr_bak_chan1_sel wb_chan_sel*/
	case TVIN_PORT_VIU1_WB1_VD1:
		wb_bits(wb, VIUIN_WB_VPP, 1, 4, 4);
		break;
	case TVIN_PORT_VIU1_WB1_VD2:
		wb_bits(wb, VIUIN_WB_VPP, 2, 4, 4);
		break;
	case TVIN_PORT_VIU1_WB1_OSD1:
		wb_bits(wb, VIUIN_WB_VPP, 3, 4, 4);
		break;
	case TVIN_PORT_VIU1_WB1_OSD2:
		wb_bits(wb, VIUIN_WB_VPP, 4, 4, 4);
		break;
	case TVIN_PORT_VIU1_WB1_POST_BLEND:
		wb_bits(wb, VIUIN_WB_VPP, 5, 4, 4);
		break;
	case TVIN_PORT_VIU1_WB1_VPP:
		wb_bits(wb, VIUIN_WB_VPP, 6, 4, 4);

		/* increase h banking in case vdin afifo overflow
		 * pre chip has 8bits
		 * tm2_revb increased 4bits, all 12bit
		 */
		wb_bits(wb, VIUIN_WB_VPP, 0xff, 16, 8);
		break;
	case TVIN_PORT_VIU1_WB1_VIDEO:
		wb_bits(wb, VIUIN_WB_VPP, 7, 4, 4);
		break;
	case TVIN_PORT_VIU2_ENCL:
	case TVIN_PORT_VIU2_ENCI:
	case TVIN_PORT_VIU2_ENCP:
		wb_bits(wb, VIUIN_WB_VPP, 6, 0, 4);

		/* increase h banking in case vdin afifo overflow
		 * pre chip has 8bits
		 * tm2_revb increased 4bits, all 12bit
		 */
		wb_bits(wb, VIUIN_WB_VPP, 0xff, 16, 8);
		break;
	case TVIN_PORT_VIU2_VD1:
		wb_bits(wb, VIUIN_WB_VPP1, 1, 0, 2);
		break;
	case TVIN_PORT_VIU2_OSD1:
		wb_bits(wb, VIUIN_WB_VPP1, 2, 0, 2);
		break;
	case TVIN_PORT_VIU2_VPP:
		wb_bits(wb, VIUIN_WB_VPP1, 3, 0, 2);
		break;
	case TVIN_PORT_VIU3_VD1:
		wb_bits(wb, VIUIN_WB_VPP2, 1, 0, 2);
		break;
	case TVIN_PORT_VIU3_OSD1:
		wb_bits(wb, VIUIN_WB_VPP2, 2, 0, 2);
		break;
	case TVIN_PORT_VIU3_VPP:
		wb_bits(wb, VIUIN_WB_VPP2, 3, 0, 2);
		break;
	case TVIN_PORT_VENC0:
	case TVIN_PORT_VENC1:
	case TVIN_PORT_VENC2:
		wb_bits(wb, VIUIN_WB_VPP, 6, 0, 4);

		/* increase h banking in case vdin afifo overflow
		 * pre chip has 8bits
		 * tm2_revb increased 4bits, all 12bit
		 */
		wb_bits(wb, VIUIN_WB_VPP, 0xff, 16, 8);
		break;
	default:
		wb_bits(wb, VIUIN_WB_VPP, 0, viuin_is_wb1(port) ? 4 : 0, 4);
		wb_bits(wb, VIUIN_WB_VPP1, 0, 0, 2);
		wb_bits(wb, VIUIN_WB_VPP2, 0, 0, 2);
		break;
	}
}

static void viuin_port_property(struct tvin_frontend_s *fe, enum tvin_port_e port,
				struct tvin_sig_property_s *prop);

/* under viuin_lock: this frontend stops reading the channels in chans */
static void viuin_put_chans(unsigned int chans)
{
	unsigned int c;

	for (c = 0; c < VIUIN_CHAN_MAX; c++) {
		if (!(chans & BIT(c)) || --viuin_chan[c].readers)
			continue;
		wr_bits_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0, c ? 16 : 0, 16);
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		/* txhd2 keystone path close */
		if (is_meson_txhd2_cpu() && (viuin_wb_get(VIUIN_WB_VPP) & BIT(11)))
			viuin_wb_bits(VIUIN_WB_VPP, 0, 11, 1);
#endif
	}
}

/* under viuin_lock: binds all inputs in mask or none, programs the channels it
 * is first to read and rewrites each port to its channel's port
 */
static int viuin_bind(struct viuin_s *devp, enum tvin_port_e *ports, unsigned int mask)
{
	unsigned int *chan = devp->chan;
	unsigned int chans = devp->chans;
	struct viuin_wb_fields *wb;
	unsigned int sw = VIUIN_CHAN_MAX;
	unsigned int fresh = 0;
	enum tvin_port_e port, tap;
	unsigned int c, i, r;
	u32 m;
	int ret = 0;

	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (!(mask & BIT(i)))
			continue;
		devp->tap[i] = viuin_tap(ports[i]);
		for (c = 0; c < viuin_chans; c++) {
			memset(&devp->wb[i][c], 0, sizeof(devp->wb[i][c]));
			if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A))
				viuin_set_wr_bak_ctrl(viuin_chan_port(ports[i], c),
						      &devp->wb[i][c]);
		}
	}
	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (!(mask & BIT(i)))
			continue;
		tap = devp->tap[i];
		/* the channel that carries the tap, else a free one; a port
		 * that is no write-back tap has its own channel. A switched
		 * channel carries no other tap.
		 */
		chan[i] = VIUIN_CHAN_MAX;
		for (c = 0; c < viuin_chans; c++) {
			if (!viuin_is_tap(tap) && c != viuin_is_wb1(ports[i]))
				continue;
			if (viuin_chan[c].readers && viuin_chan[c].tap == tap &&
			    !(viuin_wb.writer && viuin_wb.writer->swc == c)) {
				chan[i] = c;
				break;
			}
			if (!viuin_chan[c].readers && chan[i] == VIUIN_CHAN_MAX &&
			    !(viuin_wb.writer && viuin_wb.writer->swc == c))
				chan[i] = c;
		}
		c = chan[i];
		if (c == VIUIN_CHAN_MAX || (devp->chans & BIT(c)))
			continue;
		devp->chans |= BIT(c);
		if (!viuin_chan[c].readers++) {
			viuin_chan[c].tap = tap;
			fresh |= BIT(c);
		}
	}
	/* more taps than channels: one table switches the tap field of a
	 * channel only this frontend reads. The fields share one word, so only
	 * one such table runs.
	 */
	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (!(mask & BIT(i)) || chan[i] != VIUIN_CHAN_MAX)
			continue;
		/* a writer keeps its channel while it reads it alone */
		if (viuin_wb.writer == devp && (devp->chans & BIT(devp->swc)) &&
		    viuin_chan[devp->swc].readers == 1)
			sw = devp->swc;
		/* the highest such channel; 0 serves the histogram */
		c = VIUIN_CHAN_MAX;
		while (sw == VIUIN_CHAN_MAX && (!viuin_wb.writer || viuin_wb.writer == devp) &&
		       c--) {
			if ((devp->chans & BIT(c)) && viuin_chan[c].readers == 1 &&
			    viuin_is_tap(viuin_chan[c].tap))
				sw = c;
		}
		if (sw == VIUIN_CHAN_MAX || !viuin_is_tap(devp->tap[i]))
			ret = -EBUSY;
		chan[i] = sw;
	}
	if (ret) {
		for (c = 0; c < VIUIN_CHAN_MAX; c++) {
			if ((devp->chans & ~chans) & BIT(c))
				viuin_chan[c].readers--;
		}
		devp->chans = chans;
		return ret;
	}
	devp->bound |= mask;
	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (mask & BIT(i))
			ports[i] = viuin_chan_port(ports[i], chan[i]);
	}
	/* a switched input carries every input word field another one sets */
	m = 0;
	for (i = 0; i < TVIN_TAP_MAX && sw != VIUIN_CHAN_MAX; i++) {
		if ((devp->bound & BIT(i)) && chan[i] == sw)
			m |= devp->wb[i][sw].mask[VIUIN_WB_HDN];
	}
	for (i = 0; i < TVIN_TAP_MAX && m; i++) {
		if (!(devp->bound & BIT(i)) || chan[i] != sw)
			continue;
		wb = &devp->wb[i][sw];
		wb->val[VIUIN_WB_HDN] |= devp->base.val[VIUIN_WB_HDN] & m &
					 ~wb->mask[VIUIN_WB_HDN];
		wb->mask[VIUIN_WB_HDN] |= m & devp->base.mask[VIUIN_WB_HDN];
	}
	/* a fresh channel deferred to another table lands with that table's
	 * next frame
	 */
	if (viuin_wb.writer && fresh)
		ret |= TVIN_TAP_PENDING;
	/* the first reader of a channel programs it */
	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (!(mask & BIT(i)) || !(fresh & BIT(chan[i])))
			continue;
		fresh &= ~BIT(chan[i]);
		port = ports[i];
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		if (is_meson_t3x_cpu()) {
			wr_viu(VPU_VIU2VDIN0_BUF_SIZE_T3X,
				(devp->parm.v_active << 16) | (devp->parm.h_active << 0));
			wr_viu(VPP_POST_HOLD_CTRL_T3X,	(1 << 31)    |
							(1 << 30)    |
							((devp->parm.h_active - 1) << 16) |
							(4 << 8)     |
							(1 << 4)     |
							(2 << 0));
		}
#endif
		wb = &devp->wb[i][chan[i]];
		for (r = 0; r < VIUIN_WB_NUM; r++) {
			if (wb->mask[r])
				viuin_wb_set(r, (viuin_wb_get(r) & ~wb->mask[r]) | wb->val[r]);
		}
		if (port == TVIN_PORT_VIU1_VIDEO &&
		    !is_meson_s5_cpu() && !is_meson_t3x_cpu()) {/*old chip*/
			/* enable hsync for vdin loop */
			wr_bits_viu(VIU_MISC_CTRL1, 1, 28, 1);
			devp->viu_mux = 0x4;
		}

		/* enable write back hsync */
		if  (cpu_after_eq(MESON_CPU_MAJOR_ID_T7) &&
		     !is_meson_s5_cpu() && !is_meson_t3x_cpu()) {
			wr_bits_viu(VIU_FRM_CTRL, 3, 24, 2);
			wr_bits_viu(VIU1_FRM_CTRL, 3, 24, 2);
			wr_bits_viu(VIU2_FRM_CTRL, 3, 24, 2);
		}

		/* select loopback path */
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A)) {
			/* set VPU_VIU_VDIN_IF_MUX_CTRL */
			viuin_set_vdin_if_mux_ctrl(port);
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
			if (!is_meson_s5_cpu() && !is_meson_t3x_cpu()) {
				/* wr_back hsync en */
				if (port >= TVIN_PORT_VIU1_VIDEO &&
				    port <= TVIN_PORT_VIU1_WB0_POST_BLEND) {
					wr_bits_viu(WR_BACK_MISC_CTRL, 1, 0, 1);/*vd0 hsync*/
				} else {
					wr_bits_viu(WR_BACK_MISC_CTRL, 0xf, 0, 4);
				}
			}
#endif
		} else {
			wr_bits_viu(VPU_VIU_VENC_MUX_CTRL, devp->viu_mux, 4, 4);
			wr_bits_viu(VPU_VIU_VENC_MUX_CTRL, devp->viu_mux, 8, 4);
		}
	}
	if (sw != VIUIN_CHAN_MAX) {
		for (i = 0; i < TVIN_TAP_MAX; i++) {
			for (r = 0; r < VIUIN_WB_NUM; r++) {
				if ((devp->bound & BIT(i)) && chan[i] == sw &&
				    devp->wb[i][sw].mask[r])
					viuin_wb_get(r);
			}
		}
		devp->swc = sw;
		viuin_wb.writer = devp;
		ret |= TVIN_TAP_FIELD;
	}
	return ret;
}

/*g12a/g12b and before: use viu_loop encl/encp*/
/*tl1: use viu_loop vpp */
static int viuin_tap_open(struct tvin_frontend_s *fe, enum tvin_port_e *ports,
			  unsigned int mask, unsigned int home,
			  enum tvin_port_type_e port_type)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	struct viuin_wb_fields *wb;
	unsigned long flags;
	unsigned int r;
	u32 m, val;
	int ret;

	if (!(mask & BIT(home)) || mask >= BIT(TVIN_TAP_MAX))
		return -EINVAL;
	if (!memcpy(&devp->parm, fe->private_data,
		    sizeof(struct vdin_parm_s))) {
		pr_info("[viuin..]%s memcpy error.\n", __func__);
		return -1;
	}
	/*open the venc to vdin path*/
	pr_info("viu1_sel_venc: %d\n", rd_bits_viu(VPU_VIU_VENC_MUX_CTRL, 0, 2));
	pr_info("viu2_sel_venc: %d\n", rd_bits_viu(VPU_VIU_VENC_MUX_CTRL, 2, 2));
	devp->viu_mux = 0;
	switch (rd_bits_viu(VPU_VIU_VENC_MUX_CTRL, 0, 2)) {
	case 0: /* ENCL */
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A))
			devp->viu_mux = 0x4;
		else
			devp->viu_mux = 0x8;
		/* wr_bits(VPU_VIU_VENC_MUX_CTRL,0x88,4,8); */
		devp->enc_info_addr = ENCL_INFO_READ;
		break;
	case 1: /* ENCI */
		devp->viu_mux = 0x1;/* wr_bits(VPU_VIU_VENC_MUX_CTRL,0x11,4,8); */
		devp->enc_info_addr = ENCI_INFO_READ;
		break;
	case 2: /* ENCP */
		devp->viu_mux = 0x2;/* wr_bits(VPU_VIU_VENC_MUX_CTRL,0x22,4,8); */
		devp->enc_info_addr = ENCP_INFO_READ;
		break;
	case 3: /* ENCT */
		devp->viu_mux = 0x4;/* wr_bits(VPU_VIU_VENC_MUX_CTRL,0x44,4,8); */
		devp->enc_info_addr = ENCT_INFO_READ;
		break;
	default:
		break;
	}
	memset(&devp->base, 0, sizeof(devp->base));
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
	if (is_meson_gxbb_cpu() || is_meson_gxm_cpu() || is_meson_gxl_cpu()) {
#ifndef CONFIG_AMLOGIC_REMOVE_OLD
		if (devp->parm.v_active == 2160 && devp->parm.frame_rate > 30)
			/* 1/2 down scaling */
			wb_bits(&devp->base, VIUIN_WB_HDN, 0x40f00, 0, 32);
#endif
	} else if (is_meson_s5_cpu()) {
		wb_bits(&devp->base, VIUIN_WB_HDN, devp->parm.h_active, 0, 32);
		//wr_viu(S5_VPP_POST_HOLD_CTRL, 0xc77f0412);
	} else if (!is_meson_t3x_cpu())
#endif
	{
		wb_bits(&devp->base, VIUIN_WB_HDN, devp->parm.h_active, 0, 14);
	}
	/*no need check here, will timeout sometimes*/
	/*viuin_check_venc_line(devp);*/
	spin_lock_irqsave(&viuin_lock, flags);
	if (devp->chans) {
		spin_unlock_irqrestore(&viuin_lock, flags);
		return -EBUSY;
	}
	devp->bound = 0;
	ret = viuin_bind(devp, ports, mask);
	if (ret < 0) {
		spin_unlock_irqrestore(&viuin_lock, flags);
		return ret;
	}
	fe->port = ports[home];
	devp->parm.port = ports[home];
	devp->flag = 0;
	open_cnt++;
	/* the home input's fields and the input width; the start checks them
	 * before this frontend's table runs, so the cpu writes them
	 */
	wb = &devp->wb[home][devp->chan[home]];
	for (r = 0; r < VIUIN_WB_NUM; r++) {
		m = wb->mask[r];
		val = wb->val[r];
		if (r == VIUIN_WB_HDN) {
			m |= devp->base.mask[r];
			val |= devp->base.val[r] & ~wb->mask[r];
		}
		if (!m)
			continue;
		val |= viuin_wb_get(r) & ~m;
		viuin_wb_set(r, val);
		if (viuin_wb.writer == devp)
			wr_viu(viuin_wb_reg[r], val);
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
	return ret;
}

static int viuin_tap_bind(struct tvin_frontend_s *fe, enum tvin_port_e *ports,
			  unsigned int mask)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&viuin_lock, flags);
	if (devp->chans && !(mask & devp->bound) && mask < BIT(TVIN_TAP_MAX))
		ret = viuin_bind(devp, ports, mask);
	spin_unlock_irqrestore(&viuin_lock, flags);
	return ret;
}

/* the channels no input left reads are given back; a switched channel
 * left with one input keeps that input's tap
 */
static bool viuin_tap_release(struct tvin_frontend_s *fe, unsigned int mask)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	struct viuin_wb_fields *wb;
	unsigned int chans = 0;
	unsigned int last = 0;
	unsigned int on_sw = 0;
	unsigned long flags;
	unsigned int i, r;
	bool writer;

	spin_lock_irqsave(&viuin_lock, flags);
	writer = viuin_wb.writer == devp;
	devp->bound &= ~mask;
	for (i = 0; i < TVIN_TAP_MAX; i++) {
		if (!(devp->bound & BIT(i)))
			continue;
		chans |= BIT(devp->chan[i]);
		if (writer && devp->chan[i] == devp->swc) {
			on_sw++;
			last = i;
		}
	}
	viuin_put_chans(devp->chans & ~chans);
	devp->chans = chans;
	if (on_sw == 1) {
		wb = &devp->wb[last][devp->swc];
		for (r = 0; r < VIUIN_WB_NUM; r++) {
			if (wb->mask[r])
				viuin_wb_set(r, (viuin_wb_get(r) & ~wb->mask[r]) | wb->val[r]);
		}
		viuin_chan[devp->swc].tap = devp->tap[last];
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
	return on_sw > 1;
}

static void viuin_close(struct tvin_frontend_s *fe, enum tvin_port_type_e port_type);

static int viuin_open(struct tvin_frontend_s *fe, enum tvin_port_e port,
	enum tvin_port_type_e port_type)
{
	int ret = viuin_tap_open(fe, &port, BIT(0), 0, port_type);

	/* no landing check follows this open for a field another table defers */
	if (ret > 0 && (ret & TVIN_TAP_PENDING)) {
		viuin_close(fe, port_type);
		return -EBUSY;
	}
	return ret < 0 ? ret : 0;
}

/* from the vdin isr of the frontend whose table writes the registers:
 * queue the shadow with the tap fields of input beside the write address
 * of the buffer it fills; a second call in one table rewrites the entries
 */
static void viuin_tap_queue(struct tvin_frontend_s *fe, int rdma_handle,
			    unsigned int input)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	unsigned long flags;
	unsigned int c;
	unsigned int i;
	u32 val;

	spin_lock_irqsave(&viuin_lock, flags);
	c = devp->chan[input];
	for (i = 0; viuin_wb.writer == devp && i < VIUIN_WB_NUM; i++) {
		if (!(viuin_wb.valid & BIT(i)))
			continue;
		val = viuin_wb.val[i];
		if (c == devp->swc && (devp->bound & BIT(input)))
			val = (val & ~devp->wb[input][c].mask[i]) | devp->wb[input][c].val[i];
#ifdef CONFIG_AMLOGIC_MEDIA_RDMA
		rdma_write_reg_bits(rdma_handle, viuin_wb_reg[i], val, 0, 32);
#endif
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
}

static bool viuin_tap_landed(struct tvin_frontend_s *fe, unsigned int input)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	struct viuin_wb_fields *wb;
	unsigned long flags;
	bool landed;
	unsigned int i;

	spin_lock_irqsave(&viuin_lock, flags);
	landed = devp->bound & BIT(input);
	wb = &devp->wb[input][devp->chan[input]];
	for (i = 0; landed && i < VIUIN_WB_NUM; i++) {
		if (wb->mask[i] && (rd_viu(viuin_wb_reg[i]) & wb->mask[i]) != wb->val[i])
			landed = false;
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
	return landed;
}

/* after the rdma table is cleared: the cpu writes what the table carried */
static void viuin_tap_disarm(struct tvin_frontend_s *fe)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&viuin_lock, flags);
	if (viuin_wb.writer == devp) {
		viuin_wb.writer = NULL;
		for (i = 0; i < VIUIN_WB_NUM; i++) {
			if (viuin_wb.valid & BIT(i))
				wr_viu(viuin_wb_reg[i], viuin_wb.val[i]);
		}
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
}

static void viuin_close(struct tvin_frontend_s *fe, enum tvin_port_type_e port_type)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	unsigned long flags;

	/*no need check here, will timeout sometimes*/
	/*viuin_check_venc_line(devp);*/
	/* a start that failed before its rdma table ran */
	viuin_tap_disarm(fe);
	spin_lock_irqsave(&viuin_lock, flags);
	/* a frontend whose open failed reads no channel */
	if (!devp->chans) {
		spin_unlock_irqrestore(&viuin_lock, flags);
		return;
	}
	viuin_put_chans(devp->chans);
	devp->chans = 0;
	devp->bound = 0;
	memset(&devp->parm, 0, sizeof(struct vdin_parm_s));
	/*close the venc to vdin path*/
	if (open_cnt)
		open_cnt--;
	if (!viuin_chan[0].readers && !viuin_chan[1].readers) {
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A)) {
			wr_viu(VPU_VIU_VDIN_IF_MUX_CTRL, 0);
			viuin_wb_set(VIUIN_WB_VPP, 0);
		} else {
			wr_bits_viu(VPU_VIU_VENC_MUX_CTRL, 0, 8, 4);
			wr_bits_viu(VPU_VIU_VENC_MUX_CTRL, 0, 4, 4);
		}
		if (viuin_wb_get(VIUIN_WB_HDN) != 0)
			viuin_wb_set(VIUIN_WB_HDN, 0x0);
	}
	spin_unlock_irqrestore(&viuin_lock, flags);
}

static void viuin_start(struct tvin_frontend_s *fe, enum tvin_sig_fmt_e fmt,
	enum tvin_port_type_e port_type)
{
	/* do something the same as start_amvdec_viu_in */
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);

	if (devp->flag && AMVIUIN_DEC_START) {
		pr_info("[viuin..]%s viu_in is started already.\n", __func__);
		return;
	}
	vsync_enter_line_max = 0;
	vsync_enter_line_threshold_overflow_count = 0;
	devp->flag = AMVIUIN_DEC_START;
}

static void viuin_stop(struct tvin_frontend_s *fe, enum tvin_port_e port,
	enum tvin_port_type_e port_type)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);

	if (devp->flag && AMVIUIN_DEC_START)
		devp->flag |= AMVIUIN_DEC_STOP;
	else
		pr_info("[viuin..]%s viu in dec isn't start.\n", __func__);
	/* the last reader's close disables the channel */
	//pr_info("%s %d Disable VIU to VDIN\n", __func__, __LINE__);
}

static int viuin_isr(struct tvin_frontend_s *fe, unsigned int hcnt64,
	enum tvin_port_type_e port_type)
{
	int curr_port;

	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);

	curr_port = rd_bits_viu(VPU_VIU_VENC_MUX_CTRL, 0, 2);

	vsync_enter_line_curr = (rd_viu(devp->enc_info_addr) >> 16) & 0x1fff;
	if (vsync_enter_line_curr > vsync_enter_line_max)
		vsync_enter_line_max = vsync_enter_line_curr;
	if (vsync_enter_line_max_threshold > vsync_enter_line_min_threshold &&
	    curr_port == 0) {
		if (vsync_enter_line_curr > vsync_enter_line_max_threshold ||
		    vsync_enter_line_curr < vsync_enter_line_min_threshold) {
			vsync_enter_line_threshold_overflow_count++;
			return TVIN_BUF_SKIP;
		}
	}
	return 0;
}

static struct tvin_decoder_ops_s viu_dec_ops = {
	.support            = viuin_support,
	.open               = viuin_open,
	.start              = viuin_start,
	.stop               = viuin_stop,
	.close              = viuin_close,
	.decode_isr         = viuin_isr,
	.tap_open           = viuin_tap_open,
	.tap_bind           = viuin_tap_bind,
	.tap_release        = viuin_tap_release,
	.tap_queue          = viuin_tap_queue,
	.tap_landed         = viuin_tap_landed,
	.tap_disarm         = viuin_tap_disarm,
	.tap_property       = viuin_port_property,
};

static void viuin_port_property(struct tvin_frontend_s *fe, enum tvin_port_e port,
				struct tvin_sig_property_s *prop)
{
	static const struct vinfo_s *vinfo;
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);
	unsigned int line_begin = 0, line_end = 0;
	unsigned int venc_offset = 0;

	switch (port) {
	case TVIN_PORT_VIU1_VIDEO:
	case TVIN_PORT_VIU1_WB0_POST_BLEND:
	case TVIN_PORT_VIU2_VD1:
		prop->color_format = TVIN_YUV444;
		break;
	case TVIN_PORT_VIU1:
	case TVIN_PORT_VIU2:
	case TVIN_PORT_VIU1_WB0_VPP:
	case TVIN_PORT_VIU1_WB1_VPP:
		vinfo = get_current_vinfo();
		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;
		break;
	case TVIN_PORT_VIU2_VPP:
	#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
		vinfo = get_current_vinfo2();
		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;
		pr_info("color_format:%#x\n", prop->color_format);
	#else
		prop->color_format = TVIN_YUV444;
		pr_info("def color_format:%#x\n", prop->color_format);
	#endif
		break;

	/* ENCL/ENCI/ENCP is only for viu2 loopback currently
	 * though hw also support viu1 loopback through ENC
	 */
	case TVIN_PORT_VIU2_ENCL:
		vinfo = get_current_vinfo2();
		line_begin = rd_viu(ENCL_VIDEO_VSO_BLINE);
		line_end = rd_viu(ENCL_VIDEO_VSO_ELINE);

		if (line_begin < line_end)
			prop->polarity_vs = 1; /* negative */
		else if (line_begin > line_end)
			prop->polarity_vs = 0; /* positive */
		else
			pr_err("error: TVIN_PORT_VIU2_ENCL line begin = end\n");

		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;
		break;
	case TVIN_PORT_VIU2_ENCI:
		vinfo = get_current_vinfo2();

		/* always negative */
		prop->polarity_vs = 1; /* negative */
		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;
		break;
	case TVIN_PORT_VIU2_ENCP:
		vinfo = get_current_vinfo2();
		line_begin = rd_viu(ENCP_VIDEO_VSO_BLINE);
		line_end = rd_viu(ENCP_VIDEO_VSO_ELINE);

		if (line_begin < line_end)
			prop->polarity_vs = 1; /* negative */
		else if (line_begin > line_end)
			prop->polarity_vs = 0; /* positive */
		else
			pr_err("error: TVIN_PORT_VIU2_ENCP line begin = end\n");

		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;
		break;
	case TVIN_PORT_VENC0:
	case TVIN_PORT_VENC1:
	case TVIN_PORT_VENC2:
#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
		if (is_meson_s5_cpu())
			vinfo = get_current_vinfo();
		else
#endif
			vinfo = get_current_vinfo2();

		if (devp->parm.port == TVIN_PORT_VENC1)
			venc_offset = 0x600;
		else if (devp->parm.port == TVIN_PORT_VENC2)
			venc_offset = 0x800;
		else
			venc_offset = 0;
		if ((vinfo->viu_mux & 0xf) == VIU_MUX_ENCL) {
			line_begin = rd_viu(ENCL_VIDEO_VSO_BLINE + venc_offset);
			line_end = rd_viu(ENCL_VIDEO_VSO_ELINE + venc_offset);
		} else if ((vinfo->viu_mux & 0xf) == VIU_MUX_ENCI) {
			prop->polarity_vs = 1;
		} else if ((vinfo->viu_mux & 0xf) == VIU_MUX_ENCP) {
			line_begin = rd_viu(ENCP_VIDEO_VSO_BLINE + venc_offset);
			line_end = rd_viu(ENCP_VIDEO_VSO_ELINE + venc_offset);
		}

		if (line_begin < line_end || cpu_after_eq(MESON_CPU_MAJOR_ID_T7))
			prop->polarity_vs = 1; /* negative */
		else if (line_begin > line_end)
			prop->polarity_vs = 0; /* positive */
		else
			pr_err("error: TVIN_PORT_VIU2_ENCP line begin = end\n");

		prop->color_format = (enum tvin_color_fmt_e)vinfo->viu_color_fmt;

		pr_info("%s cfmt=%d;line:%d,%d,ppc:%d,name:%s,mode:%d,w:%d,h:%d\n",
			__func__, prop->color_format, line_begin, line_end,
			vinfo->cur_enc_ppc, vinfo->name, vinfo->mode,
			vinfo->width, vinfo->height);
		break;
	default:
		prop->color_format = devp->parm.cfmt;
		break;
	}

	prop->dest_cfmt = devp->parm.dfmt;

	prop->scaling4w = devp->parm.dest_h_active;
	prop->scaling4h = devp->parm.dest_v_active;

	prop->vs = v_cut_offset;
	prop->ve = 0;
	prop->hs = 0;
	prop->he = 0;
	prop->decimation_ratio = 0;
}

static void viuin_sig_property(struct tvin_frontend_s *fe,
			       struct tvin_sig_property_s *prop,
			       enum tvin_port_type_e port_type)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);

	viuin_port_property(fe, devp->parm.port, prop);
}

static bool viu_check_frame_skip(struct tvin_frontend_s *fe, enum tvin_port_type_e port_type)
{
	struct viuin_s *devp = container_of(fe, struct viuin_s, frontend);

	if (devp->parm.skip_count > 0) {
		devp->parm.skip_count--;
		return true;
	}

	return false;
}

static struct tvin_state_machine_ops_s viu_sm_ops = {
	.get_sig_property = viuin_sig_property,
	.check_frame_skip = viu_check_frame_skip,
};

static int viuin_probe(struct platform_device *pdev)
{
	struct viuin_s *viuin_devp;
	int i;

	viuin_devp = kcalloc(VIUIN_FE_NUM, sizeof(*viuin_devp), GFP_KERNEL);
	if (!viuin_devp)
		return -ENOMEM;
	for (i = 0; i < VIUIN_FE_NUM; i++) {
		if (i)
			sprintf(viuin_devp[i].frontend.name, "%s%d", DEVICE_NAME, i);
		else
			sprintf(viuin_devp[i].frontend.name, "%s", DEVICE_NAME);
		if (!tvin_frontend_init(&viuin_devp[i].frontend,
					&viu_dec_ops, &viu_sm_ops, i)) {
			if (tvin_reg_frontend(&viuin_devp[i].frontend))
				pr_info("[viuin..]%s register viu frontend error.\n",
					__func__);
		}
	}
	platform_set_drvdata(pdev, viuin_devp);
	pr_info("[viuin..]%s probe ok.\n", __func__);
	return 0;
}

static int viuin_remove(struct platform_device *pdev)
{
	struct viuin_s *devp = platform_get_drvdata(pdev);
	int i;

	if (devp) {
		for (i = 0; i < VIUIN_FE_NUM; i++)
			tvin_unreg_frontend(&devp[i].frontend);
		kfree(devp);
	}
	return 0;
}

static struct platform_driver viuin_driver = {
	.probe	= viuin_probe,
	.remove	= viuin_remove,
	.driver	= {
		.name	= DEVICE_NAME,
	}
};

static struct platform_device *viuin_device;

int __init viuin_init_module(void)
{
	pr_info("[viuin..]%s viuin module init\n", __func__);
	viuin_device = platform_device_alloc(DEVICE_NAME, 0);
	if (!viuin_device) {
		pr_err("[viuin..]%s failed to alloc viuin_device.\n",
		       __func__);
		return -ENOMEM;
	}

	if (platform_device_add(viuin_device)) {
		platform_device_put(viuin_device);
		pr_err("[viuin..]%sfailed to add viuin_device.\n", __func__);
		return -ENODEV;
	}
	if (platform_driver_register(&viuin_driver)) {
		pr_err("[viuin..]%sfailed to register viuin driver.\n",
		       __func__);
		platform_device_del(viuin_device);
		platform_device_put(viuin_device);
		return -ENODEV;
	}

	pr_info("[viuin..]%s done\n", __func__);
	return 0;
}

void __exit viuin_exit_module(void)
{
	pr_info("[viuin..]%s viuin module remove.\n", __func__);
	platform_driver_unregister(&viuin_driver);
	platform_device_unregister(viuin_device);
}

//MODULE_DESCRIPTION("AMLOGIC viu input driver");
//MODULE_LICENSE("GPL");
//MODULE_VERSION("3.0.0");
