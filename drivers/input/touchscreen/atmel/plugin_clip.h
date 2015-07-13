
#ifndef __LINUX_ATMEL_MXT_PLUG_CLIP
#define __LINUX_ATMEL_MXT_PLUG_CLIP

#include <linux/types.h>

extern struct plugin_clip mxt_plugin_clp;

ssize_t plugin_clip_show(struct plugin_clip *p, char *buf, size_t count);
int plugin_clip_store(struct plugin_clip *p, const char *buf, size_t count);
ssize_t plugin_clip_tag_show(struct plugin_clip *p, char *buf, size_t count);

enum {
	CL_LEFT = 0,
	CL_UP,
	CL_RIGHT,
	CL_DOWN,
	CL_NUM
};

enum {
	SUP_LEFT_UP = 0,
	SUP_RIGHT_UP,
	SUP_LEFT_DOWN,
	SUP_RIGHT_DOWN,
	SUP_NUM
};

enum {
	POS_LEFT = 0,
	POS_RIGHT,
	POS_NUM
};
#define NUM_CLIP_CL_AREA CL_NUM
#define NUM_CLIP_SUP_AREA SUP_NUM
#define NUM_CLIP_PA_AREA 1
#define NUM_CLIP_POS_AREA POS_NUM

#endif


