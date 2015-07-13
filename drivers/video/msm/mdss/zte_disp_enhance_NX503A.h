

struct mdp_pcc_cfg_data zte_pcc_cfg_warm = {
	.block = 0x10,
	.ops = 0x5,
    {
      .c = 0,
      .r = 0x8000,
      .g = 0,
      .b = 0,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
    {
      .c = 0,
      .r = 0,
      .g = 0x7900,
      .b = 0,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
    {
      .c = 0,
      .r = 0,
      .g = 0,
      .b = 0x7900,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
};


struct mdp_pcc_cfg_data zte_pcc_cfg_normal = {
	.block = 0x10,
	.ops = 0x5,
    {
      .c = 0,
      .r = 0x8000,
      .g = 0,
      .b = 0,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
    {
      .c = 0,
      .r = 0,
      .g = 0x8000,
      .b = 0,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
    {
      .c = 0,
      .r = 0,
      .g = 0,
      .b = 0x8000,
      .rr = 0,
      .gg = 0,
      .bb = 0,
      .rg = 0,
      .gb = 0,
      .rb = 0,
      .rgb_0 = 0,
      .rgb_1 = 0
    },
};
struct mdp_pcc_cfg_data zte_pcc_cfg_cool = {
	.block = 0x10,
	.ops = 0x5,
    {
      .c = 0,
      .r = 0x7800,
      .g = 0,
      .b = 0,
    },
    {
      .c = 0,
      .r = 0,
      .g = 0x8000,
      .b = 0,
    },
    {
      .c = 0,
      .r = 0,
      .g = 0,
      .b = 0x8000,
    },
};

static char sharpca_basic_9b[] = {0xca, 0x01,0x80, 0x98,0x98,0x9b,0x40,0xbe,0xbe,0x20,0x20,
	0x80,0xfe,0x0a, 0x4a,0x37,0xa0,0x55,0xf8,0x0c,0x0c,0x20,0x10,0x3f,0x3f,0x00,
	0x00,0x10,0x10,0x3f,0x3f,0x3f,0x3f,
};
static struct dsi_cmd_desc display_glow_cmd = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1, sizeof(sharpca_basic_9b)}, sharpca_basic_9b

};

static char sharpca_basic_92[] = {0xca, 0x01,0x80, 0x92,0x92,0x9b,0x75,0x9b,0x9b,0x20,0x20,
	0x80,0xfe,0x0a, 0x4a,0x37,0xa0,0x55,0xf8,0x0c,0x0c,0x20,0x10,0x3f,0x3f,0x00,
	0x00,0x10,0x10,0x3f,0x3f,0x3f,0x3f,
};

static struct dsi_cmd_desc display_std_cmd = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1, sizeof(sharpca_basic_92)}, sharpca_basic_92

};
static char sharpca_basic_norm[] = {0xca, 0x00,0x80, 0x80,0x80,0x80,0x80,0x80,0x80,0x08,0x20,
	0x80,0x80,0x0a, 0x4a,0x37,0xa0,0x55,0xf8,0x0c,0x0c,0x20,0x10,0x3f,0x3f,0x00,
	0x00,0x10,0x10,0x3f,0x3f,0x3f,0x3f,
};

static struct dsi_cmd_desc display_soft_cmd = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1, sizeof(sharpca_basic_norm)}, sharpca_basic_norm

};


