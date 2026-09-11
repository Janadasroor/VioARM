/*
 * Structures for model: vioarm_adc_bridge
 *
 * Hand-written (no cmpp preprocessor).
 *
 * Analog voltage input -> VioARM analog signal bank injection.
 */

#include "ngspice/ngspice.h"
#include <stdio.h>
#include "ngspice/devdefs.h"
#include "ngspice/ifsim.h"
#include "ngspice/mifdefs.h"
#include "ngspice/mifproto.h"
#include "ngspice/mifparse.h"


static IFparm MIFmPTable[] = {
    IOP("channel", 0, IF_REAL, "Starting analog signal bank channel (0-127)"),
};


static Mif_Port_Type_t MIFportEnum0[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};


static char *MIFportStr0[] = {
    "v",
    "vd",
    "i",
    "id",
};


static Mif_Port_Type_t MIFportEnum1[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};


static char *MIFportStr1[] = {
    "v",
    "vd",
    "i",
    "id",
};


static Mif_Conn_Info_t MIFconnTable[] = {
  {
    "in",
    "analog input voltage",
    MIF_IN,
    MIF_VOLTAGE,
    "v",
    4,
    MIFportEnum0,
    MIFportStr0,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
  {
    "out",
    "unity-gain output (mirrors input)",
    MIF_OUT,
    MIF_VOLTAGE,
    "v",
    4,
    MIFportEnum1,
    MIFportStr1,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};

static union Mif_Parse_Value channel_default[] = {
    { .rvalue=0.000000e+00 },
};


static Mif_Param_Info_t MIFparamTable[] = {
  {
    "channel",
    "Starting analog signal bank channel (0-127)",
    MIF_REAL,
    1,
    channel_default,
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};


extern void cm_vioarm_adc_bridge(Mif_Private_t *);
extern void cm_vioarm_dac_bridge(Mif_Private_t *);
extern void cm_vioarm_vcc_bridge(Mif_Private_t *);

/* ======================== vioarm_adc_bridge ======================== */

static int adc_val_terms             = 0;
static int adc_val_numNames          = 0;
static int adc_val_numInstanceParms  = 0;
static int adc_val_numModelParms     = 1;
static int adc_val_sizeofMIFinstance = sizeof(MIFinstance);
static int adc_val_sizeofMIFmodel    = sizeof(MIFmodel);

SPICEdev cm_vioarm_adc_bridge_info = {
    .DEVpublic = {
        .name = "vioarm_adc_bridge",
        .description = "VioARM analog signal bank injector",
        .terms = &adc_val_terms,
        .numNames = &adc_val_numNames,
        .termNames = NULL,
        .numInstanceParms = &adc_val_numInstanceParms,
        .instanceParms = NULL,
        .numModelParms = &adc_val_numModelParms,
        .modelParms = MIFmPTable,
        .flags = 0,

        .cm_func = cm_vioarm_adc_bridge,
        .num_conn = 2,
        .conn = MIFconnTable,
        .num_param = 1,
        .param = MIFparamTable,
        .num_inst_var = 0,
        .inst_var = NULL,
    },

    .DEVparam = NULL,
    .DEVmodParam = MIFmParam,
    .DEVload = MIFload,
    .DEVsetup = MIFsetup,
    .DEVunsetup = MIFunsetup,
    .DEVpzSetup = NULL,
    .DEVtemperature = NULL,
    .DEVtrunc = MIFtrunc,
    .DEVfindBranch = NULL,
    .DEVacLoad = MIFload,
    .DEVaccept = NULL,
    .DEVdestroy = NULL,
    .DEVmodDelete = MIFmDelete,
    .DEVdelete = MIFdelete,
    .DEVsetic = NULL,
    .DEVask = MIFask,
    .DEVmodAsk = MIFmAsk,
    .DEVpzLoad = NULL,
    .DEVconvTest = MIFconvTest,
    .DEVsenSetup = NULL,
    .DEVsenLoad = NULL,
    .DEVsenUpdate = NULL,
    .DEVsenAcLoad = NULL,
    .DEVsenPrint = NULL,
    .DEVsenTrunc = NULL,
    .DEVdisto = NULL,
    .DEVnoise = NULL,
    .DEVsoaCheck = NULL,
    .DEVinstSize = &adc_val_sizeofMIFinstance,
    .DEVmodSize = &adc_val_sizeofMIFmodel,

#ifdef CIDER
    .DEVdump = NULL,
    .DEVacct = NULL,
#endif

#ifdef KLU
    .DEVbindCSC = MIFbindCSC,
    .DEVbindCSCComplex = MIFbindCSCComplex,
    .DEVbindCSCComplexToReal = MIFbindCSCComplexToReal,
#endif
};

/* ======================== vioarm_dac_bridge ======================== */

static IFparm DAC_MIFmPTable[] = {
    IOP("channel", 0, IF_REAL, "DAC output channel index (0-3)"),
};

static Mif_Port_Type_t DAC_MIFportEnum0[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};

static char *DAC_MIFportStr0[] = {
    "v",
    "vd",
    "i",
    "id",
};

static Mif_Port_Type_t DAC_MIFportEnum1[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};

static char *DAC_MIFportStr1[] = {
    "v",
    "vd",
    "i",
    "id",
};

static Mif_Conn_Info_t DAC_MIFconnTable[] = {
  {
    "in",
    "dummy input (unused, for connection parsing)",
    MIF_IN,
    MIF_VOLTAGE,
    "v",
    4,
    DAC_MIFportEnum0,
    DAC_MIFportStr0,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
  {
    "out",
    "DAC analog voltage output",
    MIF_OUT,
    MIF_VOLTAGE,
    "v",
    4,
    DAC_MIFportEnum1,
    DAC_MIFportStr1,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};

static union Mif_Parse_Value DAC_channel_default[] = {
    { .rvalue=0.000000e+00 },
};

static Mif_Param_Info_t DAC_MIFparamTable[] = {
  {
    "channel",
    "DAC output channel index (0-3)",
    MIF_REAL,
    1,
    DAC_channel_default,
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};

static int dac_val_terms             = 0;
static int dac_val_numNames          = 0;
static int dac_val_numInstanceParms  = 0;
static int dac_val_numModelParms     = 1;
static int dac_val_sizeofMIFinstance = sizeof(MIFinstance);
static int dac_val_sizeofMIFmodel    = sizeof(MIFmodel);

SPICEdev cm_vioarm_dac_bridge_info = {
    .DEVpublic = {
        .name = "vioarm_dac_bridge",
        .description = "VioARM DAC analog voltage output",
        .terms = &dac_val_terms,
        .numNames = &dac_val_numNames,
        .termNames = NULL,
        .numInstanceParms = &dac_val_numInstanceParms,
        .instanceParms = NULL,
        .numModelParms = &dac_val_numModelParms,
        .modelParms = DAC_MIFmPTable,
        .flags = 0,

        .cm_func = cm_vioarm_dac_bridge,
        .num_conn = 2,
        .conn = DAC_MIFconnTable,
        .num_param = 1,
        .param = DAC_MIFparamTable,
        .num_inst_var = 0,
        .inst_var = NULL,
    },

    .DEVparam = NULL,
    .DEVmodParam = MIFmParam,
    .DEVload = MIFload,
    .DEVsetup = MIFsetup,
    .DEVunsetup = MIFunsetup,
    .DEVpzSetup = NULL,
    .DEVtemperature = NULL,
    .DEVtrunc = MIFtrunc,
    .DEVfindBranch = NULL,
    .DEVacLoad = MIFload,
    .DEVaccept = NULL,
    .DEVdestroy = NULL,
    .DEVmodDelete = MIFmDelete,
    .DEVdelete = MIFdelete,
    .DEVsetic = NULL,
    .DEVask = MIFask,
    .DEVmodAsk = MIFmAsk,
    .DEVpzLoad = NULL,
    .DEVconvTest = MIFconvTest,
    .DEVsenSetup = NULL,
    .DEVsenLoad = NULL,
    .DEVsenUpdate = NULL,
    .DEVsenAcLoad = NULL,
    .DEVsenPrint = NULL,
    .DEVsenTrunc = NULL,
    .DEVdisto = NULL,
    .DEVnoise = NULL,
    .DEVsoaCheck = NULL,
    .DEVinstSize = &dac_val_sizeofMIFinstance,
    .DEVmodSize = &dac_val_sizeofMIFmodel,

#ifdef CIDER
    .DEVdump = NULL,
    .DEVacct = NULL,
#endif

#ifdef KLU
    .DEVbindCSC = MIFbindCSC,
    .DEVbindCSCComplex = MIFbindCSCComplex,
    .DEVbindCSCComplexToReal = MIFbindCSCComplexToReal,
#endif
};

/* ======================== vioarm_vcc_bridge ======================== */

static IFparm VCC_MIFmPTable[] = {
    IOP("dummy", 0, IF_REAL, "unused (no parameters needed)"),
};

static Mif_Port_Type_t VCC_MIFportEnum0[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};

static char *VCC_MIFportStr0[] = {
    "v",
    "vd",
    "i",
    "id",
};

static Mif_Port_Type_t VCC_MIFportEnum1[] = {
    MIF_VOLTAGE,
    MIF_DIFF_VOLTAGE,
    MIF_CURRENT,
    MIF_DIFF_CURRENT,
};

static char *VCC_MIFportStr1[] = {
    "v",
    "vd",
    "i",
    "id",
};

static Mif_Conn_Info_t VCC_MIFconnTable[] = {
  {
    "vcc",
    "VCC voltage input",
    MIF_IN,
    MIF_VOLTAGE,
    "v",
    4,
    VCC_MIFportEnum0,
    VCC_MIFportStr0,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
  {
    "out",
    "unity-gain output (mirror for Newton)",
    MIF_OUT,
    MIF_VOLTAGE,
    "v",
    4,
    VCC_MIFportEnum1,
    VCC_MIFportStr1,
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};

static union Mif_Parse_Value VCC_dummy_default[] = {
    { .rvalue=0.000000e+00 },
};

static Mif_Param_Info_t VCC_MIFparamTable[] = {
  {
    "dummy",
    "unused (no parameters)",
    MIF_REAL,
    1,
    VCC_dummy_default,
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    { .bvalue=MIF_FALSE },
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_TRUE,
  },
};

static int vcc_val_terms             = 0;
static int vcc_val_numNames          = 0;
static int vcc_val_numInstanceParms  = 0;
static int vcc_val_numModelParms     = 1;
static int vcc_val_sizeofMIFinstance = sizeof(MIFinstance);
static int vcc_val_sizeofMIFmodel    = sizeof(MIFmodel);

SPICEdev cm_vioarm_vcc_bridge_info = {
    .DEVpublic = {
        .name = "vioarm_vcc_bridge",
        .description = "VioARM dynamic VCC tracking bridge",
        .terms = &vcc_val_terms,
        .numNames = &vcc_val_numNames,
        .termNames = NULL,
        .numInstanceParms = &vcc_val_numInstanceParms,
        .instanceParms = NULL,
        .numModelParms = &vcc_val_numModelParms,
        .modelParms = VCC_MIFmPTable,
        .flags = 0,

        .cm_func = cm_vioarm_vcc_bridge,
        .num_conn = 2,
        .conn = VCC_MIFconnTable,
        .num_param = 1,
        .param = VCC_MIFparamTable,
        .num_inst_var = 0,
        .inst_var = NULL,
    },

    .DEVparam = NULL,
    .DEVmodParam = MIFmParam,
    .DEVload = MIFload,
    .DEVsetup = MIFsetup,
    .DEVunsetup = MIFunsetup,
    .DEVpzSetup = NULL,
    .DEVtemperature = NULL,
    .DEVtrunc = MIFtrunc,
    .DEVfindBranch = NULL,
    .DEVacLoad = MIFload,
    .DEVaccept = NULL,
    .DEVdestroy = NULL,
    .DEVmodDelete = MIFmDelete,
    .DEVdelete = MIFdelete,
    .DEVsetic = NULL,
    .DEVask = MIFask,
    .DEVmodAsk = MIFmAsk,
    .DEVpzLoad = NULL,
    .DEVconvTest = MIFconvTest,
    .DEVsenSetup = NULL,
    .DEVsenLoad = NULL,
    .DEVsenUpdate = NULL,
    .DEVsenAcLoad = NULL,
    .DEVsenPrint = NULL,
    .DEVsenTrunc = NULL,
    .DEVdisto = NULL,
    .DEVnoise = NULL,
    .DEVsoaCheck = NULL,
    .DEVinstSize = &vcc_val_sizeofMIFinstance,
    .DEVmodSize = &vcc_val_sizeofMIFmodel,

#ifdef CIDER
    .DEVdump = NULL,
    .DEVacct = NULL,
#endif

#ifdef KLU
    .DEVbindCSC = MIFbindCSC,
    .DEVbindCSCComplex = MIFbindCSCComplex,
    .DEVbindCSCComplexToReal = MIFbindCSCComplexToReal,
#endif
};
