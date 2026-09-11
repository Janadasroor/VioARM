/* Minimal dlmain for vioarm_adc_bridge code model.
 *
 * Provides the CM exports and MIF delegation functions
 * needed by ngspice's XSPICE framework for a standalone .cm library.
 *
 * Derived from VioMATRIXC src/xspice/icm/dlmain.c.
 */

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "ngspice/config.h"
#include "ngspice/cpextern.h"
#include "ngspice/devdefs.h"
#include "ngspice/dllitf.h"
#include "ngspice/evtudn.h"
#include "ngspice/inpdefs.h"
#include "ngspice/inertial.h"

/* Three models in this .cm */
extern SPICEdev cm_vioarm_adc_bridge_info;
extern SPICEdev cm_vioarm_dac_bridge_info;
extern SPICEdev cm_vioarm_vcc_bridge_info;

SPICEdev *cmDEVices[] = {
    &cm_vioarm_adc_bridge_info,
    &cm_vioarm_dac_bridge_info,
    &cm_vioarm_vcc_bridge_info,
    NULL
};

int cmDEVicesCNT = sizeof(cmDEVices)/sizeof(SPICEdev *)-1;

/* No UDN models */
Evt_Udn_Info_t  *cmEVTudns[] = {
    NULL
};

int cmEVTudnCNT = 0;

struct coreInfo_t *coreitf = (struct coreInfo_t *) NULL;

#if defined (__MINGW32__) || defined (__CYGWIN__) || defined (_MSC_VER)
#define CM_EXPORT __declspec(dllexport)
#else
#if __GNUC__ >= 4
#define CM_EXPORT __attribute__ ((visibility ("default")))
#define CM_EXPORT_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define CM_EXPORT
#define CM_EXPORT_LOCAL
#endif
#endif

extern CM_EXPORT void *CMdevs(void);
extern CM_EXPORT void *CMdevNum(void);
extern CM_EXPORT void *CMudns(void);
extern CM_EXPORT void *CMudnNum(void);
extern CM_EXPORT void *CMgetCoreItfPtr(void);

CM_EXPORT void *CMdevs(void) {
    return (void *)cmDEVices;
}

CM_EXPORT void *CMdevNum(void) {
    return (void *)&cmDEVicesCNT;
}

CM_EXPORT void *CMudns(void) {
    return (void *)cmEVTudns;
}

CM_EXPORT void *CMudnNum(void) {
    return (void *)&cmEVTudnCNT;
}

CM_EXPORT void *CMgetCoreItfPtr(void) {
    return (void *)(&coreitf);
}

/* MIF wrapper functions — delegate through coreitf */

int MIFsetup(
    SMPmatrix     *matrix,
    GENmodel      *inModel,
    CKTcircuit    *ckt,
    int           *state)
{
    return (coreitf->dllitf_MIFsetup)(matrix,inModel,ckt,state);
}

int MIFunsetup(
    GENmodel      *inModel,
    CKTcircuit    *ckt)
{
    return (coreitf->dllitf_MIFunsetup)(inModel,ckt);
}

int MIFload(
    GENmodel      *inModel,
    CKTcircuit    *ckt)
{
    return (coreitf->dllitf_MIFload)(inModel,ckt);
}

int MIFmParam(
    int param_index,
    IFvalue *value,
    GENmodel *inModel)
{
    return (coreitf->dllitf_MIFmParam)(param_index,value,inModel);
}

int MIFask(
    CKTcircuit *ckt,
    GENinstance *inst,
    int param_index,
    IFvalue *value,
    IFvalue *select)
{
    return (coreitf->dllitf_MIFask)(ckt,inst,param_index,value,select);
}

int MIFmAsk(
    CKTcircuit *ckt,
    GENmodel *inModel,
    int param_index,
    IFvalue *value)
{
    return (coreitf->dllitf_MIFmAsk)(ckt,inModel,param_index,value);
}

int MIFtrunc(
    GENmodel   *inModel,
    CKTcircuit *ckt,
    double     *timeStep)
{
    return (coreitf->dllitf_MIFtrunc)(inModel,ckt,timeStep);
}

int MIFconvTest(
    GENmodel   *inModel,
    CKTcircuit *ckt)
{
    return (coreitf->dllitf_MIFconvTest)(inModel,ckt);
}

int MIFdelete(
    GENinstance  *inst)
{
    return (coreitf->dllitf_MIFdelete)(inst);
}

int MIFmDelete(
    GENmodel *gen_model)
{
    return (coreitf->dllitf_MIFmDelete)(gen_model);
}

void MIFdestroy(void)
{
    (coreitf->dllitf_MIFdestroy)();
}

#ifdef KLU
int MIFbindCSC(GENmodel *inModel, CKTcircuit *ckt)
{
    return (coreitf->dllitf_MIFbindCSC)(inModel,ckt);
}

int MIFbindCSCComplex(GENmodel *inModel, CKTcircuit *ckt)
{
    return (coreitf->dllitf_MIFbindCSCComplex)(inModel,ckt);
}

int MIFbindCSCComplexToReal(GENmodel *inModel, CKTcircuit *ckt)
{
    return (coreitf->dllitf_MIFbindCSCComplexToReal)(inModel,ckt);
}
#endif
