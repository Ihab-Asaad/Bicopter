#include "Control_Bicopter_capi_host.h"
static Control_Bicopter_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        Control_Bicopter_host_InitializeDataMapInfo(&(root), "Control_Bicopter");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
