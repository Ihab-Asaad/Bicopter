#include "__cf_Control_Bicopter.h"
#ifndef RTW_HEADER_Control_Bicopter_cap_host_h_
#define RTW_HEADER_Control_Bicopter_cap_host_h_
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; }
Control_Bicopter_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void Control_Bicopter_host_InitializeDataMapInfo (
Control_Bicopter_host_DataMapInfo_T * dataMap , const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
