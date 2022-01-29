#include "__cf_Control_Bicopter.h"
#ifndef RTW_HEADER_Control_Bicopter_h_
#define RTW_HEADER_Control_Bicopter_h_
#include <stddef.h>
#include <float.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef Control_Bicopter_COMMON_INCLUDES_
#define Control_Bicopter_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#include "nesl_rtw.h"
#include "Control_Bicopter_ae14a523_1_gateway.h"
#endif
#include "Control_Bicopter_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME Control_Bicopter
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (6)     
#define NBLOCKIO (31) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (79)   
#elif NCSTATES != 79
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T alnu044j5k ; real_T g41m4pmy1w ; real_T kuc2djevrk ;
real_T pzxksc1hoi [ 3 ] ; real_T m1dkcmbbey ; real_T onyodjbxop ; real_T
ibl1bvoot5 ; real_T ozxva0egn1 ; real_T eslkbwnlvg ; real_T e04tk5zbbe ;
real_T lgntheub31 ; real_T ms5qo0q32e ; real_T agzdgrfhap ; real_T eczwzi2yjw
[ 4 ] ; real_T k4jt1nr2ej ; real_T gudikbezkv [ 4 ] ; real_T b2gk3wukry ;
real_T nxogfkpljw [ 4 ] ; real_T gipodijd1l ; real_T e4vcd3rbgn [ 4 ] ;
real_T igfxglvoam [ 61 ] ; real_T oqquwhnyjp [ 9 ] ; real_T p1zfkdy0ss ;
real_T omd4dozqfc [ 3 ] ; real_T kdhjnpyryt ; real_T aj3d0xadxg ; real_T
hl1tydtlp3 ; real_T je2xqylh2t ; real_T jcianyi4kp ; real_T o2kcntnybn [ 4 ]
; real_T hqhvl0otti [ 4 ] ; } B ; typedef struct { real_T ecgt4jrfbm ; real_T
fkv2s4cq4q ; real_T pd2cnxedgx ; real_T jm54tlayqp ; real_T ikaqdpe14g ;
real_T hquutdkqak ; real_T ccksc230fn ; real_T kbkgyu20nl ; real_T b2lbbxidhp
[ 2 ] ; real_T fvqjeh3ony [ 2 ] ; real_T ggqhsa10cs ; real_T kaia4u11f2 ;
struct { real_T modelTStart ; } k21rn4i5xu ; struct { real_T modelTStart ; }
mdms1xibhu ; struct { real_T modelTStart ; } bfxyvq4vri ; struct { real_T
modelTStart ; } aqigcjflxp ; struct { void * TUbufferPtrs [ 2 ] ; }
g2lkbxgjhp ; struct { void * TUbufferPtrs [ 2 ] ; } lizyfiedvc ; struct {
void * TUbufferPtrs [ 2 ] ; } mx22ld2hzj ; struct { void * TUbufferPtrs [ 6 ]
; } diva5n50pk ; struct { void * LoggedData ; } nomdnwo2sd ; struct { void *
LoggedData ; } ouuolo4dzo ; struct { void * LoggedData ; } nlope1xt2k ;
struct { void * LoggedData ; } nwoe35ohwt ; void * edvzy5545y ; void *
m2zz55qyqo ; void * nbicecjjvm ; void * pntjz4vsvn ; void * ghee1gdvk4 ; void
* avtvu4a0bu ; void * b1ivukxskp ; void * hzasxj2szj ; void * f0bb2ffysq ;
void * fm533wjgzd ; struct { void * LoggedData ; } ihz200qec4 ; struct {
int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } j03obu1eim ; struct { int_T Tail ; int_T Head ; int_T Last
; int_T CircularBufSize ; int_T MaxNewBufSize ; } cadeui54l4 ; struct { int_T
Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize
; } dbj1dpcqc0 ; struct { int_T Tail [ 3 ] ; int_T Head [ 3 ] ; int_T Last [
3 ] ; int_T CircularBufSize [ 3 ] ; int_T MaxNewBufSize ; } ko54z1uphk ;
int_T j24ttftmn0 ; int_T ckkmlyrmch ; int8_T g5ujhhy5gx ; int8_T eghku2cvih ;
int8_T gte2z2pt0y ; int8_T n4vwxzlvdz ; boolean_T ldjeeunpd1 ; boolean_T
iura2nrvwp ; } DW ; typedef struct { real_T idxnpi20k1 ; real_T f4xemzmgtv ;
real_T bj5404lz5d ; real_T az1z4d4mub ; real_T i44qjsxqq3 ; real_T fwjq3tjwfs
; real_T cucedlakmc ; real_T lq4alwhpwv ; real_T lryf2ntwdb [ 2 ] ; real_T
px5f4y1l5a [ 2 ] ; real_T mwtxdy2wl1 ; real_T fbwrmyrd20 [ 2 ] ; real_T
lu1xqtjac1 ; real_T bl2jw4bwps [ 2 ] ; real_T fmofelvad4 [ 61 ] ; } X ;
typedef struct { real_T idxnpi20k1 ; real_T f4xemzmgtv ; real_T bj5404lz5d ;
real_T az1z4d4mub ; real_T i44qjsxqq3 ; real_T fwjq3tjwfs ; real_T cucedlakmc
; real_T lq4alwhpwv ; real_T lryf2ntwdb [ 2 ] ; real_T px5f4y1l5a [ 2 ] ;
real_T mwtxdy2wl1 ; real_T fbwrmyrd20 [ 2 ] ; real_T lu1xqtjac1 ; real_T
bl2jw4bwps [ 2 ] ; real_T fmofelvad4 [ 61 ] ; } XDot ; typedef struct {
boolean_T idxnpi20k1 ; boolean_T f4xemzmgtv ; boolean_T bj5404lz5d ;
boolean_T az1z4d4mub ; boolean_T i44qjsxqq3 ; boolean_T fwjq3tjwfs ;
boolean_T cucedlakmc ; boolean_T lq4alwhpwv ; boolean_T lryf2ntwdb [ 2 ] ;
boolean_T px5f4y1l5a [ 2 ] ; boolean_T mwtxdy2wl1 ; boolean_T fbwrmyrd20 [ 2
] ; boolean_T lu1xqtjac1 ; boolean_T bl2jw4bwps [ 2 ] ; boolean_T fmofelvad4
[ 61 ] ; } XDis ; typedef struct { real_T mkujrenrvw ; real_T ki01i2zemv ;
real_T cuajxr4igq ; real_T i5hbuq5xsv [ 3 ] ; } ExtY ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T D_Phi ;
real_T D_Psi ; real_T D_Theta ; real_T D_Z ; real_T I_Phi ; real_T I_Psi ;
real_T I_Theta ; real_T I_Z ; real_T P_Phi ; real_T P_Psi ; real_T P_Theta ;
real_T P_Z ; real_T Z_step ; real_T copter_mass ; real_T g ; real_T
time_delay ; real_T PIDPhi_InitialConditionForFilter ; real_T
PIDZ_InitialConditionForFilter ; real_T PIDTheta_InitialConditionForFilter ;
real_T PIDPsi_InitialConditionForFilter ; real_T
PIDPhi_InitialConditionForIntegrator ; real_T
PIDZ_InitialConditionForIntegrator ; real_T
PIDTheta_InitialConditionForIntegrator ; real_T
PIDPsi_InitialConditionForIntegrator ; real_T PIDPhi_N ; real_T PIDZ_N ;
real_T PIDTheta_N ; real_T PIDPsi_N ; real_T Constant_Value ; real_T
Constant_Value_opjr4pzkxp ; real_T TransportDelay3_InitOutput ; real_T
TransportDelay1_InitOutput ; real_T TransportDelay2_InitOutput ; real_T
TransportDelay4_InitOutput ; real_T Gain_Gain ; real_T Saturation_F1_UpperSat
; real_T Saturation_F1_LowerSat ; real_T Saturation_F2_UpperSat ; real_T
Saturation_F2_LowerSat ; real_T Saturation_F3_UpperSat ; real_T
Saturation_F3_LowerSat ; real_T Saturation_F4_UpperSat ; real_T
Saturation_F4_LowerSat ; real_T Gain1_Gain ; real_T Gain2_Gain ; real_T
Integrator_IC ; real_T Integrator_IC_gptrc5suwu ; real_T speedgain_Gain ;
real_T const_Gain ; real_T Gain_Gain_nt13gxv4cf ; real_T
speedgain_Gain_cayz03alro ; real_T const_Gain_nyshjvfom1 ; real_T
Gain_Gain_nz2osdhexb ; real_T Gain_Gain_m3wdc4uyp3 ; real_T
Gain_Gain_crytncxhkk ; real_T Phi_ref_Value ; real_T Psi_ref_Value ; real_T
Theta_ref_Value ; real_T Constant2_Value ; real_T Constant2_Value_bijw4dwdaf
; real_T Constant_Value_pr21eedhgl ; real_T Constant1_Value ; } ; extern
const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ;
extern DW rtDW ; extern ExtY rtY ; extern P rtP ; extern const
rtwCAPI_ModelMappingStaticInfo * Control_Bicopter_GetCAPIStaticMap ( void ) ;
extern SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern
const int_T gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
extern const int_T gblNumRootInportBlks ; extern const int_T
gblNumModelInputs ; extern const int_T gblInportDataTypeIdx [ ] ; extern
const int_T gblInportDims [ ] ; extern const int_T gblInportComplex [ ] ;
extern const int_T gblInportInterpoFlag [ ] ; extern const int_T
gblInportContinuous [ ] ; extern const int_T gblParameterTuningTid ; extern
DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void
MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ;
void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( void
) ;
#endif
