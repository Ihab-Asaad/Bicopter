#include "__cf_Control_Bicopter.h"
#ifndef RTW_HEADER_Control_Bicopter_acc_h_
#define RTW_HEADER_Control_Bicopter_acc_h_
#include <stddef.h>
#include <float.h>
#ifndef Control_Bicopter_acc_COMMON_INCLUDES_
#define Control_Bicopter_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#endif
#include "Control_Bicopter_acc_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
typedef struct { real_T B_4_0_0 ; real_T B_4_2_0 ; real_T B_4_4_0 ; real_T
B_4_6_0 [ 3 ] ; real_T B_4_18_0 ; real_T B_4_26_0 ; real_T B_4_34_0 ; real_T
B_4_41_0 ; real_T B_4_44_0 ; real_T B_4_45_0 ; real_T B_4_46_0 ; real_T
B_4_47_0 ; real_T B_4_57_0 ; real_T B_4_58_0 [ 4 ] ; real_T B_4_60_0 ; real_T
B_4_61_0 [ 4 ] ; real_T B_4_62_0 ; real_T B_4_63_0 [ 4 ] ; real_T B_4_64_0 ;
real_T B_4_65_0 [ 4 ] ; real_T B_4_66_0 [ 61 ] ; real_T B_4_67_0 [ 9 ] ;
real_T B_4_83_0 ; real_T B_4_91_0 [ 3 ] ; real_T B_4_99_0 ; real_T B_4_105_0
; real_T B_4_109_0 ; real_T B_4_114_0 ; real_T B_4_123_0 ; real_T B_4_147_0 [
4 ] ; real_T B_4_148_0 [ 4 ] ; real_T B_4_0_0_m ; real_T B_4_1_0 ; real_T
B_4_2_0_c ; real_T B_4_3_0 ; real_T B_4_4_0_k ; real_T B_4_5_0 ; real_T
B_4_6_0_c ; real_T B_4_7_0 ; real_T B_4_8_0 ; real_T B_0_0_0 [ 4 ] ; real_T
B_0_1_1 [ 4 ] ; } B_Control_Bicopter_T ; typedef struct { real_T
INPUT_2_1_1_Discrete ; real_T INPUT_2_1_1_FirstOutput ; real_T
INPUT_1_1_1_Discrete ; real_T INPUT_1_1_1_FirstOutput ; real_T
INPUT_4_1_1_Discrete ; real_T INPUT_4_1_1_FirstOutput ; real_T
INPUT_3_1_1_Discrete ; real_T INPUT_3_1_1_FirstOutput ; real_T
INPUT_5_1_1_Discrete [ 2 ] ; real_T INPUT_6_1_1_Discrete [ 2 ] ; real_T
STATE_1_Discrete ; real_T OUTPUT_1_0_Discrete ; struct { real_T modelTStart ;
} TransportDelay3_RWORK ; struct { real_T modelTStart ; }
TransportDelay1_RWORK ; struct { real_T modelTStart ; } TransportDelay2_RWORK
; struct { real_T modelTStart ; } TransportDelay4_RWORK ; struct { void *
TUbufferPtrs [ 2 ] ; } TransportDelay3_PWORK ; struct { void * TUbufferPtrs [
2 ] ; } TransportDelay1_PWORK ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay2_PWORK ; struct { void * TUbufferPtrs [ 6 ] ; }
TransportDelay4_PWORK ; void * Scope_PWORK ; void * Scope1_PWORK ; void *
Scope2_PWORK ; void * Scope3_PWORK ; void * STATE_1_Simulator ; void *
STATE_1_SimData ; void * STATE_1_DiagMgr ; void * STATE_1_ZcLogger ; void *
STATE_1_TsIndex ; void * OUTPUT_1_0_Simulator ; void * OUTPUT_1_0_SimData ;
void * OUTPUT_1_0_DiagMgr ; void * OUTPUT_1_0_ZcLogger ; void *
OUTPUT_1_0_TsIndex ; void * Scope_PWORK_k ; int32_T
IfActionSubsystem2_sysIdxToRun ; int32_T IfActionSubsystem1_sysIdxToRun ;
int32_T IfActionSubsystem_sysIdxToRun ; int32_T MATLABFunction_sysIdxToRun ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay3_IWORK ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay1_IWORK ; struct { int_T Tail ; int_T Head ; int_T Last ; int_T
CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay2_IWORK ; struct {
int_T Tail [ 3 ] ; int_T Head [ 3 ] ; int_T Last [ 3 ] ; int_T
CircularBufSize [ 3 ] ; int_T MaxNewBufSize ; } TransportDelay4_IWORK ; int_T
STATE_1_Modes ; int_T OUTPUT_1_0_Modes ; int8_T If_ActiveSubsystem ; int8_T
IfActionSubsystem2_SubsysRanBC ; int8_T IfActionSubsystem1_SubsysRanBC ;
int8_T IfActionSubsystem_SubsysRanBC ; boolean_T STATE_1_FirstOutput ;
boolean_T OUTPUT_1_0_FirstOutput ; char_T pad_OUTPUT_1_0_FirstOutput [ 2 ] ;
} DW_Control_Bicopter_T ; typedef struct { real_T Integrator_CSTATE ; real_T
Filter_CSTATE ; real_T Integrator_CSTATE_m ; real_T Filter_CSTATE_o ; real_T
Integrator_CSTATE_a ; real_T Filter_CSTATE_p ; real_T Integrator_CSTATE_m5 ;
real_T Filter_CSTATE_c ; real_T
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
2 ] ; real_T
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
2 ] ; real_T Integrator_CSTATE_h ; real_T
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 2 ] ; real_T Integrator_CSTATE_c ; real_T
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 2 ] ; real_T Control_BicopterSubsystemSixDOFPxp [ 61 ] ; }
X_Control_Bicopter_T ; typedef struct { real_T Integrator_CSTATE ; real_T
Filter_CSTATE ; real_T Integrator_CSTATE_m ; real_T Filter_CSTATE_o ; real_T
Integrator_CSTATE_a ; real_T Filter_CSTATE_p ; real_T Integrator_CSTATE_m5 ;
real_T Filter_CSTATE_c ; real_T
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
2 ] ; real_T
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
2 ] ; real_T Integrator_CSTATE_h ; real_T
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 2 ] ; real_T Integrator_CSTATE_c ; real_T
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 2 ] ; real_T Control_BicopterSubsystemSixDOFPxp [ 61 ] ; }
XDot_Control_Bicopter_T ; typedef struct { boolean_T Integrator_CSTATE ;
boolean_T Filter_CSTATE ; boolean_T Integrator_CSTATE_m ; boolean_T
Filter_CSTATE_o ; boolean_T Integrator_CSTATE_a ; boolean_T Filter_CSTATE_p ;
boolean_T Integrator_CSTATE_m5 ; boolean_T Filter_CSTATE_c ; boolean_T
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
2 ] ; boolean_T
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
2 ] ; boolean_T Integrator_CSTATE_h ; boolean_T
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 2 ] ; boolean_T Integrator_CSTATE_c ; boolean_T
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 2 ] ; boolean_T Control_BicopterSubsystemSixDOFPxp [ 61 ] ; }
XDis_Control_Bicopter_T ; typedef struct { real_T * B_4_1 ; real_T * B_4_3 ;
real_T * B_4_5 ; real_T * B_4_7 [ 3 ] ; } ExtY_Control_Bicopter_T ; struct
P_Control_Bicopter_T_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ;
real_T P_4 ; real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T P_9 ;
real_T P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T
P_15 ; real_T P_16 ; real_T P_17 ; real_T P_18 ; real_T P_19 ; real_T P_20 ;
real_T P_21 ; real_T P_22 ; real_T P_23 ; real_T P_24 ; real_T P_25 ; real_T
P_26 ; real_T P_27 ; real_T P_28 ; real_T P_29 ; real_T P_30 ; real_T P_31 ;
real_T P_32 ; real_T P_33 ; real_T P_34 ; real_T P_35 ; real_T P_36 ; real_T
P_37 ; real_T P_38 ; real_T P_39 ; real_T P_40 ; real_T P_41 ; real_T P_42 ;
real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T P_46 ; real_T P_47 ; real_T
P_48 ; real_T P_49 ; real_T P_50 ; real_T P_51 ; real_T P_52 ; real_T P_53 ;
real_T P_54 ; real_T P_55 ; real_T P_56 ; real_T P_57 ; real_T P_58 ; real_T
P_59 ; real_T P_60 ; real_T P_61 ; real_T P_62 ; real_T P_63 ; } ; extern
P_Control_Bicopter_T Control_Bicopter_rtDefaultP ;
#endif
