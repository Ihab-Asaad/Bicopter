#include "__cf_Control_Bicopter.h"
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Control_Bicopter_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "Control_Bicopter.h"
#include "Control_Bicopter_capi.h"
#include "Control_Bicopter_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay1" ) , TARGET_STRING ( "" ) , 0 , 0 ,
0 , 0 , 0 } , { 1 , 0 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay2" ) , TARGET_STRING ( "" ) , 0 , 0 ,
0 , 0 , 0 } , { 2 , 0 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay3" ) , TARGET_STRING ( "" ) , 0 , 0 ,
0 , 0 , 0 } , { 3 , 0 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay4" ) , TARGET_STRING ( "" ) , 0 , 0 ,
1 , 0 , 0 } , { 4 , 0 , TARGET_STRING ( "Control_Bicopter/Subsystem/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Sum2" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 ,
0 } , { 6 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/Gain" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 0 } , { 7 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/Integrator" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 8 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/Gain" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/const" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 10 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/Integrator" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 11 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/x5x4x3_propeller1_RIGID/Gain" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 12 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/x5x4x3_propeller2_RIGID/Gain" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 13 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Vector Concatenate"
) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 14 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Trigonometric Function1"
) , TARGET_STRING ( "r1" ) , 0 , 0 , 0 , 0 , 0 } , { 15 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Trigonometric Function3"
) , TARGET_STRING ( "r3" ) , 0 , 0 , 0 , 0 , 0 } , { 16 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/trigFcn"
) , TARGET_STRING ( "r2" ) , 0 , 0 , 0 , 0 , 0 } , { 17 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_1_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 18 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_2_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 19 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_3_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 20 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_4_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 21 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_5_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 22 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/INPUT_6_1_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 23 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/OUTPUT_1_0" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 24 , 0 , TARGET_STRING (
"Control_Bicopter/Subsystem/Solver Configuration/EVAL_KEY/STATE_1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 } , { 25 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Phi/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 26 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Phi/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 27 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Psi/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 28 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Psi/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 29 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Theta/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 30 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Theta/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 31 , 0 , TARGET_STRING (
"Control_Bicopter/PID_Controller/PID Z/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 32 , 0 , TARGET_STRING (
 "Control_Bicopter/PID_Controller/PID Z/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 33 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 34 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem1"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 35 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem2"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 36 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/Merge"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 37 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem/Constant"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 38 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem1/Constant"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 39 , 0 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem2/In"
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL
) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters
rtBlockParameters [ ] = { { 40 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay1" ) , TARGET_STRING ( "InitialOutput"
) , 0 , 0 , 0 } , { 41 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay2" ) , TARGET_STRING ( "InitialOutput"
) , 0 , 0 , 0 } , { 42 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay3" ) , TARGET_STRING ( "InitialOutput"
) , 0 , 0 , 0 } , { 43 , TARGET_STRING (
"Control_Bicopter/Delay/Transport Delay4" ) , TARGET_STRING ( "InitialOutput"
) , 0 , 0 , 0 } , { 44 , TARGET_STRING (
"Control_Bicopter/PID_Controller/PID Phi" ) , TARGET_STRING ( "N" ) , 0 , 0 ,
0 } , { 45 , TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Phi" ) ,
TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 46 ,
TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Phi" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 0 , 0 } , { 47 , TARGET_STRING (
"Control_Bicopter/PID_Controller/PID Psi" ) , TARGET_STRING ( "N" ) , 0 , 0 ,
0 } , { 48 , TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Psi" ) ,
TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 49 ,
TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Psi" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 0 , 0 } , { 50 , TARGET_STRING (
"Control_Bicopter/PID_Controller/PID Theta" ) , TARGET_STRING ( "N" ) , 0 , 0
, 0 } , { 51 , TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Theta" )
, TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 52 ,
TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Theta" ) , TARGET_STRING
( "InitialConditionForFilter" ) , 0 , 0 , 0 } , { 53 , TARGET_STRING (
"Control_Bicopter/PID_Controller/PID Z" ) , TARGET_STRING ( "N" ) , 0 , 0 , 0
} , { 54 , TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Z" ) ,
TARGET_STRING ( "InitialConditionForIntegrator" ) , 0 , 0 , 0 } , { 55 ,
TARGET_STRING ( "Control_Bicopter/PID_Controller/PID Z" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 0 , 0 } , { 56 , TARGET_STRING (
"Control_Bicopter/PID_Controller/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 ,
0 } , { 57 , TARGET_STRING ( "Control_Bicopter/Reference Signals/Phi_ref" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 58 , TARGET_STRING (
"Control_Bicopter/Reference Signals/Psi_ref" ) , TARGET_STRING ( "Value" ) ,
0 , 0 , 0 } , { 59 , TARGET_STRING (
"Control_Bicopter/Reference Signals/Theta_ref" ) , TARGET_STRING ( "Value" )
, 0 , 0 , 0 } , { 60 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F1" ) , TARGET_STRING (
"UpperLimit" ) , 0 , 0 , 0 } , { 61 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F1" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 62 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F2" ) , TARGET_STRING (
"UpperLimit" ) , 0 , 0 , 0 } , { 63 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F2" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 64 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F3" ) , TARGET_STRING (
"UpperLimit" ) , 0 , 0 , 0 } , { 65 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F3" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 66 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F4" ) , TARGET_STRING (
"UpperLimit" ) , 0 , 0 , 0 } , { 67 , TARGET_STRING (
"Control_Bicopter/Saturation Input Signals/Saturation_F4" ) , TARGET_STRING (
"LowerLimit" ) , 0 , 0 , 0 } , { 68 , TARGET_STRING (
"Control_Bicopter/Subsystem/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 ,
0 } , { 69 , TARGET_STRING ( "Control_Bicopter/Subsystem/Constant1" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 70 , TARGET_STRING (
"Control_Bicopter/Subsystem/Gain1" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 }
, { 71 , TARGET_STRING ( "Control_Bicopter/Subsystem/Gain2" ) , TARGET_STRING
( "Gain" ) , 0 , 0 , 0 } , { 72 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/Constant2" ) , TARGET_STRING
( "Value" ) , 0 , 0 , 0 } , { 73 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 74 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/const" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 75 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/speed gain" ) ,
TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 76 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller/Integrator" ) ,
TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 77 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/Constant2" ) ,
TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 78 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 79 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/const" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 80 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/speed gain" ) ,
TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 81 , TARGET_STRING (
"Control_Bicopter/Subsystem/torque for propeller1/Integrator" ) ,
TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 82 , TARGET_STRING (
"Control_Bicopter/Subsystem/x5x4x3_propeller1_RIGID/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 83 , TARGET_STRING (
"Control_Bicopter/Subsystem/x5x4x3_propeller2_RIGID/Gain" ) , TARGET_STRING (
"Gain" ) , 0 , 0 , 0 } , { 84 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 85 , TARGET_STRING (
 "Control_Bicopter/Subsystem/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem1/Constant"
) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 ,
0 , 0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { {
86 , TARGET_STRING ( "D_Phi" ) , 0 , 0 , 0 } , { 87 , TARGET_STRING ( "D_Psi"
) , 0 , 0 , 0 } , { 88 , TARGET_STRING ( "D_Theta" ) , 0 , 0 , 0 } , { 89 ,
TARGET_STRING ( "D_Z" ) , 0 , 0 , 0 } , { 90 , TARGET_STRING ( "I_Phi" ) , 0
, 0 , 0 } , { 91 , TARGET_STRING ( "I_Psi" ) , 0 , 0 , 0 } , { 92 ,
TARGET_STRING ( "I_Theta" ) , 0 , 0 , 0 } , { 93 , TARGET_STRING ( "I_Z" ) ,
0 , 0 , 0 } , { 94 , TARGET_STRING ( "P_Phi" ) , 0 , 0 , 0 } , { 95 ,
TARGET_STRING ( "P_Psi" ) , 0 , 0 , 0 } , { 96 , TARGET_STRING ( "P_Theta" )
, 0 , 0 , 0 } , { 97 , TARGET_STRING ( "P_Z" ) , 0 , 0 , 0 } , { 98 ,
TARGET_STRING ( "Z_step" ) , 0 , 0 , 0 } , { 99 , TARGET_STRING (
"copter_mass" ) , 0 , 0 , 0 } , { 100 , TARGET_STRING ( "g" ) , 0 , 0 , 0 } ,
{ 101 , TARGET_STRING ( "time_delay" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , 0 , 0
, 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . g41m4pmy1w , & rtB . kuc2djevrk ,
& rtB . alnu044j5k , & rtB . pzxksc1hoi [ 0 ] , & rtB . agzdgrfhap , & rtB .
k4jt1nr2ej , & rtB . kdhjnpyryt , & rtB . gipodijd1l , & rtB . hl1tydtlp3 , &
rtB . aj3d0xadxg , & rtB . b2gk3wukry , & rtB . je2xqylh2t , & rtB .
jcianyi4kp , & rtB . omd4dozqfc [ 0 ] , & rtB . omd4dozqfc [ 0 ] , ( & rtB .
omd4dozqfc [ 0 ] + 2 ) , ( & rtB . omd4dozqfc [ 0 ] + 1 ) , & rtB .
gudikbezkv [ 0 ] , & rtB . eczwzi2yjw [ 0 ] , & rtB . e4vcd3rbgn [ 0 ] , &
rtB . nxogfkpljw [ 0 ] , & rtB . o2kcntnybn [ 0 ] , & rtB . hqhvl0otti [ 0 ]
, & rtB . oqquwhnyjp [ 0 ] , & rtB . igfxglvoam [ 0 ] , & rtB . eslkbwnlvg ,
& rtB . m1dkcmbbey , & rtB . e04tk5zbbe , & rtB . ozxva0egn1 , & rtB .
lgntheub31 , & rtB . ibl1bvoot5 , & rtB . ms5qo0q32e , & rtB . onyodjbxop , &
rtB . p1zfkdy0ss , & rtB . p1zfkdy0ss , & rtB . p1zfkdy0ss , & rtB .
p1zfkdy0ss , & rtB . p1zfkdy0ss , & rtB . p1zfkdy0ss , & rtB . p1zfkdy0ss , &
rtP . TransportDelay1_InitOutput , & rtP . TransportDelay2_InitOutput , & rtP
. TransportDelay3_InitOutput , & rtP . TransportDelay4_InitOutput , & rtP .
PIDPhi_N , & rtP . PIDPhi_InitialConditionForIntegrator , & rtP .
PIDPhi_InitialConditionForFilter , & rtP . PIDPsi_N , & rtP .
PIDPsi_InitialConditionForIntegrator , & rtP .
PIDPsi_InitialConditionForFilter , & rtP . PIDTheta_N , & rtP .
PIDTheta_InitialConditionForIntegrator , & rtP .
PIDTheta_InitialConditionForFilter , & rtP . PIDZ_N , & rtP .
PIDZ_InitialConditionForIntegrator , & rtP . PIDZ_InitialConditionForFilter ,
& rtP . Gain_Gain , & rtP . Phi_ref_Value , & rtP . Psi_ref_Value , & rtP .
Theta_ref_Value , & rtP . Saturation_F1_UpperSat , & rtP .
Saturation_F1_LowerSat , & rtP . Saturation_F2_UpperSat , & rtP .
Saturation_F2_LowerSat , & rtP . Saturation_F3_UpperSat , & rtP .
Saturation_F3_LowerSat , & rtP . Saturation_F4_UpperSat , & rtP .
Saturation_F4_LowerSat , & rtP . Constant_Value_pr21eedhgl , & rtP .
Constant1_Value , & rtP . Gain1_Gain , & rtP . Gain2_Gain , & rtP .
Constant2_Value , & rtP . Gain_Gain_nt13gxv4cf , & rtP . const_Gain , & rtP .
speedgain_Gain , & rtP . Integrator_IC_gptrc5suwu , & rtP .
Constant2_Value_bijw4dwdaf , & rtP . Gain_Gain_nz2osdhexb , & rtP .
const_Gain_nyshjvfom1 , & rtP . speedgain_Gain_cayz03alro , & rtP .
Integrator_IC , & rtP . Gain_Gain_m3wdc4uyp3 , & rtP . Gain_Gain_crytncxhkk ,
& rtP . Constant_Value , & rtP . Constant_Value_opjr4pzkxp , & rtP . D_Phi ,
& rtP . D_Psi , & rtP . D_Theta , & rtP . D_Z , & rtP . I_Phi , & rtP . I_Psi
, & rtP . I_Theta , & rtP . I_Z , & rtP . P_Phi , & rtP . P_Psi , & rtP .
P_Theta , & rtP . P_Z , & rtP . Z_step , & rtP . copter_mass , & rtP . g , &
rtP . time_delay , } ; static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } } ; static const uint_T rtDimensionArray [ ] = {
1 , 1 , 3 , 1 , 4 , 1 , 9 , 1 , 61 , 1 } ; static const real_T
rtcapiStoredFloats [ ] = { 0.0 } ; static const rtwCAPI_FixPtMap rtFixPtMap [
] = { { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static
const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , 0 ,
0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals
, 40 , ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters , 46 ,
rtModelParameters , 16 } , { ( NULL ) , 0 } , { rtDataTypeMap ,
rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap ,
rtDimensionArray } , "float" , { 2369118705U , 162466512U , 3416941678U ,
3742909486U } , ( NULL ) , 0 , 0 } ; const rtwCAPI_ModelMappingStaticInfo *
Control_Bicopter_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void Control_Bicopter_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( (
* rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Control_Bicopter_host_InitializeDataMapInfo (
Control_Bicopter_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
