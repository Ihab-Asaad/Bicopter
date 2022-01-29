#include "__cf_Control_Bicopter.h"
#include <math.h>
#include "Control_Bicopter_acc.h"
#include "Control_Bicopter_acc_private.h"
#include <stdio.h>
#include "slexec_vm_simstruct_bridge.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_lookup_functions.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simtarget/slSimTgtMdlrefSfcnBridge.h"
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#include "simtarget/slAccSfcnBridge.h"
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T *
bufSzPtr , int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T
tMinusDelay , real_T * * tBufPtr , real_T * * uBufPtr , real_T * * xBufPtr ,
boolean_T isfixedbuf , boolean_T istransportdelay , int_T * maxNewBufSzPtr )
{ int_T testIdx ; int_T tail = * tailPtr ; int_T bufSz = * bufSzPtr ; real_T
* tBuf = * tBufPtr ; real_T * xBuf = ( NULL ) ; int_T numBuffer = 2 ; if (
istransportdelay ) { numBuffer = 3 ; xBuf = * xBufPtr ; } testIdx = ( tail <
( bufSz - 1 ) ) ? ( tail + 1 ) : 0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] )
&& ! isfixedbuf ) { int_T j ; real_T * tempT ; real_T * tempU ; real_T *
tempX = ( NULL ) ; real_T * uBuf = * uBufPtr ; int_T newBufSz = bufSz + 1024
; if ( newBufSz > * maxNewBufSzPtr ) { * maxNewBufSzPtr = newBufSz ; } tempU
= ( real_T * ) utMalloc ( numBuffer * newBufSz * sizeof ( real_T ) ) ; if (
tempU == ( NULL ) ) { return ( false ) ; } tempT = tempU + newBufSz ; if (
istransportdelay ) tempX = tempT + newBufSz ; for ( j = tail ; j < bufSz ; j
++ ) { tempT [ j - tail ] = tBuf [ j ] ; tempU [ j - tail ] = uBuf [ j ] ; if
( istransportdelay ) tempX [ j - tail ] = xBuf [ j ] ; } for ( j = 0 ; j <
tail ; j ++ ) { tempT [ j + bufSz - tail ] = tBuf [ j ] ; tempU [ j + bufSz -
tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j + bufSz - tail ] =
xBuf [ j ] ; } if ( * lastPtr > tail ) { * lastPtr -= tail ; } else { *
lastPtr += ( bufSz - tail ) ; } * tailPtr = 0 ; * headPtr = bufSz ; utFree (
uBuf ) ; * bufSzPtr = newBufSz ; * tBufPtr = tempT ; * uBufPtr = tempU ; if (
istransportdelay ) * xBufPtr = tempX ; } else { * tailPtr = testIdx ; }
return ( true ) ; } real_T Control_Bicopter_acc_rt_TDelayInterpolate ( real_T
tMinusDelay , real_T tStart , real_T * tBuf , real_T * uBuf , int_T bufSz ,
int_T * lastIdx , int_T oldestIdx , int_T newIdx , real_T initOutput ,
boolean_T discrete , boolean_T minorStepAndTAtLastMajorOutput ) { int_T i ;
real_T yout , t1 , t2 , u1 , u2 ; if ( ( newIdx == 0 ) && ( oldestIdx == 0 )
&& ( tMinusDelay > tStart ) ) return initOutput ; if ( tMinusDelay <= tStart
) return initOutput ; if ( ( tMinusDelay <= tBuf [ oldestIdx ] ) ) { if (
discrete ) { return ( uBuf [ oldestIdx ] ) ; } else { int_T tempIdx =
oldestIdx + 1 ; if ( oldestIdx == bufSz - 1 ) tempIdx = 0 ; t1 = tBuf [
oldestIdx ] ; t2 = tBuf [ tempIdx ] ; u1 = uBuf [ oldestIdx ] ; u2 = uBuf [
tempIdx ] ; if ( t2 == t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else {
yout = u1 ; } } else { real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; yout = f1 * u1 + f2 * u2 ; } return yout ; } } if (
minorStepAndTAtLastMajorOutput ) { if ( newIdx != 0 ) { if ( * lastIdx ==
newIdx ) { ( * lastIdx ) -- ; } newIdx -- ; } else { if ( * lastIdx == newIdx
) { * lastIdx = bufSz - 1 ; } newIdx = bufSz - 1 ; } } i = * lastIdx ; if (
tBuf [ i ] < tMinusDelay ) { while ( tBuf [ i ] < tMinusDelay ) { if ( i ==
newIdx ) break ; i = ( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } } else { while
( tBuf [ i ] >= tMinusDelay ) { i = ( i > 0 ) ? i - 1 : ( bufSz - 1 ) ; } i =
( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } * lastIdx = i ; if ( discrete ) {
double tempEps = ( DBL_EPSILON ) * 128.0 ; double localEps = tempEps *
muDoubleScalarAbs ( tBuf [ i ] ) ; if ( tempEps > localEps ) { localEps =
tempEps ; } localEps = localEps / 2.0 ; if ( tMinusDelay >= ( tBuf [ i ] -
localEps ) ) { yout = uBuf [ i ] ; } else { if ( i == 0 ) { yout = uBuf [
bufSz - 1 ] ; } else { yout = uBuf [ i - 1 ] ; } } } else { if ( i == 0 ) {
t1 = tBuf [ bufSz - 1 ] ; u1 = uBuf [ bufSz - 1 ] ; } else { t1 = tBuf [ i -
1 ] ; u1 = uBuf [ i - 1 ] ; } t2 = tBuf [ i ] ; u2 = uBuf [ i ] ; if ( t2 ==
t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else { yout = u1 ; } } else {
real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; yout
= f1 * u1 + f2 * u2 ; } } return ( yout ) ; } void rt_ssGetBlockPath (
SimStruct * S , int_T sysIdx , int_T blkIdx , char_T * * path ) {
_ssGetBlockPath ( S , sysIdx , blkIdx , path ) ; } void rt_ssSet_slErrMsg (
SimStruct * S , void * diag ) { if ( ! _ssIsErrorStatusAslErrMsg ( S ) ) {
_ssSet_slErrMsg ( S , diag ) ; } else { _ssDiscardDiagnostic ( S , diag ) ; }
} void rt_ssReportDiagnosticAsWarning ( SimStruct * S , void * diag ) {
_ssReportDiagnosticAsWarning ( S , diag ) ; } static void mdlOutputs (
SimStruct * S , int_T tid ) { real_T rtb_B_4_75_0 ; real_T rtb_B_4_76_0 ;
real_T rtb_B_4_78_0 ; int8_T rtAction ; real_T B_4_12_0_idx_1 ; real_T
B_4_12_0_idx_0 ; real_T B_4_12_0_idx_2 ; real_T B_4_12_0_idx_3 ; int32_T
isHit ; B_Control_Bicopter_T * _rtB ; P_Control_Bicopter_T * _rtP ;
X_Control_Bicopter_T * _rtX ; DW_Control_Bicopter_T * _rtDW ; _rtDW = ( (
DW_Control_Bicopter_T * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
X_Control_Bicopter_T * ) ssGetContStates ( S ) ) ; _rtP = ( (
P_Control_Bicopter_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_Control_Bicopter_T * ) _ssGetModelBlockIO ( S ) ) ; { real_T * * uBuffer =
( real_T * * ) & _rtDW -> TransportDelay3_PWORK . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay3_PWORK . TUbufferPtrs
[ 1 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP
-> P_2 ; _rtB -> B_4_0_0 = Control_Bicopter_acc_rt_TDelayInterpolate (
tMinusDelay , 0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay3_IWORK .
CircularBufSize , & _rtDW -> TransportDelay3_IWORK . Last , _rtDW ->
TransportDelay3_IWORK . Tail , _rtDW -> TransportDelay3_IWORK . Head , _rtP
-> P_3 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } ssCallAccelRunBlock ( S
, 4 , 1 , SS_CALL_MDL_OUTPUTS ) ; { real_T * * uBuffer = ( real_T * * ) &
_rtDW -> TransportDelay1_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = (
real_T * * ) & _rtDW -> TransportDelay1_PWORK . TUbufferPtrs [ 1 ] ; real_T
simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_4 ; _rtB ->
B_4_2_0 = Control_Bicopter_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , *
tBuffer , * uBuffer , _rtDW -> TransportDelay1_IWORK . CircularBufSize , &
_rtDW -> TransportDelay1_IWORK . Last , _rtDW -> TransportDelay1_IWORK . Tail
, _rtDW -> TransportDelay1_IWORK . Head , _rtP -> P_5 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } ssCallAccelRunBlock ( S , 4 , 3 , SS_CALL_MDL_OUTPUTS ) ; { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> TransportDelay2_PWORK . TUbufferPtrs [ 0
] ; real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay2_PWORK .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay =
simTime - _rtP -> P_6 ; _rtB -> B_4_4_0 =
Control_Bicopter_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay2_IWORK . CircularBufSize , & _rtDW ->
TransportDelay2_IWORK . Last , _rtDW -> TransportDelay2_IWORK . Tail , _rtDW
-> TransportDelay2_IWORK . Head , _rtP -> P_7 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } ssCallAccelRunBlock ( S , 4 , 5 , SS_CALL_MDL_OUTPUTS ) ; { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> TransportDelay4_PWORK . TUbufferPtrs [ 0
] ; real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay4_PWORK .
TUbufferPtrs [ 3 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay ; {
int_T i1 ; real_T * y0 = & _rtB -> B_4_6_0 [ 0 ] ; int_T * iw_Tail = & _rtDW
-> TransportDelay4_IWORK . Tail [ 0 ] ; int_T * iw_Head = & _rtDW ->
TransportDelay4_IWORK . Head [ 0 ] ; int_T * iw_Last = & _rtDW ->
TransportDelay4_IWORK . Last [ 0 ] ; int_T * iw_CircularBufSize = & _rtDW ->
TransportDelay4_IWORK . CircularBufSize [ 0 ] ; for ( i1 = 0 ; i1 < 3 ; i1 ++
) { tMinusDelay = ( ( _rtP -> P_8 > 0.0 ) ? _rtP -> P_8 : 0.0 ) ; tMinusDelay
= simTime - tMinusDelay ; y0 [ i1 ] =
Control_Bicopter_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , iw_CircularBufSize [ i1 ] , & iw_Last [ i1 ] , iw_Tail [ i1 ] ,
iw_Head [ i1 ] , _rtP -> P_9 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) &&
( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; tBuffer ++ ; uBuffer ++
; } } } ssCallAccelRunBlock ( S , 4 , 7 , SS_CALL_MDL_OUTPUTS ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S , 4
, 8 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 4 , 9 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 4 , 10 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 4 , 11 ,
SS_CALL_MDL_OUTPUTS ) ; } B_4_12_0_idx_0 = _rtB -> B_4_1_0 - _rtB -> B_4_6_0
[ 2 ] ; B_4_12_0_idx_1 = _rtB -> B_4_4_0_k - _rtB -> B_4_4_0 ; B_4_12_0_idx_2
= _rtB -> B_4_3_0 - _rtB -> B_4_6_0 [ 1 ] ; B_4_12_0_idx_3 = _rtB ->
B_4_2_0_c - _rtB -> B_4_6_0 [ 0 ] ; _rtB -> B_4_18_0 = ( _rtP -> P_12 *
B_4_12_0_idx_0 - _rtX -> Filter_CSTATE ) * _rtP -> P_14 ; _rtB -> B_4_26_0 =
( _rtP -> P_18 * B_4_12_0_idx_1 - _rtX -> Filter_CSTATE_o ) * _rtP -> P_20 ;
_rtB -> B_4_34_0 = ( _rtP -> P_23 * B_4_12_0_idx_2 - _rtX -> Filter_CSTATE_p
) * _rtP -> P_25 ; _rtB -> B_4_41_0 = ( _rtP -> P_28 * B_4_12_0_idx_3 - _rtX
-> Filter_CSTATE_c ) * _rtP -> P_30 ; _rtB -> B_0_0_0 [ 0 ] = ( ( _rtP ->
P_10 * B_4_12_0_idx_0 + _rtX -> Integrator_CSTATE ) + _rtB -> B_4_18_0 ) *
_rtP -> P_15 ; _rtB -> B_0_0_0 [ 1 ] = ( ( _rtP -> P_16 * B_4_12_0_idx_1 +
_rtX -> Integrator_CSTATE_m ) + _rtB -> B_4_26_0 ) + _rtB -> B_4_0_0_m ; _rtB
-> B_0_0_0 [ 2 ] = ( _rtP -> P_21 * B_4_12_0_idx_2 + _rtX ->
Integrator_CSTATE_a ) + _rtB -> B_4_34_0 ; _rtB -> B_0_0_0 [ 3 ] = ( _rtP ->
P_26 * B_4_12_0_idx_3 + _rtX -> Integrator_CSTATE_m5 ) + _rtB -> B_4_41_0 ;
ssCallAccelRunBlock ( S , 0 , 1 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> B_4_44_0 =
_rtP -> P_31 * B_4_12_0_idx_0 ; _rtB -> B_4_45_0 = _rtP -> P_32 *
B_4_12_0_idx_3 ; _rtB -> B_4_46_0 = _rtP -> P_33 * B_4_12_0_idx_2 ; _rtB ->
B_4_47_0 = _rtP -> P_34 * B_4_12_0_idx_1 ; if ( _rtB -> B_0_1_1 [ 0 ] > _rtP
-> P_35 ) { B_4_12_0_idx_0 = _rtP -> P_35 ; } else if ( _rtB -> B_0_1_1 [ 0 ]
< _rtP -> P_36 ) { B_4_12_0_idx_0 = _rtP -> P_36 ; } else { B_4_12_0_idx_0 =
_rtB -> B_0_1_1 [ 0 ] ; } if ( _rtB -> B_0_1_1 [ 1 ] > _rtP -> P_37 ) {
B_4_12_0_idx_1 = _rtP -> P_37 ; } else if ( _rtB -> B_0_1_1 [ 1 ] < _rtP ->
P_38 ) { B_4_12_0_idx_1 = _rtP -> P_38 ; } else { B_4_12_0_idx_1 = _rtB ->
B_0_1_1 [ 1 ] ; } if ( _rtB -> B_0_1_1 [ 2 ] > _rtP -> P_39 ) {
B_4_12_0_idx_2 = _rtP -> P_39 ; } else if ( _rtB -> B_0_1_1 [ 2 ] < _rtP ->
P_40 ) { B_4_12_0_idx_2 = _rtP -> P_40 ; } else { B_4_12_0_idx_2 = _rtB ->
B_0_1_1 [ 2 ] ; } _rtB -> B_4_57_0 = _rtP -> P_43 * B_4_12_0_idx_2 + _rtB ->
B_4_7_0 ; if ( _rtDW -> INPUT_2_1_1_FirstOutput == 0.0 ) { _rtDW ->
INPUT_2_1_1_FirstOutput = 1.0 ; _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
0 ] = _rtB -> B_4_57_0 ; _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] = 0.0 ; } _rtB -> B_4_58_0 [ 0 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
0 ] ; _rtB -> B_4_58_0 [ 1 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] ; _rtB -> B_4_58_0 [ 2 ] = ( ( _rtB -> B_4_57_0 - _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
0 ] ) * 1000.0 - 2.0 * _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] ) * 1000.0 ; _rtB -> B_4_58_0 [ 3 ] = 0.0 ; if ( _rtB -> B_0_1_1 [ 3 ] >
_rtP -> P_41 ) { B_4_12_0_idx_2 = _rtP -> P_41 ; } else if ( _rtB -> B_0_1_1
[ 3 ] < _rtP -> P_42 ) { B_4_12_0_idx_2 = _rtP -> P_42 ; } else {
B_4_12_0_idx_2 = _rtB -> B_0_1_1 [ 3 ] ; } _rtB -> B_4_60_0 = _rtP -> P_44 *
B_4_12_0_idx_2 + _rtB -> B_4_8_0 ; if ( _rtDW -> INPUT_1_1_1_FirstOutput ==
0.0 ) { _rtDW -> INPUT_1_1_1_FirstOutput = 1.0 ; _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
0 ] = _rtB -> B_4_60_0 ; _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] = 0.0 ; } _rtB -> B_4_61_0 [ 0 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
0 ] ; _rtB -> B_4_61_0 [ 1 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] ; _rtB -> B_4_61_0 [ 2 ] = ( ( _rtB -> B_4_60_0 - _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
0 ] ) * 1000.0 - 2.0 * _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] ) * 1000.0 ; _rtB -> B_4_61_0 [ 3 ] = 0.0 ; _rtB -> B_4_62_0 = _rtX ->
Integrator_CSTATE_h ; if ( _rtDW -> INPUT_4_1_1_FirstOutput == 0.0 ) { _rtDW
-> INPUT_4_1_1_FirstOutput = 1.0 ; _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 0 ] = _rtB -> B_4_62_0 ; _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] = 0.0 ; } _rtB -> B_4_63_0 [ 0 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 0 ] ; _rtB -> B_4_63_0 [ 1 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] ; _rtB -> B_4_63_0 [ 2 ] = ( ( _rtB -> B_4_62_0 - _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 0 ] ) * 1000.0 - 2.0 * _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] ) * 1000.0 ; _rtB -> B_4_63_0 [ 3 ] = 0.0 ; _rtB -> B_4_64_0 = _rtX ->
Integrator_CSTATE_c ; if ( _rtDW -> INPUT_3_1_1_FirstOutput == 0.0 ) { _rtDW
-> INPUT_3_1_1_FirstOutput = 1.0 ; _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 0 ] = _rtB -> B_4_64_0 ; _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] = 0.0 ; } _rtB -> B_4_65_0 [ 0 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 0 ] ; _rtB -> B_4_65_0 [ 1 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] ; _rtB -> B_4_65_0 [ 2 ] = ( ( _rtB -> B_4_64_0 - _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 0 ] ) * 1000.0 - 2.0 * _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] ) * 1000.0 ; _rtB -> B_4_65_0 [ 3 ] = 0.0 ; ssCallAccelRunBlock ( S , 4
, 66 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 4 , 67 ,
SS_CALL_MDL_OUTPUTS ) ; B_4_12_0_idx_2 = muDoubleScalarSqrt ( ( ( _rtB ->
B_4_67_0 [ 5 ] * _rtB -> B_4_67_0 [ 5 ] + _rtB -> B_4_67_0 [ 6 ] * _rtB ->
B_4_67_0 [ 6 ] ) + _rtB -> B_4_67_0 [ 7 ] * _rtB -> B_4_67_0 [ 7 ] ) + _rtB
-> B_4_67_0 [ 8 ] * _rtB -> B_4_67_0 [ 8 ] ) ; B_4_12_0_idx_3 = _rtB ->
B_4_67_0 [ 5 ] / B_4_12_0_idx_2 ; rtb_B_4_75_0 = _rtB -> B_4_67_0 [ 6 ] /
B_4_12_0_idx_2 ; rtb_B_4_76_0 = _rtB -> B_4_67_0 [ 7 ] / B_4_12_0_idx_2 ;
B_4_12_0_idx_2 = _rtB -> B_4_67_0 [ 8 ] / B_4_12_0_idx_2 ; rtb_B_4_78_0 = (
rtb_B_4_75_0 * B_4_12_0_idx_2 - B_4_12_0_idx_3 * rtb_B_4_76_0 ) * - 2.0 ; if
( ssIsMajorTimeStep ( S ) != 0 ) { if ( rtb_B_4_78_0 > 1.0 ) { rtAction = 0 ;
} else if ( rtb_B_4_78_0 < - 1.0 ) { rtAction = 1 ; } else { rtAction = 2 ; }
_rtDW -> If_ActiveSubsystem = rtAction ; } else { rtAction = _rtDW ->
If_ActiveSubsystem ; } switch ( rtAction ) { case 0 : isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_4_83_0 = _rtP -> P_0 ; } if (
ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
IfActionSubsystem_SubsysRanBC ) ; } break ; case 1 : isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_4_83_0 = _rtP -> P_1 ; } if (
ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
IfActionSubsystem1_SubsysRanBC ) ; } break ; case 2 : _rtB -> B_4_83_0 =
rtb_B_4_78_0 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
IfActionSubsystem2_SubsysRanBC ) ; } break ; } _rtB -> B_4_91_0 [ 0 ] =
muDoubleScalarAtan2 ( ( rtb_B_4_75_0 * rtb_B_4_76_0 + B_4_12_0_idx_3 *
B_4_12_0_idx_2 ) * 2.0 , ( ( B_4_12_0_idx_3 * B_4_12_0_idx_3 + rtb_B_4_75_0 *
rtb_B_4_75_0 ) - rtb_B_4_76_0 * rtb_B_4_76_0 ) - B_4_12_0_idx_2 *
B_4_12_0_idx_2 ) ; _rtB -> B_4_91_0 [ 2 ] = muDoubleScalarAtan2 ( (
rtb_B_4_76_0 * B_4_12_0_idx_2 + B_4_12_0_idx_3 * rtb_B_4_75_0 ) * 2.0 , ( (
B_4_12_0_idx_3 * B_4_12_0_idx_3 - rtb_B_4_75_0 * rtb_B_4_75_0 ) -
rtb_B_4_76_0 * rtb_B_4_76_0 ) + B_4_12_0_idx_2 * B_4_12_0_idx_2 ) ; if ( _rtB
-> B_4_83_0 > 1.0 ) { B_4_12_0_idx_2 = 1.0 ; } else if ( _rtB -> B_4_83_0 < -
1.0 ) { B_4_12_0_idx_2 = - 1.0 ; } else { B_4_12_0_idx_2 = _rtB -> B_4_83_0 ;
} _rtB -> B_4_91_0 [ 1 ] = muDoubleScalarAsin ( B_4_12_0_idx_2 ) ; _rtB ->
B_4_99_0 = ( ( _rtP -> P_47 * B_4_12_0_idx_0 + _rtB -> B_4_5_0 ) * _rtP ->
P_48 - _rtB -> B_4_67_0 [ 1 ] ) * _rtP -> P_49 ; _rtB -> B_4_105_0 = ( _rtP
-> P_50 * B_4_12_0_idx_1 + _rtB -> B_4_6_0_c ) * _rtP -> P_51 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S , 4
, 106 , SS_CALL_MDL_OUTPUTS ) ; } _rtB -> B_4_109_0 = ( _rtB -> B_4_105_0 -
_rtB -> B_4_67_0 [ 0 ] ) * _rtP -> P_52 ; _rtB -> B_4_114_0 = _rtP -> P_53 *
B_4_12_0_idx_1 ; _rtB -> B_4_123_0 = _rtP -> P_54 * B_4_12_0_idx_0 ; _rtB ->
B_4_147_0 [ 0 ] = _rtB -> B_4_114_0 ; _rtB -> B_4_147_0 [ 1 ] = 0.0 ; _rtB ->
B_4_147_0 [ 2 ] = 0.0 ; _rtB -> B_4_147_0 [ 3 ] = 0.0 ; _rtB -> B_4_148_0 [ 0
] = _rtB -> B_4_123_0 ; _rtB -> B_4_148_0 [ 1 ] = 0.0 ; _rtB -> B_4_148_0 [ 2
] = 0.0 ; _rtB -> B_4_148_0 [ 3 ] = 0.0 ; UNUSED_PARAMETER ( tid ) ; } static
void mdlOutputsTID2 ( SimStruct * S , int_T tid ) { B_Control_Bicopter_T *
_rtB ; P_Control_Bicopter_T * _rtP ; _rtP = ( ( P_Control_Bicopter_T * )
ssGetModelRtp ( S ) ) ; _rtB = ( ( B_Control_Bicopter_T * )
_ssGetModelBlockIO ( S ) ) ; _rtB -> B_4_0_0_m = _rtP -> P_55 ; _rtB ->
B_4_1_0 = _rtP -> P_56 ; _rtB -> B_4_2_0_c = _rtP -> P_57 ; _rtB -> B_4_3_0 =
_rtP -> P_58 ; _rtB -> B_4_4_0_k = _rtP -> P_59 ; _rtB -> B_4_5_0 = _rtP ->
P_60 ; _rtB -> B_4_6_0_c = _rtP -> P_61 ; _rtB -> B_4_7_0 = _rtP -> P_62 ;
_rtB -> B_4_8_0 = _rtP -> P_63 ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { B_Control_Bicopter_T *
_rtB ; P_Control_Bicopter_T * _rtP ; DW_Control_Bicopter_T * _rtDW ; _rtDW =
( ( DW_Control_Bicopter_T * ) ssGetRootDWork ( S ) ) ; _rtP = ( (
P_Control_Bicopter_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_Control_Bicopter_T * ) _ssGetModelBlockIO ( S ) ) ; { real_T * * uBuffer =
( real_T * * ) & _rtDW -> TransportDelay3_PWORK . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay3_PWORK . TUbufferPtrs
[ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW -> TransportDelay3_IWORK . Head
= ( ( _rtDW -> TransportDelay3_IWORK . Head < ( _rtDW ->
TransportDelay3_IWORK . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay3_IWORK . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay3_IWORK . Head == _rtDW -> TransportDelay3_IWORK . Tail ) { if
( ! Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay3_IWORK . CircularBufSize , & _rtDW -> TransportDelay3_IWORK .
Tail , & _rtDW -> TransportDelay3_IWORK . Head , & _rtDW ->
TransportDelay3_IWORK . Last , simTime - _rtP -> P_2 , tBuffer , uBuffer , (
NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay3_IWORK .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay3_IWORK . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay3_IWORK . Head ] = _rtB ->
B_4_67_0 [ 2 ] ; } { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay1_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay1_PWORK . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; _rtDW -> TransportDelay1_IWORK . Head = ( ( _rtDW ->
TransportDelay1_IWORK . Head < ( _rtDW -> TransportDelay1_IWORK .
CircularBufSize - 1 ) ) ? ( _rtDW -> TransportDelay1_IWORK . Head + 1 ) : 0 )
; if ( _rtDW -> TransportDelay1_IWORK . Head == _rtDW ->
TransportDelay1_IWORK . Tail ) { if ( !
Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay1_IWORK . CircularBufSize , & _rtDW -> TransportDelay1_IWORK .
Tail , & _rtDW -> TransportDelay1_IWORK . Head , & _rtDW ->
TransportDelay1_IWORK . Last , simTime - _rtP -> P_4 , tBuffer , uBuffer , (
NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay1_IWORK .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay1_IWORK . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay1_IWORK . Head ] = _rtB ->
B_4_67_0 [ 3 ] ; } { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay2_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay2_PWORK . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; _rtDW -> TransportDelay2_IWORK . Head = ( ( _rtDW ->
TransportDelay2_IWORK . Head < ( _rtDW -> TransportDelay2_IWORK .
CircularBufSize - 1 ) ) ? ( _rtDW -> TransportDelay2_IWORK . Head + 1 ) : 0 )
; if ( _rtDW -> TransportDelay2_IWORK . Head == _rtDW ->
TransportDelay2_IWORK . Tail ) { if ( !
Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay2_IWORK . CircularBufSize , & _rtDW -> TransportDelay2_IWORK .
Tail , & _rtDW -> TransportDelay2_IWORK . Head , & _rtDW ->
TransportDelay2_IWORK . Last , simTime - _rtP -> P_6 , tBuffer , uBuffer , (
NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay2_IWORK .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay2_IWORK . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay2_IWORK . Head ] = _rtB ->
B_4_67_0 [ 4 ] ; } { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay4_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay4_PWORK . TUbufferPtrs [ 3 ] ; real_T simTime =
ssGetT ( S ) ; _rtDW -> TransportDelay4_IWORK . Head [ 0 ] = ( ( _rtDW ->
TransportDelay4_IWORK . Head [ 0 ] < ( _rtDW -> TransportDelay4_IWORK .
CircularBufSize [ 0 ] - 1 ) ) ? ( _rtDW -> TransportDelay4_IWORK . Head [ 0 ]
+ 1 ) : 0 ) ; if ( _rtDW -> TransportDelay4_IWORK . Head [ 0 ] == _rtDW ->
TransportDelay4_IWORK . Tail [ 0 ] ) { if ( !
Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay4_IWORK . CircularBufSize [ 0 ] , & _rtDW ->
TransportDelay4_IWORK . Tail [ 0 ] , & _rtDW -> TransportDelay4_IWORK . Head
[ 0 ] , & _rtDW -> TransportDelay4_IWORK . Last [ 0 ] , simTime - _rtP -> P_8
, tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay4_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ++ ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 0 ] ] = simTime ; ( * uBuffer ++ ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 0 ] ] = _rtB -> B_4_91_0 [ 0 ] ; _rtDW ->
TransportDelay4_IWORK . Head [ 1 ] = ( ( _rtDW -> TransportDelay4_IWORK .
Head [ 1 ] < ( _rtDW -> TransportDelay4_IWORK . CircularBufSize [ 1 ] - 1 ) )
? ( _rtDW -> TransportDelay4_IWORK . Head [ 1 ] + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay4_IWORK . Head [ 1 ] == _rtDW -> TransportDelay4_IWORK . Tail [
1 ] ) { if ( ! Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay4_IWORK . CircularBufSize [ 1 ] , & _rtDW ->
TransportDelay4_IWORK . Tail [ 1 ] , & _rtDW -> TransportDelay4_IWORK . Head
[ 1 ] , & _rtDW -> TransportDelay4_IWORK . Last [ 1 ] , simTime - _rtP -> P_8
, tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay4_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ++ ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 1 ] ] = simTime ; ( * uBuffer ++ ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 1 ] ] = _rtB -> B_4_91_0 [ 1 ] ; _rtDW ->
TransportDelay4_IWORK . Head [ 2 ] = ( ( _rtDW -> TransportDelay4_IWORK .
Head [ 2 ] < ( _rtDW -> TransportDelay4_IWORK . CircularBufSize [ 2 ] - 1 ) )
? ( _rtDW -> TransportDelay4_IWORK . Head [ 2 ] + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay4_IWORK . Head [ 2 ] == _rtDW -> TransportDelay4_IWORK . Tail [
2 ] ) { if ( ! Control_Bicopter_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay4_IWORK . CircularBufSize [ 2 ] , & _rtDW ->
TransportDelay4_IWORK . Tail [ 2 ] , & _rtDW -> TransportDelay4_IWORK . Head
[ 2 ] , & _rtDW -> TransportDelay4_IWORK . Last [ 2 ] , simTime - _rtP -> P_8
, tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay4_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 2 ] ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay4_IWORK . Head [ 2 ] ] = _rtB -> B_4_91_0 [ 2 ] ; }
ssCallAccelRunBlock ( S , 4 , 66 , SS_CALL_MDL_UPDATE ) ; UNUSED_PARAMETER (
tid ) ; }
#define MDL_UPDATE
static void mdlUpdateTID2 ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER (
tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { B_Control_Bicopter_T * _rtB ;
X_Control_Bicopter_T * _rtX ; XDot_Control_Bicopter_T * _rtXdot ; _rtXdot = (
( XDot_Control_Bicopter_T * ) ssGetdX ( S ) ) ; _rtX = ( (
X_Control_Bicopter_T * ) ssGetContStates ( S ) ) ; _rtB = ( (
B_Control_Bicopter_T * ) _ssGetModelBlockIO ( S ) ) ; _rtXdot ->
Integrator_CSTATE = _rtB -> B_4_44_0 ; _rtXdot -> Filter_CSTATE = _rtB ->
B_4_18_0 ; _rtXdot -> Integrator_CSTATE_m = _rtB -> B_4_47_0 ; _rtXdot ->
Filter_CSTATE_o = _rtB -> B_4_26_0 ; _rtXdot -> Integrator_CSTATE_a = _rtB ->
B_4_46_0 ; _rtXdot -> Filter_CSTATE_p = _rtB -> B_4_34_0 ; _rtXdot ->
Integrator_CSTATE_m5 = _rtB -> B_4_45_0 ; _rtXdot -> Filter_CSTATE_c = _rtB
-> B_4_41_0 ; _rtXdot ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
0 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] ; _rtXdot ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] = ( ( _rtB -> B_4_57_0 - _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
0 ] ) * 1000.0 - 2.0 * _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter4outputFiltered_4174348068_0 [
1 ] ) * 1000.0 ; _rtXdot ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
0 ] = _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] ; _rtXdot ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] = ( ( _rtB -> B_4_60_0 - _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
0 ] ) * 1000.0 - 2.0 * _rtX ->
Control_BicopterSubsystemSimulink_PS_Converter1outputFiltered_3211724276_0 [
1 ] ) * 1000.0 ; _rtXdot -> Integrator_CSTATE_h = _rtB -> B_4_109_0 ; _rtXdot
->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 0 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] ; _rtXdot ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] = ( ( _rtB -> B_4_62_0 - _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 0 ] ) * 1000.0 - 2.0 * _rtX ->
 Control_BicopterSubsystemtorque_for_propeller1Simulink_PS_Converter3outputFiltered_2005895812_0
[ 1 ] ) * 1000.0 ; _rtXdot -> Integrator_CSTATE_c = _rtB -> B_4_99_0 ;
_rtXdot ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 0 ] = _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] ; _rtXdot ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] = ( ( _rtB -> B_4_64_0 - _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 0 ] ) * 1000.0 - 2.0 * _rtX ->
 Control_BicopterSubsystemtorque_for_propellerSimulink_PS_Converter3outputFiltered_3316602516_0
[ 1 ] ) * 1000.0 ; ssCallAccelRunBlock ( S , 4 , 66 , SS_CALL_MDL_DERIVATIVES
) ; }
#define MDL_PROJECTION
static void mdlProjection ( SimStruct * S ) { ssCallAccelRunBlock ( S , 4 ,
66 , SS_CALL_MDL_PROJECTION ) ; } static void mdlInitializeSizes ( SimStruct
* S ) { ssSetChecksumVal ( S , 0 , 1075050148U ) ; ssSetChecksumVal ( S , 1 ,
3234823736U ) ; ssSetChecksumVal ( S , 2 , 3114399334U ) ; ssSetChecksumVal (
S , 3 , 223692211U ) ; { mxArray * slVerStructMat = NULL ; mxArray * slStrMat
= mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ; int status =
mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" ) ; if ( status
== 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 , "Version" ) ;
if ( slVerMat == NULL ) { status = 1 ; } else { status = mxGetString (
slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "10.0" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
DW_Control_Bicopter_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( B_Control_Bicopter_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofY ( S ) != sizeof
( ExtY_Control_Bicopter_T ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal ExternalOutputs sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
P_Control_Bicopter_T ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetModelRtp ( S , ( real_T *
) & Control_Bicopter_rtDefaultP ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ;
} static void mdlInitializeSampleTimes ( SimStruct * S ) { { SimStruct *
childS ; SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S , 0 ) ;
callSysFcns = ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ]
= ( SysOutputFcn ) ( NULL ) ; } slAccRegPrmChangeFcn ( S , mdlOutputsTID2 ) ;
} static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
