#include "__cf_Control_Bicopter.h"
#include "rt_logging_mmi.h"
#include "Control_Bicopter_capi.h"
#include <math.h>
#include "Control_Bicopter.h"
#include "Control_Bicopter_private.h"
#include "Control_Bicopter_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 0 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 1 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "9.2 (R2019b) 18-Jul-2019" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj\\raccel\\Control_Bicopter\\Control_Bicopter_Jpattern.mat" ; const
int_T gblNumRootInportBlks = 0 ; const int_T gblNumModelInputs = 0 ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
extern void * gblAperiodicPartitionHitTimes ; const int_T
gblInportDataTypeIdx [ ] = { - 1 } ; const int_T gblInportDims [ ] = { - 1 }
; const int_T gblInportComplex [ ] = { - 1 } ; const int_T
gblInportInterpoFlag [ ] = { - 1 } ; const int_T gblInportContinuous [ ] = {
- 1 } ; int_T enableFcnCallFlag [ ] = { 1 , 1 , 1 } ; const char *
raccelLoadInputsAndAperiodicHitTimes ( const char * inportFileName , int *
matFileFormat ) { return rt_RapidReadInportsMatFile ( inportFileName ,
matFileFormat , 1 ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
B rtB ; X rtX ; DW rtDW ; ExtY rtY ; static SimStruct model_S ; SimStruct *
const rtS = & model_S ;
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ;
#endif
void * rt_TDelayCreateBuf ( int_T numBuffer , int_T bufSz , int_T elemSz ) {
return ( ( void * ) utMalloc ( numBuffer * bufSz * elemSz ) ) ; }
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T rt_TDelayUpdateTailOrGrowBuf ( int_T * bufSzPtr , int_T * tailPtr ,
int_T * headPtr , int_T * lastPtr , real_T tMinusDelay , real_T * * tBufPtr ,
real_T * * uBufPtr , real_T * * xBufPtr , boolean_T isfixedbuf , boolean_T
istransportdelay , int_T * maxNewBufSzPtr ) { int_T testIdx ; int_T tail = *
tailPtr ; int_T bufSz = * bufSzPtr ; real_T * tBuf = * tBufPtr ; real_T *
xBuf = ( NULL ) ; int_T numBuffer = 2 ; if ( istransportdelay ) { numBuffer =
3 ; xBuf = * xBufPtr ; } testIdx = ( tail < ( bufSz - 1 ) ) ? ( tail + 1 ) :
0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] ) && ! isfixedbuf ) { int_T j ;
real_T * tempT ; real_T * tempU ; real_T * tempX = ( NULL ) ; real_T * uBuf =
* uBufPtr ; int_T newBufSz = bufSz + 1024 ; if ( newBufSz > * maxNewBufSzPtr
) { * maxNewBufSzPtr = newBufSz ; } tempU = ( real_T * ) utMalloc ( numBuffer
* newBufSz * sizeof ( real_T ) ) ; if ( tempU == ( NULL ) ) { return ( false
) ; } tempT = tempU + newBufSz ; if ( istransportdelay ) tempX = tempT +
newBufSz ; for ( j = tail ; j < bufSz ; j ++ ) { tempT [ j - tail ] = tBuf [
j ] ; tempU [ j - tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j -
tail ] = xBuf [ j ] ; } for ( j = 0 ; j < tail ; j ++ ) { tempT [ j + bufSz -
tail ] = tBuf [ j ] ; tempU [ j + bufSz - tail ] = uBuf [ j ] ; if (
istransportdelay ) tempX [ j + bufSz - tail ] = xBuf [ j ] ; } if ( * lastPtr
> tail ) { * lastPtr -= tail ; } else { * lastPtr += ( bufSz - tail ) ; } *
tailPtr = 0 ; * headPtr = bufSz ; utFree ( uBuf ) ; * bufSzPtr = newBufSz ; *
tBufPtr = tempT ; * uBufPtr = tempU ; if ( istransportdelay ) * xBufPtr =
tempX ; } else { * tailPtr = testIdx ; } return ( true ) ; } real_T
rt_TDelayInterpolate ( real_T tMinusDelay , real_T tStart , real_T * tBuf ,
real_T * uBuf , int_T bufSz , int_T * lastIdx , int_T oldestIdx , int_T
newIdx , real_T initOutput , boolean_T discrete , boolean_T
minorStepAndTAtLastMajorOutput ) { int_T i ; real_T yout , t1 , t2 , u1 , u2
; if ( ( newIdx == 0 ) && ( oldestIdx == 0 ) && ( tMinusDelay > tStart ) )
return initOutput ; if ( tMinusDelay <= tStart ) return initOutput ; if ( (
tMinusDelay <= tBuf [ oldestIdx ] ) ) { if ( discrete ) { return ( uBuf [
oldestIdx ] ) ; } else { int_T tempIdx = oldestIdx + 1 ; if ( oldestIdx ==
bufSz - 1 ) tempIdx = 0 ; t1 = tBuf [ oldestIdx ] ; t2 = tBuf [ tempIdx ] ;
u1 = uBuf [ oldestIdx ] ; u2 = uBuf [ tempIdx ] ; if ( t2 == t1 ) { if (
tMinusDelay >= t2 ) { yout = u2 ; } else { yout = u1 ; } } else { real_T f1 =
( t2 - tMinusDelay ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; yout = f1 * u1 +
f2 * u2 ; } return yout ; } } if ( minorStepAndTAtLastMajorOutput ) { if (
newIdx != 0 ) { if ( * lastIdx == newIdx ) { ( * lastIdx ) -- ; } newIdx -- ;
} else { if ( * lastIdx == newIdx ) { * lastIdx = bufSz - 1 ; } newIdx =
bufSz - 1 ; } } i = * lastIdx ; if ( tBuf [ i ] < tMinusDelay ) { while (
tBuf [ i ] < tMinusDelay ) { if ( i == newIdx ) break ; i = ( i < ( bufSz - 1
) ) ? ( i + 1 ) : 0 ; } } else { while ( tBuf [ i ] >= tMinusDelay ) { i = (
i > 0 ) ? i - 1 : ( bufSz - 1 ) ; } i = ( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0
; } * lastIdx = i ; if ( discrete ) { double tempEps = ( DBL_EPSILON ) *
128.0 ; double localEps = tempEps * muDoubleScalarAbs ( tBuf [ i ] ) ; if (
tempEps > localEps ) { localEps = tempEps ; } localEps = localEps / 2.0 ; if
( tMinusDelay >= ( tBuf [ i ] - localEps ) ) { yout = uBuf [ i ] ; } else {
if ( i == 0 ) { yout = uBuf [ bufSz - 1 ] ; } else { yout = uBuf [ i - 1 ] ;
} } } else { if ( i == 0 ) { t1 = tBuf [ bufSz - 1 ] ; u1 = uBuf [ bufSz - 1
] ; } else { t1 = tBuf [ i - 1 ] ; u1 = uBuf [ i - 1 ] ; } t2 = tBuf [ i ] ;
u2 = uBuf [ i ] ; if ( t2 == t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; }
else { yout = u1 ; } } else { real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 )
; real_T f2 = 1.0 - f1 ; yout = f1 * u1 + f2 * u2 ; } } return ( yout ) ; }
#ifndef __RTW_UTFREE__  
extern void utFree ( void * ) ;
#endif
void rt_TDelayFreeBuf ( void * buf ) { utFree ( buf ) ; } void MdlInitialize
( void ) { boolean_T tmp ; int_T tmp_p ; char * tmp_e ; rtX . idxnpi20k1 =
rtP . PIDPhi_InitialConditionForIntegrator ; rtX . f4xemzmgtv = rtP .
PIDPhi_InitialConditionForFilter ; rtX . bj5404lz5d = rtP .
PIDZ_InitialConditionForIntegrator ; rtX . az1z4d4mub = rtP .
PIDZ_InitialConditionForFilter ; rtX . i44qjsxqq3 = rtP .
PIDTheta_InitialConditionForIntegrator ; rtX . fwjq3tjwfs = rtP .
PIDTheta_InitialConditionForFilter ; rtX . cucedlakmc = rtP .
PIDPsi_InitialConditionForIntegrator ; rtX . lq4alwhpwv = rtP .
PIDPsi_InitialConditionForFilter ; rtX . mwtxdy2wl1 = rtP . Integrator_IC ;
rtX . lu1xqtjac1 = rtP . Integrator_IC_gptrc5suwu ; tmp = false ; if ( tmp )
{ tmp_p = strcmp ( "ode3" , ssGetSolverName ( rtS ) ) ; if ( tmp_p != 0 ) {
tmp_e = solver_mismatch_message ( "ode3" , ssGetSolverName ( rtS ) ) ;
ssSetErrorStatus ( rtS , tmp_e ) ; } } } void MdlStart ( void ) {
NeslSimulator * tmp ; boolean_T tmp_p ; NeuDiagnosticManager *
diagnosticManager ; NeModelParameters modelParameters ; real_T tmp_e ;
NeuDiagnosticTree * diagnosticTree ; int32_T tmp_i ; char * msg ;
NeslSimulationData * simulationData ; real_T time ; NeModelParameters
modelParameters_p ; real_T time_p ; NeParameterBundle expl_temp ; { void * *
slioCatalogueAddr = rt_slioCatalogueAddr ( ) ; void * r2 = ( NULL ) ; void *
* pOSigstreamManagerAddr = ( NULL ) ; const int maxErrorBufferSize = 16384 ;
char errMsgCreatingOSigstreamManager [ 16384 ] ; bool
errorCreatingOSigstreamManager = false ; const char *
errorAddingR2SharedResource = ( NULL ) ; * slioCatalogueAddr =
rtwGetNewSlioCatalogue ( rt_GetMatSigLogSelectorFileName ( ) ) ;
errorAddingR2SharedResource = rtwAddR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) , 1 ) ; if (
errorAddingR2SharedResource != ( NULL ) ) { rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = ( NULL ) ; ssSetErrorStatus ( rtS
, errorAddingR2SharedResource ) ; return ; } r2 = rtwGetR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr ( ) ;
errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance (
rt_GetMatSigLogSelectorFileName ( ) , r2 , pOSigstreamManagerAddr ,
errMsgCreatingOSigstreamManager , maxErrorBufferSize ) ; if (
errorCreatingOSigstreamManager ) { * pOSigstreamManagerAddr = ( NULL ) ;
ssSetErrorStatus ( rtS , errMsgCreatingOSigstreamManager ) ; return ; } } {
bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } { int_T numCols = 2 ; rtDW .
nlope1xt2k . LoggedData = rt_CreateLogVar ( ssGetRTWLogInfo ( rtS ) ,
ssGetTStart ( rtS ) , ssGetTFinal ( rtS ) , 0.0 , ( & ssGetErrorStatus ( rtS
) ) , "Z_Data" , SS_DOUBLE , 0 , 0 , 0 , 2 , 1 , ( int_T * ) & numCols ,
NO_LOGVALDIMS , ( NULL ) , ( NULL ) , 0 , 1 , 0.001 , 1 ) ; if ( rtDW .
nlope1xt2k . LoggedData == ( NULL ) ) return ; } { { real_T * pBuffer = (
real_T * ) rt_TDelayCreateBuf ( 2 , 1024 , sizeof ( real_T ) ) ; if ( pBuffer
== ( NULL ) ) { ssSetErrorStatus ( rtS , "tdelay memory allocation error" ) ;
return ; } rtDW . j03obu1eim . Tail = 0 ; rtDW . j03obu1eim . Head = 0 ; rtDW
. j03obu1eim . Last = 0 ; rtDW . j03obu1eim . CircularBufSize = 1024 ;
pBuffer [ 0 ] = rtP . TransportDelay3_InitOutput ; pBuffer [ 1024 ] = ssGetT
( rtS ) ; rtDW . g2lkbxgjhp . TUbufferPtrs [ 0 ] = ( void * ) & pBuffer [ 0 ]
; rtDW . g2lkbxgjhp . TUbufferPtrs [ 1 ] = ( void * ) & pBuffer [ 1024 ] ; }
} { { real_T * pBuffer = ( real_T * ) rt_TDelayCreateBuf ( 2 , 1024 , sizeof
( real_T ) ) ; if ( pBuffer == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } rtDW . cadeui54l4 . Tail = 0
; rtDW . cadeui54l4 . Head = 0 ; rtDW . cadeui54l4 . Last = 0 ; rtDW .
cadeui54l4 . CircularBufSize = 1024 ; pBuffer [ 0 ] = rtP .
TransportDelay1_InitOutput ; pBuffer [ 1024 ] = ssGetT ( rtS ) ; rtDW .
lizyfiedvc . TUbufferPtrs [ 0 ] = ( void * ) & pBuffer [ 0 ] ; rtDW .
lizyfiedvc . TUbufferPtrs [ 1 ] = ( void * ) & pBuffer [ 1024 ] ; } } { {
real_T * pBuffer = ( real_T * ) rt_TDelayCreateBuf ( 2 , 1024 , sizeof (
real_T ) ) ; if ( pBuffer == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } rtDW . dbj1dpcqc0 . Tail = 0
; rtDW . dbj1dpcqc0 . Head = 0 ; rtDW . dbj1dpcqc0 . Last = 0 ; rtDW .
dbj1dpcqc0 . CircularBufSize = 1024 ; pBuffer [ 0 ] = rtP .
TransportDelay2_InitOutput ; pBuffer [ 1024 ] = ssGetT ( rtS ) ; rtDW .
mx22ld2hzj . TUbufferPtrs [ 0 ] = ( void * ) & pBuffer [ 0 ] ; rtDW .
mx22ld2hzj . TUbufferPtrs [ 1 ] = ( void * ) & pBuffer [ 1024 ] ; } } { {
int_T i1 ; int_T * iw_Tail = & rtDW . ko54z1uphk . Tail [ 0 ] ; int_T *
iw_Head = & rtDW . ko54z1uphk . Head [ 0 ] ; int_T * iw_Last = & rtDW .
ko54z1uphk . Last [ 0 ] ; int_T * iw_CircularBufSize = & rtDW . ko54z1uphk .
CircularBufSize [ 0 ] ; void * * pw_TUbufferPtrs = & rtDW . diva5n50pk .
TUbufferPtrs [ 0 ] ; for ( i1 = 0 ; i1 < 3 ; i1 ++ ) { real_T * pBuffer = (
real_T * ) rt_TDelayCreateBuf ( 2 , 1024 , sizeof ( real_T ) ) ; if ( pBuffer
== ( NULL ) ) { ssSetErrorStatus ( rtS , "tdelay memory allocation error" ) ;
return ; } iw_Tail [ i1 ] = 0 ; iw_Head [ i1 ] = 0 ; iw_Last [ i1 ] = 0 ;
iw_CircularBufSize [ i1 ] = 1024 ; pBuffer [ 0 ] = rtP .
TransportDelay4_InitOutput ; pBuffer [ 1024 ] = ssGetT ( rtS ) ;
pw_TUbufferPtrs [ i1 ] = ( void * ) & pBuffer [ 0 ] ; pw_TUbufferPtrs [ i1 +
3 ] = ( void * ) & pBuffer [ 1024 ] ; } } } tmp = nesl_lease_simulator (
"Control_Bicopter/Subsystem/Solver Configuration_1" , 0 , 0 ) ; rtDW .
edvzy5545y = ( void * ) tmp ; tmp_p = pointer_is_null ( rtDW . edvzy5545y ) ;
if ( tmp_p ) { Control_Bicopter_ae14a523_1_gateway ( ) ; tmp =
nesl_lease_simulator ( "Control_Bicopter/Subsystem/Solver Configuration_1" ,
0 , 0 ) ; rtDW . edvzy5545y = ( void * ) tmp ; } simulationData =
nesl_create_simulation_data ( ) ; rtDW . m2zz55qyqo = ( void * )
simulationData ; diagnosticManager = rtw_create_diagnostics ( ) ; rtDW .
nbicecjjvm = ( void * ) diagnosticManager ; modelParameters . mSolverType =
NE_SOLVER_TYPE_DAE ; modelParameters . mSolverTolerance = 0.001 ;
modelParameters . mVariableStepSolver = false ; modelParameters .
mFixedStepSize = 0.001 ; modelParameters . mStartTime = 0.0 ; modelParameters
. mLoadInitialState = false ; modelParameters . mUseSimState = false ;
modelParameters . mLinTrimCompile = false ; modelParameters . mLoggingMode =
SSC_LOGGING_NONE ; modelParameters . mRTWModifiedTimeStamp = 5.37840885E+8 ;
tmp_e = 0.001 ; modelParameters . mSolverTolerance = tmp_e ; tmp_e = 0.001 ;
modelParameters . mFixedStepSize = tmp_e ; tmp_p = false ; modelParameters .
mVariableStepSolver = tmp_p ; diagnosticManager = ( NeuDiagnosticManager * )
rtDW . nbicecjjvm ; diagnosticTree = neu_diagnostic_manager_get_initial_tree
( diagnosticManager ) ; tmp_i = nesl_initialize_simulator ( ( NeslSimulator *
) rtDW . edvzy5545y , & modelParameters , diagnosticManager ) ; if ( tmp_i !=
0 ) { tmp_p = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp_p
) { msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS ,
msg ) ; } } expl_temp . mRealParameters . mN = 0 ; expl_temp .
mRealParameters . mX = NULL ; expl_temp . mLogicalParameters . mN = 0 ;
expl_temp . mLogicalParameters . mX = NULL ; expl_temp . mIntegerParameters .
mN = 0 ; expl_temp . mIntegerParameters . mX = NULL ; expl_temp .
mIndexParameters . mN = 0 ; expl_temp . mIndexParameters . mX = NULL ;
nesl_simulator_set_rtps ( ( NeslSimulator * ) rtDW . edvzy5545y , expl_temp )
; simulationData = ( NeslSimulationData * ) rtDW . m2zz55qyqo ; time = ssGetT
( rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData
-> mTime . mX = & time ; simulationData -> mData -> mContStates . mN = 61 ;
simulationData -> mData -> mContStates . mX = & rtX . fmofelvad4 [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ggqhsa10cs ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
j24ttftmn0 ; tmp_p = false ; simulationData -> mData -> mFoundZcEvents =
tmp_p ; simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS
) ; tmp_p = ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U )
; simulationData -> mData -> mIsSolverAssertCheck = tmp_p ; simulationData ->
mData -> mIsSolverCheckingCIC = false ; tmp_p = ssIsSolverComputingJacobian (
rtS ) ; simulationData -> mData -> mIsComputingJacobian = tmp_p ;
simulationData -> mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian (
rtS ) != 0 ) ; simulationData -> mData -> mIsSolverRequestingReset = false ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . nbicecjjvm ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW . edvzy5545y ,
NESL_SIM_INITIALIZEONCE , simulationData , diagnosticManager ) ; if ( tmp_i
!= 0 ) { tmp_p = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if (
tmp_p ) { msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus (
rtS , msg ) ; } } tmp = nesl_lease_simulator (
"Control_Bicopter/Subsystem/Solver Configuration_1" , 1 , 0 ) ; rtDW .
avtvu4a0bu = ( void * ) tmp ; tmp_p = pointer_is_null ( rtDW . avtvu4a0bu ) ;
if ( tmp_p ) { Control_Bicopter_ae14a523_1_gateway ( ) ; tmp =
nesl_lease_simulator ( "Control_Bicopter/Subsystem/Solver Configuration_1" ,
1 , 0 ) ; rtDW . avtvu4a0bu = ( void * ) tmp ; } simulationData =
nesl_create_simulation_data ( ) ; rtDW . b1ivukxskp = ( void * )
simulationData ; diagnosticManager = rtw_create_diagnostics ( ) ; rtDW .
hzasxj2szj = ( void * ) diagnosticManager ; modelParameters_p . mSolverType =
NE_SOLVER_TYPE_DAE ; modelParameters_p . mSolverTolerance = 0.001 ;
modelParameters_p . mVariableStepSolver = false ; modelParameters_p .
mFixedStepSize = 0.001 ; modelParameters_p . mStartTime = 0.0 ;
modelParameters_p . mLoadInitialState = false ; modelParameters_p .
mUseSimState = false ; modelParameters_p . mLinTrimCompile = false ;
modelParameters_p . mLoggingMode = SSC_LOGGING_NONE ; modelParameters_p .
mRTWModifiedTimeStamp = 5.37840885E+8 ; tmp_e = 0.001 ; modelParameters_p .
mSolverTolerance = tmp_e ; tmp_e = 0.001 ; modelParameters_p . mFixedStepSize
= tmp_e ; tmp_p = false ; modelParameters_p . mVariableStepSolver = tmp_p ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . hzasxj2szj ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = nesl_initialize_simulator ( ( NeslSimulator * ) rtDW . avtvu4a0bu
, & modelParameters_p , diagnosticManager ) ; if ( tmp_i != 0 ) { tmp_p =
error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp_p ) { msg =
rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg ) ; } }
simulationData = ( NeslSimulationData * ) rtDW . b1ivukxskp ; time_p = ssGetT
( rtS ) ; simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData
-> mTime . mX = & time_p ; simulationData -> mData -> mContStates . mN = 0 ;
simulationData -> mData -> mContStates . mX = NULL ; simulationData -> mData
-> mDiscStates . mN = 0 ; simulationData -> mData -> mDiscStates . mX = &
rtDW . kaia4u11f2 ; simulationData -> mData -> mModeVector . mN = 0 ;
simulationData -> mData -> mModeVector . mX = & rtDW . ckkmlyrmch ; tmp_p =
false ; simulationData -> mData -> mFoundZcEvents = tmp_p ; simulationData ->
mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp_p = (
ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp_p ; simulationData ->
mData -> mIsSolverCheckingCIC = false ; simulationData -> mData ->
mIsComputingJacobian = false ; simulationData -> mData -> mIsEvaluatingF0 =
false ; simulationData -> mData -> mIsSolverRequestingReset = false ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . hzasxj2szj ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW . avtvu4a0bu ,
NESL_SIM_INITIALIZEONCE , simulationData , diagnosticManager ) ; if ( tmp_i
!= 0 ) { tmp_p = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if (
tmp_p ) { msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus (
rtS , msg ) ; } } rtDW . g5ujhhy5gx = - 1 ; MdlInitialize ( ) ; } void
MdlOutputs ( int_T tid ) { boolean_T first_output ; NeslSimulationData *
simulationData ; real_T time ; real_T tmp [ 24 ] ; int_T tmp_p [ 7 ] ;
NeuDiagnosticManager * diagnosticManager ; NeuDiagnosticTree * diagnosticTree
; int32_T tmp_e ; char * msg ; real_T time_p ; real_T tmp_i [ 85 ] ; int_T
tmp_m [ 8 ] ; int8_T rtAction ; real_T fjcz5gdia5_idx_2 ; real_T
g2ynm5vjgu_idx_1 ; real_T g2ynm5vjgu_idx_0 ; real_T g2ynm5vjgu_idx_2 ; real_T
g2ynm5vjgu_idx_3 ; real_T imakcnimkl_idx_0 ; real_T imakcnimkl_idx_1 ; real_T
imakcnimkl_idx_2 ; real_T imakcnimkl_idx_3 ; srClearBC ( rtDW . n4vwxzlvdz )
; srClearBC ( rtDW . gte2z2pt0y ) ; srClearBC ( rtDW . eghku2cvih ) ; {
real_T * * uBuffer = ( real_T * * ) & rtDW . g2lkbxgjhp . TUbufferPtrs [ 0 ]
; real_T * * tBuffer = ( real_T * * ) & rtDW . g2lkbxgjhp . TUbufferPtrs [ 1
] ; real_T simTime = ssGetT ( rtS ) ; real_T tMinusDelay = simTime - rtP .
time_delay ; rtB . alnu044j5k = rt_TDelayInterpolate ( tMinusDelay , 0.0 , *
tBuffer , * uBuffer , rtDW . j03obu1eim . CircularBufSize , & rtDW .
j03obu1eim . Last , rtDW . j03obu1eim . Tail , rtDW . j03obu1eim . Head , rtP
. TransportDelay3_InitOutput , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( rtS )
&& ( ssGetTimeOfLastOutput ( rtS ) == ssGetT ( rtS ) ) ) ) ; } rtY .
mkujrenrvw = rtB . alnu044j5k ; { real_T * * uBuffer = ( real_T * * ) & rtDW
. lizyfiedvc . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) &
rtDW . lizyfiedvc . TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( rtS ) ;
real_T tMinusDelay = simTime - rtP . time_delay ; rtB . g41m4pmy1w =
rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , * uBuffer , rtDW .
cadeui54l4 . CircularBufSize , & rtDW . cadeui54l4 . Last , rtDW . cadeui54l4
. Tail , rtDW . cadeui54l4 . Head , rtP . TransportDelay1_InitOutput , 0 , (
boolean_T ) ( ssIsMinorTimeStep ( rtS ) && ( ssGetTimeOfLastOutput ( rtS ) ==
ssGetT ( rtS ) ) ) ) ; } rtY . ki01i2zemv = rtB . g41m4pmy1w ; { real_T * *
uBuffer = ( real_T * * ) & rtDW . mx22ld2hzj . TUbufferPtrs [ 0 ] ; real_T *
* tBuffer = ( real_T * * ) & rtDW . mx22ld2hzj . TUbufferPtrs [ 1 ] ; real_T
simTime = ssGetT ( rtS ) ; real_T tMinusDelay = simTime - rtP . time_delay ;
rtB . kuc2djevrk = rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , rtDW . dbj1dpcqc0 . CircularBufSize , & rtDW . dbj1dpcqc0 . Last ,
rtDW . dbj1dpcqc0 . Tail , rtDW . dbj1dpcqc0 . Head , rtP .
TransportDelay2_InitOutput , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( rtS ) &&
( ssGetTimeOfLastOutput ( rtS ) == ssGetT ( rtS ) ) ) ) ; } rtY . cuajxr4igq
= rtB . kuc2djevrk ; { real_T * * uBuffer = ( real_T * * ) & rtDW .
diva5n50pk . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & rtDW
. diva5n50pk . TUbufferPtrs [ 3 ] ; real_T simTime = ssGetT ( rtS ) ; real_T
tMinusDelay ; { int_T i1 ; real_T * y0 = & rtB . pzxksc1hoi [ 0 ] ; int_T *
iw_Tail = & rtDW . ko54z1uphk . Tail [ 0 ] ; int_T * iw_Head = & rtDW .
ko54z1uphk . Head [ 0 ] ; int_T * iw_Last = & rtDW . ko54z1uphk . Last [ 0 ]
; int_T * iw_CircularBufSize = & rtDW . ko54z1uphk . CircularBufSize [ 0 ] ;
for ( i1 = 0 ; i1 < 3 ; i1 ++ ) { tMinusDelay = ( ( rtP . time_delay > 0.0 )
? rtP . time_delay : 0.0 ) ; tMinusDelay = simTime - tMinusDelay ; y0 [ i1 ]
= rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , * uBuffer ,
iw_CircularBufSize [ i1 ] , & iw_Last [ i1 ] , iw_Tail [ i1 ] , iw_Head [ i1
] , rtP . TransportDelay4_InitOutput , 0 , ( boolean_T ) ( ssIsMinorTimeStep
( rtS ) && ( ssGetTimeOfLastOutput ( rtS ) == ssGetT ( rtS ) ) ) ) ; tBuffer
++ ; uBuffer ++ ; } } } rtY . i5hbuq5xsv [ 0 ] = rtB . pzxksc1hoi [ 0 ] ; rtY
. i5hbuq5xsv [ 1 ] = rtB . pzxksc1hoi [ 1 ] ; rtY . i5hbuq5xsv [ 2 ] = rtB .
pzxksc1hoi [ 2 ] ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if ( ssGetLogOutput
( rtS ) ) { real_T u [ 2 ] ; u [ 0 ] = ssGetTaskTime ( rtS , 1 ) ; ; u [ 1 ]
= rtB . kuc2djevrk ; rt_UpdateLogVar ( ( LogVar * ) rtDW . nlope1xt2k .
LoggedData , u , 0 ) ; } } g2ynm5vjgu_idx_0 = rtP . Phi_ref_Value - rtB .
pzxksc1hoi [ 2 ] ; g2ynm5vjgu_idx_1 = rtP . Z_step - rtB . kuc2djevrk ;
g2ynm5vjgu_idx_2 = rtP . Theta_ref_Value - rtB . pzxksc1hoi [ 1 ] ;
g2ynm5vjgu_idx_3 = rtP . Psi_ref_Value - rtB . pzxksc1hoi [ 0 ] ; rtB .
m1dkcmbbey = ( rtP . D_Phi * g2ynm5vjgu_idx_0 - rtX . f4xemzmgtv ) * rtP .
PIDPhi_N ; rtB . onyodjbxop = ( rtP . D_Z * g2ynm5vjgu_idx_1 - rtX .
az1z4d4mub ) * rtP . PIDZ_N ; rtB . ibl1bvoot5 = ( rtP . D_Theta *
g2ynm5vjgu_idx_2 - rtX . fwjq3tjwfs ) * rtP . PIDTheta_N ; rtB . ozxva0egn1 =
( rtP . D_Psi * g2ynm5vjgu_idx_3 - rtX . lq4alwhpwv ) * rtP . PIDPsi_N ;
imakcnimkl_idx_0 = ( ( rtP . P_Phi * g2ynm5vjgu_idx_0 + rtX . idxnpi20k1 ) +
rtB . m1dkcmbbey ) * rtP . Gain_Gain ; imakcnimkl_idx_1 = ( ( rtP . P_Z *
g2ynm5vjgu_idx_1 + rtX . bj5404lz5d ) + rtB . onyodjbxop ) + rtP .
copter_mass * rtP . g ; imakcnimkl_idx_2 = ( rtP . P_Theta * g2ynm5vjgu_idx_2
+ rtX . i44qjsxqq3 ) + rtB . ibl1bvoot5 ; imakcnimkl_idx_3 = ( rtP . P_Psi *
g2ynm5vjgu_idx_3 + rtX . cucedlakmc ) + rtB . ozxva0egn1 ; if (
muDoubleScalarAbs ( imakcnimkl_idx_0 + imakcnimkl_idx_1 ) < 0.001 ) {
fjcz5gdia5_idx_2 = 0.0 ; } else { fjcz5gdia5_idx_2 = ( imakcnimkl_idx_2 +
imakcnimkl_idx_3 ) / ( imakcnimkl_idx_0 + imakcnimkl_idx_1 ) ; } if (
muDoubleScalarAbs ( imakcnimkl_idx_0 - imakcnimkl_idx_1 ) < 0.001 ) {
imakcnimkl_idx_3 = 0.0 ; } else { imakcnimkl_idx_3 = ( imakcnimkl_idx_2 -
imakcnimkl_idx_3 ) / ( imakcnimkl_idx_1 - imakcnimkl_idx_0 ) ; }
imakcnimkl_idx_2 = ( imakcnimkl_idx_0 + imakcnimkl_idx_1 ) / 2.0 ;
imakcnimkl_idx_0 = ( imakcnimkl_idx_1 - imakcnimkl_idx_0 ) / 2.0 ; rtB .
eslkbwnlvg = rtP . I_Phi * g2ynm5vjgu_idx_0 ; rtB . e04tk5zbbe = rtP . I_Psi
* g2ynm5vjgu_idx_3 ; rtB . lgntheub31 = rtP . I_Theta * g2ynm5vjgu_idx_2 ;
rtB . ms5qo0q32e = rtP . I_Z * g2ynm5vjgu_idx_1 ; if ( imakcnimkl_idx_2 > rtP
. Saturation_F1_UpperSat ) { imakcnimkl_idx_2 = rtP . Saturation_F1_UpperSat
; } else { if ( imakcnimkl_idx_2 < rtP . Saturation_F1_LowerSat ) {
imakcnimkl_idx_2 = rtP . Saturation_F1_LowerSat ; } } if ( imakcnimkl_idx_0 >
rtP . Saturation_F2_UpperSat ) { imakcnimkl_idx_0 = rtP .
Saturation_F2_UpperSat ; } else { if ( imakcnimkl_idx_0 < rtP .
Saturation_F2_LowerSat ) { imakcnimkl_idx_0 = rtP . Saturation_F2_LowerSat ;
} } if ( fjcz5gdia5_idx_2 > rtP . Saturation_F3_UpperSat ) { fjcz5gdia5_idx_2
= rtP . Saturation_F3_UpperSat ; } else { if ( fjcz5gdia5_idx_2 < rtP .
Saturation_F3_LowerSat ) { fjcz5gdia5_idx_2 = rtP . Saturation_F3_LowerSat ;
} } rtB . agzdgrfhap = rtP . Gain1_Gain * fjcz5gdia5_idx_2 + rtP .
Constant_Value_pr21eedhgl ; if ( rtDW . fkv2s4cq4q == 0.0 ) { rtDW .
fkv2s4cq4q = 1.0 ; rtX . lryf2ntwdb [ 0 ] = rtB . agzdgrfhap ; rtX .
lryf2ntwdb [ 1 ] = 0.0 ; } rtB . eczwzi2yjw [ 0 ] = rtX . lryf2ntwdb [ 0 ] ;
rtB . eczwzi2yjw [ 1 ] = rtX . lryf2ntwdb [ 1 ] ; rtB . eczwzi2yjw [ 2 ] = (
( rtB . agzdgrfhap - rtX . lryf2ntwdb [ 0 ] ) * 1000.0 - 2.0 * rtX .
lryf2ntwdb [ 1 ] ) * 1000.0 ; rtB . eczwzi2yjw [ 3 ] = 0.0 ; if (
imakcnimkl_idx_3 > rtP . Saturation_F4_UpperSat ) { imakcnimkl_idx_3 = rtP .
Saturation_F4_UpperSat ; } else { if ( imakcnimkl_idx_3 < rtP .
Saturation_F4_LowerSat ) { imakcnimkl_idx_3 = rtP . Saturation_F4_LowerSat ;
} } rtB . k4jt1nr2ej = rtP . Gain2_Gain * imakcnimkl_idx_3 + rtP .
Constant1_Value ; if ( rtDW . jm54tlayqp == 0.0 ) { rtDW . jm54tlayqp = 1.0 ;
rtX . px5f4y1l5a [ 0 ] = rtB . k4jt1nr2ej ; rtX . px5f4y1l5a [ 1 ] = 0.0 ; }
rtB . gudikbezkv [ 0 ] = rtX . px5f4y1l5a [ 0 ] ; rtB . gudikbezkv [ 1 ] =
rtX . px5f4y1l5a [ 1 ] ; rtB . gudikbezkv [ 2 ] = ( ( rtB . k4jt1nr2ej - rtX
. px5f4y1l5a [ 0 ] ) * 1000.0 - 2.0 * rtX . px5f4y1l5a [ 1 ] ) * 1000.0 ; rtB
. gudikbezkv [ 3 ] = 0.0 ; rtB . b2gk3wukry = rtX . mwtxdy2wl1 ; if ( rtDW .
hquutdkqak == 0.0 ) { rtDW . hquutdkqak = 1.0 ; rtX . fbwrmyrd20 [ 0 ] = rtB
. b2gk3wukry ; rtX . fbwrmyrd20 [ 1 ] = 0.0 ; } rtB . nxogfkpljw [ 0 ] = rtX
. fbwrmyrd20 [ 0 ] ; rtB . nxogfkpljw [ 1 ] = rtX . fbwrmyrd20 [ 1 ] ; rtB .
nxogfkpljw [ 2 ] = ( ( rtB . b2gk3wukry - rtX . fbwrmyrd20 [ 0 ] ) * 1000.0 -
2.0 * rtX . fbwrmyrd20 [ 1 ] ) * 1000.0 ; rtB . nxogfkpljw [ 3 ] = 0.0 ; rtB
. gipodijd1l = rtX . lu1xqtjac1 ; if ( rtDW . kbkgyu20nl == 0.0 ) { rtDW .
kbkgyu20nl = 1.0 ; rtX . bl2jw4bwps [ 0 ] = rtB . gipodijd1l ; rtX .
bl2jw4bwps [ 1 ] = 0.0 ; } rtB . e4vcd3rbgn [ 0 ] = rtX . bl2jw4bwps [ 0 ] ;
rtB . e4vcd3rbgn [ 1 ] = rtX . bl2jw4bwps [ 1 ] ; rtB . e4vcd3rbgn [ 2 ] = (
( rtB . gipodijd1l - rtX . bl2jw4bwps [ 0 ] ) * 1000.0 - 2.0 * rtX .
bl2jw4bwps [ 1 ] ) * 1000.0 ; rtB . e4vcd3rbgn [ 3 ] = 0.0 ; simulationData =
( NeslSimulationData * ) rtDW . m2zz55qyqo ; time = ssGetT ( rtS ) ;
simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData -> mTime
. mX = & time ; simulationData -> mData -> mContStates . mN = 61 ;
simulationData -> mData -> mContStates . mX = & rtX . fmofelvad4 [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ggqhsa10cs ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
j24ttftmn0 ; first_output = false ; simulationData -> mData -> mFoundZcEvents
= first_output ; simulationData -> mData -> mIsMajorTimeStep =
ssIsMajorTimeStep ( rtS ) ; first_output = ( ssGetMdlInfoPtr ( rtS ) ->
mdlFlags . solverAssertCheck == 1U ) ; simulationData -> mData ->
mIsSolverAssertCheck = first_output ; simulationData -> mData ->
mIsSolverCheckingCIC = false ; first_output = ssIsSolverComputingJacobian (
rtS ) ; simulationData -> mData -> mIsComputingJacobian = first_output ;
simulationData -> mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian (
rtS ) != 0 ) ; simulationData -> mData -> mIsSolverRequestingReset = false ;
tmp_p [ 0 ] = 0 ; tmp [ 0 ] = rtB . eczwzi2yjw [ 0 ] ; tmp [ 1 ] = rtB .
eczwzi2yjw [ 1 ] ; tmp [ 2 ] = rtB . eczwzi2yjw [ 2 ] ; tmp [ 3 ] = rtB .
eczwzi2yjw [ 3 ] ; tmp_p [ 1 ] = 4 ; tmp [ 4 ] = rtB . gudikbezkv [ 0 ] ; tmp
[ 5 ] = rtB . gudikbezkv [ 1 ] ; tmp [ 6 ] = rtB . gudikbezkv [ 2 ] ; tmp [ 7
] = rtB . gudikbezkv [ 3 ] ; tmp_p [ 2 ] = 8 ; tmp [ 8 ] = rtB . nxogfkpljw [
0 ] ; tmp [ 9 ] = rtB . nxogfkpljw [ 1 ] ; tmp [ 10 ] = rtB . nxogfkpljw [ 2
] ; tmp [ 11 ] = rtB . nxogfkpljw [ 3 ] ; tmp_p [ 3 ] = 12 ; tmp [ 12 ] = rtB
. e4vcd3rbgn [ 0 ] ; tmp [ 13 ] = rtB . e4vcd3rbgn [ 1 ] ; tmp [ 14 ] = rtB .
e4vcd3rbgn [ 2 ] ; tmp [ 15 ] = rtB . e4vcd3rbgn [ 3 ] ; tmp_p [ 4 ] = 16 ;
tmp [ 16 ] = rtB . o2kcntnybn [ 0 ] ; tmp [ 17 ] = rtB . o2kcntnybn [ 1 ] ;
tmp [ 18 ] = rtB . o2kcntnybn [ 2 ] ; tmp [ 19 ] = rtB . o2kcntnybn [ 3 ] ;
tmp_p [ 5 ] = 20 ; tmp [ 20 ] = rtB . hqhvl0otti [ 0 ] ; tmp [ 21 ] = rtB .
hqhvl0otti [ 1 ] ; tmp [ 22 ] = rtB . hqhvl0otti [ 2 ] ; tmp [ 23 ] = rtB .
hqhvl0otti [ 3 ] ; tmp_p [ 6 ] = 24 ; simulationData -> mData -> mInputValues
. mN = 24 ; simulationData -> mData -> mInputValues . mX = & tmp [ 0 ] ;
simulationData -> mData -> mInputOffsets . mN = 7 ; simulationData -> mData
-> mInputOffsets . mX = & tmp_p [ 0 ] ; simulationData -> mData -> mOutputs .
mN = 61 ; simulationData -> mData -> mOutputs . mX = & rtB . igfxglvoam [ 0 ]
; simulationData -> mData -> mSampleHits . mN = 0 ; simulationData -> mData
-> mSampleHits . mX = NULL ; simulationData -> mData ->
mIsFundamentalSampleHit = false ; simulationData -> mData -> mTolerances . mN
= 0 ; simulationData -> mData -> mTolerances . mX = NULL ; simulationData ->
mData -> mCstateHasChanged = false ; diagnosticManager = (
NeuDiagnosticManager * ) rtDW . nbicecjjvm ; diagnosticTree =
neu_diagnostic_manager_get_initial_tree ( diagnosticManager ) ; tmp_e =
ne_simulator_method ( ( NeslSimulator * ) rtDW . edvzy5545y ,
NESL_SIM_OUTPUTS , simulationData , diagnosticManager ) ; if ( tmp_e != 0 ) {
first_output = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if (
first_output ) { msg = rtw_diagnostics_msg ( diagnosticTree ) ;
ssSetErrorStatus ( rtS , msg ) ; } } if ( ssIsMajorTimeStep ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } simulationData = (
NeslSimulationData * ) rtDW . b1ivukxskp ; time_p = ssGetT ( rtS ) ;
simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData -> mTime
. mX = & time_p ; simulationData -> mData -> mContStates . mN = 0 ;
simulationData -> mData -> mContStates . mX = NULL ; simulationData -> mData
-> mDiscStates . mN = 0 ; simulationData -> mData -> mDiscStates . mX = &
rtDW . kaia4u11f2 ; simulationData -> mData -> mModeVector . mN = 0 ;
simulationData -> mData -> mModeVector . mX = & rtDW . ckkmlyrmch ;
first_output = false ; simulationData -> mData -> mFoundZcEvents =
first_output ; simulationData -> mData -> mIsMajorTimeStep =
ssIsMajorTimeStep ( rtS ) ; first_output = ( ssGetMdlInfoPtr ( rtS ) ->
mdlFlags . solverAssertCheck == 1U ) ; simulationData -> mData ->
mIsSolverAssertCheck = first_output ; simulationData -> mData ->
mIsSolverCheckingCIC = false ; simulationData -> mData ->
mIsComputingJacobian = false ; simulationData -> mData -> mIsEvaluatingF0 =
false ; simulationData -> mData -> mIsSolverRequestingReset = false ; tmp_m [
0 ] = 0 ; tmp_i [ 0 ] = rtB . eczwzi2yjw [ 0 ] ; tmp_i [ 1 ] = rtB .
eczwzi2yjw [ 1 ] ; tmp_i [ 2 ] = rtB . eczwzi2yjw [ 2 ] ; tmp_i [ 3 ] = rtB .
eczwzi2yjw [ 3 ] ; tmp_m [ 1 ] = 4 ; tmp_i [ 4 ] = rtB . gudikbezkv [ 0 ] ;
tmp_i [ 5 ] = rtB . gudikbezkv [ 1 ] ; tmp_i [ 6 ] = rtB . gudikbezkv [ 2 ] ;
tmp_i [ 7 ] = rtB . gudikbezkv [ 3 ] ; tmp_m [ 2 ] = 8 ; tmp_i [ 8 ] = rtB .
nxogfkpljw [ 0 ] ; tmp_i [ 9 ] = rtB . nxogfkpljw [ 1 ] ; tmp_i [ 10 ] = rtB
. nxogfkpljw [ 2 ] ; tmp_i [ 11 ] = rtB . nxogfkpljw [ 3 ] ; tmp_m [ 3 ] = 12
; tmp_i [ 12 ] = rtB . e4vcd3rbgn [ 0 ] ; tmp_i [ 13 ] = rtB . e4vcd3rbgn [ 1
] ; tmp_i [ 14 ] = rtB . e4vcd3rbgn [ 2 ] ; tmp_i [ 15 ] = rtB . e4vcd3rbgn [
3 ] ; tmp_m [ 4 ] = 16 ; tmp_i [ 16 ] = rtB . o2kcntnybn [ 0 ] ; tmp_i [ 17 ]
= rtB . o2kcntnybn [ 1 ] ; tmp_i [ 18 ] = rtB . o2kcntnybn [ 2 ] ; tmp_i [ 19
] = rtB . o2kcntnybn [ 3 ] ; tmp_m [ 5 ] = 20 ; tmp_i [ 20 ] = rtB .
hqhvl0otti [ 0 ] ; tmp_i [ 21 ] = rtB . hqhvl0otti [ 1 ] ; tmp_i [ 22 ] = rtB
. hqhvl0otti [ 2 ] ; tmp_i [ 23 ] = rtB . hqhvl0otti [ 3 ] ; tmp_m [ 6 ] = 24
; memcpy ( & tmp_i [ 24 ] , & rtB . igfxglvoam [ 0 ] , 61U * sizeof ( real_T
) ) ; tmp_m [ 7 ] = 85 ; simulationData -> mData -> mInputValues . mN = 85 ;
simulationData -> mData -> mInputValues . mX = & tmp_i [ 0 ] ; simulationData
-> mData -> mInputOffsets . mN = 8 ; simulationData -> mData -> mInputOffsets
. mX = & tmp_m [ 0 ] ; simulationData -> mData -> mOutputs . mN = 9 ;
simulationData -> mData -> mOutputs . mX = & rtB . oqquwhnyjp [ 0 ] ;
simulationData -> mData -> mSampleHits . mN = 0 ; simulationData -> mData ->
mSampleHits . mX = NULL ; simulationData -> mData -> mIsFundamentalSampleHit
= false ; simulationData -> mData -> mTolerances . mN = 0 ; simulationData ->
mData -> mTolerances . mX = NULL ; simulationData -> mData ->
mCstateHasChanged = false ; diagnosticManager = ( NeuDiagnosticManager * )
rtDW . hzasxj2szj ; diagnosticTree = neu_diagnostic_manager_get_initial_tree
( diagnosticManager ) ; tmp_e = ne_simulator_method ( ( NeslSimulator * )
rtDW . avtvu4a0bu , NESL_SIM_OUTPUTS , simulationData , diagnosticManager ) ;
if ( tmp_e != 0 ) { first_output = error_buffer_is_empty ( ssGetErrorStatus (
rtS ) ) ; if ( first_output ) { msg = rtw_diagnostics_msg ( diagnosticTree )
; ssSetErrorStatus ( rtS , msg ) ; } } if ( ssIsMajorTimeStep ( rtS ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } g2ynm5vjgu_idx_0 = rtB
. oqquwhnyjp [ 5 ] * rtB . oqquwhnyjp [ 5 ] ; g2ynm5vjgu_idx_1 = rtB .
oqquwhnyjp [ 6 ] * rtB . oqquwhnyjp [ 6 ] ; g2ynm5vjgu_idx_2 = rtB .
oqquwhnyjp [ 7 ] * rtB . oqquwhnyjp [ 7 ] ; g2ynm5vjgu_idx_3 = rtB .
oqquwhnyjp [ 8 ] * rtB . oqquwhnyjp [ 8 ] ; g2ynm5vjgu_idx_3 =
muDoubleScalarSqrt ( ( ( g2ynm5vjgu_idx_0 + g2ynm5vjgu_idx_1 ) +
g2ynm5vjgu_idx_2 ) + g2ynm5vjgu_idx_3 ) ; g2ynm5vjgu_idx_0 = rtB . oqquwhnyjp
[ 5 ] / g2ynm5vjgu_idx_3 ; g2ynm5vjgu_idx_1 = rtB . oqquwhnyjp [ 6 ] /
g2ynm5vjgu_idx_3 ; g2ynm5vjgu_idx_2 = rtB . oqquwhnyjp [ 7 ] /
g2ynm5vjgu_idx_3 ; g2ynm5vjgu_idx_3 = rtB . oqquwhnyjp [ 8 ] /
g2ynm5vjgu_idx_3 ; fjcz5gdia5_idx_2 = ( g2ynm5vjgu_idx_1 * g2ynm5vjgu_idx_3 -
g2ynm5vjgu_idx_0 * g2ynm5vjgu_idx_2 ) * - 2.0 ; if ( ssIsMajorTimeStep ( rtS
) ) { if ( fjcz5gdia5_idx_2 > 1.0 ) { rtAction = 0 ; } else if (
fjcz5gdia5_idx_2 < - 1.0 ) { rtAction = 1 ; } else { rtAction = 2 ; } rtDW .
g5ujhhy5gx = rtAction ; } else { rtAction = rtDW . g5ujhhy5gx ; } switch (
rtAction ) { case 0 : if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtB . p1zfkdy0ss
= rtP . Constant_Value ; } if ( ssIsMajorTimeStep ( rtS ) ) { srUpdateBC (
rtDW . n4vwxzlvdz ) ; } break ; case 1 : if ( ssIsSampleHit ( rtS , 1 , 0 ) )
{ rtB . p1zfkdy0ss = rtP . Constant_Value_opjr4pzkxp ; } if (
ssIsMajorTimeStep ( rtS ) ) { srUpdateBC ( rtDW . gte2z2pt0y ) ; } break ;
case 2 : rtB . p1zfkdy0ss = fjcz5gdia5_idx_2 ; if ( ssIsMajorTimeStep ( rtS )
) { srUpdateBC ( rtDW . eghku2cvih ) ; } break ; } rtB . omd4dozqfc [ 0 ] =
muDoubleScalarAtan2 ( ( g2ynm5vjgu_idx_1 * g2ynm5vjgu_idx_2 +
g2ynm5vjgu_idx_0 * g2ynm5vjgu_idx_3 ) * 2.0 , ( ( g2ynm5vjgu_idx_0 *
g2ynm5vjgu_idx_0 + g2ynm5vjgu_idx_1 * g2ynm5vjgu_idx_1 ) - g2ynm5vjgu_idx_2 *
g2ynm5vjgu_idx_2 ) - g2ynm5vjgu_idx_3 * g2ynm5vjgu_idx_3 ) ; rtB . omd4dozqfc
[ 2 ] = muDoubleScalarAtan2 ( ( g2ynm5vjgu_idx_2 * g2ynm5vjgu_idx_3 +
g2ynm5vjgu_idx_0 * g2ynm5vjgu_idx_1 ) * 2.0 , ( ( g2ynm5vjgu_idx_0 *
g2ynm5vjgu_idx_0 - g2ynm5vjgu_idx_1 * g2ynm5vjgu_idx_1 ) - g2ynm5vjgu_idx_2 *
g2ynm5vjgu_idx_2 ) + g2ynm5vjgu_idx_3 * g2ynm5vjgu_idx_3 ) ; if ( rtB .
p1zfkdy0ss > 1.0 ) { g2ynm5vjgu_idx_0 = 1.0 ; } else if ( rtB . p1zfkdy0ss <
- 1.0 ) { g2ynm5vjgu_idx_0 = - 1.0 ; } else { g2ynm5vjgu_idx_0 = rtB .
p1zfkdy0ss ; } rtB . omd4dozqfc [ 1 ] = muDoubleScalarAsin ( g2ynm5vjgu_idx_0
) ; g2ynm5vjgu_idx_0 = ( rtP . speedgain_Gain * imakcnimkl_idx_2 + rtP .
Constant2_Value ) * rtP . const_Gain - rtB . oqquwhnyjp [ 1 ] ; rtB .
kdhjnpyryt = rtP . Gain_Gain_nt13gxv4cf * g2ynm5vjgu_idx_0 ; rtB . aj3d0xadxg
= ( rtP . speedgain_Gain_cayz03alro * imakcnimkl_idx_0 + rtP .
Constant2_Value_bijw4dwdaf ) * rtP . const_Gain_nyshjvfom1 ; if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { } g2ynm5vjgu_idx_0 = rtB . aj3d0xadxg - rtB
. oqquwhnyjp [ 0 ] ; rtB . hl1tydtlp3 = rtP . Gain_Gain_nz2osdhexb *
g2ynm5vjgu_idx_0 ; rtB . je2xqylh2t = rtP . Gain_Gain_m3wdc4uyp3 *
imakcnimkl_idx_0 ; rtB . jcianyi4kp = rtP . Gain_Gain_crytncxhkk *
imakcnimkl_idx_2 ; rtB . o2kcntnybn [ 0 ] = rtB . je2xqylh2t ; rtB .
o2kcntnybn [ 1 ] = 0.0 ; rtB . o2kcntnybn [ 2 ] = 0.0 ; rtB . o2kcntnybn [ 3
] = 0.0 ; rtB . hqhvl0otti [ 0 ] = rtB . jcianyi4kp ; rtB . hqhvl0otti [ 1 ]
= 0.0 ; rtB . hqhvl0otti [ 2 ] = 0.0 ; rtB . hqhvl0otti [ 3 ] = 0.0 ;
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) {
NeslSimulationData * simulationData ; real_T time ; boolean_T tmp ; real_T
tmp_p [ 24 ] ; int_T tmp_e [ 7 ] ; NeuDiagnosticManager * diagnosticManager ;
NeuDiagnosticTree * diagnosticTree ; int32_T tmp_i ; char * msg ; { real_T *
* uBuffer = ( real_T * * ) & rtDW . g2lkbxgjhp . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & rtDW . g2lkbxgjhp . TUbufferPtrs [ 1 ] ;
real_T simTime = ssGetT ( rtS ) ; rtDW . j03obu1eim . Head = ( ( rtDW .
j03obu1eim . Head < ( rtDW . j03obu1eim . CircularBufSize - 1 ) ) ? ( rtDW .
j03obu1eim . Head + 1 ) : 0 ) ; if ( rtDW . j03obu1eim . Head == rtDW .
j03obu1eim . Tail ) { if ( ! rt_TDelayUpdateTailOrGrowBuf ( & rtDW .
j03obu1eim . CircularBufSize , & rtDW . j03obu1eim . Tail , & rtDW .
j03obu1eim . Head , & rtDW . j03obu1eim . Last , simTime - rtP . time_delay ,
tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & rtDW . j03obu1eim
. MaxNewBufSize ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ rtDW .
j03obu1eim . Head ] = simTime ; ( * uBuffer ) [ rtDW . j03obu1eim . Head ] =
rtB . oqquwhnyjp [ 2 ] ; } { real_T * * uBuffer = ( real_T * * ) & rtDW .
lizyfiedvc . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & rtDW
. lizyfiedvc . TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( rtS ) ; rtDW .
cadeui54l4 . Head = ( ( rtDW . cadeui54l4 . Head < ( rtDW . cadeui54l4 .
CircularBufSize - 1 ) ) ? ( rtDW . cadeui54l4 . Head + 1 ) : 0 ) ; if ( rtDW
. cadeui54l4 . Head == rtDW . cadeui54l4 . Tail ) { if ( !
rt_TDelayUpdateTailOrGrowBuf ( & rtDW . cadeui54l4 . CircularBufSize , & rtDW
. cadeui54l4 . Tail , & rtDW . cadeui54l4 . Head , & rtDW . cadeui54l4 . Last
, simTime - rtP . time_delay , tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0
, false , & rtDW . cadeui54l4 . MaxNewBufSize ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ rtDW .
cadeui54l4 . Head ] = simTime ; ( * uBuffer ) [ rtDW . cadeui54l4 . Head ] =
rtB . oqquwhnyjp [ 3 ] ; } { real_T * * uBuffer = ( real_T * * ) & rtDW .
mx22ld2hzj . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & rtDW
. mx22ld2hzj . TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( rtS ) ; rtDW .
dbj1dpcqc0 . Head = ( ( rtDW . dbj1dpcqc0 . Head < ( rtDW . dbj1dpcqc0 .
CircularBufSize - 1 ) ) ? ( rtDW . dbj1dpcqc0 . Head + 1 ) : 0 ) ; if ( rtDW
. dbj1dpcqc0 . Head == rtDW . dbj1dpcqc0 . Tail ) { if ( !
rt_TDelayUpdateTailOrGrowBuf ( & rtDW . dbj1dpcqc0 . CircularBufSize , & rtDW
. dbj1dpcqc0 . Tail , & rtDW . dbj1dpcqc0 . Head , & rtDW . dbj1dpcqc0 . Last
, simTime - rtP . time_delay , tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0
, false , & rtDW . dbj1dpcqc0 . MaxNewBufSize ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ rtDW .
dbj1dpcqc0 . Head ] = simTime ; ( * uBuffer ) [ rtDW . dbj1dpcqc0 . Head ] =
rtB . oqquwhnyjp [ 4 ] ; } { real_T * * uBuffer = ( real_T * * ) & rtDW .
diva5n50pk . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & rtDW
. diva5n50pk . TUbufferPtrs [ 3 ] ; real_T simTime = ssGetT ( rtS ) ; rtDW .
ko54z1uphk . Head [ 0 ] = ( ( rtDW . ko54z1uphk . Head [ 0 ] < ( rtDW .
ko54z1uphk . CircularBufSize [ 0 ] - 1 ) ) ? ( rtDW . ko54z1uphk . Head [ 0 ]
+ 1 ) : 0 ) ; if ( rtDW . ko54z1uphk . Head [ 0 ] == rtDW . ko54z1uphk . Tail
[ 0 ] ) { if ( ! rt_TDelayUpdateTailOrGrowBuf ( & rtDW . ko54z1uphk .
CircularBufSize [ 0 ] , & rtDW . ko54z1uphk . Tail [ 0 ] , & rtDW .
ko54z1uphk . Head [ 0 ] , & rtDW . ko54z1uphk . Last [ 0 ] , simTime - rtP .
time_delay , tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & rtDW
. ko54z1uphk . MaxNewBufSize ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ++ ) [ rtDW .
ko54z1uphk . Head [ 0 ] ] = simTime ; ( * uBuffer ++ ) [ rtDW . ko54z1uphk .
Head [ 0 ] ] = rtB . omd4dozqfc [ 0 ] ; rtDW . ko54z1uphk . Head [ 1 ] = ( (
rtDW . ko54z1uphk . Head [ 1 ] < ( rtDW . ko54z1uphk . CircularBufSize [ 1 ]
- 1 ) ) ? ( rtDW . ko54z1uphk . Head [ 1 ] + 1 ) : 0 ) ; if ( rtDW .
ko54z1uphk . Head [ 1 ] == rtDW . ko54z1uphk . Tail [ 1 ] ) { if ( !
rt_TDelayUpdateTailOrGrowBuf ( & rtDW . ko54z1uphk . CircularBufSize [ 1 ] ,
& rtDW . ko54z1uphk . Tail [ 1 ] , & rtDW . ko54z1uphk . Head [ 1 ] , & rtDW
. ko54z1uphk . Last [ 1 ] , simTime - rtP . time_delay , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & rtDW . ko54z1uphk . MaxNewBufSize ) )
{ ssSetErrorStatus ( rtS , "tdelay memory allocation error" ) ; return ; } }
( * tBuffer ++ ) [ rtDW . ko54z1uphk . Head [ 1 ] ] = simTime ; ( * uBuffer
++ ) [ rtDW . ko54z1uphk . Head [ 1 ] ] = rtB . omd4dozqfc [ 1 ] ; rtDW .
ko54z1uphk . Head [ 2 ] = ( ( rtDW . ko54z1uphk . Head [ 2 ] < ( rtDW .
ko54z1uphk . CircularBufSize [ 2 ] - 1 ) ) ? ( rtDW . ko54z1uphk . Head [ 2 ]
+ 1 ) : 0 ) ; if ( rtDW . ko54z1uphk . Head [ 2 ] == rtDW . ko54z1uphk . Tail
[ 2 ] ) { if ( ! rt_TDelayUpdateTailOrGrowBuf ( & rtDW . ko54z1uphk .
CircularBufSize [ 2 ] , & rtDW . ko54z1uphk . Tail [ 2 ] , & rtDW .
ko54z1uphk . Head [ 2 ] , & rtDW . ko54z1uphk . Last [ 2 ] , simTime - rtP .
time_delay , tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & rtDW
. ko54z1uphk . MaxNewBufSize ) ) { ssSetErrorStatus ( rtS ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ rtDW .
ko54z1uphk . Head [ 2 ] ] = simTime ; ( * uBuffer ) [ rtDW . ko54z1uphk .
Head [ 2 ] ] = rtB . omd4dozqfc [ 2 ] ; } simulationData = (
NeslSimulationData * ) rtDW . m2zz55qyqo ; time = ssGetT ( rtS ) ;
simulationData -> mData -> mTime . mN = 1 ; simulationData -> mData -> mTime
. mX = & time ; simulationData -> mData -> mContStates . mN = 61 ;
simulationData -> mData -> mContStates . mX = & rtX . fmofelvad4 [ 0 ] ;
simulationData -> mData -> mDiscStates . mN = 0 ; simulationData -> mData ->
mDiscStates . mX = & rtDW . ggqhsa10cs ; simulationData -> mData ->
mModeVector . mN = 0 ; simulationData -> mData -> mModeVector . mX = & rtDW .
j24ttftmn0 ; tmp = false ; simulationData -> mData -> mFoundZcEvents = tmp ;
simulationData -> mData -> mIsMajorTimeStep = ssIsMajorTimeStep ( rtS ) ; tmp
= ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags . solverAssertCheck == 1U ) ;
simulationData -> mData -> mIsSolverAssertCheck = tmp ; simulationData ->
mData -> mIsSolverCheckingCIC = false ; tmp = ssIsSolverComputingJacobian (
rtS ) ; simulationData -> mData -> mIsComputingJacobian = tmp ;
simulationData -> mData -> mIsEvaluatingF0 = ( ssGetEvaluatingF0ForJacobian (
rtS ) != 0 ) ; simulationData -> mData -> mIsSolverRequestingReset = false ;
tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB . eczwzi2yjw [ 0 ] ; tmp_p [ 1 ] = rtB .
eczwzi2yjw [ 1 ] ; tmp_p [ 2 ] = rtB . eczwzi2yjw [ 2 ] ; tmp_p [ 3 ] = rtB .
eczwzi2yjw [ 3 ] ; tmp_e [ 1 ] = 4 ; tmp_p [ 4 ] = rtB . gudikbezkv [ 0 ] ;
tmp_p [ 5 ] = rtB . gudikbezkv [ 1 ] ; tmp_p [ 6 ] = rtB . gudikbezkv [ 2 ] ;
tmp_p [ 7 ] = rtB . gudikbezkv [ 3 ] ; tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB .
nxogfkpljw [ 0 ] ; tmp_p [ 9 ] = rtB . nxogfkpljw [ 1 ] ; tmp_p [ 10 ] = rtB
. nxogfkpljw [ 2 ] ; tmp_p [ 11 ] = rtB . nxogfkpljw [ 3 ] ; tmp_e [ 3 ] = 12
; tmp_p [ 12 ] = rtB . e4vcd3rbgn [ 0 ] ; tmp_p [ 13 ] = rtB . e4vcd3rbgn [ 1
] ; tmp_p [ 14 ] = rtB . e4vcd3rbgn [ 2 ] ; tmp_p [ 15 ] = rtB . e4vcd3rbgn [
3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] = rtB . o2kcntnybn [ 0 ] ; tmp_p [ 17 ]
= rtB . o2kcntnybn [ 1 ] ; tmp_p [ 18 ] = rtB . o2kcntnybn [ 2 ] ; tmp_p [ 19
] = rtB . o2kcntnybn [ 3 ] ; tmp_e [ 5 ] = 20 ; tmp_p [ 20 ] = rtB .
hqhvl0otti [ 0 ] ; tmp_p [ 21 ] = rtB . hqhvl0otti [ 1 ] ; tmp_p [ 22 ] = rtB
. hqhvl0otti [ 2 ] ; tmp_p [ 23 ] = rtB . hqhvl0otti [ 3 ] ; tmp_e [ 6 ] = 24
; simulationData -> mData -> mInputValues . mN = 24 ; simulationData -> mData
-> mInputValues . mX = & tmp_p [ 0 ] ; simulationData -> mData ->
mInputOffsets . mN = 7 ; simulationData -> mData -> mInputOffsets . mX = &
tmp_e [ 0 ] ; diagnosticManager = ( NeuDiagnosticManager * ) rtDW .
nbicecjjvm ; diagnosticTree = neu_diagnostic_manager_get_initial_tree (
diagnosticManager ) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW
. edvzy5545y , NESL_SIM_UPDATE , simulationData , diagnosticManager ) ; if (
tmp_i != 0 ) { tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if
( tmp ) { msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus (
rtS , msg ) ; } } UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 ( int_T tid
) { UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) {
NeslSimulationData * simulationData ; real_T time ; boolean_T tmp ; real_T
tmp_p [ 24 ] ; int_T tmp_e [ 7 ] ; NeuDiagnosticManager * diagnosticManager ;
NeuDiagnosticTree * diagnosticTree ; int32_T tmp_i ; char * msg ; XDot *
_rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; _rtXdot -> idxnpi20k1 =
rtB . eslkbwnlvg ; _rtXdot -> f4xemzmgtv = rtB . m1dkcmbbey ; _rtXdot ->
bj5404lz5d = rtB . ms5qo0q32e ; _rtXdot -> az1z4d4mub = rtB . onyodjbxop ;
_rtXdot -> i44qjsxqq3 = rtB . lgntheub31 ; _rtXdot -> fwjq3tjwfs = rtB .
ibl1bvoot5 ; _rtXdot -> cucedlakmc = rtB . e04tk5zbbe ; _rtXdot -> lq4alwhpwv
= rtB . ozxva0egn1 ; _rtXdot -> lryf2ntwdb [ 0 ] = rtX . lryf2ntwdb [ 1 ] ;
_rtXdot -> lryf2ntwdb [ 1 ] = ( ( rtB . agzdgrfhap - rtX . lryf2ntwdb [ 0 ] )
* 1000.0 - 2.0 * rtX . lryf2ntwdb [ 1 ] ) * 1000.0 ; _rtXdot -> px5f4y1l5a [
0 ] = rtX . px5f4y1l5a [ 1 ] ; _rtXdot -> px5f4y1l5a [ 1 ] = ( ( rtB .
k4jt1nr2ej - rtX . px5f4y1l5a [ 0 ] ) * 1000.0 - 2.0 * rtX . px5f4y1l5a [ 1 ]
) * 1000.0 ; _rtXdot -> mwtxdy2wl1 = rtB . hl1tydtlp3 ; _rtXdot -> fbwrmyrd20
[ 0 ] = rtX . fbwrmyrd20 [ 1 ] ; _rtXdot -> fbwrmyrd20 [ 1 ] = ( ( rtB .
b2gk3wukry - rtX . fbwrmyrd20 [ 0 ] ) * 1000.0 - 2.0 * rtX . fbwrmyrd20 [ 1 ]
) * 1000.0 ; _rtXdot -> lu1xqtjac1 = rtB . kdhjnpyryt ; _rtXdot -> bl2jw4bwps
[ 0 ] = rtX . bl2jw4bwps [ 1 ] ; _rtXdot -> bl2jw4bwps [ 1 ] = ( ( rtB .
gipodijd1l - rtX . bl2jw4bwps [ 0 ] ) * 1000.0 - 2.0 * rtX . bl2jw4bwps [ 1 ]
) * 1000.0 ; simulationData = ( NeslSimulationData * ) rtDW . m2zz55qyqo ;
time = ssGetT ( rtS ) ; simulationData -> mData -> mTime . mN = 1 ;
simulationData -> mData -> mTime . mX = & time ; simulationData -> mData ->
mContStates . mN = 61 ; simulationData -> mData -> mContStates . mX = & rtX .
fmofelvad4 [ 0 ] ; simulationData -> mData -> mDiscStates . mN = 0 ;
simulationData -> mData -> mDiscStates . mX = & rtDW . ggqhsa10cs ;
simulationData -> mData -> mModeVector . mN = 0 ; simulationData -> mData ->
mModeVector . mX = & rtDW . j24ttftmn0 ; tmp = false ; simulationData ->
mData -> mFoundZcEvents = tmp ; simulationData -> mData -> mIsMajorTimeStep =
ssIsMajorTimeStep ( rtS ) ; tmp = ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags .
solverAssertCheck == 1U ) ; simulationData -> mData -> mIsSolverAssertCheck =
tmp ; simulationData -> mData -> mIsSolverCheckingCIC = false ; tmp =
ssIsSolverComputingJacobian ( rtS ) ; simulationData -> mData ->
mIsComputingJacobian = tmp ; simulationData -> mData -> mIsEvaluatingF0 = (
ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ; simulationData -> mData ->
mIsSolverRequestingReset = false ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
eczwzi2yjw [ 0 ] ; tmp_p [ 1 ] = rtB . eczwzi2yjw [ 1 ] ; tmp_p [ 2 ] = rtB .
eczwzi2yjw [ 2 ] ; tmp_p [ 3 ] = rtB . eczwzi2yjw [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . gudikbezkv [ 0 ] ; tmp_p [ 5 ] = rtB . gudikbezkv [ 1 ] ;
tmp_p [ 6 ] = rtB . gudikbezkv [ 2 ] ; tmp_p [ 7 ] = rtB . gudikbezkv [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . nxogfkpljw [ 0 ] ; tmp_p [ 9 ] = rtB .
nxogfkpljw [ 1 ] ; tmp_p [ 10 ] = rtB . nxogfkpljw [ 2 ] ; tmp_p [ 11 ] = rtB
. nxogfkpljw [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . e4vcd3rbgn [ 0 ]
; tmp_p [ 13 ] = rtB . e4vcd3rbgn [ 1 ] ; tmp_p [ 14 ] = rtB . e4vcd3rbgn [ 2
] ; tmp_p [ 15 ] = rtB . e4vcd3rbgn [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . o2kcntnybn [ 0 ] ; tmp_p [ 17 ] = rtB . o2kcntnybn [ 1 ] ; tmp_p [ 18 ]
= rtB . o2kcntnybn [ 2 ] ; tmp_p [ 19 ] = rtB . o2kcntnybn [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . hqhvl0otti [ 0 ] ; tmp_p [ 21 ] = rtB .
hqhvl0otti [ 1 ] ; tmp_p [ 22 ] = rtB . hqhvl0otti [ 2 ] ; tmp_p [ 23 ] = rtB
. hqhvl0otti [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
simulationData -> mData -> mDx . mN = 61 ; simulationData -> mData -> mDx .
mX = & _rtXdot -> fmofelvad4 [ 0 ] ; diagnosticManager = (
NeuDiagnosticManager * ) rtDW . nbicecjjvm ; diagnosticTree =
neu_diagnostic_manager_get_initial_tree ( diagnosticManager ) ; tmp_i =
ne_simulator_method ( ( NeslSimulator * ) rtDW . edvzy5545y ,
NESL_SIM_DERIVATIVES , simulationData , diagnosticManager ) ; if ( tmp_i != 0
) { tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) {
msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg )
; } } } void MdlProjection ( void ) { NeslSimulationData * simulationData ;
real_T time ; boolean_T tmp ; real_T tmp_p [ 24 ] ; int_T tmp_e [ 7 ] ;
NeuDiagnosticManager * diagnosticManager ; NeuDiagnosticTree * diagnosticTree
; int32_T tmp_i ; char * msg ; simulationData = ( NeslSimulationData * ) rtDW
. m2zz55qyqo ; time = ssGetT ( rtS ) ; simulationData -> mData -> mTime . mN
= 1 ; simulationData -> mData -> mTime . mX = & time ; simulationData ->
mData -> mContStates . mN = 61 ; simulationData -> mData -> mContStates . mX
= & rtX . fmofelvad4 [ 0 ] ; simulationData -> mData -> mDiscStates . mN = 0
; simulationData -> mData -> mDiscStates . mX = & rtDW . ggqhsa10cs ;
simulationData -> mData -> mModeVector . mN = 0 ; simulationData -> mData ->
mModeVector . mX = & rtDW . j24ttftmn0 ; tmp = false ; simulationData ->
mData -> mFoundZcEvents = tmp ; simulationData -> mData -> mIsMajorTimeStep =
ssIsMajorTimeStep ( rtS ) ; tmp = ( ssGetMdlInfoPtr ( rtS ) -> mdlFlags .
solverAssertCheck == 1U ) ; simulationData -> mData -> mIsSolverAssertCheck =
tmp ; simulationData -> mData -> mIsSolverCheckingCIC = false ; tmp =
ssIsSolverComputingJacobian ( rtS ) ; simulationData -> mData ->
mIsComputingJacobian = tmp ; simulationData -> mData -> mIsEvaluatingF0 = (
ssGetEvaluatingF0ForJacobian ( rtS ) != 0 ) ; simulationData -> mData ->
mIsSolverRequestingReset = false ; tmp_e [ 0 ] = 0 ; tmp_p [ 0 ] = rtB .
eczwzi2yjw [ 0 ] ; tmp_p [ 1 ] = rtB . eczwzi2yjw [ 1 ] ; tmp_p [ 2 ] = rtB .
eczwzi2yjw [ 2 ] ; tmp_p [ 3 ] = rtB . eczwzi2yjw [ 3 ] ; tmp_e [ 1 ] = 4 ;
tmp_p [ 4 ] = rtB . gudikbezkv [ 0 ] ; tmp_p [ 5 ] = rtB . gudikbezkv [ 1 ] ;
tmp_p [ 6 ] = rtB . gudikbezkv [ 2 ] ; tmp_p [ 7 ] = rtB . gudikbezkv [ 3 ] ;
tmp_e [ 2 ] = 8 ; tmp_p [ 8 ] = rtB . nxogfkpljw [ 0 ] ; tmp_p [ 9 ] = rtB .
nxogfkpljw [ 1 ] ; tmp_p [ 10 ] = rtB . nxogfkpljw [ 2 ] ; tmp_p [ 11 ] = rtB
. nxogfkpljw [ 3 ] ; tmp_e [ 3 ] = 12 ; tmp_p [ 12 ] = rtB . e4vcd3rbgn [ 0 ]
; tmp_p [ 13 ] = rtB . e4vcd3rbgn [ 1 ] ; tmp_p [ 14 ] = rtB . e4vcd3rbgn [ 2
] ; tmp_p [ 15 ] = rtB . e4vcd3rbgn [ 3 ] ; tmp_e [ 4 ] = 16 ; tmp_p [ 16 ] =
rtB . o2kcntnybn [ 0 ] ; tmp_p [ 17 ] = rtB . o2kcntnybn [ 1 ] ; tmp_p [ 18 ]
= rtB . o2kcntnybn [ 2 ] ; tmp_p [ 19 ] = rtB . o2kcntnybn [ 3 ] ; tmp_e [ 5
] = 20 ; tmp_p [ 20 ] = rtB . hqhvl0otti [ 0 ] ; tmp_p [ 21 ] = rtB .
hqhvl0otti [ 1 ] ; tmp_p [ 22 ] = rtB . hqhvl0otti [ 2 ] ; tmp_p [ 23 ] = rtB
. hqhvl0otti [ 3 ] ; tmp_e [ 6 ] = 24 ; simulationData -> mData ->
mInputValues . mN = 24 ; simulationData -> mData -> mInputValues . mX = &
tmp_p [ 0 ] ; simulationData -> mData -> mInputOffsets . mN = 7 ;
simulationData -> mData -> mInputOffsets . mX = & tmp_e [ 0 ] ;
diagnosticManager = ( NeuDiagnosticManager * ) rtDW . nbicecjjvm ;
diagnosticTree = neu_diagnostic_manager_get_initial_tree ( diagnosticManager
) ; tmp_i = ne_simulator_method ( ( NeslSimulator * ) rtDW . edvzy5545y ,
NESL_SIM_PROJECTION , simulationData , diagnosticManager ) ; if ( tmp_i != 0
) { tmp = error_buffer_is_empty ( ssGetErrorStatus ( rtS ) ) ; if ( tmp ) {
msg = rtw_diagnostics_msg ( diagnosticTree ) ; ssSetErrorStatus ( rtS , msg )
; } } } void MdlTerminate ( void ) { rt_TDelayFreeBuf ( rtDW . g2lkbxgjhp .
TUbufferPtrs [ 0 ] ) ; rt_TDelayFreeBuf ( rtDW . lizyfiedvc . TUbufferPtrs [
0 ] ) ; rt_TDelayFreeBuf ( rtDW . mx22ld2hzj . TUbufferPtrs [ 0 ] ) ; { int_T
i1 ; void * * pw_TUbufferPtrs = & rtDW . diva5n50pk . TUbufferPtrs [ 0 ] ;
for ( i1 = 0 ; i1 < 3 ; i1 ++ ) { rt_TDelayFreeBuf ( pw_TUbufferPtrs [ i1 ] )
; } } neu_destroy_diagnostic_manager ( ( NeuDiagnosticManager * ) rtDW .
nbicecjjvm ) ; nesl_destroy_simulation_data ( ( NeslSimulationData * ) rtDW .
m2zz55qyqo ) ; nesl_erase_simulator (
"Control_Bicopter/Subsystem/Solver Configuration_1" ) ;
neu_destroy_diagnostic_manager ( ( NeuDiagnosticManager * ) rtDW . hzasxj2szj
) ; nesl_destroy_simulation_data ( ( NeslSimulationData * ) rtDW . b1ivukxskp
) ; nesl_erase_simulator (
"Control_Bicopter/Subsystem/Solver Configuration_1" ) ; if ( rt_slioCatalogue
( ) != ( NULL ) ) { void * * slioCatalogueAddr = rt_slioCatalogueAddr ( ) ;
rtwSaveDatasetsToMatFile ( rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( )
) , rt_GetMatSigstreamLoggingFileName ( ) ) ; rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = NULL ; } } void
MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 79 ) ;
ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 6 ) ; ssSetNumU (
rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ; ssSetNumSampleTimes ( rtS ,
2 ) ; ssSetNumBlocks ( rtS , 340 ) ; ssSetNumBlockIO ( rtS , 31 ) ;
ssSetNumBlockParams ( rtS , 62 ) ; } void MdlInitializeSampleTimes ( void ) {
ssSetSampleTime ( rtS , 0 , 0.0 ) ; ssSetSampleTime ( rtS , 1 , 0.001 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 0.0 ) ; }
void raccel_set_checksum ( ) { ssSetChecksumVal ( rtS , 0 , 2369118705U ) ;
ssSetChecksumVal ( rtS , 1 , 162466512U ) ; ssSetChecksumVal ( rtS , 2 ,
3416941678U ) ; ssSetChecksumVal ( rtS , 3 , 3742909486U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( void ) { static struct _ssMdlInfo mdlInfo
; ( void ) memset ( ( char * ) rtS , 0 , sizeof ( SimStruct ) ) ; ( void )
memset ( ( char * ) & mdlInfo , 0 , sizeof ( struct _ssMdlInfo ) ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; } { ssSetY ( rtS , & rtY ) ; ( void ) memset ( ( void * ) & rtY , 0
, sizeof ( ExtY ) ) ; } { real_T * x = ( real_T * ) & rtX ; ssSetContStates (
rtS , x ) ; ( void ) memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void *
dwork = ( void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset
( dwork , 0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo dtInfo ; ( void
) memset ( ( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ;
ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo . numDataTypes = 14 ;
dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = &
rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = & rtBTransTable ; dtInfo .
PTransTable = & rtPTransTable ; dtInfo . dataTypeInfoTable =
rtDataTypeInfoTable ; } Control_Bicopter_InitializeDataMapInfo ( ) ;
ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ;
ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"Control_Bicopter" ) ; ssSetPath ( rtS , "Control_Bicopter" ) ; ssSetTStart (
rtS , 0.0 ) ; ssSetTFinal ( rtS , 5.0 ) ; ssSetStepSize ( rtS , 0.001 ) ;
ssSetFixedStepSize ( rtS , 0.001 ) ; { static RTWLogInfo rt_DataLoggingInfo ;
rt_DataLoggingInfo . loggingInterval = NULL ; ssSetRTWLogInfo ( rtS , &
rt_DataLoggingInfo ) ; } { { static int_T rt_LoggedStateWidths [ ] = { 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
4 , 1 , 1 , 1 , 3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 2 , 2 } ; static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 } ; static int_T rt_LoggedStateDimensions [ ] = { 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 4 , 1 ,
1 , 1 , 3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 2 , 2 } ; static boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 } ; static BuiltInDTypeId rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static
RTWPreprocessingFcnPtr rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) ,
( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) } ; static
const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "Discrete" , "FirstOutput" , "Discrete" , "FirstOutput" ,
"Discrete" , "FirstOutput" , "Discrete" , "FirstOutput" , "Discrete" ,
"Discrete" } ; static const char_T * rt_LoggedStateBlockNames [ ] = {
"Control_Bicopter/PID_Controller/PID Phi/Integrator/Continuous/Integrator" ,
"Control_Bicopter/PID_Controller/PID Phi/Filter/Cont. Filter/Filter" ,
"Control_Bicopter/PID_Controller/PID Z/Integrator/Continuous/Integrator" ,
"Control_Bicopter/PID_Controller/PID Z/Filter/Cont. Filter/Filter" ,
"Control_Bicopter/PID_Controller/PID Theta/Integrator/Continuous/Integrator"
, "Control_Bicopter/PID_Controller/PID Theta/Filter/Cont. Filter/Filter" ,
"Control_Bicopter/PID_Controller/PID Psi/Integrator/Continuous/Integrator" ,
"Control_Bicopter/PID_Controller/PID Psi/Filter/Cont. Filter/Filter" ,
"Control_Bicopter/Subsystem/Simulink-PS\nConverter4" ,
"Control_Bicopter/Subsystem/Simulink-PS\nConverter4" ,
"Control_Bicopter/Subsystem/Simulink-PS\nConverter1" ,
"Control_Bicopter/Subsystem/Simulink-PS\nConverter1" ,
"Control_Bicopter/Subsystem/torque for propeller1/Integrator" ,
"Control_Bicopter/Subsystem/torque for propeller1/Simulink-PS\nConverter3" ,
"Control_Bicopter/Subsystem/torque for propeller1/Simulink-PS\nConverter3" ,
"Control_Bicopter/Subsystem/torque for propeller/Integrator" ,
"Control_Bicopter/Subsystem/torque for propeller/Simulink-PS\nConverter3" ,
"Control_Bicopter/Subsystem/torque for propeller/Simulink-PS\nConverter3" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/Cylindrical1" ,
"Control_Bicopter/Subsystem/Cylindrical1" ,
"Control_Bicopter/Subsystem/Cylindrical1" ,
"Control_Bicopter/Subsystem/Cylindrical1" ,
"Control_Bicopter/Subsystem/Cylindrical3" ,
"Control_Bicopter/Subsystem/Cylindrical3" ,
"Control_Bicopter/Subsystem/Cylindrical3" ,
"Control_Bicopter/Subsystem/Cylindrical3" ,
"Control_Bicopter/Subsystem/Prismatic" ,
"Control_Bicopter/Subsystem/Prismatic" ,
"Control_Bicopter/Subsystem/Cylindrical2" ,
"Control_Bicopter/Subsystem/Cylindrical2" ,
"Control_Bicopter/Subsystem/Cylindrical2" ,
"Control_Bicopter/Subsystem/Cylindrical2" ,
"Control_Bicopter/Subsystem/Cylindrical4" ,
"Control_Bicopter/Subsystem/Cylindrical4" ,
"Control_Bicopter/Subsystem/Cylindrical4" ,
"Control_Bicopter/Subsystem/Cylindrical4" ,
"Control_Bicopter/Subsystem/Prismatic1" ,
"Control_Bicopter/Subsystem/Prismatic1" ,
"Control_Bicopter/Subsystem/Prismatic2" ,
"Control_Bicopter/Subsystem/Prismatic2" ,
"Control_Bicopter/Subsystem/Prismatic3" ,
"Control_Bicopter/Subsystem/Prismatic3" ,
"Control_Bicopter/Subsystem/Revolute2" ,
"Control_Bicopter/Subsystem/Revolute2" ,
"Control_Bicopter/Subsystem/Revolute3" ,
"Control_Bicopter/Subsystem/Revolute3" ,
"Control_Bicopter/Subsystem/Revolute4" ,
"Control_Bicopter/Subsystem/Revolute4" ,
"Control_Bicopter/Subsystem/Revolute5" ,
"Control_Bicopter/Subsystem/Revolute5" , "Control_Bicopter/Subsystem/Planar1"
, "Control_Bicopter/Subsystem/Planar1" ,
"Control_Bicopter/Subsystem/Cylindrical" ,
"Control_Bicopter/Subsystem/Cylindrical" ,
"Control_Bicopter/Subsystem/Planar" , "Control_Bicopter/Subsystem/Planar" ,
"Control_Bicopter/Subsystem/Planar4" , "Control_Bicopter/Subsystem/Planar4" ,
"Control_Bicopter/Subsystem/Planar3" , "Control_Bicopter/Subsystem/Planar3" ,
"Control_Bicopter/Subsystem/Planar2" , "Control_Bicopter/Subsystem/Planar2" ,
"Control_Bicopter/Subsystem/Planar5" , "Control_Bicopter/Subsystem/Planar5" ,
"Control_Bicopter/Subsystem/Planar6" , "Control_Bicopter/Subsystem/Planar6" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_2_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_2_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_1_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_1_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_4_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_3_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_3_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_5_1_1" ,
"Control_Bicopter/Subsystem/Solver\nConfiguration/EVAL_KEY/INPUT_6_1_1" } ;
static const char_T * rt_LoggedStateNames [ ] = { "" , "" , "" , "" , "" , ""
, "" , "" ,
"Control_Bicopter.Subsystem.Simulink_PS_Converter4.outputFiltered_4174348068_0"
,
"Control_Bicopter.Subsystem.Simulink_PS_Converter4.outputFiltered_4174348068_1"
,
"Control_Bicopter.Subsystem.Simulink_PS_Converter1.outputFiltered_3211724276_0"
,
"Control_Bicopter.Subsystem.Simulink_PS_Converter1.outputFiltered_3211724276_1"
, "" ,
 "Control_Bicopter.Subsystem.torque_for_propeller1.Simulink_PS_Converter3.outputFiltered_2005895812_0"
,
 "Control_Bicopter.Subsystem.torque_for_propeller1.Simulink_PS_Converter3.outputFiltered_2005895812_1"
, "" ,
 "Control_Bicopter.Subsystem.torque_for_propeller.Simulink_PS_Converter3.outputFiltered_3316602516_0"
,
 "Control_Bicopter.Subsystem.torque_for_propeller.Simulink_PS_Converter3.outputFiltered_3316602516_1"
, "Control_Bicopter.Subsystem.SixDOF.Px.p" ,
"Control_Bicopter.Subsystem.SixDOF.Py.p" ,
"Control_Bicopter.Subsystem.SixDOF.Pz.p" ,
"Control_Bicopter.Subsystem.SixDOF.S.Q" ,
"Control_Bicopter.Subsystem.SixDOF.Px.v" ,
"Control_Bicopter.Subsystem.SixDOF.Py.v" ,
"Control_Bicopter.Subsystem.SixDOF.Pz.v" ,
"Control_Bicopter.Subsystem.SixDOF.S.w" ,
"Control_Bicopter.Subsystem.Cylindrical1.Rz.q" ,
"Control_Bicopter.Subsystem.Cylindrical1.Pz.p" ,
"Control_Bicopter.Subsystem.Cylindrical1.Rz.w" ,
"Control_Bicopter.Subsystem.Cylindrical1.Pz.v" ,
"Control_Bicopter.Subsystem.Cylindrical3.Rz.q" ,
"Control_Bicopter.Subsystem.Cylindrical3.Pz.p" ,
"Control_Bicopter.Subsystem.Cylindrical3.Rz.w" ,
"Control_Bicopter.Subsystem.Cylindrical3.Pz.v" ,
"Control_Bicopter.Subsystem.Prismatic.Pz.p" ,
"Control_Bicopter.Subsystem.Prismatic.Pz.v" ,
"Control_Bicopter.Subsystem.Cylindrical2.Rz.q" ,
"Control_Bicopter.Subsystem.Cylindrical2.Pz.p" ,
"Control_Bicopter.Subsystem.Cylindrical2.Rz.w" ,
"Control_Bicopter.Subsystem.Cylindrical2.Pz.v" ,
"Control_Bicopter.Subsystem.Cylindrical4.Rz.q" ,
"Control_Bicopter.Subsystem.Cylindrical4.Pz.p" ,
"Control_Bicopter.Subsystem.Cylindrical4.Rz.w" ,
"Control_Bicopter.Subsystem.Cylindrical4.Pz.v" ,
"Control_Bicopter.Subsystem.Prismatic1.Pz.p" ,
"Control_Bicopter.Subsystem.Prismatic1.Pz.v" ,
"Control_Bicopter.Subsystem.Prismatic2.Pz.p" ,
"Control_Bicopter.Subsystem.Prismatic2.Pz.v" ,
"Control_Bicopter.Subsystem.Prismatic3.Pz.p" ,
"Control_Bicopter.Subsystem.Prismatic3.Pz.v" ,
"Control_Bicopter.Subsystem.Revolute2.Rz.q" ,
"Control_Bicopter.Subsystem.Revolute2.Rz.w" ,
"Control_Bicopter.Subsystem.Revolute3.Rz.q" ,
"Control_Bicopter.Subsystem.Revolute3.Rz.w" ,
"Control_Bicopter.Subsystem.Revolute4.Rz.q" ,
"Control_Bicopter.Subsystem.Revolute4.Rz.w" ,
"Control_Bicopter.Subsystem.Revolute5.Rz.q" ,
"Control_Bicopter.Subsystem.Revolute5.Rz.w" ,
"Control_Bicopter.Subsystem.Planar1.Rz.q" ,
"Control_Bicopter.Subsystem.Planar1.Rz.w" ,
"Control_Bicopter.Subsystem.Cylindrical.Rz.q" ,
"Control_Bicopter.Subsystem.Cylindrical.Rz.w" ,
"Control_Bicopter.Subsystem.Planar.Rz.q" ,
"Control_Bicopter.Subsystem.Planar.Rz.w" ,
"Control_Bicopter.Subsystem.Planar4.Rz.q" ,
"Control_Bicopter.Subsystem.Planar4.Rz.w" ,
"Control_Bicopter.Subsystem.Planar3.Rz.q" ,
"Control_Bicopter.Subsystem.Planar3.Rz.w" ,
"Control_Bicopter.Subsystem.Planar2.Rz.q" ,
"Control_Bicopter.Subsystem.Planar2.Rz.w" ,
"Control_Bicopter.Subsystem.Planar5.Rz.q" ,
"Control_Bicopter.Subsystem.Planar5.Rz.w" ,
"Control_Bicopter.Subsystem.Planar6.Rz.q" ,
"Control_Bicopter.Subsystem.Planar6.Rz.w" , "" , "" , "" , "" , "" , "" , ""
, "" , "" , "" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
0 , 0 , 0 , 0 } ; static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] =
{ { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE
, SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 ,
0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } } ; static RTWLogSignalInfo rt_LoggedStateSignalInfo = { 84 ,
rt_LoggedStateWidths , rt_LoggedStateNumDimensions , rt_LoggedStateDimensions
, rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 84 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . idxnpi20k1 ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . f4xemzmgtv ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . bj5404lz5d ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . az1z4d4mub ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . i44qjsxqq3 ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . fwjq3tjwfs ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . cucedlakmc ;
rt_LoggedStateSignalPtrs [ 7 ] = ( void * ) & rtX . lq4alwhpwv ;
rt_LoggedStateSignalPtrs [ 8 ] = ( void * ) & rtX . lryf2ntwdb [ 0 ] ;
rt_LoggedStateSignalPtrs [ 9 ] = ( void * ) & rtX . lryf2ntwdb [ 1 ] ;
rt_LoggedStateSignalPtrs [ 10 ] = ( void * ) & rtX . px5f4y1l5a [ 0 ] ;
rt_LoggedStateSignalPtrs [ 11 ] = ( void * ) & rtX . px5f4y1l5a [ 1 ] ;
rt_LoggedStateSignalPtrs [ 12 ] = ( void * ) & rtX . mwtxdy2wl1 ;
rt_LoggedStateSignalPtrs [ 13 ] = ( void * ) & rtX . fbwrmyrd20 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 14 ] = ( void * ) & rtX . fbwrmyrd20 [ 1 ] ;
rt_LoggedStateSignalPtrs [ 15 ] = ( void * ) & rtX . lu1xqtjac1 ;
rt_LoggedStateSignalPtrs [ 16 ] = ( void * ) & rtX . bl2jw4bwps [ 0 ] ;
rt_LoggedStateSignalPtrs [ 17 ] = ( void * ) & rtX . bl2jw4bwps [ 1 ] ;
rt_LoggedStateSignalPtrs [ 18 ] = ( void * ) & rtX . fmofelvad4 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 19 ] = ( void * ) & rtX . fmofelvad4 [ 1 ] ;
rt_LoggedStateSignalPtrs [ 20 ] = ( void * ) & rtX . fmofelvad4 [ 2 ] ;
rt_LoggedStateSignalPtrs [ 21 ] = ( void * ) & rtX . fmofelvad4 [ 3 ] ;
rt_LoggedStateSignalPtrs [ 22 ] = ( void * ) & rtX . fmofelvad4 [ 7 ] ;
rt_LoggedStateSignalPtrs [ 23 ] = ( void * ) & rtX . fmofelvad4 [ 8 ] ;
rt_LoggedStateSignalPtrs [ 24 ] = ( void * ) & rtX . fmofelvad4 [ 9 ] ;
rt_LoggedStateSignalPtrs [ 25 ] = ( void * ) & rtX . fmofelvad4 [ 10 ] ;
rt_LoggedStateSignalPtrs [ 26 ] = ( void * ) & rtX . fmofelvad4 [ 13 ] ;
rt_LoggedStateSignalPtrs [ 27 ] = ( void * ) & rtX . fmofelvad4 [ 14 ] ;
rt_LoggedStateSignalPtrs [ 28 ] = ( void * ) & rtX . fmofelvad4 [ 15 ] ;
rt_LoggedStateSignalPtrs [ 29 ] = ( void * ) & rtX . fmofelvad4 [ 16 ] ;
rt_LoggedStateSignalPtrs [ 30 ] = ( void * ) & rtX . fmofelvad4 [ 17 ] ;
rt_LoggedStateSignalPtrs [ 31 ] = ( void * ) & rtX . fmofelvad4 [ 18 ] ;
rt_LoggedStateSignalPtrs [ 32 ] = ( void * ) & rtX . fmofelvad4 [ 19 ] ;
rt_LoggedStateSignalPtrs [ 33 ] = ( void * ) & rtX . fmofelvad4 [ 20 ] ;
rt_LoggedStateSignalPtrs [ 34 ] = ( void * ) & rtX . fmofelvad4 [ 21 ] ;
rt_LoggedStateSignalPtrs [ 35 ] = ( void * ) & rtX . fmofelvad4 [ 22 ] ;
rt_LoggedStateSignalPtrs [ 36 ] = ( void * ) & rtX . fmofelvad4 [ 23 ] ;
rt_LoggedStateSignalPtrs [ 37 ] = ( void * ) & rtX . fmofelvad4 [ 24 ] ;
rt_LoggedStateSignalPtrs [ 38 ] = ( void * ) & rtX . fmofelvad4 [ 25 ] ;
rt_LoggedStateSignalPtrs [ 39 ] = ( void * ) & rtX . fmofelvad4 [ 26 ] ;
rt_LoggedStateSignalPtrs [ 40 ] = ( void * ) & rtX . fmofelvad4 [ 27 ] ;
rt_LoggedStateSignalPtrs [ 41 ] = ( void * ) & rtX . fmofelvad4 [ 28 ] ;
rt_LoggedStateSignalPtrs [ 42 ] = ( void * ) & rtX . fmofelvad4 [ 29 ] ;
rt_LoggedStateSignalPtrs [ 43 ] = ( void * ) & rtX . fmofelvad4 [ 30 ] ;
rt_LoggedStateSignalPtrs [ 44 ] = ( void * ) & rtX . fmofelvad4 [ 31 ] ;
rt_LoggedStateSignalPtrs [ 45 ] = ( void * ) & rtX . fmofelvad4 [ 32 ] ;
rt_LoggedStateSignalPtrs [ 46 ] = ( void * ) & rtX . fmofelvad4 [ 33 ] ;
rt_LoggedStateSignalPtrs [ 47 ] = ( void * ) & rtX . fmofelvad4 [ 34 ] ;
rt_LoggedStateSignalPtrs [ 48 ] = ( void * ) & rtX . fmofelvad4 [ 35 ] ;
rt_LoggedStateSignalPtrs [ 49 ] = ( void * ) & rtX . fmofelvad4 [ 36 ] ;
rt_LoggedStateSignalPtrs [ 50 ] = ( void * ) & rtX . fmofelvad4 [ 37 ] ;
rt_LoggedStateSignalPtrs [ 51 ] = ( void * ) & rtX . fmofelvad4 [ 38 ] ;
rt_LoggedStateSignalPtrs [ 52 ] = ( void * ) & rtX . fmofelvad4 [ 39 ] ;
rt_LoggedStateSignalPtrs [ 53 ] = ( void * ) & rtX . fmofelvad4 [ 40 ] ;
rt_LoggedStateSignalPtrs [ 54 ] = ( void * ) & rtX . fmofelvad4 [ 41 ] ;
rt_LoggedStateSignalPtrs [ 55 ] = ( void * ) & rtX . fmofelvad4 [ 42 ] ;
rt_LoggedStateSignalPtrs [ 56 ] = ( void * ) & rtX . fmofelvad4 [ 43 ] ;
rt_LoggedStateSignalPtrs [ 57 ] = ( void * ) & rtX . fmofelvad4 [ 44 ] ;
rt_LoggedStateSignalPtrs [ 58 ] = ( void * ) & rtX . fmofelvad4 [ 45 ] ;
rt_LoggedStateSignalPtrs [ 59 ] = ( void * ) & rtX . fmofelvad4 [ 46 ] ;
rt_LoggedStateSignalPtrs [ 60 ] = ( void * ) & rtX . fmofelvad4 [ 47 ] ;
rt_LoggedStateSignalPtrs [ 61 ] = ( void * ) & rtX . fmofelvad4 [ 48 ] ;
rt_LoggedStateSignalPtrs [ 62 ] = ( void * ) & rtX . fmofelvad4 [ 49 ] ;
rt_LoggedStateSignalPtrs [ 63 ] = ( void * ) & rtX . fmofelvad4 [ 50 ] ;
rt_LoggedStateSignalPtrs [ 64 ] = ( void * ) & rtX . fmofelvad4 [ 51 ] ;
rt_LoggedStateSignalPtrs [ 65 ] = ( void * ) & rtX . fmofelvad4 [ 52 ] ;
rt_LoggedStateSignalPtrs [ 66 ] = ( void * ) & rtX . fmofelvad4 [ 53 ] ;
rt_LoggedStateSignalPtrs [ 67 ] = ( void * ) & rtX . fmofelvad4 [ 54 ] ;
rt_LoggedStateSignalPtrs [ 68 ] = ( void * ) & rtX . fmofelvad4 [ 55 ] ;
rt_LoggedStateSignalPtrs [ 69 ] = ( void * ) & rtX . fmofelvad4 [ 56 ] ;
rt_LoggedStateSignalPtrs [ 70 ] = ( void * ) & rtX . fmofelvad4 [ 57 ] ;
rt_LoggedStateSignalPtrs [ 71 ] = ( void * ) & rtX . fmofelvad4 [ 58 ] ;
rt_LoggedStateSignalPtrs [ 72 ] = ( void * ) & rtX . fmofelvad4 [ 59 ] ;
rt_LoggedStateSignalPtrs [ 73 ] = ( void * ) & rtX . fmofelvad4 [ 60 ] ;
rt_LoggedStateSignalPtrs [ 74 ] = ( void * ) & rtDW . ecgt4jrfbm ;
rt_LoggedStateSignalPtrs [ 75 ] = ( void * ) & rtDW . fkv2s4cq4q ;
rt_LoggedStateSignalPtrs [ 76 ] = ( void * ) & rtDW . pd2cnxedgx ;
rt_LoggedStateSignalPtrs [ 77 ] = ( void * ) & rtDW . jm54tlayqp ;
rt_LoggedStateSignalPtrs [ 78 ] = ( void * ) & rtDW . ikaqdpe14g ;
rt_LoggedStateSignalPtrs [ 79 ] = ( void * ) & rtDW . hquutdkqak ;
rt_LoggedStateSignalPtrs [ 80 ] = ( void * ) & rtDW . ccksc230fn ;
rt_LoggedStateSignalPtrs [ 81 ] = ( void * ) & rtDW . kbkgyu20nl ;
rt_LoggedStateSignalPtrs [ 82 ] = ( void * ) rtDW . b2lbbxidhp ;
rt_LoggedStateSignalPtrs [ 83 ] = ( void * ) rtDW . fvqjeh3ony ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "tmp_raccel_xout" ) ; rtliSetLogXFinal (
ssGetRTWLogInfo ( rtS ) , "xFinal" ) ; rtliSetLogVarNameModifier (
ssGetRTWLogInfo ( rtS ) , "none" ) ; rtliSetLogFormat ( ssGetRTWLogInfo ( rtS
) , 2 ) ; rtliSetLogMaxRows ( ssGetRTWLogInfo ( rtS ) , 0 ) ;
rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS ) , 1 ) ; { static void *
rt_LoggedOutputSignalPtrs [ ] = { & rtY . mkujrenrvw , & rtY . ki01i2zemv , &
rtY . cuajxr4igq , & rtY . i5hbuq5xsv [ 0 ] } ; rtliSetLogYSignalPtrs (
ssGetRTWLogInfo ( rtS ) , ( ( LogSignalPtrsType ) rt_LoggedOutputSignalPtrs )
) ; } { static int_T rt_LoggedOutputWidths [ ] = { 1 , 1 , 1 , 3 } ; static
int_T rt_LoggedOutputNumDimensions [ ] = { 1 , 1 , 1 , 1 } ; static int_T
rt_LoggedOutputDimensions [ ] = { 1 , 1 , 1 , 3 } ; static boolean_T
rt_LoggedOutputIsVarDims [ ] = { 0 , 0 , 0 , 0 } ; static void *
rt_LoggedCurrentSignalDimensions [ ] = { ( NULL ) , ( NULL ) , ( NULL ) , (
NULL ) } ; static int_T rt_LoggedCurrentSignalDimensionsSize [ ] = { 4 , 4 ,
4 , 4 } ; static BuiltInDTypeId rt_LoggedOutputDataTypeIds [ ] = { SS_DOUBLE
, SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedOutputComplexSignals [ ] = { 0 , 0 , 0 , 0 } ; static
RTWPreprocessingFcnPtr rt_LoggingPreprocessingFcnPtrs [ ] = { ( NULL ) , (
NULL ) , ( NULL ) , ( NULL ) } ; static const char_T *
rt_LoggedOutputLabels_0 [ ] = { "" } ; static const char_T *
rt_LoggedOutputBlockNames_0 [ ] = { "Control_Bicopter/Px" } ; static const
char_T * rt_LoggedOutputLabels_1 [ ] = { "" } ; static const char_T *
rt_LoggedOutputBlockNames_1 [ ] = { "Control_Bicopter/Py" } ; static const
char_T * rt_LoggedOutputLabels_2 [ ] = { "" } ; static const char_T *
rt_LoggedOutputBlockNames_2 [ ] = { "Control_Bicopter/Pz" } ; static const
char_T * rt_LoggedOutputLabels_3 [ ] = { "" } ; static const char_T *
rt_LoggedOutputBlockNames_3 [ ] = { "Control_Bicopter/euler" } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static
RTWLogSignalInfo rt_LoggedOutputSignalInfo [ ] = { { 1 ,
rt_LoggedOutputWidths , rt_LoggedOutputNumDimensions ,
rt_LoggedOutputDimensions , rt_LoggedOutputIsVarDims ,
rt_LoggedCurrentSignalDimensions , rt_LoggedCurrentSignalDimensionsSize ,
rt_LoggedOutputDataTypeIds , rt_LoggedOutputComplexSignals , ( NULL ) ,
rt_LoggingPreprocessingFcnPtrs , { rt_LoggedOutputLabels_0 } , ( NULL ) , (
NULL ) , ( NULL ) , { rt_LoggedOutputBlockNames_0 } , { ( NULL ) } , ( NULL )
, rt_RTWLogDataTypeConvert } , { 1 , rt_LoggedOutputWidths + 1 ,
rt_LoggedOutputNumDimensions + 1 , rt_LoggedOutputDimensions + 1 ,
rt_LoggedOutputIsVarDims + 1 , rt_LoggedCurrentSignalDimensions + 1 ,
rt_LoggedCurrentSignalDimensionsSize + 1 , rt_LoggedOutputDataTypeIds + 1 ,
rt_LoggedOutputComplexSignals + 1 , ( NULL ) , rt_LoggingPreprocessingFcnPtrs
+ 1 , { rt_LoggedOutputLabels_1 } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedOutputBlockNames_1 } , { ( NULL ) } , ( NULL ) ,
rt_RTWLogDataTypeConvert + 1 } , { 1 , rt_LoggedOutputWidths + 2 ,
rt_LoggedOutputNumDimensions + 2 , rt_LoggedOutputDimensions + 2 ,
rt_LoggedOutputIsVarDims + 2 , rt_LoggedCurrentSignalDimensions + 2 ,
rt_LoggedCurrentSignalDimensionsSize + 2 , rt_LoggedOutputDataTypeIds + 2 ,
rt_LoggedOutputComplexSignals + 2 , ( NULL ) , rt_LoggingPreprocessingFcnPtrs
+ 2 , { rt_LoggedOutputLabels_2 } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedOutputBlockNames_2 } , { ( NULL ) } , ( NULL ) ,
rt_RTWLogDataTypeConvert + 2 } , { 1 , rt_LoggedOutputWidths + 3 ,
rt_LoggedOutputNumDimensions + 3 , rt_LoggedOutputDimensions + 3 ,
rt_LoggedOutputIsVarDims + 3 , rt_LoggedCurrentSignalDimensions + 3 ,
rt_LoggedCurrentSignalDimensionsSize + 3 , rt_LoggedOutputDataTypeIds + 3 ,
rt_LoggedOutputComplexSignals + 3 , ( NULL ) , rt_LoggingPreprocessingFcnPtrs
+ 3 , { rt_LoggedOutputLabels_3 } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedOutputBlockNames_3 } , { ( NULL ) } , ( NULL ) ,
rt_RTWLogDataTypeConvert + 3 } } ; rtliSetLogYSignalInfo ( ssGetRTWLogInfo (
rtS ) , rt_LoggedOutputSignalInfo ) ; rt_LoggedCurrentSignalDimensions [ 0 ]
= & rt_LoggedOutputWidths [ 0 ] ; rt_LoggedCurrentSignalDimensions [ 1 ] = &
rt_LoggedOutputWidths [ 1 ] ; rt_LoggedCurrentSignalDimensions [ 2 ] = &
rt_LoggedOutputWidths [ 2 ] ; rt_LoggedCurrentSignalDimensions [ 3 ] = &
rt_LoggedOutputWidths [ 3 ] ; } rtliSetLogY ( ssGetRTWLogInfo ( rtS ) ,
"tmp_raccel_yout1,tmp_raccel_yout2,tmp_raccel_yout3,tmp_raccel_yout4" ) ; } {
static struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS , &
statesInfo2 ) ; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssSolverInfo slvrInfo ; static struct _ssSFcnModelMethods3 mdlMethods3 ;
static struct _ssSFcnModelMethods2 mdlMethods2 ; static boolean_T
contStatesDisabled [ 79 ] ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "ode3" ) ; ssSetVariableStepSolver ( rtS , 0 ) ;
ssSetSolverConsistencyChecking ( rtS , 0 ) ; ssSetSolverAdaptiveZcDetection (
rtS , 0 ) ; ssSetSolverRobustResetMethod ( rtS , 0 ) ;
_ssSetSolverUpdateJacobianAtReset ( rtS , true ) ; ssSetSolverStateProjection
( rtS , 1 ) ; ( void ) memset ( ( void * ) & mdlMethods2 , 0 , sizeof (
mdlMethods2 ) ) ; ssSetModelMethods2 ( rtS , & mdlMethods2 ) ; ( void )
memset ( ( void * ) & mdlMethods3 , 0 , sizeof ( mdlMethods3 ) ) ;
ssSetModelMethods3 ( rtS , & mdlMethods3 ) ; ssSetModelProjection ( rtS ,
MdlProjection ) ; ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetTNextTid ( rtS , INT_MIN ) ; ssSetTNext ( rtS ,
rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ; ssSetNumNonsampledZCs ( rtS ,
0 ) ; ssSetContStateDisabled ( rtS , contStatesDisabled ) ; }
ssSetChecksumVal ( rtS , 0 , 2369118705U ) ; ssSetChecksumVal ( rtS , 1 ,
162466512U ) ; ssSetChecksumVal ( rtS , 2 , 3416941678U ) ; ssSetChecksumVal
( rtS , 3 , 3742909486U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 5 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
systemRan [ 2 ] = ( sysRanDType * ) & rtDW . n4vwxzlvdz ; systemRan [ 3 ] = (
sysRanDType * ) & rtDW . gte2z2pt0y ; systemRan [ 4 ] = ( sysRanDType * ) &
rtDW . eghku2cvih ; rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS )
, & ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr (
ssGetRTWExtModeInfo ( rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr (
ssGetRTWExtModeInfo ( rtS ) , ssGetTPtr ( rtS ) ) ; } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = 2 ; void MdlOutputsParameterSampleTime (
int_T tid ) { UNUSED_PARAMETER ( tid ) ; }
