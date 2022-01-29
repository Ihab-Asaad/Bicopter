#include "__cf_Control_Bicopter.h"
#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"
PmfMessageId sm_ssci_recordRunTimeError ( const char * errorId , const char *
errorMsg , NeuDiagnosticManager * mgr ) ;
#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))
PmIntVector * pm_create_int_vector ( size_t , PmAllocator * ) ; int_T
pm_create_int_vector_fields ( PmIntVector * , size_t , PmAllocator * ) ;
int_T pm_create_real_vector_fields ( PmRealVector * , size_t , PmAllocator *
) ; int_T pm_create_char_vector_fields ( PmCharVector * , size_t ,
PmAllocator * ) ; int_T pm_create_bool_vector_fields ( PmBoolVector * ,
size_t , PmAllocator * ) ; void pm_rv_equals_rv ( const PmRealVector * ,
const PmRealVector * ) ; void sm_ssci_setupLoggerFcn_codeGen ( const NeDae *
dae , NeLoggerBuilder * neLoggerBuilder ) ; int32_T sm_ssci_logFcn_codeGen (
const NeDae * dae , const NeSystemInput * systemInput , PmRealVector * output
) ; extern const NeAssertData Control_Bicopter_ae14a523_1_assertData [ ] ;
extern const NeZCData Control_Bicopter_ae14a523_1_ZCData [ ] ; void
Control_Bicopter_ae14a523_1_computeRuntimeParameters ( const double *
runtimeRootVariables , double * runtimeParameters ) ; void
Control_Bicopter_ae14a523_1_validateRuntimeParameters ( const double *
runtimeParameters , int32_T * assertSatisfactionFlags ) ; void
Control_Bicopter_ae14a523_1_computeAsmRuntimeDerivedValues ( const double *
runtimeParameters , RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle )
; void Control_Bicopter_ae14a523_1_computeSimRuntimeDerivedValues ( const
double * runtimeParameters , RuntimeDerivedValuesBundle *
runtimeDerivedValuesBundle ) ; PmfMessageId Control_Bicopter_ae14a523_1_deriv
( const RuntimeDerivedValuesBundle * , const int * , const double * , const
int * , const double * , const double * , const double * , const double * ,
double * , double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_checkDynamics ( const RuntimeDerivedValuesBundle
* , const double * , const double * , const double * , const double * , const
double * , double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_outputDyn ( const RuntimeDerivedValuesBundle * ,
const int * , const double * , const int * , const double * , const double *
, const double * , const double * , double * , double * , int * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_outputKin ( const RuntimeDerivedValuesBundle * ,
const double * , const int * , const double * , const double * , const double
* , const double * , double * , NeuDiagnosticManager * neDiagMgr ) ;
PmfMessageId Control_Bicopter_ae14a523_1_output ( const
RuntimeDerivedValuesBundle * , const double * , const int * , const double *
, const double * , const double * , const double * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_computeAsmModeVector ( const double * , const
double * , const double * , int * , double * , NeuDiagnosticManager *
neDiagMgr ) ; PmfMessageId Control_Bicopter_ae14a523_1_computeSimModeVector (
const double * , const double * , const double * , int * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_onModeChanged ( const double * , const double * ,
const double * , const int * , int * , double * , double * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
Control_Bicopter_ae14a523_1_computeZeroCrossings ( const double * , const
double * , const double * , double * , double * , NeuDiagnosticManager *
neDiagMgr ) ;
#if 0
void Control_Bicopter_ae14a523_1_checkTargets ( const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const double *
stateVector ) ;
#endif
void Control_Bicopter_ae14a523_1_setTargets ( const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , CTarget * targets )
; void Control_Bicopter_ae14a523_1_resetStateVector ( const void * mech ,
double * stateVector ) ; void Control_Bicopter_ae14a523_1_resetModeVector (
const void * mech , int * modeVector ) ; void
Control_Bicopter_ae14a523_1_initializeTrackedAngleState ( const void * mech ,
const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const int *
modeVector , const double * motionData , double * stateVector , void *
neDiagMgr ) ; void Control_Bicopter_ae14a523_1_computeDiscreteState ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
double * stateVector ) ; void Control_Bicopter_ae14a523_1_adjustPosition (
const void * mech , const double * dofDeltas , double * stateVector ) ; void
Control_Bicopter_ae14a523_1_perturbJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primitiveIdx , double magnitude , boolean_T
doPerturbVelocity , double * stateVector ) ; void
Control_Bicopter_ae14a523_1_perturbFlexibleBodyState ( const void * mech ,
size_t stageIdx , double magnitude , boolean_T doPerturbVelocity , double *
stateVector ) ; void Control_Bicopter_ae14a523_1_computeDofBlendMatrix (
const void * mech , size_t stageIdx , size_t primitiveIdx , const double *
stateVector , int partialType , double * matrix ) ; void
Control_Bicopter_ae14a523_1_projectPartiallyTargetedPos ( const void * mech ,
size_t stageIdx , size_t primitiveIdx , const double * origStateVector , int
partialType , double * stateVector ) ; void
Control_Bicopter_ae14a523_1_propagateMotion ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const double *
stateVector , double * motionData ) ; size_t
Control_Bicopter_ae14a523_1_computeAssemblyError ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , size_t
constraintIdx , const double * stateVector , const int * modeVector , const
double * motionData , double * error ) ; size_t
Control_Bicopter_ae14a523_1_computeAssemblyJacobian ( const void * mech ,
const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , size_t
constraintIdx , boolean_T forVelocitySatisfaction , const double *
stateVector , const int * modeVector , const double * motionData , double * J
) ; size_t Control_Bicopter_ae14a523_1_computeFullAssemblyJacobian ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
const double * stateVector , const int * modeVector , const double *
motionData , double * J ) ; int
Control_Bicopter_ae14a523_1_isInKinematicSingularity ( const void * mech ,
const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , size_t
constraintIdx , const int * modeVector , const double * motionData ) ;
PmfMessageId Control_Bicopter_ae14a523_1_convertStateVector ( const void *
asmMech , const RuntimeDerivedValuesBundle * asmRuntimeDerivedValuesBundle ,
const void * simMech , const double * asmStateVector , const int *
asmModeVector , const int * simModeVector , double * simStateVector , void *
neDiagMgr ) ; void Control_Bicopter_ae14a523_1_constructStateVector ( const
void * mech , const double * solverStateVector , const double * u , const
double * uDot , const double * discreteStateVector , double * fullStateVector
) ; void Control_Bicopter_ae14a523_1_extractSolverStateVector ( const void *
mech , const double * fullStateVector , double * solverStateVector ) ; int
Control_Bicopter_ae14a523_1_isPositionViolation ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const int *
constraintEqnEnableFlags , const double * stateVector , const int *
modeVector ) ; int Control_Bicopter_ae14a523_1_isVelocityViolation ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
const int * constraintEqnEnableFlags , const double * stateVector , const int
* modeVector ) ; PmfMessageId Control_Bicopter_ae14a523_1_projectStateSim (
const void * mech , const RuntimeDerivedValuesBundle *
runtimeDerivedValuesBundle , const int * constraintEqnEnableFlags , const int
* modeVector , const double * inputVector , double * stateVector , void *
neDiagMgr ) ; void Control_Bicopter_ae14a523_1_computeConstraintError ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
const double * stateVector , const int * modeVector , double * error ) ;
PmfMessageId Control_Bicopter_ae14a523_1_assemble ( const double * u , double
* udot , double * x , NeuDiagnosticManager * neDiagMgr ) { ( void ) x ; (
void ) u ; ( void ) udot ; ( void ) neDiagMgr ; return NULL ; } static void
dae_cg_setParameters_function ( const NeDae * dae , const NeParameterBundle *
paramBundle ) { const NeDaePrivateData * smData = dae -> mPrivateData ; const
double * runtimeRootVariables = paramBundle -> mRealParameters . mX ; if (
smData -> mRuntimeParameterScalars . mN == 0 ) return ;
Control_Bicopter_ae14a523_1_computeRuntimeParameters ( runtimeRootVariables ,
smData -> mRuntimeParameterScalars . mX ) ;
Control_Bicopter_ae14a523_1_computeAsmRuntimeDerivedValues ( smData ->
mRuntimeParameterScalars . mX , & dae -> mPrivateData ->
mAsmRuntimeDerivedValuesBundle ) ;
Control_Bicopter_ae14a523_1_computeSimRuntimeDerivedValues ( smData ->
mRuntimeParameterScalars . mX , & dae -> mPrivateData ->
mSimRuntimeDerivedValuesBundle ) ;
sm_core_computeRedundantConstraintEquations ( & dae -> mPrivateData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ) ;
#if 0
{ size_t i ; const size_t n = smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mSize ; pmf_printf (
"\nRuntime Enabled Equations (%lu)\n" , n ) ; for ( i = 0 ; i < n ; ++ i )
pmf_printf ( "  %2lu:  %d\n" , i , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues [ i ] ) ; }
#endif
} static PmfMessageId dae_cg_pAssert_method ( const NeDae * dae , const
NeSystemInput * systemInput , NeDaeMethodOutput * daeMethodOutput ,
NeuDiagnosticManager * neDiagMgr ) { const NeDaePrivateData * smData = dae ->
mPrivateData ; const double * runtimeParams = smData ->
mRuntimeParameterScalars . mX ; int32_T * assertSatisfactionFlags =
daeMethodOutput -> mPASSERT . mX ; ( void ) systemInput ; ( void ) neDiagMgr
; Control_Bicopter_ae14a523_1_validateRuntimeParameters ( runtimeParams ,
assertSatisfactionFlags ) ; return NULL ; } static PmfMessageId
dae_cg_deriv_method ( const NeDae * dae , const NeSystemInput * systemInput ,
NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager * neDiagMgr ) {
const NeDaePrivateData * smData = dae -> mPrivateData ; PmfMessageId errorId
= NULL ; double errorResult = 0.0 ; if ( smData ->
mCachedDerivativesAvailable ) memcpy ( daeMethodOutput -> mXP0 . mX , smData
-> mCachedDerivatives . mX , 61 * sizeof ( real_T ) ) ; else errorId =
Control_Bicopter_ae14a523_1_deriv ( & smData ->
mSimRuntimeDerivedValuesBundle , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues , systemInput -> mX . mX , systemInput ->
mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 6 , systemInput
-> mV . mX + 6 , systemInput -> mD . mX , daeMethodOutput -> mXP0 . mX , &
errorResult , neDiagMgr ) ; return errorId ; } static PmfMessageId
dae_cg_output_method ( const NeDae * dae , const NeSystemInput * systemInput
, NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager * neDiagMgr ) {
PmfMessageId errorId = NULL ; NeDaePrivateData * smData = dae -> mPrivateData
; errorId = Control_Bicopter_ae14a523_1_output ( & smData ->
mSimRuntimeDerivedValuesBundle , systemInput -> mX . mX , systemInput -> mM .
mX , systemInput -> mU . mX , systemInput -> mU . mX + 6 , systemInput -> mV
. mX + 6 , systemInput -> mD . mX , daeMethodOutput -> mY . mX , neDiagMgr )
; return errorId ; } static PmfMessageId dae_cg_mode_method ( const NeDae *
dae , const NeSystemInput * systemInput , NeDaeMethodOutput * daeMethodOutput
, NeuDiagnosticManager * neDiagMgr ) { const NeDaePrivateData * smData = dae
-> mPrivateData ; PmfMessageId errorId = NULL ; double errorResult = 0.0 ;
errorId = Control_Bicopter_ae14a523_1_computeSimModeVector ( systemInput ->
mU . mX , systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 ,
daeMethodOutput -> mMODE . mX , & errorResult , neDiagMgr ) ; memcpy ( smData
-> mCachedModeVector . mX , daeMethodOutput -> mMODE . mX , 0 * sizeof (
int32_T ) ) ; return errorId ; } static PmfMessageId
dae_cg_zeroCrossing_method ( const NeDae * dae , const NeSystemInput *
systemInput , NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager *
neDiagMgr ) { const NeDaePrivateData * smData = dae -> mPrivateData ; double
errorResult = 0.0 ; return Control_Bicopter_ae14a523_1_computeZeroCrossings (
systemInput -> mU . mX , systemInput -> mU . mX + 6 , systemInput -> mV . mX
+ 6 , daeMethodOutput -> mZC . mX , & errorResult , neDiagMgr ) ; } static
PmfMessageId dae_cg_project_solve ( const NeDae * dae , const NeSystemInput *
systemInput , NeuDiagnosticManager * neDiagMgr ) { NeDaePrivateData * smData
= dae -> mPrivateData ; return sm_core_projectState ( false , & smData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ,
systemInput -> mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 6
, systemInput -> mD . mX , systemInput -> mX . mX , neDiagMgr ) ; } static
PmfMessageId dae_cg_check_solve ( const NeDae * dae , const NeSystemInput *
systemInput , NeuDiagnosticManager * neDiagMgr ) { NeDaePrivateData * smData
= dae -> mPrivateData ; PmfMessageId errorId = NULL ; if ( smData ->
mNumConstraintEqns > 0 ) errorId = sm_core_projectState ( false , & smData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ,
systemInput -> mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 6
, systemInput -> mD . mX , systemInput -> mX . mX , neDiagMgr ) ; if (
errorId == NULL && smData -> mDoCheckDynamics ) { double result = 0.0 ;
errorId = Control_Bicopter_ae14a523_1_checkDynamics ( & smData ->
mSimRuntimeDerivedValuesBundle , systemInput -> mX . mX , systemInput -> mU .
mX , systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 , systemInput ->
mD . mX , & result , neDiagMgr ) ; } return errorId ; } static PmfMessageId
dae_cg_CIC_MODE_solve ( const NeDae * dae , const NeSystemInput * systemInput
, NeuDiagnosticManager * neDiagMgr ) { NeDaePrivateData * smData = dae ->
mPrivateData ; PmfMessageId errorId = NULL ; double errorResult = 0.0 ; const
size_t mvSize = smData -> mModeVectorSize ; boolean_T modeChanged = false ;
if ( mvSize > 0 ) { errorId =
Control_Bicopter_ae14a523_1_computeSimModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 , systemInput -> mM .
mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL ) return errorId ; {
size_t i ; for ( i = 0 ; i < mvSize ; ++ i ) if ( systemInput -> mM . mX [ i
] != smData -> mCachedModeVector . mX [ i ] ) { modeChanged = true ; break ;
} } } if ( modeChanged ) { errorId =
Control_Bicopter_ae14a523_1_onModeChanged ( systemInput -> mU . mX ,
systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 , smData ->
mCachedModeVector . mX , systemInput -> mM . mX , systemInput -> mX . mX ,
systemInput -> mD . mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL )
return errorId ; memcpy ( smData -> mCachedModeVector . mX , systemInput ->
mM . mX , 0 * sizeof ( int32_T ) ) ; } errorId = sm_core_projectState ( true
, & smData -> mSimulationDelegate , & smData ->
mSimRuntimeDerivedValuesBundle , systemInput -> mM . mX , systemInput -> mU .
mX , systemInput -> mU . mX + 6 , systemInput -> mD . mX , systemInput -> mX
. mX , neDiagMgr ) ; return errorId ; } static PmfMessageId
dae_cg_assemble_solve ( const NeDae * dae , const NeSystemInput * systemInput
, NeuDiagnosticManager * neDiagMgr ) { NeDaePrivateData * smData = dae ->
mPrivateData ; const SmMechanismDelegate * delegate = & smData ->
mAssemblyDelegate ; const RuntimeDerivedValuesBundle *
runtimeDerivedValuesBundle = & smData -> mAsmRuntimeDerivedValuesBundle ;
PmfMessageId errorId = NULL ; size_t i ; double errorResult = 0.0 ; const
size_t numTargets = 102 ; unsigned int asmStatus = 0 ; double *
assemblyFullStateVector = smData -> mAssemblyFullStateVector . mX ; double *
simulationFullStateVector = smData -> mSimulationFullStateVector . mX ; ( *
delegate -> mSetTargets ) ( runtimeDerivedValuesBundle , smData -> mTargets )
; { const double * u = systemInput -> mU . mX ; const double * uDot = u +
smData -> mInputVectorSize ; CTarget * target = smData -> mTargets + smData
-> mNumInternalTargets ; for ( i = 0 ; i < smData ->
mNumInputMotionPrimitives ; ++ i ) { const size_t inputOffset = smData ->
mMotionInputOffsets . mX [ i ] ; ( target ++ ) -> mValue [ 0 ] = u [
inputOffset ] ; ( target ++ ) -> mValue [ 0 ] = uDot [ inputOffset ] ; } } if
( smData -> mAssemblyModeVector . mN > 0 ) { errorId =
Control_Bicopter_ae14a523_1_computeAsmModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 , smData ->
mAssemblyModeVector . mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL
) return errorId ; } errorId = sm_core_computeStateVector ( delegate ,
runtimeDerivedValuesBundle , smData -> mAssemblyModeVector . mX , numTargets
, smData -> mTargets , assemblyFullStateVector , neDiagMgr ) ; if ( errorId
!= NULL ) return errorId ; asmStatus = sm_core_checkAssembly ( delegate ,
runtimeDerivedValuesBundle , assemblyFullStateVector , smData ->
mAssemblyModeVector . mX , NULL , NULL , NULL ) ; if ( asmStatus != 1 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:AssemblyFailure" , asmStatus == 2 ?
 "Model not assembled due to a position violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
: ( asmStatus == 3 ?
 "Model not assembled due to a velocity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
:
 "Model not assembled due to a singularity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
) , neDiagMgr ) ; }
#if 0
Control_Bicopter_ae14a523_1_checkTargets ( & smData ->
mSimRuntimeDerivedValuesBundle , assemblyFullStateVector ) ;
#endif
if ( smData -> mModeVectorSize > 0 ) { errorId =
Control_Bicopter_ae14a523_1_computeSimModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 6 , systemInput -> mV . mX + 6 , systemInput -> mM .
mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL ) return errorId ;
memcpy ( smData -> mCachedModeVector . mX , systemInput -> mM . mX , 0 *
sizeof ( int32_T ) ) ; } errorId = ( * delegate -> mConvertStateVector ) (
NULL , runtimeDerivedValuesBundle , NULL , assemblyFullStateVector , smData
-> mAssemblyModeVector . mX , systemInput -> mM . mX ,
simulationFullStateVector , neDiagMgr ) ; for ( i = 0 ; i < smData ->
mStateVectorSize ; ++ i ) systemInput -> mX . mX [ i ] =
simulationFullStateVector [ smData -> mStateVectorMap . mX [ i ] ] ; memcpy (
systemInput -> mD . mX , simulationFullStateVector + smData ->
mFullStateVectorSize - smData -> mDiscreteStateSize , smData ->
mDiscreteStateSize * sizeof ( double ) ) ; return errorId ; } typedef struct
{ size_t first ; size_t second ; } SizePair ; static void checkMemAllocStatus
( int_T status ) { ( void ) status ; } static PmCharVector
cStringToCharVector ( const char * src ) { const size_t n = strlen ( src ) ;
PmCharVector charVect ; const int_T status = pm_create_char_vector_fields ( &
charVect , n + 1 , pm_default_allocator ( ) ) ; checkMemAllocStatus ( status
) ; strcpy ( charVect . mX , src ) ; return charVect ; } static void
initBasicAttributes ( NeDaePrivateData * smData ) { size_t i ; smData ->
mStateVectorSize = 61 ; smData -> mFullStateVectorSize = 69 ; smData ->
mDiscreteStateSize = 0 ; smData -> mModeVectorSize = 0 ; smData ->
mNumZeroCrossings = 0 ; smData -> mInputVectorSize = 6 ; smData ->
mOutputVectorSize = 9 ; smData -> mNumConstraintEqns = 16 ; smData ->
mDoCheckDynamics = true ; for ( i = 0 ; i < 4 ; ++ i ) smData -> mChecksum [
i ] = 0 ; } static void initStateVector ( NeDaePrivateData * smData ) {
PmAllocator * alloc = pm_default_allocator ( ) ; const int32_T stateVectorMap
[ 61 ] = { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 ,
15 , 16 , 19 , 20 , 21 , 22 , 25 , 26 , 27 , 28 , 29 , 30 , 33 , 34 , 35 , 36
, 39 , 40 , 41 , 42 , 43 , 44 , 45 , 46 , 47 , 48 , 49 , 50 , 51 , 52 , 53 ,
54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 66 , 67 , 68 } ;
const CTarget targets [ 102 ] = { { 0 , 65 , 0 , false , 0 , 0 , "1" , false
, true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 65 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 65 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 65 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 66 , 0 , false , 0 , 1 , "deg" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { - 5.253732562902286530e-16 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 66 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 66 , 1 , false , 0 , 1 , "mm" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 66 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 67 , 0 , false , 0 , 1 , "deg" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { - 3.141592653589792672e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 67 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 67 , 1 , false , 0 , 1 , "mm" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 67 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 68 , 0 , false , 0 , 1 , "deg" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { - 2.095267767019191485e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 68 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 68 , 1 , false , 0 , 1 , "mm" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 68 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 69 , 0 , false , 0 , 1 , "deg" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 3.140719988963802667e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 69 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 69 , 1 , false , 0 , 1 , "mm" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 69 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 101 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 101 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 101 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 101 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 101 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 101 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 102 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 102 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 102 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 102 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 102 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 102 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 103 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 103 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 103 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 103 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 103 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 103 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 104 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 104 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 104 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 104 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 104 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 104 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 105 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 105 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 105 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 105 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 105 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 105 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 106 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 106 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 106 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 106 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 106 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 106 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 107 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 107 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 107 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 107 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 107 , 2 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 107 , 2
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 108 , 0 , false , 0 , 1 , "m" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 108 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 109 , 0 , false , 0 , 1 , "m" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 109 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 110 , 0 , false , 0 , 1 , "m" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 110 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 111 , 0 , false , 0 , 1 , "m" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 111 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 112 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { - 3.141592653589766471e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 112 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 113 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { + 1.823115774441286406e-14
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 113 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 114 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { + 1.570796326794904108e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 114 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 115 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { + 1.570796326794902331e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 115 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 116 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { + 1.570768162545919999e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 116 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 117 , 0 , false , 0 , 1 , "deg" , false
, true , + 1.000000000000000000e+00 , true , 1 , { - 1.570768162545916891e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 117 , 0
, false , 0 , 0 , "1" , true , true , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 118 , 0 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 118 , 0
, false , 0 , 1 , "rad/s" , true , true , + 1.000000000000000000e+00 , true ,
1 , { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 119 , 0 , false , 0 , 0 , "1" , false ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 119 , 0
, false , 0 , 2 , "rad/s" , true , true , + 1.000000000000000000e+00 , true ,
1 , { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 120 , 0 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 120 , 0
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 120 , 1 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 120 , 1
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 120 , 2 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 120 , 2
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 1 , 120 , 3 , false , 0 , 0 , "1" , false ,
false , + 1.000000000000000000e+00 , true , 4 , { + 1.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 2 , 120 , 3
, false , 0 , 0 , "1" , true , false , + 1.000000000000000000e+00 , true , 3
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 113 , 0 , false , 0 , 3 , "" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 113 , 0
, false , 0 , 3 , "" , true , false , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 118 , 0 , false , 0 , 3 , "" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 118 , 0
, false , 0 , 3 , "" , true , false , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 112 , 0 , false , 0 , 3 , "" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 112 , 0
, false , 0 , 3 , "" , true , false , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 119 , 0 , false , 0 , 3 , "" , false ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 119 , 0
, false , 0 , 3 , "" , true , false , + 1.000000000000000000e+00 , true , 1 ,
{ + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } } ; const size_t numTargets = 102 ; int_T status
; size_t i ; status = pm_create_real_vector_fields ( & smData ->
mAssemblyFullStateVector , 69 , alloc ) ; checkMemAllocStatus ( status ) ;
status = pm_create_real_vector_fields ( & smData ->
mSimulationFullStateVector , 69 , alloc ) ; checkMemAllocStatus ( status ) ;
status = pm_create_int_vector_fields ( & smData -> mStateVectorMap , smData
-> mStateVectorSize , alloc ) ; checkMemAllocStatus ( status ) ; memcpy (
smData -> mStateVectorMap . mX , stateVectorMap , smData -> mStateVectorSize
* sizeof ( int32_T ) ) ; smData -> mNumInternalTargets = 94 ; smData ->
mNumInputMotionPrimitives = 4 ; PM_ALLOCATE_ARRAY ( smData -> mTargets ,
CTarget , numTargets , alloc ) ; for ( i = 0 ; i < numTargets ; ++ i )
sm_compiler_CTarget_copy ( targets + i , smData -> mTargets + i ) ; } static
void initAsserts ( NeDaePrivateData * smData ) { PmAllocator * alloc =
pm_default_allocator ( ) ; int_T status = 0 ; smData -> mNumParamAsserts = 0
; smData -> mParamAssertObjects = NULL ; smData -> mParamAssertPaths = NULL ;
smData -> mParamAssertDescriptors = NULL ; smData -> mParamAssertMessages =
NULL ; smData -> mParamAssertMessageIds = NULL ; status =
pm_create_bool_vector_fields ( & smData -> mParamAssertIsWarnings , smData ->
mNumParamAsserts , alloc ) ; checkMemAllocStatus ( status ) ; if ( smData ->
mNumParamAsserts > 0 ) { const NeAssertData * ad =
Control_Bicopter_ae14a523_1_assertData ; size_t i ; PM_ALLOCATE_ARRAY (
smData -> mParamAssertObjects , PmCharVector , 0 , alloc ) ;
PM_ALLOCATE_ARRAY ( smData -> mParamAssertPaths , PmCharVector , 0 , alloc )
; PM_ALLOCATE_ARRAY ( smData -> mParamAssertDescriptors , PmCharVector , 0 ,
alloc ) ; PM_ALLOCATE_ARRAY ( smData -> mParamAssertMessages , PmCharVector ,
0 , alloc ) ; PM_ALLOCATE_ARRAY ( smData -> mParamAssertMessageIds ,
PmCharVector , 0 , alloc ) ; for ( i = 0 ; i < smData -> mNumParamAsserts ;
++ i , ++ ad ) { smData -> mParamAssertObjects [ i ] = cStringToCharVector (
ad -> mObject ) ; smData -> mParamAssertPaths [ i ] = cStringToCharVector (
ad -> mPath ) ; smData -> mParamAssertDescriptors [ i ] = cStringToCharVector
( ad -> mDescriptor ) ; smData -> mParamAssertMessages [ i ] =
cStringToCharVector ( ad -> mMessage ) ; smData -> mParamAssertMessageIds [ i
] = cStringToCharVector ( ad -> mMessageID ) ; smData ->
mParamAssertIsWarnings . mX [ i ] = ad -> mIsWarn ; } } } static void
initModeVector ( NeDaePrivateData * smData ) { { size_t i ; const int_T
status = pm_create_int_vector_fields ( & smData -> mAssemblyModeVector , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mAssemblyModeVector . mN ; ++ i ) smData -> mAssemblyModeVector .
mX [ i ] = 0 ; } { size_t i ; const int_T status =
pm_create_int_vector_fields ( & smData -> mCachedModeVector , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mModeVectorSize ; ++ i ) smData -> mCachedModeVector . mX [ i ] =
0 ; } } static void initZeroCrossings ( NeDaePrivateData * smData ) {
PmAllocator * alloc = pm_default_allocator ( ) ; int_T status = 0 ; smData ->
mZeroCrossingObjects = NULL ; smData -> mZeroCrossingPaths = NULL ; smData ->
mZeroCrossingDescriptors = NULL ; status = pm_create_int_vector_fields ( &
smData -> mZeroCrossingTypes , 0 , alloc ) ; checkMemAllocStatus ( status ) ;
if ( smData -> mNumZeroCrossings > 0 ) { const NeZCData * zcd =
Control_Bicopter_ae14a523_1_ZCData ; size_t i ; PM_ALLOCATE_ARRAY ( smData ->
mZeroCrossingObjects , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY (
smData -> mZeroCrossingPaths , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY
( smData -> mZeroCrossingDescriptors , PmCharVector , 0 , alloc ) ; for ( i =
0 ; i < smData -> mNumZeroCrossings ; ++ i , ++ zcd ) { smData ->
mZeroCrossingObjects [ i ] = cStringToCharVector ( zcd -> mObject ) ; smData
-> mZeroCrossingPaths [ i ] = cStringToCharVector ( zcd -> mPath ) ; smData
-> mZeroCrossingDescriptors [ i ] = cStringToCharVector ( zcd -> mDescriptor
) ; smData -> mZeroCrossingTypes . mX [ i ] = zcd -> mType ; } } } static
void initVariables ( NeDaePrivateData * smData ) { const char * varFullPaths
[ 61 ] = { "Subsystem.SixDOF.Px.p" , "Subsystem.SixDOF.Py.p" ,
"Subsystem.SixDOF.Pz.p" , "Subsystem.SixDOF.S.Q" , "Subsystem.SixDOF.S.Q" ,
"Subsystem.SixDOF.S.Q" , "Subsystem.SixDOF.S.Q" , "Subsystem.SixDOF.Px.v" ,
"Subsystem.SixDOF.Py.v" , "Subsystem.SixDOF.Pz.v" , "Subsystem.SixDOF.S.w" ,
"Subsystem.SixDOF.S.w" , "Subsystem.SixDOF.S.w" ,
"Subsystem.Cylindrical1.Rz.q" , "Subsystem.Cylindrical1.Pz.p" ,
"Subsystem.Cylindrical1.Rz.w" , "Subsystem.Cylindrical1.Pz.v" ,
"Subsystem.Cylindrical3.Rz.q" , "Subsystem.Cylindrical3.Pz.p" ,
"Subsystem.Cylindrical3.Rz.w" , "Subsystem.Cylindrical3.Pz.v" ,
"Subsystem.Prismatic.Pz.p" , "Subsystem.Prismatic.Pz.v" ,
"Subsystem.Cylindrical2.Rz.q" , "Subsystem.Cylindrical2.Pz.p" ,
"Subsystem.Cylindrical2.Rz.w" , "Subsystem.Cylindrical2.Pz.v" ,
"Subsystem.Cylindrical4.Rz.q" , "Subsystem.Cylindrical4.Pz.p" ,
"Subsystem.Cylindrical4.Rz.w" , "Subsystem.Cylindrical4.Pz.v" ,
"Subsystem.Prismatic1.Pz.p" , "Subsystem.Prismatic1.Pz.v" ,
"Subsystem.Prismatic2.Pz.p" , "Subsystem.Prismatic2.Pz.v" ,
"Subsystem.Prismatic3.Pz.p" , "Subsystem.Prismatic3.Pz.v" ,
"Subsystem.Revolute2.Rz.q" , "Subsystem.Revolute2.Rz.w" ,
"Subsystem.Revolute3.Rz.q" , "Subsystem.Revolute3.Rz.w" ,
"Subsystem.Revolute4.Rz.q" , "Subsystem.Revolute4.Rz.w" ,
"Subsystem.Revolute5.Rz.q" , "Subsystem.Revolute5.Rz.w" ,
"Subsystem.Planar1.Rz.q" , "Subsystem.Planar1.Rz.w" ,
"Subsystem.Cylindrical.Rz.q" , "Subsystem.Cylindrical.Rz.w" ,
"Subsystem.Planar.Rz.q" , "Subsystem.Planar.Rz.w" , "Subsystem.Planar4.Rz.q"
, "Subsystem.Planar4.Rz.w" , "Subsystem.Planar3.Rz.q" ,
"Subsystem.Planar3.Rz.w" , "Subsystem.Planar2.Rz.q" ,
"Subsystem.Planar2.Rz.w" , "Subsystem.Planar5.Rz.q" ,
"Subsystem.Planar5.Rz.w" , "Subsystem.Planar6.Rz.q" ,
"Subsystem.Planar6.Rz.w" } ; const char * varObjects [ 61 ] = {
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" , "Control_Bicopter/Subsystem/SixDOF" ,
"Control_Bicopter/Subsystem/SixDOF" ,
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
"Control_Bicopter/Subsystem/Planar6" , "Control_Bicopter/Subsystem/Planar6" }
; smData -> mNumVarScalars = 61 ; smData -> mVarFullPaths = NULL ; smData ->
mVarObjects = NULL ; if ( smData -> mNumVarScalars > 0 ) { size_t s ;
PmAllocator * alloc = pm_default_allocator ( ) ; PM_ALLOCATE_ARRAY ( smData
-> mVarFullPaths , PmCharVector , 61 , alloc ) ; PM_ALLOCATE_ARRAY ( smData
-> mVarObjects , PmCharVector , 61 , alloc ) ; for ( s = 0 ; s < smData ->
mNumVarScalars ; ++ s ) { smData -> mVarFullPaths [ s ] = cStringToCharVector
( varFullPaths [ s ] ) ; smData -> mVarObjects [ s ] = cStringToCharVector (
varObjects [ s ] ) ; } } } static void initRuntimeParameters (
NeDaePrivateData * smData ) { PmAllocator * alloc = pm_default_allocator ( )
; int_T status = 0 ; size_t i = 0 ; const int32_T * rtpRootVarRows = NULL ;
const int32_T * rtpRootVarCols = NULL ; const char * * rtpFullPaths = NULL ;
smData -> mNumRtpRootVars = 0 ; status = pm_create_int_vector_fields ( &
smData -> mRtpRootVarRows , smData -> mNumRtpRootVars , alloc ) ;
checkMemAllocStatus ( status ) ; memcpy ( smData -> mRtpRootVarRows . mX ,
rtpRootVarRows , smData -> mNumRtpRootVars * sizeof ( int32_T ) ) ; status =
pm_create_int_vector_fields ( & smData -> mRtpRootVarCols , smData ->
mNumRtpRootVars , alloc ) ; checkMemAllocStatus ( status ) ; memcpy ( smData
-> mRtpRootVarCols . mX , rtpRootVarCols , smData -> mNumRtpRootVars * sizeof
( int32_T ) ) ; smData -> mRtpFullPaths = NULL ; if ( smData ->
mNumRtpRootVars > 0 ) { size_t v ; PM_ALLOCATE_ARRAY ( smData ->
mRtpFullPaths , PmCharVector , 0 , alloc ) ; for ( v = 0 ; v < smData ->
mNumRtpRootVars ; ++ v ) { smData -> mRtpFullPaths [ v ] =
cStringToCharVector ( rtpFullPaths [ v ] ) ; } } smData ->
mNumRuntimeRootVarScalars = 0 ; status = pm_create_real_vector_fields ( &
smData -> mRuntimeParameterScalars , 0 , alloc ) ; checkMemAllocStatus (
status ) ; for ( i = 0 ; i < smData -> mRuntimeParameterScalars . mN ; ++ i )
smData -> mRuntimeParameterScalars . mX [ i ] = 0.0 ;
sm_core_RuntimeDerivedValuesBundle_create ( & smData ->
mAsmRuntimeDerivedValuesBundle , 0 , 0 ) ;
sm_core_RuntimeDerivedValuesBundle_create ( & smData ->
mSimRuntimeDerivedValuesBundle , 0 , 0 ) ; } static void initIoInfoHelper (
size_t n , const char * portPathsSource [ ] , const char * unitsSource [ ] ,
const SizePair dimensions [ ] , boolean_T doInputs , NeDaePrivateData *
smData ) { PmCharVector * portPaths = NULL ; PmCharVector * units = NULL ;
NeDsIoInfo * infos = NULL ; if ( n > 0 ) { size_t s ; PmAllocator * alloc =
pm_default_allocator ( ) ; PM_ALLOCATE_ARRAY ( portPaths , PmCharVector , n ,
alloc ) ; PM_ALLOCATE_ARRAY ( units , PmCharVector , n , alloc ) ;
PM_ALLOCATE_ARRAY ( infos , NeDsIoInfo , n , alloc ) ; for ( s = 0 ; s < n ;
++ s ) { portPaths [ s ] = cStringToCharVector ( portPathsSource [ s ] ) ;
units [ s ] = cStringToCharVector ( unitsSource [ s ] ) ; { NeDsIoInfo * info
= infos + s ; info -> mName = info -> mIdentifier = portPaths [ s ] . mX ;
info -> mM = dimensions [ s ] . first ; info -> mN = dimensions [ s ] .
second ; info -> mUnit = units [ s ] . mX ; } } } if ( doInputs ) { smData ->
mNumInputs = n ; smData -> mInputPortPaths = portPaths ; smData ->
mInputUnits = units ; smData -> mInputInfos = infos ; } else { smData ->
mNumOutputs = n ; smData -> mOutputPortPaths = portPaths ; smData ->
mOutputUnits = units ; smData -> mOutputInfos = infos ; } } static void
initIoInfo ( NeDaePrivateData * smData ) { const char * inputPortPaths [ 6 ]
= { "Subsystem.Revolute.qi" , "Subsystem.Revolute1.qi" ,
"Subsystem.Revolute6.qi" , "Subsystem.Revolute7.qi" ,
"Subsystem.x5x4x3_propeller1_RIGID.External_Force_and_Torque.fy" ,
"Subsystem.x5x4x3_propeller2_RIGID.External_Force_and_Torque.fy" } ; const
char * inputUnits [ 6 ] = { "rad" , "rad" , "rad" , "rad" , "m*kg/s^2" ,
"m*kg/s^2" } ; const SizePair inputDimensions [ 6 ] = { { 1 , 1 } , { 1 , 1 }
, { 1 , 1 } , { 1 , 1 } , { 1 , 1 } , { 1 , 1 } } ; const char *
outputPortPaths [ 6 ] = { "Subsystem.Revolute6.w" , "Subsystem.Revolute7.w" ,
"Subsystem.SixDOF.px" , "Subsystem.SixDOF.py" , "Subsystem.SixDOF.pz" ,
"Subsystem.SixDOF.Q" } ; const char * outputUnits [ 6 ] = { "rad/s" , "rad/s"
, "m" , "m" , "m" , "1" } ; const SizePair outputDimensions [ 6 ] = { { 1 , 1
} , { 1 , 1 } , { 1 , 1 } , { 1 , 1 } , { 1 , 1 } , { 4 , 1 } } ;
initIoInfoHelper ( 6 , inputPortPaths , inputUnits , inputDimensions , true ,
smData ) ; initIoInfoHelper ( 6 , outputPortPaths , outputUnits ,
outputDimensions , false , smData ) ; } static void initInputDerivs (
NeDaePrivateData * smData ) { const int32_T numInputDerivs [ 6 ] = { 2 , 2 ,
2 , 2 , 0 , 0 } ; PmAllocator * alloc = pm_default_allocator ( ) ; const
int_T status = pm_create_int_vector_fields ( & smData -> mNumInputDerivs ,
smData -> mInputVectorSize , alloc ) ; checkMemAllocStatus ( status ) ;
memcpy ( smData -> mNumInputDerivs . mX , numInputDerivs , 6 * sizeof (
int32_T ) ) ; smData -> mInputOrder = 2 ; } static void initDirectFeedthrough
( NeDaePrivateData * smData ) { const boolean_T directFeedthroughVector [ 6 ]
= { true , true , true , true , false , false } ; const boolean_T
directFeedthroughMatrix [ 108 ] = { true , true , true , true , true , true ,
true , true , true , true , true , true , true , true , true , true , true ,
true , true , true , true , true , true , true , true , true , true , true ,
true , true , true , true , true , true , true , true , false , false , false
, false , false , false , false , false , false , false , false , false ,
false , false , false , false , false , false , true , true , true , true ,
true , true , true , true , true , true , true , true , true , true , true ,
true , true , true , true , true , true , true , true , true , true , true ,
true , true , true , true , true , true , true , true , true , true , false ,
false , false , false , false , false , false , false , false , false , false
, false , false , false , false , false , false , false } ; PmAllocator *
alloc = pm_default_allocator ( ) ; { const int_T status =
pm_create_bool_vector_fields ( & smData -> mDirectFeedthroughVector , 6 ,
alloc ) ; checkMemAllocStatus ( status ) ; memcpy ( smData ->
mDirectFeedthroughVector . mX , directFeedthroughVector , 6 * sizeof (
boolean_T ) ) ; } { const int_T status = pm_create_bool_vector_fields ( &
smData -> mDirectFeedthroughMatrix , 108 , alloc ) ; checkMemAllocStatus (
status ) ; memcpy ( smData -> mDirectFeedthroughMatrix . mX ,
directFeedthroughMatrix , 108 * sizeof ( boolean_T ) ) ; } } static void
initOutputDerivProc ( NeDaePrivateData * smData ) { PmAllocator * alloc =
pm_default_allocator ( ) ; const int32_T outputFunctionMap [ 9 ] = { 0 , 0 ,
0 , 0 , 0 , 0 , 0 , 0 , 0 } ; smData -> mOutputFunctionMap =
pm_create_int_vector ( 9 , alloc ) ; memcpy ( smData -> mOutputFunctionMap ->
mX , outputFunctionMap , 9 * sizeof ( int32_T ) ) ; smData ->
mNumOutputClasses = 1 ; smData -> mHasKinematicOutputs = true ; smData ->
mHasDynamicOutputs = false ; smData -> mIsOutputClass0Dynamic = false ;
smData -> mDoComputeDynamicOutputs = false ; smData ->
mCachedDerivativesAvailable = false ; { size_t i = 0 ; const int_T status =
pm_create_real_vector_fields ( & smData -> mCachedDerivatives , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mCachedDerivatives . mN ; ++ i ) smData -> mCachedDerivatives .
mX [ i ] = 0.0 ; } }
#if 0
static void initializeSizePairVector ( const SmSizePair * data ,
SmSizePairVector * vector ) { const size_t n = sm_core_SmSizePairVector_size
( vector ) ; size_t i ; for ( i = 0 ; i < n ; ++ i , ++ data )
sm_core_SmSizePairVector_setValue ( vector , i , data ++ ) ; }
#endif
static void initAssemblyDelegate ( SmMechanismDelegate * delegate ) {
SmMechanismDelegateScratchpad * scratchpad = NULL ; const SmSizePair
jointToStageIdx [ 17 ] = { { 66 , 1 } , { 67 , 6 } , { 68 , 3 } , { 69 , 8 }
, { 108 , 5 } , { 109 , 10 } , { 110 , 11 } , { 111 , 12 } , { 112 , 7 } , {
113 , 2 } , { 114 , 13 } , { 115 , 14 } , { 116 , 15 } , { 117 , 16 } , { 118
, 4 } , { 119 , 9 } , { 120 , 0 } } ; const size_t primitiveIndices [ 17 + 1
] = { 0 , 4 , 6 , 7 , 9 , 10 , 11 , 13 , 14 , 16 , 17 , 18 , 19 , 20 , 21 ,
22 , 23 , 24 } ; const SmSizePair stateOffsets [ 24 ] = { { 0 , 7 } , { 1 , 8
} , { 2 , 9 } , { 3 , 10 } , { 13 , 15 } , { 14 , 16 } , { 17 , 18 } , { 19 ,
21 } , { 20 , 22 } , { 23 , 24 } , { 25 , 26 } , { 27 , 29 } , { 28 , 30 } ,
{ 31 , 32 } , { 33 , 35 } , { 34 , 36 } , { 37 , 38 } , { 39 , 40 } , { 41 ,
42 } , { 43 , 44 } , { 45 , 46 } , { 47 , 48 } , { 49 , 50 } , { 51 , 52 } }
; const SmSizePair dofOffsets [ 24 ] = { { 0 , 1 } , { 1 , 2 } , { 2 , 3 } ,
{ 3 , 6 } , { 6 , 7 } , { 7 , 8 } , { 8 , 9 } , { 9 , 10 } , { 10 , 11 } , {
11 , 12 } , { 12 , 13 } , { 13 , 14 } , { 14 , 15 } , { 15 , 16 } , { 16 , 17
} , { 17 , 18 } , { 18 , 19 } , { 19 , 20 } , { 20 , 21 } , { 21 , 22 } , {
22 , 23 } , { 23 , 24 } , { 24 , 25 } , { 25 , 26 } } ; const size_t *
flexibleStages = NULL ; const size_t remodIndices [ 12 ] = { 13 , 17 , 19 ,
23 , 27 , 31 , 33 , 37 , 45 , 47 , 49 , 51 } ; const size_t
equationsPerConstraint [ 12 ] = { 3 , 4 , 3 , 3 , 3 , 3 , 3 , 3 , 2 , 2 , 2 ,
2 } ; const size_t dofToVelSlot [ 26 ] = { 7 , 8 , 9 , 10 , 11 , 12 , 15 , 16
, 18 , 21 , 22 , 24 , 26 , 29 , 30 , 32 , 35 , 36 , 38 , 40 , 42 , 44 , 46 ,
48 , 50 , 52 } ; const size_t constraintDofs [ 34 ] = { 6 , 7 , 19 , 12 , 6 ,
7 , 8 , 21 , 15 , 13 , 14 , 12 , 13 , 14 , 20 , 9 , 10 , 11 , 16 , 17 , 18 ,
6 , 7 , 8 , 9 , 10 , 15 , 16 , 17 , 6 , 7 , 12 , 13 , 14 } ; const size_t
constraintDofOffsets [ 12 + 1 ] = { 0 , 3 , 4 , 7 , 8 , 11 , 15 , 18 , 21 ,
26 , 29 , 31 , 34 } ; const size_t Jm = 33 ; const size_t Jn = 26 ;
SmSizePair zeroSizePair ; zeroSizePair . mFirst = zeroSizePair . mSecond = 0
; sm_core_MechanismDelegate_allocScratchpad ( delegate ) ; scratchpad =
delegate -> mScratchpad ; delegate -> mTargetStrengthFree = 0 ; delegate ->
mTargetStrengthSuggested = 1 ; delegate -> mTargetStrengthDesired = 2 ;
delegate -> mTargetStrengthRequired = 3 ; delegate -> mConsistencyTol = +
1.000000000000000062e-09 ; delegate -> mTreeJointDof = 26 ; delegate -> mDof
= 26 ; delegate -> mStateSize = 69 ; delegate -> mContinuousStateSize = 69 ;
delegate -> mModeVectorSize = 0 ; delegate -> mNumStages = 17 ; delegate ->
mNumConstraints = 12 ; delegate -> mNumAllConstraintEquations = 33 ;
sm_core_SmSizePairVector_create ( & delegate -> mJointToStageIdx , 17 , &
zeroSizePair ) ; memcpy ( sm_core_SmSizePairVector_nonConstValues ( &
delegate -> mJointToStageIdx ) , jointToStageIdx , 17 * sizeof ( SmSizePair )
) ; sm_core_SmSizeTVector_create ( & delegate -> mPrimitiveIndices , delegate
-> mNumStages + 1 , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( &
delegate -> mPrimitiveIndices ) , primitiveIndices , ( delegate -> mNumStages
+ 1 ) * sizeof ( size_t ) ) ; sm_core_SmSizePairVector_create ( & delegate ->
mStateOffsets , 24 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mStateOffsets ) ,
stateOffsets , 24 * sizeof ( SmSizePair ) ) ; sm_core_SmSizePairVector_create
( & delegate -> mDofOffsets , 24 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mDofOffsets ) ,
dofOffsets , 24 * sizeof ( SmSizePair ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mFlexibleStages , 0 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mFlexibleStages ) ,
flexibleStages , 0 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mRemodIndices , 12 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mRemodIndices ) ,
remodIndices , 12 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mEquationsPerConstraint , delegate -> mNumConstraints , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mEquationsPerConstraint ) , equationsPerConstraint , delegate ->
mNumConstraints * sizeof ( size_t ) ) ; sm_core_SmIntVector_create ( &
delegate -> mRunTimeEnabledEquations , delegate -> mNumAllConstraintEquations
, 1 ) ; sm_core_SmSizeTVector_create ( & delegate -> mDofToVelSlot , delegate
-> mDof , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mDofToVelSlot ) , dofToVelSlot , delegate -> mDof * sizeof ( size_t ) ) ;
sm_core_SmSizeTVector_create ( & delegate -> mConstraintDofs , 34 , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate -> mConstraintDofs
) , constraintDofs , 34 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create
( & delegate -> mConstraintDofOffsets , delegate -> mNumConstraints + 1 , 0 )
; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mConstraintDofOffsets ) , constraintDofOffsets , ( delegate ->
mNumConstraints + 1 ) * sizeof ( size_t ) ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosRequired , 26 ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosDesired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad
-> mPosSuggested , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosFree , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosNonRequired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosSuggAndFree , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelRequired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelDesired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelSuggested , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelFree
, 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelNonRequired , 26 )
; sm_core_SmBoundedSet_create ( & scratchpad -> mVelSuggAndFree , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mConstraintFilter , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveConstraints , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs0 , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewConstraints , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewDofs , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mUnsatisfiedConstraints , 12 )
; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveConstraintsVect , 12
, 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveDofsVect , 26 ,
0 ) ; sm_core_SmSizeTVector_create ( & scratchpad -> mFullDofToActiveDof , 26
, 0 ) ; sm_core_SmSizePairVector_create ( & scratchpad ->
mPartiallyPosTargetedPrims , 24 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mPartiallyVelTargetedPrims
, 24 , & zeroSizePair ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPosPartialTypes , 24 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mVelPartialTypes , 24 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPartiallyActivePrims , 24 , 0 ) ; sm_core_SmSizePairVector_create ( &
scratchpad -> mBaseFrameVelOffsets , 1 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mCvVelOffsets , 24 , &
zeroSizePair ) ; sm_core_SmRealVector_create ( & scratchpad ->
mCvAzimuthValues , 24 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mInitialState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mStartState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mTestState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mFullStateVector , 69 , 0.0 ) ; sm_core_SmIntVector_create ( & scratchpad ->
mModeVector , 0 , 0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mJacobianRowMaj , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobian , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobianPrimSubmatrix , Jm * 6 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mConstraintNonhomoTerms , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mConstraintError , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mBestConstraintError ,
Jm , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mDeltas , Jn * (
Jm <= Jn ? Jm : Jn ) , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mSvdWork , 3239 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mLineSearchScaledDeltaVect , 26 , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mLineSearchTestStateVect , 69 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mLineSearchErrorVect , Jm , 0.0
) ; sm_core_SmRealVector_create ( & scratchpad -> mActiveDofVelsVect , 26 ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mVelSystemRhs , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mMotionData , 298 , 0.0
) ; delegate -> mSetTargets = Control_Bicopter_ae14a523_1_setTargets ;
delegate -> mResetStateVector = Control_Bicopter_ae14a523_1_resetStateVector
; delegate -> mResetModeVector = Control_Bicopter_ae14a523_1_resetModeVector
; delegate -> mInitializeTrackedAngleState =
Control_Bicopter_ae14a523_1_initializeTrackedAngleState ; delegate ->
mComputeDiscreteState = Control_Bicopter_ae14a523_1_computeDiscreteState ;
delegate -> mAdjustPosition = Control_Bicopter_ae14a523_1_adjustPosition ;
delegate -> mPerturbJointPrimitiveState =
Control_Bicopter_ae14a523_1_perturbJointPrimitiveState ; delegate ->
mPerturbFlexibleBodyState =
Control_Bicopter_ae14a523_1_perturbFlexibleBodyState ; delegate ->
mComputeDofBlendMatrix = Control_Bicopter_ae14a523_1_computeDofBlendMatrix ;
delegate -> mProjectPartiallyTargetedPos =
Control_Bicopter_ae14a523_1_projectPartiallyTargetedPos ; delegate ->
mPropagateMotion = Control_Bicopter_ae14a523_1_propagateMotion ; delegate ->
mComputeAssemblyError = Control_Bicopter_ae14a523_1_computeAssemblyError ;
delegate -> mComputeAssemblyJacobian =
Control_Bicopter_ae14a523_1_computeAssemblyJacobian ; delegate ->
mComputeFullAssemblyJacobian =
Control_Bicopter_ae14a523_1_computeFullAssemblyJacobian ; delegate ->
mIsInKinematicSingularity =
Control_Bicopter_ae14a523_1_isInKinematicSingularity ; delegate ->
mConvertStateVector = Control_Bicopter_ae14a523_1_convertStateVector ;
delegate -> mConstructStateVector =
Control_Bicopter_ae14a523_1_constructStateVector ; delegate ->
mExtractSolverStateVector =
Control_Bicopter_ae14a523_1_extractSolverStateVector ; delegate ->
mIsPositionViolation = Control_Bicopter_ae14a523_1_isPositionViolation ;
delegate -> mIsVelocityViolation =
Control_Bicopter_ae14a523_1_isVelocityViolation ; delegate ->
mProjectStateSim = Control_Bicopter_ae14a523_1_projectStateSim ; delegate ->
mComputeConstraintError = Control_Bicopter_ae14a523_1_computeConstraintError
; delegate -> mMech = NULL ; } static void initSimulationDelegate (
SmMechanismDelegate * delegate ) { SmMechanismDelegateScratchpad * scratchpad
= NULL ; const SmSizePair jointToStageIdx [ 17 ] = { { 66 , 1 } , { 67 , 6 }
, { 68 , 3 } , { 69 , 8 } , { 108 , 5 } , { 109 , 10 } , { 110 , 11 } , { 111
, 12 } , { 112 , 7 } , { 113 , 2 } , { 114 , 13 } , { 115 , 14 } , { 116 , 15
} , { 117 , 16 } , { 118 , 4 } , { 119 , 9 } , { 120 , 0 } } ; const size_t
primitiveIndices [ 17 + 1 ] = { 0 , 4 , 6 , 7 , 9 , 10 , 11 , 13 , 14 , 16 ,
17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 } ; const SmSizePair stateOffsets [ 24
] = { { 0 , 7 } , { 1 , 8 } , { 2 , 9 } , { 3 , 10 } , { 13 , 15 } , { 14 ,
16 } , { 17 , 18 } , { 19 , 21 } , { 20 , 22 } , { 23 , 24 } , { 25 , 26 } ,
{ 27 , 29 } , { 28 , 30 } , { 31 , 32 } , { 33 , 35 } , { 34 , 36 } , { 37 ,
38 } , { 39 , 40 } , { 41 , 42 } , { 43 , 44 } , { 45 , 46 } , { 47 , 48 } ,
{ 49 , 50 } , { 51 , 52 } } ; const SmSizePair dofOffsets [ 24 ] = { { 0 , 1
} , { 1 , 2 } , { 2 , 3 } , { 3 , 6 } , { 6 , 7 } , { 7 , 8 } , { 8 , 9 } , {
9 , 10 } , { 10 , 11 } , { 11 , 12 } , { 12 , 13 } , { 13 , 14 } , { 14 , 15
} , { 15 , 16 } , { 16 , 17 } , { 17 , 18 } , { 18 , 19 } , { 19 , 20 } , {
20 , 21 } , { 21 , 22 } , { 22 , 23 } , { 23 , 24 } , { 24 , 25 } , { 25 , 26
} } ; const size_t * flexibleStages = NULL ; const size_t remodIndices [ 12 ]
= { 13 , 17 , 19 , 23 , 27 , 31 , 33 , 37 , 45 , 47 , 49 , 51 } ; const
size_t equationsPerConstraint [ 12 ] = { 1 , 1 , 3 , 1 , 1 , 1 , 1 , 1 , 2 ,
2 , 1 , 1 } ; const size_t dofToVelSlot [ 26 ] = { 7 , 8 , 9 , 10 , 11 , 12 ,
15 , 16 , 18 , 21 , 22 , 24 , 26 , 29 , 30 , 32 , 35 , 36 , 38 , 40 , 42 , 44
, 46 , 48 , 50 , 52 } ; const size_t constraintDofs [ 34 ] = { 6 , 7 , 19 ,
12 , 6 , 7 , 8 , 21 , 15 , 13 , 14 , 12 , 13 , 14 , 20 , 9 , 10 , 11 , 16 ,
17 , 18 , 6 , 7 , 8 , 9 , 10 , 15 , 16 , 17 , 6 , 7 , 12 , 13 , 14 } ; const
size_t constraintDofOffsets [ 12 + 1 ] = { 0 , 3 , 4 , 7 , 8 , 11 , 15 , 18 ,
21 , 26 , 29 , 31 , 34 } ; const size_t Jm = 16 ; const size_t Jn = 26 ;
SmSizePair zeroSizePair ; zeroSizePair . mFirst = zeroSizePair . mSecond = 0
; sm_core_MechanismDelegate_allocScratchpad ( delegate ) ; scratchpad =
delegate -> mScratchpad ; delegate -> mTargetStrengthFree = 0 ; delegate ->
mTargetStrengthSuggested = 1 ; delegate -> mTargetStrengthDesired = 2 ;
delegate -> mTargetStrengthRequired = 3 ; delegate -> mConsistencyTol = +
1.000000000000000062e-09 ; delegate -> mTreeJointDof = 26 ; delegate -> mDof
= 26 ; delegate -> mStateSize = 69 ; delegate -> mContinuousStateSize = 69 ;
delegate -> mModeVectorSize = 0 ; delegate -> mNumStages = 17 ; delegate ->
mNumConstraints = 12 ; delegate -> mNumAllConstraintEquations = 16 ;
sm_core_SmSizePairVector_create ( & delegate -> mJointToStageIdx , 17 , &
zeroSizePair ) ; memcpy ( sm_core_SmSizePairVector_nonConstValues ( &
delegate -> mJointToStageIdx ) , jointToStageIdx , 17 * sizeof ( SmSizePair )
) ; sm_core_SmSizeTVector_create ( & delegate -> mPrimitiveIndices , delegate
-> mNumStages + 1 , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( &
delegate -> mPrimitiveIndices ) , primitiveIndices , ( delegate -> mNumStages
+ 1 ) * sizeof ( size_t ) ) ; sm_core_SmSizePairVector_create ( & delegate ->
mStateOffsets , 24 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mStateOffsets ) ,
stateOffsets , 24 * sizeof ( SmSizePair ) ) ; sm_core_SmSizePairVector_create
( & delegate -> mDofOffsets , 24 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mDofOffsets ) ,
dofOffsets , 24 * sizeof ( SmSizePair ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mFlexibleStages , 0 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mFlexibleStages ) ,
flexibleStages , 0 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mRemodIndices , 12 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mRemodIndices ) ,
remodIndices , 12 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mEquationsPerConstraint , delegate -> mNumConstraints , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mEquationsPerConstraint ) , equationsPerConstraint , delegate ->
mNumConstraints * sizeof ( size_t ) ) ; sm_core_SmIntVector_create ( &
delegate -> mRunTimeEnabledEquations , delegate -> mNumAllConstraintEquations
, 1 ) ; sm_core_SmSizeTVector_create ( & delegate -> mDofToVelSlot , delegate
-> mDof , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mDofToVelSlot ) , dofToVelSlot , delegate -> mDof * sizeof ( size_t ) ) ;
sm_core_SmSizeTVector_create ( & delegate -> mConstraintDofs , 34 , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate -> mConstraintDofs
) , constraintDofs , 34 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create
( & delegate -> mConstraintDofOffsets , delegate -> mNumConstraints + 1 , 0 )
; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mConstraintDofOffsets ) , constraintDofOffsets , ( delegate ->
mNumConstraints + 1 ) * sizeof ( size_t ) ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosRequired , 26 ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosDesired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad
-> mPosSuggested , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosFree , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosNonRequired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosSuggAndFree , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelRequired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelDesired , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelSuggested , 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelFree
, 26 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelNonRequired , 26 )
; sm_core_SmBoundedSet_create ( & scratchpad -> mVelSuggAndFree , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mConstraintFilter , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveConstraints , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs0 , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewConstraints , 12 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewDofs , 26 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mUnsatisfiedConstraints , 12 )
; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveConstraintsVect , 12
, 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveDofsVect , 26 ,
0 ) ; sm_core_SmSizeTVector_create ( & scratchpad -> mFullDofToActiveDof , 26
, 0 ) ; sm_core_SmSizePairVector_create ( & scratchpad ->
mPartiallyPosTargetedPrims , 24 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mPartiallyVelTargetedPrims
, 24 , & zeroSizePair ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPosPartialTypes , 24 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mVelPartialTypes , 24 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPartiallyActivePrims , 24 , 0 ) ; sm_core_SmSizePairVector_create ( &
scratchpad -> mBaseFrameVelOffsets , 1 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mCvVelOffsets , 24 , &
zeroSizePair ) ; sm_core_SmRealVector_create ( & scratchpad ->
mCvAzimuthValues , 24 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mInitialState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mStartState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mTestState , 69 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mFullStateVector , 69 , 0.0 ) ; sm_core_SmIntVector_create ( & scratchpad ->
mModeVector , 0 , 0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mJacobianRowMaj , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobian , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobianPrimSubmatrix , Jm * 6 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mConstraintNonhomoTerms , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mConstraintError , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mBestConstraintError ,
Jm , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mDeltas , Jn * (
Jm <= Jn ? Jm : Jn ) , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mSvdWork , 1881 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mLineSearchScaledDeltaVect , 26 , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mLineSearchTestStateVect , 69 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mLineSearchErrorVect , Jm , 0.0
) ; sm_core_SmRealVector_create ( & scratchpad -> mActiveDofVelsVect , 26 ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mVelSystemRhs , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mMotionData , 298 , 0.0
) ; delegate -> mSetTargets = NULL ; delegate -> mResetStateVector =
Control_Bicopter_ae14a523_1_resetStateVector ; delegate -> mResetModeVector =
Control_Bicopter_ae14a523_1_resetModeVector ; delegate ->
mInitializeTrackedAngleState =
Control_Bicopter_ae14a523_1_initializeTrackedAngleState ; delegate ->
mComputeDiscreteState = Control_Bicopter_ae14a523_1_computeDiscreteState ;
delegate -> mAdjustPosition = Control_Bicopter_ae14a523_1_adjustPosition ;
delegate -> mPerturbJointPrimitiveState =
Control_Bicopter_ae14a523_1_perturbJointPrimitiveState ; delegate ->
mPerturbFlexibleBodyState =
Control_Bicopter_ae14a523_1_perturbFlexibleBodyState ; delegate ->
mComputeDofBlendMatrix = NULL ; delegate -> mProjectPartiallyTargetedPos =
NULL ; delegate -> mPropagateMotion =
Control_Bicopter_ae14a523_1_propagateMotion ; delegate ->
mComputeAssemblyError = Control_Bicopter_ae14a523_1_computeAssemblyError ;
delegate -> mComputeAssemblyJacobian =
Control_Bicopter_ae14a523_1_computeAssemblyJacobian ; delegate ->
mComputeFullAssemblyJacobian =
Control_Bicopter_ae14a523_1_computeFullAssemblyJacobian ; delegate ->
mIsInKinematicSingularity =
Control_Bicopter_ae14a523_1_isInKinematicSingularity ; delegate ->
mConvertStateVector = Control_Bicopter_ae14a523_1_convertStateVector ;
delegate -> mConstructStateVector =
Control_Bicopter_ae14a523_1_constructStateVector ; delegate ->
mExtractSolverStateVector =
Control_Bicopter_ae14a523_1_extractSolverStateVector ; delegate ->
mIsPositionViolation = Control_Bicopter_ae14a523_1_isPositionViolation ;
delegate -> mIsVelocityViolation =
Control_Bicopter_ae14a523_1_isVelocityViolation ; delegate ->
mProjectStateSim = Control_Bicopter_ae14a523_1_projectStateSim ; delegate ->
mComputeConstraintError = Control_Bicopter_ae14a523_1_computeConstraintError
; delegate -> mMech = NULL ; } static void initMechanismDelegates (
NeDaePrivateData * smData ) { PmAllocator * alloc = pm_default_allocator ( )
; const int32_T motionInputOffsets [ 4 ] = { 1 , 2 , 0 , 3 } ; int_T status =
0 ; initAssemblyDelegate ( & smData -> mAssemblyDelegate ) ;
initSimulationDelegate ( & smData -> mSimulationDelegate ) ; status =
pm_create_int_vector_fields ( & smData -> mMotionInputOffsets , smData ->
mNumInputMotionPrimitives , alloc ) ; checkMemAllocStatus ( status ) ; memcpy
( smData -> mMotionInputOffsets . mX , motionInputOffsets , 4 * sizeof (
int32_T ) ) ; } static void initComputationFcnPtrs ( NeDaePrivateData *
smData ) { smData -> mSetParametersFcn = dae_cg_setParameters_function ;
smData -> mPAssertFcn = dae_cg_pAssert_method ; smData -> mDerivativeFcn =
dae_cg_deriv_method ; smData -> mOutputFcn = dae_cg_output_method ; smData ->
mModeFcn = dae_cg_mode_method ; smData -> mZeroCrossingFcn =
dae_cg_zeroCrossing_method ; smData -> mProjectionFcn = dae_cg_project_solve
; smData -> mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve ; smData -> mCheckFcn = (
smData -> mStateVectorSize == 0 ) ? dae_cg_check_solve : NULL ; smData ->
mAssemblyFcn = dae_cg_assemble_solve ; smData -> mSetupLoggerFcn =
sm_ssci_setupLoggerFcn_codeGen ; smData -> mLogFcn = sm_ssci_logFcn_codeGen ;
smData -> mResidualsFcn = NULL ; smData -> mLinearizeFcn = NULL ; smData ->
mGenerateFcn = NULL ; } static void initLiveLinkToSm ( NeDaePrivateData *
smData ) { smData -> mLiveSmLink = NULL ; smData -> mLiveSmLink_destroy =
NULL ; smData -> mLiveSmLink_copy = NULL ; } void
Control_Bicopter_ae14a523_1_NeDaePrivateData_create ( NeDaePrivateData *
smData ) { initBasicAttributes ( smData ) ; initStateVector ( smData ) ;
initAsserts ( smData ) ; initModeVector ( smData ) ; initZeroCrossings (
smData ) ; initVariables ( smData ) ; initRuntimeParameters ( smData ) ;
initIoInfo ( smData ) ; initInputDerivs ( smData ) ; initDirectFeedthrough (
smData ) ; initOutputDerivProc ( smData ) ; initMechanismDelegates ( smData )
; initComputationFcnPtrs ( smData ) ; initLiveLinkToSm ( smData ) ; }
