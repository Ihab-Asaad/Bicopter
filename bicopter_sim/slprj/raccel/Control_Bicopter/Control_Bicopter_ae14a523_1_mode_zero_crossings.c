#include "__cf_Control_Bicopter.h"
#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
const NeZCData * Control_Bicopter_ae14a523_1_ZCData = NULL ; PmfMessageId
Control_Bicopter_ae14a523_1_computeAsmModeVector ( const double * input ,
const double * inputDot , const double * inputDdot , int * modeVector ,
double * errorResult , NeuDiagnosticManager * neDiagMgr ) { ( void ) input ;
( void ) inputDot ; ( void ) inputDdot ; ( void ) modeVector ; ( void )
neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; } PmfMessageId
Control_Bicopter_ae14a523_1_computeSimModeVector ( const double * input ,
const double * inputDot , const double * inputDdot , int * modeVector ,
double * errorResult , NeuDiagnosticManager * neDiagMgr ) { ( void ) input ;
( void ) inputDot ; ( void ) inputDdot ; ( void ) modeVector ; ( void )
neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; } void
Control_Bicopter_ae14a523_1_resetModeVector ( const void * mech , int *
modeVector ) { ( void ) mech ; ( void ) modeVector ; } PmfMessageId
Control_Bicopter_ae14a523_1_onModeChanged ( const double * input , const
double * inputDot , const double * inputDdot , const int * prevModeVector ,
int * modeVector , double * solverStateVector , double * discreteStateVector
, double * errorResult , NeuDiagnosticManager * neDiagMgr ) { ( void ) input
; ( void ) inputDot ; ( void ) inputDdot ; ( void ) prevModeVector ; ( void )
modeVector ; ( void ) solverStateVector ; ( void ) discreteStateVector ; (
void ) neDiagMgr ; errorResult [ 0 ] = 0.0 ; return NULL ; } PmfMessageId
Control_Bicopter_ae14a523_1_computeZeroCrossings ( const double * input ,
const double * inputDot , const double * inputDdot , double *
zeroCrossingsVector , double * errorResult , NeuDiagnosticManager * neDiagMgr
) { ( void ) input ; ( void ) inputDot ; ( void ) inputDdot ; ( void )
zeroCrossingsVector ; ( void ) neDiagMgr ; errorResult [ 0 ] = 0.0 ; return
NULL ; }
