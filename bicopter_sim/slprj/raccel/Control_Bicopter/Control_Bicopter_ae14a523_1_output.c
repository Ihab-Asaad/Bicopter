#include "__cf_Control_Bicopter.h"
#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "Control_Bicopter_ae14a523_1_geometries.h"
PmfMessageId Control_Bicopter_ae14a523_1_output ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * input , const double * inputDot , const double *
inputDdot , const double * discreteState , double * output ,
NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) modeVector ; ( void ) input ; ( void ) inputDdot ; (
void ) discreteState ; ( void ) neDiagMgr ; output [ 0 ] = inputDot [ 2 ] ;
output [ 1 ] = inputDot [ 3 ] ; output [ 2 ] = state [ 0 ] ; output [ 3 ] =
state [ 1 ] ; output [ 4 ] = state [ 2 ] ; output [ 5 ] = state [ 3 ] ;
output [ 6 ] = state [ 4 ] ; output [ 7 ] = state [ 5 ] ; output [ 8 ] =
state [ 6 ] ; return NULL ; }
