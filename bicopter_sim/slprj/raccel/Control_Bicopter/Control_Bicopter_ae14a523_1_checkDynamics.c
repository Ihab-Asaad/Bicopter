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
PmfMessageId Control_Bicopter_ae14a523_1_checkDynamics ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const double *
input , const double * inputDot , const double * inputDdot , const double *
discreteState , double * result , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 21 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) state ; ( void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; (
void ) neDiagMgr ; xx [ 0 ] = - 3.865146756434337e-8 ; xx [ 1 ] = -
5.289307885790401e-3 ; xx [ 2 ] = - 3.071851004035524e-7 ; xx [ 3 ] = 2.0 ;
xx [ 4 ] = 0.3301830145835319 ; xx [ 5 ] = 8.736479765048067e-7 ; xx [ 6 ] =
xx [ 5 ] * input [ 4 ] ; xx [ 7 ] = 0.9439169332518749 ; xx [ 8 ] =
3.787473687847524e-7 ; xx [ 9 ] = xx [ 8 ] * input [ 4 ] ; xx [ 10 ] = xx [ 3
] * ( xx [ 4 ] * xx [ 6 ] - xx [ 7 ] * xx [ 9 ] ) ; xx [ 11 ] = input [ 4 ] -
( xx [ 8 ] * xx [ 9 ] + xx [ 5 ] * xx [ 6 ] ) * xx [ 3 ] ; xx [ 12 ] = ( xx [
7 ] * xx [ 6 ] + xx [ 4 ] * xx [ 9 ] ) * xx [ 3 ] ; pm_math_Vector3_cross_ra
( xx + 0 , xx + 10 , xx + 13 ) ; xx [ 6 ] = xx [ 5 ] * input [ 5 ] ; xx [ 9 ]
= xx [ 8 ] * input [ 5 ] ; xx [ 16 ] = xx [ 3 ] * ( xx [ 4 ] * xx [ 6 ] - xx
[ 7 ] * xx [ 9 ] ) ; xx [ 17 ] = input [ 5 ] - ( xx [ 8 ] * xx [ 9 ] + xx [ 5
] * xx [ 6 ] ) * xx [ 3 ] ; xx [ 5 ] = ( xx [ 7 ] * xx [ 6 ] + xx [ 4 ] * xx
[ 9 ] ) * xx [ 3 ] ; xx [ 6 ] = xx [ 16 ] ; xx [ 7 ] = xx [ 17 ] ; xx [ 8 ] =
xx [ 5 ] ; pm_math_Vector3_cross_ra ( xx + 0 , xx + 6 , xx + 18 ) ; xx [ 0 ]
= xx [ 13 ] + xx [ 18 ] ; xx [ 1 ] = xx [ 14 ] + xx [ 19 ] ; xx [ 2 ] = xx [
15 ] + xx [ 20 ] ; xx [ 3 ] = xx [ 10 ] + xx [ 16 ] ; xx [ 4 ] = xx [ 11 ] +
xx [ 17 ] ; xx [ 6 ] = xx [ 12 ] + xx [ 5 ] ; result [ 0 ] = xx [ 0 ] * xx [
0 ] + xx [ 1 ] * xx [ 1 ] + xx [ 2 ] * xx [ 2 ] + xx [ 3 ] * xx [ 3 ] + xx [
4 ] * xx [ 4 ] + xx [ 6 ] * xx [ 6 ] ; return NULL ; }
