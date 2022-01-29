#include "__cf_Control_Bicopter.h"
#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"
boolean_T Control_Bicopter_ae14a523_1_getTargetStatus ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const size_t
targetIndex ) { return true ; } void Control_Bicopter_ae14a523_1_setTargets (
const RuntimeDerivedValuesBundle * rtdv , CTarget * targets ) { ( void ) rtdv
; ( void ) targets ; } void Control_Bicopter_ae14a523_1_resetStateVector (
const void * mech , double * state ) { double xx [ 1 ] ; ( void ) mech ; xx [
0 ] = 0.0 ; state [ 0 ] = xx [ 0 ] ; state [ 1 ] = xx [ 0 ] ; state [ 2 ] =
xx [ 0 ] ; state [ 3 ] = 1.0 ; state [ 4 ] = xx [ 0 ] ; state [ 5 ] = xx [ 0
] ; state [ 6 ] = xx [ 0 ] ; state [ 7 ] = xx [ 0 ] ; state [ 8 ] = xx [ 0 ]
; state [ 9 ] = xx [ 0 ] ; state [ 10 ] = xx [ 0 ] ; state [ 11 ] = xx [ 0 ]
; state [ 12 ] = xx [ 0 ] ; state [ 13 ] = xx [ 0 ] ; state [ 14 ] = xx [ 0 ]
; state [ 15 ] = xx [ 0 ] ; state [ 16 ] = xx [ 0 ] ; state [ 17 ] = xx [ 0 ]
; state [ 18 ] = xx [ 0 ] ; state [ 19 ] = xx [ 0 ] ; state [ 20 ] = xx [ 0 ]
; state [ 21 ] = xx [ 0 ] ; state [ 22 ] = xx [ 0 ] ; state [ 23 ] = xx [ 0 ]
; state [ 24 ] = xx [ 0 ] ; state [ 25 ] = xx [ 0 ] ; state [ 26 ] = xx [ 0 ]
; state [ 27 ] = xx [ 0 ] ; state [ 28 ] = xx [ 0 ] ; state [ 29 ] = xx [ 0 ]
; state [ 30 ] = xx [ 0 ] ; state [ 31 ] = xx [ 0 ] ; state [ 32 ] = xx [ 0 ]
; state [ 33 ] = xx [ 0 ] ; state [ 34 ] = xx [ 0 ] ; state [ 35 ] = xx [ 0 ]
; state [ 36 ] = xx [ 0 ] ; state [ 37 ] = xx [ 0 ] ; state [ 38 ] = xx [ 0 ]
; state [ 39 ] = xx [ 0 ] ; state [ 40 ] = xx [ 0 ] ; state [ 41 ] = xx [ 0 ]
; state [ 42 ] = xx [ 0 ] ; state [ 43 ] = xx [ 0 ] ; state [ 44 ] = xx [ 0 ]
; state [ 45 ] = xx [ 0 ] ; state [ 46 ] = xx [ 0 ] ; state [ 47 ] = xx [ 0 ]
; state [ 48 ] = xx [ 0 ] ; state [ 49 ] = xx [ 0 ] ; state [ 50 ] = xx [ 0 ]
; state [ 51 ] = xx [ 0 ] ; state [ 52 ] = xx [ 0 ] ; state [ 53 ] = xx [ 0 ]
; state [ 54 ] = xx [ 0 ] ; state [ 55 ] = xx [ 0 ] ; state [ 56 ] = xx [ 0 ]
; state [ 57 ] = xx [ 0 ] ; state [ 58 ] = xx [ 0 ] ; state [ 59 ] = xx [ 0 ]
; state [ 60 ] = xx [ 0 ] ; state [ 61 ] = xx [ 0 ] ; state [ 62 ] = xx [ 0 ]
; state [ 63 ] = xx [ 0 ] ; state [ 64 ] = xx [ 0 ] ; state [ 65 ] = xx [ 0 ]
; state [ 66 ] = xx [ 0 ] ; state [ 67 ] = xx [ 0 ] ; state [ 68 ] = xx [ 0 ]
; } static PmfMessageId initializeTrackedAngleState_0 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 19 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = - 0.9915516631459343 ; xx [ 1
] = 0.1297122054176751 ; xx [ 2 ] = - 1.971782056416126e-4 ; xx [ 3 ] = -
6.47994571923356e-5 ; xx [ 4 ] = motionData [ 7 ] ; xx [ 5 ] = motionData [ 8
] ; xx [ 6 ] = motionData [ 9 ] ; xx [ 7 ] = motionData [ 10 ] ; xx [ 8 ] =
motionData [ 70 ] ; xx [ 9 ] = motionData [ 71 ] ; xx [ 10 ] = motionData [
72 ] ; xx [ 11 ] = motionData [ 73 ] ; pm_math_Quaternion_inverseCompose_ra (
xx + 4 , xx + 8 , xx + 12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 ,
xx + 12 , xx + 4 ) ; xx [ 0 ] = 3.974634836311581e-3 ; xx [ 1 ] =
0.9999921011077627 ; xx [ 8 ] = motionData [ 202 ] ; xx [ 9 ] = motionData [
203 ] ; xx [ 10 ] = motionData [ 204 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 12 , xx + 8 , xx + 16 ) ; state [ 53 ] = sm_core_canonicalAngle ( 2.0 *
atan2 ( sqrt ( xx [ 5 ] * xx [ 5 ] + xx [ 6 ] * xx [ 6 ] + xx [ 7 ] * xx [ 7
] ) , fabs ( xx [ 4 ] ) ) * ( ( ( xx [ 0 ] * xx [ 6 ] + xx [ 1 ] * xx [ 7 ] )
* xx [ 4 ] ) < 0.0 ? - 1.0 : + 1.0 ) ) ; state [ 54 ] = xx [ 0 ] * (
motionData [ 257 ] - xx [ 17 ] ) + xx [ 1 ] * ( motionData [ 258 ] - xx [ 18
] ) ; return NULL ; } static PmfMessageId initializeTrackedAngleState_1 (
const RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const
double * motionData , double * state , NeuDiagnosticManager * neDiagMgr ) {
const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv
-> mInts . mValues ; double xx [ 18 ] ; ( void ) rtdvd ; ( void ) rtdvi ; (
void ) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = 0.9229115780805052 ; xx [
1 ] = - 0.0208550327305835 ; xx [ 2 ] = 0.3825441798635174 ; xx [ 3 ] =
0.03819996213750993 ; xx [ 4 ] = - motionData [ 35 ] ; xx [ 5 ] = motionData
[ 36 ] ; xx [ 6 ] = motionData [ 37 ] ; xx [ 7 ] = motionData [ 38 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [ 0 ]
= - 0.7077022283680725 ; xx [ 1 ] = - 9.268355963184496e-3 ; xx [ 2 ] =
0.7064500361247104 ; xx [ 12 ] = motionData [ 226 ] ; xx [ 13 ] = motionData
[ 227 ] ; xx [ 14 ] = motionData [ 228 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 4 , xx + 12 , xx + 15 ) ; xx [ 3 ] = motionData [ 196 ] - xx [ 15 ] ;
xx [ 4 ] = motionData [ 197 ] - xx [ 16 ] ; xx [ 5 ] = motionData [ 198 ] -
xx [ 17 ] ; state [ 55 ] = sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [
9 ] * xx [ 9 ] + xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) , fabs ( xx
[ 8 ] ) ) * ( ( pm_math_Vector3_dot_ra ( xx + 9 , xx + 0 ) * xx [ 8 ] ) < 0.0
? - 1.0 : + 1.0 ) ) ; state [ 56 ] = pm_math_Vector3_dot_ra ( xx + 3 , xx + 0
) ; return NULL ; } static PmfMessageId initializeTrackedAngleState_2 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 18 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = - 0.238473190515375 ; xx [ 1 ]
= 0.3037535947864483 ; xx [ 2 ] = 0.6325302387266769 ; xx [ 3 ] =
0.6713939142977852 ; xx [ 4 ] = - motionData [ 119 ] ; xx [ 5 ] = -
motionData [ 120 ] ; xx [ 6 ] = - motionData [ 121 ] ; xx [ 7 ] = -
motionData [ 122 ] ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 4 ,
xx + 8 ) ; xx [ 0 ] = 0.9972896958011036 ; xx [ 1 ] = - 8.852854494761919e-5
; xx [ 2 ] = - 0.0735748245777017 ; xx [ 12 ] = motionData [ 196 ] ; xx [ 13
] = motionData [ 197 ] ; xx [ 14 ] = motionData [ 198 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 4 , xx + 12 , xx + 15 ) ; xx [ 3 ]
= motionData [ 208 ] - xx [ 15 ] ; xx [ 4 ] = motionData [ 209 ] - xx [ 16 ]
; xx [ 5 ] = motionData [ 210 ] - xx [ 17 ] ; state [ 57 ] =
sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [ 9 ] * xx [ 9 ] + xx [ 10 ]
* xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) , fabs ( xx [ 8 ] ) ) * ( (
pm_math_Vector3_dot_ra ( xx + 9 , xx + 0 ) * xx [ 8 ] ) < 0.0 ? - 1.0 : + 1.0
) ) ; state [ 58 ] = pm_math_Vector3_dot_ra ( xx + 3 , xx + 0 ) ; return NULL
; } static PmfMessageId initializeTrackedAngleState_3 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 18 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = 0.2934250921235846 ; xx [ 1 ]
= 0.2477015361584666 ; xx [ 2 ] = - 0.6382199804911395 ; xx [ 3 ] = -
0.6672487697994596 ; xx [ 4 ] = - motionData [ 84 ] ; xx [ 5 ] = motionData [
85 ] ; xx [ 6 ] = motionData [ 86 ] ; xx [ 7 ] = motionData [ 87 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [ 0 ]
= - 0.7077023583577413 ; xx [ 1 ] = - 9.268340058974256e-3 ; xx [ 2 ] =
0.7064499061132662 ; xx [ 12 ] = motionData [ 268 ] ; xx [ 13 ] = motionData
[ 269 ] ; xx [ 14 ] = motionData [ 270 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 4 , xx + 12 , xx + 15 ) ; xx [ 3 ] = motionData [ 196 ] - xx [ 15 ] ;
xx [ 4 ] = motionData [ 197 ] - xx [ 16 ] ; xx [ 5 ] = motionData [ 198 ] -
xx [ 17 ] ; state [ 59 ] = sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [
9 ] * xx [ 9 ] + xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) , fabs ( xx
[ 8 ] ) ) * ( ( pm_math_Vector3_dot_ra ( xx + 9 , xx + 0 ) * xx [ 8 ] ) < 0.0
? - 1.0 : + 1.0 ) ) ; state [ 60 ] = pm_math_Vector3_dot_ra ( xx + 3 , xx + 0
) ; return NULL ; } static PmfMessageId initializeTrackedAngleState_4 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 19 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = 0.4090278622146113 ; xx [ 1 ]
= 0.5769885451833041 ; xx [ 2 ] = 0.5768689378113487 ; xx [ 3 ] =
0.4086595835751388 ; xx [ 4 ] = motionData [ 49 ] ; xx [ 5 ] = motionData [
50 ] ; xx [ 6 ] = motionData [ 51 ] ; xx [ 7 ] = motionData [ 52 ] ; xx [ 8 ]
= motionData [ 42 ] ; xx [ 9 ] = motionData [ 43 ] ; xx [ 10 ] = motionData [
44 ] ; xx [ 11 ] = motionData [ 45 ] ; pm_math_Quaternion_inverseCompose_ra (
xx + 4 , xx + 8 , xx + 12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 ,
xx + 12 , xx + 4 ) ; xx [ 0 ] = - 3.734971681712052e-4 ; xx [ 1 ] = -
0.2610715860287324 ; xx [ 2 ] = - 0.965319370710185 ; xx [ 8 ] = motionData [
238 ] ; xx [ 9 ] = motionData [ 239 ] ; xx [ 10 ] = motionData [ 240 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 12 , xx + 8 , xx + 16 ) ; xx [ 8 ]
= motionData [ 232 ] - xx [ 16 ] ; xx [ 9 ] = motionData [ 233 ] - xx [ 17 ]
; xx [ 10 ] = motionData [ 234 ] - xx [ 18 ] ; state [ 61 ] =
sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [ 5 ] * xx [ 5 ] + xx [ 6 ]
* xx [ 6 ] + xx [ 7 ] * xx [ 7 ] ) , fabs ( xx [ 4 ] ) ) * ( (
pm_math_Vector3_dot_ra ( xx + 5 , xx + 0 ) * xx [ 4 ] ) < 0.0 ? - 1.0 : + 1.0
) ) ; state [ 62 ] = pm_math_Vector3_dot_ra ( xx + 8 , xx + 0 ) ; return NULL
; } static PmfMessageId initializeTrackedAngleState_5 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 19 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = - 0.9915516631640993 ; xx [ 1
] = 0.1297122054116248 ; xx [ 2 ] = - 1.970867658312426e-4 ; xx [ 3 ] = -
6.481178896480824e-5 ; xx [ 4 ] = motionData [ 126 ] ; xx [ 5 ] = motionData
[ 127 ] ; xx [ 6 ] = motionData [ 128 ] ; xx [ 7 ] = motionData [ 129 ] ; xx
[ 8 ] = motionData [ 77 ] ; xx [ 9 ] = motionData [ 78 ] ; xx [ 10 ] =
motionData [ 79 ] ; xx [ 11 ] = motionData [ 80 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 4 , xx + 8 , xx + 12 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 12 , xx + 4 ) ; xx [ 0 ]
= 1.845352212151208e-7 ; xx [ 1 ] = 3.974634836310293e-3 ; xx [ 2 ] =
0.9999921011077457 ; xx [ 8 ] = motionData [ 232 ] ; xx [ 9 ] = motionData [
233 ] ; xx [ 10 ] = motionData [ 234 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 12 , xx + 8 , xx + 16 ) ; xx [ 8 ] = motionData [ 262 ] - xx [ 16 ] ; xx
[ 9 ] = motionData [ 263 ] - xx [ 17 ] ; xx [ 10 ] = motionData [ 264 ] - xx
[ 18 ] ; state [ 63 ] = sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [ 5
] * xx [ 5 ] + xx [ 6 ] * xx [ 6 ] + xx [ 7 ] * xx [ 7 ] ) , fabs ( xx [ 4 ]
) ) * ( ( pm_math_Vector3_dot_ra ( xx + 5 , xx + 0 ) * xx [ 4 ] ) < 0.0 ? -
1.0 : + 1.0 ) ) ; state [ 64 ] = pm_math_Vector3_dot_ra ( xx + 8 , xx + 0 ) ;
return NULL ; } static PmfMessageId initializeTrackedAngleState_6 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 19 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = - 0.9384577811996331 ; xx [ 1
] = - 2.456555942788957e-3 ; xx [ 2 ] = 0.3453844350474496 ; xx [ 3 ] =
7.41798972730745e-4 ; xx [ 4 ] = motionData [ 21 ] ; xx [ 5 ] = motionData [
22 ] ; xx [ 6 ] = motionData [ 23 ] ; xx [ 7 ] = motionData [ 24 ] ; xx [ 8 ]
= motionData [ 28 ] ; xx [ 9 ] = motionData [ 29 ] ; xx [ 10 ] = motionData [
30 ] ; xx [ 11 ] = motionData [ 31 ] ; pm_math_Quaternion_inverseCompose_ra (
xx + 4 , xx + 8 , xx + 12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 ,
xx + 12 , xx + 4 ) ; xx [ 0 ] = - 1.380846652021539e-7 ; xx [ 1 ] =
0.9999999999981868 ; xx [ 2 ] = 1.899414133421651e-6 ; xx [ 8 ] = motionData
[ 214 ] ; xx [ 9 ] = motionData [ 215 ] ; xx [ 10 ] = motionData [ 216 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 12 , xx + 8 , xx + 16 ) ; xx [ 8 ]
= motionData [ 220 ] - xx [ 16 ] ; xx [ 9 ] = motionData [ 221 ] - xx [ 17 ]
; xx [ 10 ] = motionData [ 222 ] - xx [ 18 ] ; state [ 65 ] =
sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [ 5 ] * xx [ 5 ] + xx [ 6 ]
* xx [ 6 ] + xx [ 7 ] * xx [ 7 ] ) , fabs ( xx [ 4 ] ) ) * ( (
pm_math_Vector3_dot_ra ( xx + 5 , xx + 0 ) * xx [ 4 ] ) < 0.0 ? - 1.0 : + 1.0
) ) ; state [ 66 ] = pm_math_Vector3_dot_ra ( xx + 8 , xx + 0 ) ; return NULL
; } static PmfMessageId initializeTrackedAngleState_7 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData , double * state , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 19 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) neDiagMgr ; xx [ 0 ] = - 9.516740341210528e-4 ; xx [
1 ] = 0.3148932788803888 ; xx [ 2 ] = - 2.384623577931677e-3 ; xx [ 3 ] =
0.9491236119720561 ; xx [ 4 ] = motionData [ 56 ] ; xx [ 5 ] = motionData [
57 ] ; xx [ 6 ] = motionData [ 58 ] ; xx [ 7 ] = motionData [ 59 ] ; xx [ 8 ]
= motionData [ 63 ] ; xx [ 9 ] = motionData [ 64 ] ; xx [ 10 ] = motionData [
65 ] ; xx [ 11 ] = motionData [ 66 ] ; pm_math_Quaternion_inverseCompose_ra (
xx + 4 , xx + 8 , xx + 12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 ,
xx + 12 , xx + 4 ) ; xx [ 0 ] = 1.380846645915312e-7 ; xx [ 1 ] = -
0.9999999999981867 ; xx [ 2 ] = - 1.899414134864941e-6 ; xx [ 8 ] =
motionData [ 244 ] ; xx [ 9 ] = motionData [ 245 ] ; xx [ 10 ] = motionData [
246 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 12 , xx + 8 , xx + 16 ) ;
xx [ 8 ] = motionData [ 250 ] - xx [ 16 ] ; xx [ 9 ] = motionData [ 251 ] -
xx [ 17 ] ; xx [ 10 ] = motionData [ 252 ] - xx [ 18 ] ; state [ 67 ] =
sm_core_canonicalAngle ( 2.0 * atan2 ( sqrt ( xx [ 5 ] * xx [ 5 ] + xx [ 6 ]
* xx [ 6 ] + xx [ 7 ] * xx [ 7 ] ) , fabs ( xx [ 4 ] ) ) * ( (
pm_math_Vector3_dot_ra ( xx + 5 , xx + 0 ) * xx [ 4 ] ) < 0.0 ? - 1.0 : + 1.0
) ) ; state [ 68 ] = pm_math_Vector3_dot_ra ( xx + 8 , xx + 0 ) ; return NULL
; } void Control_Bicopter_ae14a523_1_initializeTrackedAngleState ( const void
* mech , const RuntimeDerivedValuesBundle * rtdv , const int * modeVector ,
const double * motionData , double * state , void * neDiagMgr0 ) {
NeuDiagnosticManager * neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; (
void ) mech ; initializeTrackedAngleState_0 ( rtdv , modeVector , motionData
, state , neDiagMgr ) ; initializeTrackedAngleState_1 ( rtdv , modeVector ,
motionData , state , neDiagMgr ) ; initializeTrackedAngleState_2 ( rtdv ,
modeVector , motionData , state , neDiagMgr ) ; initializeTrackedAngleState_3
( rtdv , modeVector , motionData , state , neDiagMgr ) ;
initializeTrackedAngleState_4 ( rtdv , modeVector , motionData , state ,
neDiagMgr ) ; initializeTrackedAngleState_5 ( rtdv , modeVector , motionData
, state , neDiagMgr ) ; initializeTrackedAngleState_6 ( rtdv , modeVector ,
motionData , state , neDiagMgr ) ; initializeTrackedAngleState_7 ( rtdv ,
modeVector , motionData , state , neDiagMgr ) ; } void
Control_Bicopter_ae14a523_1_computeDiscreteState ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , double * state ) { const double * rtdvd =
rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; (
void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; } void
Control_Bicopter_ae14a523_1_adjustPosition ( const void * mech , const double
* dofDeltas , double * state ) { double xx [ 11 ] ; ( void ) mech ; xx [ 0 ]
= state [ 3 ] ; xx [ 1 ] = state [ 4 ] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] =
state [ 6 ] ; xx [ 4 ] = dofDeltas [ 3 ] ; xx [ 5 ] = dofDeltas [ 4 ] ; xx [
6 ] = dofDeltas [ 5 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 0 , xx + 4 ,
xx + 7 ) ; xx [ 0 ] = state [ 3 ] + xx [ 7 ] ; xx [ 1 ] = state [ 4 ] + xx [
8 ] ; xx [ 2 ] = state [ 5 ] + xx [ 9 ] ; xx [ 3 ] = state [ 6 ] + xx [ 10 ]
; xx [ 4 ] = sqrt ( xx [ 0 ] * xx [ 0 ] + xx [ 1 ] * xx [ 1 ] + xx [ 2 ] * xx
[ 2 ] + xx [ 3 ] * xx [ 3 ] ) ; xx [ 5 ] = 1.0e-64 ; if ( xx [ 5 ] > xx [ 4 ]
) xx [ 4 ] = xx [ 5 ] ; state [ 0 ] = state [ 0 ] + dofDeltas [ 0 ] ; state [
1 ] = state [ 1 ] + dofDeltas [ 1 ] ; state [ 2 ] = state [ 2 ] + dofDeltas [
2 ] ; state [ 3 ] = xx [ 0 ] / xx [ 4 ] ; state [ 4 ] = xx [ 1 ] / xx [ 4 ] ;
state [ 5 ] = xx [ 2 ] / xx [ 4 ] ; state [ 6 ] = xx [ 3 ] / xx [ 4 ] ; state
[ 13 ] = state [ 13 ] + dofDeltas [ 6 ] ; state [ 14 ] = state [ 14 ] +
dofDeltas [ 7 ] ; state [ 17 ] = state [ 17 ] + dofDeltas [ 8 ] ; state [ 19
] = state [ 19 ] + dofDeltas [ 9 ] ; state [ 20 ] = state [ 20 ] + dofDeltas
[ 10 ] ; state [ 23 ] = state [ 23 ] + dofDeltas [ 11 ] ; state [ 25 ] =
state [ 25 ] + dofDeltas [ 12 ] ; state [ 27 ] = state [ 27 ] + dofDeltas [
13 ] ; state [ 28 ] = state [ 28 ] + dofDeltas [ 14 ] ; state [ 31 ] = state
[ 31 ] + dofDeltas [ 15 ] ; state [ 33 ] = state [ 33 ] + dofDeltas [ 16 ] ;
state [ 34 ] = state [ 34 ] + dofDeltas [ 17 ] ; state [ 37 ] = state [ 37 ]
+ dofDeltas [ 18 ] ; state [ 39 ] = state [ 39 ] + dofDeltas [ 19 ] ; state [
41 ] = state [ 41 ] + dofDeltas [ 20 ] ; state [ 43 ] = state [ 43 ] +
dofDeltas [ 21 ] ; state [ 45 ] = state [ 45 ] + dofDeltas [ 22 ] ; state [
47 ] = state [ 47 ] + dofDeltas [ 23 ] ; state [ 49 ] = state [ 49 ] +
dofDeltas [ 24 ] ; state [ 51 ] = state [ 51 ] + dofDeltas [ 25 ] ; } static
void perturbJointPrimitiveState_0_0 ( double mag , double * state ) { state [
0 ] = state [ 0 ] + mag ; } static void perturbJointPrimitiveState_0_0v (
double mag , double * state ) { state [ 0 ] = state [ 0 ] + mag ; state [ 7 ]
= state [ 7 ] - 0.875 * mag ; } static void perturbJointPrimitiveState_0_1 (
double mag , double * state ) { state [ 1 ] = state [ 1 ] + mag ; } static
void perturbJointPrimitiveState_0_1v ( double mag , double * state ) { state
[ 1 ] = state [ 1 ] + mag ; state [ 8 ] = state [ 8 ] - 0.875 * mag ; }
static void perturbJointPrimitiveState_0_2 ( double mag , double * state ) {
state [ 2 ] = state [ 2 ] + mag ; } static void
perturbJointPrimitiveState_0_2v ( double mag , double * state ) { state [ 2 ]
= state [ 2 ] + mag ; state [ 9 ] = state [ 9 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_0_3 ( double mag , double * state ) { double xx [
15 ] ; xx [ 0 ] = 1.0 ; xx [ 1 ] = fabs ( mag ) ; xx [ 2 ] = xx [ 0 ] / ( xx
[ 1 ] - floor ( xx [ 1 ] ) + 1.0e-9 ) ; xx [ 1 ] = sin ( xx [ 2 ] ) ; xx [ 3
] = 0.0 ; xx [ 4 ] = cos ( xx [ 2 ] ) ; xx [ 5 ] = sin ( 2.0 * xx [ 2 ] ) ;
xx [ 2 ] = 0.5 * mag ; xx [ 6 ] = sqrt ( xx [ 1 ] * xx [ 1 ] + xx [ 4 ] * xx
[ 4 ] + xx [ 5 ] * xx [ 5 ] ) ; xx [ 7 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 1 ] /
xx [ 6 ] ; xx [ 8 ] = sin ( xx [ 2 ] ) ; xx [ 9 ] = xx [ 6 ] == 0.0 ? 0.0 :
xx [ 4 ] / xx [ 6 ] ; xx [ 10 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 5 ] / xx [ 6 ]
; xx [ 11 ] = xx [ 1 ] == xx [ 3 ] && xx [ 4 ] == xx [ 3 ] && xx [ 5 ] == xx
[ 3 ] ? xx [ 0 ] : cos ( xx [ 2 ] ) ; xx [ 12 ] = xx [ 7 ] * xx [ 8 ] ; xx [
13 ] = xx [ 9 ] * xx [ 8 ] ; xx [ 14 ] = xx [ 10 ] * xx [ 8 ] ; xx [ 0 ] =
state [ 3 ] ; xx [ 1 ] = state [ 4 ] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] =
state [ 6 ] ; pm_math_Quaternion_compose_ra ( xx + 11 , xx + 0 , xx + 4 ) ;
state [ 3 ] = xx [ 4 ] ; state [ 4 ] = xx [ 5 ] ; state [ 5 ] = xx [ 6 ] ;
state [ 6 ] = xx [ 7 ] ; } static void perturbJointPrimitiveState_0_3v (
double mag , double * state ) { double xx [ 15 ] ; xx [ 0 ] = 1.0 ; xx [ 1 ]
= fabs ( mag ) ; xx [ 2 ] = xx [ 0 ] / ( xx [ 1 ] - floor ( xx [ 1 ] ) +
1.0e-9 ) ; xx [ 1 ] = sin ( xx [ 2 ] ) ; xx [ 3 ] = 0.0 ; xx [ 4 ] = cos ( xx
[ 2 ] ) ; xx [ 5 ] = sin ( 2.0 * xx [ 2 ] ) ; xx [ 2 ] = 0.5 * mag ; xx [ 6 ]
= sqrt ( xx [ 1 ] * xx [ 1 ] + xx [ 4 ] * xx [ 4 ] + xx [ 5 ] * xx [ 5 ] ) ;
xx [ 7 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 1 ] / xx [ 6 ] ; xx [ 8 ] = sin ( xx
[ 2 ] ) ; xx [ 9 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 4 ] / xx [ 6 ] ; xx [ 10 ]
= xx [ 6 ] == 0.0 ? 0.0 : xx [ 5 ] / xx [ 6 ] ; xx [ 11 ] = xx [ 1 ] == xx [
3 ] && xx [ 4 ] == xx [ 3 ] && xx [ 5 ] == xx [ 3 ] ? xx [ 0 ] : cos ( xx [ 2
] ) ; xx [ 12 ] = xx [ 7 ] * xx [ 8 ] ; xx [ 13 ] = xx [ 9 ] * xx [ 8 ] ; xx
[ 14 ] = xx [ 10 ] * xx [ 8 ] ; xx [ 3 ] = state [ 3 ] ; xx [ 4 ] = state [ 4
] ; xx [ 5 ] = state [ 5 ] ; xx [ 6 ] = state [ 6 ] ;
pm_math_Quaternion_compose_ra ( xx + 11 , xx + 3 , xx + 7 ) ; state [ 3 ] =
xx [ 7 ] ; state [ 4 ] = xx [ 8 ] ; state [ 5 ] = xx [ 9 ] ; state [ 6 ] = xx
[ 10 ] ; state [ 10 ] = state [ 10 ] + 1.2 * mag ; state [ 11 ] = state [ 11
] - xx [ 2 ] ; state [ 12 ] = state [ 12 ] + 0.9 * mag ; } static void
perturbJointPrimitiveState_1_0 ( double mag , double * state ) { state [ 13 ]
= state [ 13 ] + mag ; } static void perturbJointPrimitiveState_1_0v ( double
mag , double * state ) { state [ 13 ] = state [ 13 ] + mag ; state [ 15 ] =
state [ 15 ] - 0.875 * mag ; } static void perturbJointPrimitiveState_1_1 (
double mag , double * state ) { state [ 14 ] = state [ 14 ] + mag ; } static
void perturbJointPrimitiveState_1_1v ( double mag , double * state ) { state
[ 14 ] = state [ 14 ] + mag ; state [ 16 ] = state [ 16 ] - 0.875 * mag ; }
static void perturbJointPrimitiveState_2_0 ( double mag , double * state ) {
state [ 17 ] = state [ 17 ] + mag ; } static void
perturbJointPrimitiveState_2_0v ( double mag , double * state ) { state [ 17
] = state [ 17 ] + mag ; state [ 18 ] = state [ 18 ] - 0.875 * mag ; } static
void perturbJointPrimitiveState_3_0 ( double mag , double * state ) { state [
19 ] = state [ 19 ] + mag ; } static void perturbJointPrimitiveState_3_0v (
double mag , double * state ) { state [ 19 ] = state [ 19 ] + mag ; state [
21 ] = state [ 21 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_3_1 ( double mag , double * state ) { state [ 20 ]
= state [ 20 ] + mag ; } static void perturbJointPrimitiveState_3_1v ( double
mag , double * state ) { state [ 20 ] = state [ 20 ] + mag ; state [ 22 ] =
state [ 22 ] - 0.875 * mag ; } static void perturbJointPrimitiveState_4_0 (
double mag , double * state ) { state [ 23 ] = state [ 23 ] + mag ; } static
void perturbJointPrimitiveState_4_0v ( double mag , double * state ) { state
[ 23 ] = state [ 23 ] + mag ; state [ 24 ] = state [ 24 ] - 0.875 * mag ; }
static void perturbJointPrimitiveState_5_0 ( double mag , double * state ) {
state [ 25 ] = state [ 25 ] + mag ; } static void
perturbJointPrimitiveState_5_0v ( double mag , double * state ) { state [ 25
] = state [ 25 ] + mag ; state [ 26 ] = state [ 26 ] - 0.875 * mag ; } static
void perturbJointPrimitiveState_6_0 ( double mag , double * state ) { state [
27 ] = state [ 27 ] + mag ; } static void perturbJointPrimitiveState_6_0v (
double mag , double * state ) { state [ 27 ] = state [ 27 ] + mag ; state [
29 ] = state [ 29 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_6_1 ( double mag , double * state ) { state [ 28 ]
= state [ 28 ] + mag ; } static void perturbJointPrimitiveState_6_1v ( double
mag , double * state ) { state [ 28 ] = state [ 28 ] + mag ; state [ 30 ] =
state [ 30 ] - 0.875 * mag ; } static void perturbJointPrimitiveState_7_0 (
double mag , double * state ) { state [ 31 ] = state [ 31 ] + mag ; } static
void perturbJointPrimitiveState_7_0v ( double mag , double * state ) { state
[ 31 ] = state [ 31 ] + mag ; state [ 32 ] = state [ 32 ] - 0.875 * mag ; }
static void perturbJointPrimitiveState_8_0 ( double mag , double * state ) {
state [ 33 ] = state [ 33 ] + mag ; } static void
perturbJointPrimitiveState_8_0v ( double mag , double * state ) { state [ 33
] = state [ 33 ] + mag ; state [ 35 ] = state [ 35 ] - 0.875 * mag ; } static
void perturbJointPrimitiveState_8_1 ( double mag , double * state ) { state [
34 ] = state [ 34 ] + mag ; } static void perturbJointPrimitiveState_8_1v (
double mag , double * state ) { state [ 34 ] = state [ 34 ] + mag ; state [
36 ] = state [ 36 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_9_0 ( double mag , double * state ) { state [ 37 ]
= state [ 37 ] + mag ; } static void perturbJointPrimitiveState_9_0v ( double
mag , double * state ) { state [ 37 ] = state [ 37 ] + mag ; state [ 38 ] =
state [ 38 ] - 0.875 * mag ; } static void perturbJointPrimitiveState_10_0 (
double mag , double * state ) { state [ 39 ] = state [ 39 ] + mag ; } static
void perturbJointPrimitiveState_10_0v ( double mag , double * state ) { state
[ 39 ] = state [ 39 ] + mag ; state [ 40 ] = state [ 40 ] - 0.875 * mag ; }
static void perturbJointPrimitiveState_11_0 ( double mag , double * state ) {
state [ 41 ] = state [ 41 ] + mag ; } static void
perturbJointPrimitiveState_11_0v ( double mag , double * state ) { state [ 41
] = state [ 41 ] + mag ; state [ 42 ] = state [ 42 ] - 0.875 * mag ; } static
void perturbJointPrimitiveState_12_0 ( double mag , double * state ) { state
[ 43 ] = state [ 43 ] + mag ; } static void perturbJointPrimitiveState_12_0v
( double mag , double * state ) { state [ 43 ] = state [ 43 ] + mag ; state [
44 ] = state [ 44 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_13_0 ( double mag , double * state ) { state [ 45
] = state [ 45 ] + mag ; } static void perturbJointPrimitiveState_13_0v (
double mag , double * state ) { state [ 45 ] = state [ 45 ] + mag ; state [
46 ] = state [ 46 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_14_0 ( double mag , double * state ) { state [ 47
] = state [ 47 ] + mag ; } static void perturbJointPrimitiveState_14_0v (
double mag , double * state ) { state [ 47 ] = state [ 47 ] + mag ; state [
48 ] = state [ 48 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_15_0 ( double mag , double * state ) { state [ 49
] = state [ 49 ] + mag ; } static void perturbJointPrimitiveState_15_0v (
double mag , double * state ) { state [ 49 ] = state [ 49 ] + mag ; state [
50 ] = state [ 50 ] - 0.875 * mag ; } static void
perturbJointPrimitiveState_16_0 ( double mag , double * state ) { state [ 51
] = state [ 51 ] + mag ; } static void perturbJointPrimitiveState_16_0v (
double mag , double * state ) { state [ 51 ] = state [ 51 ] + mag ; state [
52 ] = state [ 52 ] - 0.875 * mag ; } void
Control_Bicopter_ae14a523_1_perturbJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primIdx , double mag , boolean_T doPerturbVelocity ,
double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) mag ; ( void ) doPerturbVelocity ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) * 2 + ( doPerturbVelocity ? 1 : 0 ) ) { case 0 :
perturbJointPrimitiveState_0_0 ( mag , state ) ; break ; case 1 :
perturbJointPrimitiveState_0_0v ( mag , state ) ; break ; case 2 :
perturbJointPrimitiveState_0_1 ( mag , state ) ; break ; case 3 :
perturbJointPrimitiveState_0_1v ( mag , state ) ; break ; case 4 :
perturbJointPrimitiveState_0_2 ( mag , state ) ; break ; case 5 :
perturbJointPrimitiveState_0_2v ( mag , state ) ; break ; case 6 :
perturbJointPrimitiveState_0_3 ( mag , state ) ; break ; case 7 :
perturbJointPrimitiveState_0_3v ( mag , state ) ; break ; case 12 :
perturbJointPrimitiveState_1_0 ( mag , state ) ; break ; case 13 :
perturbJointPrimitiveState_1_0v ( mag , state ) ; break ; case 14 :
perturbJointPrimitiveState_1_1 ( mag , state ) ; break ; case 15 :
perturbJointPrimitiveState_1_1v ( mag , state ) ; break ; case 24 :
perturbJointPrimitiveState_2_0 ( mag , state ) ; break ; case 25 :
perturbJointPrimitiveState_2_0v ( mag , state ) ; break ; case 36 :
perturbJointPrimitiveState_3_0 ( mag , state ) ; break ; case 37 :
perturbJointPrimitiveState_3_0v ( mag , state ) ; break ; case 38 :
perturbJointPrimitiveState_3_1 ( mag , state ) ; break ; case 39 :
perturbJointPrimitiveState_3_1v ( mag , state ) ; break ; case 48 :
perturbJointPrimitiveState_4_0 ( mag , state ) ; break ; case 49 :
perturbJointPrimitiveState_4_0v ( mag , state ) ; break ; case 60 :
perturbJointPrimitiveState_5_0 ( mag , state ) ; break ; case 61 :
perturbJointPrimitiveState_5_0v ( mag , state ) ; break ; case 72 :
perturbJointPrimitiveState_6_0 ( mag , state ) ; break ; case 73 :
perturbJointPrimitiveState_6_0v ( mag , state ) ; break ; case 74 :
perturbJointPrimitiveState_6_1 ( mag , state ) ; break ; case 75 :
perturbJointPrimitiveState_6_1v ( mag , state ) ; break ; case 84 :
perturbJointPrimitiveState_7_0 ( mag , state ) ; break ; case 85 :
perturbJointPrimitiveState_7_0v ( mag , state ) ; break ; case 96 :
perturbJointPrimitiveState_8_0 ( mag , state ) ; break ; case 97 :
perturbJointPrimitiveState_8_0v ( mag , state ) ; break ; case 98 :
perturbJointPrimitiveState_8_1 ( mag , state ) ; break ; case 99 :
perturbJointPrimitiveState_8_1v ( mag , state ) ; break ; case 108 :
perturbJointPrimitiveState_9_0 ( mag , state ) ; break ; case 109 :
perturbJointPrimitiveState_9_0v ( mag , state ) ; break ; case 120 :
perturbJointPrimitiveState_10_0 ( mag , state ) ; break ; case 121 :
perturbJointPrimitiveState_10_0v ( mag , state ) ; break ; case 132 :
perturbJointPrimitiveState_11_0 ( mag , state ) ; break ; case 133 :
perturbJointPrimitiveState_11_0v ( mag , state ) ; break ; case 144 :
perturbJointPrimitiveState_12_0 ( mag , state ) ; break ; case 145 :
perturbJointPrimitiveState_12_0v ( mag , state ) ; break ; case 156 :
perturbJointPrimitiveState_13_0 ( mag , state ) ; break ; case 157 :
perturbJointPrimitiveState_13_0v ( mag , state ) ; break ; case 168 :
perturbJointPrimitiveState_14_0 ( mag , state ) ; break ; case 169 :
perturbJointPrimitiveState_14_0v ( mag , state ) ; break ; case 180 :
perturbJointPrimitiveState_15_0 ( mag , state ) ; break ; case 181 :
perturbJointPrimitiveState_15_0v ( mag , state ) ; break ; case 192 :
perturbJointPrimitiveState_16_0 ( mag , state ) ; break ; case 193 :
perturbJointPrimitiveState_16_0v ( mag , state ) ; break ; } } void
Control_Bicopter_ae14a523_1_perturbFlexibleBodyState ( const void * mech ,
size_t stageIdx , double mag , boolean_T doPerturbVelocity , double * state )
{ ( void ) mech ; ( void ) stageIdx ; ( void ) mag ; ( void )
doPerturbVelocity ; ( void ) state ; switch ( stageIdx * 2 + (
doPerturbVelocity ? 1 : 0 ) ) { } } void
Control_Bicopter_ae14a523_1_computeDofBlendMatrix ( const void * mech ,
size_t stageIdx , size_t primIdx , const double * state , int partialType ,
double * matrix ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) state ; ( void ) partialType ; ( void ) matrix ; switch ( ( stageIdx *
6 + primIdx ) ) { } } void
Control_Bicopter_ae14a523_1_projectPartiallyTargetedPos ( const void * mech ,
size_t stageIdx , size_t primIdx , const double * origState , int partialType
, double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) origState ; ( void ) partialType ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) ) { } } void
Control_Bicopter_ae14a523_1_propagateMotion ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const double * state , double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; double xx [ 345 ] ; ( void ) mech ; (
void ) rtdvd ; ( void ) rtdvi ; xx [ 0 ] = state [ 3 ] ; xx [ 1 ] = state [ 4
] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] = state [ 6 ] ; xx [ 4 ] = -
0.2974211203852171 ; xx [ 5 ] = 0.24358232233362 ; xx [ 6 ] = -
0.637892237857866 ; xx [ 7 ] = - 0.6673093902201936 ;
pm_math_Quaternion_composeInverse_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [ 0 ]
= 1.533017838243546e-3 ; xx [ 1 ] = 1.220598588933962e-3 ; xx [ 2 ] =
8.957592155301676e-3 ; xx [ 12 ] = - xx [ 0 ] ; xx [ 13 ] = - xx [ 1 ] ; xx [
14 ] = - xx [ 2 ] ; pm_math_Quaternion_xform_ra ( xx + 8 , xx + 12 , xx + 15
) ; xx [ 3 ] = state [ 0 ] - xx [ 15 ] ; xx [ 12 ] = state [ 1 ] - xx [ 16 ]
; xx [ 13 ] = state [ 2 ] - xx [ 17 ] ; xx [ 14 ] = - 0.9120630938981265 ; xx
[ 15 ] = - 0.1421693578567881 ; xx [ 16 ] = 0.3844095801838581 ; xx [ 17 ] =
- 0.01257223522036471 ; xx [ 18 ] = 0.5 ; xx [ 19 ] = xx [ 18 ] * state [ 13
] ; xx [ 20 ] = 3.734971681880274e-4 ; xx [ 21 ] = sin ( xx [ 19 ] ) ; xx [
22 ] = 0.2610715860287157 ; xx [ 23 ] = 0.9653193707101895 ; xx [ 24 ] = cos
( xx [ 19 ] ) ; xx [ 25 ] = xx [ 20 ] * xx [ 21 ] ; xx [ 26 ] = xx [ 22 ] *
xx [ 21 ] ; xx [ 27 ] = xx [ 23 ] * xx [ 21 ] ; pm_math_Quaternion_compose_ra
( xx + 14 , xx + 24 , xx + 28 ) ; xx [ 14 ] = - 8.970481754443746e-5 ; xx [
15 ] = - 2.490130590085562e-3 ; xx [ 16 ] = 0.01561535429133853 ;
pm_math_Quaternion_xform_ra ( xx + 28 , xx + 14 , xx + 24 ) ; xx [ 14 ] =
0.07007512777103127 + 0.7077023583577552 * state [ 14 ] + xx [ 24 ] ; xx [ 15
] = - xx [ 14 ] ; xx [ 16 ] = 6.588221211357465e-3 - 9.268340058968039e-3 *
state [ 14 ] - xx [ 25 ] ; xx [ 17 ] = 0.05970036331915229 +
0.7064499061132523 * state [ 14 ] - xx [ 26 ] ; xx [ 24 ] =
0.4090279037421741 ; xx [ 25 ] = - 0.57698859548371 ; xx [ 26 ] = -
0.5768688874880377 ; xx [ 27 ] = - 0.4086595420277526 ; xx [ 19 ] = xx [ 18 ]
* state [ 17 ] ; xx [ 21 ] = 0.9972896958011036 ; xx [ 32 ] = sin ( xx [ 19 ]
) ; xx [ 33 ] = 8.852854494773021e-5 ; xx [ 34 ] = 0.07357482457770348 ; xx [
35 ] = cos ( xx [ 19 ] ) ; xx [ 36 ] = - ( xx [ 21 ] * xx [ 32 ] ) ; xx [ 37
] = xx [ 33 ] * xx [ 32 ] ; xx [ 38 ] = xx [ 34 ] * xx [ 32 ] ;
pm_math_Quaternion_compose_ra ( xx + 24 , xx + 35 , xx + 39 ) ; xx [ 19 ] =
8.39853113781329e-5 ; xx [ 24 ] = 9.122274404587457e-4 ; xx [ 25 ] =
1.459342321928613e-6 ; xx [ 26 ] = - 0.01504973426522732 ;
pm_math_Quaternion_xform_ra ( xx + 39 , xx + 24 , xx + 35 ) ; xx [ 24 ] = - (
xx [ 19 ] + xx [ 35 ] ) ; xx [ 25 ] = 1.467714669213244e-3 ; xx [ 26 ] = xx [
25 ] - xx [ 36 ] ; xx [ 27 ] = 0.03024959596740159 ; xx [ 32 ] = xx [ 27 ] -
xx [ 37 ] ; xx [ 43 ] = 0.4716196941540828 ; xx [ 44 ] = - 0.5117381057633613
; xx [ 45 ] = 0.5267317275910312 ; xx [ 46 ] = - 0.4881113216723255 ; xx [ 38
] = xx [ 18 ] * state [ 19 ] ; xx [ 47 ] = 3.05954793225871e-4 ; xx [ 48 ] =
sin ( xx [ 38 ] ) ; xx [ 49 ] = 0.9999868227746886 ; xx [ 50 ] =
5.124516430675685e-3 ; xx [ 51 ] = cos ( xx [ 38 ] ) ; xx [ 52 ] = xx [ 47 ]
* xx [ 48 ] ; xx [ 53 ] = - ( xx [ 49 ] * xx [ 48 ] ) ; xx [ 54 ] = - ( xx [
50 ] * xx [ 48 ] ) ; pm_math_Quaternion_compose_ra ( xx + 43 , xx + 51 , xx +
55 ) ; xx [ 43 ] = 8.625263905800012e-8 ; xx [ 44 ] = 4.387911391441809e-3 ;
xx [ 45 ] = 2.281586598679112e-5 ; pm_math_Quaternion_xform_ra ( xx + 55 , xx
+ 43 , xx + 51 ) ; xx [ 38 ] = 1.451813746698375e-3 + 0.07357483435689677 *
state [ 20 ] - xx [ 51 ] ; xx [ 43 ] = 1.995808362665768e-6 +
1.128000373271654e-4 * state [ 20 ] - xx [ 52 ] ; xx [ 44 ] =
0.02110873061376549 + 0.9972896926297332 * state [ 20 ] - xx [ 53 ] ; xx [ 51
] = 0.6254151447093548 ; xx [ 52 ] = - 0.6480628292580688 ; xx [ 53 ] =
0.3298467071707108 ; xx [ 54 ] = - 0.2829692843240122 ; xx [ 45 ] = xx [ 18 ]
* state [ 23 ] ; xx [ 46 ] = 1.380846645360201e-7 ; xx [ 48 ] = sin ( xx [ 45
] ) ; xx [ 59 ] = 0.9999999999981867 ; xx [ 60 ] = 1.899414133310628e-6 ; xx
[ 61 ] = cos ( xx [ 45 ] ) ; xx [ 62 ] = - ( xx [ 46 ] * xx [ 48 ] ) ; xx [
63 ] = xx [ 59 ] * xx [ 48 ] ; xx [ 64 ] = xx [ 60 ] * xx [ 48 ] ;
pm_math_Quaternion_compose_ra ( xx + 51 , xx + 61 , xx + 65 ) ; xx [ 51 ] = -
3.975614490536003e-8 ; xx [ 52 ] = 2.710692114195099e-3 ; xx [ 53 ] = -
2.919897873389293e-7 ; pm_math_Quaternion_xform_ra ( xx + 65 , xx + 51 , xx +
61 ) ; xx [ 45 ] = 9.000024890213373e-4 - xx [ 61 ] ; xx [ 48 ] =
1.149808082944376e-6 - xx [ 62 ] ; xx [ 51 ] = 0.01362905791904227 - xx [ 63
] ; xx [ 52 ] = - 0.9229115780805051 ; xx [ 53 ] = - 0.02085503273058337 ; xx
[ 54 ] = 0.3825441798635174 ; xx [ 61 ] = 0.03819996213750987 ; xx [ 62 ] =
0.08324084851537972 ; xx [ 63 ] = 0.7044167893867059 * state [ 25 ] ; xx [ 64
] = xx [ 62 ] + xx [ 63 ] ; xx [ 69 ] = 4.5034434673372e-3 ; xx [ 70 ] =
0.08618526586337766 * state [ 25 ] ; xx [ 71 ] = xx [ 69 ] - xx [ 70 ] ; xx [
72 ] = 0.7045346597422903 * state [ 25 ] ; xx [ 73 ] = 0.09384704203289078 ;
xx [ 74 ] = xx [ 72 ] - xx [ 73 ] ; xx [ 75 ] = - 0.1316824519486606 ; xx [
76 ] = 0.9912919062119712 ; xx [ 77 ] = 2.050212948449675e-4 ; xx [ 78 ] = -
2.156239646078782e-4 ; xx [ 79 ] = xx [ 18 ] * state [ 27 ] ; xx [ 80 ] =
3.734971681885031e-4 ; xx [ 81 ] = sin ( xx [ 79 ] ) ; xx [ 82 ] =
0.261071586028717 ; xx [ 83 ] = 0.965319370710189 ; xx [ 84 ] = cos ( xx [ 79
] ) ; xx [ 85 ] = xx [ 80 ] * xx [ 81 ] ; xx [ 86 ] = xx [ 82 ] * xx [ 81 ] ;
xx [ 87 ] = xx [ 83 ] * xx [ 81 ] ; pm_math_Quaternion_compose_ra ( xx + 75 ,
xx + 84 , xx + 88 ) ; xx [ 75 ] = - 8.9928915845163e-5 ; xx [ 76 ] = -
2.64677354170138e-3 ; xx [ 77 ] = 0.01503616266891429 ;
pm_math_Quaternion_xform_ra ( xx + 88 , xx + 75 , xx + 84 ) ; xx [ 75 ] =
3.383773514455306e-7 - xx [ 84 ] ; xx [ 76 ] = 4.111205245035695e-3 - xx [ 85
] ; xx [ 77 ] = 0.02364999999999497 - state [ 28 ] - xx [ 86 ] ; xx [ 84 ] =
- 0.5181324366224915 ; xx [ 85 ] = - 0.4812371049201607 ; xx [ 86 ] = -
0.4812727257575918 ; xx [ 87 ] = 0.5180021142906276 ; xx [ 78 ] = xx [ 18 ] *
state [ 31 ] ; xx [ 79 ] = 0.9972896958009458 ; xx [ 81 ] = sin ( xx [ 78 ] )
; xx [ 92 ] = 8.852854518109909e-5 ; xx [ 93 ] = 0.0735748245798411 ; xx [ 94
] = cos ( xx [ 78 ] ) ; xx [ 95 ] = - ( xx [ 79 ] * xx [ 81 ] ) ; xx [ 96 ] =
xx [ 92 ] * xx [ 81 ] ; xx [ 97 ] = xx [ 93 ] * xx [ 81 ] ;
pm_math_Quaternion_compose_ra ( xx + 84 , xx + 94 , xx + 98 ) ; xx [ 78 ] =
3.383773468563022e-7 ; xx [ 84 ] = 9.122274404458714e-4 ; xx [ 85 ] =
1.459342330053661e-6 ; xx [ 86 ] = - 0.0150497342652287 ;
pm_math_Quaternion_xform_ra ( xx + 98 , xx + 84 , xx + 94 ) ; xx [ 81 ] = xx
[ 78 ] - xx [ 94 ] ; xx [ 84 ] = 4.111205245038918e-3 ; xx [ 85 ] = xx [ 84 ]
- xx [ 95 ] ; xx [ 86 ] = 7.889999999975156e-3 ; xx [ 87 ] = xx [ 86 ] - xx [
96 ] ; xx [ 102 ] = 0.4716196941536389 ; xx [ 103 ] = - 0.5117381057636369 ;
xx [ 104 ] = 0.5267317275917935 ; xx [ 105 ] = - 0.4881113216716428 ; xx [ 97
] = xx [ 18 ] * state [ 33 ] ; xx [ 106 ] = 3.059547932236506e-4 ; xx [ 107 ]
= sin ( xx [ 97 ] ) ; xx [ 108 ] = 0.9999868227746889 ; xx [ 109 ] =
5.124516430674353e-3 ; xx [ 110 ] = cos ( xx [ 97 ] ) ; xx [ 111 ] = xx [ 106
] * xx [ 107 ] ; xx [ 112 ] = - ( xx [ 108 ] * xx [ 107 ] ) ; xx [ 113 ] = -
( xx [ 109 ] * xx [ 107 ] ) ; pm_math_Quaternion_compose_ra ( xx + 102 , xx +
110 , xx + 114 ) ; xx [ 97 ] = 1.451813746738863e-3 ; xx [ 102 ] =
8.62526392261675e-8 ; xx [ 103 ] = 4.387911391441729e-3 ; xx [ 104 ] =
2.28158659870934e-5 ; pm_math_Quaternion_xform_ra ( xx + 114 , xx + 102 , xx
+ 110 ) ; xx [ 102 ] = xx [ 97 ] + 0.07357483435904683 * state [ 34 ] - xx [
110 ] ; xx [ 103 ] = 1.995808348478373e-6 ; xx [ 104 ] = xx [ 103 ] +
1.128000365619997e-4 * state [ 34 ] - xx [ 111 ] ; xx [ 105 ] =
0.02110873061376147 ; xx [ 107 ] = xx [ 105 ] + 0.9972896926295751 * state [
34 ] - xx [ 112 ] ; xx [ 110 ] = 0.6252280994013473 ; xx [ 111 ] =
0.6477664378100223 ; xx [ 112 ] = 0.330373832435706 ; xx [ 113 ] =
0.2834459324238707 ; xx [ 118 ] = xx [ 18 ] * state [ 37 ] ; xx [ 119 ] = sin
( xx [ 118 ] ) ; xx [ 120 ] = 1.899414131645294e-6 ; xx [ 121 ] = cos ( xx [
118 ] ) ; xx [ 122 ] = - ( xx [ 46 ] * xx [ 119 ] ) ; xx [ 123 ] = xx [ 59 ]
* xx [ 119 ] ; xx [ 124 ] = xx [ 120 ] * xx [ 119 ] ;
pm_math_Quaternion_compose_ra ( xx + 110 , xx + 121 , xx + 125 ) ; xx [ 110 ]
= - 3.968710232179073e-8 ; xx [ 111 ] = 2.210692114196348e-3 ; xx [ 112 ] = -
2.929394946184722e-7 ; pm_math_Quaternion_xform_ra ( xx + 125 , xx + 110 , xx
+ 121 ) ; xx [ 110 ] = xx [ 97 ] - xx [ 121 ] ; xx [ 97 ] = xx [ 103 ] - xx [
122 ] ; xx [ 103 ] = xx [ 105 ] - xx [ 123 ] ; xx [ 105 ] = -
0.9228737611163665 ; xx [ 111 ] = - 0.02263515933877589 ; xx [ 112 ] =
0.3826221044509303 ; xx [ 113 ] = 0.03730945979482574 ; xx [ 118 ] = - (
0.05804874412964624 + 0.7077023583577557 * state [ 39 ] ) ; xx [ 119 ] = - (
6.831424848351424e-4 + 9.268340058967206e-3 * state [ 39 ] ) ; xx [ 121 ] =
0.04678282656864583 + 0.7064499061132518 * state [ 39 ] ; xx [ 122 ] = -
0.3824707906815349 ; xx [ 123 ] = - 0.03882994823691858 ; xx [ 124 ] = -
0.9229564379288121 ; xx [ 129 ] = - 0.01896689434564094 ; xx [ 130 ] =
0.05431153664435735 - 0.7077023583577554 * state [ 41 ] ; xx [ 131 ] =
7.88370615320499e-4 - 9.268340058966987e-3 * state [ 41 ] ; xx [ 132 ] =
0.7064499061132521 * state [ 41 ] - 0.06537860523419856 ; xx [ 133 ] = -
0.2934341153784394 ; xx [ 134 ] = 0.2476921755716519 ; xx [ 135 ] = -
0.6382158331684398 ; xx [ 136 ] = - 0.6672522434475355 ; xx [ 137 ] = - (
0.01025589154524426 + 0.7077023583577412 * state [ 43 ] ) ; xx [ 138 ] = - (
0.02452133274909705 + 9.268340058974089e-3 * state [ 43 ] ) ; xx [ 139 ] =
0.7064499061132667 * state [ 43 ] - 0.01799698139187411 ; xx [ 140 ] =
0.01221082284463421 ; xx [ 141 ] = xx [ 18 ] * state [ 45 ] ; xx [ 142 ] =
sin ( xx [ 141 ] ) ; xx [ 143 ] = 0.3820990926796067 ; xx [ 144 ] = cos ( xx
[ 141 ] ) ; xx [ 141 ] = xx [ 140 ] * xx [ 142 ] - xx [ 143 ] * xx [ 144 ] ;
xx [ 145 ] = xx [ 140 ] * xx [ 144 ] + xx [ 143 ] * xx [ 142 ] ; xx [ 140 ] =
0.9231025353984502 ; xx [ 143 ] = 0.04162797520696043 ; xx [ 146 ] = xx [ 140
] * xx [ 144 ] - xx [ 143 ] * xx [ 142 ] ; xx [ 147 ] = xx [ 143 ] * xx [ 144
] + xx [ 140 ] * xx [ 142 ] ; xx [ 140 ] = 2.0 ; xx [ 142 ] =
1.56851889494571e-13 ; xx [ 143 ] = xx [ 147 ] * xx [ 142 ] ; xx [ 144 ] =
0.02149999999999999 ; xx [ 148 ] = xx [ 147 ] * xx [ 144 ] ; xx [ 149 ] = xx
[ 142 ] * xx [ 145 ] + xx [ 144 ] * xx [ 146 ] ; xx [ 150 ] = - xx [ 143 ] ;
xx [ 151 ] = - xx [ 148 ] ; xx [ 152 ] = xx [ 149 ] ;
pm_math_Vector3_cross_ra ( xx + 145 , xx + 150 , xx + 153 ) ; xx [ 150 ] = -
( 0.04034737334080394 + xx [ 140 ] * ( xx [ 153 ] - xx [ 141 ] * xx [ 143 ] )
- xx [ 144 ] ) ; xx [ 143 ] = - ( 0.01130093100527478 + xx [ 142 ] + ( xx [
154 ] - xx [ 141 ] * xx [ 148 ] ) * xx [ 140 ] ) ; xx [ 144 ] = - (
0.07837290165097587 + ( xx [ 141 ] * xx [ 149 ] + xx [ 155 ] ) * xx [ 140 ] )
; xx [ 148 ] = 0.01221082284463426 ; xx [ 149 ] = xx [ 18 ] * state [ 47 ] ;
xx [ 151 ] = sin ( xx [ 149 ] ) ; xx [ 152 ] = 0.3820990926796041 ; xx [ 153
] = cos ( xx [ 149 ] ) ; xx [ 149 ] = xx [ 148 ] * xx [ 151 ] - xx [ 152 ] *
xx [ 153 ] ; xx [ 154 ] = xx [ 148 ] * xx [ 153 ] + xx [ 152 ] * xx [ 151 ] ;
xx [ 148 ] = 0.9231025353984514 ; xx [ 152 ] = 0.04162797520696046 ; xx [ 155
] = xx [ 148 ] * xx [ 153 ] - xx [ 152 ] * xx [ 151 ] ; xx [ 156 ] = xx [ 152
] * xx [ 153 ] + xx [ 148 ] * xx [ 151 ] ; xx [ 148 ] = xx [ 156 ] * xx [ 142
] ; xx [ 151 ] = 0.02149999999999999 ; xx [ 152 ] = xx [ 156 ] * xx [ 151 ] ;
xx [ 153 ] = xx [ 142 ] * xx [ 154 ] + xx [ 151 ] * xx [ 155 ] ; xx [ 157 ] =
- xx [ 148 ] ; xx [ 158 ] = - xx [ 152 ] ; xx [ 159 ] = xx [ 153 ] ;
pm_math_Vector3_cross_ra ( xx + 154 , xx + 157 , xx + 160 ) ; xx [ 157 ] = -
( 8.919807266753083e-3 + xx [ 140 ] * ( xx [ 160 ] - xx [ 149 ] * xx [ 148 ]
) - xx [ 151 ] ) ; xx [ 148 ] = 9.941671475675939e-3 - ( xx [ 142 ] + ( xx [
161 ] - xx [ 149 ] * xx [ 152 ] ) * xx [ 140 ] ) ; xx [ 152 ] = - (
0.04661092377057491 + ( xx [ 149 ] * xx [ 153 ] + xx [ 162 ] ) * xx [ 140 ] )
; xx [ 153 ] = 0.9231031215172467 ; xx [ 158 ] = xx [ 18 ] * state [ 49 ] ;
xx [ 159 ] = cos ( xx [ 158 ] ) ; xx [ 160 ] = 0.04161497595801161 ; xx [ 161
] = sin ( xx [ 158 ] ) ; xx [ 158 ] = xx [ 153 ] * xx [ 159 ] + xx [ 160 ] *
xx [ 161 ] ; xx [ 162 ] = xx [ 160 ] * xx [ 159 ] - xx [ 153 ] * xx [ 161 ] ;
xx [ 160 ] = 0.3820992645960588 ; xx [ 163 ] = 0.01220544207643509 ; xx [ 164
] = xx [ 160 ] * xx [ 159 ] + xx [ 163 ] * xx [ 161 ] ; xx [ 165 ] = xx [ 160
] * xx [ 161 ] - xx [ 163 ] * xx [ 159 ] ; xx [ 159 ] = xx [ 162 ] ; xx [ 160
] = xx [ 164 ] ; xx [ 161 ] = xx [ 165 ] ; xx [ 163 ] = 1.56860771278768e-13
; xx [ 166 ] = xx [ 165 ] * xx [ 163 ] ; xx [ 167 ] = xx [ 165 ] * xx [ 151 ]
; xx [ 168 ] = xx [ 163 ] * xx [ 162 ] + xx [ 151 ] * xx [ 164 ] ; xx [ 169 ]
= - xx [ 166 ] ; xx [ 170 ] = - xx [ 167 ] ; xx [ 171 ] = xx [ 168 ] ;
pm_math_Vector3_cross_ra ( xx + 159 , xx + 169 , xx + 172 ) ; xx [ 159 ] =
0.01923252987342762 - ( xx [ 140 ] * ( xx [ 172 ] - xx [ 158 ] * xx [ 166 ] )
- xx [ 151 ] ) ; xx [ 151 ] = 9.693066419407315e-3 - ( xx [ 163 ] + ( xx [
173 ] - xx [ 158 ] * xx [ 167 ] ) * xx [ 140 ] ) ; xx [ 160 ] =
0.04245578950477404 - ( xx [ 158 ] * xx [ 168 ] + xx [ 174 ] ) * xx [ 140 ] ;
xx [ 161 ] = xx [ 18 ] * state [ 51 ] ; xx [ 18 ] = cos ( xx [ 161 ] ) ; xx [
166 ] = 0.04161497595801117 ; xx [ 167 ] = sin ( xx [ 161 ] ) ; xx [ 161 ] =
xx [ 153 ] * xx [ 18 ] - xx [ 166 ] * xx [ 167 ] ; xx [ 168 ] = xx [ 153 ] *
xx [ 167 ] + xx [ 166 ] * xx [ 18 ] ; xx [ 153 ] = 0.3820992645960585 ; xx [
166 ] = 0.0122054420764352 ; xx [ 169 ] = xx [ 153 ] * xx [ 18 ] - xx [ 166 ]
* xx [ 167 ] ; xx [ 170 ] = xx [ 166 ] * xx [ 18 ] + xx [ 153 ] * xx [ 167 ]
; xx [ 18 ] = - xx [ 170 ] ; xx [ 171 ] = xx [ 168 ] ; xx [ 172 ] = xx [ 169
] ; xx [ 173 ] = xx [ 18 ] ; xx [ 153 ] = 1.570981369614329e-13 ; xx [ 166 ]
= xx [ 153 ] * xx [ 170 ] ; xx [ 167 ] = 0.02074999999999926 ; xx [ 174 ] =
xx [ 167 ] * xx [ 170 ] ; xx [ 170 ] = xx [ 168 ] * xx [ 153 ] + xx [ 169 ] *
xx [ 167 ] ; xx [ 175 ] = xx [ 166 ] ; xx [ 176 ] = xx [ 174 ] ; xx [ 177 ] =
xx [ 170 ] ; pm_math_Vector3_cross_ra ( xx + 171 , xx + 175 , xx + 178 ) ; xx
[ 171 ] = 9.407826863410462e-3 - ( xx [ 140 ] * ( xx [ 178 ] + xx [ 166 ] *
xx [ 161 ] ) - xx [ 167 ] ) ; xx [ 166 ] = - ( 0.01511804393362148 + xx [ 153
] + ( xx [ 174 ] * xx [ 161 ] + xx [ 179 ] ) * xx [ 140 ] ) ; xx [ 167 ] =
0.03122651021731275 - ( xx [ 170 ] * xx [ 161 ] + xx [ 180 ] ) * xx [ 140 ] ;
pm_math_Quaternion_compose_ra ( xx + 28 , xx + 39 , xx + 172 ) ; xx [ 176 ] =
xx [ 24 ] ; xx [ 177 ] = xx [ 26 ] ; xx [ 178 ] = xx [ 32 ] ;
pm_math_Quaternion_xform_ra ( xx + 28 , xx + 176 , xx + 179 ) ; xx [ 140 ] =
xx [ 179 ] - xx [ 14 ] ; xx [ 14 ] = xx [ 180 ] + xx [ 16 ] ; xx [ 170 ] = xx
[ 181 ] + xx [ 17 ] ; xx [ 179 ] = xx [ 52 ] ; xx [ 180 ] = xx [ 53 ] ; xx [
181 ] = xx [ 54 ] ; xx [ 182 ] = xx [ 61 ] ; pm_math_Quaternion_compose_ra (
xx + 179 , xx + 88 , xx + 183 ) ; pm_math_Quaternion_xform_ra ( xx + 179 , xx
+ 75 , xx + 187 ) ; pm_math_Quaternion_compose_ra ( xx + 172 , xx + 55 , xx +
190 ) ; xx [ 194 ] = xx [ 38 ] ; xx [ 195 ] = xx [ 43 ] ; xx [ 196 ] = xx [
44 ] ; pm_math_Quaternion_xform_ra ( xx + 172 , xx + 194 , xx + 197 ) ;
pm_math_Quaternion_compose_ra ( xx + 39 , xx + 55 , xx + 200 ) ;
pm_math_Quaternion_xform_ra ( xx + 39 , xx + 194 , xx + 204 ) ;
pm_math_Quaternion_compose_ra ( xx + 98 , xx + 114 , xx + 207 ) ; xx [ 211 ]
= xx [ 102 ] ; xx [ 212 ] = xx [ 104 ] ; xx [ 213 ] = xx [ 107 ] ;
pm_math_Quaternion_xform_ra ( xx + 98 , xx + 211 , xx + 214 ) ;
pm_math_Quaternion_compose_ra ( xx + 8 , xx + 28 , xx + 217 ) ;
pm_math_Quaternion_xform_ra ( xx + 8 , xx + 15 , xx + 221 ) ; xx [ 224 ] = xx
[ 221 ] + xx [ 3 ] ; xx [ 225 ] = xx [ 222 ] + xx [ 12 ] ; xx [ 221 ] = xx [
223 ] + xx [ 13 ] ; pm_math_Quaternion_compose_ra ( xx + 217 , xx + 39 , xx +
226 ) ; pm_math_Quaternion_xform_ra ( xx + 217 , xx + 176 , xx + 230 ) ; xx [
222 ] = xx [ 230 ] + xx [ 224 ] ; xx [ 223 ] = xx [ 231 ] + xx [ 225 ] ; xx [
230 ] = xx [ 232 ] + xx [ 221 ] ; pm_math_Quaternion_compose_ra ( xx + 226 ,
xx + 65 , xx + 231 ) ; xx [ 235 ] = xx [ 45 ] ; xx [ 236 ] = xx [ 48 ] ; xx [
237 ] = xx [ 51 ] ; pm_math_Quaternion_xform_ra ( xx + 226 , xx + 235 , xx +
238 ) ; pm_math_Quaternion_compose_ra ( xx + 8 , xx + 179 , xx + 241 ) ; xx [
245 ] = xx [ 64 ] ; xx [ 246 ] = xx [ 71 ] ; xx [ 247 ] = xx [ 74 ] ;
pm_math_Quaternion_xform_ra ( xx + 8 , xx + 245 , xx + 248 ) ; xx [ 251 ] =
xx [ 248 ] + xx [ 3 ] ; xx [ 252 ] = xx [ 249 ] + xx [ 12 ] ; xx [ 248 ] = xx
[ 250 ] + xx [ 13 ] ; pm_math_Quaternion_compose_ra ( xx + 241 , xx + 98 , xx
+ 253 ) ; xx [ 257 ] = xx [ 81 ] ; xx [ 258 ] = xx [ 85 ] ; xx [ 259 ] = xx [
87 ] ; pm_math_Quaternion_xform_ra ( xx + 241 , xx + 257 , xx + 260 ) ; xx [
249 ] = xx [ 260 ] + xx [ 251 ] ; xx [ 250 ] = xx [ 261 ] + xx [ 252 ] ; xx [
260 ] = xx [ 262 ] + xx [ 248 ] ; pm_math_Quaternion_compose_ra ( xx + 253 ,
xx + 125 , xx + 261 ) ; xx [ 265 ] = xx [ 110 ] ; xx [ 266 ] = xx [ 97 ] ; xx
[ 267 ] = xx [ 103 ] ; pm_math_Quaternion_xform_ra ( xx + 253 , xx + 265 , xx
+ 268 ) ; xx [ 271 ] = state [ 10 ] ; xx [ 272 ] = state [ 11 ] ; xx [ 273 ]
= state [ 12 ] ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 271 , xx + 274 )
; xx [ 4 ] = state [ 7 ] ; xx [ 5 ] = state [ 8 ] ; xx [ 6 ] = state [ 9 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 8 , xx + 4 , xx + 271 ) ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 0 , xx + 4 ) ; xx [ 0 ] = xx [ 271
] + xx [ 4 ] ; xx [ 1 ] = xx [ 272 ] + xx [ 5 ] ; xx [ 2 ] = xx [ 273 ] + xx
[ 6 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 28 , xx + 274 , xx + 4 ) ;
xx [ 7 ] = xx [ 4 ] + xx [ 20 ] * state [ 15 ] ; xx [ 271 ] = xx [ 5 ] + xx [
22 ] * state [ 15 ] ; xx [ 4 ] = xx [ 6 ] + xx [ 23 ] * state [ 15 ] ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 15 , xx + 277 ) ; xx [ 280 ] = xx
[ 277 ] + xx [ 0 ] ; xx [ 281 ] = xx [ 278 ] + xx [ 1 ] ; xx [ 282 ] = xx [
279 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 28 , xx + 280 ,
xx + 277 ) ; xx [ 5 ] = xx [ 277 ] + xx [ 20 ] * state [ 16 ] -
6.480496605447651e-3 * state [ 15 ] ; xx [ 6 ] = xx [ 278 ] +
9.242608862973643e-5 * state [ 15 ] + xx [ 22 ] * state [ 16 ] ; xx [ 20 ] =
xx [ 279 ] + xx [ 23 ] * state [ 16 ] - 2.248932226692751e-5 * state [ 15 ] ;
xx [ 277 ] = xx [ 7 ] ; xx [ 278 ] = xx [ 271 ] ; xx [ 279 ] = xx [ 4 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 39 , xx + 277 , xx + 280 ) ; xx [
22 ] = xx [ 280 ] - xx [ 21 ] * state [ 18 ] ; xx [ 21 ] = xx [ 281 ] + xx [
33 ] * state [ 18 ] ; xx [ 23 ] = xx [ 282 ] + xx [ 34 ] * state [ 18 ] ;
pm_math_Vector3_cross_ra ( xx + 277 , xx + 176 , xx + 280 ) ; xx [ 176 ] = xx
[ 280 ] + xx [ 5 ] ; xx [ 177 ] = xx [ 281 ] + xx [ 6 ] ; xx [ 178 ] = xx [
282 ] + xx [ 20 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 39 , xx + 176 ,
xx + 277 ) ; xx [ 33 ] = xx [ 277 ] + 1.439701931685288e-6 * state [ 18 ] ;
xx [ 34 ] = xx [ 278 ] + 0.01494182793334928 * state [ 18 ] ; xx [ 176 ] = xx
[ 279 ] + 1.536145228271068e-6 * state [ 18 ] ; xx [ 277 ] = xx [ 22 ] ; xx [
278 ] = xx [ 21 ] ; xx [ 279 ] = xx [ 23 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 55 , xx + 277 , xx + 280 ) ;
pm_math_Vector3_cross_ra ( xx + 277 , xx + 194 , xx + 283 ) ; xx [ 194 ] = xx
[ 283 ] + xx [ 33 ] ; xx [ 195 ] = xx [ 284 ] + xx [ 34 ] ; xx [ 196 ] = xx [
285 ] + xx [ 176 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 55 , xx + 194
, xx + 283 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 277 , xx +
194 ) ; pm_math_Vector3_cross_ra ( xx + 277 , xx + 235 , xx + 286 ) ; xx [
235 ] = xx [ 286 ] + xx [ 33 ] ; xx [ 236 ] = xx [ 287 ] + xx [ 34 ] ; xx [
237 ] = xx [ 288 ] + xx [ 176 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
65 , xx + 235 , xx + 277 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 179 ,
xx + 274 , xx + 235 ) ; pm_math_Vector3_cross_ra ( xx + 274 , xx + 245 , xx +
286 ) ; xx [ 245 ] = xx [ 286 ] + xx [ 0 ] ; xx [ 246 ] = xx [ 287 ] + xx [ 1
] ; xx [ 247 ] = xx [ 288 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 179 , xx + 245 , xx + 286 ) ; xx [ 177 ] = xx [ 286 ] +
0.9999999602050231 * state [ 26 ] ; xx [ 178 ] = xx [ 287 ] +
2.821169119813227e-4 * state [ 26 ] ; pm_math_Quaternion_inverseXform_ra ( xx
+ 88 , xx + 235 , xx + 179 ) ; pm_math_Vector3_cross_ra ( xx + 235 , xx + 75
, xx + 245 ) ; xx [ 289 ] = xx [ 245 ] + xx [ 177 ] ; xx [ 290 ] = xx [ 246 ]
+ xx [ 178 ] ; xx [ 291 ] = xx [ 247 ] + xx [ 288 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 88 , xx + 289 , xx + 245 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx + 235 , xx + 289 ) ; xx [
182 ] = xx [ 289 ] - xx [ 79 ] * state [ 32 ] ; xx [ 79 ] = xx [ 290 ] + xx [
92 ] * state [ 32 ] ; xx [ 92 ] = xx [ 291 ] + xx [ 93 ] * state [ 32 ] ;
pm_math_Vector3_cross_ra ( xx + 235 , xx + 257 , xx + 289 ) ; xx [ 257 ] = xx
[ 289 ] + xx [ 177 ] ; xx [ 258 ] = xx [ 290 ] + xx [ 178 ] ; xx [ 259 ] = xx
[ 291 ] + xx [ 288 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx +
257 , xx + 289 ) ; xx [ 93 ] = xx [ 289 ] + 1.439701935798469e-6 * state [ 32
] ; xx [ 257 ] = xx [ 290 ] + 0.01494182793334727 * state [ 32 ] ; xx [ 258 ]
= xx [ 291 ] + 1.53614523658561e-6 * state [ 32 ] ; xx [ 289 ] = xx [ 182 ] ;
xx [ 290 ] = xx [ 79 ] ; xx [ 291 ] = xx [ 92 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 114 , xx + 289 , xx + 292 ) ;
pm_math_Vector3_cross_ra ( xx + 289 , xx + 211 , xx + 295 ) ; xx [ 211 ] = xx
[ 295 ] + xx [ 93 ] ; xx [ 212 ] = xx [ 296 ] + xx [ 257 ] ; xx [ 213 ] = xx
[ 297 ] + xx [ 258 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 114 , xx +
211 , xx + 295 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 289 ,
xx + 211 ) ; pm_math_Vector3_cross_ra ( xx + 289 , xx + 265 , xx + 298 ) ; xx
[ 265 ] = xx [ 298 ] + xx [ 93 ] ; xx [ 266 ] = xx [ 299 ] + xx [ 257 ] ; xx
[ 267 ] = xx [ 300 ] + xx [ 258 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
125 , xx + 265 , xx + 289 ) ; xx [ 298 ] = xx [ 105 ] ; xx [ 299 ] = xx [ 111
] ; xx [ 300 ] = xx [ 112 ] ; xx [ 301 ] = xx [ 113 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 298 , xx + 274 , xx + 265 ) ; xx [
302 ] = xx [ 118 ] ; xx [ 303 ] = xx [ 119 ] ; xx [ 304 ] = xx [ 121 ] ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 302 , xx + 305 ) ; xx [ 302 ] = xx
[ 305 ] + xx [ 0 ] ; xx [ 303 ] = xx [ 306 ] + xx [ 1 ] ; xx [ 304 ] = xx [
307 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 298 , xx + 302 ,
xx + 305 ) ; xx [ 259 ] = 0.9999921011077627 ; xx [ 298 ] = xx [ 122 ] ; xx [
299 ] = xx [ 123 ] ; xx [ 300 ] = xx [ 124 ] ; xx [ 301 ] = xx [ 129 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 298 , xx + 274 , xx + 302 ) ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 130 , xx + 308 ) ; xx [ 311 ] = xx
[ 308 ] + xx [ 0 ] ; xx [ 312 ] = xx [ 309 ] + xx [ 1 ] ; xx [ 313 ] = xx [
310 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 298 , xx + 311 ,
xx + 308 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 133 , xx + 274 , xx +
298 ) ; pm_math_Vector3_cross_ra ( xx + 274 , xx + 137 , xx + 311 ) ; xx [
314 ] = xx [ 311 ] + xx [ 0 ] ; xx [ 315 ] = xx [ 312 ] + xx [ 1 ] ; xx [ 316
] = xx [ 313 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 133 ,
xx + 314 , xx + 311 ) ; xx [ 314 ] = xx [ 141 ] ; xx [ 315 ] = xx [ 145 ] ;
xx [ 316 ] = xx [ 146 ] ; xx [ 317 ] = xx [ 147 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 274 , xx + 318 ) ; xx [
321 ] = xx [ 150 ] ; xx [ 322 ] = xx [ 143 ] ; xx [ 323 ] = xx [ 144 ] ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 321 , xx + 324 ) ; xx [ 321 ] = xx
[ 324 ] + xx [ 0 ] ; xx [ 322 ] = xx [ 325 ] + xx [ 1 ] ; xx [ 323 ] = xx [
326 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 321 ,
xx + 324 ) ; xx [ 314 ] = xx [ 149 ] ; xx [ 315 ] = xx [ 154 ] ; xx [ 316 ] =
xx [ 155 ] ; xx [ 317 ] = xx [ 156 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 314 , xx + 274 , xx + 321 ) ; xx [ 327 ] = xx [ 157 ] ; xx [ 328 ] = xx
[ 148 ] ; xx [ 329 ] = xx [ 152 ] ; pm_math_Vector3_cross_ra ( xx + 274 , xx
+ 327 , xx + 330 ) ; xx [ 327 ] = xx [ 330 ] + xx [ 0 ] ; xx [ 328 ] = xx [
331 ] + xx [ 1 ] ; xx [ 329 ] = xx [ 332 ] + xx [ 2 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 327 , xx + 330 ) ; xx [
314 ] = xx [ 158 ] ; xx [ 315 ] = xx [ 162 ] ; xx [ 316 ] = xx [ 164 ] ; xx [
317 ] = xx [ 165 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 274
, xx + 327 ) ; xx [ 333 ] = xx [ 159 ] ; xx [ 334 ] = xx [ 151 ] ; xx [ 335 ]
= xx [ 160 ] ; pm_math_Vector3_cross_ra ( xx + 274 , xx + 333 , xx + 336 ) ;
xx [ 333 ] = xx [ 336 ] + xx [ 0 ] ; xx [ 334 ] = xx [ 337 ] + xx [ 1 ] ; xx
[ 335 ] = xx [ 338 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
314 , xx + 333 , xx + 336 ) ; xx [ 314 ] = xx [ 161 ] ; xx [ 315 ] = xx [ 168
] ; xx [ 316 ] = xx [ 169 ] ; xx [ 317 ] = xx [ 18 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 274 , xx + 333 ) ; xx [
339 ] = xx [ 171 ] ; xx [ 340 ] = xx [ 166 ] ; xx [ 341 ] = xx [ 167 ] ;
pm_math_Vector3_cross_ra ( xx + 274 , xx + 339 , xx + 342 ) ; xx [ 339 ] = xx
[ 342 ] + xx [ 0 ] ; xx [ 340 ] = xx [ 343 ] + xx [ 1 ] ; xx [ 341 ] = xx [
344 ] + xx [ 2 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 314 , xx + 339 ,
xx + 342 ) ; motionData [ 0 ] = xx [ 8 ] ; motionData [ 1 ] = xx [ 9 ] ;
motionData [ 2 ] = xx [ 10 ] ; motionData [ 3 ] = xx [ 11 ] ; motionData [ 4
] = xx [ 3 ] ; motionData [ 5 ] = xx [ 12 ] ; motionData [ 6 ] = xx [ 13 ] ;
motionData [ 7 ] = xx [ 28 ] ; motionData [ 8 ] = xx [ 29 ] ; motionData [ 9
] = xx [ 30 ] ; motionData [ 10 ] = xx [ 31 ] ; motionData [ 11 ] = xx [ 15 ]
; motionData [ 12 ] = xx [ 16 ] ; motionData [ 13 ] = xx [ 17 ] ; motionData
[ 14 ] = xx [ 39 ] ; motionData [ 15 ] = xx [ 40 ] ; motionData [ 16 ] = xx [
41 ] ; motionData [ 17 ] = xx [ 42 ] ; motionData [ 18 ] = xx [ 24 ] ;
motionData [ 19 ] = xx [ 26 ] ; motionData [ 20 ] = xx [ 32 ] ; motionData [
21 ] = xx [ 55 ] ; motionData [ 22 ] = xx [ 56 ] ; motionData [ 23 ] = xx [
57 ] ; motionData [ 24 ] = xx [ 58 ] ; motionData [ 25 ] = xx [ 38 ] ;
motionData [ 26 ] = xx [ 43 ] ; motionData [ 27 ] = xx [ 44 ] ; motionData [
28 ] = xx [ 65 ] ; motionData [ 29 ] = xx [ 66 ] ; motionData [ 30 ] = xx [
67 ] ; motionData [ 31 ] = xx [ 68 ] ; motionData [ 32 ] = xx [ 45 ] ;
motionData [ 33 ] = xx [ 48 ] ; motionData [ 34 ] = xx [ 51 ] ; motionData [
35 ] = xx [ 52 ] ; motionData [ 36 ] = xx [ 53 ] ; motionData [ 37 ] = xx [
54 ] ; motionData [ 38 ] = xx [ 61 ] ; motionData [ 39 ] = xx [ 64 ] ;
motionData [ 40 ] = xx [ 71 ] ; motionData [ 41 ] = xx [ 74 ] ; motionData [
42 ] = xx [ 88 ] ; motionData [ 43 ] = xx [ 89 ] ; motionData [ 44 ] = xx [
90 ] ; motionData [ 45 ] = xx [ 91 ] ; motionData [ 46 ] = xx [ 75 ] ;
motionData [ 47 ] = xx [ 76 ] ; motionData [ 48 ] = xx [ 77 ] ; motionData [
49 ] = xx [ 98 ] ; motionData [ 50 ] = xx [ 99 ] ; motionData [ 51 ] = xx [
100 ] ; motionData [ 52 ] = xx [ 101 ] ; motionData [ 53 ] = xx [ 81 ] ;
motionData [ 54 ] = xx [ 85 ] ; motionData [ 55 ] = xx [ 87 ] ; motionData [
56 ] = xx [ 114 ] ; motionData [ 57 ] = xx [ 115 ] ; motionData [ 58 ] = xx [
116 ] ; motionData [ 59 ] = xx [ 117 ] ; motionData [ 60 ] = xx [ 102 ] ;
motionData [ 61 ] = xx [ 104 ] ; motionData [ 62 ] = xx [ 107 ] ; motionData
[ 63 ] = xx [ 125 ] ; motionData [ 64 ] = xx [ 126 ] ; motionData [ 65 ] = xx
[ 127 ] ; motionData [ 66 ] = xx [ 128 ] ; motionData [ 67 ] = xx [ 110 ] ;
motionData [ 68 ] = xx [ 97 ] ; motionData [ 69 ] = xx [ 103 ] ; motionData [
70 ] = xx [ 105 ] ; motionData [ 71 ] = xx [ 111 ] ; motionData [ 72 ] = xx [
112 ] ; motionData [ 73 ] = xx [ 113 ] ; motionData [ 74 ] = xx [ 118 ] ;
motionData [ 75 ] = xx [ 119 ] ; motionData [ 76 ] = xx [ 121 ] ; motionData
[ 77 ] = xx [ 122 ] ; motionData [ 78 ] = xx [ 123 ] ; motionData [ 79 ] = xx
[ 124 ] ; motionData [ 80 ] = xx [ 129 ] ; motionData [ 81 ] = xx [ 130 ] ;
motionData [ 82 ] = xx [ 131 ] ; motionData [ 83 ] = xx [ 132 ] ; motionData
[ 84 ] = xx [ 133 ] ; motionData [ 85 ] = xx [ 134 ] ; motionData [ 86 ] = xx
[ 135 ] ; motionData [ 87 ] = xx [ 136 ] ; motionData [ 88 ] = xx [ 137 ] ;
motionData [ 89 ] = xx [ 138 ] ; motionData [ 90 ] = xx [ 139 ] ; motionData
[ 91 ] = xx [ 141 ] ; motionData [ 92 ] = xx [ 145 ] ; motionData [ 93 ] = xx
[ 146 ] ; motionData [ 94 ] = xx [ 147 ] ; motionData [ 95 ] = xx [ 150 ] ;
motionData [ 96 ] = xx [ 143 ] ; motionData [ 97 ] = xx [ 144 ] ; motionData
[ 98 ] = xx [ 149 ] ; motionData [ 99 ] = xx [ 154 ] ; motionData [ 100 ] =
xx [ 155 ] ; motionData [ 101 ] = xx [ 156 ] ; motionData [ 102 ] = xx [ 157
] ; motionData [ 103 ] = xx [ 148 ] ; motionData [ 104 ] = xx [ 152 ] ;
motionData [ 105 ] = xx [ 158 ] ; motionData [ 106 ] = xx [ 162 ] ;
motionData [ 107 ] = xx [ 164 ] ; motionData [ 108 ] = xx [ 165 ] ;
motionData [ 109 ] = xx [ 159 ] ; motionData [ 110 ] = xx [ 151 ] ;
motionData [ 111 ] = xx [ 160 ] ; motionData [ 112 ] = xx [ 161 ] ;
motionData [ 113 ] = xx [ 168 ] ; motionData [ 114 ] = xx [ 169 ] ;
motionData [ 115 ] = xx [ 18 ] ; motionData [ 116 ] = xx [ 171 ] ; motionData
[ 117 ] = xx [ 166 ] ; motionData [ 118 ] = xx [ 167 ] ; motionData [ 119 ] =
xx [ 172 ] ; motionData [ 120 ] = xx [ 173 ] ; motionData [ 121 ] = xx [ 174
] ; motionData [ 122 ] = xx [ 175 ] ; motionData [ 123 ] = xx [ 140 ] ;
motionData [ 124 ] = xx [ 14 ] ; motionData [ 125 ] = xx [ 170 ] ; motionData
[ 126 ] = xx [ 183 ] ; motionData [ 127 ] = xx [ 184 ] ; motionData [ 128 ] =
xx [ 185 ] ; motionData [ 129 ] = xx [ 186 ] ; motionData [ 130 ] = xx [ 187
] + xx [ 63 ] + xx [ 62 ] ; motionData [ 131 ] = xx [ 188 ] - xx [ 70 ] + xx
[ 69 ] ; motionData [ 132 ] = xx [ 189 ] + xx [ 72 ] - xx [ 73 ] ; motionData
[ 133 ] = xx [ 190 ] ; motionData [ 134 ] = xx [ 191 ] ; motionData [ 135 ] =
xx [ 192 ] ; motionData [ 136 ] = xx [ 193 ] ; motionData [ 137 ] = xx [ 197
] + xx [ 140 ] ; motionData [ 138 ] = xx [ 198 ] + xx [ 14 ] ; motionData [
139 ] = xx [ 199 ] + xx [ 170 ] ; motionData [ 140 ] = xx [ 200 ] ;
motionData [ 141 ] = xx [ 201 ] ; motionData [ 142 ] = xx [ 202 ] ;
motionData [ 143 ] = xx [ 203 ] ; motionData [ 144 ] = xx [ 204 ] - xx [ 35 ]
- xx [ 19 ] ; motionData [ 145 ] = xx [ 205 ] - xx [ 36 ] + xx [ 25 ] ;
motionData [ 146 ] = xx [ 206 ] - xx [ 37 ] + xx [ 27 ] ; motionData [ 147 ]
= xx [ 207 ] ; motionData [ 148 ] = xx [ 208 ] ; motionData [ 149 ] = xx [
209 ] ; motionData [ 150 ] = xx [ 210 ] ; motionData [ 151 ] = xx [ 214 ] -
xx [ 94 ] + xx [ 78 ] ; motionData [ 152 ] = xx [ 215 ] - xx [ 95 ] + xx [ 84
] ; motionData [ 153 ] = xx [ 216 ] - xx [ 96 ] + xx [ 86 ] ; motionData [
154 ] = xx [ 217 ] ; motionData [ 155 ] = xx [ 218 ] ; motionData [ 156 ] =
xx [ 219 ] ; motionData [ 157 ] = xx [ 220 ] ; motionData [ 158 ] = xx [ 224
] ; motionData [ 159 ] = xx [ 225 ] ; motionData [ 160 ] = xx [ 221 ] ;
motionData [ 161 ] = xx [ 226 ] ; motionData [ 162 ] = xx [ 227 ] ;
motionData [ 163 ] = xx [ 228 ] ; motionData [ 164 ] = xx [ 229 ] ;
motionData [ 165 ] = xx [ 222 ] ; motionData [ 166 ] = xx [ 223 ] ;
motionData [ 167 ] = xx [ 230 ] ; motionData [ 168 ] = xx [ 231 ] ;
motionData [ 169 ] = xx [ 232 ] ; motionData [ 170 ] = xx [ 233 ] ;
motionData [ 171 ] = xx [ 234 ] ; motionData [ 172 ] = xx [ 238 ] + xx [ 222
] ; motionData [ 173 ] = xx [ 239 ] + xx [ 223 ] ; motionData [ 174 ] = xx [
240 ] + xx [ 230 ] ; motionData [ 175 ] = xx [ 241 ] ; motionData [ 176 ] =
xx [ 242 ] ; motionData [ 177 ] = xx [ 243 ] ; motionData [ 178 ] = xx [ 244
] ; motionData [ 179 ] = xx [ 251 ] ; motionData [ 180 ] = xx [ 252 ] ;
motionData [ 181 ] = xx [ 248 ] ; motionData [ 182 ] = xx [ 253 ] ;
motionData [ 183 ] = xx [ 254 ] ; motionData [ 184 ] = xx [ 255 ] ;
motionData [ 185 ] = xx [ 256 ] ; motionData [ 186 ] = xx [ 249 ] ;
motionData [ 187 ] = xx [ 250 ] ; motionData [ 188 ] = xx [ 260 ] ;
motionData [ 189 ] = xx [ 261 ] ; motionData [ 190 ] = xx [ 262 ] ;
motionData [ 191 ] = xx [ 263 ] ; motionData [ 192 ] = xx [ 264 ] ;
motionData [ 193 ] = xx [ 268 ] + xx [ 249 ] ; motionData [ 194 ] = xx [ 269
] + xx [ 250 ] ; motionData [ 195 ] = xx [ 270 ] + xx [ 260 ] ; motionData [
196 ] = xx [ 274 ] ; motionData [ 197 ] = xx [ 275 ] ; motionData [ 198 ] =
xx [ 276 ] ; motionData [ 199 ] = xx [ 0 ] ; motionData [ 200 ] = xx [ 1 ] ;
motionData [ 201 ] = xx [ 2 ] ; motionData [ 202 ] = xx [ 7 ] ; motionData [
203 ] = xx [ 271 ] ; motionData [ 204 ] = xx [ 4 ] ; motionData [ 205 ] = xx
[ 5 ] ; motionData [ 206 ] = xx [ 6 ] ; motionData [ 207 ] = xx [ 20 ] ;
motionData [ 208 ] = xx [ 22 ] ; motionData [ 209 ] = xx [ 21 ] ; motionData
[ 210 ] = xx [ 23 ] ; motionData [ 211 ] = xx [ 33 ] ; motionData [ 212 ] =
xx [ 34 ] ; motionData [ 213 ] = xx [ 176 ] ; motionData [ 214 ] = xx [ 280 ]
+ xx [ 47 ] * state [ 21 ] ; motionData [ 215 ] = xx [ 281 ] - xx [ 49 ] *
state [ 21 ] ; motionData [ 216 ] = xx [ 282 ] - xx [ 50 ] * state [ 21 ] ;
motionData [ 217 ] = xx [ 283 ] + 3.296413151917778e-7 * state [ 21 ] + xx [
47 ] * state [ 22 ] ; motionData [ 218 ] = xx [ 284 ] + 7.422626626299722e-9
* state [ 21 ] - xx [ 49 ] * state [ 22 ] ; motionData [ 219 ] = xx [ 285 ] -
( 1.428754024949564e-6 * state [ 21 ] + xx [ 50 ] * state [ 22 ] ) ;
motionData [ 220 ] = xx [ 194 ] - xx [ 46 ] * state [ 24 ] ; motionData [ 221
] = xx [ 195 ] + xx [ 59 ] * state [ 24 ] ; motionData [ 222 ] = xx [ 196 ] +
xx [ 60 ] * state [ 24 ] ; motionData [ 223 ] = xx [ 277 ] +
2.971385142511557e-7 * state [ 24 ] ; motionData [ 224 ] = xx [ 278 ] +
1.158326953518261e-13 * state [ 24 ] ; motionData [ 225 ] = xx [ 279 ] -
3.938183989403888e-8 * state [ 24 ] ; motionData [ 226 ] = xx [ 235 ] ;
motionData [ 227 ] = xx [ 236 ] ; motionData [ 228 ] = xx [ 237 ] ;
motionData [ 229 ] = xx [ 177 ] ; motionData [ 230 ] = xx [ 178 ] ;
motionData [ 231 ] = xx [ 288 ] ; motionData [ 232 ] = xx [ 179 ] + xx [ 80 ]
* state [ 29 ] ; motionData [ 233 ] = xx [ 180 ] + xx [ 82 ] * state [ 29 ] ;
motionData [ 234 ] = xx [ 181 ] + xx [ 83 ] * state [ 29 ] ; motionData [ 235
] = xx [ 245 ] + xx [ 80 ] * state [ 30 ] - 6.480496605446796e-3 * state [ 29
] ; motionData [ 236 ] = xx [ 246 ] + 9.242608862956347e-5 * state [ 29 ] +
xx [ 82 ] * state [ 30 ] ; motionData [ 237 ] = xx [ 247 ] + xx [ 83 ] *
state [ 30 ] - 2.2489322266878e-5 * state [ 29 ] ; motionData [ 238 ] = xx [
182 ] ; motionData [ 239 ] = xx [ 79 ] ; motionData [ 240 ] = xx [ 92 ] ;
motionData [ 241 ] = xx [ 93 ] ; motionData [ 242 ] = xx [ 257 ] ; motionData
[ 243 ] = xx [ 258 ] ; motionData [ 244 ] = xx [ 292 ] + xx [ 106 ] * state [
35 ] ; motionData [ 245 ] = xx [ 293 ] - xx [ 108 ] * state [ 35 ] ;
motionData [ 246 ] = xx [ 294 ] - xx [ 109 ] * state [ 35 ] ; motionData [
247 ] = xx [ 295 ] + 3.296413155003147e-7 * state [ 35 ] + xx [ 106 ] * state
[ 36 ] ; motionData [ 248 ] = xx [ 296 ] + 7.422626627203207e-9 * state [ 35
] - xx [ 108 ] * state [ 36 ] ; motionData [ 249 ] = xx [ 297 ] - (
1.428754025107962e-6 * state [ 35 ] + xx [ 109 ] * state [ 36 ] ) ;
motionData [ 250 ] = xx [ 211 ] - xx [ 46 ] * state [ 38 ] ; motionData [ 251
] = xx [ 212 ] + xx [ 59 ] * state [ 38 ] ; motionData [ 252 ] = xx [ 213 ] +
xx [ 120 ] * state [ 38 ] ; motionData [ 253 ] = xx [ 289 ] +
2.971385144603623e-7 * state [ 38 ] ; motionData [ 254 ] = xx [ 290 ] +
1.158326948378051e-13 * state [ 38 ] ; motionData [ 255 ] = xx [ 291 ] -
3.938183964273754e-8 * state [ 38 ] ; motionData [ 256 ] = xx [ 265 ] ;
motionData [ 257 ] = xx [ 266 ] ; motionData [ 258 ] = xx [ 267 ] ;
motionData [ 259 ] = xx [ 305 ] ; motionData [ 260 ] = xx [ 306 ] +
3.974634836312298e-3 * state [ 40 ] ; motionData [ 261 ] = xx [ 307 ] + xx [
259 ] * state [ 40 ] ; motionData [ 262 ] = xx [ 302 ] ; motionData [ 263 ] =
xx [ 303 ] ; motionData [ 264 ] = xx [ 304 ] ; motionData [ 265 ] = xx [ 308
] ; motionData [ 266 ] = xx [ 309 ] - 3.974634836311137e-3 * state [ 42 ] ;
motionData [ 267 ] = xx [ 310 ] - xx [ 259 ] * state [ 42 ] ; motionData [
268 ] = xx [ 298 ] ; motionData [ 269 ] = xx [ 299 ] ; motionData [ 270 ] =
xx [ 300 ] ; motionData [ 271 ] = xx [ 311 ] + 1.805437481450789e-4 * state [
44 ] ; motionData [ 272 ] = xx [ 312 ] + 0.9999924437328848 * state [ 44 ] ;
motionData [ 273 ] = xx [ 313 ] + 3.883282257123533e-3 * state [ 44 ] ;
motionData [ 274 ] = xx [ 318 ] - state [ 46 ] ; motionData [ 275 ] = xx [
319 ] ; motionData [ 276 ] = xx [ 320 ] ; motionData [ 277 ] = xx [ 324 ] ;
motionData [ 278 ] = xx [ 325 ] ; motionData [ 279 ] = xx [ 326 ] + xx [ 142
] * state [ 46 ] ; motionData [ 280 ] = xx [ 321 ] - state [ 48 ] ;
motionData [ 281 ] = xx [ 322 ] ; motionData [ 282 ] = xx [ 323 ] ;
motionData [ 283 ] = xx [ 330 ] ; motionData [ 284 ] = xx [ 331 ] ;
motionData [ 285 ] = xx [ 332 ] + xx [ 142 ] * state [ 48 ] ; motionData [
286 ] = xx [ 327 ] - state [ 50 ] ; motionData [ 287 ] = xx [ 328 ] ;
motionData [ 288 ] = xx [ 329 ] ; motionData [ 289 ] = xx [ 336 ] ;
motionData [ 290 ] = xx [ 337 ] ; motionData [ 291 ] = xx [ 338 ] + xx [ 163
] * state [ 50 ] ; motionData [ 292 ] = xx [ 333 ] + state [ 52 ] ;
motionData [ 293 ] = xx [ 334 ] ; motionData [ 294 ] = xx [ 335 ] ;
motionData [ 295 ] = xx [ 342 ] ; motionData [ 296 ] = xx [ 343 ] ;
motionData [ 297 ] = xx [ 344 ] - xx [ 153 ] * state [ 52 ] ; } static size_t
computeAssemblyError_0 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int *
rtdvi = rtdv -> mInts . mValues ; double xx [ 22 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = motionData [ 7 ]
; xx [ 1 ] = motionData [ 8 ] ; xx [ 2 ] = motionData [ 9 ] ; xx [ 3 ] =
motionData [ 10 ] ; xx [ 4 ] = 0.9912919252698491 ; xx [ 5 ] = -
0.1316824810541687 ; xx [ 6 ] = 1.970490389246134e-4 ; xx [ 7 ] =
6.519118568758793e-5 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx +
8 ) ; xx [ 4 ] = 0.9999980252749909 ; xx [ 5 ] = 1.987321342568947e-3 ; xx [
12 ] = xx [ 4 ] * motionData [ 70 ] + xx [ 5 ] * motionData [ 71 ] ; xx [ 13
] = xx [ 4 ] * motionData [ 71 ] - xx [ 5 ] * motionData [ 70 ] ; xx [ 14 ] =
xx [ 4 ] * motionData [ 72 ] - xx [ 5 ] * motionData [ 73 ] ; xx [ 15 ] = xx
[ 4 ] * motionData [ 73 ] + xx [ 5 ] * motionData [ 72 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 12 , xx + 4 ) ; xx [ 12
] = motionData [ 70 ] ; xx [ 13 ] = motionData [ 71 ] ; xx [ 14 ] =
motionData [ 72 ] ; xx [ 15 ] = motionData [ 73 ] ; xx [ 16 ] =
9.996376071803525e-5 ; xx [ 17 ] = 0.01545456410810322 ; xx [ 18 ] =
8.60794096301822e-3 ; pm_math_Quaternion_xform_ra ( xx + 12 , xx + 16 , xx +
19 ) ; xx [ 12 ] = 2.908691710579205e-6 ; xx [ 13 ] = 2.868076938248717e-3 ;
xx [ 14 ] = 4.94643927619723e-3 ; pm_math_Quaternion_xform_ra ( xx + 0 , xx +
12 , xx + 15 ) ; xx [ 0 ] = xx [ 19 ] + motionData [ 74 ] - ( xx [ 15 ] +
motionData [ 11 ] ) ; xx [ 1 ] = xx [ 20 ] + motionData [ 75 ] - ( xx [ 16 ]
+ motionData [ 12 ] ) ; xx [ 2 ] = xx [ 21 ] + motionData [ 76 ] - ( xx [ 17
] + motionData [ 13 ] ) ; xx [ 3 ] = 2.0 ; xx [ 12 ] = ( xx [ 8 ] * xx [ 10 ]
+ xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ] ; xx [ 13 ] = xx [ 3 ] * ( xx [ 10 ] * xx
[ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx [ 14 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx
[ 10 ] * xx [ 10 ] ) * xx [ 3 ] ; error [ 0 ] = xx [ 5 ] ; error [ 1 ] = xx [
6 ] ; error [ 2 ] = pm_math_Vector3_dot_ra ( xx + 0 , xx + 12 ) ; return 3 ;
} static size_t computeAssemblyError_1 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * error ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 20 ] ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0
] = 0.9999999900512558 ; xx [ 1 ] = 1.410584573942991e-4 ; xx [ 2 ] = xx [ 0
] * motionData [ 35 ] - xx [ 1 ] * motionData [ 38 ] ; xx [ 3 ] = xx [ 0 ] *
motionData [ 36 ] + xx [ 1 ] * motionData [ 37 ] ; xx [ 4 ] = xx [ 0 ] *
motionData [ 37 ] - xx [ 1 ] * motionData [ 36 ] ; xx [ 5 ] = xx [ 1 ] *
motionData [ 35 ] + xx [ 0 ] * motionData [ 38 ] ; xx [ 6 ] = -
0.9229169573264255 ; xx [ 7 ] = - 0.02080107143120539 ; xx [ 8 ] =
0.382547117836429 ; xx [ 9 ] = 0.03806977727395289 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 2 , xx + 6 , xx + 10 ) ; xx [ 6 ]
= motionData [ 35 ] ; xx [ 7 ] = motionData [ 36 ] ; xx [ 8 ] = motionData [
37 ] ; xx [ 9 ] = motionData [ 38 ] ; xx [ 13 ] = 3.383773392768062e-7 ; xx [
14 ] = 4.111205245035194e-3 ; xx [ 15 ] = 9.999999999999629e-4 ;
pm_math_Quaternion_xform_ra ( xx + 6 , xx + 13 , xx + 16 ) ; xx [ 6 ] =
0.08275766910702829 - ( xx [ 16 ] + motionData [ 39 ] ) ; xx [ 7 ] =
8.589776459020856e-3 - ( xx [ 17 ] + motionData [ 40 ] ) ; xx [ 8 ] = - (
0.09286193847400817 + xx [ 18 ] + motionData [ 41 ] ) ; xx [ 0 ] = 1.0 ; xx [
1 ] = xx [ 5 ] * xx [ 5 ] ; xx [ 9 ] = 2.0 ; xx [ 10 ] = xx [ 5 ] * xx [ 2 ]
; xx [ 13 ] = xx [ 3 ] * xx [ 4 ] ; xx [ 14 ] = xx [ 0 ] - ( xx [ 4 ] * xx [
4 ] + xx [ 1 ] ) * xx [ 9 ] ; xx [ 15 ] = ( xx [ 10 ] + xx [ 13 ] ) * xx [ 9
] ; xx [ 16 ] = xx [ 9 ] * ( xx [ 5 ] * xx [ 3 ] - xx [ 2 ] * xx [ 4 ] ) ; xx
[ 17 ] = xx [ 9 ] * ( xx [ 13 ] - xx [ 10 ] ) ; xx [ 18 ] = xx [ 0 ] - ( xx [
1 ] + xx [ 3 ] * xx [ 3 ] ) * xx [ 9 ] ; xx [ 19 ] = ( xx [ 3 ] * xx [ 2 ] +
xx [ 5 ] * xx [ 4 ] ) * xx [ 9 ] ; error [ 0 ] = xx [ 11 ] ; error [ 1 ] = xx
[ 12 ] ; error [ 2 ] = pm_math_Vector3_dot_ra ( xx + 6 , xx + 14 ) ; error [
3 ] = pm_math_Vector3_dot_ra ( xx + 6 , xx + 17 ) ; return 4 ; } static
size_t computeAssemblyError_2 ( const RuntimeDerivedValuesBundle * rtdv ,
const double * state , const int * modeVector , const double * motionData ,
double * error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const
int * rtdvi = rtdv -> mInts . mValues ; double xx [ 16 ] ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = -
0.9229169573264254 ; xx [ 1 ] = - 0.02080107143120469 ; xx [ 2 ] =
0.3825471178364293 ; xx [ 3 ] = 0.0380697772739531 ; xx [ 4 ] = motionData [
119 ] ; xx [ 5 ] = motionData [ 120 ] ; xx [ 6 ] = motionData [ 121 ] ; xx [
7 ] = motionData [ 122 ] ; xx [ 8 ] = - 0.4813049877214196 ; xx [ 9 ] = -
0.5180593628877815 ; xx [ 10 ] = - 0.5180751960991345 ; xx [ 11 ] = -
0.4812048384063002 ; pm_math_Quaternion_compose_ra ( xx + 4 , xx + 8 , xx +
12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 12 , xx + 8 ) ;
xx [ 0 ] = 0.013582786684373 ; xx [ 1 ] = - 2.58589188160877e-6 ; xx [ 2 ] =
- 0.01849903189296259 ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 0 , xx +
11 ) ; xx [ 0 ] = xx [ 11 ] + motionData [ 123 ] + 0.09006429198674837 ; xx [
1 ] = xx [ 12 ] + motionData [ 124 ] - 3.83018891385798e-3 ; xx [ 2 ] = xx [
13 ] + motionData [ 125 ] - 0.07934515183560176 ; xx [ 3 ] = -
0.7077022283680728 ; xx [ 4 ] = - 9.268355963183032e-3 ; xx [ 5 ] =
0.7064500361247099 ; error [ 0 ] = xx [ 9 ] ; error [ 1 ] = xx [ 10 ] ; error
[ 2 ] = pm_math_Vector3_dot_ra ( xx + 0 , xx + 3 ) ; return 3 ; } static
size_t computeAssemblyError_3 ( const RuntimeDerivedValuesBundle * rtdv ,
const double * state , const int * modeVector , const double * motionData ,
double * error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const
int * rtdvi = rtdv -> mInts . mValues ; double xx [ 18 ] ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = motionData [
84 ] ; xx [ 1 ] = motionData [ 85 ] ; xx [ 2 ] = motionData [ 86 ] ; xx [ 3 ]
= motionData [ 87 ] ; xx [ 4 ] = 0.4982139734058924 ; xx [ 5 ] = -
0.4963735918252776 ; xx [ 6 ] = - 0.501668831212296 ; xx [ 7 ] = -
0.5037107084742934 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8
) ; xx [ 4 ] = - 0.6795114020738748 ; xx [ 5 ] = 0.2557971615198039 ; xx [ 6
] = 0.2852066954013848 ; xx [ 7 ] = - 0.6256909840394302 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 4 , xx + 12 ) ; xx [ 4 ]
= 1.599092056331299e-3 ; xx [ 5 ] = 0.01882102580464309 ; xx [ 6 ] = -
1.169213083133524e-4 ; pm_math_Quaternion_xform_ra ( xx + 0 , xx + 4 , xx +
15 ) ; xx [ 0 ] = - ( 0.02470914529039604 + xx [ 15 ] + motionData [ 88 ] ) ;
xx [ 1 ] = - ( 0.02476486676677759 + xx [ 16 ] + motionData [ 89 ] ) ; xx [ 2
] = - ( 5.837790395081272e-3 + xx [ 17 ] + motionData [ 90 ] ) ; xx [ 3 ] =
2.0 ; xx [ 4 ] = ( xx [ 8 ] * xx [ 10 ] + xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ] ;
xx [ 5 ] = xx [ 3 ] * ( xx [ 10 ] * xx [ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx [
6 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx [ 10 ] * xx [ 10 ] ) * xx [ 3 ] ;
error [ 0 ] = xx [ 13 ] ; error [ 1 ] = xx [ 14 ] ; error [ 2 ] =
pm_math_Vector3_dot_ra ( xx + 0 , xx + 4 ) ; return 3 ; } static size_t
computeAssemblyError_4 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int *
rtdvi = rtdv -> mInts . mValues ; double xx [ 21 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = motionData [ 49 ]
; xx [ 1 ] = motionData [ 50 ] ; xx [ 2 ] = motionData [ 51 ] ; xx [ 3 ] =
motionData [ 52 ] ; xx [ 4 ] = 0.5180593628885556 ; xx [ 5 ] = -
0.4813049877207372 ; xx [ 6 ] = - 0.4812048384058716 ; xx [ 7 ] =
0.518075196099393 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8
) ; xx [ 4 ] = motionData [ 42 ] ; xx [ 5 ] = motionData [ 43 ] ; xx [ 6 ] =
motionData [ 44 ] ; xx [ 7 ] = motionData [ 45 ] ; xx [ 12 ] =
0.1316824810541773 ; xx [ 13 ] = 0.991291925269848 ; xx [ 14 ] =
6.51911856886885e-5 ; xx [ 15 ] = - 1.970490389163408e-4 ;
pm_math_Quaternion_compose_ra ( xx + 4 , xx + 12 , xx + 16 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 16 , xx + 12 ) ; xx [ 15
] = - 9.045674901471507e-5 ; xx [ 16 ] = - 2.965461248705281e-3 ; xx [ 17 ] =
0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 15 , xx +
18 ) ; xx [ 4 ] = - 0.01609995664951487 ; xx [ 5 ] = 2.933657308331041e-6 ;
xx [ 6 ] = - 0.0137731566582556 ; pm_math_Quaternion_xform_ra ( xx + 0 , xx +
4 , xx + 15 ) ; xx [ 0 ] = xx [ 18 ] + motionData [ 46 ] - ( xx [ 15 ] +
motionData [ 53 ] ) ; xx [ 1 ] = xx [ 19 ] + motionData [ 47 ] - ( xx [ 16 ]
+ motionData [ 54 ] ) ; xx [ 2 ] = xx [ 20 ] + motionData [ 48 ] - ( xx [ 17
] + motionData [ 55 ] ) ; xx [ 3 ] = 2.0 ; xx [ 4 ] = ( xx [ 8 ] * xx [ 10 ]
+ xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ] ; xx [ 5 ] = xx [ 3 ] * ( xx [ 10 ] * xx
[ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx [ 6 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx
[ 10 ] * xx [ 10 ] ) * xx [ 3 ] ; error [ 0 ] = xx [ 13 ] ; error [ 1 ] = xx
[ 14 ] ; error [ 2 ] = pm_math_Vector3_dot_ra ( xx + 0 , xx + 4 ) ; return 3
; } static size_t computeAssemblyError_5 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * error ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 21 ] ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0
] = motionData [ 126 ] ; xx [ 1 ] = motionData [ 127 ] ; xx [ 2 ] =
motionData [ 128 ] ; xx [ 3 ] = motionData [ 129 ] ; xx [ 4 ] =
0.991291925269849 ; xx [ 5 ] = - 0.1316824810541695 ; xx [ 6 ] =
1.970490389249778e-4 ; xx [ 7 ] = 6.519118568753968e-5 ;
pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8 ) ; xx [ 4 ] =
motionData [ 77 ] ; xx [ 5 ] = motionData [ 78 ] ; xx [ 6 ] = motionData [ 79
] ; xx [ 7 ] = motionData [ 80 ] ; xx [ 12 ] = 0.9999980252749867 ; xx [ 13 ]
= - 1.987321342568328e-3 ; xx [ 14 ] = 9.226742840440916e-8 ; xx [ 15 ] = -
1.833653884871776e-10 ; pm_math_Quaternion_compose_ra ( xx + 4 , xx + 12 , xx
+ 16 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 16 , xx + 12 )
; xx [ 15 ] = 9.477449468724687e-5 ; xx [ 16 ] = 0.01545456410835202 ; xx [
17 ] = 8.607941025578736e-3 ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 15
, xx + 18 ) ; xx [ 4 ] = 2.908691710755093e-6 ; xx [ 5 ] =
2.868076938250087e-3 ; xx [ 6 ] = 4.94643927619916e-3 ;
pm_math_Quaternion_xform_ra ( xx + 0 , xx + 4 , xx + 15 ) ; xx [ 0 ] = xx [
18 ] + motionData [ 81 ] - ( xx [ 15 ] + motionData [ 130 ] ) ; xx [ 1 ] = xx
[ 19 ] + motionData [ 82 ] - ( xx [ 16 ] + motionData [ 131 ] ) ; xx [ 2 ] =
xx [ 20 ] + motionData [ 83 ] - ( xx [ 17 ] + motionData [ 132 ] ) ; xx [ 3 ]
= 2.0 ; xx [ 4 ] = ( xx [ 8 ] * xx [ 10 ] + xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ]
; xx [ 5 ] = xx [ 3 ] * ( xx [ 10 ] * xx [ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx
[ 6 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx [ 10 ] * xx [ 10 ] ) * xx [ 3 ] ;
error [ 0 ] = xx [ 13 ] ; error [ 1 ] = xx [ 14 ] ; error [ 2 ] =
pm_math_Vector3_dot_ra ( xx + 0 , xx + 4 ) ; return 3 ; } static size_t
computeAssemblyError_6 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int *
rtdvi = rtdv -> mInts . mValues ; double xx [ 21 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = motionData [ 21 ]
; xx [ 1 ] = motionData [ 22 ] ; xx [ 2 ] = motionData [ 23 ] ; xx [ 3 ] =
motionData [ 24 ] ; xx [ 4 ] = 0.4931954203595264 ; xx [ 5 ] = -
0.4905194435002385 ; xx [ 6 ] = - 0.5067824161638587 ; xx [ 7 ] = -
0.509235245786982 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8
) ; xx [ 4 ] = motionData [ 28 ] ; xx [ 5 ] = motionData [ 29 ] ; xx [ 6 ] =
motionData [ 30 ] ; xx [ 7 ] = motionData [ 31 ] ; xx [ 12 ] =
0.6370506001153763 ; xx [ 13 ] = - 0.6370493477200306 ; xx [ 14 ] = -
0.3068667118838679 ; xx [ 15 ] = - 0.3068672067844752 ;
pm_math_Quaternion_compose_ra ( xx + 4 , xx + 12 , xx + 16 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 16 , xx + 12 ) ; xx [ 15
] = - 3.865146760306702e-8 ; xx [ 16 ] = - 5.289307885790379e-3 ; xx [ 17 ] =
- 3.071851003736677e-7 ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 15 , xx
+ 18 ) ; xx [ 4 ] = 2.392300359455244e-7 ; xx [ 5 ] = 3.887917980054749e-3 ;
xx [ 6 ] = 2.025360777142851e-5 ; pm_math_Quaternion_xform_ra ( xx + 0 , xx +
4 , xx + 15 ) ; xx [ 0 ] = xx [ 18 ] + motionData [ 32 ] - ( xx [ 15 ] +
motionData [ 25 ] ) ; xx [ 1 ] = xx [ 19 ] + motionData [ 33 ] - ( xx [ 16 ]
+ motionData [ 26 ] ) ; xx [ 2 ] = xx [ 20 ] + motionData [ 34 ] - ( xx [ 17
] + motionData [ 27 ] ) ; xx [ 3 ] = 2.0 ; xx [ 4 ] = ( xx [ 8 ] * xx [ 10 ]
+ xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ] ; xx [ 5 ] = xx [ 3 ] * ( xx [ 10 ] * xx
[ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx [ 6 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx
[ 10 ] * xx [ 10 ] ) * xx [ 3 ] ; error [ 0 ] = xx [ 13 ] ; error [ 1 ] = xx
[ 14 ] ; error [ 2 ] = pm_math_Vector3_dot_ra ( xx + 0 , xx + 4 ) ; return 3
; } static size_t computeAssemblyError_7 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * error ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 21 ] ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0
] = motionData [ 56 ] ; xx [ 1 ] = motionData [ 57 ] ; xx [ 2 ] = motionData
[ 58 ] ; xx [ 3 ] = motionData [ 59 ] ; xx [ 4 ] = 0.493195420359526 ; xx [ 5
] = - 0.4905194435002388 ; xx [ 6 ] = - 0.5067824161638583 ; xx [ 7 ] = -
0.5092352457869823 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8
) ; xx [ 4 ] = motionData [ 63 ] ; xx [ 5 ] = motionData [ 64 ] ; xx [ 6 ] =
motionData [ 65 ] ; xx [ 7 ] = motionData [ 66 ] ; xx [ 12 ] =
0.6370493477200303 ; xx [ 13 ] = 0.6370506001153765 ; xx [ 14 ] = -
0.3068672067844756 ; xx [ 15 ] = 0.3068667118838675 ;
pm_math_Quaternion_compose_ra ( xx + 4 , xx + 12 , xx + 16 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 16 , xx + 12 ) ; xx [ 15
] = - 3.975614476060287e-8 ; xx [ 16 ] = 2.710692114195085e-3 ; xx [ 17 ] = -
2.919897872271535e-7 ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 15 , xx +
18 ) ; xx [ 4 ] = 2.392300359480238e-7 ; xx [ 5 ] = 3.887917980054739e-3 ; xx
[ 6 ] = 2.025360777142854e-5 ; pm_math_Quaternion_xform_ra ( xx + 0 , xx + 4
, xx + 15 ) ; xx [ 0 ] = xx [ 18 ] + motionData [ 67 ] - ( xx [ 15 ] +
motionData [ 60 ] ) ; xx [ 1 ] = xx [ 19 ] + motionData [ 68 ] - ( xx [ 16 ]
+ motionData [ 61 ] ) ; xx [ 2 ] = xx [ 20 ] + motionData [ 69 ] - ( xx [ 17
] + motionData [ 62 ] ) ; xx [ 3 ] = 2.0 ; xx [ 4 ] = ( xx [ 8 ] * xx [ 10 ]
+ xx [ 9 ] * xx [ 11 ] ) * xx [ 3 ] ; xx [ 5 ] = xx [ 3 ] * ( xx [ 10 ] * xx
[ 11 ] - xx [ 8 ] * xx [ 9 ] ) ; xx [ 6 ] = 1.0 - ( xx [ 9 ] * xx [ 9 ] + xx
[ 10 ] * xx [ 10 ] ) * xx [ 3 ] ; error [ 0 ] = xx [ 13 ] ; error [ 1 ] = xx
[ 14 ] ; error [ 2 ] = pm_math_Vector3_dot_ra ( xx + 0 , xx + 4 ) ; return 3
; } static size_t computeAssemblyError_8 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * error ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 16 ] ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0
] = 0.02080107143120473 ; xx [ 1 ] = - 0.9229169573264255 ; xx [ 2 ] =
0.0380697772739531 ; xx [ 3 ] = - 0.3825471178364295 ; xx [ 4 ] = motionData
[ 133 ] ; xx [ 5 ] = motionData [ 134 ] ; xx [ 6 ] = motionData [ 135 ] ; xx
[ 7 ] = motionData [ 136 ] ; xx [ 8 ] = 0.8739852835857713 ; xx [ 9 ] =
2.165051543389225e-3 ; xx [ 10 ] = 0.4859456095725561 ; xx [ 11 ] =
1.378827208253894e-3 ; pm_math_Quaternion_compose_ra ( xx + 4 , xx + 8 , xx +
12 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 12 , xx + 4 ) ;
error [ 0 ] = xx [ 5 ] ; error [ 1 ] = xx [ 6 ] ; return 2 ; } static size_t
computeAssemblyError_9 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int *
rtdvi = rtdv -> mInts . mValues ; double xx [ 13 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] =
0.9999999900512558 ; xx [ 1 ] = motionData [ 147 ] ; xx [ 2 ] = motionData [
148 ] ; xx [ 3 ] = motionData [ 149 ] ; xx [ 4 ] = motionData [ 150 ] ; xx [
5 ] = - 1.115736180703704e-4 ; xx [ 6 ] = - 0.01615139904553786 ; xx [ 7 ] =
- 2.564403241205184e-3 ; xx [ 8 ] = 0.9998662629053028 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 1 ] =
1.410584573942991e-4 ; error [ 0 ] = - ( xx [ 0 ] * xx [ 10 ] + xx [ 1 ] * xx
[ 11 ] ) ; error [ 1 ] = xx [ 1 ] * xx [ 10 ] - xx [ 0 ] * xx [ 11 ] ; return
2 ; } static size_t computeAssemblyError_10 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * error ) { const double *
rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts .
mValues ; double xx [ 16 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state
; ( void ) modeVector ; xx [ 0 ] = 0.6378922378578669 ; xx [ 1 ] =
0.6673093902201903 ; xx [ 2 ] = - 0.2974211203852193 ; xx [ 3 ] =
0.2435823223336235 ; xx [ 4 ] = motionData [ 7 ] ; xx [ 5 ] = motionData [ 8
] ; xx [ 6 ] = motionData [ 9 ] ; xx [ 7 ] = motionData [ 10 ] ; xx [ 8 ] =
0.7940628178106506 ; xx [ 9 ] = 0.6078356671769065 ; xx [ 10 ] =
1.854318411243853e-4 ; xx [ 11 ] = - 9.323758217514975e-5 ;
pm_math_Quaternion_compose_ra ( xx + 4 , xx + 8 , xx + 12 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 0 , xx + 12 , xx + 4 ) ; error [
0 ] = xx [ 5 ] ; error [ 1 ] = xx [ 6 ] ; return 2 ; } static size_t
computeAssemblyError_11 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int *
rtdvi = rtdv -> mInts . mValues ; double xx [ 12 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = motionData [ 126
] ; xx [ 1 ] = motionData [ 127 ] ; xx [ 2 ] = motionData [ 128 ] ; xx [ 3 ]
= motionData [ 129 ] ; xx [ 4 ] = 0.4299358422201465 ; xx [ 5 ] = -
0.5615531320886276 ; xx [ 6 ] = - 0.5614212742353903 ; xx [ 7 ] = -
0.4296736019955341 ; pm_math_Quaternion_compose_ra ( xx + 0 , xx + 4 , xx + 8
) ; xx [ 0 ] = 0.6821674619340059 ; xx [ 1 ] = - 0.6232966132288484 ; xx [ 2
] = 0.2788192726663735 ; xx [ 3 ] = 0.2615505668236233 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 8 , xx + 0 , xx + 4 ) ; error [ 0
] = xx [ 5 ] ; error [ 1 ] = xx [ 6 ] ; return 2 ; } size_t
Control_Bicopter_ae14a523_1_computeAssemblyError ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , const double *
state , const int * modeVector , const double * motionData , double * error )
{ ( void ) mech ; ( void ) rtdv ; ( void ) state ; ( void ) modeVector ; (
void ) motionData ; ( void ) error ; switch ( constraintIdx ) { case 0 :
return computeAssemblyError_0 ( rtdv , state , modeVector , motionData ,
error ) ; case 1 : return computeAssemblyError_1 ( rtdv , state , modeVector
, motionData , error ) ; case 2 : return computeAssemblyError_2 ( rtdv ,
state , modeVector , motionData , error ) ; case 3 : return
computeAssemblyError_3 ( rtdv , state , modeVector , motionData , error ) ;
case 4 : return computeAssemblyError_4 ( rtdv , state , modeVector ,
motionData , error ) ; case 5 : return computeAssemblyError_5 ( rtdv , state
, modeVector , motionData , error ) ; case 6 : return computeAssemblyError_6
( rtdv , state , modeVector , motionData , error ) ; case 7 : return
computeAssemblyError_7 ( rtdv , state , modeVector , motionData , error ) ;
case 8 : return computeAssemblyError_8 ( rtdv , state , modeVector ,
motionData , error ) ; case 9 : return computeAssemblyError_9 ( rtdv , state
, modeVector , motionData , error ) ; case 10 : return
computeAssemblyError_10 ( rtdv , state , modeVector , motionData , error ) ;
case 11 : return computeAssemblyError_11 ( rtdv , state , modeVector ,
motionData , error ) ; } return 0 ; } static size_t computeAssemblyJacobian_0
( const RuntimeDerivedValuesBundle * rtdv , const double * state , const int
* modeVector , const double * motionData , double * J ) { const double *
rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts .
mValues ; double xx [ 33 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void )
modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 7 ] ; xx [ 2 ] =
motionData [ 8 ] ; xx [ 3 ] = motionData [ 9 ] ; xx [ 4 ] = motionData [ 10 ]
; xx [ 5 ] = 0.9912919252698491 ; xx [ 6 ] = - 0.1316824810541687 ; xx [ 7 ]
= 1.970490389246134e-4 ; xx [ 8 ] = 6.519118568758793e-5 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 5 ] =
0.9999980252749909 ; xx [ 6 ] = 1.987321342568947e-3 ; xx [ 13 ] = xx [ 5 ] *
motionData [ 70 ] + xx [ 6 ] * motionData [ 71 ] ; xx [ 14 ] = xx [ 5 ] *
motionData [ 71 ] - xx [ 6 ] * motionData [ 70 ] ; xx [ 15 ] = xx [ 5 ] *
motionData [ 72 ] - xx [ 6 ] * motionData [ 73 ] ; xx [ 16 ] = xx [ 5 ] *
motionData [ 73 ] + xx [ 6 ] * motionData [ 72 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 13 , xx + 5 ) ; xx [ 17
] = 3.734971681880274e-4 ; xx [ 18 ] = 0.2610715860287157 ; xx [ 19 ] =
0.9653193707101895 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 17 , xx + 20
) ; pm_math_Quaternion_inverseXform_ra ( xx + 13 , xx + 20 , xx + 23 ) ; xx [
13 ] = - xx [ 23 ] ; xx [ 14 ] = - xx [ 24 ] ; xx [ 15 ] = - xx [ 25 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 13 , xx + 20 ) ; xx [ 13 ] =
0.0 ; xx [ 14 ] = xx [ 13 ] ; xx [ 15 ] = xx [ 13 ] ; xx [ 16 ] = xx [ 13 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 14 , xx + 24 ) ; xx [ 5 ] = -
0.9120630938981265 ; xx [ 6 ] = - 0.1421693578567881 ; xx [ 7 ] =
0.3844095801838581 ; xx [ 8 ] = - 0.01257223522036471 ; xx [ 13 ] = 0.5 *
state [ 13 ] ; xx [ 14 ] = sin ( xx [ 13 ] ) ; xx [ 27 ] = cos ( xx [ 13 ] )
; xx [ 28 ] = xx [ 17 ] * xx [ 14 ] ; xx [ 29 ] = xx [ 18 ] * xx [ 14 ] ; xx
[ 30 ] = xx [ 19 ] * xx [ 14 ] ; pm_math_Quaternion_compose_ra ( xx + 5 , xx
+ 27 , xx + 13 ) ; xx [ 5 ] = 1.110223024625157e-16 ; xx [ 6 ] = -
1.251169307048272e-16 ; xx [ 7 ] = 3.38000027272356e-17 ;
pm_math_Quaternion_xform_ra ( xx + 13 , xx + 5 , xx + 17 ) ; xx [ 5 ] =
motionData [ 70 ] ; xx [ 6 ] = motionData [ 71 ] ; xx [ 7 ] = motionData [ 72
] ; xx [ 8 ] = motionData [ 73 ] ; xx [ 27 ] = 9.996376071803525e-5 ; xx [ 28
] = 0.01545456410810322 ; xx [ 29 ] = 8.60794096301822e-3 ;
pm_math_Quaternion_xform_ra ( xx + 5 , xx + 27 , xx + 30 ) ; xx [ 5 ] =
2.908691710579205e-6 ; xx [ 6 ] = 2.868076938248717e-3 ; xx [ 7 ] =
4.94643927619723e-3 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 5 , xx + 27
) ; xx [ 1 ] = xx [ 30 ] + motionData [ 74 ] - ( xx [ 27 ] + motionData [ 11
] ) ; xx [ 2 ] = xx [ 31 ] + motionData [ 75 ] - ( xx [ 28 ] + motionData [
12 ] ) ; xx [ 3 ] = xx [ 32 ] + motionData [ 76 ] - ( xx [ 29 ] + motionData
[ 13 ] ) ; xx [ 4 ] = - 1.477235478147115e-3 ; xx [ 5 ] = 9.60335389372561e-7
; xx [ 6 ] = 3.118418564317923e-7 ; pm_math_Quaternion_xform_ra ( xx + 13 ,
xx + 4 , xx + 27 ) ; xx [ 4 ] = - 6.480496605447651e-3 ; xx [ 5 ] =
9.242608862973643e-5 ; xx [ 6 ] = - 2.248932226692751e-5 ;
pm_math_Quaternion_xform_ra ( xx + 13 , xx + 4 , xx + 30 ) ; xx [ 4 ] = xx [
27 ] + xx [ 30 ] ; xx [ 5 ] = xx [ 28 ] + xx [ 31 ] ; xx [ 6 ] = xx [ 29 ] +
xx [ 32 ] ; xx [ 7 ] = 2.0 ; xx [ 13 ] = ( xx [ 9 ] * xx [ 11 ] + xx [ 10 ] *
xx [ 12 ] ) * xx [ 7 ] ; xx [ 14 ] = xx [ 7 ] * ( xx [ 11 ] * xx [ 12 ] - xx
[ 9 ] * xx [ 10 ] ) ; xx [ 15 ] = 1.0 - ( xx [ 10 ] * xx [ 10 ] + xx [ 11 ] *
xx [ 11 ] ) * xx [ 7 ] ; xx [ 7 ] = - 0.7077023583577552 ; xx [ 8 ] = -
9.268340058968039e-3 ; xx [ 9 ] = 0.7064499061132523 ; xx [ 10 ] = -
0.7077023583577557 ; xx [ 11 ] = - 9.268340058967206e-3 ; xx [ 12 ] =
0.7064499061132518 ; J [ 6 ] = xx [ 21 ] ; J [ 7 ] = xx [ 25 ] ; J [ 19 ] =
xx [ 25 ] ; J [ 32 ] = xx [ 22 ] ; J [ 33 ] = xx [ 26 ] ; J [ 45 ] = xx [ 26
] ; J [ 58 ] = pm_math_Vector3_dot_ra ( xx + 17 , xx + 1 ) -
pm_math_Vector3_dot_ra ( xx + 4 , xx + 13 ) ; J [ 59 ] = -
pm_math_Vector3_dot_ra ( xx + 7 , xx + 13 ) ; J [ 71 ] =
pm_math_Vector3_dot_ra ( xx + 10 , xx + 13 ) ; return 3 ; } static size_t
computeAssemblyJacobian_1 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; double xx [ 21 ] ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] =
0.9999999900512558 ; xx [ 2 ] = 1.410584573942991e-4 ; xx [ 3 ] = xx [ 1 ] *
motionData [ 35 ] - xx [ 2 ] * motionData [ 38 ] ; xx [ 4 ] = xx [ 1 ] *
motionData [ 36 ] + xx [ 2 ] * motionData [ 37 ] ; xx [ 5 ] = xx [ 1 ] *
motionData [ 37 ] - xx [ 2 ] * motionData [ 36 ] ; xx [ 6 ] = xx [ 2 ] *
motionData [ 35 ] + xx [ 1 ] * motionData [ 38 ] ; xx [ 7 ] = -
0.9229169573264255 ; xx [ 8 ] = - 0.02080107143120539 ; xx [ 9 ] =
0.382547117836429 ; xx [ 10 ] = 0.03806977727395289 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 3 , xx + 7 , xx + 11 ) ; xx [ 1 ]
= 0.0 ; xx [ 7 ] = xx [ 1 ] ; xx [ 8 ] = xx [ 1 ] ; xx [ 9 ] = xx [ 1 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 11 , xx + 7 , xx + 15 ) ; xx [ 7 ] =
0.7044167893867059 ; xx [ 8 ] = - 0.08618526586337766 ; xx [ 9 ] =
0.7045346597422903 ; xx [ 1 ] = 1.0 ; xx [ 2 ] = xx [ 6 ] * xx [ 6 ] ; xx [
10 ] = 2.0 ; xx [ 11 ] = xx [ 6 ] * xx [ 3 ] ; xx [ 12 ] = xx [ 4 ] * xx [ 5
] ; xx [ 13 ] = xx [ 1 ] - ( xx [ 5 ] * xx [ 5 ] + xx [ 2 ] ) * xx [ 10 ] ;
xx [ 14 ] = ( xx [ 11 ] + xx [ 12 ] ) * xx [ 10 ] ; xx [ 15 ] = xx [ 10 ] * (
xx [ 6 ] * xx [ 4 ] - xx [ 3 ] * xx [ 5 ] ) ; xx [ 18 ] = xx [ 10 ] * ( xx [
12 ] - xx [ 11 ] ) ; xx [ 19 ] = xx [ 1 ] - ( xx [ 2 ] + xx [ 4 ] * xx [ 4 ]
) * xx [ 10 ] ; xx [ 20 ] = ( xx [ 4 ] * xx [ 3 ] + xx [ 6 ] * xx [ 5 ] ) *
xx [ 10 ] ; J [ 12 ] = xx [ 16 ] ; J [ 38 ] = xx [ 17 ] ; J [ 64 ] = -
pm_math_Vector3_dot_ra ( xx + 7 , xx + 13 ) ; J [ 90 ] = -
pm_math_Vector3_dot_ra ( xx + 7 , xx + 18 ) ; return 4 ; } static size_t
computeAssemblyJacobian_2 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; double xx [ 37 ] ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = -
0.9229169573264254 ; xx [ 2 ] = - 0.02080107143120469 ; xx [ 3 ] =
0.3825471178364293 ; xx [ 4 ] = 0.0380697772739531 ; xx [ 5 ] = motionData [
119 ] ; xx [ 6 ] = motionData [ 120 ] ; xx [ 7 ] = motionData [ 121 ] ; xx [
8 ] = motionData [ 122 ] ; xx [ 9 ] = - 0.4813049877214196 ; xx [ 10 ] = -
0.5180593628877815 ; xx [ 11 ] = - 0.5180751960991345 ; xx [ 12 ] = -
0.4812048384063002 ; pm_math_Quaternion_compose_ra ( xx + 5 , xx + 9 , xx +
13 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 1 , xx + 13 , xx + 5 ) ;
xx [ 1 ] = motionData [ 14 ] ; xx [ 2 ] = motionData [ 15 ] ; xx [ 3 ] =
motionData [ 16 ] ; xx [ 4 ] = motionData [ 17 ] ; xx [ 13 ] =
3.734971681880274e-4 ; xx [ 14 ] = 0.2610715860287157 ; xx [ 15 ] =
0.9653193707101895 ; pm_math_Quaternion_inverseXform_ra ( xx + 1 , xx + 13 ,
xx + 16 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 9 , xx + 16 , xx + 19 )
; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 19 , xx + 9 ) ; xx [ 19 ] =
0.0 ; xx [ 20 ] = xx [ 19 ] ; xx [ 21 ] = xx [ 19 ] ; xx [ 22 ] = xx [ 19 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 20 , xx + 23 ) ; xx [ 19 ] =
3.33066907387547e-16 ; xx [ 20 ] = 1.942890293094024e-15 ; xx [ 21 ] = -
1.000000000000001 ; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 19 , xx +
27 ) ; xx [ 5 ] = - 0.9120630938981265 ; xx [ 6 ] = - 0.1421693578567881 ; xx
[ 7 ] = 0.3844095801838581 ; xx [ 8 ] = - 0.01257223522036471 ; xx [ 9 ] =
0.5 ; xx [ 12 ] = xx [ 9 ] * state [ 13 ] ; xx [ 19 ] = sin ( xx [ 12 ] ) ;
xx [ 20 ] = cos ( xx [ 12 ] ) ; xx [ 21 ] = xx [ 13 ] * xx [ 19 ] ; xx [ 22 ]
= xx [ 14 ] * xx [ 19 ] ; xx [ 23 ] = xx [ 15 ] * xx [ 19 ] ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 20 , xx + 30 ) ;
pm_math_Quaternion_compose_ra ( xx + 30 , xx + 1 , xx + 5 ) ; xx [ 1 ] =
0.013582786684373 ; xx [ 2 ] = - 2.58589188160877e-6 ; xx [ 3 ] = -
0.01849903189296259 ; pm_math_Vector3_cross_ra ( xx + 16 , xx + 1 , xx + 19 )
; pm_math_Quaternion_xform_ra ( xx + 5 , xx + 19 , xx + 1 ) ; xx [ 4 ] =
motionData [ 18 ] ; xx [ 5 ] = motionData [ 19 ] ; xx [ 6 ] = motionData [ 20
] ; pm_math_Vector3_cross_ra ( xx + 13 , xx + 4 , xx + 16 ) ;
pm_math_Quaternion_xform_ra ( xx + 30 , xx + 16 , xx + 4 ) ; xx [ 12 ] = -
6.480496605447651e-3 ; xx [ 13 ] = 9.242608862973643e-5 ; xx [ 14 ] = -
2.248932226692751e-5 ; pm_math_Quaternion_xform_ra ( xx + 30 , xx + 12 , xx +
15 ) ; xx [ 12 ] = xx [ 1 ] + xx [ 4 ] + xx [ 15 ] ; xx [ 13 ] = xx [ 2 ] +
xx [ 5 ] + xx [ 16 ] ; xx [ 14 ] = xx [ 3 ] + xx [ 6 ] + xx [ 17 ] ; xx [ 1 ]
= - 0.7077022283680728 ; xx [ 2 ] = - 9.268355963183032e-3 ; xx [ 3 ] =
0.7064500361247099 ; xx [ 4 ] = motionData [ 7 ] ; xx [ 5 ] = motionData [ 8
] ; xx [ 6 ] = motionData [ 9 ] ; xx [ 7 ] = motionData [ 10 ] ; xx [ 15 ] =
0.4090279037421741 ; xx [ 16 ] = - 0.57698859548371 ; xx [ 17 ] = -
0.5768688874880377 ; xx [ 18 ] = - 0.4086595420277526 ; xx [ 8 ] = xx [ 9 ] *
state [ 17 ] ; xx [ 9 ] = sin ( xx [ 8 ] ) ; xx [ 19 ] = cos ( xx [ 8 ] ) ;
xx [ 20 ] = - ( 0.9972896958011036 * xx [ 9 ] ) ; xx [ 21 ] =
8.852854494773021e-5 * xx [ 9 ] ; xx [ 22 ] = 0.07357482457770348 * xx [ 9 ]
; pm_math_Quaternion_compose_ra ( xx + 15 , xx + 19 , xx + 30 ) ;
pm_math_Quaternion_compose_ra ( xx + 4 , xx + 30 , xx + 15 ) ; xx [ 19 ] = -
1.447435834859361e-6 ; xx [ 20 ] = - 0.01744954274156846 ; xx [ 21 ] =
1.376418986481207e-6 ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 19 , xx +
34 ) ; xx [ 15 ] = 1.439701931685288e-6 ; xx [ 16 ] = 0.01494182793334928 ;
xx [ 17 ] = 1.536145228271068e-6 ; pm_math_Quaternion_xform_ra ( xx + 30 , xx
+ 15 , xx + 18 ) ; pm_math_Quaternion_xform_ra ( xx + 4 , xx + 18 , xx + 15 )
; xx [ 4 ] = xx [ 34 ] + xx [ 15 ] ; xx [ 5 ] = xx [ 35 ] + xx [ 16 ] ; xx [
6 ] = xx [ 36 ] + xx [ 17 ] ; J [ 6 ] = xx [ 10 ] ; J [ 7 ] = xx [ 24 ] ; J [
8 ] = xx [ 28 ] ; J [ 32 ] = xx [ 11 ] ; J [ 33 ] = xx [ 25 ] ; J [ 34 ] = xx
[ 29 ] ; J [ 58 ] = pm_math_Vector3_dot_ra ( xx + 12 , xx + 1 ) ; J [ 59 ] =
0.9999999999999832 ; J [ 60 ] = pm_math_Vector3_dot_ra ( xx + 4 , xx + 1 ) ;
return 3 ; } static size_t computeAssemblyJacobian_3 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 17 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void
) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 84 ] ; xx [ 2 ] =
motionData [ 85 ] ; xx [ 3 ] = motionData [ 86 ] ; xx [ 4 ] = motionData [ 87
] ; xx [ 5 ] = 0.4982139734058924 ; xx [ 6 ] = - 0.4963735918252776 ; xx [ 7
] = - 0.501668831212296 ; xx [ 8 ] = - 0.5037107084742934 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 1 ] = -
0.6795114020738748 ; xx [ 2 ] = 0.2557971615198039 ; xx [ 3 ] =
0.2852066954013848 ; xx [ 4 ] = - 0.6256909840394302 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 1 , xx + 5 ) ; xx [ 1 ]
= 0.0 ; xx [ 2 ] = xx [ 1 ] ; xx [ 3 ] = xx [ 1 ] ; xx [ 4 ] = xx [ 1 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 2 , xx + 13 ) ; xx [ 1 ] = -
0.7077023583577412 ; xx [ 2 ] = - 9.268340058974089e-3 ; xx [ 3 ] =
0.7064499061132667 ; xx [ 4 ] = 2.0 ; xx [ 5 ] = ( xx [ 9 ] * xx [ 11 ] + xx
[ 10 ] * xx [ 12 ] ) * xx [ 4 ] ; xx [ 6 ] = xx [ 4 ] * ( xx [ 11 ] * xx [ 12
] - xx [ 9 ] * xx [ 10 ] ) ; xx [ 7 ] = 1.0 - ( xx [ 10 ] * xx [ 10 ] + xx [
11 ] * xx [ 11 ] ) * xx [ 4 ] ; J [ 21 ] = xx [ 14 ] ; J [ 47 ] = xx [ 15 ] ;
J [ 73 ] = - pm_math_Vector3_dot_ra ( xx + 1 , xx + 5 ) ; return 3 ; } static
size_t computeAssemblyJacobian_4 ( const RuntimeDerivedValuesBundle * rtdv ,
const double * state , const int * modeVector , const double * motionData ,
double * J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; double xx [ 44 ] ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [
49 ] ; xx [ 2 ] = motionData [ 50 ] ; xx [ 3 ] = motionData [ 51 ] ; xx [ 4 ]
= motionData [ 52 ] ; xx [ 5 ] = 0.5180593628885556 ; xx [ 6 ] = -
0.4813049877207372 ; xx [ 7 ] = - 0.4812048384058716 ; xx [ 8 ] =
0.518075196099393 ; pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9
) ; xx [ 5 ] = motionData [ 42 ] ; xx [ 6 ] = motionData [ 43 ] ; xx [ 7 ] =
motionData [ 44 ] ; xx [ 8 ] = motionData [ 45 ] ; xx [ 13 ] =
0.1316824810541773 ; xx [ 14 ] = 0.991291925269848 ; xx [ 15 ] =
6.51911856886885e-5 ; xx [ 16 ] = - 1.970490389163408e-4 ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 13 , xx + 17 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 17 , xx + 13 ) ; xx [ 21
] = 1.729508463527152e-14 ; xx [ 22 ] = 1.587618925213974e-14 ; xx [ 23 ] = -
1.0 ; pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 21 , xx + 24 ) ; xx [
21 ] = 0.0 ; xx [ 28 ] = xx [ 21 ] ; xx [ 29 ] = xx [ 21 ] ; xx [ 30 ] = xx [
21 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 28 , xx + 31 ) ; xx [
21 ] = 0.9972896958009458 ; xx [ 22 ] = 8.852854518109909e-5 ; xx [ 23 ] =
0.0735748245798411 ; xx [ 28 ] = - xx [ 21 ] ; xx [ 29 ] = xx [ 22 ] ; xx [
30 ] = xx [ 23 ] ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 28 , xx + 35 )
; pm_math_Quaternion_inverseXform_ra ( xx + 17 , xx + 35 , xx + 28 ) ; xx [
17 ] = - xx [ 28 ] ; xx [ 18 ] = - xx [ 29 ] ; xx [ 19 ] = - xx [ 30 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 35 ) ; xx [ 13 ] =
- 0.1316824519486606 ; xx [ 14 ] = 0.9912919062119712 ; xx [ 15 ] =
2.050212948449675e-4 ; xx [ 16 ] = - 2.156239646078782e-4 ; xx [ 17 ] = 0.5 ;
xx [ 18 ] = xx [ 17 ] * state [ 27 ] ; xx [ 19 ] = sin ( xx [ 18 ] ) ; xx [
27 ] = cos ( xx [ 18 ] ) ; xx [ 28 ] = 3.734971681885031e-4 * xx [ 19 ] ; xx
[ 29 ] = 0.261071586028717 * xx [ 19 ] ; xx [ 30 ] = 0.965319370710189 * xx [
19 ] ; pm_math_Quaternion_compose_ra ( xx + 13 , xx + 27 , xx + 38 ) ; xx [
13 ] = 6.459047385173115e-3 ; xx [ 14 ] = - 9.24648174485767e-5 ; xx [ 15 ] =
2.250809555350908e-5 ; pm_math_Quaternion_xform_ra ( xx + 38 , xx + 13 , xx +
18 ) ; xx [ 13 ] = - 6.480496605446796e-3 ; xx [ 14 ] = 9.242608862956347e-5
; xx [ 15 ] = - 2.2489322266878e-5 ; pm_math_Quaternion_xform_ra ( xx + 38 ,
xx + 13 , xx + 27 ) ; xx [ 13 ] = xx [ 18 ] + xx [ 27 ] ; xx [ 14 ] = xx [ 19
] + xx [ 28 ] ; xx [ 15 ] = xx [ 20 ] + xx [ 29 ] ; xx [ 16 ] = 2.0 ; xx [ 18
] = 1.0 ; xx [ 19 ] = ( xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) * xx
[ 16 ] ; xx [ 27 ] = ( xx [ 9 ] * xx [ 11 ] + xx [ 10 ] * xx [ 12 ] ) * xx [
16 ] ; xx [ 28 ] = xx [ 16 ] * ( xx [ 11 ] * xx [ 12 ] - xx [ 9 ] * xx [ 10 ]
) ; xx [ 29 ] = xx [ 18 ] - xx [ 19 ] ; xx [ 9 ] = - 0.5181324366224915 ; xx
[ 10 ] = - 0.4812371049201607 ; xx [ 11 ] = - 0.4812727257575918 ; xx [ 12 ]
= 0.5180021142906276 ; xx [ 16 ] = xx [ 17 ] * state [ 31 ] ; xx [ 17 ] = sin
( xx [ 16 ] ) ; xx [ 38 ] = cos ( xx [ 16 ] ) ; xx [ 39 ] = - ( xx [ 21 ] *
xx [ 17 ] ) ; xx [ 40 ] = xx [ 22 ] * xx [ 17 ] ; xx [ 41 ] = xx [ 23 ] * xx
[ 17 ] ; pm_math_Quaternion_compose_ra ( xx + 9 , xx + 38 , xx + 20 ) ; xx [
9 ] = 4.103874629447085e-18 ; xx [ 10 ] = 2.081668171172169e-16 ; xx [ 11 ] =
5.538917848685321e-17 ; pm_math_Quaternion_xform_ra ( xx + 20 , xx + 9 , xx +
38 ) ; xx [ 9 ] = - 9.045674901471507e-5 ; xx [ 10 ] = - 2.965461248705281e-3
; xx [ 11 ] = 0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 5 , xx
+ 9 , xx + 41 ) ; xx [ 5 ] = - 0.01609995664951487 ; xx [ 6 ] =
2.933657308331041e-6 ; xx [ 7 ] = - 0.0137731566582556 ;
pm_math_Quaternion_xform_ra ( xx + 1 , xx + 5 , xx + 8 ) ; xx [ 1 ] = xx [ 41
] + motionData [ 46 ] - ( xx [ 8 ] + motionData [ 53 ] ) ; xx [ 2 ] = xx [ 42
] + motionData [ 47 ] - ( xx [ 9 ] + motionData [ 54 ] ) ; xx [ 3 ] = xx [ 43
] + motionData [ 48 ] - ( xx [ 10 ] + motionData [ 55 ] ) ; xx [ 4 ] = -
1.435160843344562e-6 ; xx [ 5 ] = - 0.0149203787001616 ; xx [ 6 ] = -
1.500400464949372e-6 ; pm_math_Quaternion_xform_ra ( xx + 20 , xx + 4 , xx +
7 ) ; xx [ 4 ] = 1.439701935798469e-6 ; xx [ 5 ] = 0.01494182793334727 ; xx [
6 ] = 1.53614523658561e-6 ; pm_math_Quaternion_xform_ra ( xx + 20 , xx + 4 ,
xx + 10 ) ; xx [ 4 ] = xx [ 7 ] + xx [ 10 ] ; xx [ 5 ] = xx [ 8 ] + xx [ 11 ]
; xx [ 6 ] = xx [ 9 ] + xx [ 12 ] ; J [ 13 ] = xx [ 25 ] ; J [ 14 ] = xx [ 32
] ; J [ 15 ] = xx [ 36 ] ; J [ 39 ] = xx [ 26 ] ; J [ 40 ] = xx [ 33 ] ; J [
41 ] = xx [ 37 ] ; J [ 65 ] = pm_math_Vector3_dot_ra ( xx + 13 , xx + 27 ) ;
J [ 66 ] = xx [ 19 ] - xx [ 18 ] ; J [ 67 ] = pm_math_Vector3_dot_ra ( xx +
38 , xx + 1 ) - pm_math_Vector3_dot_ra ( xx + 4 , xx + 27 ) ; return 3 ; }
static size_t computeAssemblyJacobian_5 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * J ) { const double * rtdvd = rtdv -> mDoubles . mValues
; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 45 ] ; ( void )
rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] =
motionData [ 126 ] ; xx [ 2 ] = motionData [ 127 ] ; xx [ 3 ] = motionData [
128 ] ; xx [ 4 ] = motionData [ 129 ] ; xx [ 5 ] = 0.991291925269849 ; xx [ 6
] = - 0.1316824810541695 ; xx [ 7 ] = 1.970490389249778e-4 ; xx [ 8 ] =
6.519118568753968e-5 ; pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx +
9 ) ; xx [ 5 ] = motionData [ 77 ] ; xx [ 6 ] = motionData [ 78 ] ; xx [ 7 ]
= motionData [ 79 ] ; xx [ 8 ] = motionData [ 80 ] ; xx [ 13 ] =
0.9999980252749867 ; xx [ 14 ] = - 1.987321342568328e-3 ; xx [ 15 ] =
9.226742840440916e-8 ; xx [ 16 ] = - 1.833653884871776e-10 ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 13 , xx + 17 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 17 , xx + 13 ) ; xx [ 21
] = 0.0 ; xx [ 22 ] = xx [ 21 ] ; xx [ 23 ] = xx [ 21 ] ; xx [ 24 ] = xx [ 21
] ; pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 22 , xx + 25 ) ; xx [ 21
] = 3.734971681885031e-4 ; xx [ 22 ] = 0.261071586028717 ; xx [ 23 ] =
0.965319370710189 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 21 , xx + 29
) ; pm_math_Quaternion_inverseXform_ra ( xx + 17 , xx + 29 , xx + 32 ) ; xx [
17 ] = - xx [ 32 ] ; xx [ 18 ] = - xx [ 33 ] ; xx [ 19 ] = - xx [ 34 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 29 ) ; xx [ 13 ] =
0.7044167893867059 ; xx [ 14 ] = - 0.08618526586337766 ; xx [ 15 ] =
0.7045346597422903 ; xx [ 16 ] = 2.0 ; xx [ 17 ] = 1.0 ; xx [ 18 ] = ( xx [ 9
] * xx [ 11 ] + xx [ 10 ] * xx [ 12 ] ) * xx [ 16 ] ; xx [ 19 ] = xx [ 16 ] *
( xx [ 11 ] * xx [ 12 ] - xx [ 9 ] * xx [ 10 ] ) ; xx [ 20 ] = xx [ 17 ] - (
xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) * xx [ 16 ] ; xx [ 9 ] =
motionData [ 35 ] ; xx [ 10 ] = motionData [ 36 ] ; xx [ 11 ] = motionData [
37 ] ; xx [ 12 ] = motionData [ 38 ] ; xx [ 32 ] = - 0.1316824519486606 ; xx
[ 33 ] = 0.9912919062119712 ; xx [ 34 ] = 2.050212948449675e-4 ; xx [ 35 ] =
- 2.156239646078782e-4 ; xx [ 24 ] = 0.5 * state [ 27 ] ; xx [ 25 ] = sin (
xx [ 24 ] ) ; xx [ 36 ] = cos ( xx [ 24 ] ) ; xx [ 37 ] = xx [ 21 ] * xx [ 25
] ; xx [ 38 ] = xx [ 22 ] * xx [ 25 ] ; xx [ 39 ] = xx [ 23 ] * xx [ 25 ] ;
pm_math_Quaternion_compose_ra ( xx + 32 , xx + 36 , xx + 21 ) ;
pm_math_Quaternion_compose_ra ( xx + 9 , xx + 21 , xx + 32 ) ; xx [ 36 ] = -
2.775557561562891e-16 ; xx [ 37 ] = 1.251169307048272e-16 ; xx [ 38 ] = -
3.373224009145526e-17 ; pm_math_Quaternion_xform_ra ( xx + 32 , xx + 36 , xx
+ 39 ) ; xx [ 36 ] = 9.477449468724687e-5 ; xx [ 37 ] = 0.01545456410835202 ;
xx [ 38 ] = 8.607941025578736e-3 ; pm_math_Quaternion_xform_ra ( xx + 5 , xx
+ 36 , xx + 42 ) ; xx [ 5 ] = 2.908691710755093e-6 ; xx [ 6 ] =
2.868076938250087e-3 ; xx [ 7 ] = 4.94643927619916e-3 ;
pm_math_Quaternion_xform_ra ( xx + 1 , xx + 5 , xx + 36 ) ; xx [ 1 ] = xx [
42 ] + motionData [ 81 ] - ( xx [ 36 ] + motionData [ 130 ] ) ; xx [ 2 ] = xx
[ 43 ] + motionData [ 82 ] - ( xx [ 37 ] + motionData [ 131 ] ) ; xx [ 3 ] =
xx [ 44 ] + motionData [ 83 ] - ( xx [ 38 ] + motionData [ 132 ] ) ; xx [ 4 ]
= - 1.477235478147926e-3 ; xx [ 5 ] = 9.60335389539274e-7 ; xx [ 6 ] =
3.118418563877457e-7 ; pm_math_Quaternion_xform_ra ( xx + 32 , xx + 4 , xx +
36 ) ; xx [ 4 ] = - 6.480496605446796e-3 ; xx [ 5 ] = 9.242608862956347e-5 ;
xx [ 6 ] = - 2.2489322266878e-5 ; pm_math_Quaternion_xform_ra ( xx + 21 , xx
+ 4 , xx + 32 ) ; pm_math_Quaternion_xform_ra ( xx + 9 , xx + 32 , xx + 4 ) ;
xx [ 7 ] = xx [ 36 ] + xx [ 4 ] ; xx [ 8 ] = xx [ 37 ] + xx [ 5 ] ; xx [ 9 ]
= xx [ 38 ] + xx [ 6 ] ; xx [ 4 ] = - ( ( motionData [ 35 ] * motionData [ 37
] + motionData [ 36 ] * motionData [ 38 ] ) * xx [ 16 ] ) ; xx [ 5 ] = xx [
16 ] * ( motionData [ 35 ] * motionData [ 36 ] - motionData [ 37 ] *
motionData [ 38 ] ) ; xx [ 6 ] = ( motionData [ 36 ] * motionData [ 36 ] +
motionData [ 37 ] * motionData [ 37 ] ) * xx [ 16 ] - xx [ 17 ] ; xx [ 10 ] =
- 0.7077023583577554 ; xx [ 11 ] = - 9.268340058966987e-3 ; xx [ 12 ] =
0.7064499061132521 ; J [ 12 ] = xx [ 26 ] ; J [ 13 ] = xx [ 30 ] ; J [ 14 ] =
xx [ 26 ] ; J [ 20 ] = xx [ 26 ] ; J [ 38 ] = xx [ 27 ] ; J [ 39 ] = xx [ 31
] ; J [ 40 ] = xx [ 27 ] ; J [ 46 ] = xx [ 27 ] ; J [ 64 ] = -
pm_math_Vector3_dot_ra ( xx + 13 , xx + 18 ) ; J [ 65 ] =
pm_math_Vector3_dot_ra ( xx + 39 , xx + 1 ) - pm_math_Vector3_dot_ra ( xx + 7
, xx + 18 ) ; J [ 66 ] = - pm_math_Vector3_dot_ra ( xx + 4 , xx + 18 ) ; J [
72 ] = pm_math_Vector3_dot_ra ( xx + 10 , xx + 18 ) ; return 3 ; } static
size_t computeAssemblyJacobian_6 ( const RuntimeDerivedValuesBundle * rtdv ,
const double * state , const int * modeVector , const double * motionData ,
double * J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; double xx [ 43 ] ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [
21 ] ; xx [ 2 ] = motionData [ 22 ] ; xx [ 3 ] = motionData [ 23 ] ; xx [ 4 ]
= motionData [ 24 ] ; xx [ 5 ] = 0.4931954203595264 ; xx [ 6 ] = -
0.4905194435002385 ; xx [ 7 ] = - 0.5067824161638587 ; xx [ 8 ] = -
0.509235245786982 ; pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9
) ; xx [ 5 ] = motionData [ 28 ] ; xx [ 6 ] = motionData [ 29 ] ; xx [ 7 ] =
motionData [ 30 ] ; xx [ 8 ] = motionData [ 31 ] ; xx [ 13 ] =
0.6370506001153763 ; xx [ 14 ] = - 0.6370493477200306 ; xx [ 15 ] = -
0.3068667118838679 ; xx [ 16 ] = - 0.3068672067844752 ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 13 , xx + 17 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 17 , xx + 13 ) ; xx [ 21
] = 3.05954793225871e-4 ; xx [ 22 ] = 0.9999868227746886 ; xx [ 23 ] =
5.124516430675685e-3 ; xx [ 24 ] = xx [ 21 ] ; xx [ 25 ] = - xx [ 22 ] ; xx [
26 ] = - xx [ 23 ] ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 24 , xx + 27
) ; pm_math_Quaternion_inverseXform_ra ( xx + 17 , xx + 27 , xx + 24 ) ; xx [
17 ] = - xx [ 24 ] ; xx [ 18 ] = - xx [ 25 ] ; xx [ 19 ] = - xx [ 26 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 24 ) ; xx [ 17 ] =
0.0 ; xx [ 18 ] = xx [ 17 ] ; xx [ 19 ] = xx [ 17 ] ; xx [ 20 ] = xx [ 17 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 18 , xx + 28 ) ; xx [ 17 ] =
1.665334536937735e-16 ; xx [ 18 ] = 5.551115123125783e-16 ; xx [ 19 ] = 1.0 ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 32 ) ; xx [ 13 ] =
0.4716196941540828 ; xx [ 14 ] = - 0.5117381057633613 ; xx [ 15 ] =
0.5267317275910312 ; xx [ 16 ] = - 0.4881113216723255 ; xx [ 17 ] = 0.5 ; xx
[ 18 ] = xx [ 17 ] * state [ 19 ] ; xx [ 19 ] = sin ( xx [ 18 ] ) ; xx [ 35 ]
= cos ( xx [ 18 ] ) ; xx [ 36 ] = xx [ 21 ] * xx [ 19 ] ; xx [ 37 ] = - ( xx
[ 22 ] * xx [ 19 ] ) ; xx [ 38 ] = - ( xx [ 23 ] * xx [ 19 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 13 , xx + 35 , xx + 18 ) ; xx [ 13 ] = -
2.237793284010081e-16 ; xx [ 14 ] = 1.923188306740889e-18 ; xx [ 15 ] = -
3.886322687274291e-16 ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 13 , xx
+ 22 ) ; xx [ 13 ] = - 3.865146760306702e-8 ; xx [ 14 ] = -
5.289307885790379e-3 ; xx [ 15 ] = - 3.071851003736677e-7 ;
pm_math_Quaternion_xform_ra ( xx + 5 , xx + 13 , xx + 35 ) ; xx [ 5 ] =
2.392300359455244e-7 ; xx [ 6 ] = 3.887917980054749e-3 ; xx [ 7 ] =
2.025360777142851e-5 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 5 , xx +
13 ) ; xx [ 1 ] = xx [ 35 ] + motionData [ 32 ] - ( xx [ 13 ] + motionData [
25 ] ) ; xx [ 2 ] = xx [ 36 ] + motionData [ 33 ] - ( xx [ 14 ] + motionData
[ 26 ] ) ; xx [ 3 ] = xx [ 37 ] + motionData [ 34 ] - ( xx [ 15 ] +
motionData [ 27 ] ) ; xx [ 4 ] = - 3.296413151655537e-7 ; xx [ 5 ] = -
7.422626627699278e-9 ; xx [ 6 ] = 1.428754025224236e-6 ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 4 , xx + 13 ) ; xx [ 4 ] =
3.296413151917778e-7 ; xx [ 5 ] = 7.422626626299722e-9 ; xx [ 6 ] = -
1.428754024949564e-6 ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 4 , xx +
35 ) ; xx [ 4 ] = xx [ 13 ] + xx [ 35 ] ; xx [ 5 ] = xx [ 14 ] + xx [ 36 ] ;
xx [ 6 ] = xx [ 15 ] + xx [ 37 ] ; xx [ 7 ] = 2.0 ; xx [ 13 ] = ( xx [ 9 ] *
xx [ 11 ] + xx [ 10 ] * xx [ 12 ] ) * xx [ 7 ] ; xx [ 14 ] = xx [ 7 ] * ( xx
[ 11 ] * xx [ 12 ] - xx [ 9 ] * xx [ 10 ] ) ; xx [ 15 ] = 1.0 - ( xx [ 10 ] *
xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) * xx [ 7 ] ; xx [ 7 ] =
0.07357483435689677 ; xx [ 8 ] = 1.128000373271654e-4 ; xx [ 9 ] =
0.9972896926297332 ; xx [ 18 ] = 0.6254151447093548 ; xx [ 19 ] = -
0.6480628292580688 ; xx [ 20 ] = 0.3298467071707108 ; xx [ 21 ] = -
0.2829692843240122 ; xx [ 10 ] = xx [ 17 ] * state [ 23 ] ; xx [ 11 ] = sin (
xx [ 10 ] ) ; xx [ 35 ] = cos ( xx [ 10 ] ) ; xx [ 36 ] = - (
1.380846645360201e-7 * xx [ 11 ] ) ; xx [ 37 ] = 0.9999999999981867 * xx [ 11
] ; xx [ 38 ] = 1.899414133310628e-6 * xx [ 11 ] ;
pm_math_Quaternion_compose_ra ( xx + 18 , xx + 35 , xx + 39 ) ; xx [ 10 ] = -
2.971385142194091e-7 ; xx [ 11 ] = - 1.158326953740249e-13 ; xx [ 12 ] =
3.938183990803402e-8 ; pm_math_Quaternion_xform_ra ( xx + 39 , xx + 10 , xx +
16 ) ; xx [ 10 ] = 2.971385142511557e-7 ; xx [ 11 ] = 1.158326953518261e-13 ;
xx [ 12 ] = - 3.938183989403888e-8 ; pm_math_Quaternion_xform_ra ( xx + 39 ,
xx + 10 , xx + 19 ) ; xx [ 10 ] = xx [ 16 ] + xx [ 19 ] ; xx [ 11 ] = xx [ 17
] + xx [ 20 ] ; xx [ 12 ] = xx [ 18 ] + xx [ 21 ] ; J [ 9 ] = xx [ 25 ] ; J [
10 ] = xx [ 29 ] ; J [ 11 ] = xx [ 33 ] ; J [ 35 ] = xx [ 26 ] ; J [ 36 ] =
xx [ 30 ] ; J [ 37 ] = xx [ 34 ] ; J [ 61 ] = pm_math_Vector3_dot_ra ( xx +
22 , xx + 1 ) - pm_math_Vector3_dot_ra ( xx + 4 , xx + 13 ) ; J [ 62 ] = -
pm_math_Vector3_dot_ra ( xx + 7 , xx + 13 ) ; J [ 63 ] =
pm_math_Vector3_dot_ra ( xx + 10 , xx + 13 ) ; return 3 ; } static size_t
computeAssemblyJacobian_7 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; double xx [ 43 ] ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 56 ] ;
xx [ 2 ] = motionData [ 57 ] ; xx [ 3 ] = motionData [ 58 ] ; xx [ 4 ] =
motionData [ 59 ] ; xx [ 5 ] = 0.493195420359526 ; xx [ 6 ] = -
0.4905194435002388 ; xx [ 7 ] = - 0.5067824161638583 ; xx [ 8 ] = -
0.5092352457869823 ; pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9
) ; xx [ 5 ] = motionData [ 63 ] ; xx [ 6 ] = motionData [ 64 ] ; xx [ 7 ] =
motionData [ 65 ] ; xx [ 8 ] = motionData [ 66 ] ; xx [ 13 ] =
0.6370493477200303 ; xx [ 14 ] = 0.6370506001153765 ; xx [ 15 ] = -
0.3068672067844756 ; xx [ 16 ] = 0.3068667118838675 ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 13 , xx + 17 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 17 , xx + 13 ) ; xx [ 21
] = 3.059547932236506e-4 ; xx [ 22 ] = 0.9999868227746889 ; xx [ 23 ] =
5.124516430674353e-3 ; xx [ 24 ] = xx [ 21 ] ; xx [ 25 ] = - xx [ 22 ] ; xx [
26 ] = - xx [ 23 ] ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 24 , xx + 27
) ; pm_math_Quaternion_inverseXform_ra ( xx + 17 , xx + 27 , xx + 24 ) ; xx [
17 ] = - xx [ 24 ] ; xx [ 18 ] = - xx [ 25 ] ; xx [ 19 ] = - xx [ 26 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 24 ) ; xx [ 17 ] =
0.0 ; xx [ 18 ] = xx [ 17 ] ; xx [ 19 ] = xx [ 17 ] ; xx [ 20 ] = xx [ 17 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 18 , xx + 28 ) ; xx [ 17 ] =
- 2.55351295663786e-15 ; xx [ 18 ] = - 2.109423746787797e-15 ; xx [ 19 ] = -
1.0 ; pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 17 , xx + 32 ) ; xx [
13 ] = 0.4716196941536389 ; xx [ 14 ] = - 0.5117381057636369 ; xx [ 15 ] =
0.5267317275917935 ; xx [ 16 ] = - 0.4881113216716428 ; xx [ 17 ] = 0.5 ; xx
[ 18 ] = xx [ 17 ] * state [ 33 ] ; xx [ 19 ] = sin ( xx [ 18 ] ) ; xx [ 35 ]
= cos ( xx [ 18 ] ) ; xx [ 36 ] = xx [ 21 ] * xx [ 19 ] ; xx [ 37 ] = - ( xx
[ 22 ] * xx [ 19 ] ) ; xx [ 38 ] = - ( xx [ 23 ] * xx [ 19 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 13 , xx + 35 , xx + 18 ) ; xx [ 13 ] = -
1.334869714764153e-15 ; xx [ 14 ] = 5.850668324922266e-18 ; xx [ 15 ] = -
1.221407957413545e-15 ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 13 , xx
+ 22 ) ; xx [ 13 ] = - 3.975614476060287e-8 ; xx [ 14 ] =
2.710692114195085e-3 ; xx [ 15 ] = - 2.919897872271535e-7 ;
pm_math_Quaternion_xform_ra ( xx + 5 , xx + 13 , xx + 35 ) ; xx [ 5 ] =
2.392300359480238e-7 ; xx [ 6 ] = 3.887917980054739e-3 ; xx [ 7 ] =
2.025360777142854e-5 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 5 , xx +
13 ) ; xx [ 1 ] = xx [ 35 ] + motionData [ 67 ] - ( xx [ 13 ] + motionData [
60 ] ) ; xx [ 2 ] = xx [ 36 ] + motionData [ 68 ] - ( xx [ 14 ] + motionData
[ 61 ] ) ; xx [ 3 ] = xx [ 37 ] + motionData [ 69 ] - ( xx [ 15 ] +
motionData [ 62 ] ) ; xx [ 4 ] = - 3.296413151708188e-7 ; xx [ 5 ] = -
7.422626627666806e-9 ; xx [ 6 ] = 1.4287540252181e-6 ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 4 , xx + 13 ) ; xx [ 4 ] =
3.296413155003147e-7 ; xx [ 5 ] = 7.422626627203207e-9 ; xx [ 6 ] = -
1.428754025107962e-6 ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 4 , xx +
35 ) ; xx [ 4 ] = xx [ 13 ] + xx [ 35 ] ; xx [ 5 ] = xx [ 14 ] + xx [ 36 ] ;
xx [ 6 ] = xx [ 15 ] + xx [ 37 ] ; xx [ 7 ] = 2.0 ; xx [ 13 ] = ( xx [ 9 ] *
xx [ 11 ] + xx [ 10 ] * xx [ 12 ] ) * xx [ 7 ] ; xx [ 14 ] = xx [ 7 ] * ( xx
[ 11 ] * xx [ 12 ] - xx [ 9 ] * xx [ 10 ] ) ; xx [ 15 ] = 1.0 - ( xx [ 10 ] *
xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) * xx [ 7 ] ; xx [ 7 ] =
0.07357483435904683 ; xx [ 8 ] = 1.128000365619997e-4 ; xx [ 9 ] =
0.9972896926295751 ; xx [ 18 ] = 0.6252280994013473 ; xx [ 19 ] =
0.6477664378100223 ; xx [ 20 ] = 0.330373832435706 ; xx [ 21 ] =
0.2834459324238707 ; xx [ 10 ] = xx [ 17 ] * state [ 37 ] ; xx [ 11 ] = sin (
xx [ 10 ] ) ; xx [ 35 ] = cos ( xx [ 10 ] ) ; xx [ 36 ] = - (
1.380846645360201e-7 * xx [ 11 ] ) ; xx [ 37 ] = 0.9999999999981867 * xx [ 11
] ; xx [ 38 ] = 1.899414131645294e-6 * xx [ 11 ] ;
pm_math_Quaternion_compose_ra ( xx + 18 , xx + 35 , xx + 39 ) ; xx [ 10 ] = -
2.971385141348656e-7 ; xx [ 11 ] = - 1.158326949952305e-13 ; xx [ 12 ] =
3.938183974928171e-8 ; pm_math_Quaternion_xform_ra ( xx + 39 , xx + 10 , xx +
16 ) ; xx [ 10 ] = 2.971385144603623e-7 ; xx [ 11 ] = 1.158326948378051e-13 ;
xx [ 12 ] = - 3.938183964273754e-8 ; pm_math_Quaternion_xform_ra ( xx + 39 ,
xx + 10 , xx + 19 ) ; xx [ 10 ] = xx [ 16 ] + xx [ 19 ] ; xx [ 11 ] = xx [ 17
] + xx [ 20 ] ; xx [ 12 ] = xx [ 18 ] + xx [ 21 ] ; J [ 16 ] = xx [ 25 ] ; J
[ 17 ] = xx [ 29 ] ; J [ 18 ] = xx [ 33 ] ; J [ 42 ] = xx [ 26 ] ; J [ 43 ] =
xx [ 30 ] ; J [ 44 ] = xx [ 34 ] ; J [ 68 ] = pm_math_Vector3_dot_ra ( xx +
22 , xx + 1 ) - pm_math_Vector3_dot_ra ( xx + 4 , xx + 13 ) ; J [ 69 ] = -
pm_math_Vector3_dot_ra ( xx + 7 , xx + 13 ) ; J [ 70 ] =
pm_math_Vector3_dot_ra ( xx + 10 , xx + 13 ) ; return 3 ; } static size_t
computeAssemblyJacobian_8 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; double xx [ 27 ] ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] =
0.02080107143120473 ; xx [ 2 ] = - 0.9229169573264255 ; xx [ 3 ] =
0.0380697772739531 ; xx [ 4 ] = - 0.3825471178364295 ; xx [ 5 ] = motionData
[ 133 ] ; xx [ 6 ] = motionData [ 134 ] ; xx [ 7 ] = motionData [ 135 ] ; xx
[ 8 ] = motionData [ 136 ] ; xx [ 9 ] = 0.8739852835857713 ; xx [ 10 ] =
2.165051543389225e-3 ; xx [ 11 ] = 0.4859456095725561 ; xx [ 12 ] =
1.378827208253894e-3 ; pm_math_Quaternion_compose_ra ( xx + 5 , xx + 9 , xx +
13 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 1 , xx + 13 , xx + 5 ) ;
xx [ 1 ] = motionData [ 140 ] ; xx [ 2 ] = motionData [ 141 ] ; xx [ 3 ] =
motionData [ 142 ] ; xx [ 4 ] = motionData [ 143 ] ; xx [ 13 ] =
3.734971681880274e-4 ; xx [ 14 ] = 0.2610715860287157 ; xx [ 15 ] =
0.9653193707101895 ; pm_math_Quaternion_inverseXform_ra ( xx + 1 , xx + 13 ,
xx + 16 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 9 , xx + 16 , xx + 1 )
; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 1 , xx + 13 ) ; xx [ 1 ] =
0.0 ; xx [ 2 ] = xx [ 1 ] ; xx [ 3 ] = xx [ 1 ] ; xx [ 4 ] = xx [ 1 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 2 , xx + 17 ) ; xx [ 1 ] =
motionData [ 21 ] ; xx [ 2 ] = motionData [ 22 ] ; xx [ 3 ] = motionData [ 23
] ; xx [ 4 ] = motionData [ 24 ] ; xx [ 21 ] = - 0.9972896958011036 ; xx [ 22
] = 8.852854494773021e-5 ; xx [ 23 ] = 0.07357482457770348 ;
pm_math_Quaternion_inverseXform_ra ( xx + 1 , xx + 21 , xx + 24 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 9 , xx + 24 , xx + 1 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 1 , xx + 9 ) ; xx [ 1 ] =
1.057530799042361e-15 ; xx [ 2 ] = - 1.0 ; xx [ 3 ] = - 1.460637166772472e-15
; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 1 , xx + 21 ) ; J [ 6 ] =
xx [ 14 ] ; J [ 7 ] = xx [ 18 ] ; J [ 8 ] = xx [ 10 ] ; J [ 9 ] = xx [ 22 ] ;
J [ 10 ] = xx [ 18 ] ; J [ 32 ] = xx [ 15 ] ; J [ 33 ] = xx [ 19 ] ; J [ 34 ]
= xx [ 11 ] ; J [ 35 ] = xx [ 23 ] ; J [ 36 ] = xx [ 19 ] ; return 2 ; }
static size_t computeAssemblyJacobian_9 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * J ) { const double * rtdvd = rtdv -> mDoubles . mValues
; const int * rtdvi = rtdv -> mInts . mValues ; double xx [ 25 ] ; ( void )
rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] =
0.0 ; xx [ 1 ] = 0.9999999900512558 ; xx [ 2 ] = motionData [ 147 ] ; xx [ 3
] = motionData [ 148 ] ; xx [ 4 ] = motionData [ 149 ] ; xx [ 5 ] =
motionData [ 150 ] ; xx [ 6 ] = - 1.115736180703704e-4 ; xx [ 7 ] = -
0.01615139904553786 ; xx [ 8 ] = - 2.564403241205184e-3 ; xx [ 9 ] =
0.9998662629053028 ; pm_math_Quaternion_compose_ra ( xx + 2 , xx + 6 , xx +
10 ) ; xx [ 2 ] = 1.410584573942991e-4 ; xx [ 14 ] = - ( xx [ 1 ] * xx [ 10 ]
+ xx [ 2 ] * xx [ 13 ] ) ; xx [ 15 ] = - ( xx [ 1 ] * xx [ 11 ] + xx [ 2 ] *
xx [ 12 ] ) ; xx [ 16 ] = xx [ 2 ] * xx [ 11 ] - xx [ 1 ] * xx [ 12 ] ; xx [
17 ] = xx [ 2 ] * xx [ 10 ] - xx [ 1 ] * xx [ 13 ] ; xx [ 1 ] = motionData [
56 ] ; xx [ 2 ] = motionData [ 57 ] ; xx [ 3 ] = motionData [ 58 ] ; xx [ 4 ]
= motionData [ 59 ] ; xx [ 10 ] = - 0.9972896958009458 ; xx [ 11 ] =
8.852854518109909e-5 ; xx [ 12 ] = 0.0735748245798411 ;
pm_math_Quaternion_inverseXform_ra ( xx + 1 , xx + 10 , xx + 18 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 6 , xx + 18 , xx + 1 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 14 , xx + 1 , xx + 4 ) ; xx [ 1 ] = -
2.123019642030766e-14 ; xx [ 2 ] = 1.0 ; xx [ 3 ] = 2.92300905702092e-16 ;
pm_math_Quaternion_compDeriv_ra ( xx + 14 , xx + 1 , xx + 8 ) ; xx [ 1 ] =
0.0 ; xx [ 18 ] = xx [ 1 ] ; xx [ 19 ] = xx [ 1 ] ; xx [ 20 ] = xx [ 1 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 14 , xx + 18 , xx + 21 ) ; J [ 15 ] =
xx [ 5 ] ; J [ 16 ] = xx [ 9 ] ; J [ 17 ] = xx [ 22 ] ; J [ 41 ] = xx [ 6 ] ;
J [ 42 ] = xx [ 10 ] ; J [ 43 ] = xx [ 23 ] ; return 2 ; } static size_t
computeAssemblyJacobian_10 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; double xx [ 17 ] ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) state ; ( void ) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] =
0.6378922378578669 ; xx [ 2 ] = 0.6673093902201903 ; xx [ 3 ] = -
0.2974211203852193 ; xx [ 4 ] = 0.2435823223336235 ; xx [ 5 ] = motionData [
7 ] ; xx [ 6 ] = motionData [ 8 ] ; xx [ 7 ] = motionData [ 9 ] ; xx [ 8 ] =
motionData [ 10 ] ; xx [ 9 ] = 0.7940628178106506 ; xx [ 10 ] =
0.6078356671769065 ; xx [ 11 ] = 1.854318411243853e-4 ; xx [ 12 ] = -
9.323758217514975e-5 ; pm_math_Quaternion_compose_ra ( xx + 5 , xx + 9 , xx +
13 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 1 , xx + 13 , xx + 5 ) ;
xx [ 1 ] = 9.585431406944345e-16 ; xx [ 2 ] = 0.9999999999999999 ; xx [ 3 ] =
7.771561172376096e-16 ; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 1 ,
xx + 9 ) ; xx [ 1 ] = 0.0 ; xx [ 2 ] = xx [ 1 ] ; xx [ 3 ] = xx [ 1 ] ; xx [
4 ] = xx [ 1 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 2 , xx + 13
) ; J [ 6 ] = xx [ 10 ] ; J [ 7 ] = xx [ 14 ] ; J [ 32 ] = xx [ 11 ] ; J [ 33
] = xx [ 15 ] ; return 2 ; } static size_t computeAssemblyJacobian_11 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 24 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void
) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 126 ] ; xx [ 2 ] =
motionData [ 127 ] ; xx [ 3 ] = motionData [ 128 ] ; xx [ 4 ] = motionData [
129 ] ; xx [ 5 ] = 0.4299358422201465 ; xx [ 6 ] = - 0.5615531320886276 ; xx
[ 7 ] = - 0.5614212742353903 ; xx [ 8 ] = - 0.4296736019955341 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 5 ] =
0.6821674619340059 ; xx [ 6 ] = - 0.6232966132288484 ; xx [ 7 ] =
0.2788192726663735 ; xx [ 8 ] = 0.2615505668236233 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 5 , xx + 13 ) ; xx [ 9 ]
= 0.0 ; xx [ 10 ] = xx [ 9 ] ; xx [ 11 ] = xx [ 9 ] ; xx [ 12 ] = xx [ 9 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 10 , xx + 17 ) ; xx [ 9 ] =
3.734971681885031e-4 ; xx [ 10 ] = 0.261071586028717 ; xx [ 11 ] =
0.965319370710189 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 9 , xx + 21 )
; pm_math_Quaternion_inverseXform_ra ( xx + 5 , xx + 21 , xx + 1 ) ; xx [ 4 ]
= - xx [ 1 ] ; xx [ 5 ] = - xx [ 2 ] ; xx [ 6 ] = - xx [ 3 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 13 , xx + 4 , xx + 7 ) ; J [ 12 ] = xx
[ 18 ] ; J [ 13 ] = xx [ 8 ] ; J [ 14 ] = xx [ 18 ] ; J [ 38 ] = xx [ 19 ] ;
J [ 39 ] = xx [ 9 ] ; J [ 40 ] = xx [ 19 ] ; return 2 ; } size_t
Control_Bicopter_ae14a523_1_computeAssemblyJacobian ( const void * mech ,
const RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , boolean_T
forVelocitySatisfaction , const double * state , const int * modeVector ,
const double * motionData , double * J ) { ( void ) mech ; ( void ) rtdv ; (
void ) state ; ( void ) modeVector ; ( void ) forVelocitySatisfaction ; (
void ) motionData ; ( void ) J ; switch ( constraintIdx ) { case 0 : return
computeAssemblyJacobian_0 ( rtdv , state , modeVector , motionData , J ) ;
case 1 : return computeAssemblyJacobian_1 ( rtdv , state , modeVector ,
motionData , J ) ; case 2 : return computeAssemblyJacobian_2 ( rtdv , state ,
modeVector , motionData , J ) ; case 3 : return computeAssemblyJacobian_3 (
rtdv , state , modeVector , motionData , J ) ; case 4 : return
computeAssemblyJacobian_4 ( rtdv , state , modeVector , motionData , J ) ;
case 5 : return computeAssemblyJacobian_5 ( rtdv , state , modeVector ,
motionData , J ) ; case 6 : return computeAssemblyJacobian_6 ( rtdv , state ,
modeVector , motionData , J ) ; case 7 : return computeAssemblyJacobian_7 (
rtdv , state , modeVector , motionData , J ) ; case 8 : return
computeAssemblyJacobian_8 ( rtdv , state , modeVector , motionData , J ) ;
case 9 : return computeAssemblyJacobian_9 ( rtdv , state , modeVector ,
motionData , J ) ; case 10 : return computeAssemblyJacobian_10 ( rtdv , state
, modeVector , motionData , J ) ; case 11 : return computeAssemblyJacobian_11
( rtdv , state , modeVector , motionData , J ) ; } return 0 ; } size_t
Control_Bicopter_ae14a523_1_computeFullAssemblyJacobian ( const void * mech ,
const RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
double xx [ 250 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; xx [ 0 ] = 0.0 ; xx [ 1 ] = motionData [ 7 ] ; xx [ 2 ] =
motionData [ 8 ] ; xx [ 3 ] = motionData [ 9 ] ; xx [ 4 ] = motionData [ 10 ]
; xx [ 5 ] = 0.9912919252698491 ; xx [ 6 ] = - 0.1316824810541687 ; xx [ 7 ]
= 1.970490389246134e-4 ; xx [ 8 ] = 6.519118568758793e-5 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 5 , xx + 9 ) ; xx [ 5 ] =
0.9999980252749909 ; xx [ 6 ] = 1.987321342568947e-3 ; xx [ 13 ] = xx [ 5 ] *
motionData [ 70 ] + xx [ 6 ] * motionData [ 71 ] ; xx [ 14 ] = xx [ 5 ] *
motionData [ 71 ] - xx [ 6 ] * motionData [ 70 ] ; xx [ 15 ] = xx [ 5 ] *
motionData [ 72 ] - xx [ 6 ] * motionData [ 73 ] ; xx [ 16 ] = xx [ 5 ] *
motionData [ 73 ] + xx [ 6 ] * motionData [ 72 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 9 , xx + 13 , xx + 5 ) ; xx [ 17
] = 3.734971681880274e-4 ; xx [ 18 ] = 0.2610715860287157 ; xx [ 19 ] =
0.9653193707101895 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 17 , xx + 20
) ; pm_math_Quaternion_inverseXform_ra ( xx + 13 , xx + 20 , xx + 23 ) ; xx [
13 ] = - xx [ 23 ] ; xx [ 14 ] = - xx [ 24 ] ; xx [ 15 ] = - xx [ 25 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 13 , xx + 20 ) ; xx [ 13 ] =
0.0 ; xx [ 14 ] = xx [ 13 ] ; xx [ 15 ] = xx [ 13 ] ; xx [ 16 ] = xx [ 13 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 5 , xx + 14 , xx + 24 ) ; xx [ 5 ] = -
0.9120630938981265 ; xx [ 6 ] = - 0.1421693578567881 ; xx [ 7 ] =
0.3844095801838581 ; xx [ 8 ] = - 0.01257223522036471 ; xx [ 13 ] = 0.5 ; xx
[ 20 ] = xx [ 13 ] * state [ 13 ] ; xx [ 23 ] = sin ( xx [ 20 ] ) ; xx [ 27 ]
= cos ( xx [ 20 ] ) ; xx [ 28 ] = xx [ 17 ] * xx [ 23 ] ; xx [ 29 ] = xx [ 18
] * xx [ 23 ] ; xx [ 30 ] = xx [ 19 ] * xx [ 23 ] ;
pm_math_Quaternion_compose_ra ( xx + 5 , xx + 27 , xx + 31 ) ; xx [ 5 ] =
1.251169307048272e-16 ; xx [ 6 ] = 1.110223024625157e-16 ; xx [ 7 ] = - xx [
5 ] ; xx [ 8 ] = 3.38000027272356e-17 ; pm_math_Quaternion_xform_ra ( xx + 31
, xx + 6 , xx + 27 ) ; xx [ 35 ] = motionData [ 70 ] ; xx [ 36 ] = motionData
[ 71 ] ; xx [ 37 ] = motionData [ 72 ] ; xx [ 38 ] = motionData [ 73 ] ; xx [
6 ] = 9.996376071803525e-5 ; xx [ 7 ] = 0.01545456410810322 ; xx [ 8 ] =
8.60794096301822e-3 ; pm_math_Quaternion_xform_ra ( xx + 35 , xx + 6 , xx +
39 ) ; xx [ 6 ] = 2.908691710579205e-6 ; xx [ 7 ] = 2.868076938248717e-3 ; xx
[ 8 ] = 4.94643927619723e-3 ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 6 ,
xx + 35 ) ; xx [ 6 ] = xx [ 39 ] + motionData [ 74 ] - ( xx [ 35 ] +
motionData [ 11 ] ) ; xx [ 7 ] = xx [ 40 ] + motionData [ 75 ] - ( xx [ 36 ]
+ motionData [ 12 ] ) ; xx [ 8 ] = xx [ 41 ] + motionData [ 76 ] - ( xx [ 37
] + motionData [ 13 ] ) ; xx [ 35 ] = - 1.477235478147115e-3 ; xx [ 36 ] =
9.60335389372561e-7 ; xx [ 37 ] = 3.118418564317923e-7 ;
pm_math_Quaternion_xform_ra ( xx + 31 , xx + 35 , xx + 38 ) ; xx [ 35 ] = -
6.480496605447651e-3 ; xx [ 36 ] = 9.242608862973643e-5 ; xx [ 37 ] = -
2.248932226692751e-5 ; pm_math_Quaternion_xform_ra ( xx + 31 , xx + 35 , xx +
41 ) ; xx [ 35 ] = xx [ 38 ] + xx [ 41 ] ; xx [ 36 ] = xx [ 39 ] + xx [ 42 ]
; xx [ 37 ] = xx [ 40 ] + xx [ 43 ] ; xx [ 20 ] = 2.0 ; xx [ 23 ] = 1.0 ; xx
[ 38 ] = ( xx [ 9 ] * xx [ 11 ] + xx [ 10 ] * xx [ 12 ] ) * xx [ 20 ] ; xx [
39 ] = xx [ 20 ] * ( xx [ 11 ] * xx [ 12 ] - xx [ 9 ] * xx [ 10 ] ) ; xx [ 40
] = xx [ 23 ] - ( xx [ 10 ] * xx [ 10 ] + xx [ 11 ] * xx [ 11 ] ) * xx [ 20 ]
; xx [ 9 ] = - 0.7077023583577552 ; xx [ 10 ] = - 9.268340058968039e-3 ; xx [
11 ] = 0.7064499061132523 ; xx [ 44 ] = - 0.7077023583577557 ; xx [ 45 ] = -
9.268340058967206e-3 ; xx [ 46 ] = 0.7064499061132518 ; xx [ 12 ] =
0.9999999900512558 ; xx [ 24 ] = 1.410584573942991e-4 ; xx [ 30 ] = xx [ 12 ]
* motionData [ 35 ] - xx [ 24 ] * motionData [ 38 ] ; xx [ 47 ] = xx [ 12 ] *
motionData [ 36 ] + xx [ 24 ] * motionData [ 37 ] ; xx [ 48 ] = xx [ 12 ] *
motionData [ 37 ] - xx [ 24 ] * motionData [ 36 ] ; xx [ 49 ] = xx [ 24 ] *
motionData [ 35 ] + xx [ 12 ] * motionData [ 38 ] ; xx [ 50 ] = xx [ 30 ] ;
xx [ 51 ] = xx [ 47 ] ; xx [ 52 ] = xx [ 48 ] ; xx [ 53 ] = xx [ 49 ] ; xx [
54 ] = - 0.9229169573264255 ; xx [ 55 ] = xx [ 54 ] ; xx [ 56 ] = -
0.02080107143120539 ; xx [ 57 ] = 0.382547117836429 ; xx [ 58 ] =
0.03806977727395289 ; pm_math_Quaternion_inverseCompose_ra ( xx + 50 , xx +
55 , xx + 59 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 59 , xx + 14 , xx +
50 ) ; xx [ 55 ] = 0.7044167893867059 ; xx [ 56 ] = - 0.08618526586337766 ;
xx [ 57 ] = 0.7045346597422903 ; xx [ 50 ] = xx [ 49 ] * xx [ 49 ] ; xx [ 53
] = xx [ 49 ] * xx [ 30 ] ; xx [ 58 ] = xx [ 47 ] * xx [ 48 ] ; xx [ 59 ] =
xx [ 23 ] - ( xx [ 48 ] * xx [ 48 ] + xx [ 50 ] ) * xx [ 20 ] ; xx [ 60 ] = (
xx [ 53 ] + xx [ 58 ] ) * xx [ 20 ] ; xx [ 61 ] = xx [ 20 ] * ( xx [ 49 ] *
xx [ 47 ] - xx [ 30 ] * xx [ 48 ] ) ; xx [ 62 ] = xx [ 20 ] * ( xx [ 58 ] -
xx [ 53 ] ) ; xx [ 63 ] = xx [ 23 ] - ( xx [ 50 ] + xx [ 47 ] * xx [ 47 ] ) *
xx [ 20 ] ; xx [ 64 ] = ( xx [ 47 ] * xx [ 30 ] + xx [ 49 ] * xx [ 48 ] ) *
xx [ 20 ] ; xx [ 47 ] = - 0.9229169573264254 ; xx [ 48 ] = -
0.02080107143120469 ; xx [ 49 ] = 0.3825471178364293 ; xx [ 50 ] =
0.0380697772739531 ; xx [ 65 ] = motionData [ 119 ] ; xx [ 66 ] = motionData
[ 120 ] ; xx [ 67 ] = motionData [ 121 ] ; xx [ 68 ] = motionData [ 122 ] ;
xx [ 69 ] = - 0.4813049877214196 ; xx [ 70 ] = - 0.5180593628877815 ; xx [ 71
] = - 0.5180751960991345 ; xx [ 72 ] = - 0.4812048384063002 ;
pm_math_Quaternion_compose_ra ( xx + 65 , xx + 69 , xx + 73 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 47 , xx + 73 , xx + 65 ) ; xx [
47 ] = motionData [ 14 ] ; xx [ 48 ] = motionData [ 15 ] ; xx [ 49 ] =
motionData [ 16 ] ; xx [ 50 ] = motionData [ 17 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 47 , xx + 17 , xx + 73 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 69 , xx + 73 , xx + 76 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 65 , xx + 76 , xx + 69 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 65 , xx + 14 , xx + 76 ) ; xx [ 80 ] =
3.33066907387547e-16 ; xx [ 81 ] = 1.942890293094024e-15 ; xx [ 82 ] = -
1.000000000000001 ; pm_math_Quaternion_compDeriv_ra ( xx + 65 , xx + 80 , xx
+ 83 ) ; pm_math_Quaternion_compose_ra ( xx + 31 , xx + 47 , xx + 65 ) ; xx [
47 ] = 0.013582786684373 ; xx [ 48 ] = - 2.58589188160877e-6 ; xx [ 49 ] = -
0.01849903189296259 ; pm_math_Vector3_cross_ra ( xx + 73 , xx + 47 , xx + 79
) ; pm_math_Quaternion_xform_ra ( xx + 65 , xx + 79 , xx + 47 ) ; xx [ 65 ] =
motionData [ 18 ] ; xx [ 66 ] = motionData [ 19 ] ; xx [ 67 ] = motionData [
20 ] ; pm_math_Vector3_cross_ra ( xx + 17 , xx + 65 , xx + 72 ) ;
pm_math_Quaternion_xform_ra ( xx + 31 , xx + 72 , xx + 65 ) ; xx [ 30 ] = xx
[ 47 ] + xx [ 65 ] + xx [ 41 ] ; xx [ 31 ] = xx [ 48 ] + xx [ 66 ] + xx [ 42
] ; xx [ 32 ] = xx [ 49 ] + xx [ 67 ] + xx [ 43 ] ; xx [ 41 ] = -
0.7077022283680728 ; xx [ 42 ] = - 9.268355963183032e-3 ; xx [ 43 ] =
0.7064500361247099 ; xx [ 47 ] = 0.4090279037421741 ; xx [ 48 ] = -
0.57698859548371 ; xx [ 49 ] = - 0.5768688874880377 ; xx [ 50 ] = -
0.4086595420277526 ; xx [ 33 ] = xx [ 13 ] * state [ 17 ] ; xx [ 34 ] =
0.9972896958011036 ; xx [ 53 ] = sin ( xx [ 33 ] ) ; xx [ 58 ] =
8.852854494773021e-5 ; xx [ 65 ] = 0.07357482457770348 ; xx [ 66 ] = cos ( xx
[ 33 ] ) ; xx [ 67 ] = - ( xx [ 34 ] * xx [ 53 ] ) ; xx [ 68 ] = xx [ 58 ] *
xx [ 53 ] ; xx [ 69 ] = xx [ 65 ] * xx [ 53 ] ; pm_math_Quaternion_compose_ra
( xx + 47 , xx + 66 , xx + 72 ) ; pm_math_Quaternion_compose_ra ( xx + 1 , xx
+ 72 , xx + 47 ) ; xx [ 66 ] = - 1.447435834859361e-6 ; xx [ 67 ] = -
0.01744954274156846 ; xx [ 68 ] = 1.376418986481207e-6 ;
pm_math_Quaternion_xform_ra ( xx + 47 , xx + 66 , xx + 79 ) ; xx [ 47 ] =
1.439701931685288e-6 ; xx [ 48 ] = 0.01494182793334928 ; xx [ 49 ] =
1.536145228271068e-6 ; pm_math_Quaternion_xform_ra ( xx + 72 , xx + 47 , xx +
66 ) ; pm_math_Quaternion_xform_ra ( xx + 1 , xx + 66 , xx + 47 ) ; xx [ 66 ]
= xx [ 79 ] + xx [ 47 ] ; xx [ 67 ] = xx [ 80 ] + xx [ 48 ] ; xx [ 68 ] = xx
[ 81 ] + xx [ 49 ] ; xx [ 47 ] = motionData [ 84 ] ; xx [ 48 ] = motionData [
85 ] ; xx [ 49 ] = motionData [ 86 ] ; xx [ 50 ] = motionData [ 87 ] ; xx [
72 ] = 0.4982139734058924 ; xx [ 73 ] = - 0.4963735918252776 ; xx [ 74 ] = -
0.501668831212296 ; xx [ 75 ] = - 0.5037107084742934 ;
pm_math_Quaternion_compose_ra ( xx + 47 , xx + 72 , xx + 79 ) ; xx [ 47 ] = -
0.6795114020738748 ; xx [ 48 ] = 0.2557971615198039 ; xx [ 49 ] =
0.2852066954013848 ; xx [ 50 ] = - 0.6256909840394302 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 79 , xx + 47 , xx + 72 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 72 , xx + 14 , xx + 47 ) ; xx [ 72 ] =
- 0.7077023583577412 ; xx [ 73 ] = - 9.268340058974089e-3 ; xx [ 74 ] =
0.7064499061132667 ; xx [ 86 ] = ( xx [ 79 ] * xx [ 81 ] + xx [ 80 ] * xx [
82 ] ) * xx [ 20 ] ; xx [ 87 ] = xx [ 20 ] * ( xx [ 81 ] * xx [ 82 ] - xx [
79 ] * xx [ 80 ] ) ; xx [ 88 ] = xx [ 23 ] - ( xx [ 80 ] * xx [ 80 ] + xx [
81 ] * xx [ 81 ] ) * xx [ 20 ] ; xx [ 79 ] = motionData [ 49 ] ; xx [ 80 ] =
motionData [ 50 ] ; xx [ 81 ] = motionData [ 51 ] ; xx [ 82 ] = motionData [
52 ] ; xx [ 89 ] = 0.5180593628885556 ; xx [ 90 ] = - 0.4813049877207372 ; xx
[ 91 ] = - 0.4812048384058716 ; xx [ 92 ] = 0.518075196099393 ;
pm_math_Quaternion_compose_ra ( xx + 79 , xx + 89 , xx + 93 ) ; xx [ 89 ] =
motionData [ 42 ] ; xx [ 90 ] = motionData [ 43 ] ; xx [ 91 ] = motionData [
44 ] ; xx [ 92 ] = motionData [ 45 ] ; xx [ 97 ] = 0.1316824810541773 ; xx [
98 ] = 0.991291925269848 ; xx [ 99 ] = 6.51911856886885e-5 ; xx [ 100 ] = -
1.970490389163408e-4 ; pm_math_Quaternion_compose_ra ( xx + 89 , xx + 97 , xx
+ 101 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 93 , xx + 101 , xx + 97
) ; xx [ 33 ] = 1.0 ; xx [ 47 ] = - xx [ 33 ] ; xx [ 105 ] =
1.729508463527152e-14 ; xx [ 106 ] = 1.587618925213974e-14 ; xx [ 107 ] = xx
[ 47 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 97 , xx + 105 , xx + 108 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 97 , xx + 14 , xx + 112 ) ; xx [ 50 ]
= 0.9972896958009458 ; xx [ 53 ] = 8.852854518109909e-5 ; xx [ 69 ] =
0.0735748245798411 ; xx [ 105 ] = - xx [ 50 ] ; xx [ 106 ] = xx [ 53 ] ; xx [
107 ] = xx [ 69 ] ; pm_math_Quaternion_xform_ra ( xx + 79 , xx + 105 , xx +
116 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 101 , xx + 116 , xx + 119 )
; xx [ 101 ] = - xx [ 119 ] ; xx [ 102 ] = - xx [ 120 ] ; xx [ 103 ] = - xx [
121 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 97 , xx + 101 , xx + 116 ) ;
xx [ 97 ] = - 0.1316824519486606 ; xx [ 98 ] = 0.9912919062119712 ; xx [ 99 ]
= 2.050212948449675e-4 ; xx [ 100 ] = - 2.156239646078782e-4 ; xx [ 75 ] = xx
[ 13 ] * state [ 27 ] ; xx [ 76 ] = 3.734971681885031e-4 ; xx [ 83 ] = sin (
xx [ 75 ] ) ; xx [ 101 ] = 0.261071586028717 ; xx [ 102 ] = 0.965319370710189
; xx [ 119 ] = cos ( xx [ 75 ] ) ; xx [ 120 ] = xx [ 76 ] * xx [ 83 ] ; xx [
121 ] = xx [ 101 ] * xx [ 83 ] ; xx [ 122 ] = xx [ 102 ] * xx [ 83 ] ;
pm_math_Quaternion_compose_ra ( xx + 97 , xx + 119 , xx + 123 ) ; xx [ 97 ] =
6.459047385173115e-3 ; xx [ 98 ] = - 9.24648174485767e-5 ; xx [ 99 ] =
2.250809555350908e-5 ; pm_math_Quaternion_xform_ra ( xx + 123 , xx + 97 , xx
+ 119 ) ; xx [ 97 ] = - 6.480496605446796e-3 ; xx [ 98 ] =
9.242608862956347e-5 ; xx [ 99 ] = - 2.2489322266878e-5 ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 97 , xx + 127 ) ; xx [ 97 ] =
xx [ 119 ] + xx [ 127 ] ; xx [ 98 ] = xx [ 120 ] + xx [ 128 ] ; xx [ 99 ] =
xx [ 121 ] + xx [ 129 ] ; xx [ 75 ] = ( xx [ 94 ] * xx [ 94 ] + xx [ 95 ] *
xx [ 95 ] ) * xx [ 20 ] ; xx [ 119 ] = ( xx [ 93 ] * xx [ 95 ] + xx [ 94 ] *
xx [ 96 ] ) * xx [ 20 ] ; xx [ 120 ] = xx [ 20 ] * ( xx [ 95 ] * xx [ 96 ] -
xx [ 93 ] * xx [ 94 ] ) ; xx [ 121 ] = xx [ 23 ] - xx [ 75 ] ; xx [ 93 ] = -
0.5181324366224915 ; xx [ 94 ] = - 0.4812371049201607 ; xx [ 95 ] = -
0.4812727257575918 ; xx [ 96 ] = 0.5180021142906276 ; xx [ 83 ] = xx [ 13 ] *
state [ 31 ] ; xx [ 100 ] = sin ( xx [ 83 ] ) ; xx [ 130 ] = cos ( xx [ 83 ]
) ; xx [ 131 ] = - ( xx [ 50 ] * xx [ 100 ] ) ; xx [ 132 ] = xx [ 53 ] * xx [
100 ] ; xx [ 133 ] = xx [ 69 ] * xx [ 100 ] ; pm_math_Quaternion_compose_ra (
xx + 93 , xx + 130 , xx + 134 ) ; xx [ 93 ] = 4.103874629447085e-18 ; xx [ 94
] = 2.081668171172169e-16 ; xx [ 95 ] = 5.538917848685321e-17 ;
pm_math_Quaternion_xform_ra ( xx + 134 , xx + 93 , xx + 130 ) ; xx [ 93 ] = -
9.045674901471507e-5 ; xx [ 94 ] = - 2.965461248705281e-3 ; xx [ 95 ] =
0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 89 , xx + 93 , xx +
138 ) ; xx [ 89 ] = - 0.01609995664951487 ; xx [ 90 ] = 2.933657308331041e-6
; xx [ 91 ] = - 0.0137731566582556 ; pm_math_Quaternion_xform_ra ( xx + 79 ,
xx + 89 , xx + 92 ) ; xx [ 79 ] = xx [ 138 ] + motionData [ 46 ] - ( xx [ 92
] + motionData [ 53 ] ) ; xx [ 80 ] = xx [ 139 ] + motionData [ 47 ] - ( xx [
93 ] + motionData [ 54 ] ) ; xx [ 81 ] = xx [ 140 ] + motionData [ 48 ] - (
xx [ 94 ] + motionData [ 55 ] ) ; xx [ 89 ] = - 1.435160843344562e-6 ; xx [
90 ] = - 0.0149203787001616 ; xx [ 91 ] = - 1.500400464949372e-6 ;
pm_math_Quaternion_xform_ra ( xx + 134 , xx + 89 , xx + 92 ) ; xx [ 89 ] =
1.439701935798469e-6 ; xx [ 90 ] = 0.01494182793334727 ; xx [ 91 ] =
1.53614523658561e-6 ; pm_math_Quaternion_xform_ra ( xx + 134 , xx + 89 , xx +
138 ) ; xx [ 89 ] = xx [ 92 ] + xx [ 138 ] ; xx [ 90 ] = xx [ 93 ] + xx [ 139
] ; xx [ 91 ] = xx [ 94 ] + xx [ 140 ] ; xx [ 92 ] = motionData [ 126 ] ; xx
[ 93 ] = motionData [ 127 ] ; xx [ 94 ] = motionData [ 128 ] ; xx [ 95 ] =
motionData [ 129 ] ; xx [ 133 ] = 0.991291925269849 ; xx [ 134 ] = -
0.1316824810541695 ; xx [ 135 ] = 1.970490389249778e-4 ; xx [ 136 ] =
6.519118568753968e-5 ; pm_math_Quaternion_compose_ra ( xx + 92 , xx + 133 ,
xx + 137 ) ; xx [ 133 ] = motionData [ 77 ] ; xx [ 134 ] = motionData [ 78 ]
; xx [ 135 ] = motionData [ 79 ] ; xx [ 136 ] = motionData [ 80 ] ; xx [ 141
] = 0.9999980252749867 ; xx [ 142 ] = - 1.987321342568328e-3 ; xx [ 143 ] =
9.226742840440916e-8 ; xx [ 144 ] = - 1.833653884871776e-10 ;
pm_math_Quaternion_compose_ra ( xx + 133 , xx + 141 , xx + 145 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 137 , xx + 145 , xx + 141 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 141 , xx + 14 , xx + 149 ) ; xx [ 153
] = xx [ 76 ] ; xx [ 154 ] = xx [ 101 ] ; xx [ 155 ] = xx [ 102 ] ;
pm_math_Quaternion_xform_ra ( xx + 92 , xx + 153 , xx + 100 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 145 , xx + 100 , xx + 153 ) ; xx [
145 ] = - xx [ 153 ] ; xx [ 146 ] = - xx [ 154 ] ; xx [ 147 ] = - xx [ 155 ]
; pm_math_Quaternion_compDeriv_ra ( xx + 141 , xx + 145 , xx + 153 ) ; xx [
141 ] = ( xx [ 137 ] * xx [ 139 ] + xx [ 138 ] * xx [ 140 ] ) * xx [ 20 ] ;
xx [ 142 ] = xx [ 20 ] * ( xx [ 139 ] * xx [ 140 ] - xx [ 137 ] * xx [ 138 ]
) ; xx [ 143 ] = xx [ 23 ] - ( xx [ 138 ] * xx [ 138 ] + xx [ 139 ] * xx [
139 ] ) * xx [ 20 ] ; xx [ 137 ] = motionData [ 35 ] ; xx [ 138 ] =
motionData [ 36 ] ; xx [ 139 ] = motionData [ 37 ] ; xx [ 140 ] = motionData
[ 38 ] ; pm_math_Quaternion_compose_ra ( xx + 137 , xx + 123 , xx + 144 ) ;
xx [ 122 ] = - 2.775557561562891e-16 ; xx [ 123 ] = xx [ 5 ] ; xx [ 124 ] = -
3.373224009145526e-17 ; pm_math_Quaternion_xform_ra ( xx + 144 , xx + 122 ,
xx + 156 ) ; xx [ 122 ] = 9.477449468724687e-5 ; xx [ 123 ] =
0.01545456410835202 ; xx [ 124 ] = 8.607941025578736e-3 ;
pm_math_Quaternion_xform_ra ( xx + 133 , xx + 122 , xx + 159 ) ; xx [ 122 ] =
2.908691710755093e-6 ; xx [ 123 ] = 2.868076938250087e-3 ; xx [ 124 ] =
4.94643927619916e-3 ; pm_math_Quaternion_xform_ra ( xx + 92 , xx + 122 , xx +
133 ) ; xx [ 122 ] = xx [ 159 ] + motionData [ 81 ] - ( xx [ 133 ] +
motionData [ 130 ] ) ; xx [ 123 ] = xx [ 160 ] + motionData [ 82 ] - ( xx [
134 ] + motionData [ 131 ] ) ; xx [ 124 ] = xx [ 161 ] + motionData [ 83 ] -
( xx [ 135 ] + motionData [ 132 ] ) ; xx [ 133 ] = - 1.477235478147926e-3 ;
xx [ 134 ] = 9.60335389539274e-7 ; xx [ 135 ] = 3.118418563877457e-7 ;
pm_math_Quaternion_xform_ra ( xx + 144 , xx + 133 , xx + 159 ) ;
pm_math_Quaternion_xform_ra ( xx + 137 , xx + 127 , xx + 133 ) ; xx [ 125 ] =
xx [ 159 ] + xx [ 133 ] ; xx [ 126 ] = xx [ 160 ] + xx [ 134 ] ; xx [ 127 ] =
xx [ 161 ] + xx [ 135 ] ; xx [ 133 ] = - ( ( motionData [ 35 ] * motionData [
37 ] + motionData [ 36 ] * motionData [ 38 ] ) * xx [ 20 ] ) ; xx [ 134 ] =
xx [ 20 ] * ( motionData [ 35 ] * motionData [ 36 ] - motionData [ 37 ] *
motionData [ 38 ] ) ; xx [ 135 ] = ( motionData [ 36 ] * motionData [ 36 ] +
motionData [ 37 ] * motionData [ 37 ] ) * xx [ 20 ] - xx [ 23 ] ; xx [ 136 ]
= - 0.7077023583577554 ; xx [ 137 ] = - 9.268340058966987e-3 ; xx [ 138 ] =
0.7064499061132521 ; xx [ 144 ] = motionData [ 21 ] ; xx [ 145 ] = motionData
[ 22 ] ; xx [ 146 ] = motionData [ 23 ] ; xx [ 147 ] = motionData [ 24 ] ; xx
[ 159 ] = 0.4931954203595264 ; xx [ 160 ] = - 0.4905194435002385 ; xx [ 161 ]
= - 0.5067824161638587 ; xx [ 162 ] = - 0.509235245786982 ;
pm_math_Quaternion_compose_ra ( xx + 144 , xx + 159 , xx + 163 ) ; xx [ 159 ]
= motionData [ 28 ] ; xx [ 160 ] = motionData [ 29 ] ; xx [ 161 ] =
motionData [ 30 ] ; xx [ 162 ] = motionData [ 31 ] ; xx [ 167 ] =
0.6370506001153763 ; xx [ 168 ] = - 0.6370493477200306 ; xx [ 169 ] = -
0.3068667118838679 ; xx [ 170 ] = - 0.3068672067844752 ;
pm_math_Quaternion_compose_ra ( xx + 159 , xx + 167 , xx + 171 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 163 , xx + 171 , xx + 167 ) ; xx
[ 5 ] = 3.05954793225871e-4 ; xx [ 50 ] = 0.9999868227746886 ; xx [ 53 ] =
5.124516430675685e-3 ; xx [ 175 ] = xx [ 5 ] ; xx [ 176 ] = - xx [ 50 ] ; xx
[ 177 ] = - xx [ 53 ] ; pm_math_Quaternion_xform_ra ( xx + 144 , xx + 175 ,
xx + 178 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 171 , xx + 178 , xx +
175 ) ; xx [ 171 ] = - xx [ 175 ] ; xx [ 172 ] = - xx [ 176 ] ; xx [ 173 ] =
- xx [ 177 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 167 , xx + 171 , xx +
174 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 167 , xx + 14 , xx + 178 ) ;
xx [ 171 ] = 1.665334536937735e-16 ; xx [ 172 ] = 5.551115123125783e-16 ; xx
[ 173 ] = xx [ 33 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 167 , xx + 171 ,
xx + 182 ) ; xx [ 167 ] = 0.4716196941540828 ; xx [ 168 ] = -
0.5117381057633613 ; xx [ 169 ] = 0.5267317275910312 ; xx [ 170 ] = -
0.4881113216723255 ; xx [ 69 ] = xx [ 13 ] * state [ 19 ] ; xx [ 76 ] = sin (
xx [ 69 ] ) ; xx [ 171 ] = cos ( xx [ 69 ] ) ; xx [ 172 ] = xx [ 5 ] * xx [
76 ] ; xx [ 173 ] = - ( xx [ 50 ] * xx [ 76 ] ) ; xx [ 174 ] = - ( xx [ 53 ]
* xx [ 76 ] ) ; pm_math_Quaternion_compose_ra ( xx + 167 , xx + 171 , xx +
185 ) ; xx [ 167 ] = - 2.237793284010081e-16 ; xx [ 168 ] =
1.923188306740889e-18 ; xx [ 169 ] = - 3.886322687274291e-16 ;
pm_math_Quaternion_xform_ra ( xx + 185 , xx + 167 , xx + 170 ) ; xx [ 167 ] =
- 3.865146760306702e-8 ; xx [ 168 ] = - 5.289307885790379e-3 ; xx [ 169 ] = -
3.071851003736677e-7 ; pm_math_Quaternion_xform_ra ( xx + 159 , xx + 167 , xx
+ 189 ) ; xx [ 159 ] = 2.392300359455244e-7 ; xx [ 160 ] =
3.887917980054749e-3 ; xx [ 161 ] = 2.025360777142851e-5 ;
pm_math_Quaternion_xform_ra ( xx + 144 , xx + 159 , xx + 167 ) ; xx [ 159 ] =
xx [ 189 ] + motionData [ 32 ] - ( xx [ 167 ] + motionData [ 25 ] ) ; xx [
160 ] = xx [ 190 ] + motionData [ 33 ] - ( xx [ 168 ] + motionData [ 26 ] ) ;
xx [ 161 ] = xx [ 191 ] + motionData [ 34 ] - ( xx [ 169 ] + motionData [ 27
] ) ; xx [ 167 ] = - 3.296413151655537e-7 ; xx [ 168 ] = -
7.422626627699278e-9 ; xx [ 169 ] = 1.428754025224236e-6 ;
pm_math_Quaternion_xform_ra ( xx + 185 , xx + 167 , xx + 189 ) ; xx [ 167 ] =
3.296413151917778e-7 ; xx [ 168 ] = 7.422626626299722e-9 ; xx [ 169 ] = -
1.428754024949564e-6 ; pm_math_Quaternion_xform_ra ( xx + 185 , xx + 167 , xx
+ 192 ) ; xx [ 167 ] = xx [ 189 ] + xx [ 192 ] ; xx [ 168 ] = xx [ 190 ] + xx
[ 193 ] ; xx [ 169 ] = xx [ 191 ] + xx [ 194 ] ; xx [ 185 ] = ( xx [ 163 ] *
xx [ 165 ] + xx [ 164 ] * xx [ 166 ] ) * xx [ 20 ] ; xx [ 186 ] = xx [ 20 ] *
( xx [ 165 ] * xx [ 166 ] - xx [ 163 ] * xx [ 164 ] ) ; xx [ 187 ] = xx [ 23
] - ( xx [ 164 ] * xx [ 164 ] + xx [ 165 ] * xx [ 165 ] ) * xx [ 20 ] ; xx [
162 ] = 0.07357483435689677 ; xx [ 163 ] = 1.128000373271654e-4 ; xx [ 164 ]
= 0.9972896926297332 ; xx [ 188 ] = 0.6254151447093548 ; xx [ 189 ] = -
0.6480628292580688 ; xx [ 190 ] = 0.3298467071707108 ; xx [ 191 ] = -
0.2829692843240122 ; xx [ 5 ] = xx [ 13 ] * state [ 23 ] ; xx [ 50 ] =
1.380846645360201e-7 ; xx [ 53 ] = sin ( xx [ 5 ] ) ; xx [ 69 ] =
0.9999999999981867 ; xx [ 192 ] = cos ( xx [ 5 ] ) ; xx [ 193 ] = - ( xx [ 50
] * xx [ 53 ] ) ; xx [ 194 ] = xx [ 69 ] * xx [ 53 ] ; xx [ 195 ] =
1.899414133310628e-6 * xx [ 53 ] ; pm_math_Quaternion_compose_ra ( xx + 188 ,
xx + 192 , xx + 196 ) ; xx [ 188 ] = - 2.971385142194091e-7 ; xx [ 189 ] = -
1.158326953740249e-13 ; xx [ 190 ] = 3.938183990803402e-8 ;
pm_math_Quaternion_xform_ra ( xx + 196 , xx + 188 , xx + 191 ) ; xx [ 188 ] =
2.971385142511557e-7 ; xx [ 189 ] = 1.158326953518261e-13 ; xx [ 190 ] = -
3.938183989403888e-8 ; pm_math_Quaternion_xform_ra ( xx + 196 , xx + 188 , xx
+ 200 ) ; xx [ 188 ] = xx [ 191 ] + xx [ 200 ] ; xx [ 189 ] = xx [ 192 ] + xx
[ 201 ] ; xx [ 190 ] = xx [ 193 ] + xx [ 202 ] ; xx [ 191 ] = motionData [ 56
] ; xx [ 192 ] = motionData [ 57 ] ; xx [ 193 ] = motionData [ 58 ] ; xx [
194 ] = motionData [ 59 ] ; xx [ 195 ] = 0.493195420359526 ; xx [ 196 ] = -
0.4905194435002388 ; xx [ 197 ] = - 0.5067824161638583 ; xx [ 198 ] = -
0.5092352457869823 ; pm_math_Quaternion_compose_ra ( xx + 191 , xx + 195 , xx
+ 199 ) ; xx [ 195 ] = motionData [ 63 ] ; xx [ 196 ] = motionData [ 64 ] ;
xx [ 197 ] = motionData [ 65 ] ; xx [ 198 ] = motionData [ 66 ] ; xx [ 203 ]
= 0.6370493477200303 ; xx [ 204 ] = 0.6370506001153765 ; xx [ 205 ] = -
0.3068672067844756 ; xx [ 206 ] = 0.3068667118838675 ;
pm_math_Quaternion_compose_ra ( xx + 195 , xx + 203 , xx + 207 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 199 , xx + 207 , xx + 203 ) ; xx
[ 5 ] = 3.059547932236506e-4 ; xx [ 53 ] = 0.9999868227746889 ; xx [ 76 ] =
5.124516430674353e-3 ; xx [ 211 ] = xx [ 5 ] ; xx [ 212 ] = - xx [ 53 ] ; xx
[ 213 ] = - xx [ 76 ] ; pm_math_Quaternion_xform_ra ( xx + 191 , xx + 211 ,
xx + 214 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 207 , xx + 214 , xx +
211 ) ; xx [ 207 ] = - xx [ 211 ] ; xx [ 208 ] = - xx [ 212 ] ; xx [ 209 ] =
- xx [ 213 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 203 , xx + 207 , xx +
210 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 203 , xx + 14 , xx + 214 ) ;
xx [ 207 ] = - 2.55351295663786e-15 ; xx [ 208 ] = - 2.109423746787797e-15 ;
xx [ 209 ] = - 1.0 ; pm_math_Quaternion_compDeriv_ra ( xx + 203 , xx + 207 ,
xx + 218 ) ; xx [ 203 ] = 0.4716196941536389 ; xx [ 204 ] = -
0.5117381057636369 ; xx [ 205 ] = 0.5267317275917935 ; xx [ 206 ] = -
0.4881113216716428 ; xx [ 82 ] = xx [ 13 ] * state [ 33 ] ; xx [ 83 ] = sin (
xx [ 82 ] ) ; xx [ 207 ] = cos ( xx [ 82 ] ) ; xx [ 208 ] = xx [ 5 ] * xx [
83 ] ; xx [ 209 ] = - ( xx [ 53 ] * xx [ 83 ] ) ; xx [ 210 ] = - ( xx [ 76 ]
* xx [ 83 ] ) ; pm_math_Quaternion_compose_ra ( xx + 203 , xx + 207 , xx +
221 ) ; xx [ 203 ] = - 1.334869714764153e-15 ; xx [ 204 ] =
5.850668324922266e-18 ; xx [ 205 ] = - 1.221407957413545e-15 ;
pm_math_Quaternion_xform_ra ( xx + 221 , xx + 203 , xx + 206 ) ; xx [ 203 ] =
- 3.975614476060287e-8 ; xx [ 204 ] = 2.710692114195085e-3 ; xx [ 205 ] = -
2.919897872271535e-7 ; pm_math_Quaternion_xform_ra ( xx + 195 , xx + 203 , xx
+ 225 ) ; xx [ 195 ] = 2.392300359480238e-7 ; xx [ 196 ] =
3.887917980054739e-3 ; xx [ 197 ] = 2.025360777142854e-5 ;
pm_math_Quaternion_xform_ra ( xx + 191 , xx + 195 , xx + 203 ) ; xx [ 195 ] =
xx [ 225 ] + motionData [ 67 ] - ( xx [ 203 ] + motionData [ 60 ] ) ; xx [
196 ] = xx [ 226 ] + motionData [ 68 ] - ( xx [ 204 ] + motionData [ 61 ] ) ;
xx [ 197 ] = xx [ 227 ] + motionData [ 69 ] - ( xx [ 205 ] + motionData [ 62
] ) ; xx [ 203 ] = - 3.296413151708188e-7 ; xx [ 204 ] = -
7.422626627666806e-9 ; xx [ 205 ] = 1.4287540252181e-6 ;
pm_math_Quaternion_xform_ra ( xx + 221 , xx + 203 , xx + 225 ) ; xx [ 203 ] =
3.296413155003147e-7 ; xx [ 204 ] = 7.422626627203207e-9 ; xx [ 205 ] = -
1.428754025107962e-6 ; pm_math_Quaternion_xform_ra ( xx + 221 , xx + 203 , xx
+ 228 ) ; xx [ 203 ] = xx [ 225 ] + xx [ 228 ] ; xx [ 204 ] = xx [ 226 ] + xx
[ 229 ] ; xx [ 205 ] = xx [ 227 ] + xx [ 230 ] ; xx [ 221 ] = ( xx [ 199 ] *
xx [ 201 ] + xx [ 200 ] * xx [ 202 ] ) * xx [ 20 ] ; xx [ 222 ] = xx [ 20 ] *
( xx [ 201 ] * xx [ 202 ] - xx [ 199 ] * xx [ 200 ] ) ; xx [ 223 ] = xx [ 23
] - ( xx [ 200 ] * xx [ 200 ] + xx [ 201 ] * xx [ 201 ] ) * xx [ 20 ] ; xx [
198 ] = 0.07357483435904683 ; xx [ 199 ] = 1.128000365619997e-4 ; xx [ 200 ]
= 0.9972896926295751 ; xx [ 224 ] = 0.6252280994013473 ; xx [ 225 ] =
0.6477664378100223 ; xx [ 226 ] = 0.330373832435706 ; xx [ 227 ] =
0.2834459324238707 ; xx [ 5 ] = xx [ 13 ] * state [ 37 ] ; xx [ 13 ] = sin (
xx [ 5 ] ) ; xx [ 228 ] = cos ( xx [ 5 ] ) ; xx [ 229 ] = - ( xx [ 50 ] * xx
[ 13 ] ) ; xx [ 230 ] = xx [ 69 ] * xx [ 13 ] ; xx [ 231 ] =
1.899414131645294e-6 * xx [ 13 ] ; pm_math_Quaternion_compose_ra ( xx + 224 ,
xx + 228 , xx + 232 ) ; xx [ 224 ] = - 2.971385141348656e-7 ; xx [ 225 ] = -
1.158326949952305e-13 ; xx [ 226 ] = 3.938183974928171e-8 ;
pm_math_Quaternion_xform_ra ( xx + 232 , xx + 224 , xx + 227 ) ; xx [ 224 ] =
2.971385144603623e-7 ; xx [ 225 ] = 1.158326948378051e-13 ; xx [ 226 ] = -
3.938183964273754e-8 ; pm_math_Quaternion_xform_ra ( xx + 232 , xx + 224 , xx
+ 236 ) ; xx [ 224 ] = xx [ 227 ] + xx [ 236 ] ; xx [ 225 ] = xx [ 228 ] + xx
[ 237 ] ; xx [ 226 ] = xx [ 229 ] + xx [ 238 ] ; xx [ 227 ] =
0.02080107143120473 ; xx [ 228 ] = xx [ 54 ] ; xx [ 229 ] =
0.0380697772739531 ; xx [ 230 ] = - 0.3825471178364295 ; xx [ 231 ] =
motionData [ 133 ] ; xx [ 232 ] = motionData [ 134 ] ; xx [ 233 ] =
motionData [ 135 ] ; xx [ 234 ] = motionData [ 136 ] ; xx [ 235 ] =
0.8739852835857713 ; xx [ 236 ] = 2.165051543389225e-3 ; xx [ 237 ] =
0.4859456095725561 ; xx [ 238 ] = 1.378827208253894e-3 ;
pm_math_Quaternion_compose_ra ( xx + 231 , xx + 235 , xx + 239 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 227 , xx + 239 , xx + 231 ) ; xx
[ 227 ] = motionData [ 140 ] ; xx [ 228 ] = motionData [ 141 ] ; xx [ 229 ] =
motionData [ 142 ] ; xx [ 230 ] = motionData [ 143 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 227 , xx + 17 , xx + 239 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 235 , xx + 239 , xx + 17 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 231 , xx + 17 , xx + 227 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 231 , xx + 14 , xx + 17 ) ; xx [ 239 ]
= - xx [ 34 ] ; xx [ 240 ] = xx [ 58 ] ; xx [ 241 ] = xx [ 65 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 144 , xx + 239 , xx + 242 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 235 , xx + 242 , xx + 144 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 231 , xx + 144 , xx + 235 ) ; xx [ 144
] = 1.057530799042361e-15 ; xx [ 145 ] = xx [ 47 ] ; xx [ 146 ] = -
1.460637166772472e-15 ; pm_math_Quaternion_compDeriv_ra ( xx + 231 , xx + 144
, xx + 239 ) ; xx [ 144 ] = motionData [ 147 ] ; xx [ 145 ] = motionData [
148 ] ; xx [ 146 ] = motionData [ 149 ] ; xx [ 147 ] = motionData [ 150 ] ;
xx [ 230 ] = - 1.115736180703704e-4 ; xx [ 231 ] = - 0.01615139904553786 ; xx
[ 232 ] = - 2.564403241205184e-3 ; xx [ 233 ] = 0.9998662629053028 ;
pm_math_Quaternion_compose_ra ( xx + 144 , xx + 230 , xx + 242 ) ; xx [ 144 ]
= - ( xx [ 12 ] * xx [ 242 ] + xx [ 24 ] * xx [ 245 ] ) ; xx [ 145 ] = - ( xx
[ 12 ] * xx [ 243 ] + xx [ 24 ] * xx [ 244 ] ) ; xx [ 146 ] = xx [ 24 ] * xx
[ 243 ] - xx [ 12 ] * xx [ 244 ] ; xx [ 147 ] = xx [ 24 ] * xx [ 242 ] - xx [
12 ] * xx [ 245 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 191 , xx + 105
, xx + 242 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 230 , xx + 242 , xx
+ 103 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 144 , xx + 103 , xx + 191 )
; xx [ 103 ] = - 2.123019642030766e-14 ; xx [ 104 ] = xx [ 33 ] ; xx [ 105 ]
= 2.92300905702092e-16 ; pm_math_Quaternion_compDeriv_ra ( xx + 144 , xx +
103 , xx + 230 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 144 , xx + 14 , xx
+ 103 ) ; xx [ 144 ] = 0.6378922378578669 ; xx [ 145 ] = 0.6673093902201903 ;
xx [ 146 ] = - 0.2974211203852193 ; xx [ 147 ] = 0.2435823223336235 ; xx [
242 ] = 0.7940628178106506 ; xx [ 243 ] = 0.6078356671769065 ; xx [ 244 ] =
1.854318411243853e-4 ; xx [ 245 ] = - 9.323758217514975e-5 ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 242 , xx + 246 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 144 , xx + 246 , xx + 1 ) ; xx [
106 ] = 9.585431406944345e-16 ; xx [ 107 ] = 0.9999999999999999 ; xx [ 108 ]
= 7.771561172376096e-16 ; pm_math_Quaternion_compDeriv_ra ( xx + 1 , xx + 106
, xx + 144 ) ; pm_math_Quaternion_compDeriv_ra ( xx + 1 , xx + 14 , xx + 242
) ; xx [ 1 ] = 0.4299358422201465 ; xx [ 2 ] = - 0.5615531320886276 ; xx [ 3
] = - 0.5614212742353903 ; xx [ 4 ] = - 0.4296736019955341 ;
pm_math_Quaternion_compose_ra ( xx + 92 , xx + 1 , xx + 245 ) ; xx [ 1 ] =
0.6821674619340059 ; xx [ 2 ] = - 0.6232966132288484 ; xx [ 3 ] =
0.2788192726663735 ; xx [ 4 ] = 0.2615505668236233 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 245 , xx + 1 , xx + 92 ) ;
pm_math_Quaternion_compDeriv_ra ( xx + 92 , xx + 14 , xx + 245 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 1 , xx + 100 , xx + 12 ) ; xx [ 1 ]
= - xx [ 12 ] ; xx [ 2 ] = - xx [ 13 ] ; xx [ 3 ] = - xx [ 14 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 92 , xx + 1 , xx + 12 ) ; J [ 6 ] = xx
[ 21 ] ; J [ 7 ] = xx [ 25 ] ; J [ 19 ] = xx [ 25 ] ; J [ 32 ] = xx [ 22 ] ;
J [ 33 ] = xx [ 26 ] ; J [ 45 ] = xx [ 26 ] ; J [ 58 ] =
pm_math_Vector3_dot_ra ( xx + 27 , xx + 6 ) - pm_math_Vector3_dot_ra ( xx +
35 , xx + 38 ) ; J [ 59 ] = - pm_math_Vector3_dot_ra ( xx + 9 , xx + 38 ) ; J
[ 71 ] = pm_math_Vector3_dot_ra ( xx + 44 , xx + 38 ) ; J [ 90 ] = xx [ 51 ]
; J [ 116 ] = xx [ 52 ] ; J [ 142 ] = - pm_math_Vector3_dot_ra ( xx + 55 , xx
+ 59 ) ; J [ 168 ] = - pm_math_Vector3_dot_ra ( xx + 55 , xx + 62 ) ; J [ 188
] = xx [ 70 ] ; J [ 189 ] = xx [ 77 ] ; J [ 190 ] = xx [ 84 ] ; J [ 214 ] =
xx [ 71 ] ; J [ 215 ] = xx [ 78 ] ; J [ 216 ] = xx [ 85 ] ; J [ 240 ] =
pm_math_Vector3_dot_ra ( xx + 30 , xx + 41 ) ; J [ 241 ] = 0.9999999999999832
; J [ 242 ] = pm_math_Vector3_dot_ra ( xx + 66 , xx + 41 ) ; J [ 281 ] = xx [
48 ] ; J [ 307 ] = xx [ 49 ] ; J [ 333 ] = - pm_math_Vector3_dot_ra ( xx + 72
, xx + 86 ) ; J [ 351 ] = xx [ 109 ] ; J [ 352 ] = xx [ 113 ] ; J [ 353 ] =
xx [ 117 ] ; J [ 377 ] = xx [ 110 ] ; J [ 378 ] = xx [ 114 ] ; J [ 379 ] = xx
[ 118 ] ; J [ 403 ] = pm_math_Vector3_dot_ra ( xx + 97 , xx + 119 ) ; J [ 404
] = xx [ 75 ] - xx [ 23 ] ; J [ 405 ] = pm_math_Vector3_dot_ra ( xx + 130 ,
xx + 79 ) - pm_math_Vector3_dot_ra ( xx + 89 , xx + 119 ) ; J [ 428 ] = xx [
150 ] ; J [ 429 ] = xx [ 154 ] ; J [ 430 ] = xx [ 150 ] ; J [ 436 ] = xx [
150 ] ; J [ 454 ] = xx [ 151 ] ; J [ 455 ] = xx [ 155 ] ; J [ 456 ] = xx [
151 ] ; J [ 462 ] = xx [ 151 ] ; J [ 480 ] = - pm_math_Vector3_dot_ra ( xx +
55 , xx + 141 ) ; J [ 481 ] = pm_math_Vector3_dot_ra ( xx + 156 , xx + 122 )
- pm_math_Vector3_dot_ra ( xx + 125 , xx + 141 ) ; J [ 482 ] = -
pm_math_Vector3_dot_ra ( xx + 133 , xx + 141 ) ; J [ 488 ] =
pm_math_Vector3_dot_ra ( xx + 136 , xx + 141 ) ; J [ 503 ] = xx [ 175 ] ; J [
504 ] = xx [ 179 ] ; J [ 505 ] = xx [ 183 ] ; J [ 529 ] = xx [ 176 ] ; J [
530 ] = xx [ 180 ] ; J [ 531 ] = xx [ 184 ] ; J [ 555 ] =
pm_math_Vector3_dot_ra ( xx + 170 , xx + 159 ) - pm_math_Vector3_dot_ra ( xx
+ 167 , xx + 185 ) ; J [ 556 ] = - pm_math_Vector3_dot_ra ( xx + 162 , xx +
185 ) ; J [ 557 ] = pm_math_Vector3_dot_ra ( xx + 188 , xx + 185 ) ; J [ 588
] = xx [ 211 ] ; J [ 589 ] = xx [ 215 ] ; J [ 590 ] = xx [ 219 ] ; J [ 614 ]
= xx [ 212 ] ; J [ 615 ] = xx [ 216 ] ; J [ 616 ] = xx [ 220 ] ; J [ 640 ] =
pm_math_Vector3_dot_ra ( xx + 206 , xx + 195 ) - pm_math_Vector3_dot_ra ( xx
+ 203 , xx + 221 ) ; J [ 641 ] = - pm_math_Vector3_dot_ra ( xx + 198 , xx +
221 ) ; J [ 642 ] = pm_math_Vector3_dot_ra ( xx + 224 , xx + 221 ) ; J [ 656
] = xx [ 228 ] ; J [ 657 ] = xx [ 18 ] ; J [ 658 ] = xx [ 236 ] ; J [ 659 ] =
xx [ 240 ] ; J [ 660 ] = xx [ 18 ] ; J [ 682 ] = xx [ 229 ] ; J [ 683 ] = xx
[ 19 ] ; J [ 684 ] = xx [ 237 ] ; J [ 685 ] = xx [ 241 ] ; J [ 686 ] = xx [
19 ] ; J [ 717 ] = xx [ 192 ] ; J [ 718 ] = xx [ 231 ] ; J [ 719 ] = xx [ 104
] ; J [ 743 ] = xx [ 193 ] ; J [ 744 ] = xx [ 232 ] ; J [ 745 ] = xx [ 105 ]
; J [ 760 ] = xx [ 145 ] ; J [ 761 ] = xx [ 243 ] ; J [ 786 ] = xx [ 146 ] ;
J [ 787 ] = xx [ 244 ] ; J [ 818 ] = xx [ 246 ] ; J [ 819 ] = xx [ 13 ] ; J [
820 ] = xx [ 246 ] ; J [ 844 ] = xx [ 247 ] ; J [ 845 ] = xx [ 14 ] ; J [ 846
] = xx [ 247 ] ; return 33 ; } static int isInKinematicSingularity_0 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_1 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_2 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_3 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_4 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_5 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_6 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_7 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_8 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_9 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_10 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_11 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } int Control_Bicopter_ae14a523_1_isInKinematicSingularity ( const void *
mech , const RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , const
int * modeVector , const double * motionData ) { ( void ) mech ; ( void )
rtdv ; ( void ) modeVector ; ( void ) motionData ; switch ( constraintIdx ) {
case 0 : return isInKinematicSingularity_0 ( rtdv , modeVector , motionData )
; case 1 : return isInKinematicSingularity_1 ( rtdv , modeVector , motionData
) ; case 2 : return isInKinematicSingularity_2 ( rtdv , modeVector ,
motionData ) ; case 3 : return isInKinematicSingularity_3 ( rtdv , modeVector
, motionData ) ; case 4 : return isInKinematicSingularity_4 ( rtdv ,
modeVector , motionData ) ; case 5 : return isInKinematicSingularity_5 ( rtdv
, modeVector , motionData ) ; case 6 : return isInKinematicSingularity_6 (
rtdv , modeVector , motionData ) ; case 7 : return isInKinematicSingularity_7
( rtdv , modeVector , motionData ) ; case 8 : return
isInKinematicSingularity_8 ( rtdv , modeVector , motionData ) ; case 9 :
return isInKinematicSingularity_9 ( rtdv , modeVector , motionData ) ; case
10 : return isInKinematicSingularity_10 ( rtdv , modeVector , motionData ) ;
case 11 : return isInKinematicSingularity_11 ( rtdv , modeVector , motionData
) ; } return 0 ; } PmfMessageId
Control_Bicopter_ae14a523_1_convertStateVector ( const void * asmMech , const
RuntimeDerivedValuesBundle * rtdv , const void * simMech , const double *
asmState , const int * asmModeVector , const int * simModeVector , double *
simState , void * neDiagMgr0 ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; NeuDiagnosticManager
* neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; ( void ) asmMech ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) simMech ; ( void ) asmModeVector ; (
void ) simModeVector ; ( void ) neDiagMgr ; simState [ 0 ] = asmState [ 0 ] ;
simState [ 1 ] = asmState [ 1 ] ; simState [ 2 ] = asmState [ 2 ] ; simState
[ 3 ] = asmState [ 3 ] ; simState [ 4 ] = asmState [ 4 ] ; simState [ 5 ] =
asmState [ 5 ] ; simState [ 6 ] = asmState [ 6 ] ; simState [ 7 ] = asmState
[ 7 ] ; simState [ 8 ] = asmState [ 8 ] ; simState [ 9 ] = asmState [ 9 ] ;
simState [ 10 ] = asmState [ 10 ] ; simState [ 11 ] = asmState [ 11 ] ;
simState [ 12 ] = asmState [ 12 ] ; simState [ 13 ] = asmState [ 13 ] ;
simState [ 14 ] = asmState [ 14 ] ; simState [ 15 ] = asmState [ 15 ] ;
simState [ 16 ] = asmState [ 16 ] ; simState [ 17 ] = asmState [ 17 ] ;
simState [ 18 ] = asmState [ 18 ] ; simState [ 19 ] = asmState [ 19 ] ;
simState [ 20 ] = asmState [ 20 ] ; simState [ 21 ] = asmState [ 21 ] ;
simState [ 22 ] = asmState [ 22 ] ; simState [ 23 ] = asmState [ 23 ] ;
simState [ 24 ] = asmState [ 24 ] ; simState [ 25 ] = asmState [ 25 ] ;
simState [ 26 ] = asmState [ 26 ] ; simState [ 27 ] = asmState [ 27 ] ;
simState [ 28 ] = asmState [ 28 ] ; simState [ 29 ] = asmState [ 29 ] ;
simState [ 30 ] = asmState [ 30 ] ; simState [ 31 ] = asmState [ 31 ] ;
simState [ 32 ] = asmState [ 32 ] ; simState [ 33 ] = asmState [ 33 ] ;
simState [ 34 ] = asmState [ 34 ] ; simState [ 35 ] = asmState [ 35 ] ;
simState [ 36 ] = asmState [ 36 ] ; simState [ 37 ] = asmState [ 37 ] ;
simState [ 38 ] = asmState [ 38 ] ; simState [ 39 ] = asmState [ 39 ] ;
simState [ 40 ] = asmState [ 40 ] ; simState [ 41 ] = asmState [ 41 ] ;
simState [ 42 ] = asmState [ 42 ] ; simState [ 43 ] = asmState [ 43 ] ;
simState [ 44 ] = asmState [ 44 ] ; simState [ 45 ] = asmState [ 45 ] ;
simState [ 46 ] = asmState [ 46 ] ; simState [ 47 ] = asmState [ 47 ] ;
simState [ 48 ] = asmState [ 48 ] ; simState [ 49 ] = asmState [ 49 ] ;
simState [ 50 ] = asmState [ 50 ] ; simState [ 51 ] = asmState [ 51 ] ;
simState [ 52 ] = asmState [ 52 ] ; simState [ 53 ] = asmState [ 53 ] ;
simState [ 54 ] = asmState [ 54 ] ; simState [ 55 ] = asmState [ 55 ] ;
simState [ 56 ] = asmState [ 56 ] ; simState [ 57 ] = asmState [ 57 ] ;
simState [ 58 ] = asmState [ 58 ] ; simState [ 59 ] = asmState [ 59 ] ;
simState [ 60 ] = asmState [ 60 ] ; simState [ 61 ] = asmState [ 61 ] ;
simState [ 62 ] = asmState [ 62 ] ; simState [ 63 ] = asmState [ 63 ] ;
simState [ 64 ] = asmState [ 64 ] ; simState [ 65 ] = asmState [ 65 ] ;
simState [ 66 ] = asmState [ 66 ] ; simState [ 67 ] = asmState [ 67 ] ;
simState [ 68 ] = asmState [ 68 ] ; return NULL ; } void
Control_Bicopter_ae14a523_1_constructStateVector ( const void * mech , const
double * solverState , const double * u , const double * uDot , double *
discreteState , double * fullState ) { ( void ) mech ; ( void ) discreteState
; fullState [ 0 ] = solverState [ 0 ] ; fullState [ 1 ] = solverState [ 1 ] ;
fullState [ 2 ] = solverState [ 2 ] ; fullState [ 3 ] = solverState [ 3 ] ;
fullState [ 4 ] = solverState [ 4 ] ; fullState [ 5 ] = solverState [ 5 ] ;
fullState [ 6 ] = solverState [ 6 ] ; fullState [ 7 ] = solverState [ 7 ] ;
fullState [ 8 ] = solverState [ 8 ] ; fullState [ 9 ] = solverState [ 9 ] ;
fullState [ 10 ] = solverState [ 10 ] ; fullState [ 11 ] = solverState [ 11 ]
; fullState [ 12 ] = solverState [ 12 ] ; fullState [ 13 ] = solverState [ 13
] ; fullState [ 14 ] = solverState [ 14 ] ; fullState [ 15 ] = solverState [
15 ] ; fullState [ 16 ] = solverState [ 16 ] ; fullState [ 17 ] = u [ 1 ] ;
fullState [ 18 ] = uDot [ 1 ] ; fullState [ 19 ] = solverState [ 17 ] ;
fullState [ 20 ] = solverState [ 18 ] ; fullState [ 21 ] = solverState [ 19 ]
; fullState [ 22 ] = solverState [ 20 ] ; fullState [ 23 ] = u [ 2 ] ;
fullState [ 24 ] = uDot [ 2 ] ; fullState [ 25 ] = solverState [ 21 ] ;
fullState [ 26 ] = solverState [ 22 ] ; fullState [ 27 ] = solverState [ 23 ]
; fullState [ 28 ] = solverState [ 24 ] ; fullState [ 29 ] = solverState [ 25
] ; fullState [ 30 ] = solverState [ 26 ] ; fullState [ 31 ] = u [ 0 ] ;
fullState [ 32 ] = uDot [ 0 ] ; fullState [ 33 ] = solverState [ 27 ] ;
fullState [ 34 ] = solverState [ 28 ] ; fullState [ 35 ] = solverState [ 29 ]
; fullState [ 36 ] = solverState [ 30 ] ; fullState [ 37 ] = u [ 3 ] ;
fullState [ 38 ] = uDot [ 3 ] ; fullState [ 39 ] = solverState [ 31 ] ;
fullState [ 40 ] = solverState [ 32 ] ; fullState [ 41 ] = solverState [ 33 ]
; fullState [ 42 ] = solverState [ 34 ] ; fullState [ 43 ] = solverState [ 35
] ; fullState [ 44 ] = solverState [ 36 ] ; fullState [ 45 ] = solverState [
37 ] ; fullState [ 46 ] = solverState [ 38 ] ; fullState [ 47 ] = solverState
[ 39 ] ; fullState [ 48 ] = solverState [ 40 ] ; fullState [ 49 ] =
solverState [ 41 ] ; fullState [ 50 ] = solverState [ 42 ] ; fullState [ 51 ]
= solverState [ 43 ] ; fullState [ 52 ] = solverState [ 44 ] ; fullState [ 53
] = solverState [ 45 ] ; fullState [ 54 ] = solverState [ 46 ] ; fullState [
55 ] = solverState [ 47 ] ; fullState [ 56 ] = solverState [ 48 ] ; fullState
[ 57 ] = solverState [ 49 ] ; fullState [ 58 ] = solverState [ 50 ] ;
fullState [ 59 ] = solverState [ 51 ] ; fullState [ 60 ] = solverState [ 52 ]
; fullState [ 61 ] = solverState [ 53 ] ; fullState [ 62 ] = solverState [ 54
] ; fullState [ 63 ] = solverState [ 55 ] ; fullState [ 64 ] = solverState [
56 ] ; fullState [ 65 ] = solverState [ 57 ] ; fullState [ 66 ] = solverState
[ 58 ] ; fullState [ 67 ] = solverState [ 59 ] ; fullState [ 68 ] =
solverState [ 60 ] ; } void
Control_Bicopter_ae14a523_1_extractSolverStateVector ( const void * mech ,
const double * fullState , double * solverState ) { ( void ) mech ;
solverState [ 0 ] = fullState [ 0 ] ; solverState [ 1 ] = fullState [ 1 ] ;
solverState [ 2 ] = fullState [ 2 ] ; solverState [ 3 ] = fullState [ 3 ] ;
solverState [ 4 ] = fullState [ 4 ] ; solverState [ 5 ] = fullState [ 5 ] ;
solverState [ 6 ] = fullState [ 6 ] ; solverState [ 7 ] = fullState [ 7 ] ;
solverState [ 8 ] = fullState [ 8 ] ; solverState [ 9 ] = fullState [ 9 ] ;
solverState [ 10 ] = fullState [ 10 ] ; solverState [ 11 ] = fullState [ 11 ]
; solverState [ 12 ] = fullState [ 12 ] ; solverState [ 13 ] = fullState [ 13
] ; solverState [ 14 ] = fullState [ 14 ] ; solverState [ 15 ] = fullState [
15 ] ; solverState [ 16 ] = fullState [ 16 ] ; solverState [ 17 ] = fullState
[ 19 ] ; solverState [ 18 ] = fullState [ 20 ] ; solverState [ 19 ] =
fullState [ 21 ] ; solverState [ 20 ] = fullState [ 22 ] ; solverState [ 21 ]
= fullState [ 25 ] ; solverState [ 22 ] = fullState [ 26 ] ; solverState [ 23
] = fullState [ 27 ] ; solverState [ 24 ] = fullState [ 28 ] ; solverState [
25 ] = fullState [ 29 ] ; solverState [ 26 ] = fullState [ 30 ] ; solverState
[ 27 ] = fullState [ 33 ] ; solverState [ 28 ] = fullState [ 34 ] ;
solverState [ 29 ] = fullState [ 35 ] ; solverState [ 30 ] = fullState [ 36 ]
; solverState [ 31 ] = fullState [ 39 ] ; solverState [ 32 ] = fullState [ 40
] ; solverState [ 33 ] = fullState [ 41 ] ; solverState [ 34 ] = fullState [
42 ] ; solverState [ 35 ] = fullState [ 43 ] ; solverState [ 36 ] = fullState
[ 44 ] ; solverState [ 37 ] = fullState [ 45 ] ; solverState [ 38 ] =
fullState [ 46 ] ; solverState [ 39 ] = fullState [ 47 ] ; solverState [ 40 ]
= fullState [ 48 ] ; solverState [ 41 ] = fullState [ 49 ] ; solverState [ 42
] = fullState [ 50 ] ; solverState [ 43 ] = fullState [ 51 ] ; solverState [
44 ] = fullState [ 52 ] ; solverState [ 45 ] = fullState [ 53 ] ; solverState
[ 46 ] = fullState [ 54 ] ; solverState [ 47 ] = fullState [ 55 ] ;
solverState [ 48 ] = fullState [ 56 ] ; solverState [ 49 ] = fullState [ 57 ]
; solverState [ 50 ] = fullState [ 58 ] ; solverState [ 51 ] = fullState [ 59
] ; solverState [ 52 ] = fullState [ 60 ] ; solverState [ 53 ] = fullState [
61 ] ; solverState [ 54 ] = fullState [ 62 ] ; solverState [ 55 ] = fullState
[ 63 ] ; solverState [ 56 ] = fullState [ 64 ] ; solverState [ 57 ] =
fullState [ 65 ] ; solverState [ 58 ] = fullState [ 66 ] ; solverState [ 59 ]
= fullState [ 67 ] ; solverState [ 60 ] = fullState [ 68 ] ; } int
Control_Bicopter_ae14a523_1_isPositionViolation ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; int ii [ 3 ] ;
double xx [ 107 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) eqnEnableFlags ; ( void ) modeVector ; xx [ 0 ] = - 0.9120630938981265 ; xx
[ 1 ] = - 0.1421693578567881 ; xx [ 2 ] = 0.3844095801838581 ; xx [ 3 ] = -
0.01257223522036471 ; xx [ 4 ] = 0.5 ; xx [ 5 ] = xx [ 4 ] * state [ 13 ] ;
xx [ 6 ] = sin ( xx [ 5 ] ) ; xx [ 7 ] = cos ( xx [ 5 ] ) ; xx [ 8 ] =
3.734971681880274e-4 * xx [ 6 ] ; xx [ 9 ] = 0.2610715860287157 * xx [ 6 ] ;
xx [ 10 ] = 0.9653193707101895 * xx [ 6 ] ; pm_math_Quaternion_compose_ra (
xx + 0 , xx + 7 , xx + 11 ) ; xx [ 0 ] = 2.908691710579205e-6 ; xx [ 1 ] =
2.868076938248717e-3 ; xx [ 2 ] = 4.94643927619723e-3 ;
pm_math_Quaternion_xform_ra ( xx + 11 , xx + 0 , xx + 5 ) ; xx [ 0 ] = -
8.970481754443746e-5 ; xx [ 1 ] = - 2.490130590085562e-3 ; xx [ 2 ] =
0.01561535429133853 ; pm_math_Quaternion_xform_ra ( xx + 11 , xx + 0 , xx + 8
) ; xx [ 0 ] = 0.7077023583577552 * state [ 14 ] + xx [ 8 ] +
0.07007512777103127 ; xx [ 1 ] = 6.588221211357465e-3 - (
9.268340058968039e-3 * state [ 14 ] + xx [ 9 ] ) ; xx [ 2 ] =
0.7064499061132523 * state [ 14 ] - xx [ 10 ] + 0.05970036331915229 ; xx [ 8
] = - ( 0.7077023583577557 * state [ 39 ] + xx [ 5 ] - xx [ 0 ] +
0.06327543407822646 ) ; xx [ 9 ] = 0.01459007920097803 - (
9.268340058967206e-3 * state [ 39 ] + xx [ 6 ] + xx [ 1 ] ) ; xx [ 10 ] =
0.7064499061132518 * state [ 39 ] - ( xx [ 7 ] + xx [ 2 ] ) +
0.05401888928708277 ; xx [ 15 ] = 0.9912919252698491 ; xx [ 16 ] = -
0.1316824810541687 ; xx [ 17 ] = 1.970490389246134e-4 ; xx [ 18 ] =
6.519118568758793e-5 ; pm_math_Quaternion_compose_ra ( xx + 11 , xx + 15 , xx
+ 19 ) ; xx [ 3 ] = 2.0 ; xx [ 5 ] = 1.0 ; xx [ 15 ] = ( xx [ 19 ] * xx [ 21
] + xx [ 20 ] * xx [ 22 ] ) * xx [ 3 ] ; xx [ 16 ] = xx [ 3 ] * ( xx [ 21 ] *
xx [ 22 ] - xx [ 19 ] * xx [ 20 ] ) ; xx [ 17 ] = xx [ 5 ] - ( xx [ 20 ] * xx
[ 20 ] + xx [ 21 ] * xx [ 21 ] ) * xx [ 3 ] ; xx [ 6 ] = 0.08324084851537972
+ 0.7044167893867059 * state [ 25 ] ; xx [ 7 ] = 4.5034434673372e-3 -
0.08618526586337766 * state [ 25 ] ; xx [ 18 ] = 0.7045346597422903 * state [
25 ] - 0.09384704203289078 ; xx [ 19 ] = 0.08324084851537329 - xx [ 6 ] ; xx
[ 20 ] = 4.503443467337508e-3 - xx [ 7 ] ; xx [ 21 ] = - (
0.09384704203289711 + xx [ 18 ] ) ; xx [ 22 ] = 0.7044167893867058 ; xx [ 23
] = - 0.08618526586337705 ; xx [ 24 ] = 0.7045346597422901 ; xx [ 25 ] =
0.4090279037421741 ; xx [ 26 ] = - 0.57698859548371 ; xx [ 27 ] = -
0.5768688874880377 ; xx [ 28 ] = - 0.4086595420277526 ; xx [ 29 ] = xx [ 4 ]
* state [ 17 ] ; xx [ 30 ] = sin ( xx [ 29 ] ) ; xx [ 31 ] = cos ( xx [ 29 ]
) ; xx [ 32 ] = - ( 0.9972896958011036 * xx [ 30 ] ) ; xx [ 33 ] =
8.852854494773021e-5 * xx [ 30 ] ; xx [ 34 ] = 0.07357482457770348 * xx [ 30
] ; pm_math_Quaternion_compose_ra ( xx + 25 , xx + 31 , xx + 35 ) ;
pm_math_Quaternion_compose_ra ( xx + 11 , xx + 35 , xx + 25 ) ; xx [ 29 ] = -
0.4813049877214196 ; xx [ 30 ] = - 0.5180593628877815 ; xx [ 31 ] = -
0.5180751960991345 ; xx [ 32 ] = - 0.4812048384063002 ;
pm_math_Quaternion_compose_ra ( xx + 25 , xx + 29 , xx + 39 ) ; xx [ 29 ] = (
xx [ 39 ] * xx [ 41 ] + xx [ 40 ] * xx [ 42 ] ) * xx [ 3 ] ; xx [ 30 ] = xx [
3 ] * ( xx [ 41 ] * xx [ 42 ] - xx [ 39 ] * xx [ 40 ] ) ; xx [ 31 ] = xx [ 5
] - ( xx [ 40 ] * xx [ 40 ] + xx [ 41 ] * xx [ 41 ] ) * xx [ 3 ] ; xx [ 32 ]
= 0.7044167893867053 ; xx [ 33 ] = - 0.08618526586337708 ; xx [ 34 ] =
0.7045346597422907 ; xx [ 39 ] = 0.05435570616770889 ; xx [ 40 ] =
0.996236014971251 ; xx [ 41 ] = 0.06752229025448525 ; xx [ 42 ] =
0.013582786684373 ; xx [ 43 ] = - 2.58589188160877e-6 ; xx [ 44 ] = -
0.01849903189296259 ; pm_math_Quaternion_xform_ra ( xx + 25 , xx + 42 , xx +
45 ) ; xx [ 42 ] = 9.122274404587457e-4 ; xx [ 43 ] = 1.459342321928613e-6 ;
xx [ 44 ] = - 0.01504973426522732 ; pm_math_Quaternion_xform_ra ( xx + 35 ,
xx + 42 , xx + 48 ) ; xx [ 35 ] = - ( 8.39853113781329e-5 + xx [ 48 ] ) ; xx
[ 36 ] = 1.467714669213244e-3 - xx [ 49 ] ; xx [ 37 ] = 0.03024959596740159 -
xx [ 50 ] ; pm_math_Quaternion_xform_ra ( xx + 11 , xx + 35 , xx + 42 ) ; xx
[ 35 ] = xx [ 45 ] + xx [ 42 ] - xx [ 0 ] + 0.09006429198674837 ; xx [ 36 ] =
xx [ 46 ] + xx [ 43 ] + xx [ 1 ] - 3.83018891385798e-3 ; xx [ 37 ] = xx [ 47
] + xx [ 44 ] + xx [ 2 ] - 0.07934515183560176 ; xx [ 0 ] = -
0.7077022283680728 ; xx [ 1 ] = - 9.268355963183032e-3 ; xx [ 2 ] =
0.7064500361247099 ; xx [ 42 ] = 9.71445146547012e-17 + 0.7077023583577412 *
state [ 43 ] ; xx [ 43 ] = 9.268340058974089e-3 * state [ 43 ] -
5.204170427930421e-17 ; xx [ 44 ] = 5.030698080332741e-17 -
0.7064499061132667 * state [ 43 ] ; xx [ 45 ] = - 0.7077023583577413 ; xx [
46 ] = - 9.2683400589742e-3 ; xx [ 47 ] = 0.7064499061132663 ; xx [ 48 ] = -
0.1316824519486606 ; xx [ 49 ] = 0.9912919062119712 ; xx [ 50 ] =
2.050212948449675e-4 ; xx [ 51 ] = - 2.156239646078782e-4 ; xx [ 38 ] = xx [
4 ] * state [ 27 ] ; xx [ 52 ] = sin ( xx [ 38 ] ) ; xx [ 53 ] = cos ( xx [
38 ] ) ; xx [ 54 ] = 3.734971681885031e-4 * xx [ 52 ] ; xx [ 55 ] =
0.261071586028717 * xx [ 52 ] ; xx [ 56 ] = 0.965319370710189 * xx [ 52 ] ;
pm_math_Quaternion_compose_ra ( xx + 48 , xx + 53 , xx + 57 ) ; xx [ 48 ] = -
9.045674901471507e-5 ; xx [ 49 ] = - 2.965461248705281e-3 ; xx [ 50 ] =
0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 57 , xx + 48 , xx +
51 ) ; xx [ 48 ] = - 8.9928915845163e-5 ; xx [ 49 ] = - 2.64677354170138e-3 ;
xx [ 50 ] = 0.01503616266891429 ; pm_math_Quaternion_xform_ra ( xx + 57 , xx
+ 48 , xx + 54 ) ; xx [ 61 ] = - 0.5181324366224915 ; xx [ 62 ] = -
0.4812371049201607 ; xx [ 63 ] = - 0.4812727257575918 ; xx [ 64 ] =
0.5180021142906276 ; xx [ 38 ] = xx [ 4 ] * state [ 31 ] ; xx [ 48 ] = sin (
xx [ 38 ] ) ; xx [ 65 ] = cos ( xx [ 38 ] ) ; xx [ 66 ] = - (
0.9972896958009458 * xx [ 48 ] ) ; xx [ 67 ] = 8.852854518109909e-5 * xx [ 48
] ; xx [ 68 ] = 0.0735748245798411 * xx [ 48 ] ;
pm_math_Quaternion_compose_ra ( xx + 61 , xx + 65 , xx + 69 ) ; xx [ 48 ] = -
0.01609995664951487 ; xx [ 49 ] = 2.933657308331041e-6 ; xx [ 50 ] = -
0.0137731566582556 ; pm_math_Quaternion_xform_ra ( xx + 69 , xx + 48 , xx +
61 ) ; xx [ 48 ] = 9.122274404458714e-4 ; xx [ 49 ] = 1.459342330053661e-6 ;
xx [ 50 ] = - 0.0150497342652287 ; pm_math_Quaternion_xform_ra ( xx + 69 , xx
+ 48 , xx + 64 ) ; xx [ 38 ] = 0.02364999999999497 - state [ 28 ] - xx [ 56 ]
; xx [ 48 ] = xx [ 51 ] - xx [ 54 ] - ( xx [ 61 ] - xx [ 64 ] ) +
4.589228319872062e-15 ; xx [ 49 ] = xx [ 52 ] - xx [ 55 ] - ( xx [ 62 ] - xx
[ 65 ] ) - 3.223116218364908e-15 ; xx [ 50 ] = xx [ 53 ] + xx [ 38 ] - ( xx [
63 ] - xx [ 66 ] ) - 7.889999999975156e-3 ; xx [ 61 ] = 0.5180593628885556 ;
xx [ 62 ] = - 0.4813049877207372 ; xx [ 63 ] = - 0.4812048384058716 ; xx [ 64
] = 0.518075196099393 ; pm_math_Quaternion_compose_ra ( xx + 69 , xx + 61 ,
xx + 65 ) ; xx [ 51 ] = ( xx [ 65 ] * xx [ 67 ] + xx [ 66 ] * xx [ 68 ] ) *
xx [ 3 ] ; xx [ 52 ] = xx [ 3 ] * ( xx [ 67 ] * xx [ 68 ] - xx [ 65 ] * xx [
66 ] ) ; xx [ 53 ] = xx [ 5 ] - ( xx [ 66 ] * xx [ 66 ] + xx [ 67 ] * xx [ 67
] ) * xx [ 3 ] ; xx [ 61 ] = - 0.9229115780805051 ; xx [ 62 ] = -
0.02085503273058337 ; xx [ 63 ] = 0.3825441798635174 ; xx [ 64 ] =
0.03819996213750987 ; pm_math_Quaternion_compose_ra ( xx + 61 , xx + 57 , xx
+ 65 ) ; xx [ 56 ] = 2.908691710755093e-6 ; xx [ 57 ] = 2.868076938250087e-3
; xx [ 58 ] = 4.94643927619916e-3 ; pm_math_Quaternion_xform_ra ( xx + 65 ,
xx + 56 , xx + 73 ) ; xx [ 56 ] = 3.383773514455306e-7 - xx [ 54 ] ; xx [ 57
] = 4.111205245035695e-3 - xx [ 55 ] ; xx [ 58 ] = xx [ 38 ] ;
pm_math_Quaternion_xform_ra ( xx + 61 , xx + 56 , xx + 76 ) ; xx [ 54 ] =
0.06121823687361252 - ( 0.7077023583577554 * state [ 41 ] + xx [ 73 ] + xx [
76 ] + xx [ 6 ] ) ; xx [ 55 ] = 0.01623907589682638 - ( 9.268340058966987e-3
* state [ 41 ] + xx [ 74 ] + xx [ 77 ] + xx [ 7 ] ) ; xx [ 56 ] =
0.7064499061132521 * state [ 41 ] - ( xx [ 75 ] + xx [ 78 ] + xx [ 18 ] ) -
0.07052859367409821 ; xx [ 57 ] = 0.991291925269849 ; xx [ 58 ] = -
0.1316824810541695 ; xx [ 59 ] = 1.970490389249778e-4 ; xx [ 60 ] =
6.519118568753968e-5 ; pm_math_Quaternion_compose_ra ( xx + 65 , xx + 57 , xx
+ 61 ) ; xx [ 57 ] = ( xx [ 61 ] * xx [ 63 ] + xx [ 62 ] * xx [ 64 ] ) * xx [
3 ] ; xx [ 58 ] = xx [ 3 ] * ( xx [ 63 ] * xx [ 64 ] - xx [ 61 ] * xx [ 62 ]
) ; xx [ 59 ] = xx [ 5 ] - ( xx [ 62 ] * xx [ 62 ] + xx [ 63 ] * xx [ 63 ] )
* xx [ 3 ] ; xx [ 60 ] = 0.6254151447093548 ; xx [ 61 ] = -
0.6480628292580688 ; xx [ 62 ] = 0.3298467071707108 ; xx [ 63 ] = -
0.2829692843240122 ; xx [ 6 ] = xx [ 4 ] * state [ 23 ] ; xx [ 7 ] =
1.380846645360201e-7 ; xx [ 18 ] = sin ( xx [ 6 ] ) ; xx [ 38 ] =
0.9999999999981867 ; xx [ 73 ] = cos ( xx [ 6 ] ) ; xx [ 74 ] = - ( xx [ 7 ]
* xx [ 18 ] ) ; xx [ 75 ] = xx [ 38 ] * xx [ 18 ] ; xx [ 76 ] =
1.899414133310628e-6 * xx [ 18 ] ; pm_math_Quaternion_compose_ra ( xx + 60 ,
xx + 73 , xx + 77 ) ; xx [ 60 ] = - 3.865146760306702e-8 ; xx [ 61 ] = -
5.289307885790379e-3 ; xx [ 62 ] = - 3.071851003736677e-7 ;
pm_math_Quaternion_xform_ra ( xx + 77 , xx + 60 , xx + 73 ) ; xx [ 60 ] = -
3.975614490536003e-8 ; xx [ 61 ] = 2.710692114195099e-3 ; xx [ 62 ] = -
2.919897873389293e-7 ; pm_math_Quaternion_xform_ra ( xx + 77 , xx + 60 , xx +
81 ) ; xx [ 60 ] = 0.4716196941540828 ; xx [ 61 ] = - 0.5117381057633613 ; xx
[ 62 ] = 0.5267317275910312 ; xx [ 63 ] = - 0.4881113216723255 ; xx [ 6 ] =
xx [ 4 ] * state [ 19 ] ; xx [ 18 ] = sin ( xx [ 6 ] ) ; xx [ 76 ] = cos ( xx
[ 6 ] ) ; xx [ 77 ] = 3.05954793225871e-4 * xx [ 18 ] ; xx [ 78 ] = - (
0.9999868227746886 * xx [ 18 ] ) ; xx [ 79 ] = - ( 5.124516430675685e-3 * xx
[ 18 ] ) ; pm_math_Quaternion_compose_ra ( xx + 60 , xx + 76 , xx + 84 ) ; xx
[ 60 ] = 2.392300359455244e-7 ; xx [ 61 ] = 3.887917980054749e-3 ; xx [ 62 ]
= 2.025360777142851e-5 ; pm_math_Quaternion_xform_ra ( xx + 84 , xx + 60 , xx
+ 76 ) ; xx [ 60 ] = 8.625263905800012e-8 ; xx [ 61 ] = 4.387911391441809e-3
; xx [ 62 ] = 2.281586598679112e-5 ; pm_math_Quaternion_xform_ra ( xx + 84 ,
xx + 60 , xx + 88 ) ; xx [ 60 ] = xx [ 73 ] - xx [ 81 ] - ( xx [ 76 ] +
0.07357483435689677 * state [ 20 ] - xx [ 88 ] ) - 5.518112576770381e-4 ; xx
[ 61 ] = xx [ 74 ] - xx [ 82 ] - ( xx [ 77 ] + 1.128000373271654e-4 * state [
20 ] - xx [ 89 ] ) - 8.460002797213924e-7 ; xx [ 62 ] = xx [ 75 ] - xx [ 83 ]
- ( xx [ 78 ] + 0.9972896926297332 * state [ 20 ] - xx [ 90 ] ) -
7.47967269472322e-3 ; xx [ 73 ] = 0.4931954203595264 ; xx [ 74 ] = -
0.4905194435002385 ; xx [ 75 ] = - 0.5067824161638587 ; xx [ 76 ] = -
0.509235245786982 ; pm_math_Quaternion_compose_ra ( xx + 84 , xx + 73 , xx +
77 ) ; xx [ 73 ] = ( xx [ 77 ] * xx [ 79 ] + xx [ 78 ] * xx [ 80 ] ) * xx [ 3
] ; xx [ 74 ] = xx [ 3 ] * ( xx [ 79 ] * xx [ 80 ] - xx [ 77 ] * xx [ 78 ] )
; xx [ 75 ] = xx [ 5 ] - ( xx [ 78 ] * xx [ 78 ] + xx [ 79 ] * xx [ 79 ] ) *
xx [ 3 ] ; xx [ 76 ] = 0.6252280994013473 ; xx [ 77 ] = 0.6477664378100223 ;
xx [ 78 ] = 0.330373832435706 ; xx [ 79 ] = 0.2834459324238707 ; xx [ 6 ] =
xx [ 4 ] * state [ 37 ] ; xx [ 18 ] = sin ( xx [ 6 ] ) ; xx [ 80 ] = cos ( xx
[ 6 ] ) ; xx [ 81 ] = - ( xx [ 7 ] * xx [ 18 ] ) ; xx [ 82 ] = xx [ 38 ] * xx
[ 18 ] ; xx [ 83 ] = 1.899414131645294e-6 * xx [ 18 ] ;
pm_math_Quaternion_compose_ra ( xx + 76 , xx + 80 , xx + 88 ) ; xx [ 76 ] = -
3.975614476060287e-8 ; xx [ 77 ] = 2.710692114195085e-3 ; xx [ 78 ] = -
2.919897872271535e-7 ; pm_math_Quaternion_xform_ra ( xx + 88 , xx + 76 , xx +
79 ) ; xx [ 76 ] = - 3.968710232179073e-8 ; xx [ 77 ] = 2.210692114196348e-3
; xx [ 78 ] = - 2.929394946184722e-7 ; pm_math_Quaternion_xform_ra ( xx + 88
, xx + 76 , xx + 92 ) ; xx [ 88 ] = 0.4716196941536389 ; xx [ 89 ] = -
0.5117381057636369 ; xx [ 90 ] = 0.5267317275917935 ; xx [ 91 ] = -
0.4881113216716428 ; xx [ 6 ] = xx [ 4 ] * state [ 33 ] ; xx [ 4 ] = sin ( xx
[ 6 ] ) ; xx [ 95 ] = cos ( xx [ 6 ] ) ; xx [ 96 ] = 3.059547932236506e-4 *
xx [ 4 ] ; xx [ 97 ] = - ( 0.9999868227746889 * xx [ 4 ] ) ; xx [ 98 ] = - (
5.124516430674353e-3 * xx [ 4 ] ) ; pm_math_Quaternion_compose_ra ( xx + 88 ,
xx + 95 , xx + 99 ) ; xx [ 76 ] = 2.392300359480238e-7 ; xx [ 77 ] =
3.887917980054739e-3 ; xx [ 78 ] = 2.025360777142854e-5 ;
pm_math_Quaternion_xform_ra ( xx + 99 , xx + 76 , xx + 88 ) ; xx [ 76 ] =
8.62526392261675e-8 ; xx [ 77 ] = 4.387911391441729e-3 ; xx [ 78 ] =
2.28158659870934e-5 ; pm_math_Quaternion_xform_ra ( xx + 99 , xx + 76 , xx +
95 ) ; xx [ 76 ] = xx [ 79 ] - xx [ 92 ] - ( xx [ 88 ] + 0.07357483435904683
* state [ 34 ] - xx [ 95 ] ) ; xx [ 77 ] = xx [ 80 ] - xx [ 93 ] - ( xx [ 89
] + 1.128000365619997e-4 * state [ 34 ] - xx [ 96 ] ) ; xx [ 78 ] = xx [ 81 ]
- xx [ 94 ] - ( xx [ 90 ] + 0.9972896926295751 * state [ 34 ] - xx [ 97 ] ) ;
xx [ 79 ] = 0.493195420359526 ; xx [ 80 ] = - 0.4905194435002388 ; xx [ 81 ]
= - 0.5067824161638583 ; xx [ 82 ] = - 0.5092352457869823 ;
pm_math_Quaternion_compose_ra ( xx + 99 , xx + 79 , xx + 88 ) ; xx [ 79 ] = (
xx [ 88 ] * xx [ 90 ] + xx [ 89 ] * xx [ 91 ] ) * xx [ 3 ] ; xx [ 80 ] = xx [
3 ] * ( xx [ 90 ] * xx [ 91 ] - xx [ 88 ] * xx [ 89 ] ) ; xx [ 81 ] = xx [ 5
] - ( xx [ 89 ] * xx [ 89 ] + xx [ 90 ] * xx [ 90 ] ) * xx [ 3 ] ;
pm_math_Quaternion_compose_ra ( xx + 25 , xx + 84 , xx + 88 ) ; xx [ 25 ] =
0.8739852835857713 ; xx [ 26 ] = 2.165051543389225e-3 ; xx [ 27 ] =
0.4859456095725561 ; xx [ 28 ] = 1.378827208253894e-3 ;
pm_math_Quaternion_compose_ra ( xx + 88 , xx + 25 , xx + 82 ) ; xx [ 25 ] = (
xx [ 82 ] * xx [ 84 ] + xx [ 83 ] * xx [ 85 ] ) * xx [ 3 ] ; xx [ 26 ] = xx [
3 ] * ( xx [ 84 ] * xx [ 85 ] - xx [ 82 ] * xx [ 83 ] ) ; xx [ 27 ] = xx [ 5
] - ( xx [ 83 ] * xx [ 83 ] + xx [ 84 ] * xx [ 84 ] ) * xx [ 3 ] ; xx [ 82 ]
= 0.704416789386705 ; xx [ 83 ] = - 0.08618526586337713 ; xx [ 84 ] =
0.704534659742291 ; xx [ 85 ] = - 0.05435570616770888 ; xx [ 86 ] = -
0.9962360149712526 ; xx [ 87 ] = - 0.06752229025448532 ; xx [ 4 ] =
0.9999999602050232 ; pm_math_Quaternion_compose_ra ( xx + 69 , xx + 99 , xx +
88 ) ; xx [ 69 ] = - 1.115736180703704e-4 ; xx [ 70 ] = - 0.01615139904553786
; xx [ 71 ] = - 2.564403241205184e-3 ; xx [ 72 ] = 0.9998662629053028 ;
pm_math_Quaternion_compose_ra ( xx + 88 , xx + 69 , xx + 92 ) ; xx [ 6 ] = (
xx [ 92 ] * xx [ 94 ] + xx [ 93 ] * xx [ 95 ] ) * xx [ 3 ] ; xx [ 7 ] =
2.821169119818891e-4 ; xx [ 18 ] = xx [ 3 ] * ( xx [ 94 ] * xx [ 95 ] - xx [
92 ] * xx [ 93 ] ) ; xx [ 69 ] = 0.7940628178106506 ; xx [ 70 ] =
0.6078356671769065 ; xx [ 71 ] = 1.854318411243853e-4 ; xx [ 72 ] = -
9.323758217514975e-5 ; pm_math_Quaternion_compose_ra ( xx + 11 , xx + 69 , xx
+ 88 ) ; xx [ 11 ] = ( xx [ 88 ] * xx [ 90 ] + xx [ 89 ] * xx [ 91 ] ) * xx [
3 ] ; xx [ 12 ] = xx [ 3 ] * ( xx [ 90 ] * xx [ 91 ] - xx [ 88 ] * xx [ 89 ]
) ; xx [ 13 ] = xx [ 5 ] - ( xx [ 89 ] * xx [ 89 ] + xx [ 90 ] * xx [ 90 ] )
* xx [ 3 ] ; xx [ 69 ] = 0.7044166587907192 ; xx [ 70 ] = -
0.08618526757371031 ; xx [ 71 ] = 0.7045347901071795 ; xx [ 88 ] =
0.05435570616771113 ; xx [ 89 ] = 0.9962360149712512 ; xx [ 90 ] =
0.06752229025448353 ; xx [ 91 ] = 0.4299358422201465 ; xx [ 92 ] = -
0.5615531320886276 ; xx [ 93 ] = - 0.5614212742353903 ; xx [ 94 ] = -
0.4296736019955341 ; pm_math_Quaternion_compose_ra ( xx + 65 , xx + 91 , xx +
95 ) ; xx [ 63 ] = xx [ 3 ] * ( xx [ 96 ] * xx [ 97 ] - xx [ 95 ] * xx [ 98 ]
) ; xx [ 64 ] = xx [ 5 ] - ( xx [ 98 ] * xx [ 98 ] + xx [ 96 ] * xx [ 96 ] )
* xx [ 3 ] ; xx [ 65 ] = ( xx [ 95 ] * xx [ 96 ] + xx [ 97 ] * xx [ 98 ] ) *
xx [ 3 ] ; xx [ 91 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 8 , xx + 15 ) ) ;
xx [ 92 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 19 , xx + 22 ) ) ; xx [ 93 ]
= fabs ( pm_math_Vector3_dot_ra ( xx + 29 , xx + 32 ) ) ; xx [ 94 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 29 , xx + 39 ) ) ; xx [ 95 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 35 , xx + 0 ) ) ; xx [ 96 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 42 , xx + 45 ) ) ; xx [ 97 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 48 , xx + 51 ) ) ; xx [ 98 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 54 , xx + 57 ) ) ; xx [ 99 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 60 , xx + 73 ) ) ; xx [ 100 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 76 , xx + 79 ) ) ; xx [ 101 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 25 , xx + 82 ) ) ; xx [ 102 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 25 , xx + 85 ) ) ; xx [ 103 ] = fabs ( xx [ 4 ]
* xx [ 6 ] + xx [ 7 ] * xx [ 18 ] ) ; xx [ 104 ] = fabs ( xx [ 4 ] * xx [ 18
] - xx [ 7 ] * xx [ 6 ] ) ; xx [ 105 ] = fabs ( pm_math_Vector3_dot_ra ( xx +
11 , xx + 69 ) ) ; xx [ 106 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 88 , xx
+ 63 ) ) ; ii [ 0 ] = 91 ; { int ll ; for ( ll = 92 ; ll < 107 ; ++ ll ) if (
xx [ ll ] > xx [ ii [ 0 ] ] ) ii [ 0 ] = ll ; } ii [ 0 ] -= 91 ; xx [ 0 ] =
xx [ 91 + ( ii [ 0 ] ) ] ; xx [ 1 ] = xx [ 0 ] - 1.0e-9 ; if ( xx [ 1 ] < 0.0
) ii [ 0 ] = - 1 ; else if ( xx [ 1 ] > 0.0 ) ii [ 0 ] = + 1 ; else ii [ 0 ]
= 0 ; ii [ 1 ] = ii [ 0 ] ; if ( 0 > ii [ 1 ] ) ii [ 1 ] = 0 ; return ii [ 1
] ; } int Control_Bicopter_ae14a523_1_isVelocityViolation ( const void * mech
, const RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags ,
const double * state , const int * modeVector ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; int ii
[ 3 ] ; double xx [ 172 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ;
( void ) eqnEnableFlags ; ( void ) modeVector ; xx [ 0 ] = 0.7077023583577557
; xx [ 1 ] = - 0.9120630938981265 ; xx [ 2 ] = - 0.1421693578567881 ; xx [ 3
] = 0.3844095801838581 ; xx [ 4 ] = - 0.01257223522036471 ; xx [ 5 ] = 0.5 ;
xx [ 6 ] = xx [ 5 ] * state [ 13 ] ; xx [ 7 ] = 3.734971681880274e-4 ; xx [ 8
] = sin ( xx [ 6 ] ) ; xx [ 9 ] = 0.2610715860287157 ; xx [ 10 ] =
0.9653193707101895 ; xx [ 11 ] = cos ( xx [ 6 ] ) ; xx [ 12 ] = xx [ 7 ] * xx
[ 8 ] ; xx [ 13 ] = xx [ 9 ] * xx [ 8 ] ; xx [ 14 ] = xx [ 10 ] * xx [ 8 ] ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 11 , xx + 15 ) ; xx [ 1 ] = xx
[ 7 ] * state [ 15 ] ; xx [ 2 ] = xx [ 9 ] * state [ 15 ] ; xx [ 3 ] = xx [
10 ] * state [ 15 ] ; xx [ 6 ] = 2.908691710579205e-6 ; xx [ 7 ] =
2.868076938248717e-3 ; xx [ 8 ] = 4.94643927619723e-3 ;
pm_math_Vector3_cross_ra ( xx + 1 , xx + 6 , xx + 11 ) ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 11 , xx + 19 ) ; xx [ 11 ] = -
6.480496605447651e-3 ; xx [ 12 ] = 9.242608862973643e-5 ; xx [ 13 ] = -
2.248932226692751e-5 ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 11 , xx +
22 ) ; xx [ 4 ] = 0.7077023583577552 ; xx [ 9 ] = xx [ 22 ] * state [ 15 ] -
xx [ 4 ] * state [ 16 ] ; xx [ 11 ] = 9.268340058967206e-3 ; xx [ 12 ] =
9.268340058968039e-3 ; xx [ 13 ] = xx [ 23 ] * state [ 15 ] - xx [ 12 ] *
state [ 16 ] ; xx [ 14 ] = 0.7064499061132518 ; xx [ 22 ] =
0.7064499061132523 ; xx [ 23 ] = xx [ 24 ] * state [ 15 ] + xx [ 22 ] * state
[ 16 ] ; xx [ 24 ] = - ( xx [ 0 ] * state [ 40 ] + xx [ 19 ] + xx [ 9 ] ) ;
xx [ 25 ] = - ( xx [ 11 ] * state [ 40 ] + xx [ 20 ] + xx [ 13 ] ) ; xx [ 26
] = xx [ 14 ] * state [ 40 ] - ( xx [ 21 ] + xx [ 23 ] ) ; xx [ 27 ] =
0.9912919252698491 ; xx [ 28 ] = - 0.1316824810541687 ; xx [ 29 ] =
1.970490389246134e-4 ; xx [ 30 ] = 6.519118568758793e-5 ;
pm_math_Quaternion_compose_ra ( xx + 15 , xx + 27 , xx + 31 ) ; xx [ 19 ] =
2.0 ; xx [ 20 ] = 1.0 ; xx [ 27 ] = ( xx [ 31 ] * xx [ 33 ] + xx [ 32 ] * xx
[ 34 ] ) * xx [ 19 ] ; xx [ 28 ] = xx [ 19 ] * ( xx [ 33 ] * xx [ 34 ] - xx [
31 ] * xx [ 32 ] ) ; xx [ 29 ] = xx [ 20 ] - ( xx [ 32 ] * xx [ 32 ] + xx [
33 ] * xx [ 33 ] ) * xx [ 19 ] ; pm_math_Quaternion_xform_ra ( xx + 15 , xx +
6 , xx + 30 ) ; xx [ 6 ] = - 8.970481754443746e-5 ; xx [ 7 ] = -
2.490130590085562e-3 ; xx [ 8 ] = 0.01561535429133853 ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 6 , xx + 33 ) ; xx [ 6 ] =
6.799693692804809e-3 - ( xx [ 0 ] * state [ 39 ] + xx [ 30 ] - ( xx [ 4 ] *
state [ 14 ] + xx [ 33 ] ) ) ; xx [ 7 ] = 8.001857989620565e-3 - ( xx [ 11 ]
* state [ 39 ] + xx [ 31 ] - ( xx [ 12 ] * state [ 14 ] + xx [ 34 ] ) ) ; xx
[ 8 ] = xx [ 14 ] * state [ 39 ] - ( xx [ 32 ] + xx [ 22 ] * state [ 14 ] -
xx [ 35 ] ) - 5.681474032069526e-3 ; xx [ 30 ] = 3.734971681878978e-4 ; xx [
31 ] = 0.2610715860287156 ; xx [ 32 ] = xx [ 10 ] ; pm_math_Vector3_cross_ra
( xx + 1 , xx + 30 , xx + 10 ) ; pm_math_Quaternion_xform_ra ( xx + 15 , xx +
10 , xx + 30 ) ; xx [ 0 ] = 0.7044167893867059 ; xx [ 4 ] = xx [ 0 ] * state
[ 26 ] ; xx [ 10 ] = 0.08618526586337766 ; xx [ 11 ] = xx [ 10 ] * state [ 26
] ; xx [ 12 ] = 0.7045346597422903 ; xx [ 14 ] = xx [ 12 ] * state [ 26 ] ;
xx [ 33 ] = - xx [ 4 ] ; xx [ 34 ] = xx [ 11 ] ; xx [ 35 ] = - xx [ 14 ] ; xx
[ 36 ] = 0.7044167893867058 ; xx [ 37 ] = - 0.08618526586337705 ; xx [ 38 ] =
0.7045346597422901 ; xx [ 39 ] = 0.4090279037421741 ; xx [ 40 ] = -
0.57698859548371 ; xx [ 41 ] = - 0.5768688874880377 ; xx [ 42 ] = -
0.4086595420277526 ; xx [ 21 ] = xx [ 5 ] * state [ 17 ] ; xx [ 22 ] =
0.9972896958011036 ; xx [ 43 ] = sin ( xx [ 21 ] ) ; xx [ 44 ] =
8.852854494773021e-5 ; xx [ 45 ] = 0.07357482457770348 ; xx [ 46 ] = cos ( xx
[ 21 ] ) ; xx [ 47 ] = - ( xx [ 22 ] * xx [ 43 ] ) ; xx [ 48 ] = xx [ 44 ] *
xx [ 43 ] ; xx [ 49 ] = xx [ 45 ] * xx [ 43 ] ; pm_math_Quaternion_compose_ra
( xx + 39 , xx + 46 , xx + 50 ) ; pm_math_Quaternion_compose_ra ( xx + 15 ,
xx + 50 , xx + 39 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 50 , xx + 1 ,
xx + 46 ) ; xx [ 54 ] = xx [ 46 ] - xx [ 22 ] * state [ 18 ] ; xx [ 55 ] = xx
[ 47 ] + xx [ 44 ] * state [ 18 ] ; xx [ 56 ] = xx [ 48 ] + xx [ 45 ] * state
[ 18 ] ; xx [ 43 ] = xx [ 22 ] ; xx [ 44 ] = - 8.852854494761919e-5 ; xx [ 45
] = - 0.07357482457770126 ; pm_math_Vector3_cross_ra ( xx + 54 , xx + 43 , xx
+ 46 ) ; pm_math_Quaternion_xform_ra ( xx + 39 , xx + 46 , xx + 43 ) ; xx [
46 ] = 0.7044167893867053 ; xx [ 47 ] = - 0.08618526586337708 ; xx [ 48 ] =
0.7045346597422907 ; xx [ 57 ] = 0.05435570616770889 ; xx [ 58 ] =
0.996236014971251 ; xx [ 59 ] = 0.06752229025448525 ; xx [ 60 ] =
0.013582786684373 ; xx [ 61 ] = - 2.58589188160877e-6 ; xx [ 62 ] = -
0.01849903189296259 ; pm_math_Vector3_cross_ra ( xx + 54 , xx + 60 , xx + 63
) ; pm_math_Quaternion_xform_ra ( xx + 39 , xx + 63 , xx + 60 ) ; xx [ 63 ] =
1.439701931685288e-6 ; xx [ 64 ] = 0.01494182793334928 ; xx [ 65 ] =
1.536145228271068e-6 ; pm_math_Quaternion_xform_ra ( xx + 50 , xx + 63 , xx +
66 ) ; xx [ 63 ] = 9.122274404587457e-4 ; xx [ 64 ] = 1.459342321928613e-6 ;
xx [ 65 ] = - 0.01504973426522732 ; pm_math_Quaternion_xform_ra ( xx + 50 ,
xx + 63 , xx + 69 ) ; xx [ 49 ] = - ( 8.39853113781329e-5 + xx [ 69 ] ) ; xx
[ 50 ] = 1.467714669213244e-3 - xx [ 70 ] ; xx [ 51 ] = 0.03024959596740159 -
xx [ 71 ] ; pm_math_Vector3_cross_ra ( xx + 1 , xx + 49 , xx + 63 ) ; xx [ 49
] = xx [ 66 ] * state [ 18 ] + xx [ 63 ] ; xx [ 50 ] = xx [ 67 ] * state [ 18
] + xx [ 64 ] ; xx [ 51 ] = xx [ 68 ] * state [ 18 ] + xx [ 65 ] ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 49 , xx + 63 ) ; xx [ 49 ] = xx
[ 60 ] + xx [ 9 ] + xx [ 63 ] ; xx [ 50 ] = xx [ 61 ] + xx [ 13 ] + xx [ 64 ]
; xx [ 51 ] = xx [ 62 ] + xx [ 23 ] + xx [ 65 ] ; xx [ 21 ] = -
0.7077022283680728 ; xx [ 22 ] = - 9.268355963183032e-3 ; xx [ 23 ] =
0.7064500361247099 ; xx [ 60 ] = 0.7077023583577412 * state [ 44 ] ; xx [ 61
] = 9.268340058974089e-3 * state [ 44 ] ; xx [ 62 ] = - ( 0.7064499061132667
* state [ 44 ] ) ; xx [ 63 ] = - 0.7077023583577413 ; xx [ 64 ] = -
9.2683400589742e-3 ; xx [ 65 ] = 0.7064499061132663 ; xx [ 66 ] = -
0.1316824519486606 ; xx [ 67 ] = 0.9912919062119712 ; xx [ 68 ] =
2.050212948449675e-4 ; xx [ 69 ] = - 2.156239646078782e-4 ; xx [ 9 ] = xx [ 5
] * state [ 27 ] ; xx [ 13 ] = 3.734971681885031e-4 ; xx [ 52 ] = sin ( xx [
9 ] ) ; xx [ 53 ] = 0.261071586028717 ; xx [ 70 ] = 0.965319370710189 ; xx [
71 ] = cos ( xx [ 9 ] ) ; xx [ 72 ] = xx [ 13 ] * xx [ 52 ] ; xx [ 73 ] = xx
[ 53 ] * xx [ 52 ] ; xx [ 74 ] = xx [ 70 ] * xx [ 52 ] ;
pm_math_Quaternion_compose_ra ( xx + 66 , xx + 71 , xx + 75 ) ; xx [ 66 ] =
xx [ 13 ] * state [ 29 ] ; xx [ 67 ] = xx [ 53 ] * state [ 29 ] ; xx [ 68 ] =
xx [ 70 ] * state [ 29 ] ; xx [ 71 ] = - 9.045674901471507e-5 ; xx [ 72 ] = -
2.965461248705281e-3 ; xx [ 73 ] = 0.01377564771951815 ;
pm_math_Vector3_cross_ra ( xx + 66 , xx + 71 , xx + 79 ) ;
pm_math_Quaternion_xform_ra ( xx + 75 , xx + 79 , xx + 82 ) ; xx [ 79 ] = -
6.480496605446796e-3 ; xx [ 80 ] = 9.242608862956347e-5 ; xx [ 81 ] = -
2.2489322266878e-5 ; pm_math_Quaternion_xform_ra ( xx + 75 , xx + 79 , xx +
85 ) ; xx [ 9 ] = xx [ 85 ] * state [ 29 ] ; xx [ 88 ] = - 0.5181324366224915
; xx [ 89 ] = - 0.4812371049201607 ; xx [ 90 ] = - 0.4812727257575918 ; xx [
91 ] = 0.5180021142906276 ; xx [ 13 ] = xx [ 5 ] * state [ 31 ] ; xx [ 52 ] =
0.9972896958009458 ; xx [ 53 ] = sin ( xx [ 13 ] ) ; xx [ 69 ] =
8.852854518109909e-5 ; xx [ 74 ] = 0.0735748245798411 ; xx [ 92 ] = cos ( xx
[ 13 ] ) ; xx [ 93 ] = - ( xx [ 52 ] * xx [ 53 ] ) ; xx [ 94 ] = xx [ 69 ] *
xx [ 53 ] ; xx [ 95 ] = xx [ 74 ] * xx [ 53 ] ; pm_math_Quaternion_compose_ra
( xx + 88 , xx + 92 , xx + 96 ) ; xx [ 79 ] = - ( xx [ 52 ] * state [ 32 ] )
; xx [ 80 ] = xx [ 69 ] * state [ 32 ] ; xx [ 81 ] = xx [ 74 ] * state [ 32 ]
; xx [ 88 ] = - 0.01609995664951487 ; xx [ 89 ] = 2.933657308331041e-6 ; xx [
90 ] = - 0.0137731566582556 ; pm_math_Vector3_cross_ra ( xx + 79 , xx + 88 ,
xx + 91 ) ; pm_math_Quaternion_xform_ra ( xx + 96 , xx + 91 , xx + 100 ) ; xx
[ 91 ] = 1.439701935798469e-6 ; xx [ 92 ] = 0.01494182793334727 ; xx [ 93 ] =
1.53614523658561e-6 ; pm_math_Quaternion_xform_ra ( xx + 96 , xx + 91 , xx +
103 ) ; xx [ 13 ] = xx [ 86 ] * state [ 29 ] ; xx [ 52 ] = xx [ 87 ] * state
[ 29 ] - state [ 30 ] ; xx [ 85 ] = xx [ 82 ] + xx [ 9 ] - ( xx [ 100 ] + xx
[ 103 ] * state [ 32 ] ) ; xx [ 86 ] = xx [ 83 ] + xx [ 13 ] - ( xx [ 101 ] +
xx [ 104 ] * state [ 32 ] ) ; xx [ 87 ] = xx [ 84 ] + xx [ 52 ] - ( xx [ 102
] + xx [ 105 ] * state [ 32 ] ) ; xx [ 91 ] = 0.5180593628885556 ; xx [ 92 ]
= - 0.4813049877207372 ; xx [ 93 ] = - 0.4812048384058716 ; xx [ 94 ] =
0.518075196099393 ; pm_math_Quaternion_compose_ra ( xx + 96 , xx + 91 , xx +
100 ) ; xx [ 82 ] = ( xx [ 100 ] * xx [ 102 ] + xx [ 101 ] * xx [ 103 ] ) *
xx [ 19 ] ; xx [ 83 ] = xx [ 19 ] * ( xx [ 102 ] * xx [ 103 ] - xx [ 100 ] *
xx [ 101 ] ) ; xx [ 84 ] = xx [ 20 ] - ( xx [ 101 ] * xx [ 101 ] + xx [ 102 ]
* xx [ 102 ] ) * xx [ 19 ] ; pm_math_Quaternion_xform_ra ( xx + 75 , xx + 71
, xx + 91 ) ; xx [ 71 ] = - 8.9928915845163e-5 ; xx [ 72 ] = -
2.64677354170138e-3 ; xx [ 73 ] = 0.01503616266891429 ;
pm_math_Quaternion_xform_ra ( xx + 75 , xx + 71 , xx + 100 ) ;
pm_math_Quaternion_xform_ra ( xx + 96 , xx + 88 , xx + 71 ) ; xx [ 88 ] =
9.122274404458714e-4 ; xx [ 89 ] = 1.459342330053661e-6 ; xx [ 90 ] = -
0.0150497342652287 ; pm_math_Quaternion_xform_ra ( xx + 96 , xx + 88 , xx +
103 ) ; xx [ 53 ] = 0.02364999999999497 - state [ 28 ] - xx [ 102 ] ; xx [ 88
] = xx [ 91 ] - xx [ 100 ] - ( xx [ 71 ] - xx [ 103 ] ) +
4.589228319872062e-15 ; xx [ 89 ] = xx [ 92 ] - xx [ 101 ] - ( xx [ 72 ] - xx
[ 104 ] ) - 3.223116218364908e-15 ; xx [ 90 ] = xx [ 93 ] + xx [ 53 ] - ( xx
[ 73 ] - xx [ 105 ] ) - 7.889999999975156e-3 ; xx [ 71 ] = -
0.9972896958009461 ; xx [ 72 ] = 8.852854518104358e-5 ; xx [ 73 ] =
0.07357482457984132 ; pm_math_Vector3_cross_ra ( xx + 79 , xx + 71 , xx + 91
) ; pm_math_Quaternion_xform_ra ( xx + 96 , xx + 91 , xx + 71 ) ; xx [ 69 ] =
0.7077023583577554 ; xx [ 91 ] = - 0.9229115780805051 ; xx [ 92 ] = -
0.02085503273058337 ; xx [ 93 ] = 0.3825441798635174 ; xx [ 94 ] =
0.03819996213750987 ; pm_math_Quaternion_compose_ra ( xx + 91 , xx + 75 , xx
+ 102 ) ; xx [ 74 ] = 2.908691710755093e-6 ; xx [ 75 ] = 2.868076938250087e-3
; xx [ 76 ] = 4.94643927619916e-3 ; pm_math_Vector3_cross_ra ( xx + 66 , xx +
74 , xx + 106 ) ; pm_math_Quaternion_xform_ra ( xx + 102 , xx + 106 , xx +
109 ) ; xx [ 106 ] = xx [ 9 ] ; xx [ 107 ] = xx [ 13 ] ; xx [ 108 ] = xx [ 52
] ; pm_math_Quaternion_xform_ra ( xx + 91 , xx + 106 , xx + 112 ) ; xx [ 9 ]
= 9.268340058966987e-3 ; xx [ 13 ] = 0.7064499061132521 ; xx [ 106 ] = - ( xx
[ 69 ] * state [ 42 ] + xx [ 109 ] + xx [ 4 ] + xx [ 112 ] ) ; xx [ 107 ] = -
( xx [ 9 ] * state [ 42 ] + xx [ 110 ] + xx [ 113 ] - xx [ 11 ] ) ; xx [ 108
] = xx [ 13 ] * state [ 42 ] - ( xx [ 111 ] + xx [ 14 ] + xx [ 114 ] ) ; xx [
109 ] = 0.991291925269849 ; xx [ 110 ] = - 0.1316824810541695 ; xx [ 111 ] =
1.970490389249778e-4 ; xx [ 112 ] = 6.519118568753968e-5 ;
pm_math_Quaternion_compose_ra ( xx + 102 , xx + 109 , xx + 113 ) ; xx [ 109 ]
= ( xx [ 113 ] * xx [ 115 ] + xx [ 114 ] * xx [ 116 ] ) * xx [ 19 ] ; xx [
110 ] = xx [ 19 ] * ( xx [ 115 ] * xx [ 116 ] - xx [ 113 ] * xx [ 114 ] ) ;
xx [ 111 ] = xx [ 20 ] - ( xx [ 114 ] * xx [ 114 ] + xx [ 115 ] * xx [ 115 ]
) * xx [ 19 ] ; pm_math_Quaternion_xform_ra ( xx + 102 , xx + 74 , xx + 112 )
; xx [ 74 ] = 3.383773514455306e-7 - xx [ 100 ] ; xx [ 75 ] =
4.111205245035695e-3 - xx [ 101 ] ; xx [ 76 ] = xx [ 53 ] ;
pm_math_Quaternion_xform_ra ( xx + 91 , xx + 74 , xx + 115 ) ; xx [ 74 ] = -
( xx [ 69 ] * state [ 41 ] + xx [ 112 ] + xx [ 115 ] + xx [ 0 ] * state [ 25
] + 0.02202261164176719 ) ; xx [ 75 ] = 0.01173563242948918 - ( xx [ 9 ] *
state [ 41 ] + xx [ 113 ] + xx [ 116 ] - xx [ 10 ] * state [ 25 ] ) ; xx [ 76
] = xx [ 13 ] * state [ 41 ] - ( xx [ 114 ] + xx [ 117 ] + xx [ 12 ] * state
[ 25 ] ) + 0.02331844835879257 ; xx [ 9 ] = 3.734971681886328e-4 ; xx [ 10 ]
= 0.2610715860287173 ; xx [ 11 ] = xx [ 70 ] ; pm_math_Vector3_cross_ra ( xx
+ 66 , xx + 9 , xx + 12 ) ; pm_math_Quaternion_xform_ra ( xx + 102 , xx + 12
, xx + 9 ) ; xx [ 91 ] = 0.6254151447093548 ; xx [ 92 ] = -
0.6480628292580688 ; xx [ 93 ] = 0.3298467071707108 ; xx [ 94 ] = -
0.2829692843240122 ; xx [ 0 ] = xx [ 5 ] * state [ 23 ] ; xx [ 4 ] =
1.380846645360201e-7 ; xx [ 12 ] = sin ( xx [ 0 ] ) ; xx [ 13 ] =
0.9999999999981867 ; xx [ 14 ] = 1.899414133310628e-6 ; xx [ 112 ] = cos ( xx
[ 0 ] ) ; xx [ 113 ] = - ( xx [ 4 ] * xx [ 12 ] ) ; xx [ 114 ] = xx [ 13 ] *
xx [ 12 ] ; xx [ 115 ] = xx [ 14 ] * xx [ 12 ] ;
pm_math_Quaternion_compose_ra ( xx + 91 , xx + 112 , xx + 116 ) ; xx [ 91 ] =
- ( xx [ 4 ] * state [ 24 ] ) ; xx [ 92 ] = xx [ 13 ] * state [ 24 ] ; xx [
93 ] = xx [ 14 ] * state [ 24 ] ; xx [ 112 ] = - 3.865146760306702e-8 ; xx [
113 ] = - 5.289307885790379e-3 ; xx [ 114 ] = - 3.071851003736677e-7 ;
pm_math_Vector3_cross_ra ( xx + 91 , xx + 112 , xx + 120 ) ;
pm_math_Quaternion_xform_ra ( xx + 116 , xx + 120 , xx + 91 ) ; xx [ 120 ] =
2.971385142511557e-7 ; xx [ 121 ] = 1.158326953518261e-13 ; xx [ 122 ] = -
3.938183989403888e-8 ; pm_math_Quaternion_xform_ra ( xx + 116 , xx + 120 , xx
+ 123 ) ; xx [ 126 ] = 0.4716196941540828 ; xx [ 127 ] = - 0.5117381057633613
; xx [ 128 ] = 0.5267317275910312 ; xx [ 129 ] = - 0.4881113216723255 ; xx [
0 ] = xx [ 5 ] * state [ 19 ] ; xx [ 12 ] = 3.05954793225871e-4 ; xx [ 14 ] =
sin ( xx [ 0 ] ) ; xx [ 52 ] = 0.9999868227746886 ; xx [ 53 ] =
5.124516430675685e-3 ; xx [ 130 ] = cos ( xx [ 0 ] ) ; xx [ 131 ] = xx [ 12 ]
* xx [ 14 ] ; xx [ 132 ] = - ( xx [ 52 ] * xx [ 14 ] ) ; xx [ 133 ] = - ( xx
[ 53 ] * xx [ 14 ] ) ; pm_math_Quaternion_compose_ra ( xx + 126 , xx + 130 ,
xx + 134 ) ; xx [ 0 ] = xx [ 12 ] * state [ 21 ] ; xx [ 12 ] = xx [ 52 ] *
state [ 21 ] ; xx [ 14 ] = xx [ 53 ] * state [ 21 ] ; xx [ 120 ] = xx [ 0 ] ;
xx [ 121 ] = - xx [ 12 ] ; xx [ 122 ] = - xx [ 14 ] ; xx [ 126 ] =
2.392300359455244e-7 ; xx [ 127 ] = 3.887917980054749e-3 ; xx [ 128 ] =
2.025360777142851e-5 ; pm_math_Vector3_cross_ra ( xx + 120 , xx + 126 , xx +
129 ) ; pm_math_Quaternion_xform_ra ( xx + 134 , xx + 129 , xx + 138 ) ; xx [
129 ] = 3.296413151917778e-7 ; xx [ 130 ] = 7.422626626299722e-9 ; xx [ 131 ]
= - 1.428754024949564e-6 ; pm_math_Quaternion_xform_ra ( xx + 134 , xx + 129
, xx + 141 ) ; xx [ 52 ] = 0.07357483435689677 ; xx [ 69 ] =
1.128000373271654e-4 ; xx [ 70 ] = 0.9972896926297332 ; xx [ 129 ] = xx [ 91
] + xx [ 123 ] * state [ 24 ] - ( xx [ 138 ] + xx [ 141 ] * state [ 21 ] + xx
[ 52 ] * state [ 22 ] ) ; xx [ 130 ] = xx [ 92 ] + xx [ 124 ] * state [ 24 ]
- ( xx [ 139 ] + xx [ 142 ] * state [ 21 ] + xx [ 69 ] * state [ 22 ] ) ; xx
[ 131 ] = xx [ 93 ] + xx [ 125 ] * state [ 24 ] - ( xx [ 140 ] + xx [ 143 ] *
state [ 21 ] + xx [ 70 ] * state [ 22 ] ) ; xx [ 91 ] = 0.4931954203595264 ;
xx [ 92 ] = - 0.4905194435002385 ; xx [ 93 ] = - 0.5067824161638587 ; xx [ 94
] = - 0.509235245786982 ; pm_math_Quaternion_compose_ra ( xx + 134 , xx + 91
, xx + 138 ) ; xx [ 91 ] = ( xx [ 138 ] * xx [ 140 ] + xx [ 139 ] * xx [ 141
] ) * xx [ 19 ] ; xx [ 92 ] = xx [ 19 ] * ( xx [ 140 ] * xx [ 141 ] - xx [
138 ] * xx [ 139 ] ) ; xx [ 93 ] = xx [ 20 ] - ( xx [ 139 ] * xx [ 139 ] + xx
[ 140 ] * xx [ 140 ] ) * xx [ 19 ] ; pm_math_Quaternion_xform_ra ( xx + 116 ,
xx + 112 , xx + 123 ) ; xx [ 112 ] = - 3.975614490536003e-8 ; xx [ 113 ] =
2.710692114195099e-3 ; xx [ 114 ] = - 2.919897873389293e-7 ;
pm_math_Quaternion_xform_ra ( xx + 116 , xx + 112 , xx + 138 ) ;
pm_math_Quaternion_xform_ra ( xx + 134 , xx + 126 , xx + 112 ) ; xx [ 115 ] =
8.625263905800012e-8 ; xx [ 116 ] = 4.387911391441809e-3 ; xx [ 117 ] =
2.281586598679112e-5 ; pm_math_Quaternion_xform_ra ( xx + 134 , xx + 115 , xx
+ 126 ) ; xx [ 115 ] = xx [ 123 ] - xx [ 138 ] - ( xx [ 112 ] + xx [ 52 ] *
state [ 20 ] - xx [ 126 ] ) - 5.518112576770381e-4 ; xx [ 116 ] = xx [ 124 ]
- xx [ 139 ] - ( xx [ 113 ] + xx [ 69 ] * state [ 20 ] - xx [ 127 ] ) -
8.460002797213924e-7 ; xx [ 117 ] = xx [ 125 ] - xx [ 140 ] - ( xx [ 114 ] +
xx [ 70 ] * state [ 20 ] - xx [ 128 ] ) - 7.47967269472322e-3 ; xx [ 52 ] =
0.9999868227746884 ; xx [ 112 ] = - 3.059547932262596e-4 ; xx [ 113 ] = xx [
52 ] ; xx [ 114 ] = 5.124516430675907e-3 ; pm_math_Vector3_cross_ra ( xx +
120 , xx + 112 , xx + 123 ) ; pm_math_Quaternion_xform_ra ( xx + 134 , xx +
123 , xx + 112 ) ; xx [ 118 ] = 0.6252280994013473 ; xx [ 119 ] =
0.6477664378100223 ; xx [ 120 ] = 0.330373832435706 ; xx [ 121 ] =
0.2834459324238707 ; xx [ 69 ] = xx [ 5 ] * state [ 37 ] ; xx [ 70 ] = sin (
xx [ 69 ] ) ; xx [ 77 ] = 1.899414131645294e-6 ; xx [ 122 ] = cos ( xx [ 69 ]
) ; xx [ 123 ] = - ( xx [ 4 ] * xx [ 70 ] ) ; xx [ 124 ] = xx [ 13 ] * xx [
70 ] ; xx [ 125 ] = xx [ 77 ] * xx [ 70 ] ; pm_math_Quaternion_compose_ra (
xx + 118 , xx + 122 , xx + 138 ) ; xx [ 118 ] = - ( xx [ 4 ] * state [ 38 ] )
; xx [ 119 ] = xx [ 13 ] * state [ 38 ] ; xx [ 120 ] = xx [ 77 ] * state [ 38
] ; xx [ 121 ] = - 3.975614476060287e-8 ; xx [ 122 ] = 2.710692114195085e-3 ;
xx [ 123 ] = - 2.919897872271535e-7 ; pm_math_Vector3_cross_ra ( xx + 118 ,
xx + 121 , xx + 124 ) ; pm_math_Quaternion_xform_ra ( xx + 138 , xx + 124 ,
xx + 118 ) ; xx [ 124 ] = 2.971385144603623e-7 ; xx [ 125 ] =
1.158326948378051e-13 ; xx [ 126 ] = - 3.938183964273754e-8 ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 124 , xx + 142 ) ; xx [ 124 ] =
0.4716196941536389 ; xx [ 125 ] = - 0.5117381057636369 ; xx [ 126 ] =
0.5267317275917935 ; xx [ 127 ] = - 0.4881113216716428 ; xx [ 4 ] = xx [ 5 ]
* state [ 33 ] ; xx [ 5 ] = 3.059547932236506e-4 ; xx [ 13 ] = sin ( xx [ 4 ]
) ; xx [ 69 ] = 0.9999868227746889 ; xx [ 70 ] = 5.124516430674353e-3 ; xx [
145 ] = cos ( xx [ 4 ] ) ; xx [ 146 ] = xx [ 5 ] * xx [ 13 ] ; xx [ 147 ] = -
( xx [ 69 ] * xx [ 13 ] ) ; xx [ 148 ] = - ( xx [ 70 ] * xx [ 13 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 124 , xx + 145 , xx + 149 ) ; xx [ 4 ] =
xx [ 5 ] * state [ 35 ] ; xx [ 5 ] = xx [ 69 ] * state [ 35 ] ; xx [ 13 ] =
xx [ 70 ] * state [ 35 ] ; xx [ 124 ] = xx [ 4 ] ; xx [ 125 ] = - xx [ 5 ] ;
xx [ 126 ] = - xx [ 13 ] ; xx [ 145 ] = 2.392300359480238e-7 ; xx [ 146 ] =
3.887917980054739e-3 ; xx [ 147 ] = 2.025360777142854e-5 ;
pm_math_Vector3_cross_ra ( xx + 124 , xx + 145 , xx + 153 ) ;
pm_math_Quaternion_xform_ra ( xx + 149 , xx + 153 , xx + 156 ) ; xx [ 153 ] =
3.296413155003147e-7 ; xx [ 154 ] = 7.422626627203207e-9 ; xx [ 155 ] = -
1.428754025107962e-6 ; pm_math_Quaternion_xform_ra ( xx + 149 , xx + 153 , xx
+ 159 ) ; xx [ 69 ] = 0.07357483435904683 ; xx [ 70 ] = 1.128000365619997e-4
; xx [ 77 ] = 0.9972896926295751 ; xx [ 153 ] = xx [ 118 ] + xx [ 142 ] *
state [ 38 ] - ( xx [ 156 ] + xx [ 159 ] * state [ 35 ] + xx [ 69 ] * state [
36 ] ) ; xx [ 154 ] = xx [ 119 ] + xx [ 143 ] * state [ 38 ] - ( xx [ 157 ] +
xx [ 160 ] * state [ 35 ] + xx [ 70 ] * state [ 36 ] ) ; xx [ 155 ] = xx [
120 ] + xx [ 144 ] * state [ 38 ] - ( xx [ 158 ] + xx [ 161 ] * state [ 35 ]
+ xx [ 77 ] * state [ 36 ] ) ; xx [ 156 ] = 0.493195420359526 ; xx [ 157 ] =
- 0.4905194435002388 ; xx [ 158 ] = - 0.5067824161638583 ; xx [ 159 ] = -
0.5092352457869823 ; pm_math_Quaternion_compose_ra ( xx + 149 , xx + 156 , xx
+ 160 ) ; xx [ 118 ] = ( xx [ 160 ] * xx [ 162 ] + xx [ 161 ] * xx [ 163 ] )
* xx [ 19 ] ; xx [ 119 ] = xx [ 19 ] * ( xx [ 162 ] * xx [ 163 ] - xx [ 160 ]
* xx [ 161 ] ) ; xx [ 120 ] = xx [ 20 ] - ( xx [ 161 ] * xx [ 161 ] + xx [
162 ] * xx [ 162 ] ) * xx [ 19 ] ; pm_math_Quaternion_xform_ra ( xx + 138 ,
xx + 121 , xx + 142 ) ; xx [ 121 ] = - 3.968710232179073e-8 ; xx [ 122 ] =
2.210692114196348e-3 ; xx [ 123 ] = - 2.929394946184722e-7 ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 121 , xx + 156 ) ;
pm_math_Quaternion_xform_ra ( xx + 149 , xx + 145 , xx + 121 ) ; xx [ 138 ] =
8.62526392261675e-8 ; xx [ 139 ] = 4.387911391441729e-3 ; xx [ 140 ] =
2.28158659870934e-5 ; pm_math_Quaternion_xform_ra ( xx + 149 , xx + 138 , xx
+ 145 ) ; xx [ 138 ] = xx [ 142 ] - xx [ 156 ] - ( xx [ 121 ] + xx [ 69 ] *
state [ 34 ] - xx [ 145 ] ) ; xx [ 139 ] = xx [ 143 ] - xx [ 157 ] - ( xx [
122 ] + xx [ 70 ] * state [ 34 ] - xx [ 146 ] ) ; xx [ 140 ] = xx [ 144 ] -
xx [ 158 ] - ( xx [ 123 ] + xx [ 77 ] * state [ 34 ] - xx [ 147 ] ) ; xx [
121 ] = - 3.059547932248718e-4 ; xx [ 122 ] = xx [ 52 ] ; xx [ 123 ] = xx [
53 ] ; pm_math_Vector3_cross_ra ( xx + 124 , xx + 121 , xx + 141 ) ;
pm_math_Quaternion_xform_ra ( xx + 149 , xx + 141 , xx + 121 ) ;
pm_math_Quaternion_compose_ra ( xx + 39 , xx + 134 , xx + 124 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 134 , xx + 54 , xx + 39 ) ; xx [ 52
] = xx [ 39 ] + xx [ 0 ] ; xx [ 53 ] = xx [ 40 ] - xx [ 12 ] ; xx [ 54 ] = xx
[ 41 ] - xx [ 14 ] ; xx [ 39 ] = 0.8494245932430124 ; xx [ 40 ] = -
2.444376317833358e-3 ; xx [ 41 ] = 0.5277043541779427 ;
pm_math_Vector3_cross_ra ( xx + 52 , xx + 39 , xx + 132 ) ;
pm_math_Quaternion_xform_ra ( xx + 124 , xx + 132 , xx + 39 ) ; xx [ 52 ] =
0.704416789386705 ; xx [ 53 ] = - 0.08618526586337713 ; xx [ 54 ] =
0.704534659742291 ; xx [ 124 ] = - 0.05435570616770888 ; xx [ 125 ] = -
0.9962360149712526 ; xx [ 126 ] = - 0.06752229025448532 ; xx [ 0 ] =
0.9999999602050232 ; pm_math_Quaternion_compose_ra ( xx + 96 , xx + 149 , xx
+ 132 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 149 , xx + 79 , xx + 94 )
; xx [ 77 ] = xx [ 94 ] + xx [ 4 ] ; xx [ 78 ] = xx [ 95 ] - xx [ 5 ] ; xx [
79 ] = xx [ 96 ] - xx [ 13 ] ; xx [ 12 ] = - 0.03229790576921281 ; xx [ 13 ]
= - 5.131724710788964e-3 ; xx [ 14 ] = 0.9994651122897766 ;
pm_math_Vector3_cross_ra ( xx + 77 , xx + 12 , xx + 94 ) ;
pm_math_Quaternion_xform_ra ( xx + 132 , xx + 94 , xx + 12 ) ; xx [ 4 ] =
2.821169119818891e-4 ; xx [ 77 ] = 1.81142804615305e-4 ; xx [ 78 ] = -
0.9653194398670554 ; xx [ 79 ] = 0.2610715346452743 ;
pm_math_Vector3_cross_ra ( xx + 1 , xx + 77 , xx + 94 ) ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 94 , xx + 1 ) ; xx [ 14 ] =
0.7044166587907192 ; xx [ 15 ] = - 0.08618526757371031 ; xx [ 16 ] =
0.7045347901071795 ; xx [ 17 ] = 0.05435570616771113 ; xx [ 18 ] =
0.9962360149712512 ; xx [ 19 ] = 0.06752229025448353 ; xx [ 77 ] =
0.9999999138435713 ; xx [ 78 ] = 7.735117927198942e-5 ; xx [ 79 ] = -
4.078353164850834e-4 ; pm_math_Vector3_cross_ra ( xx + 66 , xx + 77 , xx + 94
) ; pm_math_Quaternion_xform_ra ( xx + 102 , xx + 94 , xx + 66 ) ; xx [ 156 ]
= fabs ( pm_math_Vector3_dot_ra ( xx + 24 , xx + 27 ) +
pm_math_Vector3_dot_ra ( xx + 6 , xx + 30 ) ) ; xx [ 157 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 33 , xx + 36 ) ) ; xx [ 158 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 43 , xx + 46 ) ) ; xx [ 159 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 43 , xx + 57 ) ) ; xx [ 160 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 49 , xx + 21 ) ) ; xx [ 161 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 60 , xx + 63 ) ) ; xx [ 162 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 85 , xx + 82 ) + pm_math_Vector3_dot_ra ( xx +
88 , xx + 71 ) ) ; xx [ 163 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 106 , xx
+ 109 ) + pm_math_Vector3_dot_ra ( xx + 74 , xx + 9 ) ) ; xx [ 164 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 129 , xx + 91 ) + pm_math_Vector3_dot_ra ( xx +
115 , xx + 112 ) ) ; xx [ 165 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 153 ,
xx + 118 ) + pm_math_Vector3_dot_ra ( xx + 138 , xx + 121 ) ) ; xx [ 166 ] =
fabs ( pm_math_Vector3_dot_ra ( xx + 39 , xx + 52 ) ) ; xx [ 167 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 39 , xx + 124 ) ) ; xx [ 168 ] = fabs ( xx [ 0
] * xx [ 12 ] + xx [ 4 ] * xx [ 13 ] ) ; xx [ 169 ] = fabs ( xx [ 0 ] * xx [
13 ] - xx [ 4 ] * xx [ 12 ] ) ; xx [ 170 ] = fabs ( pm_math_Vector3_dot_ra (
xx + 1 , xx + 14 ) ) ; xx [ 171 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 17 ,
xx + 66 ) ) ; ii [ 0 ] = 156 ; { int ll ; for ( ll = 157 ; ll < 172 ; ++ ll )
if ( xx [ ll ] > xx [ ii [ 0 ] ] ) ii [ 0 ] = ll ; } ii [ 0 ] -= 156 ; xx [ 0
] = xx [ 156 + ( ii [ 0 ] ) ] ; xx [ 1 ] = xx [ 0 ] - 1.0e-9 ; if ( xx [ 1 ]
< 0.0 ) ii [ 0 ] = - 1 ; else if ( xx [ 1 ] > 0.0 ) ii [ 0 ] = + 1 ; else ii
[ 0 ] = 0 ; ii [ 1 ] = ii [ 0 ] ; if ( 0 > ii [ 1 ] ) ii [ 1 ] = 0 ; return
ii [ 1 ] ; } PmfMessageId Control_Bicopter_ae14a523_1_projectStateSim ( const
void * mech , const RuntimeDerivedValuesBundle * rtdv , const int *
eqnEnableFlags , const int * modeVector , const double * input , double *
state , void * neDiagMgr0 ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; NeuDiagnosticManager
* neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; int ii [ 16 ] ; double
xx [ 1528 ] ; ( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void )
eqnEnableFlags ; ( void ) modeVector ; ( void ) input ; ( void ) neDiagMgr ;
xx [ 0 ] = 0.0 ; xx [ 1 ] = - 0.9120630938981265 ; xx [ 2 ] = -
0.1421693578567881 ; xx [ 3 ] = 0.3844095801838581 ; xx [ 4 ] = -
0.01257223522036471 ; xx [ 5 ] = 0.5 ; xx [ 6 ] = xx [ 5 ] * state [ 13 ] ;
xx [ 7 ] = 3.734971681880274e-4 ; xx [ 8 ] = sin ( xx [ 6 ] ) ; xx [ 9 ] =
0.2610715860287157 ; xx [ 10 ] = 0.9653193707101895 ; xx [ 11 ] = cos ( xx [
6 ] ) ; xx [ 12 ] = xx [ 7 ] * xx [ 8 ] ; xx [ 13 ] = xx [ 9 ] * xx [ 8 ] ;
xx [ 14 ] = xx [ 10 ] * xx [ 8 ] ; pm_math_Quaternion_compose_ra ( xx + 1 ,
xx + 11 , xx + 15 ) ; xx [ 6 ] = 1.251169307048272e-16 ; xx [ 11 ] =
1.110223024625157e-16 ; xx [ 12 ] = - xx [ 6 ] ; xx [ 13 ] =
3.38000027272356e-17 ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 11 , xx +
19 ) ; xx [ 8 ] = 0.7077023583577557 ; xx [ 14 ] = 0.06327543407822646 ; xx [
22 ] = 2.908691710579205e-6 ; xx [ 23 ] = 2.868076938248717e-3 ; xx [ 24 ] =
4.94643927619723e-3 ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 22 , xx +
25 ) ; xx [ 28 ] = 0.7077023583577552 ; xx [ 29 ] = - 8.970481754443746e-5 ;
xx [ 30 ] = - 2.490130590085562e-3 ; xx [ 31 ] = 0.01561535429133853 ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 29 , xx + 32 ) ; xx [ 35 ] =
0.07007512777103127 ; xx [ 36 ] = xx [ 28 ] * state [ 14 ] + xx [ 32 ] + xx [
35 ] ; xx [ 37 ] = 0.01459007920097803 ; xx [ 38 ] = 9.268340058967206e-3 ;
xx [ 39 ] = 6.588221211357465e-3 ; xx [ 40 ] = 9.268340058968039e-3 ; xx [ 41
] = xx [ 39 ] - ( xx [ 40 ] * state [ 14 ] + xx [ 33 ] ) ; xx [ 42 ] =
0.7064499061132518 ; xx [ 43 ] = 0.05401888928708277 ; xx [ 44 ] =
0.7064499061132523 ; xx [ 32 ] = 0.05970036331915229 ; xx [ 33 ] = xx [ 44 ]
* state [ 14 ] - xx [ 34 ] + xx [ 32 ] ; xx [ 45 ] = - ( xx [ 8 ] * state [
39 ] + xx [ 14 ] + xx [ 25 ] - xx [ 36 ] ) ; xx [ 46 ] = xx [ 37 ] - xx [ 38
] * state [ 39 ] - ( xx [ 26 ] + xx [ 41 ] ) ; xx [ 47 ] = xx [ 42 ] * state
[ 39 ] + xx [ 43 ] - ( xx [ 27 ] + xx [ 33 ] ) ; xx [ 25 ] = -
1.477235478147115e-3 ; xx [ 26 ] = 9.60335389372561e-7 ; xx [ 27 ] =
3.118418564317923e-7 ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 25 , xx +
48 ) ; xx [ 51 ] = - 6.480496605447651e-3 ; xx [ 52 ] = 9.242608862973643e-5
; xx [ 53 ] = - 2.248932226692751e-5 ; pm_math_Quaternion_xform_ra ( xx + 15
, xx + 51 , xx + 54 ) ; xx [ 57 ] = xx [ 48 ] + xx [ 54 ] ; xx [ 58 ] = xx [
49 ] + xx [ 55 ] ; xx [ 59 ] = xx [ 50 ] + xx [ 56 ] ; xx [ 60 ] =
0.9912919252698491 ; xx [ 61 ] = - 0.1316824810541687 ; xx [ 62 ] =
1.970490389246134e-4 ; xx [ 63 ] = 6.519118568758793e-5 ;
pm_math_Quaternion_compose_ra ( xx + 15 , xx + 60 , xx + 64 ) ; xx [ 34 ] =
2.0 ; xx [ 48 ] = 1.0 ; xx [ 68 ] = ( xx [ 64 ] * xx [ 66 ] + xx [ 65 ] * xx
[ 67 ] ) * xx [ 34 ] ; xx [ 69 ] = xx [ 34 ] * ( xx [ 66 ] * xx [ 67 ] - xx [
64 ] * xx [ 65 ] ) ; xx [ 70 ] = xx [ 48 ] - ( xx [ 65 ] * xx [ 65 ] + xx [
66 ] * xx [ 66 ] ) * xx [ 34 ] ; xx [ 64 ] = - xx [ 28 ] ; xx [ 65 ] = - xx [
40 ] ; xx [ 66 ] = xx [ 44 ] ; xx [ 71 ] = - xx [ 8 ] ; xx [ 72 ] = - xx [ 38
] ; xx [ 73 ] = xx [ 42 ] ; xx [ 49 ] = - 1.0 ; xx [ 74 ] =
0.4090279037421741 ; xx [ 75 ] = - 0.57698859548371 ; xx [ 76 ] = -
0.5768688874880377 ; xx [ 77 ] = - 0.4086595420277526 ; xx [ 50 ] = xx [ 5 ]
* state [ 17 ] ; xx [ 67 ] = 0.9972896958011036 ; xx [ 78 ] = sin ( xx [ 50 ]
) ; xx [ 79 ] = 8.852854494773021e-5 ; xx [ 80 ] = 0.07357482457770348 ; xx [
81 ] = cos ( xx [ 50 ] ) ; xx [ 82 ] = - ( xx [ 67 ] * xx [ 78 ] ) ; xx [ 83
] = xx [ 79 ] * xx [ 78 ] ; xx [ 84 ] = xx [ 80 ] * xx [ 78 ] ;
pm_math_Quaternion_compose_ra ( xx + 74 , xx + 81 , xx + 85 ) ;
pm_math_Quaternion_compose_ra ( xx + 15 , xx + 85 , xx + 81 ) ; xx [ 89 ] =
xx [ 7 ] ; xx [ 90 ] = xx [ 9 ] ; xx [ 91 ] = xx [ 10 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 85 , xx + 89 , xx + 92 ) ; xx [ 50
] = - 8.852854494761919e-5 ; xx [ 95 ] = xx [ 67 ] ; xx [ 96 ] = xx [ 50 ] ;
xx [ 97 ] = - 0.07357482457770126 ; pm_math_Vector3_cross_ra ( xx + 92 , xx +
95 , xx + 98 ) ; pm_math_Quaternion_xform_ra ( xx + 81 , xx + 98 , xx + 101 )
; xx [ 104 ] = 0.7044167893867053 ; xx [ 105 ] = - 0.08618526586337708 ; xx [
106 ] = 0.7045346597422907 ; xx [ 107 ] = 0.05435570616770889 ; xx [ 108 ] =
0.996236014971251 ; xx [ 109 ] = 0.06752229025448525 ; xx [ 110 ] =
0.013582786684373 ; xx [ 111 ] = - 2.58589188160877e-6 ; xx [ 112 ] = -
0.01849903189296259 ; pm_math_Vector3_cross_ra ( xx + 92 , xx + 110 , xx +
113 ) ; pm_math_Quaternion_xform_ra ( xx + 81 , xx + 113 , xx + 92 ) ; xx [
78 ] = 8.39853113781329e-5 ; xx [ 116 ] = 9.122274404587457e-4 ; xx [ 117 ] =
1.459342321928613e-6 ; xx [ 118 ] = - 0.01504973426522732 ;
pm_math_Quaternion_xform_ra ( xx + 85 , xx + 116 , xx + 119 ) ; xx [ 122 ] =
1.467714669213244e-3 ; xx [ 123 ] = 0.03024959596740159 ; xx [ 124 ] = - ( xx
[ 78 ] + xx [ 119 ] ) ; xx [ 125 ] = xx [ 122 ] - xx [ 120 ] ; xx [ 126 ] =
xx [ 123 ] - xx [ 121 ] ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 124 , xx
+ 119 ) ; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 119 , xx + 127 ) ; xx
[ 130 ] = xx [ 92 ] + xx [ 127 ] + xx [ 54 ] ; xx [ 131 ] = xx [ 93 ] + xx [
128 ] + xx [ 55 ] ; xx [ 132 ] = xx [ 94 ] + xx [ 129 ] + xx [ 56 ] ; xx [ 54
] = - 0.7077022283680728 ; xx [ 55 ] = - 9.268355963183032e-3 ; xx [ 56 ] =
0.7064500361247099 ; xx [ 92 ] = 0.9999999999999832 ; xx [ 93 ] = -
1.000000000000001 ; xx [ 133 ] = - 0.1316824519486606 ; xx [ 134 ] =
0.9912919062119712 ; xx [ 135 ] = 2.050212948449675e-4 ; xx [ 136 ] = -
2.156239646078782e-4 ; xx [ 94 ] = xx [ 5 ] * state [ 27 ] ; xx [ 127 ] =
3.734971681885031e-4 ; xx [ 128 ] = sin ( xx [ 94 ] ) ; xx [ 129 ] =
0.261071586028717 ; xx [ 137 ] = 0.965319370710189 ; xx [ 138 ] = cos ( xx [
94 ] ) ; xx [ 139 ] = xx [ 127 ] * xx [ 128 ] ; xx [ 140 ] = xx [ 129 ] * xx
[ 128 ] ; xx [ 141 ] = xx [ 137 ] * xx [ 128 ] ;
pm_math_Quaternion_compose_ra ( xx + 133 , xx + 138 , xx + 142 ) ; xx [ 138 ]
= 6.459047385173115e-3 ; xx [ 139 ] = - 9.24648174485767e-5 ; xx [ 140 ] =
2.250809555350908e-5 ; pm_math_Quaternion_xform_ra ( xx + 142 , xx + 138 , xx
+ 146 ) ; xx [ 149 ] = - 6.480496605446796e-3 ; xx [ 150 ] =
9.242608862956347e-5 ; xx [ 151 ] = - 2.2489322266878e-5 ;
pm_math_Quaternion_xform_ra ( xx + 142 , xx + 149 , xx + 152 ) ; xx [ 155 ] =
xx [ 146 ] + xx [ 152 ] ; xx [ 156 ] = xx [ 147 ] + xx [ 153 ] ; xx [ 157 ] =
xx [ 148 ] + xx [ 154 ] ; xx [ 158 ] = - 0.5181324366224915 ; xx [ 159 ] = -
0.4812371049201607 ; xx [ 160 ] = - 0.4812727257575918 ; xx [ 161 ] =
0.5180021142906276 ; xx [ 94 ] = xx [ 5 ] * state [ 31 ] ; xx [ 128 ] =
0.9972896958009458 ; xx [ 141 ] = sin ( xx [ 94 ] ) ; xx [ 146 ] =
8.852854518109909e-5 ; xx [ 147 ] = 0.0735748245798411 ; xx [ 162 ] = cos (
xx [ 94 ] ) ; xx [ 163 ] = - ( xx [ 128 ] * xx [ 141 ] ) ; xx [ 164 ] = xx [
146 ] * xx [ 141 ] ; xx [ 165 ] = xx [ 147 ] * xx [ 141 ] ;
pm_math_Quaternion_compose_ra ( xx + 158 , xx + 162 , xx + 166 ) ; xx [ 162 ]
= 0.5180593628885556 ; xx [ 163 ] = - 0.4813049877207372 ; xx [ 164 ] = -
0.4812048384058716 ; xx [ 165 ] = 0.518075196099393 ;
pm_math_Quaternion_compose_ra ( xx + 166 , xx + 162 , xx + 170 ) ; xx [ 94 ]
= ( xx [ 171 ] * xx [ 171 ] + xx [ 172 ] * xx [ 172 ] ) * xx [ 34 ] ; xx [
174 ] = ( xx [ 170 ] * xx [ 172 ] + xx [ 171 ] * xx [ 173 ] ) * xx [ 34 ] ;
xx [ 175 ] = xx [ 34 ] * ( xx [ 172 ] * xx [ 173 ] - xx [ 170 ] * xx [ 171 ]
) ; xx [ 176 ] = xx [ 48 ] - xx [ 94 ] ; xx [ 141 ] = xx [ 94 ] - xx [ 48 ] ;
xx [ 94 ] = 0.7044167893867059 ; xx [ 148 ] = 0.08618526586337766 ; xx [ 170
] = 0.7045346597422903 ; xx [ 171 ] = xx [ 94 ] ; xx [ 172 ] = - xx [ 148 ] ;
xx [ 173 ] = xx [ 170 ] ; xx [ 177 ] = 0.9229115780805051 ; xx [ 178 ] = -
0.02085503273058337 ; xx [ 179 ] = 0.3825441798635174 ; xx [ 180 ] =
0.03819996213750987 ; xx [ 181 ] = - xx [ 177 ] ; xx [ 182 ] = xx [ 178 ] ;
xx [ 183 ] = xx [ 179 ] ; xx [ 184 ] = xx [ 180 ] ;
pm_math_Quaternion_compose_ra ( xx + 181 , xx + 142 , xx + 185 ) ; xx [ 189 ]
= 0.991291925269849 ; xx [ 190 ] = - 0.1316824810541695 ; xx [ 191 ] =
1.970490389249778e-4 ; xx [ 192 ] = 6.519118568753968e-5 ;
pm_math_Quaternion_compose_ra ( xx + 185 , xx + 189 , xx + 193 ) ; xx [ 197 ]
= ( xx [ 193 ] * xx [ 195 ] + xx [ 194 ] * xx [ 196 ] ) * xx [ 34 ] ; xx [
198 ] = xx [ 34 ] * ( xx [ 195 ] * xx [ 196 ] - xx [ 193 ] * xx [ 194 ] ) ;
xx [ 199 ] = xx [ 48 ] - ( xx [ 194 ] * xx [ 194 ] + xx [ 195 ] * xx [ 195 ]
) * xx [ 34 ] ; xx [ 193 ] = - 2.775557561562891e-16 ; xx [ 194 ] = xx [ 6 ]
; xx [ 195 ] = - 3.373224009145526e-17 ; pm_math_Quaternion_xform_ra ( xx +
185 , xx + 193 , xx + 200 ) ; xx [ 6 ] = 0.06121823687361252 ; xx [ 196 ] =
0.7077023583577554 ; xx [ 203 ] = 2.908691710755093e-6 ; xx [ 204 ] =
2.868076938250087e-3 ; xx [ 205 ] = 4.94643927619916e-3 ;
pm_math_Quaternion_xform_ra ( xx + 185 , xx + 203 , xx + 206 ) ; xx [ 209 ] =
3.383773514455306e-7 ; xx [ 210 ] = - 8.9928915845163e-5 ; xx [ 211 ] = -
2.64677354170138e-3 ; xx [ 212 ] = 0.01503616266891429 ;
pm_math_Quaternion_xform_ra ( xx + 142 , xx + 210 , xx + 213 ) ; xx [ 216 ] =
4.111205245035695e-3 ; xx [ 217 ] = 0.02364999999999497 ; xx [ 218 ] = xx [
217 ] - state [ 28 ] - xx [ 215 ] ; xx [ 219 ] = xx [ 209 ] - xx [ 213 ] ; xx
[ 220 ] = xx [ 216 ] - xx [ 214 ] ; xx [ 221 ] = xx [ 218 ] ;
pm_math_Quaternion_xform_ra ( xx + 181 , xx + 219 , xx + 222 ) ; xx [ 215 ] =
0.08324084851537972 ; xx [ 219 ] = xx [ 215 ] + xx [ 94 ] * state [ 25 ] ; xx
[ 220 ] = 0.01623907589682638 ; xx [ 221 ] = 9.268340058966987e-3 ; xx [ 225
] = 4.5034434673372e-3 ; xx [ 226 ] = xx [ 225 ] - xx [ 148 ] * state [ 25 ]
; xx [ 227 ] = 0.7064499061132521 ; xx [ 228 ] = 0.07052859367409821 ; xx [
229 ] = 0.09384704203289078 ; xx [ 230 ] = xx [ 170 ] * state [ 25 ] - xx [
229 ] ; xx [ 231 ] = xx [ 6 ] - xx [ 196 ] * state [ 41 ] - ( xx [ 206 ] + xx
[ 222 ] + xx [ 219 ] ) ; xx [ 232 ] = xx [ 220 ] - xx [ 221 ] * state [ 41 ]
- ( xx [ 207 ] + xx [ 223 ] + xx [ 226 ] ) ; xx [ 233 ] = xx [ 227 ] * state
[ 41 ] - xx [ 228 ] - ( xx [ 208 ] + xx [ 224 ] + xx [ 230 ] ) ; xx [ 206 ] =
- 1.477235478147926e-3 ; xx [ 207 ] = 9.60335389539274e-7 ; xx [ 208 ] =
3.118418563877457e-7 ; pm_math_Quaternion_xform_ra ( xx + 185 , xx + 206 , xx
+ 222 ) ; pm_math_Quaternion_xform_ra ( xx + 181 , xx + 152 , xx + 234 ) ; xx
[ 152 ] = xx [ 222 ] + xx [ 234 ] ; xx [ 153 ] = xx [ 223 ] + xx [ 235 ] ; xx
[ 154 ] = xx [ 224 ] + xx [ 236 ] ; xx [ 222 ] = 0.7064500361247104 ; xx [
234 ] = 0.7077022283680723 ; xx [ 235 ] = 9.268355963184306e-3 ; xx [ 236 ] =
- xx [ 222 ] ; xx [ 237 ] = - xx [ 196 ] ; xx [ 238 ] = - xx [ 221 ] ; xx [
239 ] = xx [ 227 ] ; xx [ 240 ] = 0.4716196941540828 ; xx [ 241 ] = -
0.5117381057633613 ; xx [ 242 ] = 0.5267317275910312 ; xx [ 243 ] = -
0.4881113216723255 ; xx [ 223 ] = xx [ 5 ] * state [ 19 ] ; xx [ 224 ] =
3.05954793225871e-4 ; xx [ 244 ] = sin ( xx [ 223 ] ) ; xx [ 245 ] =
0.9999868227746886 ; xx [ 246 ] = 5.124516430675685e-3 ; xx [ 247 ] = cos (
xx [ 223 ] ) ; xx [ 248 ] = xx [ 224 ] * xx [ 244 ] ; xx [ 249 ] = - ( xx [
245 ] * xx [ 244 ] ) ; xx [ 250 ] = - ( xx [ 246 ] * xx [ 244 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 240 , xx + 247 , xx + 251 ) ; xx [ 247 ]
= - 2.237793284010081e-16 ; xx [ 248 ] = 1.923188306740889e-18 ; xx [ 249 ] =
- 3.886322687274291e-16 ; pm_math_Quaternion_xform_ra ( xx + 251 , xx + 247 ,
xx + 255 ) ; xx [ 258 ] = 0.6254151447093548 ; xx [ 259 ] = -
0.6480628292580688 ; xx [ 260 ] = 0.3298467071707108 ; xx [ 261 ] = -
0.2829692843240122 ; xx [ 223 ] = xx [ 5 ] * state [ 23 ] ; xx [ 244 ] =
1.380846645360201e-7 ; xx [ 250 ] = sin ( xx [ 223 ] ) ; xx [ 262 ] =
0.9999999999981867 ; xx [ 263 ] = 1.899414133310628e-6 ; xx [ 264 ] = cos (
xx [ 223 ] ) ; xx [ 265 ] = - ( xx [ 244 ] * xx [ 250 ] ) ; xx [ 266 ] = xx [
262 ] * xx [ 250 ] ; xx [ 267 ] = xx [ 263 ] * xx [ 250 ] ;
pm_math_Quaternion_compose_ra ( xx + 258 , xx + 264 , xx + 268 ) ; xx [ 264 ]
= - 3.865146760306702e-8 ; xx [ 265 ] = - 5.289307885790379e-3 ; xx [ 266 ] =
- 3.071851003736677e-7 ; pm_math_Quaternion_xform_ra ( xx + 268 , xx + 264 ,
xx + 272 ) ; xx [ 275 ] = - 3.975614490536003e-8 ; xx [ 276 ] =
2.710692114195099e-3 ; xx [ 277 ] = - 2.919897873389293e-7 ;
pm_math_Quaternion_xform_ra ( xx + 268 , xx + 275 , xx + 278 ) ; xx [ 223 ] =
xx [ 272 ] - xx [ 278 ] ; xx [ 250 ] = 9.000024890213373e-4 ; xx [ 267 ] = xx
[ 223 ] + xx [ 250 ] ; xx [ 268 ] = 2.392300359455244e-7 ; xx [ 269 ] =
3.887917980054749e-3 ; xx [ 270 ] = 2.025360777142851e-5 ;
pm_math_Quaternion_xform_ra ( xx + 251 , xx + 268 , xx + 281 ) ; xx [ 271 ] =
0.07357483435689677 ; xx [ 284 ] = 8.625263905800012e-8 ; xx [ 285 ] =
4.387911391441809e-3 ; xx [ 286 ] = 2.281586598679112e-5 ;
pm_math_Quaternion_xform_ra ( xx + 251 , xx + 284 , xx + 287 ) ; xx [ 290 ] =
1.451813746698375e-3 ; xx [ 291 ] = xx [ 273 ] - xx [ 279 ] ; xx [ 292 ] =
1.149808082944376e-6 ; xx [ 293 ] = xx [ 291 ] + xx [ 292 ] ; xx [ 294 ] =
1.128000373271654e-4 ; xx [ 295 ] = 1.995808362665768e-6 ; xx [ 272 ] = xx [
274 ] - xx [ 280 ] ; xx [ 273 ] = 0.01362905791904227 ; xx [ 274 ] = xx [ 272
] + xx [ 273 ] ; xx [ 278 ] = 0.9972896926297332 ; xx [ 279 ] =
0.02110873061376549 ; xx [ 296 ] = xx [ 267 ] - ( xx [ 281 ] + xx [ 271 ] *
state [ 20 ] - xx [ 287 ] + xx [ 290 ] ) ; xx [ 297 ] = xx [ 293 ] - ( xx [
282 ] + xx [ 294 ] * state [ 20 ] - xx [ 288 ] + xx [ 295 ] ) ; xx [ 298 ] =
xx [ 274 ] - ( xx [ 283 ] + xx [ 278 ] * state [ 20 ] - xx [ 289 ] + xx [ 279
] ) ; xx [ 280 ] = - 3.296413151655537e-7 ; xx [ 281 ] = -
7.422626627699278e-9 ; xx [ 282 ] = 1.428754025224236e-6 ;
pm_math_Quaternion_xform_ra ( xx + 251 , xx + 280 , xx + 287 ) ; xx [ 299 ] =
3.296413151917778e-7 ; xx [ 300 ] = 7.422626626299722e-9 ; xx [ 301 ] = -
1.428754024949564e-6 ; pm_math_Quaternion_xform_ra ( xx + 251 , xx + 299 , xx
+ 302 ) ; xx [ 305 ] = xx [ 287 ] + xx [ 302 ] ; xx [ 306 ] = xx [ 288 ] + xx
[ 303 ] ; xx [ 307 ] = xx [ 289 ] + xx [ 304 ] ; xx [ 308 ] =
0.4931954203595264 ; xx [ 309 ] = - 0.4905194435002385 ; xx [ 310 ] = -
0.5067824161638587 ; xx [ 311 ] = - 0.509235245786982 ;
pm_math_Quaternion_compose_ra ( xx + 251 , xx + 308 , xx + 312 ) ; xx [ 287 ]
= ( xx [ 312 ] * xx [ 314 ] + xx [ 313 ] * xx [ 315 ] ) * xx [ 34 ] ; xx [
288 ] = xx [ 34 ] * ( xx [ 314 ] * xx [ 315 ] - xx [ 312 ] * xx [ 313 ] ) ;
xx [ 289 ] = xx [ 48 ] - ( xx [ 313 ] * xx [ 313 ] + xx [ 314 ] * xx [ 314 ]
) * xx [ 34 ] ; xx [ 302 ] = xx [ 271 ] ; xx [ 303 ] = xx [ 294 ] ; xx [ 304
] = xx [ 278 ] ; xx [ 312 ] = 0.4716196941536389 ; xx [ 313 ] = -
0.5117381057636369 ; xx [ 314 ] = 0.5267317275917935 ; xx [ 315 ] = -
0.4881113216716428 ; xx [ 283 ] = xx [ 5 ] * state [ 33 ] ; xx [ 316 ] =
3.059547932236506e-4 ; xx [ 317 ] = sin ( xx [ 283 ] ) ; xx [ 318 ] =
0.9999868227746889 ; xx [ 319 ] = 5.124516430674353e-3 ; xx [ 320 ] = cos (
xx [ 283 ] ) ; xx [ 321 ] = xx [ 316 ] * xx [ 317 ] ; xx [ 322 ] = - ( xx [
318 ] * xx [ 317 ] ) ; xx [ 323 ] = - ( xx [ 319 ] * xx [ 317 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 312 , xx + 320 , xx + 324 ) ; xx [ 320 ]
= - 1.334869714764153e-15 ; xx [ 321 ] = 5.850668324922266e-18 ; xx [ 322 ] =
- 1.221407957413545e-15 ; pm_math_Quaternion_xform_ra ( xx + 324 , xx + 320 ,
xx + 328 ) ; xx [ 331 ] = 0.6252280994013473 ; xx [ 332 ] =
0.6477664378100223 ; xx [ 333 ] = 0.330373832435706 ; xx [ 334 ] =
0.2834459324238707 ; xx [ 283 ] = xx [ 5 ] * state [ 37 ] ; xx [ 317 ] = sin
( xx [ 283 ] ) ; xx [ 323 ] = 1.899414131645294e-6 ; xx [ 335 ] = cos ( xx [
283 ] ) ; xx [ 336 ] = - ( xx [ 244 ] * xx [ 317 ] ) ; xx [ 337 ] = xx [ 262
] * xx [ 317 ] ; xx [ 338 ] = xx [ 323 ] * xx [ 317 ] ;
pm_math_Quaternion_compose_ra ( xx + 331 , xx + 335 , xx + 339 ) ; xx [ 335 ]
= - 3.975614476060287e-8 ; xx [ 336 ] = 2.710692114195085e-3 ; xx [ 337 ] = -
2.919897872271535e-7 ; pm_math_Quaternion_xform_ra ( xx + 339 , xx + 335 , xx
+ 343 ) ; xx [ 346 ] = - 3.968710232179073e-8 ; xx [ 347 ] =
2.210692114196348e-3 ; xx [ 348 ] = - 2.929394946184722e-7 ;
pm_math_Quaternion_xform_ra ( xx + 339 , xx + 346 , xx + 349 ) ; xx [ 283 ] =
xx [ 343 ] - xx [ 349 ] ; xx [ 317 ] = 1.451813746738863e-3 ; xx [ 338 ] = xx
[ 283 ] + xx [ 317 ] ; xx [ 339 ] = 2.392300359480238e-7 ; xx [ 340 ] =
3.887917980054739e-3 ; xx [ 341 ] = 2.025360777142854e-5 ;
pm_math_Quaternion_xform_ra ( xx + 324 , xx + 339 , xx + 352 ) ; xx [ 342 ] =
0.07357483435904683 ; xx [ 355 ] = 8.62526392261675e-8 ; xx [ 356 ] =
4.387911391441729e-3 ; xx [ 357 ] = 2.28158659870934e-5 ;
pm_math_Quaternion_xform_ra ( xx + 324 , xx + 355 , xx + 358 ) ; xx [ 361 ] =
xx [ 344 ] - xx [ 350 ] ; xx [ 362 ] = 1.995808348478373e-6 ; xx [ 363 ] = xx
[ 361 ] + xx [ 362 ] ; xx [ 364 ] = 1.128000365619997e-4 ; xx [ 343 ] = xx [
345 ] - xx [ 351 ] ; xx [ 344 ] = 0.02110873061376147 ; xx [ 345 ] = xx [ 343
] + xx [ 344 ] ; xx [ 349 ] = 0.9972896926295751 ; xx [ 365 ] = xx [ 338 ] -
( xx [ 352 ] + xx [ 342 ] * state [ 34 ] - xx [ 358 ] + xx [ 317 ] ) ; xx [
366 ] = xx [ 363 ] - ( xx [ 353 ] + xx [ 364 ] * state [ 34 ] - xx [ 359 ] +
xx [ 362 ] ) ; xx [ 367 ] = xx [ 345 ] - ( xx [ 354 ] + xx [ 349 ] * state [
34 ] - xx [ 360 ] + xx [ 344 ] ) ; xx [ 350 ] = - 3.296413151708188e-7 ; xx [
351 ] = - 7.422626627666806e-9 ; xx [ 352 ] = 1.4287540252181e-6 ;
pm_math_Quaternion_xform_ra ( xx + 324 , xx + 350 , xx + 358 ) ; xx [ 368 ] =
3.296413155003147e-7 ; xx [ 369 ] = 7.422626627203207e-9 ; xx [ 370 ] = -
1.428754025107962e-6 ; pm_math_Quaternion_xform_ra ( xx + 324 , xx + 368 , xx
+ 371 ) ; xx [ 374 ] = xx [ 358 ] + xx [ 371 ] ; xx [ 375 ] = xx [ 359 ] + xx
[ 372 ] ; xx [ 376 ] = xx [ 360 ] + xx [ 373 ] ; xx [ 377 ] =
0.493195420359526 ; xx [ 378 ] = - 0.4905194435002388 ; xx [ 379 ] = -
0.5067824161638583 ; xx [ 380 ] = - 0.5092352457869823 ;
pm_math_Quaternion_compose_ra ( xx + 324 , xx + 377 , xx + 381 ) ; xx [ 358 ]
= ( xx [ 381 ] * xx [ 383 ] + xx [ 382 ] * xx [ 384 ] ) * xx [ 34 ] ; xx [
359 ] = xx [ 34 ] * ( xx [ 383 ] * xx [ 384 ] - xx [ 381 ] * xx [ 382 ] ) ;
xx [ 360 ] = xx [ 48 ] - ( xx [ 382 ] * xx [ 382 ] + xx [ 383 ] * xx [ 383 ]
) * xx [ 34 ] ; xx [ 371 ] = xx [ 342 ] ; xx [ 372 ] = xx [ 364 ] ; xx [ 373
] = xx [ 349 ] ; pm_math_Quaternion_compose_ra ( xx + 85 , xx + 251 , xx +
381 ) ; pm_math_Quaternion_compose_ra ( xx + 15 , xx + 381 , xx + 385 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 381 , xx + 89 , xx + 389 ) ; xx [
381 ] = 0.8494245932430124 ; xx [ 382 ] = - 2.444376317833358e-3 ; xx [ 383 ]
= 0.5277043541779427 ; pm_math_Vector3_cross_ra ( xx + 389 , xx + 381 , xx +
392 ) ; pm_math_Quaternion_xform_ra ( xx + 385 , xx + 392 , xx + 389 ) ; xx [
384 ] = 0.704416789386705 ; xx [ 385 ] = - 0.08618526586337713 ; xx [ 386 ] =
0.704534659742291 ; pm_math_Quaternion_compose_ra ( xx + 81 , xx + 251 , xx +
392 ) ; xx [ 251 ] = - 0.5277099267453734 ; xx [ 252 ] = -
4.514343961260733e-3 ; xx [ 253 ] = 0.8494126523151114 ;
pm_math_Quaternion_xform_ra ( xx + 392 , xx + 251 , xx + 396 ) ; xx [ 399 ] =
- 0.05435570616770888 ; xx [ 400 ] = - 0.9962360149712526 ; xx [ 401 ] = -
0.06752229025448532 ; xx [ 254 ] = 0.9999999602050232 ;
pm_math_Quaternion_compose_ra ( xx + 166 , xx + 324 , xx + 402 ) ; xx [ 324 ]
= - 0.9994782397203995 ; xx [ 325 ] = - 1.402799929741683e-4 ; xx [ 326 ] = -
0.03229905024820418 ; pm_math_Quaternion_xform_ra ( xx + 402 , xx + 324 , xx
+ 406 ) ; xx [ 327 ] = 2.821169119818891e-4 ; xx [ 408 ] = 0.9999999138435711
; xx [ 409 ] = 7.735117927541253e-5 ; xx [ 410 ] = - 4.078353164858054e-4 ;
pm_math_Quaternion_xform_ra ( xx + 15 , xx + 408 , xx + 411 ) ; xx [ 414 ] =
0.7044166587907192 ; xx [ 415 ] = - 0.08618526757371031 ; xx [ 416 ] =
0.7045347901071795 ; xx [ 353 ] = 0.9653194398670554 ; xx [ 417 ] = -
1.811428046118123e-4 ; xx [ 418 ] = xx [ 353 ] ; xx [ 419 ] = -
0.2610715346452751 ; pm_math_Quaternion_xform_ra ( xx + 185 , xx + 417 , xx +
420 ) ; xx [ 423 ] = 0.05435570616771113 ; xx [ 424 ] = 0.9962360149712512 ;
xx [ 425 ] = 0.06752229025448353 ; xx [ 426 ] = xx [ 0 ] ; xx [ 427 ] = xx [
0 ] ; xx [ 428 ] = xx [ 0 ] ; xx [ 429 ] = xx [ 0 ] ; xx [ 430 ] = xx [ 0 ] ;
xx [ 431 ] = xx [ 0 ] ; xx [ 432 ] = pm_math_Vector3_dot_ra ( xx + 19 , xx +
45 ) - pm_math_Vector3_dot_ra ( xx + 57 , xx + 68 ) ; xx [ 433 ] = -
pm_math_Vector3_dot_ra ( xx + 64 , xx + 68 ) ; xx [ 434 ] = xx [ 0 ] ; xx [
435 ] = xx [ 0 ] ; xx [ 436 ] = xx [ 0 ] ; xx [ 437 ] = xx [ 0 ] ; xx [ 438 ]
= xx [ 0 ] ; xx [ 439 ] = xx [ 0 ] ; xx [ 440 ] = xx [ 0 ] ; xx [ 441 ] =
pm_math_Vector3_dot_ra ( xx + 71 , xx + 68 ) ; xx [ 442 ] = xx [ 0 ] ; xx [
443 ] = xx [ 0 ] ; xx [ 444 ] = xx [ 0 ] ; xx [ 445 ] = xx [ 0 ] ; xx [ 446 ]
= xx [ 0 ] ; xx [ 447 ] = xx [ 0 ] ; xx [ 448 ] = xx [ 0 ] ; xx [ 449 ] = xx
[ 0 ] ; xx [ 450 ] = xx [ 0 ] ; xx [ 451 ] = xx [ 0 ] ; xx [ 452 ] = xx [ 0 ]
; xx [ 453 ] = xx [ 0 ] ; xx [ 454 ] = xx [ 0 ] ; xx [ 455 ] = xx [ 0 ] ; xx
[ 456 ] = xx [ 0 ] ; xx [ 457 ] = xx [ 0 ] ; xx [ 458 ] = xx [ 49 ] ; xx [
459 ] = xx [ 0 ] ; xx [ 460 ] = xx [ 0 ] ; xx [ 461 ] = xx [ 0 ] ; xx [ 462 ]
= xx [ 0 ] ; xx [ 463 ] = xx [ 0 ] ; xx [ 464 ] = xx [ 0 ] ; xx [ 465 ] = xx
[ 0 ] ; xx [ 466 ] = xx [ 0 ] ; xx [ 467 ] = xx [ 0 ] ; xx [ 468 ] = xx [ 0 ]
; xx [ 469 ] = xx [ 0 ] ; xx [ 470 ] = xx [ 0 ] ; xx [ 471 ] = xx [ 0 ] ; xx
[ 472 ] = xx [ 0 ] ; xx [ 473 ] = xx [ 0 ] ; xx [ 474 ] = xx [ 0 ] ; xx [ 475
] = xx [ 0 ] ; xx [ 476 ] = pm_math_Vector3_dot_ra ( xx + 101 , xx + 104 ) ;
xx [ 477 ] = xx [ 0 ] ; xx [ 478 ] = xx [ 0 ] ; xx [ 479 ] = xx [ 0 ] ; xx [
480 ] = xx [ 0 ] ; xx [ 481 ] = xx [ 0 ] ; xx [ 482 ] = xx [ 0 ] ; xx [ 483 ]
= xx [ 0 ] ; xx [ 484 ] = xx [ 0 ] ; xx [ 485 ] = xx [ 0 ] ; xx [ 486 ] = xx
[ 0 ] ; xx [ 487 ] = xx [ 0 ] ; xx [ 488 ] = xx [ 0 ] ; xx [ 489 ] = xx [ 0 ]
; xx [ 490 ] = xx [ 0 ] ; xx [ 491 ] = xx [ 0 ] ; xx [ 492 ] = xx [ 0 ] ; xx
[ 493 ] = xx [ 0 ] ; xx [ 494 ] = xx [ 0 ] ; xx [ 495 ] = xx [ 0 ] ; xx [ 496
] = xx [ 0 ] ; xx [ 497 ] = xx [ 0 ] ; xx [ 498 ] = pm_math_Vector3_dot_ra (
xx + 101 , xx + 107 ) ; xx [ 499 ] = xx [ 0 ] ; xx [ 500 ] = xx [ 0 ] ; xx [
501 ] = xx [ 0 ] ; xx [ 502 ] = xx [ 0 ] ; xx [ 503 ] = xx [ 0 ] ; xx [ 504 ]
= xx [ 0 ] ; xx [ 505 ] = xx [ 0 ] ; xx [ 506 ] = xx [ 0 ] ; xx [ 507 ] = xx
[ 0 ] ; xx [ 508 ] = xx [ 0 ] ; xx [ 509 ] = xx [ 0 ] ; xx [ 510 ] = xx [ 0 ]
; xx [ 511 ] = xx [ 0 ] ; xx [ 512 ] = xx [ 0 ] ; xx [ 513 ] = xx [ 0 ] ; xx
[ 514 ] = xx [ 0 ] ; xx [ 515 ] = xx [ 0 ] ; xx [ 516 ] = xx [ 0 ] ; xx [ 517
] = xx [ 0 ] ; xx [ 518 ] = xx [ 0 ] ; xx [ 519 ] = xx [ 0 ] ; xx [ 520 ] =
pm_math_Vector3_dot_ra ( xx + 130 , xx + 54 ) ; xx [ 521 ] = xx [ 92 ] ; xx [
522 ] = xx [ 0 ] ; xx [ 523 ] = xx [ 0 ] ; xx [ 524 ] = xx [ 0 ] ; xx [ 525 ]
= xx [ 0 ] ; xx [ 526 ] = xx [ 0 ] ; xx [ 527 ] = xx [ 0 ] ; xx [ 528 ] = xx
[ 0 ] ; xx [ 529 ] = xx [ 0 ] ; xx [ 530 ] = xx [ 0 ] ; xx [ 531 ] = xx [ 0 ]
; xx [ 532 ] = xx [ 0 ] ; xx [ 533 ] = xx [ 0 ] ; xx [ 534 ] = xx [ 0 ] ; xx
[ 535 ] = xx [ 0 ] ; xx [ 536 ] = xx [ 0 ] ; xx [ 537 ] = xx [ 0 ] ; xx [ 538
] = xx [ 0 ] ; xx [ 539 ] = xx [ 0 ] ; xx [ 540 ] = xx [ 0 ] ; xx [ 541 ] =
xx [ 0 ] ; xx [ 542 ] = xx [ 0 ] ; xx [ 543 ] = xx [ 0 ] ; xx [ 544 ] = xx [
0 ] ; xx [ 545 ] = xx [ 0 ] ; xx [ 546 ] = xx [ 0 ] ; xx [ 547 ] = xx [ 0 ] ;
xx [ 548 ] = xx [ 0 ] ; xx [ 549 ] = xx [ 0 ] ; xx [ 550 ] = xx [ 0 ] ; xx [
551 ] = xx [ 0 ] ; xx [ 552 ] = xx [ 0 ] ; xx [ 553 ] = xx [ 93 ] ; xx [ 554
] = xx [ 0 ] ; xx [ 555 ] = xx [ 0 ] ; xx [ 556 ] = xx [ 0 ] ; xx [ 557 ] =
xx [ 0 ] ; xx [ 558 ] = xx [ 0 ] ; xx [ 559 ] = xx [ 0 ] ; xx [ 560 ] = xx [
0 ] ; xx [ 561 ] = xx [ 0 ] ; xx [ 562 ] = xx [ 0 ] ; xx [ 563 ] = xx [ 0 ] ;
xx [ 564 ] = xx [ 0 ] ; xx [ 565 ] = xx [ 0 ] ; xx [ 566 ] = xx [ 0 ] ; xx [
567 ] = xx [ 0 ] ; xx [ 568 ] = xx [ 0 ] ; xx [ 569 ] =
pm_math_Vector3_dot_ra ( xx + 155 , xx + 174 ) ; xx [ 570 ] = xx [ 141 ] ; xx
[ 571 ] = xx [ 0 ] ; xx [ 572 ] = xx [ 0 ] ; xx [ 573 ] = xx [ 0 ] ; xx [ 574
] = xx [ 0 ] ; xx [ 575 ] = xx [ 0 ] ; xx [ 576 ] = xx [ 0 ] ; xx [ 577 ] =
xx [ 0 ] ; xx [ 578 ] = xx [ 0 ] ; xx [ 579 ] = xx [ 0 ] ; xx [ 580 ] = xx [
0 ] ; xx [ 581 ] = xx [ 0 ] ; xx [ 582 ] = xx [ 0 ] ; xx [ 583 ] = xx [ 0 ] ;
xx [ 584 ] = xx [ 0 ] ; xx [ 585 ] = xx [ 0 ] ; xx [ 586 ] = xx [ 0 ] ; xx [
587 ] = xx [ 0 ] ; xx [ 588 ] = xx [ 0 ] ; xx [ 589 ] = xx [ 0 ] ; xx [ 590 ]
= - pm_math_Vector3_dot_ra ( xx + 171 , xx + 197 ) ; xx [ 591 ] =
pm_math_Vector3_dot_ra ( xx + 200 , xx + 231 ) - pm_math_Vector3_dot_ra ( xx
+ 152 , xx + 197 ) ; xx [ 592 ] = - pm_math_Vector3_dot_ra ( xx + 234 , xx +
197 ) ; xx [ 593 ] = xx [ 0 ] ; xx [ 594 ] = xx [ 0 ] ; xx [ 595 ] = xx [ 0 ]
; xx [ 596 ] = pm_math_Vector3_dot_ra ( xx + 237 , xx + 197 ) ; xx [ 597 ] =
xx [ 0 ] ; xx [ 598 ] = xx [ 0 ] ; xx [ 599 ] = xx [ 0 ] ; xx [ 600 ] = xx [
0 ] ; xx [ 601 ] = xx [ 0 ] ; xx [ 602 ] = xx [ 0 ] ; xx [ 603 ] = xx [ 0 ] ;
xx [ 604 ] = xx [ 0 ] ; xx [ 605 ] = xx [ 0 ] ; xx [ 606 ] = xx [ 0 ] ; xx [
607 ] = xx [ 0 ] ; xx [ 608 ] = xx [ 0 ] ; xx [ 609 ] = xx [ 0 ] ; xx [ 610 ]
= pm_math_Vector3_dot_ra ( xx + 255 , xx + 296 ) - pm_math_Vector3_dot_ra (
xx + 305 , xx + 287 ) ; xx [ 611 ] = - pm_math_Vector3_dot_ra ( xx + 302 , xx
+ 287 ) ; xx [ 612 ] = xx [ 0 ] ; xx [ 613 ] = xx [ 0 ] ; xx [ 614 ] = xx [ 0
] ; xx [ 615 ] = xx [ 0 ] ; xx [ 616 ] = xx [ 0 ] ; xx [ 617 ] = xx [ 0 ] ;
xx [ 618 ] = xx [ 0 ] ; xx [ 619 ] = xx [ 0 ] ; xx [ 620 ] = xx [ 0 ] ; xx [
621 ] = xx [ 0 ] ; xx [ 622 ] = xx [ 0 ] ; xx [ 623 ] = xx [ 0 ] ; xx [ 624 ]
= xx [ 0 ] ; xx [ 625 ] = xx [ 0 ] ; xx [ 626 ] = xx [ 0 ] ; xx [ 627 ] = xx
[ 0 ] ; xx [ 628 ] = xx [ 0 ] ; xx [ 629 ] = xx [ 0 ] ; xx [ 630 ] = xx [ 0 ]
; xx [ 631 ] = xx [ 0 ] ; xx [ 632 ] = xx [ 0 ] ; xx [ 633 ] = xx [ 0 ] ; xx
[ 634 ] = xx [ 0 ] ; xx [ 635 ] = xx [ 0 ] ; xx [ 636 ] = xx [ 0 ] ; xx [ 637
] = pm_math_Vector3_dot_ra ( xx + 328 , xx + 365 ) - pm_math_Vector3_dot_ra (
xx + 374 , xx + 358 ) ; xx [ 638 ] = - pm_math_Vector3_dot_ra ( xx + 371 , xx
+ 358 ) ; xx [ 639 ] = xx [ 0 ] ; xx [ 640 ] = xx [ 0 ] ; xx [ 641 ] = xx [ 0
] ; xx [ 642 ] = xx [ 0 ] ; xx [ 643 ] = xx [ 0 ] ; xx [ 644 ] = xx [ 0 ] ;
xx [ 645 ] = xx [ 0 ] ; xx [ 646 ] = xx [ 0 ] ; xx [ 647 ] = xx [ 0 ] ; xx [
648 ] = xx [ 0 ] ; xx [ 649 ] = xx [ 0 ] ; xx [ 650 ] = xx [ 0 ] ; xx [ 651 ]
= xx [ 0 ] ; xx [ 652 ] = pm_math_Vector3_dot_ra ( xx + 389 , xx + 384 ) ; xx
[ 653 ] = xx [ 0 ] ; xx [ 654 ] = pm_math_Vector3_dot_ra ( xx + 396 , xx +
384 ) ; xx [ 655 ] = xx [ 0 ] ; xx [ 656 ] = xx [ 0 ] ; xx [ 657 ] = xx [ 0 ]
; xx [ 658 ] = xx [ 0 ] ; xx [ 659 ] = xx [ 0 ] ; xx [ 660 ] = xx [ 0 ] ; xx
[ 661 ] = xx [ 0 ] ; xx [ 662 ] = xx [ 0 ] ; xx [ 663 ] = xx [ 0 ] ; xx [ 664
] = xx [ 0 ] ; xx [ 665 ] = xx [ 0 ] ; xx [ 666 ] = xx [ 0 ] ; xx [ 667 ] =
xx [ 0 ] ; xx [ 668 ] = xx [ 0 ] ; xx [ 669 ] = xx [ 0 ] ; xx [ 670 ] = xx [
0 ] ; xx [ 671 ] = xx [ 0 ] ; xx [ 672 ] = xx [ 0 ] ; xx [ 673 ] = xx [ 0 ] ;
xx [ 674 ] = pm_math_Vector3_dot_ra ( xx + 389 , xx + 399 ) ; xx [ 675 ] = xx
[ 0 ] ; xx [ 676 ] = pm_math_Vector3_dot_ra ( xx + 396 , xx + 399 ) ; xx [
677 ] = xx [ 0 ] ; xx [ 678 ] = xx [ 0 ] ; xx [ 679 ] = xx [ 0 ] ; xx [ 680 ]
= xx [ 0 ] ; xx [ 681 ] = xx [ 0 ] ; xx [ 682 ] = xx [ 0 ] ; xx [ 683 ] = xx
[ 0 ] ; xx [ 684 ] = xx [ 0 ] ; xx [ 685 ] = xx [ 0 ] ; xx [ 686 ] = xx [ 0 ]
; xx [ 687 ] = xx [ 0 ] ; xx [ 688 ] = xx [ 0 ] ; xx [ 689 ] = xx [ 0 ] ; xx
[ 690 ] = xx [ 0 ] ; xx [ 691 ] = xx [ 0 ] ; xx [ 692 ] = xx [ 0 ] ; xx [ 693
] = xx [ 0 ] ; xx [ 694 ] = xx [ 0 ] ; xx [ 695 ] = xx [ 0 ] ; xx [ 696 ] =
xx [ 0 ] ; xx [ 697 ] = xx [ 0 ] ; xx [ 698 ] = xx [ 0 ] ; xx [ 699 ] = xx [
0 ] ; xx [ 700 ] = xx [ 0 ] ; xx [ 701 ] = xx [ 0 ] ; xx [ 702 ] = xx [ 0 ] ;
xx [ 703 ] = xx [ 254 ] * xx [ 406 ] + xx [ 327 ] * xx [ 407 ] ; xx [ 704 ] =
xx [ 0 ] ; xx [ 705 ] = xx [ 0 ] ; xx [ 706 ] = xx [ 0 ] ; xx [ 707 ] = xx [
0 ] ; xx [ 708 ] = xx [ 0 ] ; xx [ 709 ] = xx [ 0 ] ; xx [ 710 ] = xx [ 0 ] ;
xx [ 711 ] = xx [ 0 ] ; xx [ 712 ] = xx [ 0 ] ; xx [ 713 ] = xx [ 0 ] ; xx [
714 ] = xx [ 0 ] ; xx [ 715 ] = xx [ 0 ] ; xx [ 716 ] = xx [ 0 ] ; xx [ 717 ]
= xx [ 0 ] ; xx [ 718 ] = xx [ 0 ] ; xx [ 719 ] = xx [ 0 ] ; xx [ 720 ] = xx
[ 0 ] ; xx [ 721 ] = xx [ 0 ] ; xx [ 722 ] = xx [ 0 ] ; xx [ 723 ] = xx [ 0 ]
; xx [ 724 ] = xx [ 0 ] ; xx [ 725 ] = xx [ 254 ] * xx [ 407 ] - xx [ 327 ] *
xx [ 406 ] ; xx [ 726 ] = xx [ 0 ] ; xx [ 727 ] = xx [ 0 ] ; xx [ 728 ] = xx
[ 0 ] ; xx [ 729 ] = xx [ 0 ] ; xx [ 730 ] = xx [ 0 ] ; xx [ 731 ] = xx [ 0 ]
; xx [ 732 ] = xx [ 0 ] ; xx [ 733 ] = xx [ 0 ] ; xx [ 734 ] = xx [ 0 ] ; xx
[ 735 ] = xx [ 0 ] ; xx [ 736 ] = xx [ 0 ] ; xx [ 737 ] = xx [ 0 ] ; xx [ 738
] = xx [ 0 ] ; xx [ 739 ] = xx [ 0 ] ; xx [ 740 ] = pm_math_Vector3_dot_ra (
xx + 411 , xx + 414 ) ; xx [ 741 ] = xx [ 0 ] ; xx [ 742 ] = xx [ 0 ] ; xx [
743 ] = xx [ 0 ] ; xx [ 744 ] = xx [ 0 ] ; xx [ 745 ] = xx [ 0 ] ; xx [ 746 ]
= xx [ 0 ] ; xx [ 747 ] = xx [ 0 ] ; xx [ 748 ] = xx [ 0 ] ; xx [ 749 ] = xx
[ 0 ] ; xx [ 750 ] = xx [ 0 ] ; xx [ 751 ] = xx [ 0 ] ; xx [ 752 ] = xx [ 0 ]
; xx [ 753 ] = xx [ 0 ] ; xx [ 754 ] = xx [ 0 ] ; xx [ 755 ] = xx [ 0 ] ; xx
[ 756 ] = xx [ 0 ] ; xx [ 757 ] = xx [ 0 ] ; xx [ 758 ] = xx [ 0 ] ; xx [ 759
] = xx [ 0 ] ; xx [ 760 ] = xx [ 0 ] ; xx [ 761 ] = xx [ 0 ] ; xx [ 762 ] =
xx [ 0 ] ; xx [ 763 ] = xx [ 0 ] ; xx [ 764 ] = xx [ 0 ] ; xx [ 765 ] = xx [
0 ] ; xx [ 766 ] = xx [ 0 ] ; xx [ 767 ] = pm_math_Vector3_dot_ra ( xx + 420
, xx + 423 ) ; xx [ 768 ] = xx [ 0 ] ; xx [ 769 ] = xx [ 0 ] ; xx [ 770 ] =
xx [ 0 ] ; xx [ 771 ] = xx [ 0 ] ; xx [ 772 ] = xx [ 0 ] ; xx [ 773 ] = xx [
0 ] ; xx [ 774 ] = xx [ 0 ] ; xx [ 775 ] = xx [ 0 ] ; xx [ 776 ] = xx [ 0 ] ;
xx [ 777 ] = xx [ 0 ] ; xx [ 19 ] = 0.08324084851537329 ; xx [ 20 ] =
4.503443467337508e-3 ; xx [ 21 ] = 0.09384704203289711 ; xx [ 57 ] = xx [ 19
] - xx [ 219 ] ; xx [ 58 ] = xx [ 20 ] - xx [ 226 ] ; xx [ 59 ] = - ( xx [ 21
] + xx [ 230 ] ) ; xx [ 101 ] = 0.7044167893867058 ; xx [ 102 ] = -
0.08618526586337705 ; xx [ 103 ] = 0.7045346597422901 ; xx [ 152 ] = -
0.4813049877214196 ; xx [ 153 ] = - 0.5180593628877815 ; xx [ 154 ] = -
0.5180751960991345 ; xx [ 155 ] = - 0.4812048384063002 ;
pm_math_Quaternion_compose_ra ( xx + 81 , xx + 152 , xx + 387 ) ; xx [ 130 ]
= ( xx [ 387 ] * xx [ 389 ] + xx [ 388 ] * xx [ 390 ] ) * xx [ 34 ] ; xx [
131 ] = xx [ 34 ] * ( xx [ 389 ] * xx [ 390 ] - xx [ 387 ] * xx [ 388 ] ) ;
xx [ 132 ] = xx [ 48 ] - ( xx [ 388 ] * xx [ 388 ] + xx [ 389 ] * xx [ 389 ]
) * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 81 , xx + 110 , xx + 200 )
; pm_math_Quaternion_xform_ra ( xx + 15 , xx + 124 , xx + 81 ) ; xx [ 84 ] =
0.09006429198674837 ; xx [ 156 ] = 3.83018891385798e-3 ; xx [ 157 ] =
0.07934515183560176 ; xx [ 255 ] = xx [ 200 ] + xx [ 81 ] - xx [ 36 ] + xx [
84 ] ; xx [ 256 ] = xx [ 201 ] + xx [ 82 ] + xx [ 41 ] - xx [ 156 ] ; xx [
257 ] = xx [ 202 ] + xx [ 83 ] + xx [ 33 ] - xx [ 157 ] ; xx [ 33 ] =
9.71445146547012e-17 ; xx [ 36 ] = 0.7077023583577412 ; xx [ 41 ] =
9.268340058974089e-3 ; xx [ 81 ] = 5.204170427930421e-17 ; xx [ 82 ] =
5.030698080332741e-17 ; xx [ 83 ] = 0.7064499061132667 ; xx [ 200 ] = xx [ 33
] + xx [ 36 ] * state [ 43 ] ; xx [ 201 ] = xx [ 41 ] * state [ 43 ] - xx [
81 ] ; xx [ 202 ] = xx [ 82 ] - xx [ 83 ] * state [ 43 ] ; xx [ 219 ] = -
0.7077023583577413 ; xx [ 305 ] = xx [ 219 ] ; xx [ 306 ] = -
9.2683400589742e-3 ; xx [ 307 ] = 0.7064499061132663 ; xx [ 328 ] = -
9.045674901471507e-5 ; xx [ 329 ] = - 2.965461248705281e-3 ; xx [ 330 ] =
0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 142 , xx + 328 , xx
+ 374 ) ; xx [ 142 ] = - 0.01609995664951487 ; xx [ 143 ] =
2.933657308331041e-6 ; xx [ 144 ] = - 0.0137731566582556 ;
pm_math_Quaternion_xform_ra ( xx + 166 , xx + 142 , xx + 387 ) ; xx [ 396 ] =
9.122274404458714e-4 ; xx [ 397 ] = 1.459342330053661e-6 ; xx [ 398 ] = -
0.0150497342652287 ; pm_math_Quaternion_xform_ra ( xx + 166 , xx + 396 , xx +
411 ) ; xx [ 145 ] = xx [ 387 ] - xx [ 411 ] ; xx [ 226 ] =
4.589228319872062e-15 ; xx [ 230 ] = xx [ 388 ] - xx [ 412 ] ; xx [ 354 ] =
3.223116218364908e-15 ; xx [ 387 ] = xx [ 389 ] - xx [ 413 ] ; xx [ 388 ] =
7.889999999975156e-3 ; xx [ 389 ] = xx [ 374 ] - xx [ 213 ] - xx [ 145 ] + xx
[ 226 ] ; xx [ 390 ] = xx [ 375 ] - xx [ 214 ] - xx [ 230 ] - xx [ 354 ] ; xx
[ 391 ] = xx [ 376 ] + xx [ 218 ] - xx [ 387 ] - xx [ 388 ] ; xx [ 778 ] =
0.8739852835857713 ; xx [ 779 ] = 2.165051543389225e-3 ; xx [ 780 ] =
0.4859456095725561 ; xx [ 781 ] = 1.378827208253894e-3 ;
pm_math_Quaternion_compose_ra ( xx + 392 , xx + 778 , xx + 782 ) ; xx [ 374 ]
= ( xx [ 782 ] * xx [ 784 ] + xx [ 783 ] * xx [ 785 ] ) * xx [ 34 ] ; xx [
375 ] = xx [ 34 ] * ( xx [ 784 ] * xx [ 785 ] - xx [ 782 ] * xx [ 783 ] ) ;
xx [ 376 ] = xx [ 48 ] - ( xx [ 783 ] * xx [ 783 ] + xx [ 784 ] * xx [ 784 ]
) * xx [ 34 ] ; xx [ 392 ] = - 1.115736180703704e-4 ; xx [ 393 ] = -
0.01615139904553786 ; xx [ 394 ] = - 2.564403241205184e-3 ; xx [ 395 ] =
0.9998662629053028 ; pm_math_Quaternion_compose_ra ( xx + 402 , xx + 392 , xx
+ 782 ) ; xx [ 213 ] = ( xx [ 782 ] * xx [ 784 ] + xx [ 783 ] * xx [ 785 ] )
* xx [ 34 ] ; xx [ 214 ] = xx [ 34 ] * ( xx [ 784 ] * xx [ 785 ] - xx [ 782 ]
* xx [ 783 ] ) ; xx [ 402 ] = 0.7940628178106506 ; xx [ 403 ] =
0.6078356671769065 ; xx [ 404 ] = 1.854318411243853e-4 ; xx [ 405 ] = -
9.323758217514975e-5 ; pm_math_Quaternion_compose_ra ( xx + 15 , xx + 402 ,
xx + 782 ) ; xx [ 15 ] = ( xx [ 782 ] * xx [ 784 ] + xx [ 783 ] * xx [ 785 ]
) * xx [ 34 ] ; xx [ 16 ] = xx [ 34 ] * ( xx [ 784 ] * xx [ 785 ] - xx [ 782
] * xx [ 783 ] ) ; xx [ 17 ] = xx [ 48 ] - ( xx [ 783 ] * xx [ 783 ] + xx [
784 ] * xx [ 784 ] ) * xx [ 34 ] ; xx [ 782 ] = 0.4299358422201465 ; xx [ 783
] = - 0.5615531320886276 ; xx [ 784 ] = - 0.5614212742353903 ; xx [ 785 ] = -
0.4296736019955341 ; pm_math_Quaternion_compose_ra ( xx + 185 , xx + 782 , xx
+ 786 ) ; xx [ 185 ] = xx [ 34 ] * ( xx [ 787 ] * xx [ 788 ] - xx [ 786 ] *
xx [ 789 ] ) ; xx [ 186 ] = xx [ 48 ] - ( xx [ 789 ] * xx [ 789 ] + xx [ 787
] * xx [ 787 ] ) * xx [ 34 ] ; xx [ 187 ] = ( xx [ 786 ] * xx [ 787 ] + xx [
788 ] * xx [ 789 ] ) * xx [ 34 ] ; xx [ 786 ] = - pm_math_Vector3_dot_ra ( xx
+ 45 , xx + 68 ) ; xx [ 787 ] = - pm_math_Vector3_dot_ra ( xx + 57 , xx + 101
) ; xx [ 788 ] = - pm_math_Vector3_dot_ra ( xx + 130 , xx + 104 ) ; xx [ 789
] = - pm_math_Vector3_dot_ra ( xx + 130 , xx + 107 ) ; xx [ 790 ] = -
pm_math_Vector3_dot_ra ( xx + 255 , xx + 54 ) ; xx [ 791 ] = -
pm_math_Vector3_dot_ra ( xx + 200 , xx + 305 ) ; xx [ 792 ] = -
pm_math_Vector3_dot_ra ( xx + 389 , xx + 174 ) ; xx [ 793 ] = -
pm_math_Vector3_dot_ra ( xx + 231 , xx + 197 ) ; xx [ 794 ] = -
pm_math_Vector3_dot_ra ( xx + 296 , xx + 287 ) ; xx [ 795 ] = -
pm_math_Vector3_dot_ra ( xx + 365 , xx + 358 ) ; xx [ 796 ] = -
pm_math_Vector3_dot_ra ( xx + 374 , xx + 384 ) ; xx [ 797 ] = -
pm_math_Vector3_dot_ra ( xx + 374 , xx + 399 ) ; xx [ 798 ] = - ( xx [ 254 ]
* xx [ 213 ] + xx [ 327 ] * xx [ 214 ] ) ; xx [ 799 ] = - ( xx [ 254 ] * xx [
214 ] - xx [ 327 ] * xx [ 213 ] ) ; xx [ 800 ] = - pm_math_Vector3_dot_ra (
xx + 15 , xx + 414 ) ; xx [ 801 ] = - pm_math_Vector3_dot_ra ( xx + 423 , xx
+ 185 ) ; xx [ 15 ] = 1.0e-8 ; memcpy ( xx + 824 , xx + 426 , 352 * sizeof (
double ) ) ; factorAndSolveWide ( 16 , 22 , xx + 824 , xx + 1176 , xx + 1192
, ii + 0 , xx + 786 , xx [ 15 ] , xx + 802 ) ; xx [ 16 ] = state [ 13 ] + xx
[ 808 ] ; xx [ 17 ] = xx [ 16 ] * xx [ 5 ] ; xx [ 18 ] = sin ( xx [ 17 ] ) ;
xx [ 185 ] = cos ( xx [ 17 ] ) ; xx [ 186 ] = xx [ 7 ] * xx [ 18 ] ; xx [ 187
] = xx [ 9 ] * xx [ 18 ] ; xx [ 188 ] = xx [ 10 ] * xx [ 18 ] ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 185 , xx + 197 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 11 , xx + 45 ) ; xx [ 17 ] =
state [ 39 ] + xx [ 817 ] ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 22
, xx + 57 ) ; xx [ 18 ] = state [ 14 ] + xx [ 809 ] ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 29 , xx + 68 ) ; xx [ 130 ] =
xx [ 18 ] * xx [ 28 ] + xx [ 68 ] + xx [ 35 ] ; xx [ 131 ] = xx [ 39 ] - ( xx
[ 18 ] * xx [ 40 ] + xx [ 69 ] ) ; xx [ 68 ] = xx [ 18 ] * xx [ 44 ] - xx [
70 ] + xx [ 32 ] ; xx [ 185 ] = - ( xx [ 17 ] * xx [ 8 ] + xx [ 14 ] + xx [
57 ] - xx [ 130 ] ) ; xx [ 186 ] = xx [ 37 ] - xx [ 17 ] * xx [ 38 ] - ( xx [
58 ] + xx [ 131 ] ) ; xx [ 187 ] = xx [ 17 ] * xx [ 42 ] + xx [ 43 ] - ( xx [
59 ] + xx [ 68 ] ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 25 , xx +
57 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 51 , xx + 231 ) ; xx [
255 ] = xx [ 57 ] + xx [ 231 ] ; xx [ 256 ] = xx [ 58 ] + xx [ 232 ] ; xx [
257 ] = xx [ 59 ] + xx [ 233 ] ; pm_math_Quaternion_compose_ra ( xx + 197 ,
xx + 60 , xx + 426 ) ; xx [ 57 ] = ( xx [ 426 ] * xx [ 428 ] + xx [ 427 ] *
xx [ 429 ] ) * xx [ 34 ] ; xx [ 58 ] = xx [ 34 ] * ( xx [ 428 ] * xx [ 429 ]
- xx [ 426 ] * xx [ 427 ] ) ; xx [ 59 ] = xx [ 48 ] - ( xx [ 427 ] * xx [ 427
] + xx [ 428 ] * xx [ 428 ] ) * xx [ 34 ] ; pm_math_Quaternion_compose_ra (
xx + 197 , xx + 85 , xx + 426 ) ; pm_math_Quaternion_xform_ra ( xx + 426 , xx
+ 98 , xx + 287 ) ; pm_math_Quaternion_xform_ra ( xx + 426 , xx + 113 , xx +
98 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 119 , xx + 113 ) ; xx [
119 ] = xx [ 98 ] + xx [ 113 ] + xx [ 231 ] ; xx [ 120 ] = xx [ 99 ] + xx [
114 ] + xx [ 232 ] ; xx [ 121 ] = xx [ 100 ] + xx [ 115 ] + xx [ 233 ] ; xx [
69 ] = state [ 27 ] + xx [ 813 ] ; xx [ 70 ] = xx [ 69 ] * xx [ 5 ] ; xx [ 98
] = sin ( xx [ 70 ] ) ; xx [ 430 ] = cos ( xx [ 70 ] ) ; xx [ 431 ] = xx [
127 ] * xx [ 98 ] ; xx [ 432 ] = xx [ 129 ] * xx [ 98 ] ; xx [ 433 ] = xx [
137 ] * xx [ 98 ] ; pm_math_Quaternion_compose_ra ( xx + 133 , xx + 430 , xx
+ 434 ) ; pm_math_Quaternion_xform_ra ( xx + 434 , xx + 138 , xx + 98 ) ;
pm_math_Quaternion_xform_ra ( xx + 434 , xx + 149 , xx + 113 ) ; xx [ 231 ] =
xx [ 98 ] + xx [ 113 ] ; xx [ 232 ] = xx [ 99 ] + xx [ 114 ] ; xx [ 233 ] =
xx [ 100 ] + xx [ 115 ] ; pm_math_Quaternion_compose_ra ( xx + 181 , xx + 434
, xx + 430 ) ; pm_math_Quaternion_compose_ra ( xx + 430 , xx + 189 , xx + 438
) ; xx [ 98 ] = ( xx [ 438 ] * xx [ 440 ] + xx [ 439 ] * xx [ 441 ] ) * xx [
34 ] ; xx [ 99 ] = xx [ 34 ] * ( xx [ 440 ] * xx [ 441 ] - xx [ 438 ] * xx [
439 ] ) ; xx [ 100 ] = xx [ 48 ] - ( xx [ 439 ] * xx [ 439 ] + xx [ 440 ] *
xx [ 440 ] ) * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 430 , xx + 193
, xx + 296 ) ; xx [ 70 ] = state [ 41 ] + xx [ 818 ] ;
pm_math_Quaternion_xform_ra ( xx + 430 , xx + 203 , xx + 358 ) ;
pm_math_Quaternion_xform_ra ( xx + 434 , xx + 210 , xx + 365 ) ; xx [ 132 ] =
state [ 28 ] + xx [ 814 ] ; xx [ 188 ] = xx [ 217 ] - xx [ 132 ] - xx [ 367 ]
; xx [ 374 ] = xx [ 209 ] - xx [ 365 ] ; xx [ 375 ] = xx [ 216 ] - xx [ 366 ]
; xx [ 376 ] = xx [ 188 ] ; pm_math_Quaternion_xform_ra ( xx + 181 , xx + 374
, xx + 389 ) ; xx [ 201 ] = state [ 25 ] + xx [ 812 ] ; xx [ 202 ] = xx [ 215
] + xx [ 201 ] * xx [ 94 ] ; xx [ 213 ] = xx [ 225 ] - xx [ 201 ] * xx [ 148
] ; xx [ 214 ] = xx [ 201 ] * xx [ 170 ] - xx [ 229 ] ; xx [ 374 ] = xx [ 6 ]
- xx [ 70 ] * xx [ 196 ] - ( xx [ 358 ] + xx [ 389 ] + xx [ 202 ] ) ; xx [
375 ] = xx [ 220 ] - xx [ 70 ] * xx [ 221 ] - ( xx [ 359 ] + xx [ 390 ] + xx
[ 213 ] ) ; xx [ 376 ] = xx [ 70 ] * xx [ 227 ] - xx [ 228 ] - ( xx [ 360 ] +
xx [ 391 ] + xx [ 214 ] ) ; pm_math_Quaternion_xform_ra ( xx + 430 , xx + 206
, xx + 358 ) ; pm_math_Quaternion_xform_ra ( xx + 181 , xx + 113 , xx + 389 )
; xx [ 113 ] = xx [ 358 ] + xx [ 389 ] ; xx [ 114 ] = xx [ 359 ] + xx [ 390 ]
; xx [ 115 ] = xx [ 360 ] + xx [ 391 ] ; xx [ 218 ] = state [ 19 ] + xx [ 810
] ; xx [ 358 ] = xx [ 218 ] * xx [ 5 ] ; xx [ 359 ] = sin ( xx [ 358 ] ) ; xx
[ 438 ] = cos ( xx [ 358 ] ) ; xx [ 439 ] = xx [ 224 ] * xx [ 359 ] ; xx [
440 ] = - ( xx [ 245 ] * xx [ 359 ] ) ; xx [ 441 ] = - ( xx [ 246 ] * xx [
359 ] ) ; pm_math_Quaternion_compose_ra ( xx + 240 , xx + 438 , xx + 442 ) ;
pm_math_Quaternion_xform_ra ( xx + 442 , xx + 247 , xx + 358 ) ;
pm_math_Quaternion_xform_ra ( xx + 442 , xx + 268 , xx + 389 ) ; xx [ 367 ] =
state [ 20 ] + xx [ 811 ] ; pm_math_Quaternion_xform_ra ( xx + 442 , xx + 284
, xx + 411 ) ; xx [ 420 ] = xx [ 267 ] - ( xx [ 389 ] + xx [ 367 ] * xx [ 271
] - xx [ 411 ] + xx [ 290 ] ) ; xx [ 421 ] = xx [ 293 ] - ( xx [ 390 ] + xx [
367 ] * xx [ 294 ] - xx [ 412 ] + xx [ 295 ] ) ; xx [ 422 ] = xx [ 274 ] - (
xx [ 391 ] + xx [ 367 ] * xx [ 278 ] - xx [ 413 ] + xx [ 279 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 442 , xx + 280 , xx + 389 ) ;
pm_math_Quaternion_xform_ra ( xx + 442 , xx + 299 , xx + 411 ) ; xx [ 438 ] =
xx [ 389 ] + xx [ 411 ] ; xx [ 439 ] = xx [ 390 ] + xx [ 412 ] ; xx [ 440 ] =
xx [ 391 ] + xx [ 413 ] ; pm_math_Quaternion_compose_ra ( xx + 442 , xx + 308
, xx + 446 ) ; xx [ 389 ] = ( xx [ 446 ] * xx [ 448 ] + xx [ 447 ] * xx [ 449
] ) * xx [ 34 ] ; xx [ 390 ] = xx [ 34 ] * ( xx [ 448 ] * xx [ 449 ] - xx [
446 ] * xx [ 447 ] ) ; xx [ 391 ] = xx [ 48 ] - ( xx [ 447 ] * xx [ 447 ] +
xx [ 448 ] * xx [ 448 ] ) * xx [ 34 ] ; xx [ 267 ] = state [ 33 ] + xx [ 815
] ; xx [ 274 ] = xx [ 267 ] * xx [ 5 ] ; xx [ 293 ] = sin ( xx [ 274 ] ) ; xx
[ 446 ] = cos ( xx [ 274 ] ) ; xx [ 447 ] = xx [ 316 ] * xx [ 293 ] ; xx [
448 ] = - ( xx [ 318 ] * xx [ 293 ] ) ; xx [ 449 ] = - ( xx [ 319 ] * xx [
293 ] ) ; pm_math_Quaternion_compose_ra ( xx + 312 , xx + 446 , xx + 450 ) ;
pm_math_Quaternion_xform_ra ( xx + 450 , xx + 320 , xx + 411 ) ;
pm_math_Quaternion_xform_ra ( xx + 450 , xx + 339 , xx + 446 ) ; xx [ 274 ] =
state [ 34 ] + xx [ 816 ] ; pm_math_Quaternion_xform_ra ( xx + 450 , xx + 355
, xx + 454 ) ; xx [ 457 ] = xx [ 338 ] - ( xx [ 446 ] + xx [ 274 ] * xx [ 342
] - xx [ 454 ] + xx [ 317 ] ) ; xx [ 458 ] = xx [ 363 ] - ( xx [ 447 ] + xx [
274 ] * xx [ 364 ] - xx [ 455 ] + xx [ 362 ] ) ; xx [ 459 ] = xx [ 345 ] - (
xx [ 448 ] + xx [ 274 ] * xx [ 349 ] - xx [ 456 ] + xx [ 344 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 450 , xx + 350 , xx + 446 ) ;
pm_math_Quaternion_xform_ra ( xx + 450 , xx + 368 , xx + 454 ) ; xx [ 460 ] =
xx [ 446 ] + xx [ 454 ] ; xx [ 461 ] = xx [ 447 ] + xx [ 455 ] ; xx [ 462 ] =
xx [ 448 ] + xx [ 456 ] ; pm_math_Quaternion_compose_ra ( xx + 450 , xx + 377
, xx + 446 ) ; xx [ 454 ] = ( xx [ 446 ] * xx [ 448 ] + xx [ 447 ] * xx [ 449
] ) * xx [ 34 ] ; xx [ 455 ] = xx [ 34 ] * ( xx [ 448 ] * xx [ 449 ] - xx [
446 ] * xx [ 447 ] ) ; xx [ 456 ] = xx [ 48 ] - ( xx [ 447 ] * xx [ 447 ] +
xx [ 448 ] * xx [ 448 ] ) * xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx +
85 , xx + 442 , xx + 446 ) ; pm_math_Quaternion_compose_ra ( xx + 197 , xx +
446 , xx + 463 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 446 , xx + 89 ,
xx + 467 ) ; pm_math_Vector3_cross_ra ( xx + 467 , xx + 381 , xx + 446 ) ;
pm_math_Quaternion_xform_ra ( xx + 463 , xx + 446 , xx + 467 ) ;
pm_math_Quaternion_compose_ra ( xx + 426 , xx + 442 , xx + 446 ) ;
pm_math_Quaternion_xform_ra ( xx + 446 , xx + 251 , xx + 441 ) ;
pm_math_Quaternion_compose_ra ( xx + 166 , xx + 450 , xx + 463 ) ;
pm_math_Quaternion_xform_ra ( xx + 463 , xx + 324 , xx + 450 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 408 , xx + 470 ) ;
pm_math_Quaternion_xform_ra ( xx + 430 , xx + 417 , xx + 473 ) ; xx [ 824 ] =
xx [ 0 ] ; xx [ 825 ] = xx [ 0 ] ; xx [ 826 ] = xx [ 0 ] ; xx [ 827 ] = xx [
0 ] ; xx [ 828 ] = xx [ 0 ] ; xx [ 829 ] = xx [ 0 ] ; xx [ 830 ] =
pm_math_Vector3_dot_ra ( xx + 45 , xx + 185 ) - pm_math_Vector3_dot_ra ( xx +
255 , xx + 57 ) ; xx [ 831 ] = - pm_math_Vector3_dot_ra ( xx + 64 , xx + 57 )
; xx [ 832 ] = xx [ 0 ] ; xx [ 833 ] = xx [ 0 ] ; xx [ 834 ] = xx [ 0 ] ; xx
[ 835 ] = xx [ 0 ] ; xx [ 836 ] = xx [ 0 ] ; xx [ 837 ] = xx [ 0 ] ; xx [ 838
] = xx [ 0 ] ; xx [ 839 ] = pm_math_Vector3_dot_ra ( xx + 71 , xx + 57 ) ; xx
[ 840 ] = xx [ 0 ] ; xx [ 841 ] = xx [ 0 ] ; xx [ 842 ] = xx [ 0 ] ; xx [ 843
] = xx [ 0 ] ; xx [ 844 ] = xx [ 0 ] ; xx [ 845 ] = xx [ 0 ] ; xx [ 846 ] =
xx [ 0 ] ; xx [ 847 ] = xx [ 0 ] ; xx [ 848 ] = xx [ 0 ] ; xx [ 849 ] = xx [
0 ] ; xx [ 850 ] = xx [ 0 ] ; xx [ 851 ] = xx [ 0 ] ; xx [ 852 ] = xx [ 0 ] ;
xx [ 853 ] = xx [ 0 ] ; xx [ 854 ] = xx [ 0 ] ; xx [ 855 ] = xx [ 0 ] ; xx [
856 ] = xx [ 49 ] ; xx [ 857 ] = xx [ 0 ] ; xx [ 858 ] = xx [ 0 ] ; xx [ 859
] = xx [ 0 ] ; xx [ 860 ] = xx [ 0 ] ; xx [ 861 ] = xx [ 0 ] ; xx [ 862 ] =
xx [ 0 ] ; xx [ 863 ] = xx [ 0 ] ; xx [ 864 ] = xx [ 0 ] ; xx [ 865 ] = xx [
0 ] ; xx [ 866 ] = xx [ 0 ] ; xx [ 867 ] = xx [ 0 ] ; xx [ 868 ] = xx [ 0 ] ;
xx [ 869 ] = xx [ 0 ] ; xx [ 870 ] = xx [ 0 ] ; xx [ 871 ] = xx [ 0 ] ; xx [
872 ] = xx [ 0 ] ; xx [ 873 ] = xx [ 0 ] ; xx [ 874 ] =
pm_math_Vector3_dot_ra ( xx + 287 , xx + 104 ) ; xx [ 875 ] = xx [ 0 ] ; xx [
876 ] = xx [ 0 ] ; xx [ 877 ] = xx [ 0 ] ; xx [ 878 ] = xx [ 0 ] ; xx [ 879 ]
= xx [ 0 ] ; xx [ 880 ] = xx [ 0 ] ; xx [ 881 ] = xx [ 0 ] ; xx [ 882 ] = xx
[ 0 ] ; xx [ 883 ] = xx [ 0 ] ; xx [ 884 ] = xx [ 0 ] ; xx [ 885 ] = xx [ 0 ]
; xx [ 886 ] = xx [ 0 ] ; xx [ 887 ] = xx [ 0 ] ; xx [ 888 ] = xx [ 0 ] ; xx
[ 889 ] = xx [ 0 ] ; xx [ 890 ] = xx [ 0 ] ; xx [ 891 ] = xx [ 0 ] ; xx [ 892
] = xx [ 0 ] ; xx [ 893 ] = xx [ 0 ] ; xx [ 894 ] = xx [ 0 ] ; xx [ 895 ] =
xx [ 0 ] ; xx [ 896 ] = pm_math_Vector3_dot_ra ( xx + 287 , xx + 107 ) ; xx [
897 ] = xx [ 0 ] ; xx [ 898 ] = xx [ 0 ] ; xx [ 899 ] = xx [ 0 ] ; xx [ 900 ]
= xx [ 0 ] ; xx [ 901 ] = xx [ 0 ] ; xx [ 902 ] = xx [ 0 ] ; xx [ 903 ] = xx
[ 0 ] ; xx [ 904 ] = xx [ 0 ] ; xx [ 905 ] = xx [ 0 ] ; xx [ 906 ] = xx [ 0 ]
; xx [ 907 ] = xx [ 0 ] ; xx [ 908 ] = xx [ 0 ] ; xx [ 909 ] = xx [ 0 ] ; xx
[ 910 ] = xx [ 0 ] ; xx [ 911 ] = xx [ 0 ] ; xx [ 912 ] = xx [ 0 ] ; xx [ 913
] = xx [ 0 ] ; xx [ 914 ] = xx [ 0 ] ; xx [ 915 ] = xx [ 0 ] ; xx [ 916 ] =
xx [ 0 ] ; xx [ 917 ] = xx [ 0 ] ; xx [ 918 ] = pm_math_Vector3_dot_ra ( xx +
119 , xx + 54 ) ; xx [ 919 ] = xx [ 92 ] ; xx [ 920 ] = xx [ 0 ] ; xx [ 921 ]
= xx [ 0 ] ; xx [ 922 ] = xx [ 0 ] ; xx [ 923 ] = xx [ 0 ] ; xx [ 924 ] = xx
[ 0 ] ; xx [ 925 ] = xx [ 0 ] ; xx [ 926 ] = xx [ 0 ] ; xx [ 927 ] = xx [ 0 ]
; xx [ 928 ] = xx [ 0 ] ; xx [ 929 ] = xx [ 0 ] ; xx [ 930 ] = xx [ 0 ] ; xx
[ 931 ] = xx [ 0 ] ; xx [ 932 ] = xx [ 0 ] ; xx [ 933 ] = xx [ 0 ] ; xx [ 934
] = xx [ 0 ] ; xx [ 935 ] = xx [ 0 ] ; xx [ 936 ] = xx [ 0 ] ; xx [ 937 ] =
xx [ 0 ] ; xx [ 938 ] = xx [ 0 ] ; xx [ 939 ] = xx [ 0 ] ; xx [ 940 ] = xx [
0 ] ; xx [ 941 ] = xx [ 0 ] ; xx [ 942 ] = xx [ 0 ] ; xx [ 943 ] = xx [ 0 ] ;
xx [ 944 ] = xx [ 0 ] ; xx [ 945 ] = xx [ 0 ] ; xx [ 946 ] = xx [ 0 ] ; xx [
947 ] = xx [ 0 ] ; xx [ 948 ] = xx [ 0 ] ; xx [ 949 ] = xx [ 0 ] ; xx [ 950 ]
= xx [ 0 ] ; xx [ 951 ] = xx [ 93 ] ; xx [ 952 ] = xx [ 0 ] ; xx [ 953 ] = xx
[ 0 ] ; xx [ 954 ] = xx [ 0 ] ; xx [ 955 ] = xx [ 0 ] ; xx [ 956 ] = xx [ 0 ]
; xx [ 957 ] = xx [ 0 ] ; xx [ 958 ] = xx [ 0 ] ; xx [ 959 ] = xx [ 0 ] ; xx
[ 960 ] = xx [ 0 ] ; xx [ 961 ] = xx [ 0 ] ; xx [ 962 ] = xx [ 0 ] ; xx [ 963
] = xx [ 0 ] ; xx [ 964 ] = xx [ 0 ] ; xx [ 965 ] = xx [ 0 ] ; xx [ 966 ] =
xx [ 0 ] ; xx [ 967 ] = pm_math_Vector3_dot_ra ( xx + 231 , xx + 174 ) ; xx [
968 ] = xx [ 141 ] ; xx [ 969 ] = xx [ 0 ] ; xx [ 970 ] = xx [ 0 ] ; xx [ 971
] = xx [ 0 ] ; xx [ 972 ] = xx [ 0 ] ; xx [ 973 ] = xx [ 0 ] ; xx [ 974 ] =
xx [ 0 ] ; xx [ 975 ] = xx [ 0 ] ; xx [ 976 ] = xx [ 0 ] ; xx [ 977 ] = xx [
0 ] ; xx [ 978 ] = xx [ 0 ] ; xx [ 979 ] = xx [ 0 ] ; xx [ 980 ] = xx [ 0 ] ;
xx [ 981 ] = xx [ 0 ] ; xx [ 982 ] = xx [ 0 ] ; xx [ 983 ] = xx [ 0 ] ; xx [
984 ] = xx [ 0 ] ; xx [ 985 ] = xx [ 0 ] ; xx [ 986 ] = xx [ 0 ] ; xx [ 987 ]
= xx [ 0 ] ; xx [ 988 ] = - pm_math_Vector3_dot_ra ( xx + 171 , xx + 98 ) ;
xx [ 989 ] = pm_math_Vector3_dot_ra ( xx + 296 , xx + 374 ) -
pm_math_Vector3_dot_ra ( xx + 113 , xx + 98 ) ; xx [ 990 ] = -
pm_math_Vector3_dot_ra ( xx + 234 , xx + 98 ) ; xx [ 991 ] = xx [ 0 ] ; xx [
992 ] = xx [ 0 ] ; xx [ 993 ] = xx [ 0 ] ; xx [ 994 ] =
pm_math_Vector3_dot_ra ( xx + 237 , xx + 98 ) ; xx [ 995 ] = xx [ 0 ] ; xx [
996 ] = xx [ 0 ] ; xx [ 997 ] = xx [ 0 ] ; xx [ 998 ] = xx [ 0 ] ; xx [ 999 ]
= xx [ 0 ] ; xx [ 1000 ] = xx [ 0 ] ; xx [ 1001 ] = xx [ 0 ] ; xx [ 1002 ] =
xx [ 0 ] ; xx [ 1003 ] = xx [ 0 ] ; xx [ 1004 ] = xx [ 0 ] ; xx [ 1005 ] = xx
[ 0 ] ; xx [ 1006 ] = xx [ 0 ] ; xx [ 1007 ] = xx [ 0 ] ; xx [ 1008 ] =
pm_math_Vector3_dot_ra ( xx + 358 , xx + 420 ) - pm_math_Vector3_dot_ra ( xx
+ 438 , xx + 389 ) ; xx [ 1009 ] = - pm_math_Vector3_dot_ra ( xx + 302 , xx +
389 ) ; xx [ 1010 ] = xx [ 0 ] ; xx [ 1011 ] = xx [ 0 ] ; xx [ 1012 ] = xx [
0 ] ; xx [ 1013 ] = xx [ 0 ] ; xx [ 1014 ] = xx [ 0 ] ; xx [ 1015 ] = xx [ 0
] ; xx [ 1016 ] = xx [ 0 ] ; xx [ 1017 ] = xx [ 0 ] ; xx [ 1018 ] = xx [ 0 ]
; xx [ 1019 ] = xx [ 0 ] ; xx [ 1020 ] = xx [ 0 ] ; xx [ 1021 ] = xx [ 0 ] ;
xx [ 1022 ] = xx [ 0 ] ; xx [ 1023 ] = xx [ 0 ] ; xx [ 1024 ] = xx [ 0 ] ; xx
[ 1025 ] = xx [ 0 ] ; xx [ 1026 ] = xx [ 0 ] ; xx [ 1027 ] = xx [ 0 ] ; xx [
1028 ] = xx [ 0 ] ; xx [ 1029 ] = xx [ 0 ] ; xx [ 1030 ] = xx [ 0 ] ; xx [
1031 ] = xx [ 0 ] ; xx [ 1032 ] = xx [ 0 ] ; xx [ 1033 ] = xx [ 0 ] ; xx [
1034 ] = xx [ 0 ] ; xx [ 1035 ] = pm_math_Vector3_dot_ra ( xx + 411 , xx +
457 ) - pm_math_Vector3_dot_ra ( xx + 460 , xx + 454 ) ; xx [ 1036 ] = -
pm_math_Vector3_dot_ra ( xx + 371 , xx + 454 ) ; xx [ 1037 ] = xx [ 0 ] ; xx
[ 1038 ] = xx [ 0 ] ; xx [ 1039 ] = xx [ 0 ] ; xx [ 1040 ] = xx [ 0 ] ; xx [
1041 ] = xx [ 0 ] ; xx [ 1042 ] = xx [ 0 ] ; xx [ 1043 ] = xx [ 0 ] ; xx [
1044 ] = xx [ 0 ] ; xx [ 1045 ] = xx [ 0 ] ; xx [ 1046 ] = xx [ 0 ] ; xx [
1047 ] = xx [ 0 ] ; xx [ 1048 ] = xx [ 0 ] ; xx [ 1049 ] = xx [ 0 ] ; xx [
1050 ] = pm_math_Vector3_dot_ra ( xx + 467 , xx + 384 ) ; xx [ 1051 ] = xx [
0 ] ; xx [ 1052 ] = pm_math_Vector3_dot_ra ( xx + 441 , xx + 384 ) ; xx [
1053 ] = xx [ 0 ] ; xx [ 1054 ] = xx [ 0 ] ; xx [ 1055 ] = xx [ 0 ] ; xx [
1056 ] = xx [ 0 ] ; xx [ 1057 ] = xx [ 0 ] ; xx [ 1058 ] = xx [ 0 ] ; xx [
1059 ] = xx [ 0 ] ; xx [ 1060 ] = xx [ 0 ] ; xx [ 1061 ] = xx [ 0 ] ; xx [
1062 ] = xx [ 0 ] ; xx [ 1063 ] = xx [ 0 ] ; xx [ 1064 ] = xx [ 0 ] ; xx [
1065 ] = xx [ 0 ] ; xx [ 1066 ] = xx [ 0 ] ; xx [ 1067 ] = xx [ 0 ] ; xx [
1068 ] = xx [ 0 ] ; xx [ 1069 ] = xx [ 0 ] ; xx [ 1070 ] = xx [ 0 ] ; xx [
1071 ] = xx [ 0 ] ; xx [ 1072 ] = pm_math_Vector3_dot_ra ( xx + 467 , xx +
399 ) ; xx [ 1073 ] = xx [ 0 ] ; xx [ 1074 ] = pm_math_Vector3_dot_ra ( xx +
441 , xx + 399 ) ; xx [ 1075 ] = xx [ 0 ] ; xx [ 1076 ] = xx [ 0 ] ; xx [
1077 ] = xx [ 0 ] ; xx [ 1078 ] = xx [ 0 ] ; xx [ 1079 ] = xx [ 0 ] ; xx [
1080 ] = xx [ 0 ] ; xx [ 1081 ] = xx [ 0 ] ; xx [ 1082 ] = xx [ 0 ] ; xx [
1083 ] = xx [ 0 ] ; xx [ 1084 ] = xx [ 0 ] ; xx [ 1085 ] = xx [ 0 ] ; xx [
1086 ] = xx [ 0 ] ; xx [ 1087 ] = xx [ 0 ] ; xx [ 1088 ] = xx [ 0 ] ; xx [
1089 ] = xx [ 0 ] ; xx [ 1090 ] = xx [ 0 ] ; xx [ 1091 ] = xx [ 0 ] ; xx [
1092 ] = xx [ 0 ] ; xx [ 1093 ] = xx [ 0 ] ; xx [ 1094 ] = xx [ 0 ] ; xx [
1095 ] = xx [ 0 ] ; xx [ 1096 ] = xx [ 0 ] ; xx [ 1097 ] = xx [ 0 ] ; xx [
1098 ] = xx [ 0 ] ; xx [ 1099 ] = xx [ 0 ] ; xx [ 1100 ] = xx [ 0 ] ; xx [
1101 ] = xx [ 254 ] * xx [ 450 ] + xx [ 327 ] * xx [ 451 ] ; xx [ 1102 ] = xx
[ 0 ] ; xx [ 1103 ] = xx [ 0 ] ; xx [ 1104 ] = xx [ 0 ] ; xx [ 1105 ] = xx [
0 ] ; xx [ 1106 ] = xx [ 0 ] ; xx [ 1107 ] = xx [ 0 ] ; xx [ 1108 ] = xx [ 0
] ; xx [ 1109 ] = xx [ 0 ] ; xx [ 1110 ] = xx [ 0 ] ; xx [ 1111 ] = xx [ 0 ]
; xx [ 1112 ] = xx [ 0 ] ; xx [ 1113 ] = xx [ 0 ] ; xx [ 1114 ] = xx [ 0 ] ;
xx [ 1115 ] = xx [ 0 ] ; xx [ 1116 ] = xx [ 0 ] ; xx [ 1117 ] = xx [ 0 ] ; xx
[ 1118 ] = xx [ 0 ] ; xx [ 1119 ] = xx [ 0 ] ; xx [ 1120 ] = xx [ 0 ] ; xx [
1121 ] = xx [ 0 ] ; xx [ 1122 ] = xx [ 0 ] ; xx [ 1123 ] = xx [ 254 ] * xx [
451 ] - xx [ 327 ] * xx [ 450 ] ; xx [ 1124 ] = xx [ 0 ] ; xx [ 1125 ] = xx [
0 ] ; xx [ 1126 ] = xx [ 0 ] ; xx [ 1127 ] = xx [ 0 ] ; xx [ 1128 ] = xx [ 0
] ; xx [ 1129 ] = xx [ 0 ] ; xx [ 1130 ] = xx [ 0 ] ; xx [ 1131 ] = xx [ 0 ]
; xx [ 1132 ] = xx [ 0 ] ; xx [ 1133 ] = xx [ 0 ] ; xx [ 1134 ] = xx [ 0 ] ;
xx [ 1135 ] = xx [ 0 ] ; xx [ 1136 ] = xx [ 0 ] ; xx [ 1137 ] = xx [ 0 ] ; xx
[ 1138 ] = pm_math_Vector3_dot_ra ( xx + 470 , xx + 414 ) ; xx [ 1139 ] = xx
[ 0 ] ; xx [ 1140 ] = xx [ 0 ] ; xx [ 1141 ] = xx [ 0 ] ; xx [ 1142 ] = xx [
0 ] ; xx [ 1143 ] = xx [ 0 ] ; xx [ 1144 ] = xx [ 0 ] ; xx [ 1145 ] = xx [ 0
] ; xx [ 1146 ] = xx [ 0 ] ; xx [ 1147 ] = xx [ 0 ] ; xx [ 1148 ] = xx [ 0 ]
; xx [ 1149 ] = xx [ 0 ] ; xx [ 1150 ] = xx [ 0 ] ; xx [ 1151 ] = xx [ 0 ] ;
xx [ 1152 ] = xx [ 0 ] ; xx [ 1153 ] = xx [ 0 ] ; xx [ 1154 ] = xx [ 0 ] ; xx
[ 1155 ] = xx [ 0 ] ; xx [ 1156 ] = xx [ 0 ] ; xx [ 1157 ] = xx [ 0 ] ; xx [
1158 ] = xx [ 0 ] ; xx [ 1159 ] = xx [ 0 ] ; xx [ 1160 ] = xx [ 0 ] ; xx [
1161 ] = xx [ 0 ] ; xx [ 1162 ] = xx [ 0 ] ; xx [ 1163 ] = xx [ 0 ] ; xx [
1164 ] = xx [ 0 ] ; xx [ 1165 ] = pm_math_Vector3_dot_ra ( xx + 473 , xx +
423 ) ; xx [ 1166 ] = xx [ 0 ] ; xx [ 1167 ] = xx [ 0 ] ; xx [ 1168 ] = xx [
0 ] ; xx [ 1169 ] = xx [ 0 ] ; xx [ 1170 ] = xx [ 0 ] ; xx [ 1171 ] = xx [ 0
] ; xx [ 1172 ] = xx [ 0 ] ; xx [ 1173 ] = xx [ 0 ] ; xx [ 1174 ] = xx [ 0 ]
; xx [ 1175 ] = xx [ 0 ] ; xx [ 45 ] = xx [ 19 ] - xx [ 202 ] ; xx [ 46 ] =
xx [ 20 ] - xx [ 213 ] ; xx [ 47 ] = - ( xx [ 21 ] + xx [ 214 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 426 , xx + 152 , xx + 438 ) ; xx [ 113 ]
= ( xx [ 438 ] * xx [ 440 ] + xx [ 439 ] * xx [ 441 ] ) * xx [ 34 ] ; xx [
114 ] = xx [ 34 ] * ( xx [ 440 ] * xx [ 441 ] - xx [ 438 ] * xx [ 439 ] ) ;
xx [ 115 ] = xx [ 48 ] - ( xx [ 439 ] * xx [ 439 ] + xx [ 440 ] * xx [ 440 ]
) * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 426 , xx + 110 , xx + 119
) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 124 , xx + 231 ) ; xx [ 255
] = xx [ 119 ] + xx [ 231 ] - xx [ 130 ] + xx [ 84 ] ; xx [ 256 ] = xx [ 120
] + xx [ 232 ] + xx [ 131 ] - xx [ 156 ] ; xx [ 257 ] = xx [ 121 ] + xx [ 233
] + xx [ 68 ] - xx [ 157 ] ; xx [ 68 ] = state [ 43 ] + xx [ 819 ] ; xx [ 119
] = xx [ 33 ] + xx [ 68 ] * xx [ 36 ] ; xx [ 120 ] = xx [ 68 ] * xx [ 41 ] -
xx [ 81 ] ; xx [ 121 ] = xx [ 82 ] - xx [ 68 ] * xx [ 83 ] ;
pm_math_Quaternion_xform_ra ( xx + 434 , xx + 328 , xx + 231 ) ; xx [ 287 ] =
xx [ 231 ] - xx [ 365 ] - xx [ 145 ] + xx [ 226 ] ; xx [ 288 ] = xx [ 232 ] -
xx [ 366 ] - xx [ 230 ] - xx [ 354 ] ; xx [ 289 ] = xx [ 233 ] + xx [ 188 ] -
xx [ 387 ] - xx [ 388 ] ; pm_math_Quaternion_compose_ra ( xx + 446 , xx + 778
, xx + 426 ) ; xx [ 231 ] = ( xx [ 426 ] * xx [ 428 ] + xx [ 427 ] * xx [ 429
] ) * xx [ 34 ] ; xx [ 232 ] = xx [ 34 ] * ( xx [ 428 ] * xx [ 429 ] - xx [
426 ] * xx [ 427 ] ) ; xx [ 233 ] = xx [ 48 ] - ( xx [ 427 ] * xx [ 427 ] +
xx [ 428 ] * xx [ 428 ] ) * xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx +
463 , xx + 392 , xx + 426 ) ; xx [ 130 ] = ( xx [ 426 ] * xx [ 428 ] + xx [
427 ] * xx [ 429 ] ) * xx [ 34 ] ; xx [ 131 ] = xx [ 34 ] * ( xx [ 428 ] * xx
[ 429 ] - xx [ 426 ] * xx [ 427 ] ) ; pm_math_Quaternion_compose_ra ( xx +
197 , xx + 402 , xx + 426 ) ; xx [ 197 ] = ( xx [ 426 ] * xx [ 428 ] + xx [
427 ] * xx [ 429 ] ) * xx [ 34 ] ; xx [ 198 ] = xx [ 34 ] * ( xx [ 428 ] * xx
[ 429 ] - xx [ 426 ] * xx [ 427 ] ) ; xx [ 199 ] = xx [ 48 ] - ( xx [ 427 ] *
xx [ 427 ] + xx [ 428 ] * xx [ 428 ] ) * xx [ 34 ] ;
pm_math_Quaternion_compose_ra ( xx + 430 , xx + 782 , xx + 426 ) ; xx [ 296 ]
= xx [ 34 ] * ( xx [ 427 ] * xx [ 428 ] - xx [ 426 ] * xx [ 429 ] ) ; xx [
297 ] = xx [ 48 ] - ( xx [ 429 ] * xx [ 429 ] + xx [ 427 ] * xx [ 427 ] ) *
xx [ 34 ] ; xx [ 298 ] = ( xx [ 426 ] * xx [ 427 ] + xx [ 428 ] * xx [ 429 ]
) * xx [ 34 ] ; xx [ 426 ] = - pm_math_Vector3_dot_ra ( xx + 185 , xx + 57 )
; xx [ 427 ] = - pm_math_Vector3_dot_ra ( xx + 45 , xx + 101 ) ; xx [ 428 ] =
- pm_math_Vector3_dot_ra ( xx + 113 , xx + 104 ) ; xx [ 429 ] = -
pm_math_Vector3_dot_ra ( xx + 113 , xx + 107 ) ; xx [ 430 ] = -
pm_math_Vector3_dot_ra ( xx + 255 , xx + 54 ) ; xx [ 431 ] = -
pm_math_Vector3_dot_ra ( xx + 119 , xx + 305 ) ; xx [ 432 ] = -
pm_math_Vector3_dot_ra ( xx + 287 , xx + 174 ) ; xx [ 433 ] = -
pm_math_Vector3_dot_ra ( xx + 374 , xx + 98 ) ; xx [ 434 ] = -
pm_math_Vector3_dot_ra ( xx + 420 , xx + 389 ) ; xx [ 435 ] = -
pm_math_Vector3_dot_ra ( xx + 457 , xx + 454 ) ; xx [ 436 ] = -
pm_math_Vector3_dot_ra ( xx + 231 , xx + 384 ) ; xx [ 437 ] = -
pm_math_Vector3_dot_ra ( xx + 231 , xx + 399 ) ; xx [ 438 ] = - ( xx [ 254 ]
* xx [ 130 ] + xx [ 327 ] * xx [ 131 ] ) ; xx [ 439 ] = - ( xx [ 254 ] * xx [
131 ] - xx [ 327 ] * xx [ 130 ] ) ; xx [ 440 ] = - pm_math_Vector3_dot_ra (
xx + 197 , xx + 414 ) ; xx [ 441 ] = - pm_math_Vector3_dot_ra ( xx + 423 , xx
+ 296 ) ; memcpy ( xx + 1176 , xx + 824 , 352 * sizeof ( double ) ) ;
factorAndSolveWide ( 16 , 22 , xx + 1176 , xx + 464 , xx + 480 , ii + 0 , xx
+ 426 , xx [ 15 ] , xx + 442 ) ; xx [ 45 ] = xx [ 48 ] / sqrt ( state [ 3 ] *
state [ 3 ] + state [ 4 ] * state [ 4 ] + state [ 5 ] * state [ 5 ] + state [
6 ] * state [ 6 ] ) ; xx [ 46 ] = xx [ 45 ] * state [ 3 ] ; xx [ 47 ] = xx [
45 ] * state [ 4 ] ; xx [ 57 ] = xx [ 45 ] * state [ 5 ] ; xx [ 58 ] = xx [
45 ] * state [ 6 ] ; xx [ 185 ] = xx [ 46 ] ; xx [ 186 ] = xx [ 47 ] ; xx [
187 ] = xx [ 57 ] ; xx [ 188 ] = xx [ 58 ] ; pm_math_Quaternion_compDeriv_ra
( xx + 185 , xx + 805 , xx + 197 ) ; xx [ 45 ] = xx [ 46 ] + xx [ 197 ] ; xx
[ 46 ] = xx [ 47 ] + xx [ 198 ] ; xx [ 47 ] = xx [ 57 ] + xx [ 199 ] ; xx [
57 ] = xx [ 58 ] + xx [ 200 ] ; xx [ 58 ] = 1.0e-64 ; xx [ 59 ] = sqrt ( xx [
45 ] * xx [ 45 ] + xx [ 46 ] * xx [ 46 ] + xx [ 47 ] * xx [ 47 ] + xx [ 57 ]
* xx [ 57 ] ) ; if ( xx [ 58 ] > xx [ 59 ] ) xx [ 59 ] = xx [ 58 ] ; xx [ 98
] = xx [ 45 ] / xx [ 59 ] ; xx [ 45 ] = xx [ 46 ] / xx [ 59 ] ; xx [ 46 ] =
xx [ 47 ] / xx [ 59 ] ; xx [ 47 ] = xx [ 57 ] / xx [ 59 ] ; xx [ 185 ] = xx [
98 ] ; xx [ 186 ] = xx [ 45 ] ; xx [ 187 ] = xx [ 46 ] ; xx [ 188 ] = xx [ 47
] ; pm_math_Quaternion_compDeriv_ra ( xx + 185 , xx + 445 , xx + 197 ) ; xx [
57 ] = xx [ 98 ] + xx [ 197 ] ; xx [ 59 ] = xx [ 45 ] + xx [ 198 ] ; xx [ 45
] = xx [ 46 ] + xx [ 199 ] ; xx [ 46 ] = xx [ 47 ] + xx [ 200 ] ; xx [ 47 ] =
sqrt ( xx [ 57 ] * xx [ 57 ] + xx [ 59 ] * xx [ 59 ] + xx [ 45 ] * xx [ 45 ]
+ xx [ 46 ] * xx [ 46 ] ) ; if ( xx [ 58 ] > xx [ 47 ] ) xx [ 47 ] = xx [ 58
] ; xx [ 58 ] = xx [ 16 ] + xx [ 448 ] ; xx [ 16 ] = xx [ 18 ] + xx [ 449 ] ;
xx [ 18 ] = xx [ 218 ] + xx [ 450 ] ; xx [ 98 ] = xx [ 367 ] + xx [ 451 ] ;
xx [ 99 ] = xx [ 201 ] + xx [ 452 ] ; xx [ 100 ] = xx [ 69 ] + xx [ 453 ] ;
xx [ 69 ] = xx [ 132 ] + xx [ 454 ] ; xx [ 113 ] = xx [ 267 ] + xx [ 455 ] ;
xx [ 114 ] = xx [ 274 ] + xx [ 456 ] ; xx [ 115 ] = xx [ 17 ] + xx [ 457 ] ;
xx [ 17 ] = xx [ 70 ] + xx [ 458 ] ; xx [ 70 ] = xx [ 68 ] + xx [ 459 ] ; xx
[ 464 ] = state [ 0 ] + xx [ 802 ] + xx [ 442 ] ; xx [ 465 ] = state [ 1 ] +
xx [ 803 ] + xx [ 443 ] ; xx [ 466 ] = state [ 2 ] + xx [ 804 ] + xx [ 444 ]
; xx [ 467 ] = xx [ 57 ] / xx [ 47 ] ; xx [ 468 ] = xx [ 59 ] / xx [ 47 ] ;
xx [ 469 ] = xx [ 45 ] / xx [ 47 ] ; xx [ 470 ] = xx [ 46 ] / xx [ 47 ] ; xx
[ 471 ] = state [ 7 ] ; xx [ 472 ] = state [ 8 ] ; xx [ 473 ] = state [ 9 ] ;
xx [ 474 ] = state [ 10 ] ; xx [ 475 ] = state [ 11 ] ; xx [ 476 ] = state [
12 ] ; xx [ 477 ] = xx [ 58 ] ; xx [ 478 ] = xx [ 16 ] ; xx [ 479 ] = state [
15 ] ; xx [ 480 ] = state [ 16 ] ; xx [ 481 ] = state [ 17 ] ; xx [ 482 ] =
state [ 18 ] ; xx [ 483 ] = xx [ 18 ] ; xx [ 484 ] = xx [ 98 ] ; xx [ 485 ] =
state [ 21 ] ; xx [ 486 ] = state [ 22 ] ; xx [ 487 ] = state [ 23 ] ; xx [
488 ] = state [ 24 ] ; xx [ 489 ] = xx [ 99 ] ; xx [ 490 ] = state [ 26 ] ;
xx [ 491 ] = xx [ 100 ] ; xx [ 492 ] = xx [ 69 ] ; xx [ 493 ] = state [ 29 ]
; xx [ 494 ] = state [ 30 ] ; xx [ 495 ] = state [ 31 ] ; xx [ 496 ] = state
[ 32 ] ; xx [ 497 ] = xx [ 113 ] ; xx [ 498 ] = xx [ 114 ] ; xx [ 499 ] =
state [ 35 ] ; xx [ 500 ] = state [ 36 ] ; xx [ 501 ] = state [ 37 ] ; xx [
502 ] = state [ 38 ] ; xx [ 503 ] = xx [ 115 ] ; xx [ 504 ] = state [ 40 ] ;
xx [ 505 ] = xx [ 17 ] ; xx [ 506 ] = state [ 42 ] ; xx [ 507 ] = xx [ 70 ] ;
xx [ 508 ] = state [ 44 ] ; xx [ 509 ] = state [ 45 ] + xx [ 820 ] + xx [ 460
] ; xx [ 510 ] = state [ 46 ] ; xx [ 511 ] = state [ 47 ] + xx [ 821 ] + xx [
461 ] ; xx [ 512 ] = state [ 48 ] ; xx [ 513 ] = state [ 49 ] + xx [ 822 ] +
xx [ 462 ] ; xx [ 514 ] = state [ 50 ] ; xx [ 515 ] = state [ 51 ] + xx [ 823
] + xx [ 463 ] ; xx [ 516 ] = state [ 52 ] ; xx [ 517 ] = state [ 53 ] ; xx [
518 ] = state [ 54 ] ; xx [ 519 ] = state [ 55 ] ; xx [ 520 ] = state [ 56 ]
; xx [ 521 ] = state [ 57 ] ; xx [ 522 ] = state [ 58 ] ; xx [ 523 ] = state
[ 59 ] ; xx [ 524 ] = state [ 60 ] ; xx [ 525 ] = state [ 61 ] ; xx [ 526 ] =
state [ 62 ] ; xx [ 527 ] = state [ 63 ] ; xx [ 528 ] = state [ 64 ] ; xx [
529 ] = state [ 65 ] ; xx [ 530 ] = state [ 66 ] ; xx [ 531 ] = state [ 67 ]
; xx [ 532 ] = state [ 68 ] ; xx [ 45 ] = xx [ 58 ] * xx [ 5 ] ; xx [ 46 ] =
sin ( xx [ 45 ] ) ; xx [ 185 ] = cos ( xx [ 45 ] ) ; xx [ 186 ] = xx [ 7 ] *
xx [ 46 ] ; xx [ 187 ] = xx [ 9 ] * xx [ 46 ] ; xx [ 188 ] = xx [ 10 ] * xx [
46 ] ; pm_math_Quaternion_compose_ra ( xx + 1 , xx + 185 , xx + 197 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 22 , xx + 45 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 29 , xx + 57 ) ; xx [ 68 ] = xx
[ 16 ] * xx [ 28 ] + xx [ 57 ] + xx [ 35 ] ; xx [ 119 ] = xx [ 39 ] - ( xx [
16 ] * xx [ 40 ] + xx [ 58 ] ) ; xx [ 57 ] = xx [ 16 ] * xx [ 44 ] - xx [ 59
] + xx [ 32 ] ; xx [ 130 ] = - ( xx [ 115 ] * xx [ 8 ] + xx [ 45 ] - xx [ 68
] + xx [ 14 ] ) ; xx [ 131 ] = xx [ 37 ] - ( xx [ 115 ] * xx [ 38 ] + xx [ 46
] + xx [ 119 ] ) ; xx [ 132 ] = xx [ 115 ] * xx [ 42 ] - ( xx [ 47 ] + xx [
57 ] ) + xx [ 43 ] ; pm_math_Quaternion_compose_ra ( xx + 197 , xx + 60 , xx
+ 185 ) ; xx [ 45 ] = ( xx [ 185 ] * xx [ 187 ] + xx [ 186 ] * xx [ 188 ] ) *
xx [ 34 ] ; xx [ 46 ] = xx [ 34 ] * ( xx [ 187 ] * xx [ 188 ] - xx [ 185 ] *
xx [ 186 ] ) ; xx [ 47 ] = xx [ 48 ] - ( xx [ 186 ] * xx [ 186 ] + xx [ 187 ]
* xx [ 187 ] ) * xx [ 34 ] ; xx [ 16 ] = xx [ 215 ] + xx [ 99 ] * xx [ 94 ] ;
xx [ 58 ] = xx [ 225 ] - xx [ 99 ] * xx [ 148 ] ; xx [ 59 ] = xx [ 99 ] * xx
[ 170 ] - xx [ 229 ] ; xx [ 185 ] = xx [ 19 ] - xx [ 16 ] ; xx [ 186 ] = xx [
20 ] - xx [ 58 ] ; xx [ 187 ] = - ( xx [ 21 ] + xx [ 59 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 197 , xx + 85 , xx + 426 ) ;
pm_math_Quaternion_compose_ra ( xx + 426 , xx + 152 , xx + 85 ) ; xx [ 19 ] =
( xx [ 85 ] * xx [ 87 ] + xx [ 86 ] * xx [ 88 ] ) * xx [ 34 ] ; xx [ 20 ] =
xx [ 34 ] * ( xx [ 87 ] * xx [ 88 ] - xx [ 85 ] * xx [ 86 ] ) ; xx [ 21 ] =
xx [ 48 ] - ( xx [ 86 ] * xx [ 86 ] + xx [ 87 ] * xx [ 87 ] ) * xx [ 34 ] ;
pm_math_Quaternion_xform_ra ( xx + 426 , xx + 110 , xx + 85 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 124 , xx + 152 ) ; xx [ 124 ] =
xx [ 85 ] + xx [ 152 ] - xx [ 68 ] + xx [ 84 ] ; xx [ 125 ] = xx [ 86 ] + xx
[ 153 ] + xx [ 119 ] - xx [ 156 ] ; xx [ 126 ] = xx [ 87 ] + xx [ 154 ] + xx
[ 57 ] - xx [ 157 ] ; xx [ 84 ] = xx [ 33 ] + xx [ 70 ] * xx [ 36 ] ; xx [ 85
] = xx [ 70 ] * xx [ 41 ] - xx [ 81 ] ; xx [ 86 ] = xx [ 82 ] - xx [ 70 ] *
xx [ 83 ] ; xx [ 33 ] = xx [ 100 ] * xx [ 5 ] ; xx [ 57 ] = sin ( xx [ 33 ] )
; xx [ 152 ] = cos ( xx [ 33 ] ) ; xx [ 153 ] = xx [ 127 ] * xx [ 57 ] ; xx [
154 ] = xx [ 129 ] * xx [ 57 ] ; xx [ 155 ] = xx [ 137 ] * xx [ 57 ] ;
pm_math_Quaternion_compose_ra ( xx + 133 , xx + 152 , xx + 430 ) ;
pm_math_Quaternion_xform_ra ( xx + 430 , xx + 328 , xx + 119 ) ;
pm_math_Quaternion_xform_ra ( xx + 430 , xx + 210 , xx + 152 ) ; xx [ 33 ] =
xx [ 217 ] - xx [ 69 ] - xx [ 154 ] ; xx [ 68 ] = xx [ 119 ] - xx [ 152 ] -
xx [ 145 ] + xx [ 226 ] ; xx [ 69 ] = xx [ 120 ] - xx [ 153 ] - xx [ 230 ] -
xx [ 354 ] ; xx [ 70 ] = xx [ 121 ] + xx [ 33 ] - xx [ 387 ] - xx [ 388 ] ;
pm_math_Quaternion_compose_ra ( xx + 181 , xx + 430 , xx + 154 ) ;
pm_math_Quaternion_xform_ra ( xx + 154 , xx + 203 , xx + 119 ) ; xx [ 230 ] =
xx [ 209 ] - xx [ 152 ] ; xx [ 231 ] = xx [ 216 ] - xx [ 153 ] ; xx [ 232 ] =
xx [ 33 ] ; pm_math_Quaternion_xform_ra ( xx + 181 , xx + 230 , xx + 255 ) ;
xx [ 230 ] = xx [ 6 ] - ( xx [ 17 ] * xx [ 196 ] + xx [ 119 ] + xx [ 255 ] +
xx [ 16 ] ) ; xx [ 231 ] = xx [ 220 ] - ( xx [ 17 ] * xx [ 221 ] + xx [ 120 ]
+ xx [ 256 ] + xx [ 58 ] ) ; xx [ 232 ] = xx [ 17 ] * xx [ 227 ] - ( xx [ 121
] + xx [ 257 ] + xx [ 59 ] ) - xx [ 228 ] ; pm_math_Quaternion_compose_ra (
xx + 154 , xx + 189 , xx + 430 ) ; xx [ 57 ] = ( xx [ 430 ] * xx [ 432 ] + xx
[ 431 ] * xx [ 433 ] ) * xx [ 34 ] ; xx [ 58 ] = xx [ 34 ] * ( xx [ 432 ] *
xx [ 433 ] - xx [ 430 ] * xx [ 431 ] ) ; xx [ 59 ] = xx [ 48 ] - ( xx [ 431 ]
* xx [ 431 ] + xx [ 432 ] * xx [ 432 ] ) * xx [ 34 ] ; xx [ 16 ] = xx [ 18 ]
* xx [ 5 ] ; xx [ 17 ] = sin ( xx [ 16 ] ) ; xx [ 430 ] = cos ( xx [ 16 ] ) ;
xx [ 431 ] = xx [ 224 ] * xx [ 17 ] ; xx [ 432 ] = - ( xx [ 245 ] * xx [ 17 ]
) ; xx [ 433 ] = - ( xx [ 246 ] * xx [ 17 ] ) ; pm_math_Quaternion_compose_ra
( xx + 240 , xx + 430 , xx + 434 ) ; pm_math_Quaternion_xform_ra ( xx + 434 ,
xx + 268 , xx + 16 ) ; pm_math_Quaternion_xform_ra ( xx + 434 , xx + 284 , xx
+ 119 ) ; xx [ 33 ] = 5.518112576770381e-4 ; xx [ 81 ] = 8.460002797213924e-7
; xx [ 82 ] = 7.47967269472322e-3 ; xx [ 255 ] = xx [ 223 ] - ( xx [ 16 ] +
xx [ 98 ] * xx [ 271 ] - xx [ 119 ] ) - xx [ 33 ] ; xx [ 256 ] = xx [ 291 ] -
( xx [ 17 ] + xx [ 98 ] * xx [ 294 ] - xx [ 120 ] ) - xx [ 81 ] ; xx [ 257 ]
= xx [ 272 ] - ( xx [ 18 ] + xx [ 98 ] * xx [ 278 ] - xx [ 121 ] ) - xx [ 82
] ; pm_math_Quaternion_compose_ra ( xx + 434 , xx + 308 , xx + 430 ) ; xx [
16 ] = ( xx [ 430 ] * xx [ 432 ] + xx [ 431 ] * xx [ 433 ] ) * xx [ 34 ] ; xx
[ 17 ] = xx [ 34 ] * ( xx [ 432 ] * xx [ 433 ] - xx [ 430 ] * xx [ 431 ] ) ;
xx [ 18 ] = xx [ 48 ] - ( xx [ 431 ] * xx [ 431 ] + xx [ 432 ] * xx [ 432 ] )
* xx [ 34 ] ; xx [ 87 ] = xx [ 113 ] * xx [ 5 ] ; xx [ 88 ] = sin ( xx [ 87 ]
) ; xx [ 430 ] = cos ( xx [ 87 ] ) ; xx [ 431 ] = xx [ 316 ] * xx [ 88 ] ; xx
[ 432 ] = - ( xx [ 318 ] * xx [ 88 ] ) ; xx [ 433 ] = - ( xx [ 319 ] * xx [
88 ] ) ; pm_math_Quaternion_compose_ra ( xx + 312 , xx + 430 , xx + 438 ) ;
pm_math_Quaternion_xform_ra ( xx + 438 , xx + 339 , xx + 98 ) ;
pm_math_Quaternion_xform_ra ( xx + 438 , xx + 355 , xx + 119 ) ; xx [ 287 ] =
xx [ 283 ] - ( xx [ 98 ] + xx [ 114 ] * xx [ 342 ] - xx [ 119 ] ) ; xx [ 288
] = xx [ 361 ] - ( xx [ 99 ] + xx [ 114 ] * xx [ 364 ] - xx [ 120 ] ) ; xx [
289 ] = xx [ 343 ] - ( xx [ 100 ] + xx [ 114 ] * xx [ 349 ] - xx [ 121 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 438 , xx + 377 , xx + 358 ) ; xx [ 98 ]
= ( xx [ 358 ] * xx [ 360 ] + xx [ 359 ] * xx [ 361 ] ) * xx [ 34 ] ; xx [ 99
] = xx [ 34 ] * ( xx [ 360 ] * xx [ 361 ] - xx [ 358 ] * xx [ 359 ] ) ; xx [
100 ] = xx [ 48 ] - ( xx [ 359 ] * xx [ 359 ] + xx [ 360 ] * xx [ 360 ] ) *
xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx + 426 , xx + 434 , xx + 358 )
; pm_math_Quaternion_compose_ra ( xx + 358 , xx + 778 , xx + 426 ) ; xx [ 113
] = ( xx [ 426 ] * xx [ 428 ] + xx [ 427 ] * xx [ 429 ] ) * xx [ 34 ] ; xx [
114 ] = xx [ 34 ] * ( xx [ 428 ] * xx [ 429 ] - xx [ 426 ] * xx [ 427 ] ) ;
xx [ 115 ] = xx [ 48 ] - ( xx [ 427 ] * xx [ 427 ] + xx [ 428 ] * xx [ 428 ]
) * xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx + 166 , xx + 438 , xx +
358 ) ; pm_math_Quaternion_compose_ra ( xx + 358 , xx + 392 , xx + 166 ) ; xx
[ 87 ] = ( xx [ 166 ] * xx [ 168 ] + xx [ 167 ] * xx [ 169 ] ) * xx [ 34 ] ;
xx [ 88 ] = xx [ 34 ] * ( xx [ 168 ] * xx [ 169 ] - xx [ 166 ] * xx [ 167 ] )
; pm_math_Quaternion_compose_ra ( xx + 197 , xx + 402 , xx + 166 ) ; xx [ 119
] = ( xx [ 166 ] * xx [ 168 ] + xx [ 167 ] * xx [ 169 ] ) * xx [ 34 ] ; xx [
120 ] = xx [ 34 ] * ( xx [ 168 ] * xx [ 169 ] - xx [ 166 ] * xx [ 167 ] ) ;
xx [ 121 ] = xx [ 48 ] - ( xx [ 167 ] * xx [ 167 ] + xx [ 168 ] * xx [ 168 ]
) * xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx + 154 , xx + 782 , xx +
166 ) ; xx [ 152 ] = xx [ 34 ] * ( xx [ 167 ] * xx [ 168 ] - xx [ 166 ] * xx
[ 169 ] ) ; xx [ 153 ] = xx [ 48 ] - ( xx [ 169 ] * xx [ 169 ] + xx [ 167 ] *
xx [ 167 ] ) * xx [ 34 ] ; xx [ 154 ] = ( xx [ 166 ] * xx [ 167 ] + xx [ 168
] * xx [ 169 ] ) * xx [ 34 ] ; xx [ 426 ] = fabs ( pm_math_Vector3_dot_ra (
xx + 130 , xx + 45 ) ) ; xx [ 427 ] = fabs ( pm_math_Vector3_dot_ra ( xx +
185 , xx + 101 ) ) ; xx [ 428 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 19 ,
xx + 104 ) ) ; xx [ 429 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 19 , xx +
107 ) ) ; xx [ 430 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 124 , xx + 54 ) )
; xx [ 431 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 84 , xx + 305 ) ) ; xx [
432 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 68 , xx + 174 ) ) ; xx [ 433 ] =
fabs ( pm_math_Vector3_dot_ra ( xx + 230 , xx + 57 ) ) ; xx [ 434 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 255 , xx + 16 ) ) ; xx [ 435 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 287 , xx + 98 ) ) ; xx [ 436 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 113 , xx + 384 ) ) ; xx [ 437 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 113 , xx + 399 ) ) ; xx [ 438 ] = fabs ( xx [
254 ] * xx [ 87 ] + xx [ 327 ] * xx [ 88 ] ) ; xx [ 439 ] = fabs ( xx [ 254 ]
* xx [ 88 ] - xx [ 327 ] * xx [ 87 ] ) ; xx [ 440 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 119 , xx + 414 ) ) ; xx [ 441 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 423 , xx + 152 ) ) ; ii [ 0 ] = 426 ; { int ll
; for ( ll = 427 ; ll < 442 ; ++ ll ) if ( xx [ ll ] > xx [ ii [ 0 ] ] ) ii [
0 ] = ll ; } ii [ 0 ] -= 426 ; xx [ 16 ] = xx [ 426 + ( ii [ 0 ] ) ] ; xx [
17 ] = 1.0e-9 ; xx [ 18 ] = xx [ 16 ] - xx [ 17 ] ; if ( xx [ 18 ] < 0.0 ) ii
[ 1 ] = - 1 ; else if ( xx [ 18 ] > 0.0 ) ii [ 1 ] = + 1 ; else ii [ 1 ] = 0
; ii [ 2 ] = ii [ 1 ] ; if ( 0 > ii [ 2 ] ) ii [ 2 ] = 0 ; if ( ii [ 2 ] != 0
) { switch ( ii [ 0 ] ) { case 0 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar1' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 1 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Cylindrical' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 2 : case 3 : case 4 : { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 5 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar4' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 6 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar3' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 7 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar2' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 8 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar5' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 9 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar6' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 10 : case 11 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 12 : case 13 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel1' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 14 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel2' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 15 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel3' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } } } xx [ 16 ] = xx [ 5 ] * xx [ 477 ] ; xx [ 18 ] = sin (
xx [ 16 ] ) ; xx [ 84 ] = cos ( xx [ 16 ] ) ; xx [ 85 ] = xx [ 7 ] * xx [ 18
] ; xx [ 86 ] = xx [ 9 ] * xx [ 18 ] ; xx [ 87 ] = xx [ 10 ] * xx [ 18 ] ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 84 , xx + 18 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 11 , xx + 45 ) ; xx [ 11 ] = xx
[ 8 ] * xx [ 503 ] ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 22 , xx +
57 ) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 29 , xx + 68 ) ; xx [ 12
] = xx [ 57 ] - ( xx [ 28 ] * xx [ 478 ] + xx [ 68 ] ) ; xx [ 13 ] = xx [ 38
] * xx [ 503 ] ; xx [ 16 ] = xx [ 58 ] - ( xx [ 40 ] * xx [ 478 ] + xx [ 69 ]
) ; xx [ 29 ] = xx [ 42 ] * xx [ 503 ] ; xx [ 30 ] = xx [ 59 ] + xx [ 44 ] *
xx [ 478 ] - xx [ 70 ] ; xx [ 57 ] = - ( xx [ 11 ] + xx [ 14 ] + xx [ 12 ] -
xx [ 35 ] ) ; xx [ 58 ] = xx [ 37 ] - xx [ 13 ] - ( xx [ 16 ] + xx [ 39 ] ) ;
xx [ 59 ] = xx [ 29 ] + xx [ 43 ] - ( xx [ 30 ] + xx [ 32 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 25 , xx + 68 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 51 , xx + 25 ) ; xx [ 51 ] = xx
[ 68 ] + xx [ 25 ] ; xx [ 52 ] = xx [ 69 ] + xx [ 26 ] ; xx [ 53 ] = xx [ 70
] + xx [ 27 ] ; pm_math_Quaternion_compose_ra ( xx + 18 , xx + 60 , xx + 84 )
; xx [ 60 ] = ( xx [ 84 ] * xx [ 86 ] + xx [ 85 ] * xx [ 87 ] ) * xx [ 34 ] ;
xx [ 61 ] = xx [ 34 ] * ( xx [ 86 ] * xx [ 87 ] - xx [ 84 ] * xx [ 85 ] ) ;
xx [ 62 ] = xx [ 48 ] - ( xx [ 85 ] * xx [ 85 ] + xx [ 86 ] * xx [ 86 ] ) *
xx [ 34 ] ; xx [ 14 ] = xx [ 5 ] * xx [ 481 ] ; xx [ 31 ] = sin ( xx [ 14 ] )
; xx [ 84 ] = cos ( xx [ 14 ] ) ; xx [ 85 ] = - ( xx [ 67 ] * xx [ 31 ] ) ;
xx [ 86 ] = xx [ 79 ] * xx [ 31 ] ; xx [ 87 ] = xx [ 80 ] * xx [ 31 ] ;
pm_math_Quaternion_compose_ra ( xx + 74 , xx + 84 , xx + 152 ) ;
pm_math_Quaternion_compose_ra ( xx + 18 , xx + 152 , xx + 84 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 152 , xx + 89 , xx + 68 ) ;
pm_math_Vector3_cross_ra ( xx + 68 , xx + 95 , xx + 98 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 98 , xx + 113 ) ;
pm_math_Vector3_cross_ra ( xx + 68 , xx + 110 , xx + 98 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 98 , xx + 68 ) ;
pm_math_Quaternion_xform_ra ( xx + 152 , xx + 116 , xx + 98 ) ; xx [ 116 ] =
- ( xx [ 78 ] + xx [ 98 ] ) ; xx [ 117 ] = xx [ 122 ] - xx [ 99 ] ; xx [ 118
] = xx [ 123 ] - xx [ 100 ] ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 116 ,
xx + 98 ) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 98 , xx + 119 ) ; xx
[ 98 ] = xx [ 68 ] + xx [ 119 ] + xx [ 25 ] ; xx [ 99 ] = xx [ 69 ] + xx [
120 ] + xx [ 26 ] ; xx [ 100 ] = xx [ 70 ] + xx [ 121 ] + xx [ 27 ] ; xx [ 14
] = xx [ 5 ] * xx [ 491 ] ; xx [ 31 ] = sin ( xx [ 14 ] ) ; xx [ 119 ] = cos
( xx [ 14 ] ) ; xx [ 120 ] = xx [ 127 ] * xx [ 31 ] ; xx [ 121 ] = xx [ 129 ]
* xx [ 31 ] ; xx [ 122 ] = xx [ 137 ] * xx [ 31 ] ;
pm_math_Quaternion_compose_ra ( xx + 133 , xx + 119 , xx + 123 ) ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 138 , xx + 68 ) ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 149 , xx + 119 ) ; xx [ 130 ] =
xx [ 68 ] + xx [ 119 ] ; xx [ 131 ] = xx [ 69 ] + xx [ 120 ] ; xx [ 132 ] =
xx [ 70 ] + xx [ 121 ] ; xx [ 14 ] = xx [ 5 ] * xx [ 495 ] ; xx [ 31 ] = sin
( xx [ 14 ] ) ; xx [ 138 ] = cos ( xx [ 14 ] ) ; xx [ 139 ] = - ( xx [ 128 ]
* xx [ 31 ] ) ; xx [ 140 ] = xx [ 146 ] * xx [ 31 ] ; xx [ 141 ] = xx [ 147 ]
* xx [ 31 ] ; pm_math_Quaternion_compose_ra ( xx + 158 , xx + 138 , xx + 166
) ; pm_math_Quaternion_compose_ra ( xx + 166 , xx + 162 , xx + 138 ) ; xx [
14 ] = ( xx [ 139 ] * xx [ 139 ] + xx [ 140 ] * xx [ 140 ] ) * xx [ 34 ] ; xx
[ 68 ] = ( xx [ 138 ] * xx [ 140 ] + xx [ 139 ] * xx [ 141 ] ) * xx [ 34 ] ;
xx [ 69 ] = xx [ 34 ] * ( xx [ 140 ] * xx [ 141 ] - xx [ 138 ] * xx [ 139 ] )
; xx [ 70 ] = xx [ 48 ] - xx [ 14 ] ; pm_math_Quaternion_compose_ra ( xx +
181 , xx + 123 , xx + 138 ) ; pm_math_Quaternion_compose_ra ( xx + 138 , xx +
189 , xx + 162 ) ; xx [ 149 ] = ( xx [ 162 ] * xx [ 164 ] + xx [ 163 ] * xx [
165 ] ) * xx [ 34 ] ; xx [ 150 ] = xx [ 34 ] * ( xx [ 164 ] * xx [ 165 ] - xx
[ 162 ] * xx [ 163 ] ) ; xx [ 151 ] = xx [ 48 ] - ( xx [ 163 ] * xx [ 163 ] +
xx [ 164 ] * xx [ 164 ] ) * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx +
138 , xx + 193 , xx + 162 ) ; xx [ 31 ] = xx [ 196 ] * xx [ 505 ] ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 203 , xx + 174 ) ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 210 , xx + 185 ) ; xx [ 32 ] =
xx [ 217 ] - xx [ 492 ] - xx [ 187 ] ; xx [ 187 ] = xx [ 209 ] - xx [ 185 ] ;
xx [ 188 ] = xx [ 216 ] - xx [ 186 ] ; xx [ 189 ] = xx [ 32 ] ;
pm_math_Quaternion_xform_ra ( xx + 181 , xx + 187 , xx + 190 ) ; xx [ 35 ] =
xx [ 174 ] + xx [ 190 ] + xx [ 94 ] * xx [ 489 ] ; xx [ 37 ] = xx [ 221 ] *
xx [ 505 ] ; xx [ 39 ] = xx [ 175 ] + xx [ 191 ] - xx [ 148 ] * xx [ 489 ] ;
xx [ 43 ] = xx [ 227 ] * xx [ 505 ] ; xx [ 63 ] = xx [ 176 ] + xx [ 192 ] +
xx [ 170 ] * xx [ 489 ] ; xx [ 174 ] = xx [ 6 ] - xx [ 31 ] - ( xx [ 35 ] +
xx [ 215 ] ) ; xx [ 175 ] = xx [ 220 ] - xx [ 37 ] - ( xx [ 39 ] + xx [ 225 ]
) ; xx [ 176 ] = xx [ 43 ] - xx [ 228 ] - ( xx [ 63 ] - xx [ 229 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 206 , xx + 187 ) ;
pm_math_Quaternion_xform_ra ( xx + 181 , xx + 119 , xx + 190 ) ; xx [ 193 ] =
xx [ 187 ] + xx [ 190 ] ; xx [ 194 ] = xx [ 188 ] + xx [ 191 ] ; xx [ 195 ] =
xx [ 189 ] + xx [ 192 ] ; xx [ 6 ] = xx [ 5 ] * xx [ 483 ] ; xx [ 78 ] = sin
( xx [ 6 ] ) ; xx [ 187 ] = cos ( xx [ 6 ] ) ; xx [ 188 ] = xx [ 224 ] * xx [
78 ] ; xx [ 189 ] = - ( xx [ 245 ] * xx [ 78 ] ) ; xx [ 190 ] = - ( xx [ 246
] * xx [ 78 ] ) ; pm_math_Quaternion_compose_ra ( xx + 240 , xx + 187 , xx +
197 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 247 , xx + 187 ) ; xx [
6 ] = xx [ 5 ] * xx [ 487 ] ; xx [ 78 ] = sin ( xx [ 6 ] ) ; xx [ 206 ] = cos
( xx [ 6 ] ) ; xx [ 207 ] = - ( xx [ 244 ] * xx [ 78 ] ) ; xx [ 208 ] = xx [
262 ] * xx [ 78 ] ; xx [ 209 ] = xx [ 263 ] * xx [ 78 ] ;
pm_math_Quaternion_compose_ra ( xx + 258 , xx + 206 , xx + 210 ) ;
pm_math_Quaternion_xform_ra ( xx + 210 , xx + 264 , xx + 190 ) ;
pm_math_Quaternion_xform_ra ( xx + 210 , xx + 275 , xx + 206 ) ; xx [ 6 ] =
xx [ 190 ] - xx [ 206 ] ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 268 ,
xx + 214 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 284 , xx + 228 ) ;
xx [ 78 ] = xx [ 214 ] + xx [ 271 ] * xx [ 484 ] - xx [ 228 ] ; xx [ 88 ] =
xx [ 191 ] - xx [ 207 ] ; xx [ 122 ] = xx [ 215 ] + xx [ 294 ] * xx [ 484 ] -
xx [ 229 ] ; xx [ 145 ] = xx [ 192 ] - xx [ 208 ] ; xx [ 156 ] = xx [ 216 ] +
xx [ 278 ] * xx [ 484 ] - xx [ 230 ] ; xx [ 190 ] = xx [ 6 ] + xx [ 250 ] - (
xx [ 78 ] + xx [ 290 ] ) ; xx [ 191 ] = xx [ 88 ] + xx [ 292 ] - ( xx [ 122 ]
+ xx [ 295 ] ) ; xx [ 192 ] = xx [ 145 ] + xx [ 273 ] - ( xx [ 156 ] + xx [
279 ] ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 280 , xx + 206 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 299 , xx + 214 ) ; xx [ 228 ] =
xx [ 206 ] + xx [ 214 ] ; xx [ 229 ] = xx [ 207 ] + xx [ 215 ] ; xx [ 230 ] =
xx [ 208 ] + xx [ 216 ] ; pm_math_Quaternion_compose_ra ( xx + 197 , xx + 308
, xx + 206 ) ; xx [ 231 ] = ( xx [ 206 ] * xx [ 208 ] + xx [ 207 ] * xx [ 209
] ) * xx [ 34 ] ; xx [ 232 ] = xx [ 34 ] * ( xx [ 208 ] * xx [ 209 ] - xx [
206 ] * xx [ 207 ] ) ; xx [ 233 ] = xx [ 48 ] - ( xx [ 207 ] * xx [ 207 ] +
xx [ 208 ] * xx [ 208 ] ) * xx [ 34 ] ; xx [ 157 ] = xx [ 5 ] * xx [ 497 ] ;
xx [ 165 ] = sin ( xx [ 157 ] ) ; xx [ 206 ] = cos ( xx [ 157 ] ) ; xx [ 207
] = xx [ 316 ] * xx [ 165 ] ; xx [ 208 ] = - ( xx [ 318 ] * xx [ 165 ] ) ; xx
[ 209 ] = - ( xx [ 319 ] * xx [ 165 ] ) ; pm_math_Quaternion_compose_ra ( xx
+ 312 , xx + 206 , xx + 247 ) ; pm_math_Quaternion_xform_ra ( xx + 247 , xx +
320 , xx + 206 ) ; xx [ 157 ] = xx [ 5 ] * xx [ 501 ] ; xx [ 165 ] = sin ( xx
[ 157 ] ) ; xx [ 272 ] = cos ( xx [ 157 ] ) ; xx [ 273 ] = - ( xx [ 244 ] *
xx [ 165 ] ) ; xx [ 274 ] = xx [ 262 ] * xx [ 165 ] ; xx [ 275 ] = xx [ 323 ]
* xx [ 165 ] ; pm_math_Quaternion_compose_ra ( xx + 331 , xx + 272 , xx + 279
) ; pm_math_Quaternion_xform_ra ( xx + 279 , xx + 335 , xx + 255 ) ;
pm_math_Quaternion_xform_ra ( xx + 279 , xx + 346 , xx + 272 ) ; xx [ 157 ] =
xx [ 255 ] - xx [ 272 ] ; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 339 ,
xx + 275 ) ; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 355 , xx + 283 ) ;
xx [ 165 ] = xx [ 275 ] + xx [ 342 ] * xx [ 498 ] - xx [ 283 ] ; xx [ 201 ] =
xx [ 256 ] - xx [ 273 ] ; xx [ 202 ] = xx [ 276 ] + xx [ 364 ] * xx [ 498 ] -
xx [ 284 ] ; xx [ 209 ] = xx [ 257 ] - xx [ 274 ] ; xx [ 217 ] = xx [ 277 ] +
xx [ 349 ] * xx [ 498 ] - xx [ 285 ] ; xx [ 255 ] = xx [ 157 ] + xx [ 317 ] -
( xx [ 165 ] + xx [ 317 ] ) ; xx [ 256 ] = xx [ 201 ] + xx [ 362 ] - ( xx [
202 ] + xx [ 362 ] ) ; xx [ 257 ] = xx [ 209 ] + xx [ 344 ] - ( xx [ 217 ] +
xx [ 344 ] ) ; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 350 , xx + 272 )
; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 368 , xx + 275 ) ; xx [ 283 ]
= xx [ 272 ] + xx [ 275 ] ; xx [ 284 ] = xx [ 273 ] + xx [ 276 ] ; xx [ 285 ]
= xx [ 274 ] + xx [ 277 ] ; pm_math_Quaternion_compose_ra ( xx + 247 , xx +
377 , xx + 286 ) ; xx [ 272 ] = ( xx [ 286 ] * xx [ 288 ] + xx [ 287 ] * xx [
289 ] ) * xx [ 34 ] ; xx [ 273 ] = xx [ 34 ] * ( xx [ 288 ] * xx [ 289 ] - xx
[ 286 ] * xx [ 287 ] ) ; xx [ 274 ] = xx [ 48 ] - ( xx [ 287 ] * xx [ 287 ] +
xx [ 288 ] * xx [ 288 ] ) * xx [ 34 ] ; pm_math_Quaternion_compose_ra ( xx +
152 , xx + 197 , xx + 286 ) ; pm_math_Quaternion_compose_ra ( xx + 18 , xx +
286 , xx + 290 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 286 , xx + 89 ,
xx + 295 ) ; pm_math_Vector3_cross_ra ( xx + 295 , xx + 381 , xx + 89 ) ;
pm_math_Quaternion_xform_ra ( xx + 290 , xx + 89 , xx + 286 ) ;
pm_math_Quaternion_compose_ra ( xx + 84 , xx + 197 , xx + 289 ) ;
pm_math_Quaternion_xform_ra ( xx + 289 , xx + 251 , xx + 89 ) ;
pm_math_Quaternion_compose_ra ( xx + 166 , xx + 247 , xx + 295 ) ;
pm_math_Quaternion_xform_ra ( xx + 295 , xx + 324 , xx + 251 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 408 , xx + 299 ) ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 417 , xx + 308 ) ; xx [ 533 ] =
xx [ 0 ] ; xx [ 534 ] = xx [ 0 ] ; xx [ 535 ] = xx [ 0 ] ; xx [ 536 ] = xx [
0 ] ; xx [ 537 ] = xx [ 0 ] ; xx [ 538 ] = xx [ 0 ] ; xx [ 539 ] =
pm_math_Vector3_dot_ra ( xx + 45 , xx + 57 ) - pm_math_Vector3_dot_ra ( xx +
51 , xx + 60 ) ; xx [ 540 ] = - pm_math_Vector3_dot_ra ( xx + 64 , xx + 60 )
; xx [ 541 ] = xx [ 0 ] ; xx [ 542 ] = xx [ 0 ] ; xx [ 543 ] = xx [ 0 ] ; xx
[ 544 ] = xx [ 0 ] ; xx [ 545 ] = xx [ 0 ] ; xx [ 546 ] = xx [ 0 ] ; xx [ 547
] = xx [ 0 ] ; xx [ 548 ] = pm_math_Vector3_dot_ra ( xx + 71 , xx + 60 ) ; xx
[ 549 ] = xx [ 0 ] ; xx [ 550 ] = xx [ 0 ] ; xx [ 551 ] = xx [ 0 ] ; xx [ 552
] = xx [ 0 ] ; xx [ 553 ] = xx [ 0 ] ; xx [ 554 ] = xx [ 0 ] ; xx [ 555 ] =
xx [ 0 ] ; xx [ 556 ] = xx [ 0 ] ; xx [ 557 ] = xx [ 0 ] ; xx [ 558 ] = xx [
0 ] ; xx [ 559 ] = xx [ 0 ] ; xx [ 560 ] = xx [ 0 ] ; xx [ 561 ] = xx [ 0 ] ;
xx [ 562 ] = xx [ 0 ] ; xx [ 563 ] = xx [ 0 ] ; xx [ 564 ] = xx [ 0 ] ; xx [
565 ] = xx [ 49 ] ; xx [ 566 ] = xx [ 0 ] ; xx [ 567 ] = xx [ 0 ] ; xx [ 568
] = xx [ 0 ] ; xx [ 569 ] = xx [ 0 ] ; xx [ 570 ] = xx [ 0 ] ; xx [ 571 ] =
xx [ 0 ] ; xx [ 572 ] = xx [ 0 ] ; xx [ 573 ] = xx [ 0 ] ; xx [ 574 ] = xx [
0 ] ; xx [ 575 ] = xx [ 0 ] ; xx [ 576 ] = xx [ 0 ] ; xx [ 577 ] = xx [ 0 ] ;
xx [ 578 ] = xx [ 0 ] ; xx [ 579 ] = xx [ 0 ] ; xx [ 580 ] = xx [ 0 ] ; xx [
581 ] = xx [ 0 ] ; xx [ 582 ] = xx [ 0 ] ; xx [ 583 ] =
pm_math_Vector3_dot_ra ( xx + 113 , xx + 104 ) ; xx [ 584 ] = xx [ 0 ] ; xx [
585 ] = xx [ 0 ] ; xx [ 586 ] = xx [ 0 ] ; xx [ 587 ] = xx [ 0 ] ; xx [ 588 ]
= xx [ 0 ] ; xx [ 589 ] = xx [ 0 ] ; xx [ 590 ] = xx [ 0 ] ; xx [ 591 ] = xx
[ 0 ] ; xx [ 592 ] = xx [ 0 ] ; xx [ 593 ] = xx [ 0 ] ; xx [ 594 ] = xx [ 0 ]
; xx [ 595 ] = xx [ 0 ] ; xx [ 596 ] = xx [ 0 ] ; xx [ 597 ] = xx [ 0 ] ; xx
[ 598 ] = xx [ 0 ] ; xx [ 599 ] = xx [ 0 ] ; xx [ 600 ] = xx [ 0 ] ; xx [ 601
] = xx [ 0 ] ; xx [ 602 ] = xx [ 0 ] ; xx [ 603 ] = xx [ 0 ] ; xx [ 604 ] =
xx [ 0 ] ; xx [ 605 ] = pm_math_Vector3_dot_ra ( xx + 113 , xx + 107 ) ; xx [
606 ] = xx [ 0 ] ; xx [ 607 ] = xx [ 0 ] ; xx [ 608 ] = xx [ 0 ] ; xx [ 609 ]
= xx [ 0 ] ; xx [ 610 ] = xx [ 0 ] ; xx [ 611 ] = xx [ 0 ] ; xx [ 612 ] = xx
[ 0 ] ; xx [ 613 ] = xx [ 0 ] ; xx [ 614 ] = xx [ 0 ] ; xx [ 615 ] = xx [ 0 ]
; xx [ 616 ] = xx [ 0 ] ; xx [ 617 ] = xx [ 0 ] ; xx [ 618 ] = xx [ 0 ] ; xx
[ 619 ] = xx [ 0 ] ; xx [ 620 ] = xx [ 0 ] ; xx [ 621 ] = xx [ 0 ] ; xx [ 622
] = xx [ 0 ] ; xx [ 623 ] = xx [ 0 ] ; xx [ 624 ] = xx [ 0 ] ; xx [ 625 ] =
xx [ 0 ] ; xx [ 626 ] = xx [ 0 ] ; xx [ 627 ] = pm_math_Vector3_dot_ra ( xx +
98 , xx + 54 ) ; xx [ 628 ] = xx [ 92 ] ; xx [ 629 ] = xx [ 0 ] ; xx [ 630 ]
= xx [ 0 ] ; xx [ 631 ] = xx [ 0 ] ; xx [ 632 ] = xx [ 0 ] ; xx [ 633 ] = xx
[ 0 ] ; xx [ 634 ] = xx [ 0 ] ; xx [ 635 ] = xx [ 0 ] ; xx [ 636 ] = xx [ 0 ]
; xx [ 637 ] = xx [ 0 ] ; xx [ 638 ] = xx [ 0 ] ; xx [ 639 ] = xx [ 0 ] ; xx
[ 640 ] = xx [ 0 ] ; xx [ 641 ] = xx [ 0 ] ; xx [ 642 ] = xx [ 0 ] ; xx [ 643
] = xx [ 0 ] ; xx [ 644 ] = xx [ 0 ] ; xx [ 645 ] = xx [ 0 ] ; xx [ 646 ] =
xx [ 0 ] ; xx [ 647 ] = xx [ 0 ] ; xx [ 648 ] = xx [ 0 ] ; xx [ 649 ] = xx [
0 ] ; xx [ 650 ] = xx [ 0 ] ; xx [ 651 ] = xx [ 0 ] ; xx [ 652 ] = xx [ 0 ] ;
xx [ 653 ] = xx [ 0 ] ; xx [ 654 ] = xx [ 0 ] ; xx [ 655 ] = xx [ 0 ] ; xx [
656 ] = xx [ 0 ] ; xx [ 657 ] = xx [ 0 ] ; xx [ 658 ] = xx [ 0 ] ; xx [ 659 ]
= xx [ 0 ] ; xx [ 660 ] = xx [ 93 ] ; xx [ 661 ] = xx [ 0 ] ; xx [ 662 ] = xx
[ 0 ] ; xx [ 663 ] = xx [ 0 ] ; xx [ 664 ] = xx [ 0 ] ; xx [ 665 ] = xx [ 0 ]
; xx [ 666 ] = xx [ 0 ] ; xx [ 667 ] = xx [ 0 ] ; xx [ 668 ] = xx [ 0 ] ; xx
[ 669 ] = xx [ 0 ] ; xx [ 670 ] = xx [ 0 ] ; xx [ 671 ] = xx [ 0 ] ; xx [ 672
] = xx [ 0 ] ; xx [ 673 ] = xx [ 0 ] ; xx [ 674 ] = xx [ 0 ] ; xx [ 675 ] =
xx [ 0 ] ; xx [ 676 ] = pm_math_Vector3_dot_ra ( xx + 130 , xx + 68 ) ; xx [
677 ] = xx [ 14 ] - xx [ 48 ] ; xx [ 678 ] = xx [ 0 ] ; xx [ 679 ] = xx [ 0 ]
; xx [ 680 ] = xx [ 0 ] ; xx [ 681 ] = xx [ 0 ] ; xx [ 682 ] = xx [ 0 ] ; xx
[ 683 ] = xx [ 0 ] ; xx [ 684 ] = xx [ 0 ] ; xx [ 685 ] = xx [ 0 ] ; xx [ 686
] = xx [ 0 ] ; xx [ 687 ] = xx [ 0 ] ; xx [ 688 ] = xx [ 0 ] ; xx [ 689 ] =
xx [ 0 ] ; xx [ 690 ] = xx [ 0 ] ; xx [ 691 ] = xx [ 0 ] ; xx [ 692 ] = xx [
0 ] ; xx [ 693 ] = xx [ 0 ] ; xx [ 694 ] = xx [ 0 ] ; xx [ 695 ] = xx [ 0 ] ;
xx [ 696 ] = xx [ 0 ] ; xx [ 697 ] = - pm_math_Vector3_dot_ra ( xx + 171 , xx
+ 149 ) ; xx [ 698 ] = pm_math_Vector3_dot_ra ( xx + 162 , xx + 174 ) -
pm_math_Vector3_dot_ra ( xx + 193 , xx + 149 ) ; xx [ 699 ] = -
pm_math_Vector3_dot_ra ( xx + 234 , xx + 149 ) ; xx [ 700 ] = xx [ 0 ] ; xx [
701 ] = xx [ 0 ] ; xx [ 702 ] = xx [ 0 ] ; xx [ 703 ] =
pm_math_Vector3_dot_ra ( xx + 237 , xx + 149 ) ; xx [ 704 ] = xx [ 0 ] ; xx [
705 ] = xx [ 0 ] ; xx [ 706 ] = xx [ 0 ] ; xx [ 707 ] = xx [ 0 ] ; xx [ 708 ]
= xx [ 0 ] ; xx [ 709 ] = xx [ 0 ] ; xx [ 710 ] = xx [ 0 ] ; xx [ 711 ] = xx
[ 0 ] ; xx [ 712 ] = xx [ 0 ] ; xx [ 713 ] = xx [ 0 ] ; xx [ 714 ] = xx [ 0 ]
; xx [ 715 ] = xx [ 0 ] ; xx [ 716 ] = xx [ 0 ] ; xx [ 717 ] =
pm_math_Vector3_dot_ra ( xx + 187 , xx + 190 ) - pm_math_Vector3_dot_ra ( xx
+ 228 , xx + 231 ) ; xx [ 718 ] = - pm_math_Vector3_dot_ra ( xx + 302 , xx +
231 ) ; xx [ 719 ] = xx [ 0 ] ; xx [ 720 ] = xx [ 0 ] ; xx [ 721 ] = xx [ 0 ]
; xx [ 722 ] = xx [ 0 ] ; xx [ 723 ] = xx [ 0 ] ; xx [ 724 ] = xx [ 0 ] ; xx
[ 725 ] = xx [ 0 ] ; xx [ 726 ] = xx [ 0 ] ; xx [ 727 ] = xx [ 0 ] ; xx [ 728
] = xx [ 0 ] ; xx [ 729 ] = xx [ 0 ] ; xx [ 730 ] = xx [ 0 ] ; xx [ 731 ] =
xx [ 0 ] ; xx [ 732 ] = xx [ 0 ] ; xx [ 733 ] = xx [ 0 ] ; xx [ 734 ] = xx [
0 ] ; xx [ 735 ] = xx [ 0 ] ; xx [ 736 ] = xx [ 0 ] ; xx [ 737 ] = xx [ 0 ] ;
xx [ 738 ] = xx [ 0 ] ; xx [ 739 ] = xx [ 0 ] ; xx [ 740 ] = xx [ 0 ] ; xx [
741 ] = xx [ 0 ] ; xx [ 742 ] = xx [ 0 ] ; xx [ 743 ] = xx [ 0 ] ; xx [ 744 ]
= pm_math_Vector3_dot_ra ( xx + 206 , xx + 255 ) - pm_math_Vector3_dot_ra (
xx + 283 , xx + 272 ) ; xx [ 745 ] = - pm_math_Vector3_dot_ra ( xx + 371 , xx
+ 272 ) ; xx [ 746 ] = xx [ 0 ] ; xx [ 747 ] = xx [ 0 ] ; xx [ 748 ] = xx [ 0
] ; xx [ 749 ] = xx [ 0 ] ; xx [ 750 ] = xx [ 0 ] ; xx [ 751 ] = xx [ 0 ] ;
xx [ 752 ] = xx [ 0 ] ; xx [ 753 ] = xx [ 0 ] ; xx [ 754 ] = xx [ 0 ] ; xx [
755 ] = xx [ 0 ] ; xx [ 756 ] = xx [ 0 ] ; xx [ 757 ] = xx [ 0 ] ; xx [ 758 ]
= xx [ 0 ] ; xx [ 759 ] = pm_math_Vector3_dot_ra ( xx + 286 , xx + 384 ) ; xx
[ 760 ] = xx [ 0 ] ; xx [ 761 ] = pm_math_Vector3_dot_ra ( xx + 89 , xx + 384
) ; xx [ 762 ] = xx [ 0 ] ; xx [ 763 ] = xx [ 0 ] ; xx [ 764 ] = xx [ 0 ] ;
xx [ 765 ] = xx [ 0 ] ; xx [ 766 ] = xx [ 0 ] ; xx [ 767 ] = xx [ 0 ] ; xx [
768 ] = xx [ 0 ] ; xx [ 769 ] = xx [ 0 ] ; xx [ 770 ] = xx [ 0 ] ; xx [ 771 ]
= xx [ 0 ] ; xx [ 772 ] = xx [ 0 ] ; xx [ 773 ] = xx [ 0 ] ; xx [ 774 ] = xx
[ 0 ] ; xx [ 775 ] = xx [ 0 ] ; xx [ 776 ] = xx [ 0 ] ; xx [ 777 ] = xx [ 0 ]
; xx [ 778 ] = xx [ 0 ] ; xx [ 779 ] = xx [ 0 ] ; xx [ 780 ] = xx [ 0 ] ; xx
[ 781 ] = pm_math_Vector3_dot_ra ( xx + 286 , xx + 399 ) ; xx [ 782 ] = xx [
0 ] ; xx [ 783 ] = pm_math_Vector3_dot_ra ( xx + 89 , xx + 399 ) ; xx [ 784 ]
= xx [ 0 ] ; xx [ 785 ] = xx [ 0 ] ; xx [ 786 ] = xx [ 0 ] ; xx [ 787 ] = xx
[ 0 ] ; xx [ 788 ] = xx [ 0 ] ; xx [ 789 ] = xx [ 0 ] ; xx [ 790 ] = xx [ 0 ]
; xx [ 791 ] = xx [ 0 ] ; xx [ 792 ] = xx [ 0 ] ; xx [ 793 ] = xx [ 0 ] ; xx
[ 794 ] = xx [ 0 ] ; xx [ 795 ] = xx [ 0 ] ; xx [ 796 ] = xx [ 0 ] ; xx [ 797
] = xx [ 0 ] ; xx [ 798 ] = xx [ 0 ] ; xx [ 799 ] = xx [ 0 ] ; xx [ 800 ] =
xx [ 0 ] ; xx [ 801 ] = xx [ 0 ] ; xx [ 802 ] = xx [ 0 ] ; xx [ 803 ] = xx [
0 ] ; xx [ 804 ] = xx [ 0 ] ; xx [ 805 ] = xx [ 0 ] ; xx [ 806 ] = xx [ 0 ] ;
xx [ 807 ] = xx [ 0 ] ; xx [ 808 ] = xx [ 0 ] ; xx [ 809 ] = xx [ 0 ] ; xx [
810 ] = xx [ 254 ] * xx [ 251 ] + xx [ 327 ] * xx [ 252 ] ; xx [ 811 ] = xx [
0 ] ; xx [ 812 ] = xx [ 0 ] ; xx [ 813 ] = xx [ 0 ] ; xx [ 814 ] = xx [ 0 ] ;
xx [ 815 ] = xx [ 0 ] ; xx [ 816 ] = xx [ 0 ] ; xx [ 817 ] = xx [ 0 ] ; xx [
818 ] = xx [ 0 ] ; xx [ 819 ] = xx [ 0 ] ; xx [ 820 ] = xx [ 0 ] ; xx [ 821 ]
= xx [ 0 ] ; xx [ 822 ] = xx [ 0 ] ; xx [ 823 ] = xx [ 0 ] ; xx [ 824 ] = xx
[ 0 ] ; xx [ 825 ] = xx [ 0 ] ; xx [ 826 ] = xx [ 0 ] ; xx [ 827 ] = xx [ 0 ]
; xx [ 828 ] = xx [ 0 ] ; xx [ 829 ] = xx [ 0 ] ; xx [ 830 ] = xx [ 0 ] ; xx
[ 831 ] = xx [ 0 ] ; xx [ 832 ] = xx [ 254 ] * xx [ 252 ] - xx [ 327 ] * xx [
251 ] ; xx [ 833 ] = xx [ 0 ] ; xx [ 834 ] = xx [ 0 ] ; xx [ 835 ] = xx [ 0 ]
; xx [ 836 ] = xx [ 0 ] ; xx [ 837 ] = xx [ 0 ] ; xx [ 838 ] = xx [ 0 ] ; xx
[ 839 ] = xx [ 0 ] ; xx [ 840 ] = xx [ 0 ] ; xx [ 841 ] = xx [ 0 ] ; xx [ 842
] = xx [ 0 ] ; xx [ 843 ] = xx [ 0 ] ; xx [ 844 ] = xx [ 0 ] ; xx [ 845 ] =
xx [ 0 ] ; xx [ 846 ] = xx [ 0 ] ; xx [ 847 ] = pm_math_Vector3_dot_ra ( xx +
299 , xx + 414 ) ; xx [ 848 ] = xx [ 0 ] ; xx [ 849 ] = xx [ 0 ] ; xx [ 850 ]
= xx [ 0 ] ; xx [ 851 ] = xx [ 0 ] ; xx [ 852 ] = xx [ 0 ] ; xx [ 853 ] = xx
[ 0 ] ; xx [ 854 ] = xx [ 0 ] ; xx [ 855 ] = xx [ 0 ] ; xx [ 856 ] = xx [ 0 ]
; xx [ 857 ] = xx [ 0 ] ; xx [ 858 ] = xx [ 0 ] ; xx [ 859 ] = xx [ 0 ] ; xx
[ 860 ] = xx [ 0 ] ; xx [ 861 ] = xx [ 0 ] ; xx [ 862 ] = xx [ 0 ] ; xx [ 863
] = xx [ 0 ] ; xx [ 864 ] = xx [ 0 ] ; xx [ 865 ] = xx [ 0 ] ; xx [ 866 ] =
xx [ 0 ] ; xx [ 867 ] = xx [ 0 ] ; xx [ 868 ] = xx [ 0 ] ; xx [ 869 ] = xx [
0 ] ; xx [ 870 ] = xx [ 0 ] ; xx [ 871 ] = xx [ 0 ] ; xx [ 872 ] = xx [ 0 ] ;
xx [ 873 ] = xx [ 0 ] ; xx [ 874 ] = pm_math_Vector3_dot_ra ( xx + 308 , xx +
423 ) ; xx [ 875 ] = xx [ 0 ] ; xx [ 876 ] = xx [ 0 ] ; xx [ 877 ] = xx [ 0 ]
; xx [ 878 ] = xx [ 0 ] ; xx [ 879 ] = xx [ 0 ] ; xx [ 880 ] = xx [ 0 ] ; xx
[ 881 ] = xx [ 0 ] ; xx [ 882 ] = xx [ 0 ] ; xx [ 883 ] = xx [ 0 ] ; xx [ 884
] = xx [ 0 ] ; xx [ 45 ] = xx [ 7 ] * xx [ 479 ] ; xx [ 46 ] = xx [ 9 ] * xx
[ 479 ] ; xx [ 47 ] = xx [ 10 ] * xx [ 479 ] ; pm_math_Vector3_cross_ra ( xx
+ 45 , xx + 22 , xx + 51 ) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 51
, xx + 64 ) ; xx [ 0 ] = xx [ 25 ] * xx [ 479 ] - xx [ 28 ] * xx [ 480 ] ; xx
[ 14 ] = xx [ 26 ] * xx [ 479 ] - xx [ 40 ] * xx [ 480 ] ; xx [ 48 ] = xx [
27 ] * xx [ 479 ] + xx [ 44 ] * xx [ 480 ] ; xx [ 51 ] = - ( xx [ 8 ] * xx [
504 ] + xx [ 64 ] + xx [ 0 ] ) ; xx [ 52 ] = - ( xx [ 38 ] * xx [ 504 ] + xx
[ 65 ] + xx [ 14 ] ) ; xx [ 53 ] = xx [ 42 ] * xx [ 504 ] - ( xx [ 66 ] + xx
[ 48 ] ) ; xx [ 64 ] = 3.734971681878978e-4 ; xx [ 65 ] = 0.2610715860287156
; xx [ 66 ] = xx [ 10 ] ; pm_math_Vector3_cross_ra ( xx + 45 , xx + 64 , xx +
71 ) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 71 , xx + 89 ) ; xx [ 49
] = xx [ 94 ] * xx [ 490 ] ; xx [ 71 ] = xx [ 148 ] * xx [ 490 ] ; xx [ 72 ]
= xx [ 170 ] * xx [ 490 ] ; xx [ 98 ] = - xx [ 49 ] ; xx [ 99 ] = xx [ 71 ] ;
xx [ 100 ] = - xx [ 72 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 152 , xx
+ 45 , xx + 113 ) ; xx [ 73 ] = xx [ 67 ] * xx [ 482 ] ; xx [ 92 ] = xx [ 79
] * xx [ 482 ] ; xx [ 93 ] = xx [ 80 ] * xx [ 482 ] ; xx [ 130 ] = xx [ 113 ]
- xx [ 73 ] ; xx [ 131 ] = xx [ 114 ] + xx [ 92 ] ; xx [ 132 ] = xx [ 115 ] +
xx [ 93 ] ; pm_math_Vector3_cross_ra ( xx + 130 , xx + 95 , xx + 113 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 113 , xx + 162 ) ;
pm_math_Vector3_cross_ra ( xx + 130 , xx + 110 , xx + 113 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 113 , xx + 171 ) ; xx [ 113 ] =
1.439701931685288e-6 ; xx [ 114 ] = 0.01494182793334928 ; xx [ 115 ] =
1.536145228271068e-6 ; pm_math_Quaternion_xform_ra ( xx + 152 , xx + 113 , xx
+ 187 ) ; xx [ 113 ] = xx [ 187 ] * xx [ 482 ] ; pm_math_Vector3_cross_ra (
xx + 45 , xx + 116 , xx + 193 ) ; xx [ 114 ] = xx [ 188 ] * xx [ 482 ] ; xx [
115 ] = xx [ 189 ] * xx [ 482 ] ; xx [ 187 ] = xx [ 113 ] + xx [ 193 ] ; xx [
188 ] = xx [ 114 ] + xx [ 194 ] ; xx [ 189 ] = xx [ 115 ] + xx [ 195 ] ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 187 , xx + 193 ) ; xx [ 187 ] =
xx [ 171 ] + xx [ 0 ] + xx [ 193 ] ; xx [ 188 ] = xx [ 172 ] + xx [ 14 ] + xx
[ 194 ] ; xx [ 189 ] = xx [ 173 ] + xx [ 48 ] + xx [ 195 ] ; xx [ 171 ] = xx
[ 36 ] * xx [ 508 ] ; xx [ 172 ] = xx [ 41 ] * xx [ 508 ] ; xx [ 173 ] = - (
xx [ 83 ] * xx [ 508 ] ) ; xx [ 193 ] = xx [ 127 ] * xx [ 493 ] ; xx [ 194 ]
= xx [ 129 ] * xx [ 493 ] ; xx [ 195 ] = xx [ 137 ] * xx [ 493 ] ;
pm_math_Vector3_cross_ra ( xx + 193 , xx + 328 , xx + 206 ) ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 206 , xx + 228 ) ; xx [ 0 ] =
xx [ 119 ] * xx [ 493 ] ; xx [ 206 ] = - ( xx [ 128 ] * xx [ 496 ] ) ; xx [
207 ] = xx [ 146 ] * xx [ 496 ] ; xx [ 208 ] = xx [ 147 ] * xx [ 496 ] ;
pm_math_Vector3_cross_ra ( xx + 206 , xx + 142 , xx + 234 ) ;
pm_math_Quaternion_xform_ra ( xx + 166 , xx + 234 , xx + 237 ) ; xx [ 234 ] =
1.439701935798469e-6 ; xx [ 235 ] = 0.01494182793334727 ; xx [ 236 ] =
1.53614523658561e-6 ; pm_math_Quaternion_xform_ra ( xx + 166 , xx + 234 , xx
+ 251 ) ; xx [ 14 ] = xx [ 237 ] + xx [ 251 ] * xx [ 496 ] ; xx [ 48 ] = xx [
120 ] * xx [ 493 ] ; xx [ 218 ] = xx [ 238 ] + xx [ 252 ] * xx [ 496 ] ; xx [
220 ] = xx [ 121 ] * xx [ 493 ] - xx [ 494 ] ; xx [ 223 ] = xx [ 239 ] + xx [
253 ] * xx [ 496 ] ; xx [ 234 ] = xx [ 228 ] + xx [ 0 ] - xx [ 14 ] ; xx [
235 ] = xx [ 229 ] + xx [ 48 ] - xx [ 218 ] ; xx [ 236 ] = xx [ 230 ] + xx [
220 ] - xx [ 223 ] ; pm_math_Quaternion_xform_ra ( xx + 123 , xx + 328 , xx +
228 ) ; pm_math_Quaternion_xform_ra ( xx + 166 , xx + 142 , xx + 237 ) ;
pm_math_Quaternion_xform_ra ( xx + 166 , xx + 396 , xx + 142 ) ; xx [ 251 ] =
xx [ 228 ] - xx [ 185 ] - ( xx [ 237 ] - xx [ 142 ] ) + xx [ 226 ] ; xx [ 252
] = xx [ 229 ] - xx [ 186 ] - ( xx [ 238 ] - xx [ 143 ] ) - xx [ 354 ] ; xx [
253 ] = xx [ 230 ] + xx [ 32 ] - ( xx [ 239 ] - xx [ 144 ] ) - xx [ 388 ] ;
xx [ 142 ] = - 0.9972896958009461 ; xx [ 143 ] = 8.852854518104358e-5 ; xx [
144 ] = 0.07357482457984132 ; pm_math_Vector3_cross_ra ( xx + 206 , xx + 142
, xx + 228 ) ; pm_math_Quaternion_xform_ra ( xx + 166 , xx + 228 , xx + 142 )
; xx [ 32 ] = pm_math_Vector3_dot_ra ( xx + 251 , xx + 142 ) ;
pm_math_Vector3_cross_ra ( xx + 193 , xx + 203 , xx + 142 ) ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 142 , xx + 166 ) ; xx [ 142 ] =
xx [ 0 ] ; xx [ 143 ] = xx [ 48 ] ; xx [ 144 ] = xx [ 220 ] ;
pm_math_Quaternion_xform_ra ( xx + 181 , xx + 142 , xx + 228 ) ; xx [ 142 ] =
- ( xx [ 196 ] * xx [ 506 ] + xx [ 166 ] + xx [ 49 ] + xx [ 228 ] ) ; xx [
143 ] = - ( xx [ 221 ] * xx [ 506 ] + xx [ 167 ] + xx [ 229 ] - xx [ 71 ] ) ;
xx [ 144 ] = xx [ 227 ] * xx [ 506 ] - ( xx [ 168 ] + xx [ 72 ] + xx [ 230 ]
) ; xx [ 166 ] = 3.734971681886328e-4 ; xx [ 167 ] = 0.2610715860287173 ; xx
[ 168 ] = xx [ 137 ] ; pm_math_Vector3_cross_ra ( xx + 193 , xx + 166 , xx +
228 ) ; pm_math_Quaternion_xform_ra ( xx + 138 , xx + 228 , xx + 237 ) ; xx [
228 ] = - ( xx [ 244 ] * xx [ 488 ] ) ; xx [ 229 ] = xx [ 262 ] * xx [ 488 ]
; xx [ 230 ] = xx [ 263 ] * xx [ 488 ] ; pm_math_Vector3_cross_ra ( xx + 228
, xx + 264 , xx + 251 ) ; pm_math_Quaternion_xform_ra ( xx + 210 , xx + 251 ,
xx + 228 ) ; xx [ 251 ] = 2.971385142511557e-7 ; xx [ 252 ] =
1.158326953518261e-13 ; xx [ 253 ] = - 3.938183989403888e-8 ;
pm_math_Quaternion_xform_ra ( xx + 210 , xx + 251 , xx + 264 ) ; xx [ 0 ] =
xx [ 228 ] + xx [ 264 ] * xx [ 488 ] ; xx [ 48 ] = xx [ 224 ] * xx [ 485 ] ;
xx [ 49 ] = xx [ 245 ] * xx [ 485 ] ; xx [ 71 ] = xx [ 246 ] * xx [ 485 ] ;
xx [ 210 ] = xx [ 48 ] ; xx [ 211 ] = - xx [ 49 ] ; xx [ 212 ] = - xx [ 71 ]
; pm_math_Vector3_cross_ra ( xx + 210 , xx + 268 , xx + 251 ) ;
pm_math_Quaternion_xform_ra ( xx + 197 , xx + 251 , xx + 283 ) ; xx [ 72 ] =
xx [ 229 ] + xx [ 265 ] * xx [ 488 ] ; xx [ 169 ] = xx [ 230 ] + xx [ 266 ] *
xx [ 488 ] ; xx [ 228 ] = xx [ 0 ] - ( xx [ 283 ] + xx [ 214 ] * xx [ 485 ] +
xx [ 271 ] * xx [ 486 ] ) ; xx [ 229 ] = xx [ 72 ] - ( xx [ 284 ] + xx [ 215
] * xx [ 485 ] + xx [ 294 ] * xx [ 486 ] ) ; xx [ 230 ] = xx [ 169 ] - ( xx [
285 ] + xx [ 216 ] * xx [ 485 ] + xx [ 278 ] * xx [ 486 ] ) ; xx [ 185 ] =
0.9999868227746884 ; xx [ 251 ] = - 3.059547932262596e-4 ; xx [ 252 ] = xx [
185 ] ; xx [ 253 ] = 5.124516430675907e-3 ; pm_math_Vector3_cross_ra ( xx +
210 , xx + 251 , xx + 264 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx +
264 , xx + 210 ) ; xx [ 264 ] = - ( xx [ 244 ] * xx [ 502 ] ) ; xx [ 265 ] =
xx [ 262 ] * xx [ 502 ] ; xx [ 266 ] = xx [ 323 ] * xx [ 502 ] ;
pm_math_Vector3_cross_ra ( xx + 264 , xx + 335 , xx + 283 ) ;
pm_math_Quaternion_xform_ra ( xx + 279 , xx + 283 , xx + 264 ) ; xx [ 283 ] =
2.971385144603623e-7 ; xx [ 284 ] = 1.158326948378051e-13 ; xx [ 285 ] = -
3.938183964273754e-8 ; pm_math_Quaternion_xform_ra ( xx + 279 , xx + 283 , xx
+ 286 ) ; xx [ 186 ] = xx [ 264 ] + xx [ 286 ] * xx [ 502 ] ; xx [ 213 ] = xx
[ 316 ] * xx [ 499 ] ; xx [ 220 ] = xx [ 318 ] * xx [ 499 ] ; xx [ 225 ] = xx
[ 319 ] * xx [ 499 ] ; xx [ 279 ] = xx [ 213 ] ; xx [ 280 ] = - xx [ 220 ] ;
xx [ 281 ] = - xx [ 225 ] ; pm_math_Vector3_cross_ra ( xx + 279 , xx + 339 ,
xx + 282 ) ; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 282 , xx + 299 ) ;
xx [ 226 ] = xx [ 265 ] + xx [ 287 ] * xx [ 502 ] ; xx [ 264 ] = xx [ 266 ] +
xx [ 288 ] * xx [ 502 ] ; xx [ 265 ] = xx [ 186 ] - ( xx [ 299 ] + xx [ 275 ]
* xx [ 499 ] + xx [ 342 ] * xx [ 500 ] ) ; xx [ 266 ] = xx [ 226 ] - ( xx [
300 ] + xx [ 276 ] * xx [ 499 ] + xx [ 364 ] * xx [ 500 ] ) ; xx [ 267 ] = xx
[ 264 ] - ( xx [ 301 ] + xx [ 277 ] * xx [ 499 ] + xx [ 349 ] * xx [ 500 ] )
; xx [ 282 ] = - 3.059547932248718e-4 ; xx [ 283 ] = xx [ 185 ] ; xx [ 284 ]
= xx [ 246 ] ; pm_math_Vector3_cross_ra ( xx + 279 , xx + 282 , xx + 285 ) ;
pm_math_Quaternion_xform_ra ( xx + 247 , xx + 285 , xx + 279 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 197 , xx + 130 , xx + 285 ) ; xx [
130 ] = xx [ 285 ] + xx [ 48 ] ; xx [ 131 ] = xx [ 286 ] - xx [ 49 ] ; xx [
132 ] = xx [ 287 ] - xx [ 71 ] ; pm_math_Vector3_cross_ra ( xx + 130 , xx +
381 , xx + 285 ) ; pm_math_Quaternion_xform_ra ( xx + 289 , xx + 285 , xx +
130 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 247 , xx + 206 , xx + 285 )
; xx [ 206 ] = xx [ 285 ] + xx [ 213 ] ; xx [ 207 ] = xx [ 286 ] - xx [ 220 ]
; xx [ 208 ] = xx [ 287 ] - xx [ 225 ] ; xx [ 299 ] = - 0.03229790576921281 ;
xx [ 300 ] = - 5.131724710788964e-3 ; xx [ 301 ] = 0.9994651122897766 ;
pm_math_Vector3_cross_ra ( xx + 206 , xx + 299 , xx + 302 ) ;
pm_math_Quaternion_xform_ra ( xx + 295 , xx + 302 , xx + 206 ) ; xx [ 302 ] =
1.81142804615305e-4 ; xx [ 303 ] = - xx [ 353 ] ; xx [ 304 ] =
0.2610715346452743 ; pm_math_Vector3_cross_ra ( xx + 45 , xx + 302 , xx + 308
) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 308 , xx + 45 ) ; xx [ 308 ]
= 0.9999999138435713 ; xx [ 309 ] = 7.735117927198942e-5 ; xx [ 310 ] = -
4.078353164850834e-4 ; pm_math_Vector3_cross_ra ( xx + 193 , xx + 308 , xx +
320 ) ; pm_math_Quaternion_xform_ra ( xx + 138 , xx + 320 , xx + 193 ) ; xx [
365 ] = - ( pm_math_Vector3_dot_ra ( xx + 51 , xx + 60 ) +
pm_math_Vector3_dot_ra ( xx + 57 , xx + 89 ) ) ; xx [ 366 ] = -
pm_math_Vector3_dot_ra ( xx + 98 , xx + 101 ) ; xx [ 367 ] = -
pm_math_Vector3_dot_ra ( xx + 162 , xx + 104 ) ; xx [ 368 ] = -
pm_math_Vector3_dot_ra ( xx + 162 , xx + 107 ) ; xx [ 369 ] = -
pm_math_Vector3_dot_ra ( xx + 187 , xx + 54 ) ; xx [ 370 ] = -
pm_math_Vector3_dot_ra ( xx + 171 , xx + 305 ) ; xx [ 371 ] = - (
pm_math_Vector3_dot_ra ( xx + 234 , xx + 68 ) + xx [ 32 ] ) ; xx [ 372 ] = -
( pm_math_Vector3_dot_ra ( xx + 142 , xx + 149 ) + pm_math_Vector3_dot_ra (
xx + 174 , xx + 237 ) ) ; xx [ 373 ] = - ( pm_math_Vector3_dot_ra ( xx + 228
, xx + 231 ) + pm_math_Vector3_dot_ra ( xx + 190 , xx + 210 ) ) ; xx [ 374 ]
= - ( pm_math_Vector3_dot_ra ( xx + 265 , xx + 272 ) + pm_math_Vector3_dot_ra
( xx + 255 , xx + 279 ) ) ; xx [ 375 ] = - pm_math_Vector3_dot_ra ( xx + 130
, xx + 384 ) ; xx [ 376 ] = - pm_math_Vector3_dot_ra ( xx + 130 , xx + 399 )
; xx [ 377 ] = - ( xx [ 254 ] * xx [ 206 ] + xx [ 327 ] * xx [ 207 ] ) ; xx [
378 ] = - ( xx [ 254 ] * xx [ 207 ] - xx [ 327 ] * xx [ 206 ] ) ; xx [ 379 ]
= - pm_math_Vector3_dot_ra ( xx + 45 , xx + 414 ) ; xx [ 380 ] = -
pm_math_Vector3_dot_ra ( xx + 423 , xx + 193 ) ; memcpy ( xx + 885 , xx + 533
, 352 * sizeof ( double ) ) ; factorAndSolveWide ( 16 , 22 , xx + 885 , xx +
448 , xx + 1237 , ii + 0 , xx + 365 , xx [ 15 ] , xx + 426 ) ; xx [ 15 ] = xx
[ 479 ] + xx [ 432 ] ; xx [ 45 ] = xx [ 480 ] + xx [ 433 ] ; xx [ 46 ] = xx [
485 ] + xx [ 434 ] ; xx [ 47 ] = xx [ 486 ] + xx [ 435 ] ; xx [ 48 ] = xx [
490 ] + xx [ 436 ] ; xx [ 49 ] = xx [ 493 ] + xx [ 437 ] ; xx [ 51 ] = xx [
494 ] + xx [ 438 ] ; xx [ 52 ] = xx [ 499 ] + xx [ 439 ] ; xx [ 53 ] = xx [
500 ] + xx [ 440 ] ; xx [ 57 ] = xx [ 504 ] + xx [ 441 ] ; xx [ 58 ] = xx [
506 ] + xx [ 442 ] ; xx [ 59 ] = xx [ 508 ] + xx [ 443 ] ; xx [ 533 ] = xx [
464 ] ; xx [ 534 ] = xx [ 465 ] ; xx [ 535 ] = xx [ 466 ] ; xx [ 536 ] = xx [
467 ] ; xx [ 537 ] = xx [ 468 ] ; xx [ 538 ] = xx [ 469 ] ; xx [ 539 ] = xx [
470 ] ; xx [ 540 ] = xx [ 471 ] + xx [ 426 ] ; xx [ 541 ] = xx [ 472 ] + xx [
427 ] ; xx [ 542 ] = xx [ 473 ] + xx [ 428 ] ; xx [ 543 ] = xx [ 474 ] + xx [
429 ] ; xx [ 544 ] = xx [ 475 ] + xx [ 430 ] ; xx [ 545 ] = xx [ 476 ] + xx [
431 ] ; xx [ 546 ] = xx [ 477 ] ; xx [ 547 ] = xx [ 478 ] ; xx [ 548 ] = xx [
15 ] ; xx [ 549 ] = xx [ 45 ] ; xx [ 550 ] = xx [ 481 ] ; xx [ 551 ] = xx [
482 ] ; xx [ 552 ] = xx [ 483 ] ; xx [ 553 ] = xx [ 484 ] ; xx [ 554 ] = xx [
46 ] ; xx [ 555 ] = xx [ 47 ] ; xx [ 556 ] = xx [ 487 ] ; xx [ 557 ] = xx [
488 ] ; xx [ 558 ] = xx [ 489 ] ; xx [ 559 ] = xx [ 48 ] ; xx [ 560 ] = xx [
491 ] ; xx [ 561 ] = xx [ 492 ] ; xx [ 562 ] = xx [ 49 ] ; xx [ 563 ] = xx [
51 ] ; xx [ 564 ] = xx [ 495 ] ; xx [ 565 ] = xx [ 496 ] ; xx [ 566 ] = xx [
497 ] ; xx [ 567 ] = xx [ 498 ] ; xx [ 568 ] = xx [ 52 ] ; xx [ 569 ] = xx [
53 ] ; xx [ 570 ] = xx [ 501 ] ; xx [ 571 ] = xx [ 502 ] ; xx [ 572 ] = xx [
503 ] ; xx [ 573 ] = xx [ 57 ] ; xx [ 574 ] = xx [ 505 ] ; xx [ 575 ] = xx [
58 ] ; xx [ 576 ] = xx [ 507 ] ; xx [ 577 ] = xx [ 59 ] ; xx [ 578 ] = xx [
509 ] ; xx [ 579 ] = xx [ 510 ] + xx [ 444 ] ; xx [ 580 ] = xx [ 511 ] ; xx [
581 ] = xx [ 512 ] + xx [ 445 ] ; xx [ 582 ] = xx [ 513 ] ; xx [ 583 ] = xx [
514 ] + xx [ 446 ] ; xx [ 584 ] = xx [ 515 ] ; xx [ 585 ] = xx [ 516 ] + xx [
447 ] ; xx [ 586 ] = xx [ 517 ] ; xx [ 587 ] = xx [ 518 ] ; xx [ 588 ] = xx [
519 ] ; xx [ 589 ] = xx [ 520 ] ; xx [ 590 ] = xx [ 521 ] ; xx [ 591 ] = xx [
522 ] ; xx [ 592 ] = xx [ 523 ] ; xx [ 593 ] = xx [ 524 ] ; xx [ 594 ] = xx [
525 ] ; xx [ 595 ] = xx [ 526 ] ; xx [ 596 ] = xx [ 527 ] ; xx [ 597 ] = xx [
528 ] ; xx [ 598 ] = xx [ 529 ] ; xx [ 599 ] = xx [ 530 ] ; xx [ 600 ] = xx [
531 ] ; xx [ 601 ] = xx [ 532 ] ; xx [ 89 ] = xx [ 15 ] * xx [ 7 ] ; xx [ 90
] = xx [ 15 ] * xx [ 9 ] ; xx [ 91 ] = xx [ 15 ] * xx [ 10 ] ;
pm_math_Vector3_cross_ra ( xx + 89 , xx + 22 , xx + 98 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 98 , xx + 22 ) ; xx [ 71 ] = xx
[ 15 ] * xx [ 25 ] - xx [ 45 ] * xx [ 28 ] ; xx [ 25 ] = xx [ 15 ] * xx [ 26
] - xx [ 45 ] * xx [ 40 ] ; xx [ 26 ] = xx [ 15 ] * xx [ 27 ] + xx [ 45 ] *
xx [ 44 ] ; xx [ 98 ] = - ( xx [ 57 ] * xx [ 8 ] + xx [ 22 ] + xx [ 71 ] ) ;
xx [ 99 ] = - ( xx [ 57 ] * xx [ 38 ] + xx [ 23 ] + xx [ 25 ] ) ; xx [ 100 ]
= xx [ 57 ] * xx [ 42 ] - ( xx [ 24 ] + xx [ 26 ] ) ; xx [ 22 ] =
6.799693692804809e-3 - ( xx [ 11 ] + xx [ 12 ] ) ; xx [ 23 ] =
8.001857989620565e-3 - ( xx [ 13 ] + xx [ 16 ] ) ; xx [ 24 ] = xx [ 29 ] - xx
[ 30 ] - 5.681474032069526e-3 ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 64
, xx + 11 ) ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 11 , xx + 27 ) ;
xx [ 8 ] = xx [ 48 ] * xx [ 94 ] ; xx [ 11 ] = xx [ 48 ] * xx [ 148 ] ; xx [
12 ] = xx [ 48 ] * xx [ 170 ] ; xx [ 64 ] = - xx [ 8 ] ; xx [ 65 ] = xx [ 11
] ; xx [ 66 ] = - xx [ 12 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 152 ,
xx + 89 , xx + 130 ) ; xx [ 142 ] = xx [ 130 ] - xx [ 73 ] ; xx [ 143 ] = xx
[ 131 ] + xx [ 92 ] ; xx [ 144 ] = xx [ 132 ] + xx [ 93 ] ;
pm_math_Vector3_cross_ra ( xx + 142 , xx + 95 , xx + 92 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 92 , xx + 95 ) ;
pm_math_Vector3_cross_ra ( xx + 142 , xx + 110 , xx + 92 ) ;
pm_math_Quaternion_xform_ra ( xx + 84 , xx + 92 , xx + 110 ) ;
pm_math_Vector3_cross_ra ( xx + 89 , xx + 116 , xx + 84 ) ; xx [ 92 ] = xx [
113 ] + xx [ 84 ] ; xx [ 93 ] = xx [ 114 ] + xx [ 85 ] ; xx [ 94 ] = xx [ 115
] + xx [ 86 ] ; pm_math_Quaternion_xform_ra ( xx + 18 , xx + 92 , xx + 84 ) ;
xx [ 92 ] = xx [ 110 ] + xx [ 71 ] + xx [ 84 ] ; xx [ 93 ] = xx [ 111 ] + xx
[ 25 ] + xx [ 85 ] ; xx [ 94 ] = xx [ 112 ] + xx [ 26 ] + xx [ 86 ] ; xx [ 84
] = xx [ 59 ] * xx [ 36 ] ; xx [ 85 ] = xx [ 59 ] * xx [ 41 ] ; xx [ 86 ] = -
( xx [ 59 ] * xx [ 83 ] ) ; xx [ 40 ] = xx [ 49 ] * xx [ 127 ] ; xx [ 41 ] =
xx [ 49 ] * xx [ 129 ] ; xx [ 42 ] = xx [ 49 ] * xx [ 137 ] ;
pm_math_Vector3_cross_ra ( xx + 40 , xx + 328 , xx + 110 ) ;
pm_math_Quaternion_xform_ra ( xx + 123 , xx + 110 , xx + 113 ) ; xx [ 13 ] =
xx [ 49 ] * xx [ 119 ] ; xx [ 15 ] = xx [ 49 ] * xx [ 120 ] ; xx [ 16 ] = xx
[ 49 ] * xx [ 121 ] - xx [ 51 ] ; xx [ 110 ] = xx [ 113 ] + xx [ 13 ] - xx [
14 ] ; xx [ 111 ] = xx [ 114 ] + xx [ 15 ] - xx [ 218 ] ; xx [ 112 ] = xx [
115 ] + xx [ 16 ] - xx [ 223 ] ; pm_math_Vector3_cross_ra ( xx + 40 , xx +
203 , xx + 113 ) ; pm_math_Quaternion_xform_ra ( xx + 138 , xx + 113 , xx +
116 ) ; xx [ 113 ] = xx [ 13 ] ; xx [ 114 ] = xx [ 15 ] ; xx [ 115 ] = xx [
16 ] ; pm_math_Quaternion_xform_ra ( xx + 181 , xx + 113 , xx + 13 ) ; xx [
113 ] = - ( xx [ 58 ] * xx [ 196 ] + xx [ 116 ] + xx [ 8 ] + xx [ 13 ] ) ; xx
[ 114 ] = - ( xx [ 58 ] * xx [ 221 ] + xx [ 117 ] + xx [ 14 ] - xx [ 11 ] ) ;
xx [ 115 ] = xx [ 58 ] * xx [ 227 ] - ( xx [ 118 ] + xx [ 12 ] + xx [ 15 ] )
; xx [ 11 ] = - ( xx [ 31 ] + xx [ 35 ] + 0.02202261164176719 ) ; xx [ 12 ] =
0.01173563242948918 - ( xx [ 37 ] + xx [ 39 ] ) ; xx [ 13 ] = xx [ 43 ] - xx
[ 63 ] + 0.02331844835879257 ; pm_math_Vector3_cross_ra ( xx + 40 , xx + 166
, xx + 14 ) ; pm_math_Quaternion_xform_ra ( xx + 138 , xx + 14 , xx + 35 ) ;
xx [ 8 ] = xx [ 46 ] * xx [ 224 ] ; xx [ 14 ] = xx [ 46 ] * xx [ 245 ] ; xx [
15 ] = xx [ 46 ] * xx [ 246 ] ; xx [ 43 ] = xx [ 8 ] ; xx [ 44 ] = - xx [ 14
] ; xx [ 45 ] = - xx [ 15 ] ; pm_math_Vector3_cross_ra ( xx + 43 , xx + 268 ,
xx + 57 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 57 , xx + 116 ) ;
xx [ 57 ] = xx [ 0 ] - ( xx [ 116 ] + xx [ 46 ] * xx [ 214 ] + xx [ 47 ] * xx
[ 271 ] ) ; xx [ 58 ] = xx [ 72 ] - ( xx [ 117 ] + xx [ 46 ] * xx [ 215 ] +
xx [ 47 ] * xx [ 294 ] ) ; xx [ 59 ] = xx [ 169 ] - ( xx [ 118 ] + xx [ 46 ]
* xx [ 216 ] + xx [ 47 ] * xx [ 278 ] ) ; xx [ 46 ] = xx [ 6 ] - xx [ 78 ] -
xx [ 33 ] ; xx [ 47 ] = xx [ 88 ] - xx [ 122 ] - xx [ 81 ] ; xx [ 48 ] = xx [
145 ] - xx [ 156 ] - xx [ 82 ] ; pm_math_Vector3_cross_ra ( xx + 43 , xx +
251 , xx + 71 ) ; pm_math_Quaternion_xform_ra ( xx + 197 , xx + 71 , xx + 43
) ; xx [ 0 ] = xx [ 52 ] * xx [ 316 ] ; xx [ 6 ] = xx [ 52 ] * xx [ 318 ] ;
xx [ 16 ] = xx [ 52 ] * xx [ 319 ] ; xx [ 71 ] = xx [ 0 ] ; xx [ 72 ] = - xx
[ 6 ] ; xx [ 73 ] = - xx [ 16 ] ; pm_math_Vector3_cross_ra ( xx + 71 , xx +
339 , xx + 81 ) ; pm_math_Quaternion_xform_ra ( xx + 247 , xx + 81 , xx + 116
) ; xx [ 81 ] = xx [ 186 ] - ( xx [ 116 ] + xx [ 52 ] * xx [ 275 ] + xx [ 53
] * xx [ 342 ] ) ; xx [ 82 ] = xx [ 226 ] - ( xx [ 117 ] + xx [ 52 ] * xx [
276 ] + xx [ 53 ] * xx [ 364 ] ) ; xx [ 83 ] = xx [ 264 ] - ( xx [ 118 ] + xx
[ 52 ] * xx [ 277 ] + xx [ 53 ] * xx [ 349 ] ) ; xx [ 51 ] = xx [ 157 ] - xx
[ 165 ] ; xx [ 52 ] = xx [ 201 ] - xx [ 202 ] ; xx [ 53 ] = xx [ 209 ] - xx [
217 ] ; pm_math_Vector3_cross_ra ( xx + 71 , xx + 282 , xx + 116 ) ;
pm_math_Quaternion_xform_ra ( xx + 247 , xx + 116 , xx + 71 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 197 , xx + 142 , xx + 116 ) ; xx [
119 ] = xx [ 116 ] + xx [ 8 ] ; xx [ 120 ] = xx [ 117 ] - xx [ 14 ] ; xx [
121 ] = xx [ 118 ] - xx [ 15 ] ; pm_math_Vector3_cross_ra ( xx + 119 , xx +
381 , xx + 116 ) ; pm_math_Quaternion_xform_ra ( xx + 289 , xx + 116 , xx +
119 ) ; xx [ 116 ] = xx [ 285 ] + xx [ 0 ] ; xx [ 117 ] = xx [ 286 ] - xx [ 6
] ; xx [ 118 ] = xx [ 287 ] - xx [ 16 ] ; pm_math_Vector3_cross_ra ( xx + 116
, xx + 299 , xx + 14 ) ; pm_math_Quaternion_xform_ra ( xx + 295 , xx + 14 ,
xx + 116 ) ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 302 , xx + 14 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 14 , xx + 87 ) ;
pm_math_Vector3_cross_ra ( xx + 40 , xx + 308 , xx + 14 ) ;
pm_math_Quaternion_xform_ra ( xx + 138 , xx + 14 , xx + 18 ) ; xx [ 185 ] =
fabs ( pm_math_Vector3_dot_ra ( xx + 98 , xx + 60 ) + pm_math_Vector3_dot_ra
( xx + 22 , xx + 27 ) ) ; xx [ 186 ] = fabs ( pm_math_Vector3_dot_ra ( xx +
64 , xx + 101 ) ) ; xx [ 187 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 95 , xx
+ 104 ) ) ; xx [ 188 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 95 , xx + 107 )
) ; xx [ 189 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 92 , xx + 54 ) ) ; xx [
190 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 84 , xx + 305 ) ) ; xx [ 191 ] =
fabs ( pm_math_Vector3_dot_ra ( xx + 110 , xx + 68 ) + xx [ 32 ] ) ; xx [ 192
] = fabs ( pm_math_Vector3_dot_ra ( xx + 113 , xx + 149 ) +
pm_math_Vector3_dot_ra ( xx + 11 , xx + 35 ) ) ; xx [ 193 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 57 , xx + 231 ) + pm_math_Vector3_dot_ra ( xx +
46 , xx + 43 ) ) ; xx [ 194 ] = fabs ( pm_math_Vector3_dot_ra ( xx + 81 , xx
+ 272 ) + pm_math_Vector3_dot_ra ( xx + 51 , xx + 71 ) ) ; xx [ 195 ] = fabs
( pm_math_Vector3_dot_ra ( xx + 119 , xx + 384 ) ) ; xx [ 196 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 119 , xx + 399 ) ) ; xx [ 197 ] = fabs ( xx [
254 ] * xx [ 116 ] + xx [ 327 ] * xx [ 117 ] ) ; xx [ 198 ] = fabs ( xx [ 254
] * xx [ 117 ] - xx [ 327 ] * xx [ 116 ] ) ; xx [ 199 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 87 , xx + 414 ) ) ; xx [ 200 ] = fabs (
pm_math_Vector3_dot_ra ( xx + 423 , xx + 18 ) ) ; ii [ 0 ] = 185 ; { int ll ;
for ( ll = 186 ; ll < 201 ; ++ ll ) if ( xx [ ll ] > xx [ ii [ 0 ] ] ) ii [ 0
] = ll ; } ii [ 0 ] -= 185 ; xx [ 0 ] = xx [ 185 + ( ii [ 0 ] ) ] ; xx [ 6 ]
= xx [ 0 ] - xx [ 17 ] ; if ( xx [ 6 ] < 0.0 ) ii [ 1 ] = - 1 ; else if ( xx
[ 6 ] > 0.0 ) ii [ 1 ] = + 1 ; else ii [ 1 ] = 0 ; ii [ 2 ] = ii [ 1 ] ; if (
0 > ii [ 2 ] ) ii [ 2 ] = 0 ; if ( ii [ 2 ] != 0 ) { switch ( ii [ 0 ] ) {
case 0 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar1' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 1 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Cylindrical' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 2 : case 3 : case 4 : { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 5 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar4' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 6 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar3' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 7 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar2' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 8 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar5' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 9 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Planar6' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 10 : case 11 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 12 : case 13 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel1' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 14 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel2' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } case 15 : { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:ConstraintViolation" ,
 "'Control_Bicopter/Subsystem/Parallel3' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem."
, neDiagMgr ) ; } } } xx [ 11 ] = - 0.9915516631459343 ; xx [ 12 ] =
0.1297122054176751 ; xx [ 13 ] = - 1.971782056416126e-4 ; xx [ 14 ] = -
6.47994571923356e-5 ; xx [ 0 ] = xx [ 5 ] * xx [ 546 ] ; xx [ 6 ] = sin ( xx
[ 0 ] ) ; xx [ 15 ] = cos ( xx [ 0 ] ) ; xx [ 16 ] = xx [ 7 ] * xx [ 6 ] ; xx
[ 17 ] = xx [ 9 ] * xx [ 6 ] ; xx [ 18 ] = xx [ 10 ] * xx [ 6 ] ;
pm_math_Quaternion_compose_ra ( xx + 1 , xx + 15 , xx + 19 ) ; xx [ 0 ] = -
0.9228737611163665 ; xx [ 1 ] = - 0.02263515933877589 ; xx [ 2 ] =
0.3826221044509303 ; xx [ 3 ] = 0.03730945979482574 ;
pm_math_Quaternion_inverseCompose_ra ( xx + 19 , xx + 0 , xx + 15 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 11 , xx + 15 , xx + 23 ) ; xx [ 4
] = 3.974634836311581e-3 ; xx [ 6 ] = 0.9999921011077627 ; xx [ 11 ] = -
0.2974211203852171 ; xx [ 12 ] = 0.24358232233362 ; xx [ 13 ] = -
0.637892237857866 ; xx [ 14 ] = - 0.6673093902201936 ;
pm_math_Quaternion_xform_ra ( xx + 11 , xx + 543 , xx + 27 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 0 , xx + 27 , xx + 11 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 19 , xx + 27 , xx + 0 ) ; xx [ 30 ]
= xx [ 0 ] + xx [ 7 ] * xx [ 548 ] ; xx [ 31 ] = xx [ 1 ] + xx [ 9 ] * xx [
548 ] ; xx [ 32 ] = xx [ 2 ] + xx [ 10 ] * xx [ 548 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 15 , xx + 30 , xx + 0 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 181 , xx + 27 , xx + 7 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 177 , xx + 7 , xx + 14 ) ; xx [ 35
] = xx [ 27 ] - xx [ 14 ] ; xx [ 36 ] = xx [ 28 ] - xx [ 15 ] ; xx [ 37 ] =
xx [ 29 ] - xx [ 16 ] ; xx [ 14 ] = - 0.7077022283680725 ; xx [ 15 ] = -
9.268355963184496e-3 ; xx [ 16 ] = xx [ 222 ] ; xx [ 38 ] = -
0.238473190515375 ; xx [ 39 ] = 0.3037535947864483 ; xx [ 40 ] =
0.6325302387266769 ; xx [ 41 ] = 0.6713939142977852 ; xx [ 0 ] = xx [ 5 ] *
xx [ 550 ] ; xx [ 3 ] = sin ( xx [ 0 ] ) ; xx [ 42 ] = cos ( xx [ 0 ] ) ; xx
[ 43 ] = - ( xx [ 67 ] * xx [ 3 ] ) ; xx [ 44 ] = xx [ 79 ] * xx [ 3 ] ; xx [
45 ] = xx [ 80 ] * xx [ 3 ] ; pm_math_Quaternion_compose_ra ( xx + 74 , xx +
42 , xx + 46 ) ; pm_math_Quaternion_compose_ra ( xx + 19 , xx + 46 , xx + 42
) ; xx [ 17 ] = - xx [ 42 ] ; xx [ 18 ] = - xx [ 43 ] ; xx [ 19 ] = - xx [ 44
] ; xx [ 20 ] = - xx [ 45 ] ; pm_math_Quaternion_inverseCompose_ra ( xx + 38
, xx + 17 , xx + 42 ) ; xx [ 38 ] = xx [ 67 ] ; xx [ 39 ] = xx [ 50 ] ; xx [
40 ] = - 0.0735748245777017 ; pm_math_Quaternion_inverseXform_ra ( xx + 46 ,
xx + 30 , xx + 50 ) ; xx [ 0 ] = xx [ 50 ] - xx [ 67 ] * xx [ 551 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 17 , xx + 27 , xx + 30 ) ; xx [ 3 ]
= xx [ 51 ] + xx [ 79 ] * xx [ 551 ] ; xx [ 10 ] = xx [ 52 ] + xx [ 80 ] * xx
[ 551 ] ; xx [ 17 ] = xx [ 0 ] - xx [ 30 ] ; xx [ 18 ] = xx [ 3 ] - xx [ 31 ]
; xx [ 19 ] = xx [ 10 ] - xx [ 32 ] ; xx [ 11 ] = 0.2934341153784394 ; xx [
20 ] = 0.2476921755716519 ; xx [ 21 ] = - 0.6382158331684398 ; xx [ 22 ] = -
0.6672522434475355 ; xx [ 30 ] = xx [ 11 ] ; xx [ 31 ] = xx [ 20 ] ; xx [ 32
] = xx [ 21 ] ; xx [ 33 ] = xx [ 22 ] ; xx [ 46 ] = - xx [ 11 ] ; xx [ 47 ] =
xx [ 20 ] ; xx [ 48 ] = xx [ 21 ] ; xx [ 49 ] = xx [ 22 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 46 , xx + 27 , xx + 20 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 30 , xx + 20 , xx + 46 ) ; xx [ 20
] = xx [ 27 ] - xx [ 46 ] ; xx [ 21 ] = xx [ 28 ] - xx [ 47 ] ; xx [ 22 ] =
xx [ 29 ] - xx [ 48 ] ; xx [ 30 ] = xx [ 219 ] ; xx [ 31 ] = -
9.268340058974256e-3 ; xx [ 32 ] = 0.7064499061132662 ; xx [ 46 ] =
0.4090278622146113 ; xx [ 47 ] = 0.5769885451833041 ; xx [ 48 ] =
0.5768689378113487 ; xx [ 49 ] = 0.4086595835751388 ; xx [ 11 ] = xx [ 5 ] *
xx [ 564 ] ; xx [ 33 ] = sin ( xx [ 11 ] ) ; xx [ 50 ] = cos ( xx [ 11 ] ) ;
xx [ 51 ] = - ( xx [ 128 ] * xx [ 33 ] ) ; xx [ 52 ] = xx [ 146 ] * xx [ 33 ]
; xx [ 53 ] = xx [ 147 ] * xx [ 33 ] ; pm_math_Quaternion_compose_ra ( xx +
158 , xx + 50 , xx + 54 ) ; xx [ 11 ] = xx [ 5 ] * xx [ 560 ] ; xx [ 33 ] =
sin ( xx [ 11 ] ) ; xx [ 50 ] = cos ( xx [ 11 ] ) ; xx [ 51 ] = xx [ 127 ] *
xx [ 33 ] ; xx [ 52 ] = xx [ 129 ] * xx [ 33 ] ; xx [ 53 ] = xx [ 137 ] * xx
[ 33 ] ; pm_math_Quaternion_compose_ra ( xx + 133 , xx + 50 , xx + 58 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 54 , xx + 58 , xx + 50 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 46 , xx + 50 , xx + 62 ) ; xx [
46 ] = - 3.734971681712052e-4 ; xx [ 47 ] = - 0.2610715860287324 ; xx [ 48 ]
= - 0.965319370710185 ; pm_math_Quaternion_inverseXform_ra ( xx + 58 , xx + 7
, xx + 66 ) ; xx [ 11 ] = xx [ 66 ] + xx [ 127 ] * xx [ 562 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 54 , xx + 7 , xx + 69 ) ; xx [ 7 ]
= xx [ 69 ] - xx [ 128 ] * xx [ 565 ] ; xx [ 8 ] = xx [ 70 ] + xx [ 146 ] *
xx [ 565 ] ; xx [ 9 ] = xx [ 71 ] + xx [ 147 ] * xx [ 565 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 50 , xx + 7 , xx + 54 ) ; xx [ 33 ]
= xx [ 67 ] + xx [ 129 ] * xx [ 562 ] ; xx [ 41 ] = xx [ 68 ] + xx [ 137 ] *
xx [ 562 ] ; xx [ 49 ] = xx [ 11 ] - xx [ 54 ] ; xx [ 50 ] = xx [ 33 ] - xx [
55 ] ; xx [ 51 ] = xx [ 41 ] - xx [ 56 ] ; xx [ 52 ] = - 0.9915516631640993 ;
xx [ 53 ] = 0.1297122054116248 ; xx [ 54 ] = - 1.970867658312426e-4 ; xx [ 55
] = - 6.481178896480824e-5 ; pm_math_Quaternion_compose_ra ( xx + 181 , xx +
58 , xx + 66 ) ; xx [ 56 ] = - 0.3824707906815349 ; xx [ 57 ] = -
0.03882994823691858 ; xx [ 58 ] = - 0.9229564379288121 ; xx [ 59 ] = -
0.01896689434564094 ; pm_math_Quaternion_inverseCompose_ra ( xx + 66 , xx +
56 , xx + 70 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 52 , xx + 70 ,
xx + 66 ) ; xx [ 52 ] = 1.845352212151208e-7 ; xx [ 53 ] =
3.974634836310293e-3 ; xx [ 54 ] = 0.9999921011077457 ;
pm_math_Quaternion_inverseXform_ra ( xx + 56 , xx + 27 , xx + 74 ) ; xx [ 27
] = xx [ 11 ] ; xx [ 28 ] = xx [ 33 ] ; xx [ 29 ] = xx [ 41 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 70 , xx + 27 , xx + 55 ) ; xx [ 27
] = xx [ 74 ] - xx [ 55 ] ; xx [ 28 ] = xx [ 75 ] - xx [ 56 ] ; xx [ 29 ] =
xx [ 76 ] - xx [ 57 ] ; xx [ 55 ] = - 0.9384577811996331 ; xx [ 56 ] = -
2.456555942788957e-3 ; xx [ 57 ] = 0.3453844350474496 ; xx [ 58 ] =
7.41798972730745e-4 ; xx [ 11 ] = xx [ 5 ] * xx [ 552 ] ; xx [ 33 ] = sin (
xx [ 11 ] ) ; xx [ 70 ] = cos ( xx [ 11 ] ) ; xx [ 71 ] = xx [ 224 ] * xx [
33 ] ; xx [ 72 ] = - ( xx [ 245 ] * xx [ 33 ] ) ; xx [ 73 ] = - ( xx [ 246 ]
* xx [ 33 ] ) ; pm_math_Quaternion_compose_ra ( xx + 240 , xx + 70 , xx + 74
) ; xx [ 11 ] = xx [ 5 ] * xx [ 556 ] ; xx [ 33 ] = sin ( xx [ 11 ] ) ; xx [
70 ] = cos ( xx [ 11 ] ) ; xx [ 71 ] = - ( xx [ 244 ] * xx [ 33 ] ) ; xx [ 72
] = xx [ 262 ] * xx [ 33 ] ; xx [ 73 ] = xx [ 263 ] * xx [ 33 ] ;
pm_math_Quaternion_compose_ra ( xx + 258 , xx + 70 , xx + 78 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 74 , xx + 78 , xx + 70 ) ;
pm_math_Quaternion_inverseCompose_ra ( xx + 55 , xx + 70 , xx + 82 ) ; xx [
55 ] = - 1.380846652021539e-7 ; xx [ 56 ] = 0.9999999999981868 ; xx [ 57 ] =
1.899414133421651e-6 ; xx [ 58 ] = xx [ 0 ] ; xx [ 59 ] = xx [ 3 ] ; xx [ 60
] = xx [ 10 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 58 , xx +
86 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 74 , xx + 58 , xx + 78 ) ;
xx [ 58 ] = xx [ 78 ] + xx [ 224 ] * xx [ 554 ] ; xx [ 59 ] = xx [ 79 ] - xx
[ 245 ] * xx [ 554 ] ; xx [ 60 ] = xx [ 80 ] - xx [ 246 ] * xx [ 554 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 70 , xx + 58 , xx + 74 ) ; xx [ 58
] = xx [ 86 ] - xx [ 244 ] * xx [ 557 ] - xx [ 74 ] ; xx [ 59 ] = xx [ 87 ] +
xx [ 262 ] * xx [ 557 ] - xx [ 75 ] ; xx [ 60 ] = xx [ 88 ] + xx [ 263 ] * xx
[ 557 ] - xx [ 76 ] ; xx [ 70 ] = - 9.516740341210528e-4 ; xx [ 71 ] =
0.3148932788803888 ; xx [ 72 ] = - 2.384623577931677e-3 ; xx [ 73 ] =
0.9491236119720561 ; xx [ 0 ] = xx [ 5 ] * xx [ 566 ] ; xx [ 3 ] = sin ( xx [
0 ] ) ; xx [ 74 ] = cos ( xx [ 0 ] ) ; xx [ 75 ] = xx [ 316 ] * xx [ 3 ] ; xx
[ 76 ] = - ( xx [ 318 ] * xx [ 3 ] ) ; xx [ 77 ] = - ( xx [ 319 ] * xx [ 3 ]
) ; pm_math_Quaternion_compose_ra ( xx + 312 , xx + 74 , xx + 78 ) ; xx [ 0 ]
= xx [ 5 ] * xx [ 570 ] ; xx [ 3 ] = sin ( xx [ 0 ] ) ; xx [ 74 ] = cos ( xx
[ 0 ] ) ; xx [ 75 ] = - ( xx [ 244 ] * xx [ 3 ] ) ; xx [ 76 ] = xx [ 262 ] *
xx [ 3 ] ; xx [ 77 ] = xx [ 323 ] * xx [ 3 ] ; pm_math_Quaternion_compose_ra
( xx + 331 , xx + 74 , xx + 86 ) ; pm_math_Quaternion_inverseCompose_ra ( xx
+ 78 , xx + 86 , xx + 74 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 70 ,
xx + 74 , xx + 90 ) ; xx [ 70 ] = 1.380846645915312e-7 ; xx [ 71 ] = - xx [
262 ] ; xx [ 72 ] = - 1.899414134864941e-6 ;
pm_math_Quaternion_inverseXform_ra ( xx + 86 , xx + 7 , xx + 94 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 7 , xx + 86 ) ; xx [ 7 ]
= xx [ 86 ] + xx [ 316 ] * xx [ 568 ] ; xx [ 8 ] = xx [ 87 ] - xx [ 318 ] *
xx [ 568 ] ; xx [ 9 ] = xx [ 88 ] - xx [ 319 ] * xx [ 568 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 74 , xx + 7 , xx + 78 ) ; xx [ 7 ]
= xx [ 94 ] - xx [ 244 ] * xx [ 571 ] - xx [ 78 ] ; xx [ 8 ] = xx [ 95 ] + xx
[ 262 ] * xx [ 571 ] - xx [ 79 ] ; xx [ 9 ] = xx [ 96 ] + xx [ 323 ] * xx [
571 ] - xx [ 80 ] ; state [ 0 ] = xx [ 533 ] ; state [ 1 ] = xx [ 534 ] ;
state [ 2 ] = xx [ 535 ] ; state [ 3 ] = xx [ 536 ] ; state [ 4 ] = xx [ 537
] ; state [ 5 ] = xx [ 538 ] ; state [ 6 ] = xx [ 539 ] ; state [ 7 ] = xx [
540 ] ; state [ 8 ] = xx [ 541 ] ; state [ 9 ] = xx [ 542 ] ; state [ 10 ] =
xx [ 543 ] ; state [ 11 ] = xx [ 544 ] ; state [ 12 ] = xx [ 545 ] ; state [
13 ] = xx [ 546 ] ; state [ 14 ] = xx [ 547 ] ; state [ 15 ] = xx [ 548 ] ;
state [ 16 ] = xx [ 549 ] ; state [ 17 ] = xx [ 550 ] ; state [ 18 ] = xx [
551 ] ; state [ 19 ] = xx [ 552 ] ; state [ 20 ] = xx [ 553 ] ; state [ 21 ]
= xx [ 554 ] ; state [ 22 ] = xx [ 555 ] ; state [ 23 ] = xx [ 556 ] ; state
[ 24 ] = xx [ 557 ] ; state [ 25 ] = xx [ 558 ] ; state [ 26 ] = xx [ 559 ] ;
state [ 27 ] = xx [ 560 ] ; state [ 28 ] = xx [ 561 ] ; state [ 29 ] = xx [
562 ] ; state [ 30 ] = xx [ 563 ] ; state [ 31 ] = xx [ 564 ] ; state [ 32 ]
= xx [ 565 ] ; state [ 33 ] = xx [ 566 ] ; state [ 34 ] = xx [ 567 ] ; state
[ 35 ] = xx [ 568 ] ; state [ 36 ] = xx [ 569 ] ; state [ 37 ] = xx [ 570 ] ;
state [ 38 ] = xx [ 571 ] ; state [ 39 ] = xx [ 572 ] ; state [ 40 ] = xx [
573 ] ; state [ 41 ] = xx [ 574 ] ; state [ 42 ] = xx [ 575 ] ; state [ 43 ]
= xx [ 576 ] ; state [ 44 ] = xx [ 577 ] ; state [ 45 ] = xx [ 578 ] ; state
[ 46 ] = xx [ 579 ] ; state [ 47 ] = xx [ 580 ] ; state [ 48 ] = xx [ 581 ] ;
state [ 49 ] = xx [ 582 ] ; state [ 50 ] = xx [ 583 ] ; state [ 51 ] = xx [
584 ] ; state [ 52 ] = xx [ 585 ] ; state [ 53 ] = xx [ 586 ] +
sm_core_canonicalAngle ( xx [ 34 ] * atan2 ( sqrt ( xx [ 24 ] * xx [ 24 ] +
xx [ 25 ] * xx [ 25 ] + xx [ 26 ] * xx [ 26 ] ) , fabs ( xx [ 23 ] ) ) * ( (
( xx [ 4 ] * xx [ 25 ] + xx [ 6 ] * xx [ 26 ] ) * xx [ 23 ] ) < 0.0 ? - 1.0 :
+ 1.0 ) - xx [ 586 ] ) ; state [ 54 ] = xx [ 4 ] * ( xx [ 12 ] - xx [ 1 ] ) +
xx [ 6 ] * ( xx [ 13 ] - xx [ 2 ] ) ; state [ 55 ] = xx [ 588 ] +
sm_core_canonicalAngle ( - ( 2.742016529660768e-16 + xx [ 588 ] ) ) ; state [
56 ] = pm_math_Vector3_dot_ra ( xx + 35 , xx + 14 ) ; state [ 57 ] = xx [ 590
] + sm_core_canonicalAngle ( xx [ 34 ] * atan2 ( sqrt ( xx [ 43 ] * xx [ 43 ]
+ xx [ 44 ] * xx [ 44 ] + xx [ 45 ] * xx [ 45 ] ) , fabs ( xx [ 42 ] ) ) * (
( pm_math_Vector3_dot_ra ( xx + 43 , xx + 38 ) * xx [ 42 ] ) < 0.0 ? - 1.0 :
+ 1.0 ) - xx [ 590 ] ) ; state [ 58 ] = pm_math_Vector3_dot_ra ( xx + 17 , xx
+ 38 ) ; state [ 59 ] = xx [ 592 ] + sm_core_canonicalAngle (
2.816424898526927e-5 - xx [ 592 ] ) ; state [ 60 ] = pm_math_Vector3_dot_ra (
xx + 20 , xx + 30 ) ; state [ 61 ] = xx [ 594 ] + sm_core_canonicalAngle ( xx
[ 34 ] * atan2 ( sqrt ( xx [ 63 ] * xx [ 63 ] + xx [ 64 ] * xx [ 64 ] + xx [
65 ] * xx [ 65 ] ) , fabs ( xx [ 62 ] ) ) * ( ( pm_math_Vector3_dot_ra ( xx +
63 , xx + 46 ) * xx [ 62 ] ) < 0.0 ? - 1.0 : + 1.0 ) - xx [ 594 ] ) ; state [
62 ] = pm_math_Vector3_dot_ra ( xx + 49 , xx + 46 ) ; state [ 63 ] = xx [ 596
] + sm_core_canonicalAngle ( xx [ 34 ] * atan2 ( sqrt ( xx [ 67 ] * xx [ 67 ]
+ xx [ 68 ] * xx [ 68 ] + xx [ 69 ] * xx [ 69 ] ) , fabs ( xx [ 66 ] ) ) * (
( pm_math_Vector3_dot_ra ( xx + 67 , xx + 52 ) * xx [ 66 ] ) < 0.0 ? - 1.0 :
+ 1.0 ) - xx [ 596 ] ) ; state [ 64 ] = pm_math_Vector3_dot_ra ( xx + 27 , xx
+ 52 ) ; state [ 65 ] = xx [ 598 ] + sm_core_canonicalAngle ( xx [ 34 ] *
atan2 ( sqrt ( xx [ 83 ] * xx [ 83 ] + xx [ 84 ] * xx [ 84 ] + xx [ 85 ] * xx
[ 85 ] ) , fabs ( xx [ 82 ] ) ) * ( ( pm_math_Vector3_dot_ra ( xx + 83 , xx +
55 ) * xx [ 82 ] ) < 0.0 ? - 1.0 : + 1.0 ) - xx [ 598 ] ) ; state [ 66 ] =
pm_math_Vector3_dot_ra ( xx + 58 , xx + 55 ) ; state [ 67 ] = xx [ 600 ] +
sm_core_canonicalAngle ( xx [ 34 ] * atan2 ( sqrt ( xx [ 91 ] * xx [ 91 ] +
xx [ 92 ] * xx [ 92 ] + xx [ 93 ] * xx [ 93 ] ) , fabs ( xx [ 90 ] ) ) * ( (
pm_math_Vector3_dot_ra ( xx + 91 , xx + 70 ) * xx [ 90 ] ) < 0.0 ? - 1.0 : +
1.0 ) - xx [ 600 ] ) ; state [ 68 ] = pm_math_Vector3_dot_ra ( xx + 7 , xx +
70 ) ; return NULL ; }
