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
PmfMessageId Control_Bicopter_ae14a523_1_deriv ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
deriv , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; int ii [ 16 ] ; double xx [ 2031 ] ; ( void ) rtdvd ; (
void ) rtdvi ; ( void ) eqnEnableFlags ; ( void ) modeVector ; ( void )
discreteState ; ( void ) neDiagMgr ; xx [ 0 ] = state [ 3 ] ; xx [ 1 ] =
state [ 4 ] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] = state [ 6 ] ; xx [ 4 ] =
state [ 10 ] ; xx [ 5 ] = state [ 11 ] ; xx [ 6 ] = state [ 12 ] ;
pm_math_Quaternion_compDeriv_ra ( xx + 0 , xx + 4 , xx + 7 ) ; xx [ 11 ] =
1.0 ; xx [ 12 ] = - 0.2974211203852171 ; xx [ 13 ] = 0.24358232233362 ; xx [
14 ] = - 0.637892237857866 ; xx [ 15 ] = - 0.6673093902201936 ;
pm_math_Quaternion_composeInverse_ra ( xx + 0 , xx + 12 , xx + 16 ) ; xx [ 0
] = xx [ 18 ] * xx [ 18 ] ; xx [ 1 ] = xx [ 19 ] * xx [ 19 ] ; xx [ 2 ] = 2.0
; xx [ 3 ] = xx [ 11 ] - ( xx [ 0 ] + xx [ 1 ] ) * xx [ 2 ] ; xx [ 20 ] = xx
[ 17 ] * xx [ 18 ] ; xx [ 21 ] = xx [ 16 ] * xx [ 19 ] ; xx [ 22 ] = xx [ 2 ]
* ( xx [ 20 ] - xx [ 21 ] ) ; xx [ 23 ] = xx [ 16 ] * xx [ 18 ] ; xx [ 24 ] =
xx [ 17 ] * xx [ 19 ] ; xx [ 25 ] = ( xx [ 23 ] + xx [ 24 ] ) * xx [ 2 ] ; xx
[ 26 ] = xx [ 3 ] ; xx [ 27 ] = xx [ 22 ] ; xx [ 28 ] = xx [ 25 ] ; xx [ 29 ]
= 0.9231031215172467 ; xx [ 30 ] = 0.5 ; xx [ 31 ] = xx [ 30 ] * state [ 43 ]
; xx [ 32 ] = cos ( xx [ 31 ] ) ; xx [ 33 ] = 0.04161497595801117 ; xx [ 34 ]
= sin ( xx [ 31 ] ) ; xx [ 31 ] = xx [ 29 ] * xx [ 32 ] - xx [ 33 ] * xx [ 34
] ; xx [ 35 ] = xx [ 31 ] * xx [ 31 ] ; xx [ 36 ] = xx [ 29 ] * xx [ 34 ] +
xx [ 33 ] * xx [ 32 ] ; xx [ 33 ] = ( xx [ 35 ] + xx [ 36 ] * xx [ 36 ] ) *
xx [ 2 ] - xx [ 11 ] ; xx [ 37 ] = 0.3820992645960585 ; xx [ 38 ] =
0.0122054420764352 ; xx [ 39 ] = xx [ 37 ] * xx [ 32 ] - xx [ 38 ] * xx [ 34
] ; xx [ 40 ] = xx [ 39 ] * xx [ 36 ] ; xx [ 41 ] = xx [ 38 ] * xx [ 32 ] +
xx [ 37 ] * xx [ 34 ] ; xx [ 32 ] = xx [ 31 ] * xx [ 41 ] ; xx [ 34 ] = xx [
2 ] * ( xx [ 40 ] + xx [ 32 ] ) ; xx [ 37 ] = xx [ 39 ] * xx [ 31 ] ; xx [ 38
] = xx [ 36 ] * xx [ 41 ] ; xx [ 42 ] = ( xx [ 37 ] - xx [ 38 ] ) * xx [ 2 ]
; xx [ 43 ] = ( xx [ 40 ] - xx [ 32 ] ) * xx [ 2 ] ; xx [ 32 ] = ( xx [ 35 ]
+ xx [ 39 ] * xx [ 39 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 40 ] = xx [ 39 ] * xx
[ 41 ] ; xx [ 44 ] = xx [ 36 ] * xx [ 31 ] ; xx [ 45 ] = xx [ 2 ] * ( xx [ 40
] + xx [ 44 ] ) ; xx [ 46 ] = xx [ 2 ] * ( xx [ 38 ] + xx [ 37 ] ) ; xx [ 37
] = ( xx [ 44 ] - xx [ 40 ] ) * xx [ 2 ] ; xx [ 38 ] = ( xx [ 35 ] + xx [ 41
] * xx [ 41 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 47 ] = xx [ 33 ] ; xx [ 48 ] =
xx [ 34 ] ; xx [ 49 ] = xx [ 42 ] ; xx [ 50 ] = xx [ 43 ] ; xx [ 51 ] = xx [
32 ] ; xx [ 52 ] = - xx [ 45 ] ; xx [ 53 ] = - xx [ 46 ] ; xx [ 54 ] = xx [
37 ] ; xx [ 55 ] = xx [ 38 ] ; xx [ 35 ] = 3.039490892348256e-4 ; xx [ 40 ] =
4.774983564991541e-17 ; xx [ 44 ] = 3.419427034516897e-10 ; memcpy ( xx + 56
, xx + 44 , 1 * sizeof ( double ) ) ; ii [ 0 ] = factorSymmetricPosDef ( xx +
56 , 1 , xx + 57 ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError
( "sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Revolute5' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 57 , xx + 56 , 1 * sizeof ( double ) ) ; xx [
58 ] = xx [ 40 ] / xx [ 57 ] ; xx [ 59 ] = xx [ 35 ] - xx [ 40 ] * xx [ 58 ]
; xx [ 60 ] = xx [ 35 ] * xx [ 33 ] ; xx [ 61 ] = xx [ 35 ] * xx [ 43 ] ; xx
[ 62 ] = - ( xx [ 35 ] * xx [ 46 ] ) ; xx [ 63 ] = xx [ 35 ] * xx [ 34 ] ; xx
[ 64 ] = xx [ 35 ] * xx [ 32 ] ; xx [ 65 ] = xx [ 35 ] * xx [ 37 ] ; xx [ 66
] = xx [ 42 ] * xx [ 59 ] ; xx [ 67 ] = - ( xx [ 45 ] * xx [ 59 ] ) ; xx [ 68
] = xx [ 59 ] * xx [ 38 ] ; pm_math_Matrix3x3_compose_ra ( xx + 47 , xx + 60
, xx + 69 ) ; xx [ 59 ] = xx [ 30 ] * state [ 41 ] ; xx [ 60 ] = cos ( xx [
59 ] ) ; xx [ 61 ] = 0.04161497595801161 ; xx [ 62 ] = sin ( xx [ 59 ] ) ; xx
[ 59 ] = xx [ 29 ] * xx [ 60 ] + xx [ 61 ] * xx [ 62 ] ; xx [ 63 ] = xx [ 59
] * xx [ 59 ] ; xx [ 64 ] = xx [ 61 ] * xx [ 60 ] - xx [ 29 ] * xx [ 62 ] ;
xx [ 29 ] = ( xx [ 63 ] + xx [ 64 ] * xx [ 64 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 61 ] = 0.3820992645960588 ; xx [ 65 ] = 0.01220544207643509 ; xx [ 66 ] =
xx [ 61 ] * xx [ 60 ] + xx [ 65 ] * xx [ 62 ] ; xx [ 67 ] = xx [ 64 ] * xx [
66 ] ; xx [ 68 ] = xx [ 61 ] * xx [ 62 ] - xx [ 65 ] * xx [ 60 ] ; xx [ 60 ]
= xx [ 59 ] * xx [ 68 ] ; xx [ 61 ] = xx [ 2 ] * ( xx [ 67 ] - xx [ 60 ] ) ;
xx [ 62 ] = xx [ 68 ] * xx [ 64 ] ; xx [ 65 ] = xx [ 59 ] * xx [ 66 ] ; xx [
78 ] = ( xx [ 62 ] + xx [ 65 ] ) * xx [ 2 ] ; xx [ 79 ] = ( xx [ 67 ] + xx [
60 ] ) * xx [ 2 ] ; xx [ 60 ] = ( xx [ 63 ] + xx [ 66 ] * xx [ 66 ] ) * xx [
2 ] - xx [ 11 ] ; xx [ 67 ] = xx [ 68 ] * xx [ 66 ] ; xx [ 80 ] = xx [ 59 ] *
xx [ 64 ] ; xx [ 81 ] = xx [ 2 ] * ( xx [ 67 ] - xx [ 80 ] ) ; xx [ 82 ] = xx
[ 2 ] * ( xx [ 62 ] - xx [ 65 ] ) ; xx [ 62 ] = ( xx [ 67 ] + xx [ 80 ] ) *
xx [ 2 ] ; xx [ 65 ] = ( xx [ 63 ] + xx [ 68 ] * xx [ 68 ] ) * xx [ 2 ] - xx
[ 11 ] ; xx [ 83 ] = xx [ 29 ] ; xx [ 84 ] = xx [ 61 ] ; xx [ 85 ] = xx [ 78
] ; xx [ 86 ] = xx [ 79 ] ; xx [ 87 ] = xx [ 60 ] ; xx [ 88 ] = xx [ 81 ] ;
xx [ 89 ] = xx [ 82 ] ; xx [ 90 ] = xx [ 62 ] ; xx [ 91 ] = xx [ 65 ] ; xx [
63 ] = 4.767768856685382e-17 ; if ( ii [ 0 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Revolute4' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } memcpy ( xx + 67 , xx + 56 , 1 * sizeof ( double ) ) ; xx [
80 ] = xx [ 63 ] / xx [ 67 ] ; xx [ 92 ] = xx [ 35 ] - xx [ 63 ] * xx [ 80 ]
; xx [ 93 ] = xx [ 35 ] * xx [ 29 ] ; xx [ 94 ] = xx [ 35 ] * xx [ 79 ] ; xx
[ 95 ] = xx [ 35 ] * xx [ 82 ] ; xx [ 96 ] = xx [ 35 ] * xx [ 61 ] ; xx [ 97
] = xx [ 35 ] * xx [ 60 ] ; xx [ 98 ] = xx [ 35 ] * xx [ 62 ] ; xx [ 99 ] =
xx [ 78 ] * xx [ 92 ] ; xx [ 100 ] = xx [ 81 ] * xx [ 92 ] ; xx [ 101 ] = xx
[ 92 ] * xx [ 65 ] ; pm_math_Matrix3x3_compose_ra ( xx + 83 , xx + 93 , xx +
102 ) ; xx [ 92 ] = 0.01221082284463426 ; xx [ 93 ] = xx [ 30 ] * state [ 39
] ; xx [ 94 ] = sin ( xx [ 93 ] ) ; xx [ 95 ] = 0.3820990926796041 ; xx [ 96
] = cos ( xx [ 93 ] ) ; xx [ 93 ] = xx [ 92 ] * xx [ 94 ] - xx [ 95 ] * xx [
96 ] ; xx [ 97 ] = xx [ 93 ] * xx [ 93 ] ; xx [ 98 ] = xx [ 92 ] * xx [ 96 ]
+ xx [ 95 ] * xx [ 94 ] ; xx [ 92 ] = ( xx [ 97 ] + xx [ 98 ] * xx [ 98 ] ) *
xx [ 2 ] - xx [ 11 ] ; xx [ 95 ] = 0.9231025353984514 ; xx [ 99 ] =
0.04162797520696046 ; xx [ 100 ] = xx [ 95 ] * xx [ 96 ] - xx [ 99 ] * xx [
94 ] ; xx [ 101 ] = xx [ 98 ] * xx [ 100 ] ; xx [ 111 ] = xx [ 99 ] * xx [ 96
] + xx [ 95 ] * xx [ 94 ] ; xx [ 94 ] = xx [ 111 ] * xx [ 93 ] ; xx [ 95 ] =
xx [ 2 ] * ( xx [ 101 ] - xx [ 94 ] ) ; xx [ 96 ] = xx [ 111 ] * xx [ 98 ] ;
xx [ 99 ] = xx [ 93 ] * xx [ 100 ] ; xx [ 112 ] = ( xx [ 96 ] + xx [ 99 ] ) *
xx [ 2 ] ; xx [ 113 ] = ( xx [ 101 ] + xx [ 94 ] ) * xx [ 2 ] ; xx [ 94 ] = (
xx [ 97 ] + xx [ 100 ] * xx [ 100 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 101 ] =
xx [ 111 ] * xx [ 100 ] ; xx [ 114 ] = xx [ 93 ] * xx [ 98 ] ; xx [ 115 ] =
xx [ 2 ] * ( xx [ 101 ] - xx [ 114 ] ) ; xx [ 116 ] = xx [ 2 ] * ( xx [ 96 ]
- xx [ 99 ] ) ; xx [ 96 ] = ( xx [ 101 ] + xx [ 114 ] ) * xx [ 2 ] ; xx [ 99
] = ( xx [ 97 ] + xx [ 111 ] * xx [ 111 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 117
] = xx [ 92 ] ; xx [ 118 ] = xx [ 95 ] ; xx [ 119 ] = xx [ 112 ] ; xx [ 120 ]
= xx [ 113 ] ; xx [ 121 ] = xx [ 94 ] ; xx [ 122 ] = xx [ 115 ] ; xx [ 123 ]
= xx [ 116 ] ; xx [ 124 ] = xx [ 96 ] ; xx [ 125 ] = xx [ 99 ] ; xx [ 97 ] =
4.767498895663637e-17 ; if ( ii [ 0 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Revolute3' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } memcpy ( xx + 101 , xx + 56 , 1 * sizeof ( double ) ) ; xx
[ 114 ] = xx [ 97 ] / xx [ 101 ] ; xx [ 126 ] = xx [ 35 ] - xx [ 97 ] * xx [
114 ] ; xx [ 127 ] = xx [ 35 ] * xx [ 92 ] ; xx [ 128 ] = xx [ 35 ] * xx [
113 ] ; xx [ 129 ] = xx [ 35 ] * xx [ 116 ] ; xx [ 130 ] = xx [ 35 ] * xx [
95 ] ; xx [ 131 ] = xx [ 35 ] * xx [ 94 ] ; xx [ 132 ] = xx [ 35 ] * xx [ 96
] ; xx [ 133 ] = xx [ 112 ] * xx [ 126 ] ; xx [ 134 ] = xx [ 115 ] * xx [ 126
] ; xx [ 135 ] = xx [ 126 ] * xx [ 99 ] ; pm_math_Matrix3x3_compose_ra ( xx +
117 , xx + 127 , xx + 136 ) ; xx [ 126 ] = 0.01221082284463421 ; xx [ 127 ] =
xx [ 30 ] * state [ 37 ] ; xx [ 128 ] = sin ( xx [ 127 ] ) ; xx [ 129 ] =
0.3820990926796067 ; xx [ 130 ] = cos ( xx [ 127 ] ) ; xx [ 127 ] = xx [ 126
] * xx [ 128 ] - xx [ 129 ] * xx [ 130 ] ; xx [ 131 ] = xx [ 127 ] * xx [ 127
] ; xx [ 132 ] = xx [ 126 ] * xx [ 130 ] + xx [ 129 ] * xx [ 128 ] ; xx [ 126
] = ( xx [ 131 ] + xx [ 132 ] * xx [ 132 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [
129 ] = 0.9231025353984502 ; xx [ 133 ] = 0.04162797520696043 ; xx [ 134 ] =
xx [ 129 ] * xx [ 130 ] - xx [ 133 ] * xx [ 128 ] ; xx [ 135 ] = xx [ 132 ] *
xx [ 134 ] ; xx [ 145 ] = xx [ 133 ] * xx [ 130 ] + xx [ 129 ] * xx [ 128 ] ;
xx [ 128 ] = xx [ 145 ] * xx [ 127 ] ; xx [ 129 ] = xx [ 2 ] * ( xx [ 135 ] -
xx [ 128 ] ) ; xx [ 130 ] = xx [ 145 ] * xx [ 132 ] ; xx [ 133 ] = xx [ 127 ]
* xx [ 134 ] ; xx [ 146 ] = ( xx [ 130 ] + xx [ 133 ] ) * xx [ 2 ] ; xx [ 147
] = ( xx [ 135 ] + xx [ 128 ] ) * xx [ 2 ] ; xx [ 128 ] = ( xx [ 131 ] + xx [
134 ] * xx [ 134 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 135 ] = xx [ 145 ] * xx [
134 ] ; xx [ 148 ] = xx [ 127 ] * xx [ 132 ] ; xx [ 149 ] = xx [ 2 ] * ( xx [
135 ] - xx [ 148 ] ) ; xx [ 150 ] = xx [ 2 ] * ( xx [ 130 ] - xx [ 133 ] ) ;
xx [ 130 ] = ( xx [ 135 ] + xx [ 148 ] ) * xx [ 2 ] ; xx [ 133 ] = ( xx [ 131
] + xx [ 145 ] * xx [ 145 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 151 ] = xx [ 126
] ; xx [ 152 ] = xx [ 129 ] ; xx [ 153 ] = xx [ 146 ] ; xx [ 154 ] = xx [ 147
] ; xx [ 155 ] = xx [ 128 ] ; xx [ 156 ] = xx [ 149 ] ; xx [ 157 ] = xx [ 150
] ; xx [ 158 ] = xx [ 130 ] ; xx [ 159 ] = xx [ 133 ] ; if ( ii [ 0 ] != 0 )
{ return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Revolute2' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 131 ] = xx [ 97 ] / xx [ 56 ] ; xx [ 135 ] = xx [ 35 ]
- xx [ 97 ] * xx [ 131 ] ; xx [ 160 ] = xx [ 35 ] * xx [ 126 ] ; xx [ 161 ] =
xx [ 35 ] * xx [ 147 ] ; xx [ 162 ] = xx [ 35 ] * xx [ 150 ] ; xx [ 163 ] =
xx [ 35 ] * xx [ 129 ] ; xx [ 164 ] = xx [ 35 ] * xx [ 128 ] ; xx [ 165 ] =
xx [ 35 ] * xx [ 130 ] ; xx [ 166 ] = xx [ 146 ] * xx [ 135 ] ; xx [ 167 ] =
xx [ 149 ] * xx [ 135 ] ; xx [ 168 ] = xx [ 135 ] * xx [ 133 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 151 , xx + 160 , xx + 169 ) ; xx [ 160 ]
= - 0.7050900121853094 ; xx [ 161 ] = - 0.7077512799841439 ; xx [ 162 ] =
0.04400227718341848 ; xx [ 163 ] = 0.07542700717708278 ; xx [ 164 ] = -
0.01315394045037388 ; xx [ 165 ] = 0.9970645618208165 ; xx [ 166 ] = -
0.7050949165217606 ; xx [ 167 ] = 0.7063392241207016 ; xx [ 168 ] =
0.06265827290739301 ; xx [ 135 ] = 0.1432898122275 ; xx [ 148 ] =
2.587007977055741e-5 ; xx [ 178 ] = 0.1432898122275 ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 178 , 1 , xx + 179 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Prismatic3' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 179 ] = xx [ 148 ] / xx [ 178 ] ; xx [ 180 ] =
0.1432887294914039 ; xx [ 181 ] = - ( xx [ 180 ] * xx [ 179 ] ) ; xx [ 182 ]
= 5.564347854496134e-4 ; xx [ 183 ] = - ( xx [ 182 ] * xx [ 179 ] ) ; xx [
184 ] = xx [ 180 ] / xx [ 178 ] ; xx [ 185 ] = - ( xx [ 182 ] * xx [ 184 ] )
; xx [ 186 ] = xx [ 135 ] - xx [ 148 ] * xx [ 179 ] ; xx [ 187 ] = xx [ 181 ]
; xx [ 188 ] = xx [ 183 ] ; xx [ 189 ] = xx [ 181 ] ; xx [ 190 ] = xx [ 135 ]
- xx [ 180 ] * xx [ 184 ] ; xx [ 191 ] = xx [ 185 ] ; xx [ 192 ] = xx [ 183 ]
; xx [ 193 ] = xx [ 185 ] ; xx [ 194 ] = xx [ 135 ] - 3.096196704583573e-7 /
xx [ 178 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 186 , xx + 160 , xx
+ 195 ) ; pm_math_Matrix3x3_compose_ra ( xx + 160 , xx + 195 , xx + 185 ) ;
xx [ 160 ] = 0.0571681352651122 ; xx [ 161 ] = 4.688912383704416e-3 ; xx [
162 ] = 1.863671450468227e-5 ; xx [ 163 ] = 4.688912383704415e-3 ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 163 , 1 , xx + 164 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Prismatic2' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } memcpy ( xx + 164 , xx + 163 , 1 * sizeof ( double ) ) ; xx
[ 165 ] = xx [ 162 ] / xx [ 164 ] ; xx [ 166 ] = xx [ 161 ] - xx [ 162 ] * xx
[ 165 ] ; xx [ 167 ] = 0.7074807242098043 ; xx [ 168 ] = 4.688875346490787e-3
; xx [ 181 ] = xx [ 168 ] * xx [ 165 ] ; xx [ 183 ] = xx [ 160 ] * xx [ 166 ]
- xx [ 167 ] * xx [ 181 ] ; xx [ 194 ] = 2.326651590580089e-3 + xx [ 160 ] *
xx [ 183 ] ; xx [ 195 ] = xx [ 168 ] / xx [ 164 ] ; xx [ 196 ] = xx [ 161 ] -
xx [ 168 ] * xx [ 195 ] ; xx [ 197 ] = xx [ 167 ] * xx [ 196 ] - xx [ 160 ] *
xx [ 181 ] ; xx [ 198 ] = xx [ 167 ] * xx [ 197 ] ; xx [ 199 ] =
0.05154241837057226 ; xx [ 200 ] = 1.863671450468771e-5 ; if ( ii [ 0 ] != 0
) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Prismatic1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 201 ] = xx [ 200 ] / xx [ 163 ] ; xx [ 202 ] = xx [
161 ] - xx [ 200 ] * xx [ 201 ] ; xx [ 203 ] = 0.7079128123763776 ; xx [ 204
] = xx [ 168 ] * xx [ 201 ] ; xx [ 205 ] = xx [ 199 ] * xx [ 202 ] + xx [ 203
] * xx [ 204 ] ; xx [ 206 ] = 2.326651590580094e-3 + xx [ 199 ] * xx [ 205 ]
; xx [ 207 ] = xx [ 168 ] / xx [ 163 ] ; xx [ 208 ] = xx [ 161 ] - xx [ 168 ]
* xx [ 207 ] ; xx [ 209 ] = xx [ 203 ] * xx [ 208 ] + xx [ 199 ] * xx [ 204 ]
; xx [ 210 ] = xx [ 203 ] * xx [ 209 ] ; xx [ 211 ] = 0.7077022283680723 ; xx
[ 212 ] = 9.268355963184306e-3 ; xx [ 213 ] = 0.7044014266904837 ; xx [ 214 ]
= 0.0545544318939939 ; xx [ 215 ] = - xx [ 211 ] ; xx [ 216 ] = -
0.08646631746178521 ; xx [ 217 ] = 0.9962116610049985 ; xx [ 218 ] = - xx [
212 ] ; xx [ 219 ] = 0.7045155825253331 ; xx [ 220 ] = 0.06772104871002882 ;
xx [ 221 ] = 0.7064500361247108 ; xx [ 222 ] = 7.925328667747152e-3 ; xx [
223 ] = - 0.5181324366224915 ; xx [ 224 ] = - 0.4812371049201607 ; xx [ 225 ]
= - 0.4812727257575918 ; xx [ 226 ] = 0.5180021142906276 ; xx [ 227 ] = xx [
30 ] * input [ 0 ] ; xx [ 228 ] = 0.9972896958009458 ; xx [ 229 ] = sin ( xx
[ 227 ] ) ; xx [ 230 ] = 8.852854518109909e-5 ; xx [ 231 ] =
0.0735748245798411 ; xx [ 232 ] = cos ( xx [ 227 ] ) ; xx [ 233 ] = - ( xx [
228 ] * xx [ 229 ] ) ; xx [ 234 ] = xx [ 230 ] * xx [ 229 ] ; xx [ 235 ] = xx
[ 231 ] * xx [ 229 ] ; pm_math_Quaternion_compose_ra ( xx + 223 , xx + 232 ,
xx + 236 ) ; xx [ 223 ] = xx [ 236 ] * xx [ 236 ] ; xx [ 224 ] = ( xx [ 223 ]
+ xx [ 237 ] * xx [ 237 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 225 ] = xx [ 237 ]
* xx [ 238 ] ; xx [ 226 ] = xx [ 236 ] * xx [ 239 ] ; xx [ 227 ] = xx [ 2 ] *
( xx [ 225 ] - xx [ 226 ] ) ; xx [ 229 ] = xx [ 237 ] * xx [ 239 ] ; xx [ 232
] = xx [ 236 ] * xx [ 238 ] ; xx [ 233 ] = ( xx [ 229 ] + xx [ 232 ] ) * xx [
2 ] ; xx [ 234 ] = ( xx [ 225 ] + xx [ 226 ] ) * xx [ 2 ] ; xx [ 225 ] = ( xx
[ 223 ] + xx [ 238 ] * xx [ 238 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 226 ] = xx
[ 238 ] * xx [ 239 ] ; xx [ 235 ] = xx [ 236 ] * xx [ 237 ] ; xx [ 240 ] = xx
[ 2 ] * ( xx [ 226 ] - xx [ 235 ] ) ; xx [ 241 ] = xx [ 2 ] * ( xx [ 229 ] -
xx [ 232 ] ) ; xx [ 229 ] = ( xx [ 226 ] + xx [ 235 ] ) * xx [ 2 ] ; xx [ 226
] = ( xx [ 223 ] + xx [ 239 ] * xx [ 239 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [
242 ] = xx [ 224 ] ; xx [ 243 ] = xx [ 227 ] ; xx [ 244 ] = xx [ 233 ] ; xx [
245 ] = xx [ 234 ] ; xx [ 246 ] = xx [ 225 ] ; xx [ 247 ] = xx [ 240 ] ; xx [
248 ] = xx [ 241 ] ; xx [ 249 ] = xx [ 229 ] ; xx [ 250 ] = xx [ 226 ] ; xx [
223 ] = 0.04599646396324401 ; xx [ 251 ] = 0.6252280994013473 ; xx [ 252 ] =
0.6477664378100223 ; xx [ 253 ] = 0.330373832435706 ; xx [ 254 ] =
0.2834459324238707 ; xx [ 232 ] = xx [ 30 ] * input [ 3 ] ; xx [ 235 ] =
1.380846645360201e-7 ; xx [ 255 ] = sin ( xx [ 232 ] ) ; xx [ 256 ] =
0.9999999999981867 ; xx [ 257 ] = 1.899414131645294e-6 ; xx [ 258 ] = cos (
xx [ 232 ] ) ; xx [ 259 ] = - ( xx [ 235 ] * xx [ 255 ] ) ; xx [ 260 ] = xx [
256 ] * xx [ 255 ] ; xx [ 261 ] = xx [ 257 ] * xx [ 255 ] ;
pm_math_Quaternion_compose_ra ( xx + 251 , xx + 258 , xx + 262 ) ; xx [ 232 ]
= xx [ 262 ] * xx [ 262 ] ; xx [ 251 ] = ( xx [ 232 ] + xx [ 263 ] * xx [ 263
] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 252 ] = xx [ 263 ] * xx [ 264 ] ; xx [ 253
] = xx [ 262 ] * xx [ 265 ] ; xx [ 254 ] = xx [ 2 ] * ( xx [ 252 ] - xx [ 253
] ) ; xx [ 255 ] = xx [ 263 ] * xx [ 265 ] ; xx [ 258 ] = xx [ 262 ] * xx [
264 ] ; xx [ 259 ] = ( xx [ 255 ] + xx [ 258 ] ) * xx [ 2 ] ; xx [ 260 ] = (
xx [ 252 ] + xx [ 253 ] ) * xx [ 2 ] ; xx [ 252 ] = ( xx [ 232 ] + xx [ 264 ]
* xx [ 264 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 253 ] = xx [ 264 ] * xx [ 265 ]
; xx [ 261 ] = xx [ 262 ] * xx [ 263 ] ; xx [ 266 ] = xx [ 2 ] * ( xx [ 253 ]
- xx [ 261 ] ) ; xx [ 267 ] = xx [ 2 ] * ( xx [ 255 ] - xx [ 258 ] ) ; xx [
255 ] = ( xx [ 253 ] + xx [ 261 ] ) * xx [ 2 ] ; xx [ 253 ] = ( xx [ 232 ] +
xx [ 265 ] * xx [ 265 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 268 ] = xx [ 251 ] ;
xx [ 269 ] = xx [ 254 ] ; xx [ 270 ] = xx [ 259 ] ; xx [ 271 ] = xx [ 260 ] ;
xx [ 272 ] = xx [ 252 ] ; xx [ 273 ] = xx [ 266 ] ; xx [ 274 ] = xx [ 267 ] ;
xx [ 275 ] = xx [ 255 ] ; xx [ 276 ] = xx [ 253 ] ; xx [ 232 ] =
3.705034714586038e-3 ; xx [ 277 ] = xx [ 232 ] * xx [ 251 ] ; xx [ 278 ] = xx
[ 232 ] * xx [ 260 ] ; xx [ 279 ] = xx [ 232 ] * xx [ 267 ] ; xx [ 280 ] = xx
[ 232 ] * xx [ 254 ] ; xx [ 281 ] = xx [ 232 ] * xx [ 252 ] ; xx [ 282 ] = xx
[ 232 ] * xx [ 255 ] ; xx [ 283 ] = xx [ 232 ] * xx [ 259 ] ; xx [ 284 ] = xx
[ 232 ] * xx [ 266 ] ; xx [ 285 ] = xx [ 232 ] * xx [ 253 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 268 , xx + 277 , xx + 286 ) ; xx [ 277 ]
= 0.4716196941536389 ; xx [ 278 ] = - 0.5117381057636369 ; xx [ 279 ] =
0.5267317275917935 ; xx [ 280 ] = - 0.4881113216716428 ; xx [ 258 ] = xx [ 30
] * state [ 27 ] ; xx [ 261 ] = 3.059547932236506e-4 ; xx [ 281 ] = sin ( xx
[ 258 ] ) ; xx [ 282 ] = 0.9999868227746889 ; xx [ 283 ] =
5.124516430674353e-3 ; xx [ 295 ] = cos ( xx [ 258 ] ) ; xx [ 296 ] = xx [
261 ] * xx [ 281 ] ; xx [ 297 ] = - ( xx [ 282 ] * xx [ 281 ] ) ; xx [ 298 ]
= - ( xx [ 283 ] * xx [ 281 ] ) ; pm_math_Quaternion_compose_ra ( xx + 277 ,
xx + 295 , xx + 299 ) ; xx [ 258 ] = xx [ 299 ] * xx [ 299 ] ; xx [ 277 ] = (
xx [ 258 ] + xx [ 300 ] * xx [ 300 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 278 ] =
xx [ 300 ] * xx [ 301 ] ; xx [ 279 ] = xx [ 299 ] * xx [ 302 ] ; xx [ 280 ] =
xx [ 2 ] * ( xx [ 278 ] - xx [ 279 ] ) ; xx [ 281 ] = xx [ 300 ] * xx [ 302 ]
; xx [ 284 ] = xx [ 299 ] * xx [ 301 ] ; xx [ 285 ] = ( xx [ 281 ] + xx [ 284
] ) * xx [ 2 ] ; xx [ 295 ] = ( xx [ 278 ] + xx [ 279 ] ) * xx [ 2 ] ; xx [
278 ] = ( xx [ 258 ] + xx [ 301 ] * xx [ 301 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 279 ] = xx [ 301 ] * xx [ 302 ] ; xx [ 296 ] = xx [ 299 ] * xx [ 300 ] ; xx
[ 297 ] = xx [ 2 ] * ( xx [ 279 ] - xx [ 296 ] ) ; xx [ 298 ] = xx [ 2 ] * (
xx [ 281 ] - xx [ 284 ] ) ; xx [ 281 ] = ( xx [ 279 ] + xx [ 296 ] ) * xx [ 2
] ; xx [ 279 ] = ( xx [ 258 ] + xx [ 302 ] * xx [ 302 ] ) * xx [ 2 ] - xx [
11 ] ; xx [ 303 ] = xx [ 277 ] ; xx [ 304 ] = xx [ 280 ] ; xx [ 305 ] = xx [
285 ] ; xx [ 306 ] = xx [ 295 ] ; xx [ 307 ] = xx [ 278 ] ; xx [ 308 ] = xx [
297 ] ; xx [ 309 ] = xx [ 298 ] ; xx [ 310 ] = xx [ 281 ] ; xx [ 311 ] = xx [
279 ] ; xx [ 258 ] = 6.563137762047828e-4 ; xx [ 284 ] =
2.163481365691237e-10 ; xx [ 296 ] = 8.077935669463161e-28 ; xx [ 312 ] =
6.381124474950283e-9 ; xx [ 313 ] = xx [ 296 ] ; xx [ 314 ] = xx [ 296 ] ; xx
[ 315 ] = 6.563137762047832e-4 ; ii [ 0 ] = factorSymmetricPosDef ( xx + 312
, 2 , xx + 316 ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Cylindrical4' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 316 ] = 2.208066226790097e-12 ; xx [ 317 ] = 0.0 ; xx
[ 318 ] = 6.381017038462024e-9 ; xx [ 319 ] = 3.69662015662382e-11 ; xx [ 320
] = 2.008023456885676e-7 ; xx [ 321 ] = 4.871572111057907e-12 ; xx [ 322 ] =
6.563051278102789e-4 ; xx [ 323 ] = 9.377109494863897e-10 ; xx [ 324 ] =
3.36329072983934e-6 ; xx [ 325 ] = xx [ 316 ] ; xx [ 326 ] = xx [ 317 ] ; xx
[ 327 ] = - xx [ 318 ] ; xx [ 328 ] = xx [ 317 ] ; xx [ 329 ] = - xx [ 319 ]
; xx [ 330 ] = xx [ 317 ] ; xx [ 331 ] = xx [ 284 ] ; xx [ 332 ] = xx [ 320 ]
; xx [ 333 ] = xx [ 321 ] ; xx [ 334 ] = - xx [ 322 ] ; xx [ 335 ] = - xx [
323 ] ; xx [ 336 ] = - xx [ 324 ] ; solveSymmetricPosDef ( xx + 312 , xx +
325 , 2 , 6 , xx + 337 , xx + 349 ) ; xx [ 325 ] = - ( xx [ 321 ] * xx [ 343
] - xx [ 322 ] * xx [ 344 ] ) ; xx [ 326 ] = xx [ 323 ] * xx [ 343 ] + xx [
324 ] * xx [ 344 ] ; xx [ 327 ] = xx [ 323 ] * xx [ 345 ] + xx [ 324 ] * xx [
346 ] ; xx [ 328 ] = xx [ 258 ] - ( xx [ 284 ] * xx [ 343 ] + xx [ 320 ] * xx
[ 344 ] ) ; xx [ 329 ] = xx [ 325 ] ; xx [ 330 ] = xx [ 326 ] ; xx [ 331 ] =
xx [ 325 ] ; xx [ 332 ] = xx [ 258 ] - ( xx [ 321 ] * xx [ 345 ] - xx [ 322 ]
* xx [ 346 ] ) ; xx [ 333 ] = xx [ 327 ] ; xx [ 334 ] = xx [ 326 ] ; xx [ 335
] = xx [ 327 ] ; xx [ 336 ] = xx [ 258 ] + xx [ 323 ] * xx [ 347 ] + xx [ 324
] * xx [ 348 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 328 , xx + 303
, xx + 349 ) ; pm_math_Matrix3x3_compose_ra ( xx + 303 , xx + 349 , xx + 325
) ; xx [ 349 ] = xx [ 223 ] + xx [ 286 ] + xx [ 325 ] ; xx [ 350 ] = xx [ 287
] + xx [ 326 ] ; xx [ 351 ] = xx [ 288 ] + xx [ 327 ] ; xx [ 352 ] = xx [ 289
] + xx [ 328 ] ; xx [ 353 ] = xx [ 223 ] + xx [ 290 ] + xx [ 329 ] ; xx [ 354
] = xx [ 291 ] + xx [ 330 ] ; xx [ 355 ] = xx [ 292 ] + xx [ 331 ] ; xx [ 356
] = xx [ 293 ] + xx [ 332 ] ; xx [ 357 ] = xx [ 223 ] + xx [ 294 ] + xx [ 333
] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 349 , xx + 242 , xx + 358 )
; pm_math_Matrix3x3_compose_ra ( xx + 242 , xx + 358 , xx + 367 ) ; xx [ 358
] = - 0.1316824519486606 ; xx [ 359 ] = 0.9912919062119712 ; xx [ 360 ] =
2.050212948449675e-4 ; xx [ 361 ] = - 2.156239646078782e-4 ; xx [ 334 ] = xx
[ 30 ] * state [ 23 ] ; xx [ 335 ] = 3.734971681885031e-4 ; xx [ 336 ] = sin
( xx [ 334 ] ) ; xx [ 362 ] = 0.261071586028717 ; xx [ 363 ] =
0.965319370710189 ; xx [ 376 ] = cos ( xx [ 334 ] ) ; xx [ 377 ] = xx [ 335 ]
* xx [ 336 ] ; xx [ 378 ] = xx [ 362 ] * xx [ 336 ] ; xx [ 379 ] = xx [ 363 ]
* xx [ 336 ] ; pm_math_Quaternion_compose_ra ( xx + 358 , xx + 376 , xx + 380
) ; xx [ 334 ] = xx [ 380 ] * xx [ 380 ] ; xx [ 336 ] = ( xx [ 334 ] + xx [
381 ] * xx [ 381 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 358 ] = xx [ 381 ] * xx [
382 ] ; xx [ 359 ] = xx [ 380 ] * xx [ 383 ] ; xx [ 360 ] = xx [ 2 ] * ( xx [
358 ] - xx [ 359 ] ) ; xx [ 361 ] = xx [ 381 ] * xx [ 383 ] ; xx [ 364 ] = xx
[ 380 ] * xx [ 382 ] ; xx [ 365 ] = ( xx [ 361 ] + xx [ 364 ] ) * xx [ 2 ] ;
xx [ 366 ] = ( xx [ 358 ] + xx [ 359 ] ) * xx [ 2 ] ; xx [ 358 ] = ( xx [ 334
] + xx [ 382 ] * xx [ 382 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 359 ] = xx [ 382
] * xx [ 383 ] ; xx [ 376 ] = xx [ 380 ] * xx [ 381 ] ; xx [ 377 ] = xx [ 2 ]
* ( xx [ 359 ] - xx [ 376 ] ) ; xx [ 378 ] = xx [ 2 ] * ( xx [ 361 ] - xx [
364 ] ) ; xx [ 361 ] = ( xx [ 359 ] + xx [ 376 ] ) * xx [ 2 ] ; xx [ 359 ] =
( xx [ 334 ] + xx [ 383 ] * xx [ 383 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 384 ]
= xx [ 336 ] ; xx [ 385 ] = xx [ 360 ] ; xx [ 386 ] = xx [ 365 ] ; xx [ 387 ]
= xx [ 366 ] ; xx [ 388 ] = xx [ 358 ] ; xx [ 389 ] = xx [ 377 ] ; xx [ 390 ]
= xx [ 378 ] ; xx [ 391 ] = xx [ 361 ] ; xx [ 392 ] = xx [ 359 ] ; xx [ 334 ]
= 0.01903226903914041 ; xx [ 364 ] = 7.108498590320665e-6 ; xx [ 376 ] =
5.293955920339377e-23 ; xx [ 393 ] = 1.92364382987888e-6 ; xx [ 394 ] = xx [
376 ] ; xx [ 395 ] = xx [ 376 ] ; xx [ 396 ] = 0.0190322690391404 ; ii [ 0 ]
= factorSymmetricPosDef ( xx + 393 , 2 , xx + 397 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Cylindrical2' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 376 ] = 7.195732331503883e-10 ; xx [ 379 ] =
3.249102609586574e-7 ; xx [ 397 ] = 1.076691775268914e-6 ; xx [ 398 ] =
1.233385549020995e-4 ; xx [ 399 ] = 1.759078185033288e-6 ; xx [ 400 ] =
4.968784663773632e-3 ; xx [ 401 ] = 4.280228318911531e-7 ; xx [ 402 ] =
0.01837221797205003 ; xx [ 403 ] = xx [ 376 ] ; xx [ 404 ] = xx [ 317 ] ; xx
[ 405 ] = xx [ 379 ] ; xx [ 406 ] = xx [ 317 ] ; xx [ 407 ] = xx [ 397 ] ; xx
[ 408 ] = xx [ 317 ] ; xx [ 409 ] = - xx [ 398 ] ; xx [ 410 ] = xx [ 364 ] ;
xx [ 411 ] = xx [ 399 ] ; xx [ 412 ] = xx [ 400 ] ; xx [ 413 ] = - xx [ 401 ]
; xx [ 414 ] = xx [ 402 ] ; solveSymmetricPosDef ( xx + 393 , xx + 403 , 2 ,
6 , xx + 415 , xx + 427 ) ; xx [ 403 ] = - ( xx [ 399 ] * xx [ 421 ] + xx [
400 ] * xx [ 422 ] ) ; xx [ 404 ] = - ( xx [ 402 ] * xx [ 422 ] - xx [ 401 ]
* xx [ 421 ] ) ; xx [ 405 ] = - ( xx [ 402 ] * xx [ 424 ] - xx [ 401 ] * xx [
423 ] ) ; xx [ 406 ] = xx [ 334 ] - ( xx [ 364 ] * xx [ 422 ] - xx [ 398 ] *
xx [ 421 ] ) ; xx [ 407 ] = xx [ 403 ] ; xx [ 408 ] = xx [ 404 ] ; xx [ 409 ]
= xx [ 403 ] ; xx [ 410 ] = xx [ 334 ] - ( xx [ 399 ] * xx [ 423 ] + xx [ 400
] * xx [ 424 ] ) ; xx [ 411 ] = xx [ 405 ] ; xx [ 412 ] = xx [ 404 ] ; xx [
413 ] = xx [ 405 ] ; xx [ 414 ] = xx [ 334 ] - ( xx [ 402 ] * xx [ 426 ] - xx
[ 401 ] * xx [ 425 ] ) ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 406 ,
xx + 384 , xx + 427 ) ; pm_math_Matrix3x3_compose_ra ( xx + 384 , xx + 427 ,
xx + 403 ) ; xx [ 412 ] = xx [ 222 ] + xx [ 367 ] + xx [ 403 ] ; xx [ 413 ] =
0.9999999602050231 ; xx [ 414 ] = xx [ 368 ] + xx [ 404 ] ; xx [ 427 ] =
2.821169119813227e-4 ; xx [ 428 ] = xx [ 412 ] * xx [ 413 ] + xx [ 414 ] * xx
[ 427 ] ; xx [ 429 ] = xx [ 370 ] + xx [ 406 ] ; xx [ 430 ] = xx [ 222 ] + xx
[ 371 ] + xx [ 407 ] ; xx [ 431 ] = xx [ 429 ] * xx [ 413 ] + xx [ 430 ] * xx
[ 427 ] ; xx [ 432 ] = xx [ 428 ] * xx [ 413 ] + xx [ 431 ] * xx [ 427 ] ; ii
[ 0 ] = factorSymmetricPosDef ( xx + 432 , 1 , xx + 433 ) ; if ( ii [ 0 ] !=
0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Prismatic' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 433 ] = xx [ 428 ] / xx [ 432 ] ; xx [ 434 ] = xx [
431 ] * xx [ 433 ] ; xx [ 435 ] = xx [ 369 ] + xx [ 405 ] ; xx [ 436 ] = xx [
373 ] + xx [ 409 ] ; xx [ 437 ] = xx [ 374 ] + xx [ 410 ] ; xx [ 438 ] = xx [
436 ] * xx [ 413 ] + xx [ 437 ] * xx [ 427 ] ; xx [ 439 ] = xx [ 438 ] * xx [
433 ] ; xx [ 440 ] = xx [ 431 ] / xx [ 432 ] ; xx [ 441 ] = xx [ 372 ] + xx [
408 ] ; xx [ 442 ] = xx [ 438 ] * xx [ 440 ] ; xx [ 443 ] = xx [ 222 ] + xx [
375 ] + xx [ 411 ] ; xx [ 222 ] = xx [ 438 ] / xx [ 432 ] ; xx [ 444 ] = xx [
412 ] - xx [ 428 ] * xx [ 433 ] ; xx [ 445 ] = xx [ 414 ] - xx [ 434 ] ; xx [
446 ] = xx [ 435 ] - xx [ 439 ] ; xx [ 447 ] = xx [ 429 ] - xx [ 434 ] ; xx [
448 ] = xx [ 430 ] - xx [ 431 ] * xx [ 440 ] ; xx [ 449 ] = xx [ 441 ] - xx [
442 ] ; xx [ 450 ] = xx [ 436 ] - xx [ 439 ] ; xx [ 451 ] = xx [ 437 ] - xx [
442 ] ; xx [ 452 ] = xx [ 443 ] - xx [ 438 ] * xx [ 222 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 444 , xx + 213 , xx + 453 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 213 , xx + 453 , xx + 444 ) ; xx [ 434 ]
= 0.1822080146530136 ; xx [ 453 ] = - 0.9120630938981265 ; xx [ 454 ] = -
0.1421693578567881 ; xx [ 455 ] = 0.3844095801838581 ; xx [ 456 ] = -
0.01257223522036471 ; xx [ 439 ] = xx [ 30 ] * state [ 13 ] ; xx [ 442 ] =
3.734971681880274e-4 ; xx [ 457 ] = sin ( xx [ 439 ] ) ; xx [ 458 ] =
0.2610715860287157 ; xx [ 459 ] = 0.9653193707101895 ; xx [ 460 ] = cos ( xx
[ 439 ] ) ; xx [ 461 ] = xx [ 442 ] * xx [ 457 ] ; xx [ 462 ] = xx [ 458 ] *
xx [ 457 ] ; xx [ 463 ] = xx [ 459 ] * xx [ 457 ] ;
pm_math_Quaternion_compose_ra ( xx + 453 , xx + 460 , xx + 464 ) ; xx [ 439 ]
= xx [ 464 ] * xx [ 464 ] ; xx [ 453 ] = ( xx [ 439 ] + xx [ 465 ] * xx [ 465
] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 454 ] = xx [ 465 ] * xx [ 466 ] ; xx [ 455
] = xx [ 464 ] * xx [ 467 ] ; xx [ 456 ] = xx [ 2 ] * ( xx [ 454 ] - xx [ 455
] ) ; xx [ 457 ] = xx [ 465 ] * xx [ 467 ] ; xx [ 460 ] = xx [ 464 ] * xx [
466 ] ; xx [ 461 ] = ( xx [ 457 ] + xx [ 460 ] ) * xx [ 2 ] ; xx [ 462 ] = (
xx [ 454 ] + xx [ 455 ] ) * xx [ 2 ] ; xx [ 454 ] = ( xx [ 439 ] + xx [ 466 ]
* xx [ 466 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 455 ] = xx [ 466 ] * xx [ 467 ]
; xx [ 463 ] = xx [ 464 ] * xx [ 465 ] ; xx [ 468 ] = xx [ 2 ] * ( xx [ 455 ]
- xx [ 463 ] ) ; xx [ 469 ] = xx [ 2 ] * ( xx [ 457 ] - xx [ 460 ] ) ; xx [
457 ] = ( xx [ 455 ] + xx [ 463 ] ) * xx [ 2 ] ; xx [ 455 ] = ( xx [ 439 ] +
xx [ 467 ] * xx [ 467 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 470 ] = xx [ 453 ] ;
xx [ 471 ] = xx [ 456 ] ; xx [ 472 ] = xx [ 461 ] ; xx [ 473 ] = xx [ 462 ] ;
xx [ 474 ] = xx [ 454 ] ; xx [ 475 ] = xx [ 468 ] ; xx [ 476 ] = xx [ 469 ] ;
xx [ 477 ] = xx [ 457 ] ; xx [ 478 ] = xx [ 455 ] ; xx [ 479 ] =
0.4090279037421741 ; xx [ 480 ] = - 0.57698859548371 ; xx [ 481 ] = -
0.5768688874880377 ; xx [ 482 ] = - 0.4086595420277526 ; xx [ 439 ] = xx [ 30
] * input [ 1 ] ; xx [ 460 ] = 0.9972896958011036 ; xx [ 463 ] = sin ( xx [
439 ] ) ; xx [ 483 ] = 8.852854494773021e-5 ; xx [ 484 ] =
0.07357482457770348 ; xx [ 485 ] = cos ( xx [ 439 ] ) ; xx [ 486 ] = - ( xx [
460 ] * xx [ 463 ] ) ; xx [ 487 ] = xx [ 483 ] * xx [ 463 ] ; xx [ 488 ] = xx
[ 484 ] * xx [ 463 ] ; pm_math_Quaternion_compose_ra ( xx + 479 , xx + 485 ,
xx + 489 ) ; xx [ 439 ] = xx [ 489 ] * xx [ 489 ] ; xx [ 463 ] = ( xx [ 439 ]
+ xx [ 490 ] * xx [ 490 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 479 ] = xx [ 490 ]
* xx [ 491 ] ; xx [ 480 ] = xx [ 489 ] * xx [ 492 ] ; xx [ 481 ] = xx [ 2 ] *
( xx [ 479 ] - xx [ 480 ] ) ; xx [ 482 ] = xx [ 490 ] * xx [ 492 ] ; xx [ 485
] = xx [ 489 ] * xx [ 491 ] ; xx [ 486 ] = ( xx [ 482 ] + xx [ 485 ] ) * xx [
2 ] ; xx [ 487 ] = ( xx [ 479 ] + xx [ 480 ] ) * xx [ 2 ] ; xx [ 479 ] = ( xx
[ 439 ] + xx [ 491 ] * xx [ 491 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 480 ] = xx
[ 491 ] * xx [ 492 ] ; xx [ 488 ] = xx [ 489 ] * xx [ 490 ] ; xx [ 493 ] = xx
[ 2 ] * ( xx [ 480 ] - xx [ 488 ] ) ; xx [ 494 ] = xx [ 2 ] * ( xx [ 482 ] -
xx [ 485 ] ) ; xx [ 482 ] = ( xx [ 480 ] + xx [ 488 ] ) * xx [ 2 ] ; xx [ 480
] = ( xx [ 439 ] + xx [ 492 ] * xx [ 492 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [
495 ] = xx [ 463 ] ; xx [ 496 ] = xx [ 481 ] ; xx [ 497 ] = xx [ 486 ] ; xx [
498 ] = xx [ 487 ] ; xx [ 499 ] = xx [ 479 ] ; xx [ 500 ] = xx [ 493 ] ; xx [
501 ] = xx [ 494 ] ; xx [ 502 ] = xx [ 482 ] ; xx [ 503 ] = xx [ 480 ] ; xx [
504 ] = 0.6254151447093548 ; xx [ 505 ] = - 0.6480628292580688 ; xx [ 506 ] =
0.3298467071707108 ; xx [ 507 ] = - 0.2829692843240122 ; xx [ 439 ] = xx [ 30
] * input [ 2 ] ; xx [ 485 ] = sin ( xx [ 439 ] ) ; xx [ 488 ] =
1.899414133310628e-6 ; xx [ 508 ] = cos ( xx [ 439 ] ) ; xx [ 509 ] = - ( xx
[ 235 ] * xx [ 485 ] ) ; xx [ 510 ] = xx [ 256 ] * xx [ 485 ] ; xx [ 511 ] =
xx [ 488 ] * xx [ 485 ] ; pm_math_Quaternion_compose_ra ( xx + 504 , xx + 508
, xx + 512 ) ; xx [ 439 ] = xx [ 512 ] * xx [ 512 ] ; xx [ 485 ] = ( xx [ 439
] + xx [ 513 ] * xx [ 513 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 504 ] = xx [ 513
] * xx [ 514 ] ; xx [ 505 ] = xx [ 512 ] * xx [ 515 ] ; xx [ 506 ] = xx [ 2 ]
* ( xx [ 504 ] - xx [ 505 ] ) ; xx [ 507 ] = xx [ 513 ] * xx [ 515 ] ; xx [
508 ] = xx [ 512 ] * xx [ 514 ] ; xx [ 509 ] = ( xx [ 507 ] + xx [ 508 ] ) *
xx [ 2 ] ; xx [ 510 ] = ( xx [ 504 ] + xx [ 505 ] ) * xx [ 2 ] ; xx [ 504 ] =
( xx [ 439 ] + xx [ 514 ] * xx [ 514 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 505 ]
= xx [ 514 ] * xx [ 515 ] ; xx [ 511 ] = xx [ 512 ] * xx [ 513 ] ; xx [ 516 ]
= xx [ 2 ] * ( xx [ 505 ] - xx [ 511 ] ) ; xx [ 517 ] = xx [ 2 ] * ( xx [ 507
] - xx [ 508 ] ) ; xx [ 507 ] = ( xx [ 505 ] + xx [ 511 ] ) * xx [ 2 ] ; xx [
505 ] = ( xx [ 439 ] + xx [ 515 ] * xx [ 515 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 518 ] = xx [ 485 ] ; xx [ 519 ] = xx [ 506 ] ; xx [ 520 ] = xx [ 509 ] ; xx
[ 521 ] = xx [ 510 ] ; xx [ 522 ] = xx [ 504 ] ; xx [ 523 ] = xx [ 516 ] ; xx
[ 524 ] = xx [ 517 ] ; xx [ 525 ] = xx [ 507 ] ; xx [ 526 ] = xx [ 505 ] ; xx
[ 527 ] = xx [ 232 ] * xx [ 485 ] ; xx [ 528 ] = xx [ 232 ] * xx [ 510 ] ; xx
[ 529 ] = xx [ 232 ] * xx [ 517 ] ; xx [ 530 ] = xx [ 232 ] * xx [ 506 ] ; xx
[ 531 ] = xx [ 232 ] * xx [ 504 ] ; xx [ 532 ] = xx [ 232 ] * xx [ 507 ] ; xx
[ 533 ] = xx [ 232 ] * xx [ 509 ] ; xx [ 534 ] = xx [ 232 ] * xx [ 516 ] ; xx
[ 535 ] = xx [ 232 ] * xx [ 505 ] ; pm_math_Matrix3x3_compose_ra ( xx + 518 ,
xx + 527 , xx + 536 ) ; xx [ 527 ] = 0.4716196941540828 ; xx [ 528 ] = -
0.5117381057633613 ; xx [ 529 ] = 0.5267317275910312 ; xx [ 530 ] = -
0.4881113216723255 ; xx [ 439 ] = xx [ 30 ] * state [ 17 ] ; xx [ 30 ] =
3.05954793225871e-4 ; xx [ 508 ] = sin ( xx [ 439 ] ) ; xx [ 511 ] =
0.9999868227746886 ; xx [ 531 ] = 5.124516430675685e-3 ; xx [ 532 ] = cos (
xx [ 439 ] ) ; xx [ 533 ] = xx [ 30 ] * xx [ 508 ] ; xx [ 534 ] = - ( xx [
511 ] * xx [ 508 ] ) ; xx [ 535 ] = - ( xx [ 531 ] * xx [ 508 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 527 , xx + 532 , xx + 545 ) ; xx [ 439 ]
= xx [ 545 ] * xx [ 545 ] ; xx [ 508 ] = ( xx [ 439 ] + xx [ 546 ] * xx [ 546
] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 527 ] = xx [ 546 ] * xx [ 547 ] ; xx [ 528
] = xx [ 545 ] * xx [ 548 ] ; xx [ 529 ] = xx [ 2 ] * ( xx [ 527 ] - xx [ 528
] ) ; xx [ 530 ] = xx [ 546 ] * xx [ 548 ] ; xx [ 532 ] = xx [ 545 ] * xx [
547 ] ; xx [ 533 ] = ( xx [ 530 ] + xx [ 532 ] ) * xx [ 2 ] ; xx [ 534 ] = (
xx [ 527 ] + xx [ 528 ] ) * xx [ 2 ] ; xx [ 527 ] = ( xx [ 439 ] + xx [ 547 ]
* xx [ 547 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 528 ] = xx [ 547 ] * xx [ 548 ]
; xx [ 535 ] = xx [ 545 ] * xx [ 546 ] ; xx [ 549 ] = xx [ 2 ] * ( xx [ 528 ]
- xx [ 535 ] ) ; xx [ 550 ] = xx [ 2 ] * ( xx [ 530 ] - xx [ 532 ] ) ; xx [
530 ] = ( xx [ 528 ] + xx [ 535 ] ) * xx [ 2 ] ; xx [ 528 ] = ( xx [ 439 ] +
xx [ 548 ] * xx [ 548 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 551 ] = xx [ 508 ] ;
xx [ 552 ] = xx [ 529 ] ; xx [ 553 ] = xx [ 533 ] ; xx [ 554 ] = xx [ 534 ] ;
xx [ 555 ] = xx [ 527 ] ; xx [ 556 ] = xx [ 549 ] ; xx [ 557 ] = xx [ 550 ] ;
xx [ 558 ] = xx [ 530 ] ; xx [ 559 ] = xx [ 528 ] ; xx [ 439 ] =
2.163481363666267e-10 ; xx [ 560 ] = 6.381124474950281e-9 ; xx [ 561 ] = xx [
296 ] ; xx [ 562 ] = xx [ 296 ] ; xx [ 563 ] = 6.56313776204783e-4 ; ii [ 0 ]
= factorSymmetricPosDef ( xx + 560 , 2 , xx + 564 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Cylindrical3' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 296 ] = 2.208066226806122e-12 ; xx [ 532 ] =
6.381017038462023e-9 ; xx [ 535 ] = 3.696620156624781e-11 ; xx [ 564 ] =
2.008023456900249e-7 ; xx [ 565 ] = 4.871572110464938e-12 ; xx [ 566 ] =
6.563051278102788e-4 ; xx [ 567 ] = 9.37710949382431e-10 ; xx [ 568 ] =
3.363290729840214e-6 ; xx [ 569 ] = xx [ 296 ] ; xx [ 570 ] = xx [ 317 ] ; xx
[ 571 ] = - xx [ 532 ] ; xx [ 572 ] = xx [ 317 ] ; xx [ 573 ] = - xx [ 535 ]
; xx [ 574 ] = xx [ 317 ] ; xx [ 575 ] = xx [ 439 ] ; xx [ 576 ] = xx [ 564 ]
; xx [ 577 ] = xx [ 565 ] ; xx [ 578 ] = - xx [ 566 ] ; xx [ 579 ] = - xx [
567 ] ; xx [ 580 ] = - xx [ 568 ] ; solveSymmetricPosDef ( xx + 560 , xx +
569 , 2 , 6 , xx + 581 , xx + 593 ) ; xx [ 569 ] = - ( xx [ 565 ] * xx [ 587
] - xx [ 566 ] * xx [ 588 ] ) ; xx [ 570 ] = xx [ 567 ] * xx [ 587 ] + xx [
568 ] * xx [ 588 ] ; xx [ 571 ] = xx [ 567 ] * xx [ 589 ] + xx [ 568 ] * xx [
590 ] ; xx [ 572 ] = xx [ 258 ] - ( xx [ 439 ] * xx [ 587 ] + xx [ 564 ] * xx
[ 588 ] ) ; xx [ 573 ] = xx [ 569 ] ; xx [ 574 ] = xx [ 570 ] ; xx [ 575 ] =
xx [ 569 ] ; xx [ 576 ] = xx [ 258 ] - ( xx [ 565 ] * xx [ 589 ] - xx [ 566 ]
* xx [ 590 ] ) ; xx [ 577 ] = xx [ 571 ] ; xx [ 578 ] = xx [ 570 ] ; xx [ 579
] = xx [ 571 ] ; xx [ 580 ] = xx [ 258 ] + xx [ 567 ] * xx [ 591 ] + xx [ 568
] * xx [ 592 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 572 , xx + 551
, xx + 593 ) ; pm_math_Matrix3x3_compose_ra ( xx + 551 , xx + 593 , xx + 569
) ; xx [ 593 ] = xx [ 223 ] + xx [ 536 ] + xx [ 569 ] ; xx [ 594 ] = xx [ 537
] + xx [ 570 ] ; xx [ 595 ] = xx [ 538 ] + xx [ 571 ] ; xx [ 596 ] = xx [ 539
] + xx [ 572 ] ; xx [ 597 ] = xx [ 223 ] + xx [ 540 ] + xx [ 573 ] ; xx [ 598
] = xx [ 541 ] + xx [ 574 ] ; xx [ 599 ] = xx [ 542 ] + xx [ 575 ] ; xx [ 600
] = xx [ 543 ] + xx [ 576 ] ; xx [ 601 ] = xx [ 223 ] + xx [ 544 ] + xx [ 577
] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 593 , xx + 495 , xx + 602 )
; pm_math_Matrix3x3_compose_ra ( xx + 495 , xx + 602 , xx + 611 ) ; xx [ 602
] = - ( xx [ 296 ] * xx [ 587 ] ) ; xx [ 603 ] = - ( xx [ 296 ] * xx [ 589 ]
) ; xx [ 604 ] = - ( xx [ 296 ] * xx [ 591 ] ) ; xx [ 605 ] = xx [ 532 ] * xx
[ 587 ] ; xx [ 606 ] = xx [ 532 ] * xx [ 589 ] ; xx [ 607 ] = xx [ 532 ] * xx
[ 591 ] ; xx [ 608 ] = xx [ 535 ] * xx [ 587 ] ; xx [ 609 ] = xx [ 535 ] * xx
[ 589 ] ; xx [ 610 ] = xx [ 535 ] * xx [ 591 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 602 , xx + 551 , xx + 620 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 551 , xx + 620 , xx + 602 ) ; xx [ 223 ]
= 0.07357483435689677 ; xx [ 578 ] = 8.625263905800012e-8 ; xx [ 579 ] =
4.387911391441809e-3 ; xx [ 580 ] = 2.281586598679112e-5 ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 578 , xx + 620 ) ; xx [ 578 ] =
1.451813746698375e-3 + xx [ 223 ] * state [ 18 ] - xx [ 620 ] ; xx [ 579 ] =
1.128000373271654e-4 ; xx [ 580 ] = 1.995808362665768e-6 + xx [ 579 ] * state
[ 18 ] - xx [ 621 ] ; xx [ 623 ] = 0.9972896926297332 ; xx [ 620 ] =
0.02110873061376549 + xx [ 623 ] * state [ 18 ] - xx [ 622 ] ; xx [ 624 ] =
xx [ 578 ] ; xx [ 625 ] = xx [ 580 ] ; xx [ 626 ] = xx [ 620 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 569 , xx + 624 , xx + 627 ) ; xx [ 569
] = 9.000024890213373e-4 ; xx [ 570 ] = - 3.975614490536003e-8 ; xx [ 571 ] =
2.710692114195099e-3 ; xx [ 572 ] = - 2.919897873389293e-7 ;
pm_math_Quaternion_xform_ra ( xx + 512 , xx + 570 , xx + 573 ) ; xx [ 570 ] =
1.149808082944376e-6 ; xx [ 571 ] = 0.01362905791904227 ; xx [ 636 ] = xx [
569 ] - xx [ 573 ] ; xx [ 637 ] = xx [ 570 ] - xx [ 574 ] ; xx [ 638 ] = xx [
571 ] - xx [ 575 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 536 , xx + 636 ,
xx + 639 ) ; xx [ 536 ] = xx [ 602 ] - xx [ 627 ] - xx [ 639 ] ; xx [ 537 ] =
xx [ 603 ] - xx [ 630 ] - xx [ 642 ] ; xx [ 538 ] = xx [ 604 ] - xx [ 633 ] -
xx [ 645 ] ; xx [ 539 ] = xx [ 605 ] - xx [ 628 ] - xx [ 640 ] ; xx [ 540 ] =
xx [ 606 ] - xx [ 631 ] - xx [ 643 ] ; xx [ 541 ] = xx [ 607 ] - xx [ 634 ] -
xx [ 646 ] ; xx [ 542 ] = xx [ 608 ] - xx [ 629 ] - xx [ 641 ] ; xx [ 543 ] =
xx [ 609 ] - xx [ 632 ] - xx [ 644 ] ; xx [ 544 ] = xx [ 610 ] - xx [ 635 ] -
xx [ 647 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 536 , xx + 495 , xx
+ 648 ) ; pm_math_Matrix3x3_compose_ra ( xx + 495 , xx + 648 , xx + 657 ) ;
xx [ 648 ] = 9.122274404587457e-4 ; xx [ 649 ] = 1.459342321928613e-6 ; xx [
650 ] = - 0.01504973426522732 ; pm_math_Quaternion_xform_ra ( xx + 489 , xx +
648 , xx + 651 ) ; xx [ 648 ] = - ( 8.39853113781329e-5 + xx [ 651 ] ) ; xx [
649 ] = 1.467714669213244e-3 - xx [ 652 ] ; xx [ 650 ] = 0.03024959596740159
- xx [ 653 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 611 , xx + 648 , xx +
666 ) ; xx [ 572 ] = xx [ 657 ] - xx [ 666 ] ; xx [ 576 ] = xx [ 658 ] - xx [
669 ] ; xx [ 577 ] = xx [ 659 ] - xx [ 672 ] ; xx [ 621 ] = xx [ 660 ] - xx [
667 ] ; xx [ 622 ] = xx [ 661 ] - xx [ 670 ] ; xx [ 651 ] = xx [ 662 ] - xx [
673 ] ; xx [ 652 ] = xx [ 663 ] - xx [ 668 ] ; xx [ 653 ] = xx [ 664 ] - xx [
671 ] ; xx [ 654 ] = xx [ 665 ] - xx [ 674 ] ; xx [ 675 ] = xx [ 572 ] ; xx [
676 ] = xx [ 576 ] ; xx [ 677 ] = xx [ 577 ] ; xx [ 678 ] = xx [ 621 ] ; xx [
679 ] = xx [ 622 ] ; xx [ 680 ] = xx [ 651 ] ; xx [ 681 ] = xx [ 652 ] ; xx [
682 ] = xx [ 653 ] ; xx [ 683 ] = xx [ 654 ] ; xx [ 684 ] = xx [ 442 ] ; xx [
685 ] = xx [ 458 ] ; xx [ 686 ] = xx [ 459 ] ;
pm_math_Matrix3x3_transposeXform_ra ( xx + 675 , xx + 684 , xx + 687 ) ; xx [
690 ] = xx [ 334 ] + xx [ 611 ] ; xx [ 691 ] = xx [ 612 ] ; xx [ 692 ] = xx [
613 ] ; xx [ 693 ] = xx [ 614 ] ; xx [ 694 ] = xx [ 334 ] + xx [ 615 ] ; xx [
695 ] = xx [ 616 ] ; xx [ 696 ] = xx [ 617 ] ; xx [ 697 ] = xx [ 618 ] ; xx [
698 ] = xx [ 334 ] + xx [ 619 ] ; xx [ 655 ] = 6.480496605447651e-3 ; xx [
656 ] = 9.242608862973643e-5 ; xx [ 699 ] = 2.248932226692751e-5 ; xx [ 700 ]
= - xx [ 655 ] ; xx [ 701 ] = xx [ 656 ] ; xx [ 702 ] = - xx [ 699 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 690 , xx + 700 , xx + 703 ) ; xx [ 706 ] =
xx [ 687 ] + xx [ 703 ] ; xx [ 707 ] = 1.926582834992798e-6 ; xx [ 708 ] =
5.727332635362873e-6 ; xx [ 709 ] = 1.414037047065465e-6 ; xx [ 710 ] =
2.78779312581908e-6 ; xx [ 711 ] = 1.414039856752149e-6 ; xx [ 712 ] = xx [
709 ] * xx [ 485 ] ; xx [ 713 ] = xx [ 709 ] * xx [ 510 ] ; xx [ 714 ] = xx [
709 ] * xx [ 517 ] ; xx [ 715 ] = xx [ 710 ] * xx [ 506 ] ; xx [ 716 ] = xx [
710 ] * xx [ 504 ] ; xx [ 717 ] = xx [ 710 ] * xx [ 507 ] ; xx [ 718 ] = xx [
711 ] * xx [ 509 ] ; xx [ 719 ] = xx [ 711 ] * xx [ 516 ] ; xx [ 720 ] = xx [
711 ] * xx [ 505 ] ; pm_math_Matrix3x3_compose_ra ( xx + 518 , xx + 712 , xx
+ 721 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 639 , xx + 636 , xx + 518 ) ;
xx [ 639 ] = 7.216968897676388e-9 ; xx [ 640 ] = xx [ 532 ] * xx [ 581 ] ; xx
[ 641 ] = xx [ 535 ] * xx [ 581 ] ; xx [ 642 ] = 6.381101123669264e-9 ; xx [
643 ] = xx [ 535 ] * xx [ 583 ] ; xx [ 644 ] = 7.213598017749685e-9 ; xx [
712 ] = xx [ 639 ] - xx [ 296 ] * xx [ 581 ] ; xx [ 713 ] = xx [ 640 ] ; xx [
714 ] = xx [ 641 ] ; xx [ 715 ] = xx [ 640 ] ; xx [ 716 ] = xx [ 642 ] + xx [
532 ] * xx [ 583 ] ; xx [ 717 ] = xx [ 643 ] ; xx [ 718 ] = xx [ 641 ] ; xx [
719 ] = xx [ 643 ] ; xx [ 720 ] = xx [ 644 ] + xx [ 535 ] * xx [ 585 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 712 , xx + 551 , xx + 730 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 551 , xx + 730 , xx + 712 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 602 , xx + 624 , xx + 730 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 627 , xx + 624 , xx + 602 ) ; xx [ 627 ]
= 6.304651877425359e-6 ; xx [ 628 ] = 4.963260250276951e-6 ; xx [ 739 ] = xx
[ 708 ] + xx [ 721 ] - xx [ 518 ] + xx [ 712 ] - xx [ 730 ] - xx [ 730 ] - xx
[ 602 ] ; xx [ 740 ] = xx [ 722 ] - xx [ 519 ] + xx [ 713 ] - xx [ 731 ] - xx
[ 733 ] - xx [ 603 ] ; xx [ 741 ] = xx [ 723 ] - xx [ 520 ] + xx [ 714 ] - xx
[ 732 ] - xx [ 736 ] - xx [ 604 ] ; xx [ 742 ] = xx [ 724 ] - xx [ 521 ] + xx
[ 715 ] - xx [ 733 ] - xx [ 731 ] - xx [ 605 ] ; xx [ 743 ] = xx [ 627 ] + xx
[ 725 ] - xx [ 522 ] + xx [ 716 ] - xx [ 734 ] - xx [ 734 ] - xx [ 606 ] ; xx
[ 744 ] = xx [ 726 ] - xx [ 523 ] + xx [ 717 ] - xx [ 735 ] - xx [ 737 ] - xx
[ 607 ] ; xx [ 745 ] = xx [ 727 ] - xx [ 524 ] + xx [ 718 ] - xx [ 736 ] - xx
[ 732 ] - xx [ 608 ] ; xx [ 746 ] = xx [ 728 ] - xx [ 525 ] + xx [ 719 ] - xx
[ 737 ] - xx [ 735 ] - xx [ 609 ] ; xx [ 747 ] = xx [ 628 ] + xx [ 729 ] - xx
[ 526 ] + xx [ 720 ] - xx [ 738 ] - xx [ 738 ] - xx [ 610 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 739 , xx + 495 , xx + 518 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 495 , xx + 518 , xx + 602 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 657 , xx + 648 , xx + 518 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 666 , xx + 648 , xx + 657 ) ; xx [ 629 ]
= xx [ 602 ] - xx [ 518 ] - xx [ 518 ] - xx [ 657 ] ; xx [ 630 ] = xx [ 603 ]
- xx [ 519 ] - xx [ 521 ] - xx [ 658 ] ; xx [ 631 ] = xx [ 604 ] - xx [ 520 ]
- xx [ 524 ] - xx [ 659 ] ; xx [ 632 ] = xx [ 605 ] - xx [ 521 ] - xx [ 519 ]
- xx [ 660 ] ; xx [ 633 ] = 1.24452555676786e-6 ; xx [ 634 ] = xx [ 606 ] -
xx [ 522 ] - xx [ 522 ] - xx [ 661 ] ; xx [ 635 ] = xx [ 607 ] - xx [ 523 ] -
xx [ 525 ] - xx [ 662 ] ; xx [ 640 ] = xx [ 608 ] - xx [ 524 ] - xx [ 520 ] -
xx [ 663 ] ; xx [ 641 ] = xx [ 609 ] - xx [ 525 ] - xx [ 523 ] - xx [ 664 ] ;
xx [ 643 ] = 1.115373634817654e-6 ; xx [ 518 ] = xx [ 610 ] - xx [ 526 ] - xx
[ 526 ] - xx [ 665 ] ; xx [ 602 ] = xx [ 707 ] + xx [ 629 ] ; xx [ 603 ] = xx
[ 630 ] ; xx [ 604 ] = xx [ 631 ] ; xx [ 605 ] = xx [ 632 ] ; xx [ 606 ] = xx
[ 633 ] + xx [ 634 ] ; xx [ 607 ] = xx [ 635 ] ; xx [ 608 ] = xx [ 640 ] ; xx
[ 609 ] = xx [ 641 ] ; xx [ 610 ] = xx [ 643 ] + xx [ 518 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 602 , xx + 684 , xx + 519 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 675 , xx + 700 , xx + 522 ) ; xx [ 525 ] =
xx [ 519 ] + xx [ 522 ] ; xx [ 526 ] = xx [ 520 ] + xx [ 523 ] ; xx [ 519 ] =
xx [ 521 ] + xx [ 524 ] ; xx [ 520 ] = xx [ 525 ] ; xx [ 521 ] = xx [ 526 ] ;
xx [ 522 ] = xx [ 519 ] ; xx [ 523 ] = xx [ 688 ] + xx [ 704 ] ; xx [ 524 ] =
xx [ 689 ] + xx [ 705 ] ; xx [ 645 ] = xx [ 706 ] ; xx [ 646 ] = xx [ 523 ] ;
xx [ 647 ] = xx [ 524 ] ; pm_math_Matrix3x3_xform_ra ( xx + 675 , xx + 684 ,
xx + 657 ) ; pm_math_Matrix3x3_xform_ra ( xx + 690 , xx + 684 , xx + 660 ) ;
xx [ 663 ] = pm_math_Vector3_dot_ra ( xx + 684 , xx + 657 ) +
pm_math_Vector3_dot_ra ( xx + 700 , xx + 660 ) ; xx [ 664 ] =
pm_math_Vector3_dot_ra ( xx + 684 , xx + 520 ) + pm_math_Vector3_dot_ra ( xx
+ 700 , xx + 645 ) ; xx [ 665 ] = xx [ 663 ] ; xx [ 666 ] = xx [ 663 ] ; xx [
667 ] = pm_math_Vector3_dot_ra ( xx + 684 , xx + 660 ) ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 664 , 2 , xx + 520 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/Cylindrical1' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 712 ] = xx [ 525 ] ; xx [ 713 ] = xx [ 657 ] ; xx [
714 ] = xx [ 526 ] ; xx [ 715 ] = xx [ 658 ] ; xx [ 716 ] = xx [ 519 ] ; xx [
717 ] = xx [ 659 ] ; xx [ 718 ] = xx [ 706 ] ; xx [ 719 ] = xx [ 660 ] ; xx [
720 ] = xx [ 523 ] ; xx [ 721 ] = xx [ 661 ] ; xx [ 722 ] = xx [ 524 ] ; xx [
723 ] = xx [ 662 ] ; solveSymmetricPosDef ( xx + 664 , xx + 712 , 2 , 6 , xx
+ 724 , xx + 520 ) ; xx [ 520 ] = xx [ 523 ] * xx [ 730 ] + xx [ 661 ] * xx [
731 ] ; xx [ 521 ] = xx [ 524 ] * xx [ 730 ] + xx [ 662 ] * xx [ 731 ] ; xx [
522 ] = xx [ 524 ] * xx [ 732 ] + xx [ 662 ] * xx [ 733 ] ; xx [ 712 ] = xx [
611 ] - ( xx [ 706 ] * xx [ 730 ] + xx [ 660 ] * xx [ 731 ] ) + xx [ 334 ] ;
xx [ 713 ] = xx [ 612 ] - xx [ 520 ] ; xx [ 714 ] = xx [ 613 ] - xx [ 521 ] ;
xx [ 715 ] = xx [ 614 ] - xx [ 520 ] ; xx [ 716 ] = xx [ 615 ] - ( xx [ 523 ]
* xx [ 732 ] + xx [ 661 ] * xx [ 733 ] ) + xx [ 334 ] ; xx [ 717 ] = xx [ 616
] - xx [ 522 ] ; xx [ 718 ] = xx [ 617 ] - xx [ 521 ] ; xx [ 719 ] = xx [ 618
] - xx [ 522 ] ; xx [ 720 ] = xx [ 619 ] - ( xx [ 524 ] * xx [ 734 ] + xx [
662 ] * xx [ 735 ] ) + xx [ 334 ] ; pm_math_Matrix3x3_composeTranspose_ra (
xx + 712 , xx + 470 , xx + 611 ) ; pm_math_Matrix3x3_compose_ra ( xx + 470 ,
xx + 611 , xx + 712 ) ; xx [ 520 ] = 0.9962649840775994 ; xx [ 521 ] =
5.30859247905514e-3 ; xx [ 522 ] = xx [ 520 ] * xx [ 166 ] - xx [ 521 ] * xx
[ 181 ] ; xx [ 611 ] = xx [ 521 ] * xx [ 196 ] - xx [ 520 ] * xx [ 181 ] ; xx
[ 612 ] = xx [ 160 ] * xx [ 522 ] - 2.846654567045481e-4 + xx [ 167 ] * xx [
611 ] ; xx [ 613 ] = 0.996191307543054 ; xx [ 614 ] = 0.01322794121964153 ;
xx [ 615 ] = xx [ 613 ] * xx [ 202 ] + xx [ 614 ] * xx [ 204 ] ; xx [ 616 ] =
2.846654567045372e-4 ; xx [ 617 ] = xx [ 614 ] * xx [ 208 ] + xx [ 613 ] * xx
[ 204 ] ; xx [ 618 ] = xx [ 199 ] * xx [ 615 ] - xx [ 616 ] + xx [ 203 ] * xx
[ 617 ] ; xx [ 619 ] = 2.327041772174898e-3 ; xx [ 645 ] =
0.06471387649624442 ; xx [ 646 ] = 0.7067127023886455 ; xx [ 647 ] = xx [ 645
] * xx [ 166 ] + xx [ 646 ] * xx [ 181 ] ; xx [ 166 ] = xx [ 646 ] * xx [ 196
] + xx [ 645 ] * xx [ 181 ] ; xx [ 181 ] = xx [ 619 ] + xx [ 160 ] * xx [ 647
] - xx [ 167 ] * xx [ 166 ] ; xx [ 160 ] = 2.327041772174903e-3 ; xx [ 167 ]
= 0.07032963731013592 ; xx [ 196 ] = 0.7061759494945001 ; xx [ 663 ] = xx [
167 ] * xx [ 202 ] - xx [ 196 ] * xx [ 204 ] ; xx [ 202 ] = xx [ 196 ] * xx [
208 ] - xx [ 167 ] * xx [ 204 ] ; xx [ 204 ] = xx [ 160 ] + xx [ 199 ] * xx [
663 ] - xx [ 203 ] * xx [ 202 ] ; xx [ 199 ] = xx [ 520 ] * xx [ 183 ] -
2.846654567045481e-4 + xx [ 521 ] * xx [ 197 ] ; xx [ 203 ] = xx [ 613 ] * xx
[ 205 ] - xx [ 616 ] + xx [ 614 ] * xx [ 209 ] ; xx [ 208 ] =
3.482877392081089e-5 + xx [ 520 ] * xx [ 522 ] ; xx [ 616 ] = xx [ 521 ] * xx
[ 611 ] ; xx [ 668 ] = 3.482877392080813e-5 + xx [ 613 ] * xx [ 615 ] ; xx [
669 ] = xx [ 614 ] * xx [ 617 ] ; xx [ 670 ] = 2.847131953613947e-4 ; xx [
671 ] = xx [ 520 ] * xx [ 647 ] - xx [ 670 ] - xx [ 521 ] * xx [ 166 ] ; xx [
520 ] = 2.847131953613837e-4 ; xx [ 521 ] = xx [ 613 ] * xx [ 663 ] - xx [
520 ] - xx [ 614 ] * xx [ 202 ] ; xx [ 613 ] = xx [ 619 ] + xx [ 645 ] * xx [
183 ] - xx [ 646 ] * xx [ 197 ] ; xx [ 183 ] = xx [ 160 ] + xx [ 167 ] * xx [
205 ] - xx [ 196 ] * xx [ 209 ] ; xx [ 160 ] = xx [ 645 ] * xx [ 522 ] - xx [
670 ] - xx [ 646 ] * xx [ 611 ] ; xx [ 197 ] = xx [ 167 ] * xx [ 615 ] - xx [
520 ] - xx [ 196 ] * xx [ 617 ] ; xx [ 205 ] = 2.327432019203518e-3 + xx [
645 ] * xx [ 647 ] ; xx [ 209 ] = xx [ 646 ] * xx [ 166 ] ; xx [ 166 ] =
2.327432019203522e-3 + xx [ 167 ] * xx [ 663 ] ; xx [ 167 ] = xx [ 196 ] * xx
[ 202 ] ; xx [ 748 ] = xx [ 69 ] + xx [ 102 ] + xx [ 136 ] + xx [ 169 ] + xx
[ 185 ] + xx [ 194 ] + xx [ 198 ] + xx [ 206 ] + xx [ 210 ] + xx [ 444 ] + xx
[ 434 ] + xx [ 712 ] ; xx [ 749 ] = xx [ 70 ] + xx [ 103 ] + xx [ 137 ] + xx
[ 170 ] + xx [ 186 ] + xx [ 612 ] + xx [ 618 ] + xx [ 445 ] + xx [ 713 ] ; xx
[ 750 ] = xx [ 71 ] + xx [ 104 ] + xx [ 138 ] + xx [ 171 ] + xx [ 187 ] + xx
[ 181 ] + xx [ 204 ] + xx [ 446 ] + xx [ 714 ] ; xx [ 751 ] = xx [ 72 ] + xx
[ 105 ] + xx [ 139 ] + xx [ 172 ] + xx [ 188 ] + xx [ 199 ] + xx [ 203 ] + xx
[ 447 ] + xx [ 715 ] ; xx [ 752 ] = xx [ 73 ] + xx [ 106 ] + xx [ 140 ] + xx
[ 173 ] + xx [ 189 ] + xx [ 208 ] + xx [ 616 ] + xx [ 668 ] + xx [ 669 ] + xx
[ 448 ] + xx [ 434 ] + xx [ 716 ] ; xx [ 753 ] = xx [ 74 ] + xx [ 107 ] + xx
[ 141 ] + xx [ 174 ] + xx [ 190 ] + xx [ 671 ] + xx [ 521 ] + xx [ 449 ] + xx
[ 717 ] ; xx [ 754 ] = xx [ 75 ] + xx [ 108 ] + xx [ 142 ] + xx [ 175 ] + xx
[ 191 ] + xx [ 613 ] + xx [ 183 ] + xx [ 450 ] + xx [ 718 ] ; xx [ 755 ] = xx
[ 76 ] + xx [ 109 ] + xx [ 143 ] + xx [ 176 ] + xx [ 192 ] + xx [ 160 ] + xx
[ 197 ] + xx [ 451 ] + xx [ 719 ] ; xx [ 756 ] = xx [ 77 ] + xx [ 110 ] + xx
[ 144 ] + xx [ 177 ] + xx [ 193 ] + xx [ 205 ] + xx [ 209 ] + xx [ 166 ] + xx
[ 167 ] + xx [ 452 ] + xx [ 434 ] + xx [ 720 ] ; pm_math_Matrix3x3_xform_ra (
xx + 748 , xx + 26 , xx + 645 ) ; xx [ 196 ] = ( xx [ 21 ] + xx [ 20 ] ) * xx
[ 2 ] ; xx [ 20 ] = xx [ 17 ] * xx [ 17 ] ; xx [ 21 ] = xx [ 11 ] - ( xx [ 1
] + xx [ 20 ] ) * xx [ 2 ] ; xx [ 1 ] = xx [ 18 ] * xx [ 19 ] ; xx [ 202 ] =
xx [ 16 ] * xx [ 17 ] ; xx [ 434 ] = xx [ 2 ] * ( xx [ 1 ] - xx [ 202 ] ) ;
xx [ 672 ] = xx [ 196 ] ; xx [ 673 ] = xx [ 21 ] ; xx [ 674 ] = xx [ 434 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 748 , xx + 672 , xx + 687 ) ; xx [ 520 ] =
pm_math_Vector3_dot_ra ( xx + 26 , xx + 687 ) ; xx [ 522 ] = xx [ 2 ] * ( xx
[ 24 ] - xx [ 23 ] ) ; xx [ 23 ] = ( xx [ 202 ] + xx [ 1 ] ) * xx [ 2 ] ; xx
[ 1 ] = xx [ 11 ] - ( xx [ 20 ] + xx [ 0 ] ) * xx [ 2 ] ; xx [ 703 ] = xx [
522 ] ; xx [ 704 ] = xx [ 23 ] ; xx [ 705 ] = xx [ 1 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 748 , xx + 703 , xx + 721 ) ; xx [ 0 ] =
pm_math_Vector3_dot_ra ( xx + 26 , xx + 721 ) ; xx [ 757 ] = - ( xx [ 316 ] *
xx [ 343 ] ) ; xx [ 758 ] = - ( xx [ 316 ] * xx [ 345 ] ) ; xx [ 759 ] = - (
xx [ 316 ] * xx [ 347 ] ) ; xx [ 760 ] = xx [ 318 ] * xx [ 343 ] ; xx [ 761 ]
= xx [ 318 ] * xx [ 345 ] ; xx [ 762 ] = xx [ 318 ] * xx [ 347 ] ; xx [ 763 ]
= xx [ 319 ] * xx [ 343 ] ; xx [ 764 ] = xx [ 319 ] * xx [ 345 ] ; xx [ 765 ]
= xx [ 319 ] * xx [ 347 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 757
, xx + 303 , xx + 766 ) ; pm_math_Matrix3x3_compose_ra ( xx + 303 , xx + 766
, xx + 757 ) ; xx [ 20 ] = 1.451813746738863e-3 ; xx [ 24 ] =
0.07357483435904683 ; xx [ 736 ] = 8.62526392261675e-8 ; xx [ 737 ] =
4.387911391441729e-3 ; xx [ 738 ] = 2.28158659870934e-5 ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 736 , xx + 766 ) ; xx [ 202 ] =
xx [ 20 ] + xx [ 24 ] * state [ 28 ] - xx [ 766 ] ; xx [ 611 ] =
1.995808348478373e-6 ; xx [ 614 ] = 1.128000365619997e-4 ; xx [ 615 ] = xx [
611 ] + xx [ 614 ] * state [ 28 ] - xx [ 767 ] ; xx [ 617 ] =
0.02110873061376147 ; xx [ 619 ] = 0.9972896926295751 ; xx [ 663 ] = xx [ 617
] + xx [ 619 ] * state [ 28 ] - xx [ 768 ] ; xx [ 736 ] = xx [ 202 ] ; xx [
737 ] = xx [ 615 ] ; xx [ 738 ] = xx [ 663 ] ; pm_math_Matrix3x3_postCross_ra
( xx + 325 , xx + 736 , xx + 766 ) ; xx [ 325 ] = - 3.968710232179073e-8 ; xx
[ 326 ] = 2.210692114196348e-3 ; xx [ 327 ] = - 2.929394946184722e-7 ;
pm_math_Quaternion_xform_ra ( xx + 262 , xx + 325 , xx + 328 ) ; xx [ 325 ] =
xx [ 20 ] - xx [ 328 ] ; xx [ 326 ] = xx [ 611 ] - xx [ 329 ] ; xx [ 327 ] =
xx [ 617 ] - xx [ 330 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 286 , xx +
325 , xx + 775 ) ; xx [ 286 ] = xx [ 757 ] - xx [ 766 ] - xx [ 775 ] ; xx [
287 ] = xx [ 758 ] - xx [ 769 ] - xx [ 778 ] ; xx [ 288 ] = xx [ 759 ] - xx [
772 ] - xx [ 781 ] ; xx [ 289 ] = xx [ 760 ] - xx [ 767 ] - xx [ 776 ] ; xx [
290 ] = xx [ 761 ] - xx [ 770 ] - xx [ 779 ] ; xx [ 291 ] = xx [ 762 ] - xx [
773 ] - xx [ 782 ] ; xx [ 292 ] = xx [ 763 ] - xx [ 768 ] - xx [ 777 ] ; xx [
293 ] = xx [ 764 ] - xx [ 771 ] - xx [ 780 ] ; xx [ 294 ] = xx [ 765 ] - xx [
774 ] - xx [ 783 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 286 , xx +
242 , xx + 784 ) ; pm_math_Matrix3x3_compose_ra ( xx + 242 , xx + 784 , xx +
793 ) ; xx [ 331 ] = 3.383773468563022e-7 ; xx [ 784 ] = 9.122274404458714e-4
; xx [ 785 ] = 1.459342330053661e-6 ; xx [ 786 ] = - 0.0150497342652287 ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 784 , xx + 787 ) ; xx [ 332 ] =
4.111205245038918e-3 ; xx [ 333 ] = 7.889999999975156e-3 ; xx [ 784 ] = xx [
331 ] - xx [ 787 ] ; xx [ 785 ] = xx [ 332 ] - xx [ 788 ] ; xx [ 786 ] = xx [
333 ] - xx [ 789 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 367 , xx + 784 ,
xx + 802 ) ; xx [ 367 ] = - ( xx [ 376 ] * xx [ 421 ] ) ; xx [ 368 ] = - ( xx
[ 376 ] * xx [ 423 ] ) ; xx [ 369 ] = - ( xx [ 376 ] * xx [ 425 ] ) ; xx [
370 ] = - ( xx [ 379 ] * xx [ 421 ] ) ; xx [ 371 ] = - ( xx [ 379 ] * xx [
423 ] ) ; xx [ 372 ] = - ( xx [ 379 ] * xx [ 425 ] ) ; xx [ 373 ] = - ( xx [
397 ] * xx [ 421 ] ) ; xx [ 374 ] = - ( xx [ 397 ] * xx [ 423 ] ) ; xx [ 375
] = - ( xx [ 397 ] * xx [ 425 ] ) ; pm_math_Matrix3x3_composeTranspose_ra (
xx + 367 , xx + 384 , xx + 811 ) ; pm_math_Matrix3x3_compose_ra ( xx + 384 ,
xx + 811 , xx + 367 ) ; xx [ 670 ] = 3.383773514455306e-7 ; xx [ 790 ] = -
8.9928915845163e-5 ; xx [ 791 ] = - 2.64677354170138e-3 ; xx [ 792 ] =
0.01503616266891429 ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 790 , xx
+ 811 ) ; xx [ 790 ] = 4.111205245035695e-3 ; xx [ 791 ] =
0.02364999999999497 - state [ 24 ] - xx [ 813 ] ; xx [ 813 ] = xx [ 670 ] -
xx [ 811 ] ; xx [ 814 ] = xx [ 790 ] - xx [ 812 ] ; xx [ 815 ] = xx [ 791 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 403 , xx + 813 , xx + 816 ) ; xx [ 403
] = xx [ 793 ] - xx [ 802 ] + xx [ 367 ] - xx [ 816 ] ; xx [ 404 ] = xx [ 794
] - xx [ 805 ] + xx [ 368 ] - xx [ 819 ] ; xx [ 405 ] = xx [ 403 ] * xx [ 413
] + xx [ 404 ] * xx [ 427 ] ; xx [ 406 ] = xx [ 795 ] - xx [ 808 ] + xx [ 369
] - xx [ 822 ] ; xx [ 407 ] = xx [ 796 ] - xx [ 803 ] + xx [ 370 ] - xx [ 817
] ; xx [ 408 ] = xx [ 797 ] - xx [ 806 ] + xx [ 371 ] - xx [ 820 ] ; xx [ 409
] = xx [ 407 ] * xx [ 413 ] + xx [ 408 ] * xx [ 427 ] ; xx [ 410 ] = xx [ 798
] - xx [ 809 ] + xx [ 372 ] - xx [ 823 ] ; xx [ 411 ] = xx [ 799 ] - xx [ 804
] + xx [ 373 ] - xx [ 818 ] ; xx [ 792 ] = xx [ 800 ] - xx [ 807 ] + xx [ 374
] - xx [ 821 ] ; xx [ 825 ] = xx [ 411 ] * xx [ 413 ] + xx [ 792 ] * xx [ 427
] ; xx [ 826 ] = xx [ 801 ] - xx [ 810 ] + xx [ 375 ] - xx [ 824 ] ; xx [ 827
] = xx [ 403 ] - xx [ 405 ] * xx [ 433 ] ; xx [ 828 ] = xx [ 404 ] - xx [ 405
] * xx [ 440 ] ; xx [ 829 ] = xx [ 406 ] - xx [ 405 ] * xx [ 222 ] ; xx [ 830
] = xx [ 407 ] - xx [ 409 ] * xx [ 433 ] ; xx [ 831 ] = xx [ 408 ] - xx [ 409
] * xx [ 440 ] ; xx [ 832 ] = xx [ 410 ] - xx [ 409 ] * xx [ 222 ] ; xx [ 833
] = xx [ 411 ] - xx [ 825 ] * xx [ 433 ] ; xx [ 834 ] = xx [ 792 ] - xx [ 825
] * xx [ 440 ] ; xx [ 835 ] = xx [ 826 ] - xx [ 825 ] * xx [ 222 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 827 , xx + 213 , xx + 836 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 213 , xx + 836 , xx + 827 ) ; xx [ 836 ]
= 0.08324084851537972 ; xx [ 837 ] = 0.7044167893867059 ; xx [ 838 ] = xx [
837 ] * state [ 21 ] ; xx [ 839 ] = 4.5034434673372e-3 ; xx [ 840 ] =
0.08618526586337766 ; xx [ 841 ] = xx [ 840 ] * state [ 21 ] ; xx [ 842 ] =
0.7045346597422903 ; xx [ 843 ] = xx [ 842 ] * state [ 21 ] ; xx [ 844 ] =
0.09384704203289078 ; xx [ 845 ] = xx [ 836 ] + xx [ 838 ] ; xx [ 846 ] = xx
[ 839 ] - xx [ 841 ] ; xx [ 847 ] = xx [ 843 ] - xx [ 844 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 444 , xx + 845 , xx + 848 ) ; xx [ 444
] = - xx [ 41 ] ; xx [ 445 ] = xx [ 36 ] ; xx [ 446 ] = xx [ 39 ] ; xx [ 447
] = xx [ 444 ] ; xx [ 448 ] = 1.570981369614329e-13 ; xx [ 449 ] = xx [ 448 ]
* xx [ 41 ] ; xx [ 450 ] = 0.02074999999999926 ; xx [ 451 ] = xx [ 450 ] * xx
[ 41 ] ; xx [ 41 ] = xx [ 36 ] * xx [ 448 ] + xx [ 39 ] * xx [ 450 ] ; xx [
857 ] = xx [ 449 ] ; xx [ 858 ] = xx [ 451 ] ; xx [ 859 ] = xx [ 41 ] ;
pm_math_Vector3_cross_ra ( xx + 445 , xx + 857 , xx + 860 ) ; xx [ 445 ] =
9.407826863410462e-3 - ( xx [ 2 ] * ( xx [ 860 ] + xx [ 449 ] * xx [ 31 ] ) -
xx [ 450 ] ) ; xx [ 446 ] = - ( 0.01511804393362148 + xx [ 448 ] + ( xx [ 451
] * xx [ 31 ] + xx [ 861 ] ) * xx [ 2 ] ) ; xx [ 447 ] = 0.03122651021731275
- ( xx [ 41 ] * xx [ 31 ] + xx [ 862 ] ) * xx [ 2 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 69 , xx + 445 , xx + 857 ) ; xx [ 41 ]
= xx [ 44 ] * xx [ 58 ] ; xx [ 69 ] = xx [ 42 ] * xx [ 41 ] ; xx [ 70 ] = xx
[ 69 ] * xx [ 33 ] ; xx [ 71 ] = xx [ 64 ] ; xx [ 72 ] = xx [ 66 ] ; xx [ 73
] = xx [ 68 ] ; xx [ 74 ] = 1.56860771278768e-13 ; xx [ 75 ] = xx [ 68 ] * xx
[ 74 ] ; xx [ 76 ] = 0.02149999999999999 ; xx [ 77 ] = xx [ 68 ] * xx [ 76 ]
; xx [ 449 ] = xx [ 74 ] * xx [ 64 ] + xx [ 76 ] * xx [ 66 ] ; xx [ 450 ] = -
xx [ 75 ] ; xx [ 451 ] = - xx [ 77 ] ; xx [ 452 ] = xx [ 449 ] ;
pm_math_Vector3_cross_ra ( xx + 71 , xx + 450 , xx + 866 ) ; xx [ 71 ] =
0.01923252987342762 - ( xx [ 2 ] * ( xx [ 866 ] - xx [ 59 ] * xx [ 75 ] ) -
xx [ 76 ] ) ; xx [ 72 ] = 9.693066419407315e-3 - ( xx [ 74 ] + ( xx [ 867 ] -
xx [ 59 ] * xx [ 77 ] ) * xx [ 2 ] ) ; xx [ 73 ] = 0.04245578950477404 - ( xx
[ 59 ] * xx [ 449 ] + xx [ 868 ] ) * xx [ 2 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 102 , xx + 71 , xx + 866 ) ; xx [ 75 ]
= xx [ 44 ] * xx [ 80 ] ; xx [ 77 ] = xx [ 78 ] * xx [ 75 ] ; xx [ 102 ] = xx
[ 77 ] * xx [ 29 ] ; xx [ 103 ] = xx [ 98 ] ; xx [ 104 ] = xx [ 100 ] ; xx [
105 ] = xx [ 111 ] ; xx [ 106 ] = 1.56851889494571e-13 ; xx [ 107 ] = xx [
111 ] * xx [ 106 ] ; xx [ 108 ] = xx [ 111 ] * xx [ 76 ] ; xx [ 109 ] = xx [
106 ] * xx [ 98 ] + xx [ 76 ] * xx [ 100 ] ; xx [ 449 ] = - xx [ 107 ] ; xx [
450 ] = - xx [ 108 ] ; xx [ 451 ] = xx [ 109 ] ; pm_math_Vector3_cross_ra (
xx + 103 , xx + 449 , xx + 875 ) ; xx [ 103 ] = - ( 8.919807266753083e-3 + xx
[ 2 ] * ( xx [ 875 ] - xx [ 93 ] * xx [ 107 ] ) - xx [ 76 ] ) ; xx [ 104 ] =
9.941671475675939e-3 - ( xx [ 106 ] + ( xx [ 876 ] - xx [ 93 ] * xx [ 108 ] )
* xx [ 2 ] ) ; xx [ 105 ] = - ( 0.04661092377057491 + ( xx [ 93 ] * xx [ 109
] + xx [ 877 ] ) * xx [ 2 ] ) ; pm_math_Matrix3x3_postCross_ra ( xx + 136 ,
xx + 103 , xx + 875 ) ; xx [ 76 ] = xx [ 44 ] * xx [ 114 ] ; xx [ 107 ] = xx
[ 112 ] * xx [ 76 ] ; xx [ 108 ] = xx [ 107 ] * xx [ 92 ] ; xx [ 136 ] = xx [
132 ] ; xx [ 137 ] = xx [ 134 ] ; xx [ 138 ] = xx [ 145 ] ; xx [ 109 ] = xx [
145 ] * xx [ 106 ] ; xx [ 110 ] = 0.02149999999999999 ; xx [ 139 ] = xx [ 145
] * xx [ 110 ] ; xx [ 140 ] = xx [ 106 ] * xx [ 132 ] + xx [ 110 ] * xx [ 134
] ; xx [ 141 ] = - xx [ 109 ] ; xx [ 142 ] = - xx [ 139 ] ; xx [ 143 ] = xx [
140 ] ; pm_math_Vector3_cross_ra ( xx + 136 , xx + 141 , xx + 449 ) ; xx [
136 ] = - ( 0.04034737334080394 + xx [ 2 ] * ( xx [ 449 ] - xx [ 127 ] * xx [
109 ] ) - xx [ 110 ] ) ; xx [ 137 ] = - ( 0.01130093100527478 + xx [ 106 ] +
( xx [ 450 ] - xx [ 127 ] * xx [ 139 ] ) * xx [ 2 ] ) ; xx [ 138 ] = - (
0.07837290165097587 + ( xx [ 127 ] * xx [ 140 ] + xx [ 451 ] ) * xx [ 2 ] ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 169 , xx + 136 , xx + 884 ) ; xx [ 109
] = xx [ 44 ] * xx [ 131 ] ; xx [ 110 ] = xx [ 146 ] * xx [ 109 ] ; xx [ 139
] = xx [ 110 ] * xx [ 126 ] ; xx [ 140 ] = - ( 0.01025589154524426 +
0.7077023583577412 * state [ 35 ] ) ; xx [ 141 ] = - ( 0.02452133274909705 +
9.268340058974089e-3 * state [ 35 ] ) ; xx [ 142 ] = 0.7064499061132667 *
state [ 35 ] - 0.01799698139187411 ; pm_math_Matrix3x3_postCross_ra ( xx +
185 , xx + 140 , xx + 169 ) ; xx [ 185 ] = xx [ 194 ] + xx [ 198 ] ; xx [ 186
] = xx [ 612 ] ; xx [ 187 ] = xx [ 181 ] ; xx [ 188 ] = xx [ 199 ] ; xx [ 189
] = xx [ 208 ] + xx [ 616 ] ; xx [ 190 ] = xx [ 671 ] ; xx [ 191 ] = xx [ 613
] ; xx [ 192 ] = xx [ 160 ] ; xx [ 193 ] = xx [ 205 ] + xx [ 209 ] ; xx [ 143
] = 0.7077023583577554 ; xx [ 144 ] = xx [ 143 ] * state [ 33 ] ; xx [ 160 ]
= 9.268340058966987e-3 ; xx [ 181 ] = xx [ 160 ] * state [ 33 ] ; xx [ 194 ]
= 0.7064499061132521 ; xx [ 198 ] = xx [ 194 ] * state [ 33 ] ; xx [ 449 ] =
0.05431153664435735 - xx [ 144 ] ; xx [ 450 ] = 7.88370615320499e-4 - xx [
181 ] ; xx [ 451 ] = xx [ 198 ] - 0.06537860523419856 ;
pm_math_Matrix3x3_postCross_ra ( xx + 185 , xx + 449 , xx + 893 ) ; xx [ 185
] = xx [ 206 ] + xx [ 210 ] ; xx [ 186 ] = xx [ 618 ] ; xx [ 187 ] = xx [ 204
] ; xx [ 188 ] = xx [ 203 ] ; xx [ 189 ] = xx [ 668 ] + xx [ 669 ] ; xx [ 190
] = xx [ 521 ] ; xx [ 191 ] = xx [ 183 ] ; xx [ 192 ] = xx [ 197 ] ; xx [ 193
] = xx [ 166 ] + xx [ 167 ] ; xx [ 166 ] = 0.7077023583577557 ; xx [ 167 ] =
xx [ 166 ] * state [ 31 ] ; xx [ 183 ] = 9.268340058967206e-3 ; xx [ 197 ] =
xx [ 183 ] * state [ 31 ] ; xx [ 199 ] = 0.7064499061132518 ; xx [ 203 ] = xx
[ 199 ] * state [ 31 ] ; xx [ 204 ] = - ( 0.05804874412964624 + xx [ 167 ] )
; xx [ 205 ] = - ( 6.831424848351424e-4 + xx [ 197 ] ) ; xx [ 206 ] =
0.04678282656864583 + xx [ 203 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 185
, xx + 204 , xx + 902 ) ; xx [ 185 ] = xx [ 572 ] - ( xx [ 525 ] * xx [ 730 ]
+ xx [ 657 ] * xx [ 731 ] ) ; xx [ 186 ] = xx [ 576 ] - ( xx [ 525 ] * xx [
732 ] + xx [ 657 ] * xx [ 733 ] ) ; xx [ 187 ] = xx [ 577 ] - ( xx [ 525 ] *
xx [ 734 ] + xx [ 657 ] * xx [ 735 ] ) ; xx [ 188 ] = xx [ 621 ] - ( xx [ 526
] * xx [ 730 ] + xx [ 658 ] * xx [ 731 ] ) ; xx [ 189 ] = xx [ 622 ] - ( xx [
526 ] * xx [ 732 ] + xx [ 658 ] * xx [ 733 ] ) ; xx [ 190 ] = xx [ 651 ] - (
xx [ 526 ] * xx [ 734 ] + xx [ 658 ] * xx [ 735 ] ) ; xx [ 191 ] = xx [ 652 ]
- ( xx [ 519 ] * xx [ 730 ] + xx [ 659 ] * xx [ 731 ] ) ; xx [ 192 ] = xx [
653 ] - ( xx [ 519 ] * xx [ 732 ] + xx [ 659 ] * xx [ 733 ] ) ; xx [ 193 ] =
xx [ 654 ] - ( xx [ 519 ] * xx [ 734 ] + xx [ 659 ] * xx [ 735 ] ) ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 185 , xx + 470 , xx + 911 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 470 , xx + 911 , xx + 185 ) ; xx [ 208 ]
= 0.7077023583577552 ; xx [ 651 ] = - 8.970481754443746e-5 ; xx [ 652 ] = -
2.490130590085562e-3 ; xx [ 653 ] = 0.01561535429133853 ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 651 , xx + 911 ) ; xx [ 209 ] =
0.07007512777103127 + xx [ 208 ] * state [ 14 ] + xx [ 911 ] ; xx [ 210 ] =
9.268340058968039e-3 ; xx [ 452 ] = 6.588221211357465e-3 - xx [ 210 ] * state
[ 14 ] - xx [ 912 ] ; xx [ 521 ] = 0.7064499061132523 ; xx [ 572 ] =
0.05970036331915229 + xx [ 521 ] * state [ 14 ] - xx [ 913 ] ; xx [ 651 ] = -
xx [ 209 ] ; xx [ 652 ] = xx [ 452 ] ; xx [ 653 ] = xx [ 572 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 712 , xx + 651 , xx + 911 ) ; xx [ 576
] = xx [ 41 ] * xx [ 45 ] ; xx [ 577 ] = xx [ 576 ] * xx [ 33 ] ; xx [ 612 ]
= xx [ 75 ] * xx [ 81 ] ; xx [ 613 ] = xx [ 612 ] * xx [ 29 ] ; xx [ 616 ] =
xx [ 76 ] * xx [ 115 ] ; xx [ 618 ] = xx [ 616 ] * xx [ 92 ] ; xx [ 621 ] =
xx [ 109 ] * xx [ 149 ] ; xx [ 622 ] = xx [ 621 ] * xx [ 126 ] ; xx [ 654 ] =
xx [ 41 ] * xx [ 38 ] ; xx [ 41 ] = xx [ 654 ] * xx [ 33 ] ; xx [ 668 ] = xx
[ 75 ] * xx [ 65 ] ; xx [ 75 ] = xx [ 668 ] * xx [ 29 ] ; xx [ 669 ] = xx [
76 ] * xx [ 99 ] ; xx [ 76 ] = xx [ 669 ] * xx [ 92 ] ; xx [ 671 ] = xx [ 109
] * xx [ 133 ] ; xx [ 109 ] = xx [ 671 ] * xx [ 126 ] ; xx [ 712 ] = xx [ 43
] * xx [ 69 ] ; xx [ 713 ] = xx [ 79 ] * xx [ 77 ] ; xx [ 714 ] = xx [ 113 ]
* xx [ 107 ] ; xx [ 715 ] = xx [ 147 ] * xx [ 110 ] ; xx [ 716 ] = xx [ 43 ]
* xx [ 576 ] ; xx [ 717 ] = xx [ 79 ] * xx [ 612 ] ; xx [ 718 ] = xx [ 113 ]
* xx [ 616 ] ; xx [ 719 ] = xx [ 147 ] * xx [ 621 ] ; xx [ 720 ] = xx [ 43 ]
* xx [ 654 ] ; xx [ 920 ] = xx [ 79 ] * xx [ 668 ] ; xx [ 921 ] = xx [ 113 ]
* xx [ 669 ] ; xx [ 922 ] = xx [ 147 ] * xx [ 671 ] ; xx [ 923 ] = xx [ 46 ]
* xx [ 69 ] ; xx [ 69 ] = xx [ 82 ] * xx [ 77 ] ; xx [ 77 ] = xx [ 116 ] * xx
[ 107 ] ; xx [ 107 ] = xx [ 150 ] * xx [ 110 ] ; xx [ 110 ] = xx [ 46 ] * xx
[ 576 ] ; xx [ 576 ] = xx [ 82 ] * xx [ 612 ] ; xx [ 612 ] = xx [ 116 ] * xx
[ 616 ] ; xx [ 616 ] = xx [ 150 ] * xx [ 621 ] ; xx [ 621 ] = xx [ 46 ] * xx
[ 654 ] ; xx [ 654 ] = xx [ 82 ] * xx [ 668 ] ; xx [ 668 ] = xx [ 116 ] * xx
[ 669 ] ; xx [ 669 ] = xx [ 150 ] * xx [ 671 ] ; xx [ 924 ] = xx [ 827 ] - xx
[ 848 ] - ( xx [ 857 ] - xx [ 70 ] + xx [ 866 ] - xx [ 102 ] + xx [ 875 ] -
xx [ 108 ] + xx [ 884 ] - xx [ 139 ] + xx [ 169 ] + xx [ 893 ] + xx [ 902 ] )
+ xx [ 185 ] - xx [ 911 ] ; xx [ 925 ] = xx [ 828 ] - xx [ 851 ] - ( xx [ 577
] + xx [ 860 ] + xx [ 869 ] - xx [ 613 ] + xx [ 878 ] - xx [ 618 ] + xx [ 887
] - xx [ 622 ] + xx [ 172 ] + xx [ 896 ] + xx [ 905 ] ) + xx [ 186 ] - xx [
914 ] ; xx [ 926 ] = xx [ 829 ] - xx [ 854 ] - ( xx [ 863 ] - xx [ 41 ] + xx
[ 872 ] - xx [ 75 ] + xx [ 881 ] - xx [ 76 ] + xx [ 890 ] - xx [ 109 ] + xx [
175 ] + xx [ 899 ] + xx [ 908 ] ) + xx [ 187 ] - xx [ 917 ] ; xx [ 927 ] = xx
[ 830 ] - xx [ 849 ] - ( xx [ 858 ] - xx [ 712 ] + xx [ 867 ] - xx [ 713 ] +
xx [ 876 ] - xx [ 714 ] + xx [ 885 ] - xx [ 715 ] + xx [ 170 ] + xx [ 894 ] +
xx [ 903 ] ) + xx [ 188 ] - xx [ 912 ] ; xx [ 928 ] = xx [ 831 ] - xx [ 852 ]
- ( xx [ 716 ] + xx [ 861 ] + xx [ 870 ] - xx [ 717 ] + xx [ 879 ] - xx [ 718
] + xx [ 888 ] - xx [ 719 ] + xx [ 173 ] + xx [ 897 ] + xx [ 906 ] ) + xx [
189 ] - xx [ 915 ] ; xx [ 929 ] = xx [ 832 ] - xx [ 855 ] - ( xx [ 864 ] - xx
[ 720 ] + xx [ 873 ] - xx [ 920 ] + xx [ 882 ] - xx [ 921 ] + xx [ 891 ] - xx
[ 922 ] + xx [ 176 ] + xx [ 900 ] + xx [ 909 ] ) + xx [ 190 ] - xx [ 918 ] ;
xx [ 930 ] = xx [ 833 ] - xx [ 850 ] - ( xx [ 923 ] + xx [ 859 ] + xx [ 868 ]
- xx [ 69 ] + xx [ 877 ] - xx [ 77 ] + xx [ 886 ] - xx [ 107 ] + xx [ 171 ] +
xx [ 895 ] + xx [ 904 ] ) + xx [ 191 ] - xx [ 913 ] ; xx [ 931 ] = xx [ 834 ]
- xx [ 853 ] - ( xx [ 862 ] - xx [ 110 ] + xx [ 871 ] - xx [ 576 ] + xx [ 880
] - xx [ 612 ] + xx [ 889 ] - xx [ 616 ] + xx [ 174 ] + xx [ 898 ] + xx [ 907
] ) + xx [ 192 ] - xx [ 916 ] ; xx [ 932 ] = xx [ 835 ] - xx [ 856 ] - ( xx [
621 ] + xx [ 865 ] + xx [ 874 ] - xx [ 654 ] + xx [ 883 ] - xx [ 668 ] + xx [
892 ] - xx [ 669 ] + xx [ 177 ] + xx [ 901 ] + xx [ 910 ] ) + xx [ 193 ] - xx
[ 919 ] ; xx [ 671 ] = 0.7044166587907259 ; xx [ 933 ] = 0.0861852675737142 ;
xx [ 934 ] = 0.704534790107173 ; xx [ 935 ] = - xx [ 671 ] ; xx [ 936 ] = xx
[ 933 ] ; xx [ 937 ] = - xx [ 934 ] ; pm_math_Matrix3x3_transposeXform_ra (
xx + 924 , xx + 935 , xx + 938 ) ; xx [ 941 ] = 1.631966647380579e-3 ; xx [
942 ] = 5.229812735950155e-3 ; xx [ 943 ] = 9.919335323258331e-4 ; xx [ 944 ]
= xx [ 941 ] ; xx [ 945 ] = xx [ 942 ] ; xx [ 946 ] = - xx [ 943 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 748 , xx + 944 , xx + 947 ) ; xx [ 950 ] =
xx [ 938 ] + xx [ 947 ] ; xx [ 951 ] = xx [ 939 ] + xx [ 948 ] ; xx [ 938 ] =
xx [ 940 ] + xx [ 949 ] ; xx [ 947 ] = xx [ 950 ] ; xx [ 948 ] = xx [ 951 ] ;
xx [ 949 ] = xx [ 938 ] ; xx [ 939 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx
+ 947 ) ; xx [ 940 ] = 0.7077023583577498 ; xx [ 952 ] = 9.268340058972591e-3
; xx [ 953 ] = 0.7064499061132579 ; xx [ 954 ] = - xx [ 940 ] ; xx [ 955 ] =
- xx [ 952 ] ; xx [ 956 ] = xx [ 953 ] ; pm_math_Matrix3x3_transposeXform_ra
( xx + 924 , xx + 954 , xx + 957 ) ; xx [ 960 ] = 9.453137687592937e-4 ; xx [
961 ] = 7.422309401410978e-3 ; xx [ 962 ] = 8.496119693553942e-4 ; xx [ 963 ]
= - xx [ 960 ] ; xx [ 964 ] = xx [ 961 ] ; xx [ 965 ] = - xx [ 962 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 748 , xx + 963 , xx + 966 ) ; xx [ 969 ] =
xx [ 957 ] + xx [ 966 ] ; xx [ 970 ] = xx [ 958 ] + xx [ 967 ] ; xx [ 957 ] =
xx [ 959 ] + xx [ 968 ] ; xx [ 966 ] = xx [ 969 ] ; xx [ 967 ] = xx [ 970 ] ;
xx [ 968 ] = xx [ 957 ] ; xx [ 958 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx
+ 966 ) ; xx [ 959 ] = 0.05435570616770646 ; xx [ 971 ] = 0.9962360149712512
; xx [ 972 ] = 0.06752229025448842 ; xx [ 973 ] = xx [ 959 ] ; xx [ 974 ] =
xx [ 971 ] ; xx [ 975 ] = xx [ 972 ] ; pm_math_Matrix3x3_transposeXform_ra (
xx + 924 , xx + 973 , xx + 976 ) ; xx [ 979 ] = 8.841458300329267e-3 ; xx [
980 ] = 3.833833717245413e-4 ; xx [ 981 ] = 1.460901083802781e-3 ; xx [ 982 ]
= xx [ 979 ] ; xx [ 983 ] = - xx [ 980 ] ; xx [ 984 ] = - xx [ 981 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 748 , xx + 982 , xx + 985 ) ; xx [ 988 ] =
xx [ 976 ] + xx [ 985 ] ; xx [ 989 ] = xx [ 977 ] + xx [ 986 ] ; xx [ 976 ] =
xx [ 978 ] + xx [ 987 ] ; xx [ 985 ] = xx [ 988 ] ; xx [ 986 ] = xx [ 989 ] ;
xx [ 987 ] = xx [ 976 ] ; xx [ 977 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx
+ 985 ) ; xx [ 978 ] = pm_math_Vector3_dot_ra ( xx + 672 , xx + 721 ) ; xx [
990 ] = pm_math_Vector3_dot_ra ( xx + 672 , xx + 947 ) ; xx [ 991 ] =
pm_math_Vector3_dot_ra ( xx + 672 , xx + 966 ) ; xx [ 992 ] =
pm_math_Vector3_dot_ra ( xx + 672 , xx + 985 ) ; xx [ 993 ] =
pm_math_Vector3_dot_ra ( xx + 703 , xx + 947 ) ; xx [ 994 ] =
pm_math_Vector3_dot_ra ( xx + 703 , xx + 966 ) ; xx [ 995 ] =
pm_math_Vector3_dot_ra ( xx + 703 , xx + 985 ) ; xx [ 996 ] = xx [ 44 ] / xx
[ 57 ] ; xx [ 997 ] = xx [ 44 ] - xx [ 44 ] * xx [ 996 ] ; xx [ 998 ] =
4.700446016264957e-8 ; xx [ 999 ] = 4.700446020666758e-8 ; xx [ 1000 ] = xx [
997 ] * xx [ 33 ] ; xx [ 1001 ] = xx [ 43 ] * xx [ 997 ] ; xx [ 1002 ] = - (
xx [ 46 ] * xx [ 997 ] ) ; xx [ 1003 ] = xx [ 998 ] * xx [ 34 ] ; xx [ 1004 ]
= xx [ 998 ] * xx [ 32 ] ; xx [ 1005 ] = xx [ 998 ] * xx [ 37 ] ; xx [ 1006 ]
= xx [ 999 ] * xx [ 42 ] ; xx [ 1007 ] = - ( xx [ 999 ] * xx [ 45 ] ) ; xx [
1008 ] = xx [ 999 ] * xx [ 38 ] ; pm_math_Matrix3x3_compose_ra ( xx + 47 , xx
+ 1000 , xx + 1009 ) ; xx [ 45 ] = xx [ 70 ] ; xx [ 46 ] = - xx [ 577 ] ; xx
[ 47 ] = xx [ 41 ] ; xx [ 48 ] = xx [ 712 ] ; xx [ 49 ] = - xx [ 716 ] ; xx [
50 ] = xx [ 720 ] ; xx [ 51 ] = - xx [ 923 ] ; xx [ 52 ] = xx [ 110 ] ; xx [
53 ] = - xx [ 621 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 45 , xx + 445 ,
xx + 1000 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 857 , xx + 445 , xx + 45 )
; xx [ 32 ] = xx [ 44 ] / xx [ 67 ] ; xx [ 33 ] = xx [ 44 ] - xx [ 44 ] * xx
[ 32 ] ; xx [ 857 ] = xx [ 33 ] * xx [ 29 ] ; xx [ 858 ] = xx [ 79 ] * xx [
33 ] ; xx [ 859 ] = xx [ 82 ] * xx [ 33 ] ; xx [ 860 ] = xx [ 998 ] * xx [ 61
] ; xx [ 861 ] = xx [ 998 ] * xx [ 60 ] ; xx [ 862 ] = xx [ 998 ] * xx [ 62 ]
; xx [ 863 ] = xx [ 999 ] * xx [ 78 ] ; xx [ 864 ] = xx [ 999 ] * xx [ 81 ] ;
xx [ 865 ] = xx [ 999 ] * xx [ 65 ] ; pm_math_Matrix3x3_compose_ra ( xx + 83
, xx + 857 , xx + 1018 ) ; xx [ 81 ] = xx [ 102 ] ; xx [ 82 ] = xx [ 613 ] ;
xx [ 83 ] = xx [ 75 ] ; xx [ 84 ] = xx [ 713 ] ; xx [ 85 ] = xx [ 717 ] ; xx
[ 86 ] = xx [ 920 ] ; xx [ 87 ] = xx [ 69 ] ; xx [ 88 ] = xx [ 576 ] ; xx [
89 ] = xx [ 654 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 81 , xx + 71 , xx +
857 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 866 , xx + 71 , xx + 81 ) ; xx [
29 ] = xx [ 44 ] / xx [ 101 ] ; xx [ 33 ] = xx [ 44 ] - xx [ 44 ] * xx [ 29 ]
; xx [ 866 ] = xx [ 33 ] * xx [ 92 ] ; xx [ 867 ] = xx [ 113 ] * xx [ 33 ] ;
xx [ 868 ] = xx [ 116 ] * xx [ 33 ] ; xx [ 869 ] = xx [ 998 ] * xx [ 95 ] ;
xx [ 870 ] = xx [ 998 ] * xx [ 94 ] ; xx [ 871 ] = xx [ 998 ] * xx [ 96 ] ;
xx [ 872 ] = xx [ 999 ] * xx [ 112 ] ; xx [ 873 ] = xx [ 999 ] * xx [ 115 ] ;
xx [ 874 ] = xx [ 999 ] * xx [ 99 ] ; pm_math_Matrix3x3_compose_ra ( xx + 117
, xx + 866 , xx + 1027 ) ; xx [ 115 ] = xx [ 108 ] ; xx [ 116 ] = xx [ 618 ]
; xx [ 117 ] = xx [ 76 ] ; xx [ 118 ] = xx [ 714 ] ; xx [ 119 ] = xx [ 718 ]
; xx [ 120 ] = xx [ 921 ] ; xx [ 121 ] = xx [ 77 ] ; xx [ 122 ] = xx [ 612 ]
; xx [ 123 ] = xx [ 668 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 115 , xx +
103 , xx + 866 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 875 , xx + 103 , xx +
115 ) ; xx [ 33 ] = xx [ 44 ] / xx [ 56 ] ; xx [ 34 ] = xx [ 44 ] - xx [ 44 ]
* xx [ 33 ] ; xx [ 875 ] = xx [ 34 ] * xx [ 126 ] ; xx [ 876 ] = xx [ 147 ] *
xx [ 34 ] ; xx [ 877 ] = xx [ 150 ] * xx [ 34 ] ; xx [ 878 ] = xx [ 998 ] *
xx [ 129 ] ; xx [ 879 ] = xx [ 998 ] * xx [ 128 ] ; xx [ 880 ] = xx [ 998 ] *
xx [ 130 ] ; xx [ 881 ] = xx [ 999 ] * xx [ 146 ] ; xx [ 882 ] = xx [ 999 ] *
xx [ 149 ] ; xx [ 883 ] = xx [ 999 ] * xx [ 133 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 151 , xx + 875 , xx + 1036 ) ; xx [ 149 ]
= xx [ 139 ] ; xx [ 150 ] = xx [ 622 ] ; xx [ 151 ] = xx [ 109 ] ; xx [ 152 ]
= xx [ 715 ] ; xx [ 153 ] = xx [ 719 ] ; xx [ 154 ] = xx [ 922 ] ; xx [ 155 ]
= xx [ 107 ] ; xx [ 156 ] = xx [ 616 ] ; xx [ 157 ] = xx [ 669 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 149 , xx + 136 , xx + 712 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 884 , xx + 136 , xx + 149 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 169 , xx + 140 , xx + 875 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 893 , xx + 449 , xx + 169 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 902 , xx + 204 , xx + 884 ) ; xx [ 34 ]
= 5.727332635362021e-6 ; xx [ 893 ] = xx [ 709 ] * xx [ 251 ] ; xx [ 894 ] =
xx [ 709 ] * xx [ 260 ] ; xx [ 895 ] = xx [ 709 ] * xx [ 267 ] ; xx [ 896 ] =
xx [ 710 ] * xx [ 254 ] ; xx [ 897 ] = xx [ 710 ] * xx [ 252 ] ; xx [ 898 ] =
xx [ 710 ] * xx [ 255 ] ; xx [ 899 ] = xx [ 711 ] * xx [ 259 ] ; xx [ 900 ] =
xx [ 711 ] * xx [ 266 ] ; xx [ 901 ] = xx [ 711 ] * xx [ 253 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 268 , xx + 893 , xx + 902 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 775 , xx + 325 , xx + 268 ) ; xx [ 37 ]
= xx [ 318 ] * xx [ 337 ] ; xx [ 38 ] = xx [ 319 ] * xx [ 337 ] ; xx [ 41 ] =
xx [ 319 ] * xx [ 339 ] ; xx [ 775 ] = xx [ 639 ] - xx [ 316 ] * xx [ 337 ] ;
xx [ 776 ] = xx [ 37 ] ; xx [ 777 ] = xx [ 38 ] ; xx [ 778 ] = xx [ 37 ] ; xx
[ 779 ] = xx [ 642 ] + xx [ 318 ] * xx [ 339 ] ; xx [ 780 ] = xx [ 41 ] ; xx
[ 781 ] = xx [ 38 ] ; xx [ 782 ] = xx [ 41 ] ; xx [ 783 ] = xx [ 644 ] + xx [
319 ] * xx [ 341 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 775 , xx +
303 , xx + 893 ) ; pm_math_Matrix3x3_compose_ra ( xx + 303 , xx + 893 , xx +
775 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 757 , xx + 736 , xx + 893 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 766 , xx + 736 , xx + 757 ) ; xx [ 37 ]
= 6.304651877425679e-6 ; xx [ 38 ] = 4.963260250277482e-6 ; xx [ 766 ] = xx [
34 ] + xx [ 902 ] - xx [ 268 ] + xx [ 775 ] - xx [ 893 ] - xx [ 893 ] - xx [
757 ] ; xx [ 767 ] = xx [ 903 ] - xx [ 269 ] + xx [ 776 ] - xx [ 894 ] - xx [
896 ] - xx [ 758 ] ; xx [ 768 ] = xx [ 904 ] - xx [ 270 ] + xx [ 777 ] - xx [
895 ] - xx [ 899 ] - xx [ 759 ] ; xx [ 769 ] = xx [ 905 ] - xx [ 271 ] + xx [
778 ] - xx [ 896 ] - xx [ 894 ] - xx [ 760 ] ; xx [ 770 ] = xx [ 37 ] + xx [
906 ] - xx [ 272 ] + xx [ 779 ] - xx [ 897 ] - xx [ 897 ] - xx [ 761 ] ; xx [
771 ] = xx [ 907 ] - xx [ 273 ] + xx [ 780 ] - xx [ 898 ] - xx [ 900 ] - xx [
762 ] ; xx [ 772 ] = xx [ 908 ] - xx [ 274 ] + xx [ 781 ] - xx [ 899 ] - xx [
895 ] - xx [ 763 ] ; xx [ 773 ] = xx [ 909 ] - xx [ 275 ] + xx [ 782 ] - xx [
900 ] - xx [ 898 ] - xx [ 764 ] ; xx [ 774 ] = xx [ 38 ] + xx [ 910 ] - xx [
276 ] + xx [ 783 ] - xx [ 901 ] - xx [ 901 ] - xx [ 765 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 766 , xx + 242 , xx + 268 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 242 , xx + 268 , xx + 757 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 793 , xx + 784 , xx + 268 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 802 , xx + 784 , xx + 775 ) ; xx [ 41 ]
= - ( xx [ 379 ] * xx [ 415 ] ) ; xx [ 42 ] = - ( xx [ 397 ] * xx [ 415 ] ) ;
xx [ 43 ] = - ( xx [ 397 ] * xx [ 417 ] ) ; xx [ 793 ] = xx [ 707 ] - xx [
376 ] * xx [ 415 ] ; xx [ 794 ] = xx [ 41 ] ; xx [ 795 ] = xx [ 42 ] ; xx [
796 ] = xx [ 41 ] ; xx [ 797 ] = xx [ 633 ] - xx [ 379 ] * xx [ 417 ] ; xx [
798 ] = xx [ 43 ] ; xx [ 799 ] = xx [ 42 ] ; xx [ 800 ] = xx [ 43 ] ; xx [
801 ] = xx [ 643 ] - xx [ 397 ] * xx [ 419 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 793 , xx + 384 , xx + 802 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 384 , xx + 802 , xx + 793 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 367 , xx + 813 , xx + 384 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 816 , xx + 813 , xx + 367 ) ; xx [ 41 ]
= xx [ 405 ] / xx [ 432 ] ; xx [ 42 ] = 8.621548008553888e-7 ; xx [ 43 ] = xx
[ 409 ] * xx [ 41 ] ; xx [ 54 ] = xx [ 825 ] * xx [ 41 ] ; xx [ 55 ] = xx [
409 ] / xx [ 432 ] ; xx [ 60 ] = 7.923725535971254e-7 ; xx [ 61 ] = xx [ 825
] * xx [ 55 ] ; xx [ 62 ] = xx [ 825 ] / xx [ 432 ] ; xx [ 65 ] =
1.90802841351583e-7 ; xx [ 802 ] = xx [ 757 ] - xx [ 268 ] - xx [ 268 ] - xx
[ 775 ] + xx [ 793 ] - xx [ 384 ] - xx [ 384 ] - xx [ 367 ] - xx [ 405 ] * xx
[ 41 ] + xx [ 42 ] ; xx [ 803 ] = xx [ 758 ] - xx [ 269 ] - xx [ 271 ] - xx [
776 ] + xx [ 794 ] - xx [ 385 ] - xx [ 387 ] - xx [ 368 ] - xx [ 43 ] ; xx [
804 ] = xx [ 759 ] - xx [ 270 ] - xx [ 274 ] - xx [ 777 ] + xx [ 795 ] - xx [
386 ] - xx [ 390 ] - xx [ 369 ] - xx [ 54 ] ; xx [ 805 ] = xx [ 760 ] - xx [
271 ] - xx [ 269 ] - xx [ 778 ] + xx [ 796 ] - xx [ 387 ] - xx [ 385 ] - xx [
370 ] - xx [ 43 ] ; xx [ 806 ] = xx [ 761 ] - xx [ 272 ] - xx [ 272 ] - xx [
779 ] + xx [ 797 ] - xx [ 388 ] - xx [ 388 ] - xx [ 371 ] - xx [ 409 ] * xx [
55 ] + xx [ 60 ] ; xx [ 807 ] = xx [ 762 ] - xx [ 273 ] - xx [ 275 ] - xx [
780 ] + xx [ 798 ] - xx [ 389 ] - xx [ 391 ] - xx [ 372 ] - xx [ 61 ] ; xx [
808 ] = xx [ 763 ] - xx [ 274 ] - xx [ 270 ] - xx [ 781 ] + xx [ 799 ] - xx [
390 ] - xx [ 386 ] - xx [ 373 ] - xx [ 54 ] ; xx [ 809 ] = xx [ 764 ] - xx [
275 ] - xx [ 273 ] - xx [ 782 ] + xx [ 800 ] - xx [ 391 ] - xx [ 389 ] - xx [
374 ] - xx [ 61 ] ; xx [ 810 ] = xx [ 765 ] - xx [ 276 ] - xx [ 276 ] - xx [
783 ] + xx [ 801 ] - xx [ 392 ] - xx [ 392 ] - xx [ 375 ] - xx [ 825 ] * xx [
62 ] + xx [ 65 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 802 , xx +
213 , xx + 268 ) ; pm_math_Matrix3x3_compose_ra ( xx + 213 , xx + 268 , xx +
367 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 827 , xx + 845 , xx + 268 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 848 , xx + 845 , xx + 384 ) ; xx [ 43 ]
= xx [ 526 ] * xx [ 724 ] + xx [ 658 ] * xx [ 725 ] ; xx [ 54 ] = xx [ 519 ]
* xx [ 724 ] + xx [ 725 ] * xx [ 659 ] ; xx [ 61 ] = xx [ 519 ] * xx [ 726 ]
+ xx [ 659 ] * xx [ 727 ] ; xx [ 757 ] = xx [ 629 ] - ( xx [ 525 ] * xx [ 724
] + xx [ 657 ] * xx [ 725 ] ) + xx [ 707 ] ; xx [ 758 ] = xx [ 630 ] - xx [
43 ] ; xx [ 759 ] = xx [ 631 ] - xx [ 54 ] ; xx [ 760 ] = xx [ 632 ] - xx [
43 ] ; xx [ 761 ] = xx [ 634 ] - ( xx [ 526 ] * xx [ 726 ] + xx [ 658 ] * xx
[ 727 ] ) + xx [ 633 ] ; xx [ 762 ] = xx [ 635 ] - xx [ 61 ] ; xx [ 763 ] =
xx [ 640 ] - xx [ 54 ] ; xx [ 764 ] = xx [ 641 ] - xx [ 61 ] ; xx [ 765 ] =
xx [ 518 ] - ( xx [ 519 ] * xx [ 728 ] + xx [ 659 ] * xx [ 729 ] ) + xx [ 643
] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 757 , xx + 470 , xx + 775 )
; pm_math_Matrix3x3_compose_ra ( xx + 470 , xx + 775 , xx + 757 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 185 , xx + 651 , xx + 775 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 911 , xx + 651 , xx + 185 ) ; xx [ 43 ]
= 3.547073187284647e-6 ; xx [ 54 ] = 3.608837842093127e-6 ; xx [ 793 ] = xx [
1009 ] - xx [ 1000 ] - xx [ 1000 ] - xx [ 45 ] + xx [ 1018 ] - xx [ 857 ] -
xx [ 857 ] - xx [ 81 ] + xx [ 1027 ] - xx [ 866 ] - xx [ 866 ] - xx [ 115 ] +
xx [ 1036 ] - xx [ 712 ] - xx [ 712 ] - xx [ 149 ] - xx [ 875 ] - xx [ 169 ]
- xx [ 884 ] + xx [ 367 ] - xx [ 268 ] - xx [ 268 ] - xx [ 384 ] +
4.730609027013792e-4 + xx [ 757 ] - xx [ 775 ] - xx [ 775 ] - xx [ 185 ] ; xx
[ 794 ] = xx [ 1010 ] - xx [ 1001 ] - xx [ 1003 ] - xx [ 46 ] + xx [ 1019 ] -
xx [ 858 ] - xx [ 860 ] - xx [ 82 ] + xx [ 1028 ] - xx [ 867 ] - xx [ 869 ] -
xx [ 116 ] + xx [ 1037 ] - xx [ 713 ] - xx [ 715 ] - xx [ 150 ] - xx [ 876 ]
- xx [ 170 ] - xx [ 885 ] + xx [ 368 ] - xx [ 269 ] - xx [ 271 ] - xx [ 385 ]
+ xx [ 43 ] + xx [ 758 ] - xx [ 776 ] - xx [ 778 ] - xx [ 186 ] ; xx [ 795 ]
= xx [ 1011 ] - xx [ 1002 ] - xx [ 1006 ] - xx [ 47 ] + xx [ 1020 ] - xx [
859 ] - xx [ 863 ] - xx [ 83 ] + xx [ 1029 ] - xx [ 868 ] - xx [ 872 ] - xx [
117 ] + xx [ 1038 ] - xx [ 714 ] - xx [ 718 ] - xx [ 151 ] - xx [ 877 ] - xx
[ 171 ] - xx [ 886 ] + xx [ 369 ] - xx [ 270 ] - xx [ 274 ] - xx [ 386 ] -
3.174747343713644e-5 + xx [ 759 ] - xx [ 777 ] - xx [ 781 ] - xx [ 187 ] ; xx
[ 796 ] = xx [ 1012 ] - xx [ 1003 ] - xx [ 1001 ] - xx [ 48 ] + xx [ 1021 ] -
xx [ 860 ] - xx [ 858 ] - xx [ 84 ] + xx [ 1030 ] - xx [ 869 ] - xx [ 867 ] -
xx [ 118 ] + xx [ 1039 ] - xx [ 715 ] - xx [ 713 ] - xx [ 152 ] - xx [ 878 ]
- xx [ 172 ] - xx [ 887 ] + xx [ 370 ] - xx [ 271 ] - xx [ 269 ] - xx [ 387 ]
+ xx [ 43 ] + xx [ 760 ] - xx [ 778 ] - xx [ 776 ] - xx [ 188 ] ; xx [ 797 ]
= xx [ 1013 ] - xx [ 1004 ] - xx [ 1004 ] - xx [ 49 ] + xx [ 1022 ] - xx [
861 ] - xx [ 861 ] - xx [ 85 ] + xx [ 1031 ] - xx [ 870 ] - xx [ 870 ] - xx [
119 ] + xx [ 1040 ] - xx [ 716 ] - xx [ 716 ] - xx [ 153 ] - xx [ 879 ] - xx
[ 173 ] - xx [ 888 ] + xx [ 371 ] - xx [ 272 ] - xx [ 272 ] - xx [ 388 ] +
8.576972592810991e-4 + xx [ 761 ] - xx [ 779 ] - xx [ 779 ] - xx [ 189 ] ; xx
[ 798 ] = xx [ 1014 ] - xx [ 1005 ] - xx [ 1007 ] - xx [ 50 ] + xx [ 1023 ] -
xx [ 862 ] - xx [ 864 ] - xx [ 86 ] + xx [ 1032 ] - xx [ 871 ] - xx [ 873 ] -
xx [ 120 ] + xx [ 1041 ] - xx [ 717 ] - xx [ 719 ] - xx [ 154 ] - xx [ 880 ]
- xx [ 174 ] - xx [ 889 ] + xx [ 372 ] - xx [ 273 ] - xx [ 275 ] - xx [ 389 ]
+ xx [ 54 ] + xx [ 762 ] - xx [ 780 ] - xx [ 782 ] - xx [ 190 ] ; xx [ 799 ]
= xx [ 1015 ] - xx [ 1006 ] - xx [ 1002 ] - xx [ 51 ] + xx [ 1024 ] - xx [
863 ] - xx [ 859 ] - xx [ 87 ] + xx [ 1033 ] - xx [ 872 ] - xx [ 868 ] - xx [
121 ] + xx [ 1042 ] - xx [ 718 ] - xx [ 714 ] - xx [ 155 ] - xx [ 881 ] - xx
[ 175 ] - xx [ 890 ] + xx [ 373 ] - xx [ 274 ] - xx [ 270 ] - xx [ 390 ] -
3.174747343713645e-5 + xx [ 763 ] - xx [ 781 ] - xx [ 777 ] - xx [ 191 ] ; xx
[ 800 ] = xx [ 1016 ] - xx [ 1007 ] - xx [ 1005 ] - xx [ 52 ] + xx [ 1025 ] -
xx [ 864 ] - xx [ 862 ] - xx [ 88 ] + xx [ 1034 ] - xx [ 873 ] - xx [ 871 ] -
xx [ 122 ] + xx [ 1043 ] - xx [ 719 ] - xx [ 717 ] - xx [ 156 ] - xx [ 882 ]
- xx [ 176 ] - xx [ 891 ] + xx [ 374 ] - xx [ 275 ] - xx [ 273 ] - xx [ 391 ]
+ xx [ 54 ] + xx [ 764 ] - xx [ 782 ] - xx [ 780 ] - xx [ 192 ] ; xx [ 801 ]
= xx [ 1017 ] - xx [ 1008 ] - xx [ 1008 ] - xx [ 53 ] + xx [ 1026 ] - xx [
865 ] - xx [ 865 ] - xx [ 89 ] + xx [ 1035 ] - xx [ 874 ] - xx [ 874 ] - xx [
123 ] + xx [ 1044 ] - xx [ 720 ] - xx [ 720 ] - xx [ 157 ] - xx [ 883 ] - xx
[ 177 ] - xx [ 892 ] + xx [ 375 ] - xx [ 276 ] - xx [ 276 ] - xx [ 392 ] +
4.875290169197813e-4 + xx [ 765 ] - xx [ 783 ] - xx [ 783 ] - xx [ 193 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 793 , xx + 935 , xx + 45 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 944 , xx + 48 ) ; xx [ 43 ] = xx
[ 45 ] + xx [ 48 ] ; xx [ 51 ] = xx [ 46 ] + xx [ 49 ] ; xx [ 45 ] = xx [ 47
] + xx [ 50 ] ; xx [ 46 ] = xx [ 43 ] ; xx [ 47 ] = xx [ 51 ] ; xx [ 48 ] =
xx [ 45 ] ; pm_math_Matrix3x3_xform_ra ( xx + 793 , xx + 954 , xx + 52 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 963 , xx + 75 ) ; xx [ 49 ] = xx
[ 52 ] + xx [ 75 ] ; xx [ 50 ] = xx [ 53 ] + xx [ 76 ] ; xx [ 52 ] = xx [ 54
] + xx [ 77 ] ; xx [ 75 ] = xx [ 49 ] ; xx [ 76 ] = xx [ 50 ] ; xx [ 77 ] =
xx [ 52 ] ; xx [ 53 ] = pm_math_Vector3_dot_ra ( xx + 935 , xx + 75 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 966 ) ; pm_math_Matrix3x3_xform_ra (
xx + 793 , xx + 973 , xx + 81 ) ; pm_math_Matrix3x3_xform_ra ( xx + 924 , xx
+ 982 , xx + 84 ) ; xx [ 54 ] = xx [ 81 ] + xx [ 84 ] ; xx [ 61 ] = xx [ 82 ]
+ xx [ 85 ] ; xx [ 69 ] = xx [ 83 ] + xx [ 86 ] ; xx [ 81 ] = xx [ 54 ] ; xx
[ 82 ] = xx [ 61 ] ; xx [ 83 ] = xx [ 69 ] ; xx [ 70 ] =
pm_math_Vector3_dot_ra ( xx + 935 , xx + 81 ) + pm_math_Vector3_dot_ra ( xx +
944 , xx + 985 ) ; xx [ 78 ] = pm_math_Vector3_dot_ra ( xx + 954 , xx + 81 )
+ pm_math_Vector3_dot_ra ( xx + 963 , xx + 985 ) ; xx [ 848 ] =
pm_math_Vector3_dot_ra ( xx + 26 , xx + 645 ) ; xx [ 849 ] = xx [ 520 ] ; xx
[ 850 ] = xx [ 0 ] ; xx [ 851 ] = xx [ 939 ] ; xx [ 852 ] = xx [ 958 ] ; xx [
853 ] = xx [ 977 ] ; xx [ 854 ] = xx [ 520 ] ; xx [ 855 ] =
pm_math_Vector3_dot_ra ( xx + 672 , xx + 687 ) ; xx [ 856 ] = xx [ 978 ] ; xx
[ 857 ] = xx [ 990 ] ; xx [ 858 ] = xx [ 991 ] ; xx [ 859 ] = xx [ 992 ] ; xx
[ 860 ] = xx [ 0 ] ; xx [ 861 ] = xx [ 978 ] ; xx [ 862 ] =
pm_math_Vector3_dot_ra ( xx + 703 , xx + 721 ) ; xx [ 863 ] = xx [ 993 ] ; xx
[ 864 ] = xx [ 994 ] ; xx [ 865 ] = xx [ 995 ] ; xx [ 866 ] = xx [ 939 ] ; xx
[ 867 ] = xx [ 990 ] ; xx [ 868 ] = xx [ 993 ] ; xx [ 869 ] =
pm_math_Vector3_dot_ra ( xx + 935 , xx + 46 ) + pm_math_Vector3_dot_ra ( xx +
944 , xx + 947 ) ; xx [ 870 ] = xx [ 53 ] ; xx [ 871 ] = xx [ 70 ] ; xx [ 872
] = xx [ 958 ] ; xx [ 873 ] = xx [ 991 ] ; xx [ 874 ] = xx [ 994 ] ; xx [ 875
] = xx [ 53 ] ; xx [ 876 ] = pm_math_Vector3_dot_ra ( xx + 954 , xx + 75 ) +
pm_math_Vector3_dot_ra ( xx + 963 , xx + 966 ) ; xx [ 877 ] = xx [ 78 ] ; xx
[ 878 ] = xx [ 977 ] ; xx [ 879 ] = xx [ 992 ] ; xx [ 880 ] = xx [ 995 ] ; xx
[ 881 ] = xx [ 70 ] ; xx [ 882 ] = xx [ 78 ] ; xx [ 883 ] =
pm_math_Vector3_dot_ra ( xx + 973 , xx + 81 ) + pm_math_Vector3_dot_ra ( xx +
982 , xx + 985 ) ; ii [ 0 ] = factorSymmetricPosDef ( xx + 848 , 6 , xx + 81
) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'Control_Bicopter/Subsystem/SixDOF' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 75 ] = xx [ 31 ] ; xx [ 76 ] = xx [ 36 ] ; xx [ 77 ] =
xx [ 39 ] ; xx [ 78 ] = xx [ 444 ] ; pm_math_Quaternion_xform_ra ( xx + 12 ,
xx + 4 , xx + 46 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 445 , xx + 4 )
; pm_math_Vector3_cross_ra ( xx + 46 , xx + 4 , xx + 12 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 75 , xx + 12 , xx + 4 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 75 , xx + 46 , xx + 12 ) ; xx [ 0 ]
= xx [ 448 ] * state [ 44 ] ; xx [ 15 ] = xx [ 12 ] + state [ 44 ] ; xx [ 31
] = xx [ 35 ] * xx [ 6 ] ; xx [ 81 ] = xx [ 15 ] ; xx [ 82 ] = xx [ 13 ] ; xx
[ 83 ] = xx [ 14 ] ; xx [ 84 ] = xx [ 15 ] * xx [ 44 ] ; xx [ 85 ] = xx [ 998
] * xx [ 13 ] ; xx [ 86 ] = xx [ 999 ] * xx [ 14 ] ; pm_math_Vector3_cross_ra
( xx + 81 , xx + 84 , xx + 87 ) ; xx [ 6 ] = ( xx [ 87 ] - xx [ 448 ] * xx [
31 ] ) / xx [ 57 ] ; xx [ 81 ] = ( xx [ 4 ] - ( xx [ 13 ] + xx [ 13 ] ) * xx
[ 0 ] ) * xx [ 35 ] ; xx [ 82 ] = xx [ 35 ] * ( xx [ 5 ] + ( xx [ 12 ] + xx [
15 ] ) * xx [ 0 ] ) ; xx [ 83 ] = xx [ 31 ] + xx [ 40 ] * xx [ 6 ] ;
pm_math_Quaternion_xform_ra ( xx + 75 , xx + 81 , xx + 84 ) ; xx [ 107 ] = xx
[ 59 ] ; xx [ 108 ] = xx [ 64 ] ; xx [ 109 ] = xx [ 66 ] ; xx [ 110 ] = xx [
68 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 107 , xx + 46 , xx + 81 ) ;
xx [ 0 ] = xx [ 74 ] * state [ 42 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx
+ 71 , xx + 90 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 90 , xx + 94 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 107 , xx + 94 , xx + 90 ) ; xx [ 4
] = xx [ 81 ] - state [ 42 ] ; xx [ 5 ] = xx [ 35 ] * xx [ 92 ] ; xx [ 94 ] =
xx [ 4 ] ; xx [ 95 ] = xx [ 82 ] ; xx [ 96 ] = xx [ 83 ] ; xx [ 115 ] = xx [
44 ] * xx [ 4 ] ; xx [ 116 ] = xx [ 998 ] * xx [ 82 ] ; xx [ 117 ] = xx [ 999
] * xx [ 83 ] ; pm_math_Vector3_cross_ra ( xx + 94 , xx + 115 , xx + 118 ) ;
xx [ 12 ] = ( xx [ 74 ] * xx [ 5 ] - xx [ 118 ] ) / xx [ 67 ] ; xx [ 66 ] = (
( xx [ 82 ] + xx [ 82 ] ) * xx [ 0 ] + xx [ 90 ] ) * xx [ 35 ] ; xx [ 67 ] =
xx [ 35 ] * ( xx [ 91 ] - ( xx [ 81 ] + xx [ 4 ] ) * xx [ 0 ] ) ; xx [ 68 ] =
xx [ 5 ] - xx [ 63 ] * xx [ 12 ] ; pm_math_Quaternion_xform_ra ( xx + 107 ,
xx + 66 , xx + 90 ) ; xx [ 121 ] = xx [ 93 ] ; xx [ 122 ] = xx [ 98 ] ; xx [
123 ] = xx [ 100 ] ; xx [ 124 ] = xx [ 111 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 46 , xx + 66 ) ; xx [ 0
] = xx [ 106 ] * state [ 40 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 103
, xx + 93 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 93 , xx + 98 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 98 , xx + 93 ) ; xx [ 4
] = xx [ 66 ] - state [ 40 ] ; xx [ 5 ] = xx [ 35 ] * xx [ 95 ] ; xx [ 98 ] =
xx [ 4 ] ; xx [ 99 ] = xx [ 67 ] ; xx [ 100 ] = xx [ 68 ] ; xx [ 111 ] = xx [
44 ] * xx [ 4 ] ; xx [ 112 ] = xx [ 998 ] * xx [ 67 ] ; xx [ 113 ] = xx [ 999
] * xx [ 68 ] ; pm_math_Vector3_cross_ra ( xx + 98 , xx + 111 , xx + 115 ) ;
xx [ 15 ] = ( xx [ 106 ] * xx [ 5 ] - xx [ 115 ] ) / xx [ 101 ] ; xx [ 98 ] =
( ( xx [ 67 ] + xx [ 67 ] ) * xx [ 0 ] + xx [ 93 ] ) * xx [ 35 ] ; xx [ 99 ]
= xx [ 35 ] * ( xx [ 94 ] - ( xx [ 66 ] + xx [ 4 ] ) * xx [ 0 ] ) ; xx [ 100
] = xx [ 5 ] - xx [ 97 ] * xx [ 15 ] ; pm_math_Quaternion_xform_ra ( xx + 121
, xx + 98 , xx + 93 ) ; xx [ 98 ] = xx [ 127 ] ; xx [ 99 ] = xx [ 132 ] ; xx
[ 100 ] = xx [ 134 ] ; xx [ 101 ] = xx [ 145 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx + 46 , xx + 111 ) ; xx [ 0
] = xx [ 106 ] * state [ 38 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 136
, xx + 125 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 125 , xx + 128 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx + 128 , xx + 125 ) ; xx [ 4
] = xx [ 111 ] - state [ 38 ] ; xx [ 5 ] = xx [ 35 ] * xx [ 127 ] ; xx [ 127
] = xx [ 4 ] ; xx [ 128 ] = xx [ 112 ] ; xx [ 129 ] = xx [ 113 ] ; xx [ 132 ]
= xx [ 44 ] * xx [ 4 ] ; xx [ 133 ] = xx [ 998 ] * xx [ 112 ] ; xx [ 134 ] =
xx [ 999 ] * xx [ 113 ] ; pm_math_Vector3_cross_ra ( xx + 127 , xx + 132 , xx
+ 145 ) ; xx [ 31 ] = ( xx [ 106 ] * xx [ 5 ] - xx [ 145 ] ) / xx [ 56 ] ; xx
[ 127 ] = ( ( xx [ 112 ] + xx [ 112 ] ) * xx [ 0 ] + xx [ 125 ] ) * xx [ 35 ]
; xx [ 128 ] = xx [ 35 ] * ( xx [ 126 ] - ( xx [ 111 ] + xx [ 4 ] ) * xx [ 0
] ) ; xx [ 129 ] = xx [ 5 ] - xx [ 97 ] * xx [ 31 ] ;
pm_math_Quaternion_xform_ra ( xx + 98 , xx + 127 , xx + 132 ) ; xx [ 0 ] = xx
[ 84 ] + xx [ 90 ] + xx [ 93 ] + xx [ 132 ] ; xx [ 4 ] = 0.2934341153784394 ;
xx [ 5 ] = 0.2476921755716519 ; xx [ 35 ] = - 0.6382158331684398 ; xx [ 36 ]
= - 0.6672522434475355 ; xx [ 125 ] = - xx [ 4 ] ; xx [ 126 ] = xx [ 5 ] ; xx
[ 127 ] = xx [ 35 ] ; xx [ 128 ] = xx [ 36 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 46 , xx + 149 ) ; xx [
39 ] = 1.805437481450789e-4 ; xx [ 40 ] = 0.9999924437328848 ; xx [ 53 ] =
3.883282257123533e-3 ; xx [ 152 ] = xx [ 39 ] * state [ 36 ] ; xx [ 153 ] =
xx [ 40 ] * state [ 36 ] ; xx [ 154 ] = xx [ 53 ] * state [ 36 ] ;
pm_math_Vector3_cross_ra ( xx + 149 , xx + 152 , xx + 155 ) ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 140 , xx + 152 ) ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 152 , xx + 169 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 169 , xx + 152 ) ; xx [
56 ] = ( xx [ 2 ] * xx [ 155 ] + xx [ 152 ] ) * xx [ 135 ] ; xx [ 57 ] =
1.000000000000001 ; xx [ 59 ] = 1.251169307048272e-16 ; xx [ 169 ] =
1.110223024625157e-16 ; xx [ 170 ] = - xx [ 59 ] ; xx [ 171 ] =
3.38000027272356e-17 ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 169 , xx
+ 172 ) ; xx [ 169 ] = 2.908691710579205e-6 ; xx [ 170 ] =
2.868076938248717e-3 ; xx [ 171 ] = 4.94643927619723e-3 ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 169 , xx + 175 ) ; xx [ 185 ] =
- ( xx [ 167 ] + 0.06327543407822646 + xx [ 175 ] - xx [ 209 ] ) ; xx [ 186 ]
= 0.01459007920097803 - xx [ 197 ] - ( xx [ 176 ] + xx [ 452 ] ) ; xx [ 187 ]
= xx [ 203 ] + 0.05401888928708277 - ( xx [ 177 ] + xx [ 572 ] ) ; xx [ 175 ]
= - 1.477235478147115e-3 ; xx [ 176 ] = 9.60335389372561e-7 ; xx [ 177 ] =
3.118418564317923e-7 ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 175 , xx
+ 188 ) ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 700 , xx + 175 ) ; xx
[ 191 ] = xx [ 188 ] + xx [ 175 ] ; xx [ 192 ] = xx [ 189 ] + xx [ 176 ] ; xx
[ 193 ] = xx [ 190 ] + xx [ 177 ] ; xx [ 268 ] = 0.9912919252698491 ; xx [
269 ] = - 0.1316824810541687 ; xx [ 270 ] = 1.970490389246134e-4 ; xx [ 271 ]
= 6.519118568758793e-5 ; pm_math_Quaternion_compose_ra ( xx + 464 , xx + 268
, xx + 272 ) ; xx [ 188 ] = ( xx [ 272 ] * xx [ 274 ] + xx [ 273 ] * xx [ 275
] ) * xx [ 2 ] ; xx [ 189 ] = xx [ 2 ] * ( xx [ 274 ] * xx [ 275 ] - xx [ 272
] * xx [ 273 ] ) ; xx [ 190 ] = xx [ 11 ] - ( xx [ 273 ] * xx [ 273 ] + xx [
274 ] * xx [ 274 ] ) * xx [ 2 ] ; xx [ 63 ] = pm_math_Vector3_dot_ra ( xx +
172 , xx + 185 ) - pm_math_Vector3_dot_ra ( xx + 191 , xx + 188 ) ; xx [ 172
] = - xx [ 208 ] ; xx [ 173 ] = - xx [ 210 ] ; xx [ 174 ] = xx [ 521 ] ; xx [
64 ] = pm_math_Vector3_dot_ra ( xx + 172 , xx + 188 ) ; xx [ 96 ] = xx [ 63 ]
; xx [ 97 ] = - xx [ 64 ] ; solveSymmetricPosDef ( xx + 664 , xx + 96 , 2 , 1
, xx + 129 , xx + 158 ) ; xx [ 172 ] = xx [ 724 ] ; xx [ 173 ] = xx [ 726 ] ;
xx [ 174 ] = xx [ 728 ] ; xx [ 191 ] = xx [ 706 ] * xx [ 129 ] + xx [ 660 ] *
xx [ 130 ] ; xx [ 192 ] = xx [ 523 ] * xx [ 129 ] + xx [ 661 ] * xx [ 130 ] ;
xx [ 193 ] = xx [ 524 ] * xx [ 129 ] + xx [ 130 ] * xx [ 662 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 191 , xx + 268 ) ; xx [ 66 ] =
0.02263515933877589 ; xx [ 70 ] = - xx [ 66 ] ; xx [ 74 ] =
0.3826221044509303 ; xx [ 79 ] = 0.03730945979482574 ; xx [ 191 ] = xx [ 70 ]
; xx [ 192 ] = xx [ 74 ] ; xx [ 193 ] = xx [ 79 ] ; xx [ 271 ] = - xx [ 166 ]
; xx [ 272 ] = - xx [ 183 ] ; xx [ 273 ] = xx [ 199 ] ; xx [ 81 ] =
pm_math_Vector3_dot_ra ( xx + 271 , xx + 188 ) ; xx [ 96 ] = xx [ 81 ] / xx [
163 ] ; xx [ 97 ] = xx [ 168 ] * xx [ 96 ] ; xx [ 102 ] = xx [ 200 ] * xx [
96 ] ; xx [ 106 ] = xx [ 74 ] * xx [ 97 ] - xx [ 79 ] * xx [ 102 ] ; xx [ 111
] = xx [ 66 ] * xx [ 97 ] ; xx [ 271 ] = xx [ 106 ] ; xx [ 272 ] = xx [ 111 ]
; xx [ 273 ] = - ( xx [ 66 ] * xx [ 102 ] ) ; pm_math_Vector3_cross_ra ( xx +
191 , xx + 271 , xx + 274 ) ; xx [ 66 ] = 0.9228737611163665 ; xx [ 139 ] = (
xx [ 274 ] - xx [ 66 ] * xx [ 106 ] ) * xx [ 2 ] ; xx [ 106 ] = xx [ 102 ] +
xx [ 2 ] * ( xx [ 275 ] - xx [ 66 ] * xx [ 111 ] ) ; xx [ 111 ] = xx [ 97 ] +
( 0.02088939463244436 * xx [ 102 ] + xx [ 276 ] ) * xx [ 2 ] ; xx [ 191 ] =
xx [ 268 ] + xx [ 139 ] ; xx [ 192 ] = xx [ 269 ] + xx [ 106 ] ; xx [ 193 ] =
xx [ 270 ] + xx [ 111 ] ; xx [ 271 ] = xx [ 525 ] * xx [ 129 ] + xx [ 657 ] *
xx [ 130 ] ; xx [ 272 ] = xx [ 526 ] * xx [ 129 ] + xx [ 658 ] * xx [ 130 ] ;
xx [ 273 ] = xx [ 519 ] * xx [ 129 ] + xx [ 130 ] * xx [ 659 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 271 , xx + 274 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 268 , xx + 271 ) ; xx [ 268 ] = xx
[ 139 ] ; xx [ 269 ] = xx [ 106 ] ; xx [ 270 ] = xx [ 111 ] ;
pm_math_Vector3_cross_ra ( xx + 204 , xx + 268 , xx + 367 ) ; xx [ 268 ] = xx
[ 274 ] + xx [ 271 ] + xx [ 367 ] ; xx [ 269 ] = xx [ 275 ] + xx [ 272 ] + xx
[ 368 ] ; xx [ 270 ] = xx [ 276 ] + xx [ 273 ] + xx [ 369 ] ; xx [ 271 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 191 ) ; xx [ 272 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 191 ) ; xx [ 273 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 191 ) ; xx [ 274 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 268 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 191 ) ) ; xx [ 275 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 268 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 191 ) ) ; xx [ 276 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 268 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 191 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 271 , 6 , 1
, xx + 367 , xx + 384 ) ; xx [ 191 ] = xx [ 959 ] * xx [ 372 ] - ( xx [ 671 ]
* xx [ 370 ] + xx [ 940 ] * xx [ 371 ] ) ; xx [ 192 ] = xx [ 933 ] * xx [ 370
] - xx [ 952 ] * xx [ 371 ] + xx [ 971 ] * xx [ 372 ] ; xx [ 193 ] = xx [ 953
] * xx [ 371 ] - xx [ 934 ] * xx [ 370 ] + xx [ 972 ] * xx [ 372 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 191 , xx + 268 ) ; xx [
271 ] = xx [ 730 ] ; xx [ 272 ] = xx [ 732 ] ; xx [ 273 ] = xx [ 734 ] ; xx [
97 ] = xx [ 367 ] * xx [ 3 ] + xx [ 368 ] * xx [ 196 ] + xx [ 369 ] * xx [
522 ] + xx [ 941 ] * xx [ 370 ] - xx [ 960 ] * xx [ 371 ] + xx [ 979 ] * xx [
372 ] ; pm_math_Vector3_cross_ra ( xx + 191 , xx + 651 , xx + 274 ) ; xx [
102 ] = xx [ 367 ] * xx [ 22 ] + xx [ 368 ] * xx [ 21 ] + xx [ 369 ] * xx [
23 ] + xx [ 942 ] * xx [ 370 ] + xx [ 961 ] * xx [ 371 ] - xx [ 980 ] * xx [
372 ] ; xx [ 106 ] = xx [ 367 ] * xx [ 25 ] + xx [ 368 ] * xx [ 434 ] + xx [
369 ] * xx [ 1 ] - xx [ 943 ] * xx [ 370 ] - xx [ 962 ] * xx [ 371 ] - xx [
981 ] * xx [ 372 ] ; xx [ 367 ] = xx [ 97 ] + xx [ 274 ] ; xx [ 368 ] = xx [
102 ] + xx [ 275 ] ; xx [ 369 ] = xx [ 106 ] + xx [ 276 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 367 , xx + 274 ) ; xx [
367 ] = xx [ 725 ] ; xx [ 368 ] = xx [ 727 ] ; xx [ 369 ] = xx [ 729 ] ; xx [
370 ] = xx [ 731 ] ; xx [ 371 ] = xx [ 733 ] ; xx [ 372 ] = xx [ 735 ] ; xx [
384 ] = - xx [ 66 ] ; xx [ 385 ] = xx [ 70 ] ; xx [ 386 ] = xx [ 74 ] ; xx [
387 ] = xx [ 79 ] ; pm_math_Vector3_cross_ra ( xx + 191 , xx + 204 , xx + 373
) ; xx [ 191 ] = xx [ 97 ] + xx [ 373 ] ; xx [ 192 ] = xx [ 102 ] + xx [ 374
] ; xx [ 193 ] = xx [ 106 ] + xx [ 375 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 384 , xx + 191 , xx + 373 ) ; xx [ 66 ] = 0.9229115780805051 ; xx [ 70
] = - 0.02085503273058337 ; xx [ 74 ] = 0.3825441798635174 ; xx [ 79 ] =
0.03819996213750987 ; xx [ 388 ] = - xx [ 66 ] ; xx [ 389 ] = xx [ 70 ] ; xx
[ 390 ] = xx [ 74 ] ; xx [ 391 ] = xx [ 79 ] ; xx [ 97 ] = 1.0 ; xx [ 102 ] =
xx [ 97 ] / xx [ 432 ] ; xx [ 191 ] = - ( xx [ 428 ] * xx [ 102 ] ) ; xx [
192 ] = - ( xx [ 431 ] * xx [ 102 ] ) ; xx [ 193 ] = - ( xx [ 438 ] * xx [
102 ] ) ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 191 , xx + 629 ) ; xx
[ 191 ] = - ( xx [ 405 ] * xx [ 102 ] ) ; xx [ 192 ] = - ( xx [ 409 ] * xx [
102 ] ) ; xx [ 193 ] = - ( xx [ 825 ] * xx [ 102 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 191 , xx + 712 ) ;
pm_math_Vector3_cross_ra ( xx + 845 , xx + 629 , xx + 191 ) ; xx [ 715 ] = xx
[ 712 ] + xx [ 191 ] ; xx [ 716 ] = xx [ 713 ] + xx [ 192 ] ; xx [ 717 ] = xx
[ 714 ] + xx [ 193 ] ; xx [ 724 ] = - pm_math_Vector3_dot_ra ( xx + 26 , xx +
629 ) ; xx [ 725 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx + 629 ) ; xx [
726 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 629 ) ; xx [ 727 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 715 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 629 ) ) ; xx [ 728 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 715 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 629 ) ) ; xx [ 729 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 715 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 629 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 724 , 6 , 1
, xx + 712 , xx + 730 ) ; xx [ 191 ] = xx [ 959 ] * xx [ 717 ] - ( xx [ 671 ]
* xx [ 715 ] + xx [ 940 ] * xx [ 716 ] ) ; xx [ 192 ] = xx [ 933 ] * xx [ 715
] - xx [ 952 ] * xx [ 716 ] + xx [ 971 ] * xx [ 717 ] ; xx [ 193 ] = xx [ 953
] * xx [ 716 ] - xx [ 934 ] * xx [ 715 ] + xx [ 972 ] * xx [ 717 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 191 , xx + 629 ) ; xx [
106 ] = xx [ 712 ] * xx [ 3 ] + xx [ 713 ] * xx [ 196 ] + xx [ 714 ] * xx [
522 ] + xx [ 941 ] * xx [ 715 ] - xx [ 960 ] * xx [ 716 ] + xx [ 979 ] * xx [
717 ] ; pm_math_Vector3_cross_ra ( xx + 191 , xx + 651 , xx + 718 ) ; xx [
111 ] = xx [ 712 ] * xx [ 22 ] + xx [ 713 ] * xx [ 21 ] + xx [ 714 ] * xx [
23 ] + xx [ 942 ] * xx [ 715 ] + xx [ 961 ] * xx [ 716 ] - xx [ 980 ] * xx [
717 ] ; xx [ 139 ] = xx [ 712 ] * xx [ 25 ] + xx [ 713 ] * xx [ 434 ] + xx [
714 ] * xx [ 1 ] - xx [ 943 ] * xx [ 715 ] - xx [ 962 ] * xx [ 716 ] - xx [
981 ] * xx [ 717 ] ; xx [ 712 ] = xx [ 106 ] + xx [ 718 ] ; xx [ 713 ] = xx [
111 ] + xx [ 719 ] ; xx [ 714 ] = xx [ 139 ] + xx [ 720 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 712 , xx + 715 ) ;
pm_math_Vector3_cross_ra ( xx + 191 , xx + 204 , xx + 712 ) ; xx [ 718 ] = xx
[ 106 ] + xx [ 712 ] ; xx [ 719 ] = xx [ 111 ] + xx [ 713 ] ; xx [ 720 ] = xx
[ 139 ] + xx [ 714 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx +
718 , xx + 712 ) ; xx [ 158 ] = xx [ 64 ] * ( pm_math_Vector3_dot_ra ( xx +
367 , xx + 629 ) + pm_math_Vector3_dot_ra ( xx + 370 , xx + 715 ) ) - (
pm_math_Vector3_dot_ra ( xx + 172 , xx + 629 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 715 ) ) * xx [ 63 ] - xx [ 81 ] * ( xx [ 201 ] * xx [ 713 ] + xx
[ 207 ] * xx [ 714 ] ) ; pm_math_Quaternion_compose_ra ( xx + 464 , xx + 489
, xx + 629 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 684 , xx
+ 712 ) ; xx [ 159 ] = - 8.852854494761919e-5 ; xx [ 715 ] = xx [ 460 ] ; xx
[ 716 ] = xx [ 159 ] ; xx [ 717 ] = - 0.07357482457770126 ;
pm_math_Vector3_cross_ra ( xx + 712 , xx + 715 , xx + 718 ) ;
pm_math_Quaternion_xform_ra ( xx + 629 , xx + 718 , xx + 724 ) ; xx [ 718 ] =
0.7044167893867053 ; xx [ 719 ] = - 0.08618526586337708 ; xx [ 720 ] =
0.7045346597422907 ; xx [ 167 ] = pm_math_Vector3_dot_ra ( xx + 724 , xx +
718 ) ; xx [ 576 ] = xx [ 167 ] ; xx [ 577 ] = xx [ 317 ] ;
solveSymmetricPosDef ( xx + 664 , xx + 576 , 2 , 1 , xx + 612 , xx + 621 ) ;
xx [ 727 ] = xx [ 706 ] * xx [ 612 ] + xx [ 660 ] * xx [ 613 ] ; xx [ 728 ] =
xx [ 523 ] * xx [ 612 ] + xx [ 661 ] * xx [ 613 ] ; xx [ 729 ] = xx [ 524 ] *
xx [ 612 ] + xx [ 613 ] * xx [ 662 ] ; pm_math_Quaternion_xform_ra ( xx + 464
, xx + 727 , xx + 730 ) ; xx [ 727 ] = xx [ 525 ] * xx [ 612 ] + xx [ 657 ] *
xx [ 613 ] ; xx [ 728 ] = xx [ 526 ] * xx [ 612 ] + xx [ 658 ] * xx [ 613 ] ;
xx [ 729 ] = xx [ 519 ] * xx [ 612 ] + xx [ 613 ] * xx [ 659 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 727 , xx + 733 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 730 , xx + 727 ) ; xx [ 757 ] = xx
[ 733 ] + xx [ 727 ] ; xx [ 758 ] = xx [ 734 ] + xx [ 728 ] ; xx [ 759 ] = xx
[ 735 ] + xx [ 729 ] ; xx [ 760 ] = - pm_math_Vector3_dot_ra ( xx + 26 , xx +
730 ) ; xx [ 761 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx + 730 ) ; xx [
762 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 730 ) ; xx [ 763 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 757 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 730 ) ) ; xx [ 764 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 757 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 730 ) ) ; xx [ 765 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 757 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 730 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 760 , 6 , 1
, xx + 727 , xx + 775 ) ; xx [ 733 ] = xx [ 959 ] * xx [ 732 ] - ( xx [ 671 ]
* xx [ 730 ] + xx [ 940 ] * xx [ 731 ] ) ; xx [ 734 ] = xx [ 933 ] * xx [ 730
] - xx [ 952 ] * xx [ 731 ] + xx [ 971 ] * xx [ 732 ] ; xx [ 735 ] = xx [ 953
] * xx [ 731 ] - xx [ 934 ] * xx [ 730 ] + xx [ 972 ] * xx [ 732 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 733 , xx + 757 ) ; xx [
197 ] = xx [ 727 ] * xx [ 3 ] + xx [ 728 ] * xx [ 196 ] + xx [ 729 ] * xx [
522 ] + xx [ 941 ] * xx [ 730 ] - xx [ 960 ] * xx [ 731 ] + xx [ 979 ] * xx [
732 ] ; pm_math_Vector3_cross_ra ( xx + 733 , xx + 651 , xx + 760 ) ; xx [
203 ] = xx [ 727 ] * xx [ 22 ] + xx [ 728 ] * xx [ 21 ] + xx [ 729 ] * xx [
23 ] + xx [ 942 ] * xx [ 730 ] + xx [ 961 ] * xx [ 731 ] - xx [ 980 ] * xx [
732 ] ; xx [ 209 ] = xx [ 727 ] * xx [ 25 ] + xx [ 728 ] * xx [ 434 ] + xx [
729 ] * xx [ 1 ] - xx [ 943 ] * xx [ 730 ] - xx [ 962 ] * xx [ 731 ] - xx [
981 ] * xx [ 732 ] ; xx [ 727 ] = xx [ 197 ] + xx [ 760 ] ; xx [ 728 ] = xx [
203 ] + xx [ 761 ] ; xx [ 729 ] = xx [ 209 ] + xx [ 762 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 727 , xx + 730 ) ; xx [
373 ] = xx [ 612 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx + 757 ) +
pm_math_Vector3_dot_ra ( xx + 271 , xx + 730 ) ) ; pm_math_Vector3_cross_ra (
xx + 733 , xx + 204 , xx + 727 ) ; xx [ 760 ] = xx [ 197 ] + xx [ 727 ] ; xx
[ 761 ] = xx [ 203 ] + xx [ 728 ] ; xx [ 762 ] = xx [ 209 ] + xx [ 729 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 760 , xx + 727 ) ; xx [
392 ] = xx [ 63 ] * xx [ 373 ] - xx [ 64 ] * ( xx [ 613 ] - (
pm_math_Vector3_dot_ra ( xx + 367 , xx + 757 ) + pm_math_Vector3_dot_ra ( xx
+ 370 , xx + 730 ) ) ) - xx [ 81 ] * ( xx [ 201 ] * xx [ 728 ] + xx [ 207 ] *
xx [ 729 ] ) ; xx [ 727 ] = 0.05435570616770889 ; xx [ 728 ] =
0.996236014971251 ; xx [ 729 ] = 0.06752229025448525 ; xx [ 444 ] =
pm_math_Vector3_dot_ra ( xx + 724 , xx + 727 ) ; xx [ 576 ] = xx [ 444 ] ; xx
[ 577 ] = xx [ 317 ] ; solveSymmetricPosDef ( xx + 664 , xx + 576 , 2 , 1 ,
xx + 612 , xx + 621 ) ; xx [ 724 ] = xx [ 706 ] * xx [ 612 ] + xx [ 660 ] *
xx [ 613 ] ; xx [ 725 ] = xx [ 523 ] * xx [ 612 ] + xx [ 661 ] * xx [ 613 ] ;
xx [ 726 ] = xx [ 524 ] * xx [ 612 ] + xx [ 613 ] * xx [ 662 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 724 , xx + 730 ) ; xx [ 724 ] =
xx [ 525 ] * xx [ 612 ] + xx [ 657 ] * xx [ 613 ] ; xx [ 725 ] = xx [ 526 ] *
xx [ 612 ] + xx [ 658 ] * xx [ 613 ] ; xx [ 726 ] = xx [ 519 ] * xx [ 612 ] +
xx [ 613 ] * xx [ 659 ] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 724 ,
xx + 757 ) ; pm_math_Vector3_cross_ra ( xx + 651 , xx + 730 , xx + 724 ) ; xx
[ 760 ] = xx [ 757 ] + xx [ 724 ] ; xx [ 761 ] = xx [ 758 ] + xx [ 725 ] ; xx
[ 762 ] = xx [ 759 ] + xx [ 726 ] ; xx [ 775 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 730 ) ; xx [ 776 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx
+ 730 ) ; xx [ 777 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 730 ) ; xx
[ 778 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 760 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 730 ) ) ; xx [ 779 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 760 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 730 ) ) ; xx [ 780 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 760 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 730 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 775 , 6 , 1 , xx + 757 , xx + 793 ) ;
xx [ 724 ] = xx [ 959 ] * xx [ 762 ] - ( xx [ 671 ] * xx [ 760 ] + xx [ 940 ]
* xx [ 761 ] ) ; xx [ 725 ] = xx [ 933 ] * xx [ 760 ] - xx [ 952 ] * xx [ 761
] + xx [ 971 ] * xx [ 762 ] ; xx [ 726 ] = xx [ 953 ] * xx [ 761 ] - xx [ 934
] * xx [ 760 ] + xx [ 972 ] * xx [ 762 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 464 , xx + 724 , xx + 730 ) ; xx [ 448 ] = xx [ 757 ] * xx [ 3 ] + xx
[ 758 ] * xx [ 196 ] + xx [ 759 ] * xx [ 522 ] + xx [ 941 ] * xx [ 760 ] - xx
[ 960 ] * xx [ 761 ] + xx [ 979 ] * xx [ 762 ] ; pm_math_Vector3_cross_ra (
xx + 724 , xx + 651 , xx + 763 ) ; xx [ 452 ] = xx [ 757 ] * xx [ 22 ] + xx [
758 ] * xx [ 21 ] + xx [ 759 ] * xx [ 23 ] + xx [ 942 ] * xx [ 760 ] + xx [
961 ] * xx [ 761 ] - xx [ 980 ] * xx [ 762 ] ; xx [ 518 ] = xx [ 757 ] * xx [
25 ] + xx [ 758 ] * xx [ 434 ] + xx [ 759 ] * xx [ 1 ] - xx [ 943 ] * xx [
760 ] - xx [ 962 ] * xx [ 761 ] - xx [ 981 ] * xx [ 762 ] ; xx [ 757 ] = xx [
448 ] + xx [ 763 ] ; xx [ 758 ] = xx [ 452 ] + xx [ 764 ] ; xx [ 759 ] = xx [
518 ] + xx [ 765 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 757
, xx + 760 ) ; xx [ 520 ] = xx [ 612 ] - ( pm_math_Vector3_dot_ra ( xx + 172
, xx + 730 ) + pm_math_Vector3_dot_ra ( xx + 271 , xx + 760 ) ) ;
pm_math_Vector3_cross_ra ( xx + 724 , xx + 204 , xx + 757 ) ; xx [ 763 ] = xx
[ 448 ] + xx [ 757 ] ; xx [ 764 ] = xx [ 452 ] + xx [ 758 ] ; xx [ 765 ] = xx
[ 518 ] + xx [ 759 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx +
763 , xx + 757 ) ; xx [ 572 ] = xx [ 63 ] * xx [ 520 ] - xx [ 64 ] * ( xx [
613 ] - ( pm_math_Vector3_dot_ra ( xx + 367 , xx + 730 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 760 ) ) ) - xx [ 81 ] * ( xx [ 201 ]
* xx [ 758 ] + xx [ 207 ] * xx [ 759 ] ) ; xx [ 730 ] = 0.013582786684373 ;
xx [ 731 ] = - 2.58589188160877e-6 ; xx [ 732 ] = - 0.01849903189296259 ;
pm_math_Vector3_cross_ra ( xx + 712 , xx + 730 , xx + 757 ) ;
pm_math_Quaternion_xform_ra ( xx + 629 , xx + 757 , xx + 712 ) ;
pm_math_Vector3_cross_ra ( xx + 684 , xx + 648 , xx + 757 ) ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 757 , xx + 760 ) ; xx [ 757 ] =
xx [ 712 ] + xx [ 760 ] + xx [ 175 ] ; xx [ 758 ] = xx [ 713 ] + xx [ 761 ] +
xx [ 176 ] ; xx [ 759 ] = xx [ 714 ] + xx [ 762 ] + xx [ 177 ] ; xx [ 712 ] =
- 0.7077022283680728 ; xx [ 713 ] = - 9.268355963183032e-3 ; xx [ 714 ] =
0.7064500361247099 ; xx [ 576 ] = pm_math_Vector3_dot_ra ( xx + 757 , xx +
712 ) ; xx [ 577 ] = 0.9999999999999832 ; solveSymmetricPosDef ( xx + 664 ,
xx + 576 , 2 , 1 , xx + 612 , xx + 621 ) ; xx [ 757 ] = xx [ 706 ] * xx [ 612
] + xx [ 660 ] * xx [ 613 ] ; xx [ 758 ] = xx [ 523 ] * xx [ 612 ] + xx [ 661
] * xx [ 613 ] ; xx [ 759 ] = xx [ 524 ] * xx [ 612 ] + xx [ 613 ] * xx [ 662
] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 757 , xx + 760 ) ; xx [ 757
] = xx [ 525 ] * xx [ 612 ] + xx [ 657 ] * xx [ 613 ] ; xx [ 758 ] = xx [ 526
] * xx [ 612 ] + xx [ 658 ] * xx [ 613 ] ; xx [ 759 ] = xx [ 519 ] * xx [ 612
] + xx [ 613 ] * xx [ 659 ] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx +
757 , xx + 763 ) ; pm_math_Vector3_cross_ra ( xx + 651 , xx + 760 , xx + 757
) ; xx [ 775 ] = xx [ 763 ] + xx [ 757 ] ; xx [ 776 ] = xx [ 764 ] + xx [ 758
] ; xx [ 777 ] = xx [ 765 ] + xx [ 759 ] ; xx [ 778 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 760 ) ; xx [ 779 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 760 ) ; xx [ 780 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 760 ) ; xx [ 781 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 775 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 760 ) ) ; xx [ 782 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 775 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 760 ) ) ; xx [ 783 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 775 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 760 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 778 , 6 , 1
, xx + 757 , xx + 793 ) ; xx [ 763 ] = xx [ 959 ] * xx [ 762 ] - ( xx [ 671 ]
* xx [ 760 ] + xx [ 940 ] * xx [ 761 ] ) ; xx [ 764 ] = xx [ 933 ] * xx [ 760
] - xx [ 952 ] * xx [ 761 ] + xx [ 971 ] * xx [ 762 ] ; xx [ 765 ] = xx [ 953
] * xx [ 761 ] - xx [ 934 ] * xx [ 760 ] + xx [ 972 ] * xx [ 762 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 763 , xx + 775 ) ; xx [
616 ] = xx [ 757 ] * xx [ 3 ] + xx [ 758 ] * xx [ 196 ] + xx [ 759 ] * xx [
522 ] + xx [ 941 ] * xx [ 760 ] - xx [ 960 ] * xx [ 761 ] + xx [ 979 ] * xx [
762 ] ; pm_math_Vector3_cross_ra ( xx + 763 , xx + 651 , xx + 778 ) ; xx [
618 ] = xx [ 757 ] * xx [ 22 ] + xx [ 758 ] * xx [ 21 ] + xx [ 759 ] * xx [
23 ] + xx [ 942 ] * xx [ 760 ] + xx [ 961 ] * xx [ 761 ] - xx [ 980 ] * xx [
762 ] ; xx [ 621 ] = xx [ 757 ] * xx [ 25 ] + xx [ 758 ] * xx [ 434 ] + xx [
759 ] * xx [ 1 ] - xx [ 943 ] * xx [ 760 ] - xx [ 962 ] * xx [ 761 ] - xx [
981 ] * xx [ 762 ] ; xx [ 757 ] = xx [ 616 ] + xx [ 778 ] ; xx [ 758 ] = xx [
618 ] + xx [ 779 ] ; xx [ 759 ] = xx [ 621 ] + xx [ 780 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 757 , xx + 760 ) ; xx [
622 ] = xx [ 612 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx + 775 ) +
pm_math_Vector3_dot_ra ( xx + 271 , xx + 760 ) ) ; xx [ 612 ] = xx [ 613 ] -
( pm_math_Vector3_dot_ra ( xx + 367 , xx + 775 ) + pm_math_Vector3_dot_ra (
xx + 370 , xx + 760 ) ) ; pm_math_Vector3_cross_ra ( xx + 763 , xx + 204 , xx
+ 757 ) ; xx [ 760 ] = xx [ 616 ] + xx [ 757 ] ; xx [ 761 ] = xx [ 618 ] + xx
[ 758 ] ; xx [ 762 ] = xx [ 621 ] + xx [ 759 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 760 , xx + 757 ) ; xx [
613 ] = xx [ 63 ] * xx [ 622 ] - xx [ 64 ] * xx [ 612 ] - xx [ 81 ] * ( xx [
201 ] * xx [ 758 ] + xx [ 207 ] * xx [ 759 ] ) ; xx [ 634 ] = xx [ 57 ] / xx
[ 178 ] ; xx [ 757 ] = - ( xx [ 148 ] * xx [ 634 ] ) ; xx [ 758 ] = - ( xx [
180 ] * xx [ 634 ] ) ; xx [ 759 ] = - ( xx [ 182 ] * xx [ 634 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 125 , xx + 757 , xx + 760 ) ;
pm_math_Vector3_cross_ra ( xx + 140 , xx + 760 , xx + 757 ) ; xx [ 775 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 760 ) ; xx [ 776 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 760 ) ; xx [ 777 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 760 ) ; xx [ 778 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 757 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 760 ) ) ; xx [ 779 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 757 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 760 ) ) ; xx [ 780 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 757 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 760 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 775 , 6 , 1
, xx + 757 , xx + 793 ) ; xx [ 775 ] = xx [ 959 ] * xx [ 762 ] - ( xx [ 671 ]
* xx [ 760 ] + xx [ 940 ] * xx [ 761 ] ) ; xx [ 776 ] = xx [ 933 ] * xx [ 760
] - xx [ 952 ] * xx [ 761 ] + xx [ 971 ] * xx [ 762 ] ; xx [ 777 ] = xx [ 953
] * xx [ 761 ] - xx [ 934 ] * xx [ 760 ] + xx [ 972 ] * xx [ 762 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 775 , xx + 778 ) ; xx [
635 ] = xx [ 757 ] * xx [ 3 ] + xx [ 758 ] * xx [ 196 ] + xx [ 759 ] * xx [
522 ] + xx [ 941 ] * xx [ 760 ] - xx [ 960 ] * xx [ 761 ] + xx [ 979 ] * xx [
762 ] ; pm_math_Vector3_cross_ra ( xx + 775 , xx + 651 , xx + 781 ) ; xx [
640 ] = xx [ 757 ] * xx [ 22 ] + xx [ 758 ] * xx [ 21 ] + xx [ 759 ] * xx [
23 ] + xx [ 942 ] * xx [ 760 ] + xx [ 961 ] * xx [ 761 ] - xx [ 980 ] * xx [
762 ] ; xx [ 641 ] = xx [ 757 ] * xx [ 25 ] + xx [ 758 ] * xx [ 434 ] + xx [
759 ] * xx [ 1 ] - xx [ 943 ] * xx [ 760 ] - xx [ 962 ] * xx [ 761 ] - xx [
981 ] * xx [ 762 ] ; xx [ 757 ] = xx [ 635 ] + xx [ 781 ] ; xx [ 758 ] = xx [
640 ] + xx [ 782 ] ; xx [ 759 ] = xx [ 641 ] + xx [ 783 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 757 , xx + 760 ) ; xx [
654 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 778 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 760 ) ; xx [ 668 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 778 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 760 ) ; pm_math_Vector3_cross_ra ( xx + 775 , xx + 204 , xx +
757 ) ; xx [ 760 ] = xx [ 635 ] + xx [ 757 ] ; xx [ 761 ] = xx [ 640 ] + xx [
758 ] ; xx [ 762 ] = xx [ 641 ] + xx [ 759 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 760 , xx + 757 ) ; xx [
669 ] = xx [ 64 ] * xx [ 654 ] - xx [ 668 ] * xx [ 63 ] - xx [ 81 ] * ( xx [
201 ] * xx [ 758 ] + xx [ 207 ] * xx [ 759 ] ) ; xx [ 757 ] =
6.459047385173115e-3 ; xx [ 758 ] = - 9.24648174485767e-5 ; xx [ 759 ] =
2.250809555350908e-5 ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 757 , xx
+ 760 ) ; xx [ 757 ] = 6.480496605446796e-3 ; xx [ 758 ] =
9.242608862956347e-5 ; xx [ 759 ] = 2.2489322266878e-5 ; xx [ 778 ] = - xx [
757 ] ; xx [ 779 ] = xx [ 758 ] ; xx [ 780 ] = - xx [ 759 ] ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 778 , xx + 781 ) ; xx [ 793 ] =
xx [ 760 ] + xx [ 781 ] ; xx [ 794 ] = xx [ 761 ] + xx [ 782 ] ; xx [ 795 ] =
xx [ 762 ] + xx [ 783 ] ; xx [ 796 ] = 0.5180593628885556 ; xx [ 797 ] = -
0.4813049877207372 ; xx [ 798 ] = - 0.4812048384058716 ; xx [ 799 ] =
0.518075196099393 ; pm_math_Quaternion_compose_ra ( xx + 236 , xx + 796 , xx
+ 800 ) ; xx [ 760 ] = xx [ 11 ] - ( xx [ 801 ] * xx [ 801 ] + xx [ 802 ] *
xx [ 802 ] ) * xx [ 2 ] ; xx [ 796 ] = ( xx [ 800 ] * xx [ 802 ] + xx [ 801 ]
* xx [ 803 ] ) * xx [ 2 ] ; xx [ 797 ] = xx [ 2 ] * ( xx [ 802 ] * xx [ 803 ]
- xx [ 800 ] * xx [ 801 ] ) ; xx [ 798 ] = xx [ 760 ] ; xx [ 761 ] =
pm_math_Vector3_dot_ra ( xx + 793 , xx + 796 ) ; xx [ 793 ] = xx [ 761 ] ; xx
[ 794 ] = - xx [ 760 ] ; solveSymmetricPosDef ( xx + 393 , xx + 793 , 2 , 1 ,
xx + 799 , xx + 801 ) ; xx [ 793 ] = xx [ 364 ] * xx [ 800 ] - xx [ 398 ] *
xx [ 799 ] ; xx [ 794 ] = xx [ 399 ] * xx [ 799 ] + xx [ 400 ] * xx [ 800 ] ;
xx [ 795 ] = xx [ 402 ] * xx [ 800 ] - xx [ 401 ] * xx [ 799 ] ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 793 , xx + 801 ) ; xx [ 762 ] =
( xx [ 413 ] * xx [ 801 ] + xx [ 427 ] * xx [ 802 ] ) / xx [ 432 ] ; xx [ 793
] = xx [ 801 ] - xx [ 428 ] * xx [ 762 ] ; xx [ 794 ] = xx [ 802 ] - xx [ 431
] * xx [ 762 ] ; xx [ 795 ] = xx [ 803 ] - xx [ 438 ] * xx [ 762 ] ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 793 , xx + 804 ) ; xx [ 793 ] =
xx [ 376 ] * xx [ 799 ] ; xx [ 794 ] = xx [ 379 ] * xx [ 799 ] ; xx [ 795 ] =
xx [ 397 ] * xx [ 799 ] ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 793 ,
xx + 807 ) ; pm_math_Vector3_cross_ra ( xx + 813 , xx + 801 , xx + 793 ) ; xx
[ 801 ] = xx [ 807 ] + xx [ 793 ] - xx [ 405 ] * xx [ 762 ] ; xx [ 802 ] = xx
[ 808 ] + xx [ 794 ] - xx [ 409 ] * xx [ 762 ] ; xx [ 803 ] = xx [ 809 ] + xx
[ 795 ] - xx [ 825 ] * xx [ 762 ] ; pm_math_Quaternion_xform_ra ( xx + 388 ,
xx + 801 , xx + 793 ) ; pm_math_Vector3_cross_ra ( xx + 845 , xx + 804 , xx +
801 ) ; xx [ 807 ] = xx [ 793 ] + xx [ 801 ] ; xx [ 808 ] = xx [ 794 ] + xx [
802 ] ; xx [ 809 ] = xx [ 795 ] + xx [ 803 ] ; xx [ 816 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 804 ) ; xx [ 817 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 804 ) ; xx [ 818 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 804 ) ; xx [ 819 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 807 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 804 ) ) ; xx [ 820 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 807 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 804 ) ) ; xx [ 821 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 807 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 804 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 816 , 6 , 1
, xx + 801 , xx + 827 ) ; xx [ 793 ] = xx [ 959 ] * xx [ 806 ] - ( xx [ 671 ]
* xx [ 804 ] + xx [ 940 ] * xx [ 805 ] ) ; xx [ 794 ] = xx [ 933 ] * xx [ 804
] - xx [ 952 ] * xx [ 805 ] + xx [ 971 ] * xx [ 806 ] ; xx [ 795 ] = xx [ 953
] * xx [ 805 ] - xx [ 934 ] * xx [ 804 ] + xx [ 972 ] * xx [ 806 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 793 , xx + 807 ) ; xx [
810 ] = xx [ 801 ] * xx [ 3 ] + xx [ 802 ] * xx [ 196 ] + xx [ 803 ] * xx [
522 ] + xx [ 941 ] * xx [ 804 ] - xx [ 960 ] * xx [ 805 ] + xx [ 979 ] * xx [
806 ] ; pm_math_Vector3_cross_ra ( xx + 793 , xx + 651 , xx + 816 ) ; xx [
819 ] = xx [ 801 ] * xx [ 22 ] + xx [ 802 ] * xx [ 21 ] + xx [ 803 ] * xx [
23 ] + xx [ 942 ] * xx [ 804 ] + xx [ 961 ] * xx [ 805 ] - xx [ 980 ] * xx [
806 ] ; xx [ 820 ] = xx [ 801 ] * xx [ 25 ] + xx [ 802 ] * xx [ 434 ] + xx [
803 ] * xx [ 1 ] - xx [ 943 ] * xx [ 804 ] - xx [ 962 ] * xx [ 805 ] - xx [
981 ] * xx [ 806 ] ; xx [ 801 ] = xx [ 810 ] + xx [ 816 ] ; xx [ 802 ] = xx [
819 ] + xx [ 817 ] ; xx [ 803 ] = xx [ 820 ] + xx [ 818 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 801 , xx + 804 ) ; xx [
801 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 807 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 804 ) ; xx [ 802 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 807 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 804 ) ; pm_math_Vector3_cross_ra ( xx + 793 , xx + 204 , xx +
803 ) ; xx [ 806 ] = xx [ 810 ] + xx [ 803 ] ; xx [ 807 ] = xx [ 819 ] + xx [
804 ] ; xx [ 808 ] = xx [ 820 ] + xx [ 805 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 806 , xx + 803 ) ; xx [
803 ] = xx [ 64 ] * xx [ 801 ] - xx [ 802 ] * xx [ 63 ] - xx [ 81 ] * ( xx [
201 ] * xx [ 804 ] + xx [ 207 ] * xx [ 805 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 388 , xx + 380 , xx + 804 ) ; xx [ 816 ]
= - 2.775557561562891e-16 ; xx [ 817 ] = xx [ 59 ] ; xx [ 818 ] = -
3.373224009145526e-17 ; pm_math_Quaternion_xform_ra ( xx + 804 , xx + 816 ,
xx + 821 ) ; xx [ 816 ] = 2.908691710755093e-6 ; xx [ 817 ] =
2.868076938250087e-3 ; xx [ 818 ] = 4.94643927619916e-3 ;
pm_math_Quaternion_xform_ra ( xx + 804 , xx + 816 , xx + 827 ) ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 813 , xx + 830 ) ; xx [ 833 ] =
0.06121823687361252 - xx [ 144 ] - ( xx [ 827 ] + xx [ 830 ] + xx [ 838 ] +
xx [ 836 ] ) ; xx [ 834 ] = 0.01623907589682638 - xx [ 181 ] - ( xx [ 828 ] +
xx [ 831 ] - xx [ 841 ] + xx [ 839 ] ) ; xx [ 835 ] = xx [ 198 ] -
0.07052859367409821 - ( xx [ 829 ] + xx [ 832 ] + xx [ 843 ] - xx [ 844 ] ) ;
xx [ 827 ] = - 1.477235478147926e-3 ; xx [ 828 ] = 9.60335389539274e-7 ; xx [
829 ] = 3.118418563877457e-7 ; pm_math_Quaternion_xform_ra ( xx + 804 , xx +
827 , xx + 830 ) ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 781 , xx +
827 ) ; xx [ 884 ] = xx [ 830 ] + xx [ 827 ] ; xx [ 885 ] = xx [ 831 ] + xx [
828 ] ; xx [ 886 ] = xx [ 832 ] + xx [ 829 ] ; xx [ 827 ] = 0.991291925269849
; xx [ 828 ] = - 0.1316824810541695 ; xx [ 829 ] = 1.970490389249778e-4 ; xx
[ 830 ] = 6.519118568753968e-5 ; pm_math_Quaternion_compose_ra ( xx + 804 ,
xx + 827 , xx + 887 ) ; xx [ 827 ] = ( xx [ 887 ] * xx [ 889 ] + xx [ 888 ] *
xx [ 890 ] ) * xx [ 2 ] ; xx [ 828 ] = xx [ 2 ] * ( xx [ 889 ] * xx [ 890 ] -
xx [ 887 ] * xx [ 888 ] ) ; xx [ 829 ] = xx [ 11 ] - ( xx [ 888 ] * xx [ 888
] + xx [ 889 ] * xx [ 889 ] ) * xx [ 2 ] ; xx [ 59 ] = pm_math_Vector3_dot_ra
( xx + 821 , xx + 833 ) - pm_math_Vector3_dot_ra ( xx + 884 , xx + 827 ) ; xx
[ 144 ] = 0.7064500361247104 ; xx [ 821 ] = xx [ 211 ] ; xx [ 822 ] = xx [
212 ] ; xx [ 823 ] = - xx [ 144 ] ; xx [ 181 ] = pm_math_Vector3_dot_ra ( xx
+ 821 , xx + 827 ) ; xx [ 211 ] = xx [ 59 ] ; xx [ 212 ] = - xx [ 181 ] ;
solveSymmetricPosDef ( xx + 393 , xx + 211 , 2 , 1 , xx + 808 , xx + 821 ) ;
xx [ 821 ] = xx [ 364 ] * xx [ 809 ] - xx [ 398 ] * xx [ 808 ] ; xx [ 822 ] =
xx [ 399 ] * xx [ 808 ] + xx [ 400 ] * xx [ 809 ] ; xx [ 823 ] = xx [ 402 ] *
xx [ 809 ] - xx [ 401 ] * xx [ 808 ] ; pm_math_Quaternion_xform_ra ( xx + 380
, xx + 821 , xx + 830 ) ; xx [ 821 ] = xx [ 837 ] ; xx [ 822 ] = - xx [ 840 ]
; xx [ 823 ] = xx [ 842 ] ; xx [ 198 ] = pm_math_Vector3_dot_ra ( xx + 821 ,
xx + 827 ) ; xx [ 211 ] = ( xx [ 413 ] * xx [ 830 ] + xx [ 427 ] * xx [ 831 ]
+ xx [ 198 ] ) / xx [ 432 ] ; xx [ 821 ] = xx [ 830 ] - xx [ 428 ] * xx [ 211
] ; xx [ 822 ] = xx [ 831 ] - xx [ 431 ] * xx [ 211 ] ; xx [ 823 ] = xx [ 832
] - xx [ 438 ] * xx [ 211 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx +
821 , xx + 884 ) ; xx [ 212 ] = 0.03882994823691858 ; xx [ 821 ] = - xx [ 212
] ; xx [ 822 ] = 0.9229564379288121 ; xx [ 823 ] = - xx [ 822 ] ; xx [ 824 ]
= 0.01896689434564094 ; xx [ 836 ] = - xx [ 824 ] ; xx [ 887 ] = xx [ 821 ] ;
xx [ 888 ] = xx [ 823 ] ; xx [ 889 ] = xx [ 836 ] ; xx [ 890 ] = - xx [ 143 ]
; xx [ 891 ] = - xx [ 160 ] ; xx [ 892 ] = xx [ 194 ] ; xx [ 838 ] =
pm_math_Vector3_dot_ra ( xx + 890 , xx + 827 ) ; xx [ 839 ] = xx [ 838 ] / xx
[ 164 ] ; xx [ 841 ] = xx [ 168 ] * xx [ 839 ] ; xx [ 843 ] = xx [ 162 ] * xx
[ 839 ] ; xx [ 844 ] = xx [ 822 ] * xx [ 841 ] - xx [ 824 ] * xx [ 843 ] ; xx
[ 822 ] = xx [ 212 ] * xx [ 841 ] ; xx [ 890 ] = xx [ 844 ] ; xx [ 891 ] = -
xx [ 822 ] ; xx [ 892 ] = xx [ 212 ] * xx [ 843 ] ; pm_math_Vector3_cross_ra
( xx + 887 , xx + 890 , xx + 893 ) ; xx [ 212 ] = 0.3824707906815349 ; xx [
824 ] = ( xx [ 893 ] - xx [ 212 ] * xx [ 844 ] ) * xx [ 2 ] ; xx [ 844 ] = xx
[ 2 ] * ( xx [ 894 ] + xx [ 212 ] * xx [ 822 ] ) - xx [ 843 ] ; xx [ 822 ] =
( xx [ 895 ] - 0.01485132100429732 * xx [ 843 ] ) * xx [ 2 ] - xx [ 841 ] ;
xx [ 887 ] = xx [ 884 ] + xx [ 824 ] ; xx [ 888 ] = xx [ 885 ] + xx [ 844 ] ;
xx [ 889 ] = xx [ 886 ] + xx [ 822 ] ; xx [ 890 ] = xx [ 376 ] * xx [ 808 ] ;
xx [ 891 ] = xx [ 379 ] * xx [ 808 ] ; xx [ 892 ] = xx [ 397 ] * xx [ 808 ] ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 890 , xx + 893 ) ;
pm_math_Vector3_cross_ra ( xx + 813 , xx + 830 , xx + 890 ) ; xx [ 830 ] = xx
[ 893 ] + xx [ 890 ] - xx [ 405 ] * xx [ 211 ] ; xx [ 831 ] = xx [ 894 ] + xx
[ 891 ] - xx [ 409 ] * xx [ 211 ] ; xx [ 832 ] = xx [ 895 ] + xx [ 892 ] - xx
[ 825 ] * xx [ 211 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 830 , xx
+ 890 ) ; pm_math_Vector3_cross_ra ( xx + 845 , xx + 884 , xx + 830 ) ; xx [
884 ] = xx [ 824 ] ; xx [ 885 ] = xx [ 844 ] ; xx [ 886 ] = xx [ 822 ] ;
pm_math_Vector3_cross_ra ( xx + 449 , xx + 884 , xx + 893 ) ; xx [ 884 ] = xx
[ 890 ] + xx [ 830 ] + xx [ 893 ] ; xx [ 885 ] = xx [ 891 ] + xx [ 831 ] + xx
[ 894 ] ; xx [ 886 ] = xx [ 892 ] + xx [ 832 ] + xx [ 895 ] ; xx [ 890 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 887 ) ; xx [ 891 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 887 ) ; xx [ 892 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 887 ) ; xx [ 893 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 884 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 887 ) ) ; xx [ 894 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 884 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 887 ) ) ; xx [ 895 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 884 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 887 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 890 , 6 , 1
, xx + 884 , xx + 896 ) ; xx [ 830 ] = xx [ 959 ] * xx [ 889 ] - ( xx [ 671 ]
* xx [ 887 ] + xx [ 940 ] * xx [ 888 ] ) ; xx [ 831 ] = xx [ 933 ] * xx [ 887
] - xx [ 952 ] * xx [ 888 ] + xx [ 971 ] * xx [ 889 ] ; xx [ 832 ] = xx [ 953
] * xx [ 888 ] - xx [ 934 ] * xx [ 887 ] + xx [ 972 ] * xx [ 889 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 830 , xx + 890 ) ; xx [
822 ] = xx [ 884 ] * xx [ 3 ] + xx [ 885 ] * xx [ 196 ] + xx [ 886 ] * xx [
522 ] + xx [ 941 ] * xx [ 887 ] - xx [ 960 ] * xx [ 888 ] + xx [ 979 ] * xx [
889 ] ; pm_math_Vector3_cross_ra ( xx + 830 , xx + 651 , xx + 893 ) ; xx [
824 ] = xx [ 884 ] * xx [ 22 ] + xx [ 885 ] * xx [ 21 ] + xx [ 886 ] * xx [
23 ] + xx [ 942 ] * xx [ 887 ] + xx [ 961 ] * xx [ 888 ] - xx [ 980 ] * xx [
889 ] ; xx [ 841 ] = xx [ 884 ] * xx [ 25 ] + xx [ 885 ] * xx [ 434 ] + xx [
886 ] * xx [ 1 ] - xx [ 943 ] * xx [ 887 ] - xx [ 962 ] * xx [ 888 ] - xx [
981 ] * xx [ 889 ] ; xx [ 884 ] = xx [ 822 ] + xx [ 893 ] ; xx [ 885 ] = xx [
824 ] + xx [ 894 ] ; xx [ 886 ] = xx [ 841 ] + xx [ 895 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 884 , xx + 887 ) ; xx [
843 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 890 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 887 ) ; xx [ 844 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 890 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 887 ) ; pm_math_Vector3_cross_ra ( xx + 830 , xx + 204 , xx +
884 ) ; xx [ 887 ] = xx [ 822 ] + xx [ 884 ] ; xx [ 888 ] = xx [ 824 ] + xx [
885 ] ; xx [ 889 ] = xx [ 841 ] + xx [ 886 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 887 , xx + 884 ) ; xx [
884 ] = xx [ 64 ] * xx [ 843 ] - xx [ 844 ] * xx [ 63 ] - xx [ 81 ] * ( xx [
201 ] * xx [ 885 ] + xx [ 207 ] * xx [ 886 ] ) ; xx [ 885 ] = -
2.237793284010081e-16 ; xx [ 886 ] = 1.923188306740889e-18 ; xx [ 887 ] = -
3.886322687274291e-16 ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 885 ,
xx + 888 ) ; xx [ 885 ] = - 3.865146760306702e-8 ; xx [ 886 ] = -
5.289307885790379e-3 ; xx [ 887 ] = - 3.071851003736677e-7 ;
pm_math_Quaternion_xform_ra ( xx + 512 , xx + 885 , xx + 891 ) ; xx [ 894 ] =
2.392300359455244e-7 ; xx [ 895 ] = 3.887917980054749e-3 ; xx [ 896 ] =
2.025360777142851e-5 ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 894 , xx
+ 897 ) ; xx [ 900 ] = xx [ 891 ] - xx [ 573 ] + xx [ 569 ] - ( xx [ 897 ] +
xx [ 578 ] ) ; xx [ 901 ] = xx [ 892 ] - xx [ 574 ] + xx [ 570 ] - ( xx [ 898
] + xx [ 580 ] ) ; xx [ 902 ] = xx [ 893 ] - xx [ 575 ] + xx [ 571 ] - ( xx [
899 ] + xx [ 620 ] ) ; xx [ 569 ] = - 3.296413151655537e-7 ; xx [ 570 ] = -
7.422626627699278e-9 ; xx [ 571 ] = 1.428754025224236e-6 ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 569 , xx + 573 ) ; xx [ 569 ] =
3.296413151917778e-7 ; xx [ 570 ] = 7.422626626299722e-9 ; xx [ 571 ] =
1.428754024949564e-6 ; xx [ 891 ] = xx [ 569 ] ; xx [ 892 ] = xx [ 570 ] ; xx
[ 893 ] = - xx [ 571 ] ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 891 ,
xx + 897 ) ; xx [ 903 ] = xx [ 573 ] + xx [ 897 ] ; xx [ 904 ] = xx [ 574 ] +
xx [ 898 ] ; xx [ 905 ] = xx [ 575 ] + xx [ 899 ] ; xx [ 906 ] =
0.4931954203595264 ; xx [ 907 ] = - 0.4905194435002385 ; xx [ 908 ] = -
0.5067824161638587 ; xx [ 909 ] = - 0.509235245786982 ;
pm_math_Quaternion_compose_ra ( xx + 545 , xx + 906 , xx + 910 ) ; xx [ 573 ]
= ( xx [ 910 ] * xx [ 912 ] + xx [ 911 ] * xx [ 913 ] ) * xx [ 2 ] ; xx [ 574
] = xx [ 2 ] * ( xx [ 912 ] * xx [ 913 ] - xx [ 910 ] * xx [ 911 ] ) ; xx [
575 ] = xx [ 11 ] - ( xx [ 911 ] * xx [ 911 ] + xx [ 912 ] * xx [ 912 ] ) *
xx [ 2 ] ; xx [ 578 ] = pm_math_Vector3_dot_ra ( xx + 888 , xx + 900 ) -
pm_math_Vector3_dot_ra ( xx + 903 , xx + 573 ) ; xx [ 888 ] = xx [ 223 ] ; xx
[ 889 ] = xx [ 579 ] ; xx [ 890 ] = xx [ 623 ] ; xx [ 580 ] =
pm_math_Vector3_dot_ra ( xx + 888 , xx + 573 ) ; xx [ 888 ] = xx [ 578 ] ; xx
[ 889 ] = - xx [ 580 ] ; solveSymmetricPosDef ( xx + 560 , xx + 888 , 2 , 1 ,
xx + 903 , xx + 905 ) ; xx [ 888 ] = xx [ 296 ] * xx [ 903 ] ; xx [ 889 ] = -
( xx [ 532 ] * xx [ 903 ] ) ; xx [ 890 ] = - ( xx [ 535 ] * xx [ 903 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 888 , xx + 905 ) ; xx [ 888 ] =
xx [ 439 ] * xx [ 903 ] + xx [ 564 ] * xx [ 904 ] ; xx [ 889 ] = xx [ 565 ] *
xx [ 903 ] - xx [ 566 ] * xx [ 904 ] ; xx [ 890 ] = - ( xx [ 567 ] * xx [ 903
] + xx [ 568 ] * xx [ 904 ] ) ; pm_math_Quaternion_xform_ra ( xx + 545 , xx +
888 , xx + 908 ) ; pm_math_Vector3_cross_ra ( xx + 624 , xx + 908 , xx + 888
) ; xx [ 911 ] = xx [ 905 ] + xx [ 888 ] ; xx [ 912 ] = xx [ 906 ] + xx [ 889
] ; xx [ 913 ] = xx [ 907 ] + xx [ 890 ] ; pm_math_Quaternion_xform_ra ( xx +
489 , xx + 911 , xx + 888 ) ; pm_math_Quaternion_xform_ra ( xx + 489 , xx +
908 , xx + 905 ) ; pm_math_Vector3_cross_ra ( xx + 648 , xx + 905 , xx + 908
) ; xx [ 620 ] = xx [ 888 ] + xx [ 908 ] ; xx [ 911 ] = xx [ 889 ] + xx [ 909
] ; xx [ 888 ] = xx [ 890 ] + xx [ 910 ] ; xx [ 908 ] = xx [ 620 ] ; xx [ 909
] = xx [ 911 ] ; xx [ 910 ] = xx [ 888 ] ; xx [ 889 ] = - (
pm_math_Vector3_dot_ra ( xx + 684 , xx + 908 ) + pm_math_Vector3_dot_ra ( xx
+ 700 , xx + 905 ) ) ; xx [ 890 ] = - pm_math_Vector3_dot_ra ( xx + 684 , xx
+ 905 ) ; solveSymmetricPosDef ( xx + 664 , xx + 889 , 2 , 1 , xx + 908 , xx
+ 912 ) ; xx [ 912 ] = xx [ 905 ] + xx [ 706 ] * xx [ 908 ] + xx [ 660 ] * xx
[ 909 ] ; xx [ 913 ] = xx [ 906 ] + xx [ 523 ] * xx [ 908 ] + xx [ 661 ] * xx
[ 909 ] ; xx [ 914 ] = xx [ 907 ] + xx [ 524 ] * xx [ 908 ] + xx [ 909 ] * xx
[ 662 ] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 912 , xx + 905 ) ; xx
[ 912 ] = xx [ 620 ] + xx [ 525 ] * xx [ 908 ] + xx [ 657 ] * xx [ 909 ] ; xx
[ 913 ] = xx [ 911 ] + xx [ 526 ] * xx [ 908 ] + xx [ 658 ] * xx [ 909 ] ; xx
[ 914 ] = xx [ 888 ] + xx [ 519 ] * xx [ 908 ] + xx [ 909 ] * xx [ 659 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 912 , xx + 888 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 905 , xx + 910 ) ; xx [ 913 ] = xx
[ 888 ] + xx [ 910 ] ; xx [ 914 ] = xx [ 889 ] + xx [ 911 ] ; xx [ 915 ] = xx
[ 890 ] + xx [ 912 ] ; xx [ 916 ] = - pm_math_Vector3_dot_ra ( xx + 26 , xx +
905 ) ; xx [ 917 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx + 905 ) ; xx [
918 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 905 ) ; xx [ 919 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 913 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 905 ) ) ; xx [ 920 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 913 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 905 ) ) ; xx [ 921 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 913 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 905 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 916 , 6 , 1
, xx + 910 , xx + 990 ) ; xx [ 888 ] = xx [ 959 ] * xx [ 915 ] - ( xx [ 671 ]
* xx [ 913 ] + xx [ 940 ] * xx [ 914 ] ) ; xx [ 889 ] = xx [ 933 ] * xx [ 913
] - xx [ 952 ] * xx [ 914 ] + xx [ 971 ] * xx [ 915 ] ; xx [ 890 ] = xx [ 953
] * xx [ 914 ] - xx [ 934 ] * xx [ 913 ] + xx [ 972 ] * xx [ 915 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 888 , xx + 905 ) ; xx [
620 ] = xx [ 910 ] * xx [ 3 ] + xx [ 911 ] * xx [ 196 ] + xx [ 912 ] * xx [
522 ] + xx [ 941 ] * xx [ 913 ] - xx [ 960 ] * xx [ 914 ] + xx [ 979 ] * xx [
915 ] ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 651 , xx + 916 ) ; xx [
919 ] = xx [ 910 ] * xx [ 22 ] + xx [ 911 ] * xx [ 21 ] + xx [ 912 ] * xx [
23 ] + xx [ 942 ] * xx [ 913 ] + xx [ 961 ] * xx [ 914 ] - xx [ 980 ] * xx [
915 ] ; xx [ 920 ] = xx [ 910 ] * xx [ 25 ] + xx [ 911 ] * xx [ 434 ] + xx [
912 ] * xx [ 1 ] - xx [ 943 ] * xx [ 913 ] - xx [ 962 ] * xx [ 914 ] - xx [
981 ] * xx [ 915 ] ; xx [ 910 ] = xx [ 620 ] + xx [ 916 ] ; xx [ 911 ] = xx [
919 ] + xx [ 917 ] ; xx [ 912 ] = xx [ 920 ] + xx [ 918 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 910 , xx + 913 ) ; xx [
910 ] = xx [ 908 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx + 905 ) +
pm_math_Vector3_dot_ra ( xx + 271 , xx + 913 ) ) ; xx [ 908 ] = xx [ 909 ] -
( pm_math_Vector3_dot_ra ( xx + 367 , xx + 905 ) + pm_math_Vector3_dot_ra (
xx + 370 , xx + 913 ) ) ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 204 , xx
+ 916 ) ; xx [ 921 ] = xx [ 620 ] + xx [ 916 ] ; xx [ 922 ] = xx [ 919 ] + xx
[ 917 ] ; xx [ 923 ] = xx [ 920 ] + xx [ 918 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 921 , xx + 916 ) ; xx [
909 ] = xx [ 63 ] * xx [ 910 ] - xx [ 64 ] * xx [ 908 ] - xx [ 81 ] * ( xx [
201 ] * xx [ 917 ] + xx [ 207 ] * xx [ 918 ] ) ; xx [ 916 ] = -
1.334869714764153e-15 ; xx [ 917 ] = 5.850668324922266e-18 ; xx [ 918 ] = -
1.221407957413545e-15 ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 916 ,
xx + 921 ) ; xx [ 916 ] = - 3.975614476060287e-8 ; xx [ 917 ] =
2.710692114195085e-3 ; xx [ 918 ] = - 2.919897872271535e-7 ;
pm_math_Quaternion_xform_ra ( xx + 262 , xx + 916 , xx + 947 ) ; xx [ 966 ] =
2.392300359480238e-7 ; xx [ 967 ] = 3.887917980054739e-3 ; xx [ 968 ] =
2.025360777142854e-5 ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 966 , xx
+ 985 ) ; xx [ 990 ] = xx [ 947 ] - xx [ 328 ] + xx [ 20 ] - ( xx [ 985 ] +
xx [ 202 ] ) ; xx [ 991 ] = xx [ 948 ] - xx [ 329 ] + xx [ 611 ] - ( xx [ 986
] + xx [ 615 ] ) ; xx [ 992 ] = xx [ 949 ] - xx [ 330 ] + xx [ 617 ] - ( xx [
987 ] + xx [ 663 ] ) ; xx [ 328 ] = - 3.296413151708188e-7 ; xx [ 329 ] = -
7.422626627666806e-9 ; xx [ 330 ] = 1.4287540252181e-6 ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 328 , xx + 947 ) ; xx [ 20 ] =
3.296413155003147e-7 ; xx [ 202 ] = 7.422626627203207e-9 ; xx [ 328 ] =
1.428754025107962e-6 ; xx [ 985 ] = xx [ 20 ] ; xx [ 986 ] = xx [ 202 ] ; xx
[ 987 ] = - xx [ 328 ] ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 985 ,
xx + 993 ) ; xx [ 1000 ] = xx [ 947 ] + xx [ 993 ] ; xx [ 1001 ] = xx [ 948 ]
+ xx [ 994 ] ; xx [ 1002 ] = xx [ 949 ] + xx [ 995 ] ; xx [ 1003 ] =
0.493195420359526 ; xx [ 1004 ] = - 0.4905194435002388 ; xx [ 1005 ] = -
0.5067824161638583 ; xx [ 1006 ] = - 0.5092352457869823 ;
pm_math_Quaternion_compose_ra ( xx + 299 , xx + 1003 , xx + 1007 ) ; xx [ 947
] = ( xx [ 1007 ] * xx [ 1009 ] + xx [ 1008 ] * xx [ 1010 ] ) * xx [ 2 ] ; xx
[ 948 ] = xx [ 2 ] * ( xx [ 1009 ] * xx [ 1010 ] - xx [ 1007 ] * xx [ 1008 ]
) ; xx [ 949 ] = xx [ 11 ] - ( xx [ 1008 ] * xx [ 1008 ] + xx [ 1009 ] * xx [
1009 ] ) * xx [ 2 ] ; xx [ 329 ] = pm_math_Vector3_dot_ra ( xx + 921 , xx +
990 ) - pm_math_Vector3_dot_ra ( xx + 1000 , xx + 947 ) ; xx [ 921 ] = xx [
24 ] ; xx [ 922 ] = xx [ 614 ] ; xx [ 923 ] = xx [ 619 ] ; xx [ 330 ] =
pm_math_Vector3_dot_ra ( xx + 921 , xx + 947 ) ; xx [ 911 ] = xx [ 329 ] ; xx
[ 912 ] = - xx [ 330 ] ; solveSymmetricPosDef ( xx + 312 , xx + 911 , 2 , 1 ,
xx + 921 , xx + 977 ) ; xx [ 1000 ] = xx [ 284 ] * xx [ 921 ] + xx [ 320 ] *
xx [ 922 ] ; xx [ 1001 ] = xx [ 321 ] * xx [ 921 ] - xx [ 322 ] * xx [ 922 ]
; xx [ 1002 ] = - ( xx [ 323 ] * xx [ 921 ] + xx [ 324 ] * xx [ 922 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 1000 , xx + 1003 ) ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 1003 , xx + 1000 ) ; xx [ 611 ]
= ( xx [ 413 ] * xx [ 1000 ] + xx [ 427 ] * xx [ 1001 ] ) / xx [ 432 ] ; xx [
1006 ] = xx [ 1000 ] - xx [ 428 ] * xx [ 611 ] ; xx [ 1007 ] = xx [ 1001 ] -
xx [ 431 ] * xx [ 611 ] ; xx [ 1008 ] = xx [ 1002 ] - xx [ 438 ] * xx [ 611 ]
; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1006 , xx + 1009 ) ; xx [
1006 ] = xx [ 316 ] * xx [ 921 ] ; xx [ 1007 ] = - ( xx [ 318 ] * xx [ 921 ]
) ; xx [ 1008 ] = - ( xx [ 319 ] * xx [ 921 ] ) ; pm_math_Quaternion_xform_ra
( xx + 299 , xx + 1006 , xx + 1012 ) ; pm_math_Vector3_cross_ra ( xx + 736 ,
xx + 1003 , xx + 1006 ) ; xx [ 1003 ] = xx [ 1012 ] + xx [ 1006 ] ; xx [ 1004
] = xx [ 1013 ] + xx [ 1007 ] ; xx [ 1005 ] = xx [ 1014 ] + xx [ 1008 ] ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 1003 , xx + 1006 ) ;
pm_math_Vector3_cross_ra ( xx + 784 , xx + 1000 , xx + 1003 ) ; xx [ 1000 ] =
xx [ 1006 ] + xx [ 1003 ] - xx [ 405 ] * xx [ 611 ] ; xx [ 1001 ] = xx [ 1007
] + xx [ 1004 ] - xx [ 409 ] * xx [ 611 ] ; xx [ 1002 ] = xx [ 1008 ] + xx [
1005 ] - xx [ 825 ] * xx [ 611 ] ; pm_math_Quaternion_xform_ra ( xx + 388 ,
xx + 1000 , xx + 1003 ) ; pm_math_Vector3_cross_ra ( xx + 845 , xx + 1009 ,
xx + 1000 ) ; xx [ 1006 ] = xx [ 1003 ] + xx [ 1000 ] ; xx [ 1007 ] = xx [
1004 ] + xx [ 1001 ] ; xx [ 1008 ] = xx [ 1005 ] + xx [ 1002 ] ; xx [ 1000 ]
= - pm_math_Vector3_dot_ra ( xx + 26 , xx + 1009 ) ; xx [ 1001 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 1009 ) ; xx [ 1002 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 1009 ) ; xx [ 1003 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 1006 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 1009 ) ) ; xx [ 1004 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 1006 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 1009 ) ) ; xx [ 1005 ]
= - ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 1006 ) +
pm_math_Vector3_dot_ra ( xx + 982 , xx + 1009 ) ) ; solveSymmetricPosDef ( xx
+ 848 , xx + 1000 , 6 , 1 , xx + 1006 , xx + 1012 ) ; xx [ 1000 ] = xx [ 959
] * xx [ 1011 ] - ( xx [ 671 ] * xx [ 1009 ] + xx [ 940 ] * xx [ 1010 ] ) ;
xx [ 1001 ] = xx [ 933 ] * xx [ 1009 ] - xx [ 952 ] * xx [ 1010 ] + xx [ 971
] * xx [ 1011 ] ; xx [ 1002 ] = xx [ 953 ] * xx [ 1010 ] - xx [ 934 ] * xx [
1009 ] + xx [ 972 ] * xx [ 1011 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
464 , xx + 1000 , xx + 1003 ) ; xx [ 615 ] = xx [ 1006 ] * xx [ 3 ] + xx [
1007 ] * xx [ 196 ] + xx [ 1008 ] * xx [ 522 ] + xx [ 941 ] * xx [ 1009 ] -
xx [ 960 ] * xx [ 1010 ] + xx [ 979 ] * xx [ 1011 ] ;
pm_math_Vector3_cross_ra ( xx + 1000 , xx + 651 , xx + 1012 ) ; xx [ 617 ] =
xx [ 1006 ] * xx [ 22 ] + xx [ 1007 ] * xx [ 21 ] + xx [ 1008 ] * xx [ 23 ] +
xx [ 942 ] * xx [ 1009 ] + xx [ 961 ] * xx [ 1010 ] - xx [ 980 ] * xx [ 1011
] ; xx [ 663 ] = xx [ 1006 ] * xx [ 25 ] + xx [ 1007 ] * xx [ 434 ] + xx [
1008 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1009 ] - xx [ 962 ] * xx [ 1010 ] - xx
[ 981 ] * xx [ 1011 ] ; xx [ 1006 ] = xx [ 615 ] + xx [ 1012 ] ; xx [ 1007 ]
= xx [ 617 ] + xx [ 1013 ] ; xx [ 1008 ] = xx [ 663 ] + xx [ 1014 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1006 , xx + 1009 ) ; xx
[ 911 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 1003 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1009 ) ; xx [ 912 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 1003 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 1009 ) ; pm_math_Vector3_cross_ra ( xx + 1000 , xx + 204 , xx +
1006 ) ; xx [ 1012 ] = xx [ 615 ] + xx [ 1006 ] ; xx [ 1013 ] = xx [ 617 ] +
xx [ 1007 ] ; xx [ 1014 ] = xx [ 663 ] + xx [ 1008 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1012 , xx + 1006 ) ; xx
[ 923 ] = xx [ 64 ] * xx [ 911 ] - xx [ 912 ] * xx [ 63 ] - xx [ 81 ] * ( xx
[ 201 ] * xx [ 1007 ] + xx [ 207 ] * xx [ 1008 ] ) ;
pm_math_Quaternion_compose_ra ( xx + 489 , xx + 545 , xx + 1012 ) ;
pm_math_Quaternion_compose_ra ( xx + 464 , xx + 1012 , xx + 1016 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 1012 , xx + 684 , xx + 1006 ) ; xx
[ 1012 ] = 0.8494245932430124 ; xx [ 1013 ] = - 2.444376317833358e-3 ; xx [
1014 ] = 0.5277043541779427 ; pm_math_Vector3_cross_ra ( xx + 1006 , xx +
1012 , xx + 1020 ) ; pm_math_Quaternion_xform_ra ( xx + 1016 , xx + 1020 , xx
+ 1006 ) ; xx [ 1015 ] = 0.704416789386705 ; xx [ 1016 ] = -
0.08618526586337713 ; xx [ 1017 ] = 0.704534659742291 ; xx [ 939 ] =
pm_math_Vector3_dot_ra ( xx + 1006 , xx + 1015 ) ;
pm_math_Quaternion_compose_ra ( xx + 629 , xx + 545 , xx + 1018 ) ; xx [ 1022
] = - 0.5277099267453734 ; xx [ 1023 ] = - 4.514343961260733e-3 ; xx [ 1024 ]
= 0.8494126523151114 ; pm_math_Quaternion_xform_ra ( xx + 1018 , xx + 1022 ,
xx + 1025 ) ; xx [ 958 ] = pm_math_Vector3_dot_ra ( xx + 1025 , xx + 1015 ) ;
xx [ 977 ] = xx [ 958 ] ; xx [ 978 ] = xx [ 317 ] ; solveSymmetricPosDef ( xx
+ 560 , xx + 977 , 2 , 1 , xx + 1022 , xx + 1028 ) ; xx [ 1028 ] = xx [ 296 ]
* xx [ 1022 ] ; xx [ 1029 ] = - ( xx [ 532 ] * xx [ 1022 ] ) ; xx [ 1030 ] =
- ( xx [ 535 ] * xx [ 1022 ] ) ; pm_math_Quaternion_xform_ra ( xx + 545 , xx
+ 1028 , xx + 1031 ) ; xx [ 1028 ] = xx [ 439 ] * xx [ 1022 ] + xx [ 564 ] *
xx [ 1023 ] ; xx [ 1029 ] = xx [ 565 ] * xx [ 1022 ] - xx [ 566 ] * xx [ 1023
] ; xx [ 1030 ] = - ( xx [ 567 ] * xx [ 1022 ] + xx [ 568 ] * xx [ 1023 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 1028 , xx + 1034 ) ;
pm_math_Vector3_cross_ra ( xx + 624 , xx + 1034 , xx + 1028 ) ; xx [ 1037 ] =
xx [ 1031 ] + xx [ 1028 ] ; xx [ 1038 ] = xx [ 1032 ] + xx [ 1029 ] ; xx [
1039 ] = xx [ 1033 ] + xx [ 1030 ] ; pm_math_Quaternion_xform_ra ( xx + 489 ,
xx + 1037 , xx + 1028 ) ; pm_math_Quaternion_xform_ra ( xx + 489 , xx + 1034
, xx + 1031 ) ; pm_math_Vector3_cross_ra ( xx + 648 , xx + 1031 , xx + 1034 )
; xx [ 977 ] = xx [ 1028 ] + xx [ 1034 ] ; xx [ 978 ] = xx [ 1029 ] + xx [
1035 ] ; xx [ 997 ] = xx [ 1030 ] + xx [ 1036 ] ; xx [ 1028 ] = xx [ 977 ] ;
xx [ 1029 ] = xx [ 978 ] ; xx [ 1030 ] = xx [ 997 ] ; xx [ 1034 ] = xx [ 939
] - ( pm_math_Vector3_dot_ra ( xx + 684 , xx + 1028 ) +
pm_math_Vector3_dot_ra ( xx + 700 , xx + 1031 ) ) ; xx [ 1035 ] = -
pm_math_Vector3_dot_ra ( xx + 684 , xx + 1031 ) ; solveSymmetricPosDef ( xx +
664 , xx + 1034 , 2 , 1 , xx + 1028 , xx + 1036 ) ; xx [ 1034 ] = xx [ 1031 ]
+ xx [ 706 ] * xx [ 1028 ] + xx [ 660 ] * xx [ 1029 ] ; xx [ 1035 ] = xx [
1032 ] + xx [ 523 ] * xx [ 1028 ] + xx [ 661 ] * xx [ 1029 ] ; xx [ 1036 ] =
xx [ 1033 ] + xx [ 524 ] * xx [ 1028 ] + xx [ 1029 ] * xx [ 662 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1034 , xx + 1030 ) ; xx [ 1033
] = xx [ 977 ] + xx [ 525 ] * xx [ 1028 ] + xx [ 657 ] * xx [ 1029 ] ; xx [
1034 ] = xx [ 978 ] + xx [ 526 ] * xx [ 1028 ] + xx [ 658 ] * xx [ 1029 ] ;
xx [ 1035 ] = xx [ 997 ] + xx [ 519 ] * xx [ 1028 ] + xx [ 1029 ] * xx [ 659
] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1033 , xx + 1036 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 1030 , xx + 1033 ) ; xx [ 1039 ] =
xx [ 1036 ] + xx [ 1033 ] ; xx [ 1040 ] = xx [ 1037 ] + xx [ 1034 ] ; xx [
1041 ] = xx [ 1038 ] + xx [ 1035 ] ; xx [ 1033 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 1030 ) ; xx [ 1034 ] = - pm_math_Vector3_dot_ra ( xx + 672 ,
xx + 1030 ) ; xx [ 1035 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 1030 )
; xx [ 1036 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 1039 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 1030 ) ) ; xx [ 1037 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 1039 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 1030 ) ) ; xx [ 1038 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 1039 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 1030 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 1033 , 6 , 1 , xx + 1039 , xx + 1045 )
; xx [ 1030 ] = xx [ 959 ] * xx [ 1044 ] - ( xx [ 671 ] * xx [ 1042 ] + xx [
940 ] * xx [ 1043 ] ) ; xx [ 1031 ] = xx [ 933 ] * xx [ 1042 ] - xx [ 952 ] *
xx [ 1043 ] + xx [ 971 ] * xx [ 1044 ] ; xx [ 1032 ] = xx [ 953 ] * xx [ 1043
] - xx [ 934 ] * xx [ 1042 ] + xx [ 972 ] * xx [ 1044 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1030 , xx + 1033 ) ; xx
[ 977 ] = xx [ 1039 ] * xx [ 3 ] + xx [ 1040 ] * xx [ 196 ] + xx [ 1041 ] *
xx [ 522 ] + xx [ 941 ] * xx [ 1042 ] - xx [ 960 ] * xx [ 1043 ] + xx [ 979 ]
* xx [ 1044 ] ; pm_math_Vector3_cross_ra ( xx + 1030 , xx + 651 , xx + 1036 )
; xx [ 978 ] = xx [ 1039 ] * xx [ 22 ] + xx [ 1040 ] * xx [ 21 ] + xx [ 1041
] * xx [ 23 ] + xx [ 942 ] * xx [ 1042 ] + xx [ 961 ] * xx [ 1043 ] - xx [
980 ] * xx [ 1044 ] ; xx [ 997 ] = xx [ 1039 ] * xx [ 25 ] + xx [ 1040 ] * xx
[ 434 ] + xx [ 1041 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1042 ] - xx [ 962 ] * xx
[ 1043 ] - xx [ 981 ] * xx [ 1044 ] ; xx [ 1039 ] = xx [ 977 ] + xx [ 1036 ]
; xx [ 1040 ] = xx [ 978 ] + xx [ 1037 ] ; xx [ 1041 ] = xx [ 997 ] + xx [
1038 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1039 , xx +
1036 ) ; xx [ 1024 ] = xx [ 1028 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx
+ 1033 ) + pm_math_Vector3_dot_ra ( xx + 271 , xx + 1036 ) ) ; xx [ 1028 ] =
xx [ 1029 ] - ( pm_math_Vector3_dot_ra ( xx + 367 , xx + 1033 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1036 ) ) ; pm_math_Vector3_cross_ra
( xx + 1030 , xx + 204 , xx + 1039 ) ; xx [ 1042 ] = xx [ 977 ] + xx [ 1039 ]
; xx [ 1043 ] = xx [ 978 ] + xx [ 1040 ] ; xx [ 1044 ] = xx [ 997 ] + xx [
1041 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1042 , xx +
1039 ) ; xx [ 1029 ] = xx [ 63 ] * xx [ 1024 ] - xx [ 64 ] * xx [ 1028 ] - xx
[ 81 ] * ( xx [ 201 ] * xx [ 1040 ] + xx [ 207 ] * xx [ 1041 ] ) ; xx [ 1039
] = - 0.05435570616770888 ; xx [ 1040 ] = - 0.9962360149712526 ; xx [ 1041 ]
= - 0.06752229025448532 ; xx [ 1042 ] = pm_math_Vector3_dot_ra ( xx + 1006 ,
xx + 1039 ) ; xx [ 1006 ] = pm_math_Vector3_dot_ra ( xx + 1025 , xx + 1039 )
; xx [ 1007 ] = xx [ 1006 ] ; xx [ 1008 ] = xx [ 317 ] ; solveSymmetricPosDef
( xx + 560 , xx + 1007 , 2 , 1 , xx + 1025 , xx + 1043 ) ; xx [ 1043 ] = xx [
296 ] * xx [ 1025 ] ; xx [ 1044 ] = - ( xx [ 532 ] * xx [ 1025 ] ) ; xx [
1045 ] = - ( xx [ 535 ] * xx [ 1025 ] ) ; pm_math_Quaternion_xform_ra ( xx +
545 , xx + 1043 , xx + 1046 ) ; xx [ 1043 ] = xx [ 439 ] * xx [ 1025 ] + xx [
564 ] * xx [ 1026 ] ; xx [ 1044 ] = xx [ 565 ] * xx [ 1025 ] - xx [ 566 ] *
xx [ 1026 ] ; xx [ 1045 ] = - ( xx [ 567 ] * xx [ 1025 ] + xx [ 568 ] * xx [
1026 ] ) ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 1043 , xx + 1049 ) ;
pm_math_Vector3_cross_ra ( xx + 624 , xx + 1049 , xx + 1043 ) ; xx [ 1052 ] =
xx [ 1046 ] + xx [ 1043 ] ; xx [ 1053 ] = xx [ 1047 ] + xx [ 1044 ] ; xx [
1054 ] = xx [ 1048 ] + xx [ 1045 ] ; pm_math_Quaternion_xform_ra ( xx + 489 ,
xx + 1052 , xx + 1043 ) ; pm_math_Quaternion_xform_ra ( xx + 489 , xx + 1049
, xx + 1046 ) ; pm_math_Vector3_cross_ra ( xx + 648 , xx + 1046 , xx + 1049 )
; xx [ 1007 ] = xx [ 1043 ] + xx [ 1049 ] ; xx [ 1008 ] = xx [ 1044 ] + xx [
1050 ] ; xx [ 1027 ] = xx [ 1045 ] + xx [ 1051 ] ; xx [ 1043 ] = xx [ 1007 ]
; xx [ 1044 ] = xx [ 1008 ] ; xx [ 1045 ] = xx [ 1027 ] ; xx [ 1049 ] = xx [
1042 ] - ( pm_math_Vector3_dot_ra ( xx + 684 , xx + 1043 ) +
pm_math_Vector3_dot_ra ( xx + 700 , xx + 1046 ) ) ; xx [ 1050 ] = -
pm_math_Vector3_dot_ra ( xx + 684 , xx + 1046 ) ; solveSymmetricPosDef ( xx +
664 , xx + 1049 , 2 , 1 , xx + 1043 , xx + 1051 ) ; xx [ 1049 ] = xx [ 1046 ]
+ xx [ 706 ] * xx [ 1043 ] + xx [ 660 ] * xx [ 1044 ] ; xx [ 1050 ] = xx [
1047 ] + xx [ 523 ] * xx [ 1043 ] + xx [ 661 ] * xx [ 1044 ] ; xx [ 1051 ] =
xx [ 1048 ] + xx [ 524 ] * xx [ 1043 ] + xx [ 1044 ] * xx [ 662 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1049 , xx + 1045 ) ; xx [ 1048
] = xx [ 1007 ] + xx [ 525 ] * xx [ 1043 ] + xx [ 657 ] * xx [ 1044 ] ; xx [
1049 ] = xx [ 1008 ] + xx [ 526 ] * xx [ 1043 ] + xx [ 658 ] * xx [ 1044 ] ;
xx [ 1050 ] = xx [ 1027 ] + xx [ 519 ] * xx [ 1043 ] + xx [ 1044 ] * xx [ 659
] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1048 , xx + 1051 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 1045 , xx + 1048 ) ; xx [ 1054 ] =
xx [ 1051 ] + xx [ 1048 ] ; xx [ 1055 ] = xx [ 1052 ] + xx [ 1049 ] ; xx [
1056 ] = xx [ 1053 ] + xx [ 1050 ] ; xx [ 1048 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 1045 ) ; xx [ 1049 ] = - pm_math_Vector3_dot_ra ( xx + 672 ,
xx + 1045 ) ; xx [ 1050 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 1045 )
; xx [ 1051 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 1054 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 1045 ) ) ; xx [ 1052 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 1054 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 1045 ) ) ; xx [ 1053 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 1054 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 1045 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 1048 , 6 , 1 , xx + 1054 , xx + 1060 )
; xx [ 1045 ] = xx [ 959 ] * xx [ 1059 ] - ( xx [ 671 ] * xx [ 1057 ] + xx [
940 ] * xx [ 1058 ] ) ; xx [ 1046 ] = xx [ 933 ] * xx [ 1057 ] - xx [ 952 ] *
xx [ 1058 ] + xx [ 971 ] * xx [ 1059 ] ; xx [ 1047 ] = xx [ 953 ] * xx [ 1058
] - xx [ 934 ] * xx [ 1057 ] + xx [ 972 ] * xx [ 1059 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1045 , xx + 1048 ) ; xx
[ 1007 ] = xx [ 1054 ] * xx [ 3 ] + xx [ 1055 ] * xx [ 196 ] + xx [ 1056 ] *
xx [ 522 ] + xx [ 941 ] * xx [ 1057 ] - xx [ 960 ] * xx [ 1058 ] + xx [ 979 ]
* xx [ 1059 ] ; pm_math_Vector3_cross_ra ( xx + 1045 , xx + 651 , xx + 1051 )
; xx [ 1008 ] = xx [ 1054 ] * xx [ 22 ] + xx [ 1055 ] * xx [ 21 ] + xx [ 1056
] * xx [ 23 ] + xx [ 942 ] * xx [ 1057 ] + xx [ 961 ] * xx [ 1058 ] - xx [
980 ] * xx [ 1059 ] ; xx [ 1027 ] = xx [ 1054 ] * xx [ 25 ] + xx [ 1055 ] *
xx [ 434 ] + xx [ 1056 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1057 ] - xx [ 962 ] *
xx [ 1058 ] - xx [ 981 ] * xx [ 1059 ] ; xx [ 1054 ] = xx [ 1007 ] + xx [
1051 ] ; xx [ 1055 ] = xx [ 1008 ] + xx [ 1052 ] ; xx [ 1056 ] = xx [ 1027 ]
+ xx [ 1053 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1054 ,
xx + 1051 ) ; xx [ 1054 ] = xx [ 1043 ] - ( pm_math_Vector3_dot_ra ( xx + 172
, xx + 1048 ) + pm_math_Vector3_dot_ra ( xx + 271 , xx + 1051 ) ) ; xx [ 1043
] = xx [ 1044 ] - ( pm_math_Vector3_dot_ra ( xx + 367 , xx + 1048 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1051 ) ) ; pm_math_Vector3_cross_ra
( xx + 1045 , xx + 204 , xx + 1055 ) ; xx [ 1058 ] = xx [ 1007 ] + xx [ 1055
] ; xx [ 1059 ] = xx [ 1008 ] + xx [ 1056 ] ; xx [ 1060 ] = xx [ 1027 ] + xx
[ 1057 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1058 , xx +
1055 ) ; xx [ 1044 ] = xx [ 63 ] * xx [ 1054 ] - xx [ 64 ] * xx [ 1043 ] - xx
[ 81 ] * ( xx [ 201 ] * xx [ 1056 ] + xx [ 207 ] * xx [ 1057 ] ) ; xx [ 1055
] = 0.9999999602050232 ; pm_math_Quaternion_compose_ra ( xx + 236 , xx + 299
, xx + 1056 ) ; xx [ 1060 ] = - 0.9994782397203995 ; xx [ 1061 ] = -
1.402799929741683e-4 ; xx [ 1062 ] = - 0.03229905024820418 ;
pm_math_Quaternion_xform_ra ( xx + 1056 , xx + 1060 , xx + 1063 ) ; xx [ 1060
] = 2.821169119818891e-4 ; xx [ 1061 ] = xx [ 1055 ] * xx [ 1063 ] + xx [
1060 ] * xx [ 1064 ] ; xx [ 1065 ] = xx [ 1061 ] ; xx [ 1066 ] = xx [ 317 ] ;
solveSymmetricPosDef ( xx + 312 , xx + 1065 , 2 , 1 , xx + 1067 , xx + 1069 )
; xx [ 1069 ] = xx [ 284 ] * xx [ 1067 ] + xx [ 320 ] * xx [ 1068 ] ; xx [
1070 ] = xx [ 321 ] * xx [ 1067 ] - xx [ 322 ] * xx [ 1068 ] ; xx [ 1071 ] =
- ( xx [ 323 ] * xx [ 1067 ] + xx [ 324 ] * xx [ 1068 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 1069 , xx + 1072 ) ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 1072 , xx + 1069 ) ; xx [ 1062
] = ( xx [ 413 ] * xx [ 1069 ] + xx [ 427 ] * xx [ 1070 ] ) / xx [ 432 ] ; xx
[ 1075 ] = xx [ 1069 ] - xx [ 428 ] * xx [ 1062 ] ; xx [ 1076 ] = xx [ 1070 ]
- xx [ 431 ] * xx [ 1062 ] ; xx [ 1077 ] = xx [ 1071 ] - xx [ 438 ] * xx [
1062 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1075 , xx + 1078 ) ;
xx [ 1075 ] = xx [ 316 ] * xx [ 1067 ] ; xx [ 1076 ] = - ( xx [ 318 ] * xx [
1067 ] ) ; xx [ 1077 ] = - ( xx [ 319 ] * xx [ 1067 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 1075 , xx + 1081 ) ;
pm_math_Vector3_cross_ra ( xx + 736 , xx + 1072 , xx + 1075 ) ; xx [ 1072 ] =
xx [ 1081 ] + xx [ 1075 ] ; xx [ 1073 ] = xx [ 1082 ] + xx [ 1076 ] ; xx [
1074 ] = xx [ 1083 ] + xx [ 1077 ] ; pm_math_Quaternion_xform_ra ( xx + 236 ,
xx + 1072 , xx + 1075 ) ; pm_math_Vector3_cross_ra ( xx + 784 , xx + 1069 ,
xx + 1072 ) ; xx [ 1069 ] = xx [ 1075 ] + xx [ 1072 ] - xx [ 405 ] * xx [
1062 ] ; xx [ 1070 ] = xx [ 1076 ] + xx [ 1073 ] - xx [ 409 ] * xx [ 1062 ] ;
xx [ 1071 ] = xx [ 1077 ] + xx [ 1074 ] - xx [ 825 ] * xx [ 1062 ] ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1069 , xx + 1072 ) ;
pm_math_Vector3_cross_ra ( xx + 845 , xx + 1078 , xx + 1069 ) ; xx [ 1075 ] =
xx [ 1072 ] + xx [ 1069 ] ; xx [ 1076 ] = xx [ 1073 ] + xx [ 1070 ] ; xx [
1077 ] = xx [ 1074 ] + xx [ 1071 ] ; xx [ 1069 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 1078 ) ; xx [ 1070 ] = - pm_math_Vector3_dot_ra ( xx + 672 ,
xx + 1078 ) ; xx [ 1071 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 1078 )
; xx [ 1072 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 1075 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 1078 ) ) ; xx [ 1073 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 1075 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 1078 ) ) ; xx [ 1074 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 1075 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 1078 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 1069 , 6 , 1 , xx + 1075 , xx + 1081 )
; xx [ 1069 ] = xx [ 959 ] * xx [ 1080 ] - ( xx [ 671 ] * xx [ 1078 ] + xx [
940 ] * xx [ 1079 ] ) ; xx [ 1070 ] = xx [ 933 ] * xx [ 1078 ] - xx [ 952 ] *
xx [ 1079 ] + xx [ 971 ] * xx [ 1080 ] ; xx [ 1071 ] = xx [ 953 ] * xx [ 1079
] - xx [ 934 ] * xx [ 1078 ] + xx [ 972 ] * xx [ 1080 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1069 , xx + 1072 ) ; xx
[ 1065 ] = xx [ 1075 ] * xx [ 3 ] + xx [ 1076 ] * xx [ 196 ] + xx [ 1077 ] *
xx [ 522 ] + xx [ 941 ] * xx [ 1078 ] - xx [ 960 ] * xx [ 1079 ] + xx [ 979 ]
* xx [ 1080 ] ; pm_math_Vector3_cross_ra ( xx + 1069 , xx + 651 , xx + 1081 )
; xx [ 1066 ] = xx [ 1075 ] * xx [ 22 ] + xx [ 1076 ] * xx [ 21 ] + xx [ 1077
] * xx [ 23 ] + xx [ 942 ] * xx [ 1078 ] + xx [ 961 ] * xx [ 1079 ] - xx [
980 ] * xx [ 1080 ] ; xx [ 1084 ] = xx [ 1075 ] * xx [ 25 ] + xx [ 1076 ] *
xx [ 434 ] + xx [ 1077 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1078 ] - xx [ 962 ] *
xx [ 1079 ] - xx [ 981 ] * xx [ 1080 ] ; xx [ 1075 ] = xx [ 1065 ] + xx [
1081 ] ; xx [ 1076 ] = xx [ 1066 ] + xx [ 1082 ] ; xx [ 1077 ] = xx [ 1084 ]
+ xx [ 1083 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1075 ,
xx + 1078 ) ; xx [ 1075 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 1072 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1078 ) ; xx [ 1076 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 1072 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 1078 ) ; pm_math_Vector3_cross_ra ( xx + 1069 , xx + 204 , xx +
1081 ) ; xx [ 1085 ] = xx [ 1065 ] + xx [ 1081 ] ; xx [ 1086 ] = xx [ 1066 ]
+ xx [ 1082 ] ; xx [ 1087 ] = xx [ 1084 ] + xx [ 1083 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1085 , xx + 1081 ) ; xx
[ 1077 ] = xx [ 64 ] * xx [ 1075 ] - xx [ 1076 ] * xx [ 63 ] - xx [ 81 ] * (
xx [ 201 ] * xx [ 1082 ] + xx [ 207 ] * xx [ 1083 ] ) ; xx [ 1081 ] = xx [
1055 ] * xx [ 1064 ] - xx [ 1060 ] * xx [ 1063 ] ; xx [ 1063 ] = xx [ 1081 ]
; xx [ 1064 ] = xx [ 317 ] ; solveSymmetricPosDef ( xx + 312 , xx + 1063 , 2
, 1 , xx + 1082 , xx + 1085 ) ; xx [ 1085 ] = xx [ 284 ] * xx [ 1082 ] + xx [
320 ] * xx [ 1083 ] ; xx [ 1086 ] = xx [ 321 ] * xx [ 1082 ] - xx [ 322 ] *
xx [ 1083 ] ; xx [ 1087 ] = - ( xx [ 323 ] * xx [ 1082 ] + xx [ 324 ] * xx [
1083 ] ) ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 1085 , xx + 1088 ) ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 1088 , xx + 1085 ) ; xx [ 1063
] = ( xx [ 413 ] * xx [ 1085 ] + xx [ 427 ] * xx [ 1086 ] ) / xx [ 432 ] ; xx
[ 1091 ] = xx [ 1085 ] - xx [ 428 ] * xx [ 1063 ] ; xx [ 1092 ] = xx [ 1086 ]
- xx [ 431 ] * xx [ 1063 ] ; xx [ 1093 ] = xx [ 1087 ] - xx [ 438 ] * xx [
1063 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1091 , xx + 1094 ) ;
xx [ 1091 ] = xx [ 316 ] * xx [ 1082 ] ; xx [ 1092 ] = - ( xx [ 318 ] * xx [
1082 ] ) ; xx [ 1093 ] = - ( xx [ 319 ] * xx [ 1082 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 1091 , xx + 1097 ) ;
pm_math_Vector3_cross_ra ( xx + 736 , xx + 1088 , xx + 1091 ) ; xx [ 1088 ] =
xx [ 1097 ] + xx [ 1091 ] ; xx [ 1089 ] = xx [ 1098 ] + xx [ 1092 ] ; xx [
1090 ] = xx [ 1099 ] + xx [ 1093 ] ; pm_math_Quaternion_xform_ra ( xx + 236 ,
xx + 1088 , xx + 1091 ) ; pm_math_Vector3_cross_ra ( xx + 784 , xx + 1085 ,
xx + 1088 ) ; xx [ 1085 ] = xx [ 1091 ] + xx [ 1088 ] - xx [ 405 ] * xx [
1063 ] ; xx [ 1086 ] = xx [ 1092 ] + xx [ 1089 ] - xx [ 409 ] * xx [ 1063 ] ;
xx [ 1087 ] = xx [ 1093 ] + xx [ 1090 ] - xx [ 825 ] * xx [ 1063 ] ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1085 , xx + 1088 ) ;
pm_math_Vector3_cross_ra ( xx + 845 , xx + 1094 , xx + 1085 ) ; xx [ 1091 ] =
xx [ 1088 ] + xx [ 1085 ] ; xx [ 1092 ] = xx [ 1089 ] + xx [ 1086 ] ; xx [
1093 ] = xx [ 1090 ] + xx [ 1087 ] ; xx [ 1085 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 1094 ) ; xx [ 1086 ] = - pm_math_Vector3_dot_ra ( xx + 672 ,
xx + 1094 ) ; xx [ 1087 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 1094 )
; xx [ 1088 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 1091 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 1094 ) ) ; xx [ 1089 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 1091 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 1094 ) ) ; xx [ 1090 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 1091 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 1094 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 1085 , 6 , 1 , xx + 1091 , xx + 1097 )
; xx [ 1085 ] = xx [ 959 ] * xx [ 1096 ] - ( xx [ 671 ] * xx [ 1094 ] + xx [
940 ] * xx [ 1095 ] ) ; xx [ 1086 ] = xx [ 933 ] * xx [ 1094 ] - xx [ 952 ] *
xx [ 1095 ] + xx [ 971 ] * xx [ 1096 ] ; xx [ 1087 ] = xx [ 953 ] * xx [ 1095
] - xx [ 934 ] * xx [ 1094 ] + xx [ 972 ] * xx [ 1096 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1085 , xx + 1088 ) ; xx
[ 1064 ] = xx [ 1091 ] * xx [ 3 ] + xx [ 1092 ] * xx [ 196 ] + xx [ 1093 ] *
xx [ 522 ] + xx [ 941 ] * xx [ 1094 ] - xx [ 960 ] * xx [ 1095 ] + xx [ 979 ]
* xx [ 1096 ] ; pm_math_Vector3_cross_ra ( xx + 1085 , xx + 651 , xx + 1097 )
; xx [ 1100 ] = xx [ 1091 ] * xx [ 22 ] + xx [ 1092 ] * xx [ 21 ] + xx [ 1093
] * xx [ 23 ] + xx [ 942 ] * xx [ 1094 ] + xx [ 961 ] * xx [ 1095 ] - xx [
980 ] * xx [ 1096 ] ; xx [ 1101 ] = xx [ 1091 ] * xx [ 25 ] + xx [ 1092 ] *
xx [ 434 ] + xx [ 1093 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1094 ] - xx [ 962 ] *
xx [ 1095 ] - xx [ 981 ] * xx [ 1096 ] ; xx [ 1091 ] = xx [ 1064 ] + xx [
1097 ] ; xx [ 1092 ] = xx [ 1100 ] + xx [ 1098 ] ; xx [ 1093 ] = xx [ 1101 ]
+ xx [ 1099 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1091 ,
xx + 1094 ) ; xx [ 1091 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 1088 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1094 ) ; xx [ 1092 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 1088 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 1094 ) ; pm_math_Vector3_cross_ra ( xx + 1085 , xx + 204 , xx +
1097 ) ; xx [ 1102 ] = xx [ 1064 ] + xx [ 1097 ] ; xx [ 1103 ] = xx [ 1100 ]
+ xx [ 1098 ] ; xx [ 1104 ] = xx [ 1101 ] + xx [ 1099 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1102 , xx + 1097 ) ; xx
[ 1093 ] = xx [ 64 ] * xx [ 1091 ] - xx [ 1092 ] * xx [ 63 ] - xx [ 81 ] * (
xx [ 201 ] * xx [ 1098 ] + xx [ 207 ] * xx [ 1099 ] ) ; xx [ 1097 ] =
0.9999999138435711 ; xx [ 1098 ] = 7.735117927541253e-5 ; xx [ 1099 ] = -
4.078353164858054e-4 ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1097 ,
xx + 1102 ) ; xx [ 1097 ] = 0.7044166587907192 ; xx [ 1098 ] = -
0.08618526757371031 ; xx [ 1099 ] = 0.7045347901071795 ; xx [ 1105 ] =
pm_math_Vector3_dot_ra ( xx + 1102 , xx + 1097 ) ; xx [ 1102 ] = xx [ 1105 ]
; xx [ 1103 ] = xx [ 317 ] ; solveSymmetricPosDef ( xx + 664 , xx + 1102 , 2
, 1 , xx + 1106 , xx + 1108 ) ; xx [ 1102 ] = xx [ 706 ] * xx [ 1106 ] + xx [
660 ] * xx [ 1107 ] ; xx [ 1103 ] = xx [ 523 ] * xx [ 1106 ] + xx [ 661 ] *
xx [ 1107 ] ; xx [ 1104 ] = xx [ 524 ] * xx [ 1106 ] + xx [ 1107 ] * xx [ 662
] ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 1102 , xx + 1108 ) ; xx [
1102 ] = xx [ 525 ] * xx [ 1106 ] + xx [ 657 ] * xx [ 1107 ] ; xx [ 1103 ] =
xx [ 526 ] * xx [ 1106 ] + xx [ 658 ] * xx [ 1107 ] ; xx [ 1104 ] = xx [ 519
] * xx [ 1106 ] + xx [ 1107 ] * xx [ 659 ] ; pm_math_Quaternion_xform_ra ( xx
+ 464 , xx + 1102 , xx + 1111 ) ; pm_math_Vector3_cross_ra ( xx + 651 , xx +
1108 , xx + 1102 ) ; xx [ 1114 ] = xx [ 1111 ] + xx [ 1102 ] ; xx [ 1115 ] =
xx [ 1112 ] + xx [ 1103 ] ; xx [ 1116 ] = xx [ 1113 ] + xx [ 1104 ] ; xx [
1117 ] = - pm_math_Vector3_dot_ra ( xx + 26 , xx + 1108 ) ; xx [ 1118 ] = -
pm_math_Vector3_dot_ra ( xx + 672 , xx + 1108 ) ; xx [ 1119 ] = -
pm_math_Vector3_dot_ra ( xx + 703 , xx + 1108 ) ; xx [ 1120 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 1114 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 1108 ) ) ; xx [ 1121 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 1114 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 1108 ) ) ; xx [ 1122 ]
= - ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 1114 ) +
pm_math_Vector3_dot_ra ( xx + 982 , xx + 1108 ) ) ; solveSymmetricPosDef ( xx
+ 848 , xx + 1117 , 6 , 1 , xx + 1108 , xx + 1123 ) ; xx [ 1102 ] = xx [ 959
] * xx [ 1113 ] - ( xx [ 671 ] * xx [ 1111 ] + xx [ 940 ] * xx [ 1112 ] ) ;
xx [ 1103 ] = xx [ 933 ] * xx [ 1111 ] - xx [ 952 ] * xx [ 1112 ] + xx [ 971
] * xx [ 1113 ] ; xx [ 1104 ] = xx [ 953 ] * xx [ 1112 ] - xx [ 934 ] * xx [
1111 ] + xx [ 972 ] * xx [ 1113 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
464 , xx + 1102 , xx + 1114 ) ; xx [ 1117 ] = xx [ 1108 ] * xx [ 3 ] + xx [
1109 ] * xx [ 196 ] + xx [ 1110 ] * xx [ 522 ] + xx [ 941 ] * xx [ 1111 ] -
xx [ 960 ] * xx [ 1112 ] + xx [ 979 ] * xx [ 1113 ] ;
pm_math_Vector3_cross_ra ( xx + 1102 , xx + 651 , xx + 1118 ) ; xx [ 1121 ] =
xx [ 1108 ] * xx [ 22 ] + xx [ 1109 ] * xx [ 21 ] + xx [ 1110 ] * xx [ 23 ] +
xx [ 942 ] * xx [ 1111 ] + xx [ 961 ] * xx [ 1112 ] - xx [ 980 ] * xx [ 1113
] ; xx [ 1122 ] = xx [ 1108 ] * xx [ 25 ] + xx [ 1109 ] * xx [ 434 ] + xx [
1110 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1111 ] - xx [ 962 ] * xx [ 1112 ] - xx
[ 981 ] * xx [ 1113 ] ; xx [ 1108 ] = xx [ 1117 ] + xx [ 1118 ] ; xx [ 1109 ]
= xx [ 1121 ] + xx [ 1119 ] ; xx [ 1110 ] = xx [ 1122 ] + xx [ 1120 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1108 , xx + 1111 ) ; xx
[ 1108 ] = xx [ 1106 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx + 1114 ) +
pm_math_Vector3_dot_ra ( xx + 271 , xx + 1111 ) ) ; xx [ 1106 ] = xx [ 1107 ]
- ( pm_math_Vector3_dot_ra ( xx + 367 , xx + 1114 ) + pm_math_Vector3_dot_ra
( xx + 370 , xx + 1111 ) ) ; pm_math_Vector3_cross_ra ( xx + 1102 , xx + 204
, xx + 1118 ) ; xx [ 1123 ] = xx [ 1117 ] + xx [ 1118 ] ; xx [ 1124 ] = xx [
1121 ] + xx [ 1119 ] ; xx [ 1125 ] = xx [ 1122 ] + xx [ 1120 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1123 , xx + 1118 ) ; xx
[ 1107 ] = xx [ 63 ] * xx [ 1108 ] - xx [ 64 ] * xx [ 1106 ] - xx [ 81 ] * (
xx [ 201 ] * xx [ 1119 ] + xx [ 207 ] * xx [ 1120 ] ) ; xx [ 1109 ] =
0.9653194398670554 ; xx [ 1118 ] = - 1.811428046118123e-4 ; xx [ 1119 ] = xx
[ 1109 ] ; xx [ 1120 ] = - 0.2610715346452751 ; pm_math_Quaternion_xform_ra (
xx + 804 , xx + 1118 , xx + 1123 ) ; xx [ 1118 ] = 0.05435570616771113 ; xx [
1119 ] = xx [ 971 ] ; xx [ 1120 ] = 0.06752229025448353 ; xx [ 1110 ] =
pm_math_Vector3_dot_ra ( xx + 1123 , xx + 1118 ) ; xx [ 1123 ] = xx [ 1110 ]
; xx [ 1124 ] = xx [ 317 ] ; solveSymmetricPosDef ( xx + 393 , xx + 1123 , 2
, 1 , xx + 1125 , xx + 1127 ) ; xx [ 1127 ] = xx [ 364 ] * xx [ 1126 ] - xx [
398 ] * xx [ 1125 ] ; xx [ 1128 ] = xx [ 399 ] * xx [ 1125 ] + xx [ 400 ] *
xx [ 1126 ] ; xx [ 1129 ] = xx [ 402 ] * xx [ 1126 ] - xx [ 401 ] * xx [ 1125
] ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 1127 , xx + 1130 ) ; xx [
1123 ] = ( xx [ 413 ] * xx [ 1130 ] + xx [ 427 ] * xx [ 1131 ] ) / xx [ 432 ]
; xx [ 1127 ] = xx [ 1130 ] - xx [ 428 ] * xx [ 1123 ] ; xx [ 1128 ] = xx [
1131 ] - xx [ 431 ] * xx [ 1123 ] ; xx [ 1129 ] = xx [ 1132 ] - xx [ 438 ] *
xx [ 1123 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1127 , xx + 1133
) ; xx [ 1127 ] = xx [ 376 ] * xx [ 1125 ] ; xx [ 1128 ] = xx [ 379 ] * xx [
1125 ] ; xx [ 1129 ] = xx [ 397 ] * xx [ 1125 ] ; pm_math_Quaternion_xform_ra
( xx + 380 , xx + 1127 , xx + 1136 ) ; pm_math_Vector3_cross_ra ( xx + 813 ,
xx + 1130 , xx + 1127 ) ; xx [ 1130 ] = xx [ 1136 ] + xx [ 1127 ] - xx [ 405
] * xx [ 1123 ] ; xx [ 1131 ] = xx [ 1137 ] + xx [ 1128 ] - xx [ 409 ] * xx [
1123 ] ; xx [ 1132 ] = xx [ 1138 ] + xx [ 1129 ] - xx [ 825 ] * xx [ 1123 ] ;
pm_math_Quaternion_xform_ra ( xx + 388 , xx + 1130 , xx + 1127 ) ;
pm_math_Vector3_cross_ra ( xx + 845 , xx + 1133 , xx + 1130 ) ; xx [ 1136 ] =
xx [ 1127 ] + xx [ 1130 ] ; xx [ 1137 ] = xx [ 1128 ] + xx [ 1131 ] ; xx [
1138 ] = xx [ 1129 ] + xx [ 1132 ] ; xx [ 1127 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 1133 ) ; xx [ 1128 ] = - pm_math_Vector3_dot_ra ( xx + 672 ,
xx + 1133 ) ; xx [ 1129 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 1133 )
; xx [ 1130 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 1136 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 1133 ) ) ; xx [ 1131 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 1136 ) + pm_math_Vector3_dot_ra ( xx
+ 963 , xx + 1133 ) ) ; xx [ 1132 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 ,
xx + 1136 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 1133 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 1127 , 6 , 1 , xx + 1133 , xx + 1139 )
; xx [ 1127 ] = xx [ 959 ] * xx [ 1138 ] - ( xx [ 671 ] * xx [ 1136 ] + xx [
940 ] * xx [ 1137 ] ) ; xx [ 1128 ] = xx [ 933 ] * xx [ 1136 ] - xx [ 952 ] *
xx [ 1137 ] + xx [ 971 ] * xx [ 1138 ] ; xx [ 1129 ] = xx [ 953 ] * xx [ 1137
] - xx [ 934 ] * xx [ 1136 ] + xx [ 972 ] * xx [ 1138 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1127 , xx + 1130 ) ; xx
[ 1124 ] = xx [ 1133 ] * xx [ 3 ] + xx [ 1134 ] * xx [ 196 ] + xx [ 1135 ] *
xx [ 522 ] + xx [ 941 ] * xx [ 1136 ] - xx [ 960 ] * xx [ 1137 ] + xx [ 979 ]
* xx [ 1138 ] ; pm_math_Vector3_cross_ra ( xx + 1127 , xx + 651 , xx + 1139 )
; xx [ 1142 ] = xx [ 1133 ] * xx [ 22 ] + xx [ 1134 ] * xx [ 21 ] + xx [ 1135
] * xx [ 23 ] + xx [ 942 ] * xx [ 1136 ] + xx [ 961 ] * xx [ 1137 ] - xx [
980 ] * xx [ 1138 ] ; xx [ 1143 ] = xx [ 1133 ] * xx [ 25 ] + xx [ 1134 ] *
xx [ 434 ] + xx [ 1135 ] * xx [ 1 ] - xx [ 943 ] * xx [ 1136 ] - xx [ 962 ] *
xx [ 1137 ] - xx [ 981 ] * xx [ 1138 ] ; xx [ 1133 ] = xx [ 1124 ] + xx [
1139 ] ; xx [ 1134 ] = xx [ 1142 ] + xx [ 1140 ] ; xx [ 1135 ] = xx [ 1143 ]
+ xx [ 1141 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 1133 ,
xx + 1136 ) ; xx [ 1133 ] = pm_math_Vector3_dot_ra ( xx + 367 , xx + 1130 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 1136 ) ; xx [ 1134 ] =
pm_math_Vector3_dot_ra ( xx + 172 , xx + 1130 ) + pm_math_Vector3_dot_ra ( xx
+ 271 , xx + 1136 ) ; pm_math_Vector3_cross_ra ( xx + 1127 , xx + 204 , xx +
1139 ) ; xx [ 1144 ] = xx [ 1124 ] + xx [ 1139 ] ; xx [ 1145 ] = xx [ 1142 ]
+ xx [ 1140 ] ; xx [ 1146 ] = xx [ 1143 ] + xx [ 1141 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 1144 , xx + 1139 ) ; xx
[ 1135 ] = xx [ 64 ] * xx [ 1133 ] - xx [ 1134 ] * xx [ 63 ] - xx [ 81 ] * (
xx [ 201 ] * xx [ 1140 ] + xx [ 207 ] * xx [ 1141 ] ) ; xx [ 1139 ] = xx [ 41
] ; xx [ 1140 ] = xx [ 55 ] ; xx [ 1141 ] = xx [ 62 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 191 , xx + 1144 ) ; xx [
1147 ] = xx [ 433 ] ; xx [ 1148 ] = xx [ 440 ] ; xx [ 1149 ] = xx [ 222 ] ;
pm_math_Vector3_cross_ra ( xx + 191 , xx + 845 , xx + 1150 ) ; xx [ 191 ] =
xx [ 106 ] + xx [ 1150 ] ; xx [ 192 ] = xx [ 111 ] + xx [ 1151 ] ; xx [ 193 ]
= xx [ 139 ] + xx [ 1152 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 ,
xx + 191 , xx + 1150 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx +
733 , xx + 191 ) ; pm_math_Vector3_cross_ra ( xx + 733 , xx + 845 , xx + 1153
) ; xx [ 733 ] = xx [ 197 ] + xx [ 1153 ] ; xx [ 734 ] = xx [ 203 ] + xx [
1154 ] ; xx [ 735 ] = xx [ 209 ] + xx [ 1155 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 733 , xx + 1153 ) ; xx [
41 ] = ( pm_math_Vector3_dot_ra ( xx + 1139 , xx + 191 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1153 ) ) * xx [ 97 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 724 , xx + 191 ) ;
pm_math_Vector3_cross_ra ( xx + 724 , xx + 845 , xx + 733 ) ; xx [ 724 ] = xx
[ 448 ] + xx [ 733 ] ; xx [ 725 ] = xx [ 452 ] + xx [ 734 ] ; xx [ 726 ] = xx
[ 518 ] + xx [ 735 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx +
724 , xx + 733 ) ; xx [ 55 ] = ( pm_math_Vector3_dot_ra ( xx + 1139 , xx +
191 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 733 ) ) * xx [ 97 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 763 , xx + 191 ) ;
pm_math_Vector3_cross_ra ( xx + 763 , xx + 845 , xx + 724 ) ; xx [ 733 ] = xx
[ 616 ] + xx [ 724 ] ; xx [ 734 ] = xx [ 618 ] + xx [ 725 ] ; xx [ 735 ] = xx
[ 621 ] + xx [ 726 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx +
733 , xx + 724 ) ; xx [ 62 ] = ( pm_math_Vector3_dot_ra ( xx + 1139 , xx +
191 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 724 ) ) * xx [ 97 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 775 , xx + 191 ) ;
pm_math_Vector3_cross_ra ( xx + 775 , xx + 845 , xx + 724 ) ; xx [ 733 ] = xx
[ 635 ] + xx [ 724 ] ; xx [ 734 ] = xx [ 640 ] + xx [ 725 ] ; xx [ 735 ] = xx
[ 641 ] + xx [ 726 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx +
733 , xx + 724 ) ; xx [ 106 ] = ( pm_math_Vector3_dot_ra ( xx + 1139 , xx +
191 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 724 ) ) * xx [ 97 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 793 , xx + 191 ) ;
pm_math_Vector3_cross_ra ( xx + 793 , xx + 845 , xx + 724 ) ; xx [ 733 ] = xx
[ 810 ] + xx [ 724 ] ; xx [ 734 ] = xx [ 819 ] + xx [ 725 ] ; xx [ 735 ] = xx
[ 820 ] + xx [ 726 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx +
733 , xx + 724 ) ; xx [ 111 ] = xx [ 762 ] + pm_math_Vector3_dot_ra ( xx +
1139 , xx + 191 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 724 ) ; xx [
139 ] = xx [ 111 ] * xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
388 , xx + 830 , xx + 733 ) ; pm_math_Vector3_cross_ra ( xx + 830 , xx + 845
, xx + 762 ) ; xx [ 1153 ] = xx [ 822 ] + xx [ 762 ] ; xx [ 1154 ] = xx [ 824
] + xx [ 763 ] ; xx [ 1155 ] = xx [ 841 ] + xx [ 764 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1153 , xx + 762 ) ; xx [
197 ] = xx [ 211 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 733 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 762 ) ; xx [ 203 ] = xx [ 197 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 888 , xx +
1153 ) ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 845 , xx + 1156 ) ; xx [
1159 ] = xx [ 620 ] + xx [ 1156 ] ; xx [ 1160 ] = xx [ 919 ] + xx [ 1157 ] ;
xx [ 1161 ] = xx [ 920 ] + xx [ 1158 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 388 , xx + 1159 , xx + 1156 ) ; xx [ 209 ] = pm_math_Vector3_dot_ra ( xx
+ 1139 , xx + 1153 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1156 ) ; xx
[ 211 ] = xx [ 209 ] * xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
388 , xx + 1000 , xx + 1159 ) ; pm_math_Vector3_cross_ra ( xx + 1000 , xx +
845 , xx + 1162 ) ; xx [ 1165 ] = xx [ 615 ] + xx [ 1162 ] ; xx [ 1166 ] = xx
[ 617 ] + xx [ 1163 ] ; xx [ 1167 ] = xx [ 663 ] + xx [ 1164 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1165 , xx + 1162 ) ; xx
[ 222 ] = xx [ 611 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1159 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1162 ) ; xx [ 433 ] = xx [ 222 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1030 , xx +
1165 ) ; pm_math_Vector3_cross_ra ( xx + 1030 , xx + 845 , xx + 1168 ) ; xx [
1171 ] = xx [ 977 ] + xx [ 1168 ] ; xx [ 1172 ] = xx [ 978 ] + xx [ 1169 ] ;
xx [ 1173 ] = xx [ 997 ] + xx [ 1170 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 388 , xx + 1171 , xx + 1168 ) ; xx [ 440 ] = pm_math_Vector3_dot_ra ( xx
+ 1139 , xx + 1165 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1168 ) ; xx
[ 448 ] = xx [ 440 ] * xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
388 , xx + 1045 , xx + 1171 ) ; pm_math_Vector3_cross_ra ( xx + 1045 , xx +
845 , xx + 1174 ) ; xx [ 1177 ] = xx [ 1007 ] + xx [ 1174 ] ; xx [ 1178 ] =
xx [ 1008 ] + xx [ 1175 ] ; xx [ 1179 ] = xx [ 1027 ] + xx [ 1176 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1177 , xx + 1174 ) ; xx
[ 452 ] = pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1171 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1174 ) ; xx [ 518 ] = xx [ 452 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1069 , xx +
1177 ) ; pm_math_Vector3_cross_ra ( xx + 1069 , xx + 845 , xx + 1180 ) ; xx [
1183 ] = xx [ 1065 ] + xx [ 1180 ] ; xx [ 1184 ] = xx [ 1066 ] + xx [ 1181 ]
; xx [ 1185 ] = xx [ 1084 ] + xx [ 1182 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1183 , xx + 1180 ) ; xx
[ 611 ] = xx [ 1062 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1177 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1180 ) ; xx [ 616 ] = xx [ 611 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1085 , xx +
1183 ) ; pm_math_Vector3_cross_ra ( xx + 1085 , xx + 845 , xx + 1186 ) ; xx [
1189 ] = xx [ 1064 ] + xx [ 1186 ] ; xx [ 1190 ] = xx [ 1100 ] + xx [ 1187 ]
; xx [ 1191 ] = xx [ 1101 ] + xx [ 1188 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1189 , xx + 1186 ) ; xx
[ 618 ] = xx [ 1063 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1183 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1186 ) ; xx [ 621 ] = xx [ 618 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1102 , xx +
1189 ) ; pm_math_Vector3_cross_ra ( xx + 1102 , xx + 845 , xx + 1192 ) ; xx [
1195 ] = xx [ 1117 ] + xx [ 1192 ] ; xx [ 1196 ] = xx [ 1121 ] + xx [ 1193 ]
; xx [ 1197 ] = xx [ 1122 ] + xx [ 1194 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1195 , xx + 1192 ) ; xx
[ 765 ] = pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1189 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1192 ) ; xx [ 1062 ] = xx [ 765 ] *
xx [ 97 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1127 , xx +
1195 ) ; pm_math_Vector3_cross_ra ( xx + 1127 , xx + 845 , xx + 1198 ) ; xx [
1201 ] = xx [ 1124 ] + xx [ 1198 ] ; xx [ 1202 ] = xx [ 1142 ] + xx [ 1199 ]
; xx [ 1203 ] = xx [ 1143 ] + xx [ 1200 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 1201 , xx + 1198 ) ; xx
[ 1063 ] = xx [ 1123 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 1195 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1198 ) ; xx [ 1123 ] = xx [ 1063 ]
* xx [ 97 ] ; xx [ 1201 ] = xx [ 167 ] * xx [ 520 ] ; xx [ 1202 ] = xx [ 167
] * xx [ 622 ] ; xx [ 1203 ] = - ( xx [ 167 ] * xx [ 668 ] ) ; xx [ 1204 ] =
- ( xx [ 167 ] * xx [ 802 ] ) ; xx [ 1205 ] = - ( xx [ 167 ] * xx [ 844 ] ) ;
xx [ 1206 ] = xx [ 167 ] * xx [ 910 ] ; xx [ 1207 ] = - ( xx [ 167 ] * xx [
912 ] ) ; xx [ 1208 ] = xx [ 167 ] * xx [ 1024 ] ; xx [ 1209 ] = xx [ 167 ] *
xx [ 1054 ] ; xx [ 1210 ] = - ( xx [ 167 ] * xx [ 1076 ] ) ; xx [ 1211 ] = -
( xx [ 167 ] * xx [ 1092 ] ) ; xx [ 1212 ] = xx [ 167 ] * xx [ 1108 ] ; xx [
1213 ] = - ( xx [ 167 ] * xx [ 1134 ] ) ; xx [ 1214 ] = xx [ 444 ] * xx [ 622
] ; xx [ 1215 ] = - ( xx [ 444 ] * xx [ 668 ] ) ; xx [ 1216 ] = - ( xx [ 444
] * xx [ 802 ] ) ; xx [ 1217 ] = - ( xx [ 444 ] * xx [ 844 ] ) ; xx [ 1218 ]
= xx [ 444 ] * xx [ 910 ] ; xx [ 1219 ] = - ( xx [ 444 ] * xx [ 912 ] ) ; xx
[ 1220 ] = xx [ 444 ] * xx [ 1024 ] ; xx [ 1221 ] = xx [ 444 ] * xx [ 1054 ]
; xx [ 1222 ] = - ( xx [ 444 ] * xx [ 1076 ] ) ; xx [ 1223 ] = - ( xx [ 444 ]
* xx [ 1092 ] ) ; xx [ 1224 ] = xx [ 444 ] * xx [ 1108 ] ; xx [ 1225 ] = - (
xx [ 444 ] * xx [ 1134 ] ) ; xx [ 1226 ] = - ( xx [ 576 ] * xx [ 668 ] + xx [
654 ] * xx [ 577 ] ) ; xx [ 654 ] = - ( xx [ 576 ] * xx [ 802 ] + xx [ 801 ]
* xx [ 577 ] ) ; xx [ 668 ] = - ( xx [ 576 ] * xx [ 844 ] + xx [ 843 ] * xx [
577 ] ) ; xx [ 801 ] = xx [ 576 ] * xx [ 910 ] + xx [ 577 ] * xx [ 908 ] ; xx
[ 802 ] = - ( xx [ 576 ] * xx [ 912 ] + xx [ 911 ] * xx [ 577 ] ) ; xx [ 843
] = xx [ 576 ] * xx [ 1024 ] + xx [ 577 ] * xx [ 1028 ] ; xx [ 844 ] = xx [
576 ] * xx [ 1054 ] + xx [ 577 ] * xx [ 1043 ] ; xx [ 1227 ] = - ( xx [ 576 ]
* xx [ 1076 ] + xx [ 1075 ] * xx [ 577 ] ) ; xx [ 1228 ] = - ( xx [ 576 ] *
xx [ 1092 ] + xx [ 1091 ] * xx [ 577 ] ) ; xx [ 1229 ] = xx [ 576 ] * xx [
1108 ] + xx [ 577 ] * xx [ 1106 ] ; xx [ 1230 ] = - ( xx [ 576 ] * xx [ 1134
] + xx [ 1133 ] * xx [ 577 ] ) ; xx [ 1231 ] = xx [ 179 ] ; xx [ 1232 ] = xx
[ 184 ] ; xx [ 1233 ] = xx [ 182 ] / xx [ 178 ] ; pm_math_Vector3_cross_ra (
xx + 775 , xx + 140 , xx + 1234 ) ; xx [ 775 ] = xx [ 635 ] + xx [ 1234 ] ;
xx [ 776 ] = xx [ 640 ] + xx [ 1235 ] ; xx [ 777 ] = xx [ 641 ] + xx [ 1236 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 775 , xx + 1234 ) ;
pm_math_Vector3_cross_ra ( xx + 793 , xx + 140 , xx + 775 ) ; xx [ 793 ] = xx
[ 810 ] + xx [ 775 ] ; xx [ 794 ] = xx [ 819 ] + xx [ 776 ] ; xx [ 795 ] = xx
[ 820 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx +
793 , xx + 775 ) ; xx [ 179 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx + 775
) * xx [ 57 ] ; pm_math_Vector3_cross_ra ( xx + 830 , xx + 140 , xx + 775 ) ;
xx [ 793 ] = xx [ 822 ] + xx [ 775 ] ; xx [ 794 ] = xx [ 824 ] + xx [ 776 ] ;
xx [ 795 ] = xx [ 841 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 125 , xx + 793 , xx + 775 ) ; xx [ 184 ] = pm_math_Vector3_dot_ra ( xx +
1231 , xx + 775 ) * xx [ 57 ] ; pm_math_Vector3_cross_ra ( xx + 888 , xx +
140 , xx + 775 ) ; xx [ 793 ] = xx [ 620 ] + xx [ 775 ] ; xx [ 794 ] = xx [
919 ] + xx [ 776 ] ; xx [ 795 ] = xx [ 920 ] + xx [ 777 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 793 , xx + 775 ) ; xx [
635 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx + 775 ) * xx [ 57 ] ;
pm_math_Vector3_cross_ra ( xx + 1000 , xx + 140 , xx + 775 ) ; xx [ 793 ] =
xx [ 615 ] + xx [ 775 ] ; xx [ 794 ] = xx [ 617 ] + xx [ 776 ] ; xx [ 795 ] =
xx [ 663 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx
+ 793 , xx + 775 ) ; xx [ 640 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx +
775 ) * xx [ 57 ] ; pm_math_Vector3_cross_ra ( xx + 1030 , xx + 140 , xx +
775 ) ; xx [ 793 ] = xx [ 977 ] + xx [ 775 ] ; xx [ 794 ] = xx [ 978 ] + xx [
776 ] ; xx [ 795 ] = xx [ 997 ] + xx [ 777 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 793 , xx + 775 ) ; xx [
641 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx + 775 ) * xx [ 57 ] ;
pm_math_Vector3_cross_ra ( xx + 1045 , xx + 140 , xx + 775 ) ; xx [ 793 ] =
xx [ 1007 ] + xx [ 775 ] ; xx [ 794 ] = xx [ 1008 ] + xx [ 776 ] ; xx [ 795 ]
= xx [ 1027 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 125 ,
xx + 793 , xx + 775 ) ; xx [ 793 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx
+ 775 ) * xx [ 57 ] ; pm_math_Vector3_cross_ra ( xx + 1069 , xx + 140 , xx +
775 ) ; xx [ 1237 ] = xx [ 1065 ] + xx [ 775 ] ; xx [ 1238 ] = xx [ 1066 ] +
xx [ 776 ] ; xx [ 1239 ] = xx [ 1084 ] + xx [ 777 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 1237 , xx + 775 ) ; xx [
794 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx + 775 ) * xx [ 57 ] ;
pm_math_Vector3_cross_ra ( xx + 1085 , xx + 140 , xx + 775 ) ; xx [ 1237 ] =
xx [ 1064 ] + xx [ 775 ] ; xx [ 1238 ] = xx [ 1100 ] + xx [ 776 ] ; xx [ 1239
] = xx [ 1101 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 125
, xx + 1237 , xx + 775 ) ; xx [ 795 ] = pm_math_Vector3_dot_ra ( xx + 1231 ,
xx + 775 ) * xx [ 57 ] ; pm_math_Vector3_cross_ra ( xx + 1102 , xx + 140 , xx
+ 775 ) ; xx [ 1237 ] = xx [ 1117 ] + xx [ 775 ] ; xx [ 1238 ] = xx [ 1121 ]
+ xx [ 776 ] ; xx [ 1239 ] = xx [ 1122 ] + xx [ 777 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 1237 , xx + 775 ) ; xx [
810 ] = pm_math_Vector3_dot_ra ( xx + 1231 , xx + 775 ) * xx [ 57 ] ;
pm_math_Vector3_cross_ra ( xx + 1127 , xx + 140 , xx + 775 ) ; xx [ 1237 ] =
xx [ 1124 ] + xx [ 775 ] ; xx [ 1238 ] = xx [ 1142 ] + xx [ 776 ] ; xx [ 1239
] = xx [ 1143 ] + xx [ 777 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 125
, xx + 1237 , xx + 775 ) ; xx [ 819 ] = pm_math_Vector3_dot_ra ( xx + 1231 ,
xx + 775 ) * xx [ 57 ] ; xx [ 775 ] = xx [ 415 ] ; xx [ 776 ] = xx [ 417 ] ;
xx [ 777 ] = xx [ 419 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx
+ 191 , xx + 1237 ) ; xx [ 1240 ] = xx [ 421 ] ; xx [ 1241 ] = xx [ 423 ] ;
xx [ 1242 ] = xx [ 425 ] ; pm_math_Vector3_cross_ra ( xx + 191 , xx + 813 ,
xx + 1243 ) ; xx [ 191 ] = xx [ 724 ] - xx [ 111 ] * xx [ 413 ] + xx [ 1243 ]
; xx [ 192 ] = xx [ 725 ] - xx [ 111 ] * xx [ 427 ] + xx [ 1244 ] ; xx [ 193
] = xx [ 726 ] + xx [ 1245 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380
, xx + 191 , xx + 724 ) ; xx [ 191 ] = xx [ 416 ] ; xx [ 192 ] = xx [ 418 ] ;
xx [ 193 ] = xx [ 420 ] ; xx [ 415 ] = xx [ 422 ] ; xx [ 416 ] = xx [ 424 ] ;
xx [ 417 ] = xx [ 426 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx
+ 733 , xx + 418 ) ; pm_math_Vector3_cross_ra ( xx + 733 , xx + 813 , xx +
421 ) ; xx [ 424 ] = xx [ 762 ] - xx [ 197 ] * xx [ 413 ] + xx [ 421 ] ; xx [
425 ] = xx [ 763 ] - xx [ 197 ] * xx [ 427 ] + xx [ 422 ] ; xx [ 426 ] = xx [
764 ] + xx [ 423 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 424
, xx + 421 ) ; xx [ 111 ] = xx [ 808 ] - ( pm_math_Vector3_dot_ra ( xx + 775
, xx + 418 ) + pm_math_Vector3_dot_ra ( xx + 1240 , xx + 421 ) ) ; xx [ 424 ]
= xx [ 809 ] - ( pm_math_Vector3_dot_ra ( xx + 191 , xx + 418 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 421 ) ) ; xx [ 418 ] = xx [ 761 ] *
xx [ 111 ] - xx [ 760 ] * xx [ 424 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 380 , xx + 1153 , xx + 419 ) ; pm_math_Vector3_cross_ra ( xx + 1153 , xx
+ 813 , xx + 733 ) ; xx [ 762 ] = xx [ 1156 ] - xx [ 209 ] * xx [ 413 ] + xx
[ 733 ] ; xx [ 763 ] = xx [ 1157 ] - xx [ 209 ] * xx [ 427 ] + xx [ 734 ] ;
xx [ 764 ] = xx [ 1158 ] + xx [ 735 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 380 , xx + 762 , xx + 733 ) ; xx [ 422 ] = pm_math_Vector3_dot_ra ( xx +
191 , xx + 419 ) + pm_math_Vector3_dot_ra ( xx + 415 , xx + 733 ) ; xx [ 423
] = pm_math_Vector3_dot_ra ( xx + 775 , xx + 419 ) + pm_math_Vector3_dot_ra (
xx + 1240 , xx + 733 ) ; xx [ 419 ] = xx [ 422 ] * xx [ 760 ] - xx [ 761 ] *
xx [ 423 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1159 , xx +
733 ) ; xx [ 420 ] = xx [ 1162 ] - xx [ 222 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1159 , xx + 813 , xx + 762 ) ; xx [ 421 ] =
xx [ 1163 ] - xx [ 222 ] * xx [ 427 ] ; xx [ 1153 ] = xx [ 420 ] + xx [ 762 ]
; xx [ 1154 ] = xx [ 421 ] + xx [ 763 ] ; xx [ 1155 ] = xx [ 1164 ] + xx [
764 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1153 , xx + 762
) ; xx [ 425 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 733 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 762 ) ; xx [ 426 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 733 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 762 ) ; xx [ 733 ] = xx [ 425 ] * xx [ 760 ] - xx [ 761 ] * xx
[ 426 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1165 , xx +
762 ) ; xx [ 734 ] = xx [ 1168 ] - xx [ 440 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1165 , xx + 813 , xx + 1153 ) ; xx [ 735 ] =
xx [ 1169 ] - xx [ 440 ] * xx [ 427 ] ; xx [ 1156 ] = xx [ 734 ] + xx [ 1153
] ; xx [ 1157 ] = xx [ 735 ] + xx [ 1154 ] ; xx [ 1158 ] = xx [ 1170 ] + xx [
1155 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1156 , xx +
1153 ) ; xx [ 808 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 762 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1153 ) ; xx [ 809 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 762 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 1153 ) ; xx [ 762 ] = xx [ 808 ] * xx [ 760 ] - xx [ 761 ] * xx
[ 809 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1171 , xx +
1153 ) ; xx [ 763 ] = xx [ 1174 ] - xx [ 452 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1171 , xx + 813 , xx + 1156 ) ; xx [ 764 ] =
xx [ 1175 ] - xx [ 452 ] * xx [ 427 ] ; xx [ 1243 ] = xx [ 763 ] + xx [ 1156
] ; xx [ 1244 ] = xx [ 764 ] + xx [ 1157 ] ; xx [ 1245 ] = xx [ 1176 ] + xx [
1158 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1243 , xx +
1156 ) ; xx [ 820 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 1153 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1156 ) ; xx [ 1162 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 1153 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 1156 ) ; xx [ 1153 ] = xx [ 820 ] * xx [ 760 ] - xx [ 761 ] *
xx [ 1162 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1177 , xx
+ 1154 ) ; xx [ 1157 ] = xx [ 1180 ] - xx [ 611 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1177 , xx + 813 , xx + 1243 ) ; xx [ 1158 ] =
xx [ 1181 ] - xx [ 611 ] * xx [ 427 ] ; xx [ 1246 ] = xx [ 1157 ] + xx [ 1243
] ; xx [ 1247 ] = xx [ 1158 ] + xx [ 1244 ] ; xx [ 1248 ] = xx [ 1182 ] + xx
[ 1245 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1246 , xx +
1243 ) ; xx [ 1163 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 1154 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1243 ) ; xx [ 1168 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 1154 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 1243 ) ; xx [ 1154 ] = xx [ 1163 ] * xx [ 760 ] - xx [ 761 ] *
xx [ 1168 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1183 , xx
+ 1243 ) ; xx [ 1155 ] = xx [ 1186 ] - xx [ 618 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1183 , xx + 813 , xx + 1246 ) ; xx [ 1156 ] =
xx [ 1187 ] - xx [ 618 ] * xx [ 427 ] ; xx [ 1249 ] = xx [ 1155 ] + xx [ 1246
] ; xx [ 1250 ] = xx [ 1156 ] + xx [ 1247 ] ; xx [ 1251 ] = xx [ 1188 ] + xx
[ 1248 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1249 , xx +
1246 ) ; xx [ 1169 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 1243 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1246 ) ; xx [ 1174 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 1243 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 1246 ) ; xx [ 1175 ] = xx [ 1169 ] * xx [ 760 ] - xx [ 761 ] *
xx [ 1174 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1189 , xx
+ 1243 ) ; xx [ 1180 ] = xx [ 1192 ] - xx [ 765 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1189 , xx + 813 , xx + 1246 ) ; xx [ 1181 ] =
xx [ 1193 ] - xx [ 765 ] * xx [ 427 ] ; xx [ 1249 ] = xx [ 1180 ] + xx [ 1246
] ; xx [ 1250 ] = xx [ 1181 ] + xx [ 1247 ] ; xx [ 1251 ] = xx [ 1194 ] + xx
[ 1248 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1249 , xx +
1246 ) ; xx [ 1186 ] = pm_math_Vector3_dot_ra ( xx + 191 , xx + 1243 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1246 ) ; xx [ 1187 ] =
pm_math_Vector3_dot_ra ( xx + 775 , xx + 1243 ) + pm_math_Vector3_dot_ra ( xx
+ 1240 , xx + 1246 ) ; xx [ 1192 ] = xx [ 1186 ] * xx [ 760 ] - xx [ 761 ] *
xx [ 1187 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1195 , xx
+ 1243 ) ; xx [ 1193 ] = xx [ 1198 ] - xx [ 1063 ] * xx [ 413 ] ;
pm_math_Vector3_cross_ra ( xx + 1195 , xx + 813 , xx + 1246 ) ; xx [ 1198 ] =
xx [ 1199 ] - xx [ 1063 ] * xx [ 427 ] ; xx [ 1249 ] = xx [ 1193 ] + xx [
1246 ] ; xx [ 1250 ] = xx [ 1198 ] + xx [ 1247 ] ; xx [ 1251 ] = xx [ 1200 ]
+ xx [ 1248 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 1249 ,
xx + 1246 ) ; xx [ 1199 ] = xx [ 1125 ] - ( pm_math_Vector3_dot_ra ( xx + 775
, xx + 1243 ) + pm_math_Vector3_dot_ra ( xx + 1240 , xx + 1246 ) ) ; xx [
1125 ] = xx [ 1126 ] - ( pm_math_Vector3_dot_ra ( xx + 191 , xx + 1243 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 1246 ) ) ; xx [ 1126 ] = xx [ 761 ]
* xx [ 1199 ] - xx [ 760 ] * xx [ 1125 ] ; xx [ 1243 ] = - xx [ 212 ] ; xx [
1244 ] = xx [ 821 ] ; xx [ 1245 ] = xx [ 823 ] ; xx [ 1246 ] = xx [ 836 ] ;
pm_math_Vector3_cross_ra ( xx + 830 , xx + 449 , xx + 1247 ) ; xx [ 830 ] =
xx [ 822 ] + xx [ 1247 ] ; xx [ 831 ] = xx [ 824 ] + xx [ 1248 ] ; xx [ 832 ]
= xx [ 841 ] + xx [ 1249 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 ,
xx + 830 , xx + 821 ) ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 449 , xx +
830 ) ; xx [ 888 ] = xx [ 620 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 919 ] + xx [
831 ] ; xx [ 890 ] = xx [ 920 ] + xx [ 832 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 888 , xx + 830 ) ; xx [
212 ] = xx [ 198 ] * xx [ 209 ] - xx [ 423 ] * xx [ 59 ] + xx [ 181 ] * xx [
422 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ] + xx [ 195 ] * xx [ 832 ] ) ;
pm_math_Vector3_cross_ra ( xx + 1000 , xx + 449 , xx + 830 ) ; xx [ 888 ] =
xx [ 615 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 617 ] + xx [ 831 ] ; xx [ 890 ] =
xx [ 663 ] + xx [ 832 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx
+ 888 , xx + 830 ) ; xx [ 209 ] = xx [ 198 ] * xx [ 222 ] - xx [ 426 ] * xx [
59 ] + xx [ 181 ] * xx [ 425 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ] + xx
[ 195 ] * xx [ 832 ] ) ; pm_math_Vector3_cross_ra ( xx + 1030 , xx + 449 , xx
+ 830 ) ; xx [ 888 ] = xx [ 977 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 978 ] + xx
[ 831 ] ; xx [ 890 ] = xx [ 997 ] + xx [ 832 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 888 , xx + 830 ) ; xx [
222 ] = xx [ 198 ] * xx [ 440 ] - xx [ 809 ] * xx [ 59 ] + xx [ 181 ] * xx [
808 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ] + xx [ 195 ] * xx [ 832 ] ) ;
pm_math_Vector3_cross_ra ( xx + 1045 , xx + 449 , xx + 830 ) ; xx [ 888 ] =
xx [ 1007 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 1008 ] + xx [ 831 ] ; xx [ 890 ]
= xx [ 1027 ] + xx [ 832 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 ,
xx + 888 , xx + 830 ) ; xx [ 422 ] = xx [ 198 ] * xx [ 452 ] - xx [ 1162 ] *
xx [ 59 ] + xx [ 181 ] * xx [ 820 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ]
+ xx [ 195 ] * xx [ 832 ] ) ; pm_math_Vector3_cross_ra ( xx + 1069 , xx + 449
, xx + 830 ) ; xx [ 888 ] = xx [ 1065 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 1066
] + xx [ 831 ] ; xx [ 890 ] = xx [ 1084 ] + xx [ 832 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 888 , xx + 830 ) ; xx [
423 ] = xx [ 198 ] * xx [ 611 ] - xx [ 1168 ] * xx [ 59 ] + xx [ 181 ] * xx [
1163 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ] + xx [ 195 ] * xx [ 832 ] ) ;
pm_math_Vector3_cross_ra ( xx + 1085 , xx + 449 , xx + 830 ) ; xx [ 888 ] =
xx [ 1064 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 1100 ] + xx [ 831 ] ; xx [ 890 ]
= xx [ 1101 ] + xx [ 832 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 ,
xx + 888 , xx + 830 ) ; xx [ 425 ] = xx [ 198 ] * xx [ 618 ] - xx [ 1174 ] *
xx [ 59 ] + xx [ 181 ] * xx [ 1169 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ]
+ xx [ 195 ] * xx [ 832 ] ) ; pm_math_Vector3_cross_ra ( xx + 1102 , xx + 449
, xx + 830 ) ; xx [ 888 ] = xx [ 1117 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 1121
] + xx [ 831 ] ; xx [ 890 ] = xx [ 1122 ] + xx [ 832 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 888 , xx + 830 ) ; xx [
426 ] = xx [ 198 ] * xx [ 765 ] - xx [ 1187 ] * xx [ 59 ] + xx [ 181 ] * xx [
1186 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831 ] + xx [ 195 ] * xx [ 832 ] ) ;
pm_math_Vector3_cross_ra ( xx + 1127 , xx + 449 , xx + 830 ) ; xx [ 888 ] =
xx [ 1124 ] + xx [ 830 ] ; xx [ 889 ] = xx [ 1142 ] + xx [ 831 ] ; xx [ 890 ]
= xx [ 1143 ] + xx [ 832 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 ,
xx + 888 , xx + 830 ) ; xx [ 440 ] = xx [ 198 ] * xx [ 1063 ] + xx [ 59 ] *
xx [ 1199 ] - xx [ 181 ] * xx [ 1125 ] + xx [ 838 ] * ( xx [ 165 ] * xx [ 831
] + xx [ 195 ] * xx [ 832 ] ) ; xx [ 830 ] = xx [ 581 ] ; xx [ 831 ] = xx [
583 ] ; xx [ 832 ] = xx [ 585 ] ; xx [ 888 ] = xx [ 905 ] + xx [ 442 ] * xx [
910 ] ; xx [ 889 ] = xx [ 906 ] + xx [ 458 ] * xx [ 910 ] ; xx [ 890 ] = xx [
907 ] + xx [ 459 ] * xx [ 910 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
489 , xx + 888 , xx + 905 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 545 ,
xx + 905 , xx + 1000 ) ; xx [ 1030 ] = xx [ 587 ] ; xx [ 1031 ] = xx [ 589 ]
; xx [ 1032 ] = xx [ 591 ] ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 648 ,
xx + 1045 ) ; xx [ 888 ] = xx [ 913 ] - xx [ 655 ] * xx [ 910 ] + xx [ 442 ]
* xx [ 908 ] + xx [ 1045 ] ; xx [ 889 ] = xx [ 914 ] + xx [ 656 ] * xx [ 910
] + xx [ 458 ] * xx [ 908 ] + xx [ 1046 ] ; xx [ 890 ] = xx [ 915 ] - xx [
699 ] * xx [ 910 ] + xx [ 459 ] * xx [ 908 ] + xx [ 1047 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 888 , xx + 913 ) ;
pm_math_Vector3_cross_ra ( xx + 905 , xx + 624 , xx + 888 ) ; xx [ 905 ] = xx
[ 913 ] + xx [ 888 ] ; xx [ 906 ] = xx [ 914 ] + xx [ 889 ] ; xx [ 907 ] = xx
[ 915 ] + xx [ 890 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx +
905 , xx + 888 ) ; xx [ 905 ] = xx [ 582 ] ; xx [ 906 ] = xx [ 584 ] ; xx [
907 ] = xx [ 586 ] ; xx [ 581 ] = xx [ 588 ] ; xx [ 582 ] = xx [ 590 ] ; xx [
583 ] = xx [ 592 ] ; xx [ 584 ] = xx [ 1003 ] - xx [ 912 ] * xx [ 442 ] ; xx
[ 585 ] = xx [ 1004 ] - xx [ 912 ] * xx [ 458 ] ; xx [ 586 ] = xx [ 1005 ] -
xx [ 912 ] * xx [ 459 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx
+ 584 , xx + 587 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 587
, xx + 590 ) ; pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 913 ) ;
xx [ 584 ] = xx [ 1009 ] + xx [ 912 ] * xx [ 655 ] - xx [ 911 ] * xx [ 442 ]
+ xx [ 913 ] ; xx [ 585 ] = xx [ 1010 ] - xx [ 912 ] * xx [ 656 ] - xx [ 911
] * xx [ 458 ] + xx [ 914 ] ; xx [ 586 ] = xx [ 1011 ] + xx [ 912 ] * xx [
699 ] - xx [ 911 ] * xx [ 459 ] + xx [ 915 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 910 ) ;
pm_math_Vector3_cross_ra ( xx + 587 , xx + 624 , xx + 584 ) ; xx [ 587 ] = xx
[ 910 ] + xx [ 584 ] ; xx [ 588 ] = xx [ 911 ] + xx [ 585 ] ; xx [ 589 ] = xx
[ 912 ] + xx [ 586 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx +
587 , xx + 584 ) ; xx [ 452 ] = xx [ 580 ] * ( pm_math_Vector3_dot_ra ( xx +
905 , xx + 590 ) + pm_math_Vector3_dot_ra ( xx + 581 , xx + 584 ) ) - (
pm_math_Vector3_dot_ra ( xx + 830 , xx + 590 ) + pm_math_Vector3_dot_ra ( xx
+ 1030 , xx + 584 ) ) * xx [ 578 ] ; xx [ 584 ] = xx [ 1033 ] + xx [ 442 ] *
xx [ 1024 ] ; xx [ 585 ] = xx [ 1034 ] + xx [ 458 ] * xx [ 1024 ] ; xx [ 586
] = xx [ 1035 ] + xx [ 459 ] * xx [ 1024 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 587 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 587 , xx + 590 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 910 ) ; xx [ 584 ] = xx
[ 1036 ] - xx [ 655 ] * xx [ 1024 ] + xx [ 442 ] * xx [ 1028 ] + xx [ 910 ] ;
xx [ 585 ] = xx [ 1037 ] + xx [ 656 ] * xx [ 1024 ] + xx [ 458 ] * xx [ 1028
] + xx [ 911 ] ; xx [ 586 ] = xx [ 1038 ] - xx [ 699 ] * xx [ 1024 ] + xx [
459 ] * xx [ 1028 ] + xx [ 912 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
489 , xx + 584 , xx + 910 ) ; pm_math_Vector3_cross_ra ( xx + 587 , xx + 624
, xx + 584 ) ; xx [ 587 ] = xx [ 910 ] + xx [ 584 ] ; xx [ 588 ] = xx [ 911 ]
+ xx [ 585 ] ; xx [ 589 ] = xx [ 912 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 587 , xx + 584 ) ; xx [
587 ] = xx [ 1022 ] - ( pm_math_Vector3_dot_ra ( xx + 830 , xx + 590 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ) ; xx [ 588 ] = xx [ 578 ] *
xx [ 587 ] - xx [ 580 ] * ( xx [ 1023 ] - ( pm_math_Vector3_dot_ra ( xx + 905
, xx + 590 ) + pm_math_Vector3_dot_ra ( xx + 581 , xx + 584 ) ) ) ; xx [ 584
] = xx [ 1048 ] + xx [ 442 ] * xx [ 1054 ] ; xx [ 585 ] = xx [ 1049 ] + xx [
458 ] * xx [ 1054 ] ; xx [ 586 ] = xx [ 1050 ] + xx [ 459 ] * xx [ 1054 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 589 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 589 , xx + 910 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 913 ) ; xx [ 584 ] = xx
[ 1051 ] - xx [ 655 ] * xx [ 1054 ] + xx [ 442 ] * xx [ 1043 ] + xx [ 913 ] ;
xx [ 585 ] = xx [ 1052 ] + xx [ 656 ] * xx [ 1054 ] + xx [ 458 ] * xx [ 1043
] + xx [ 914 ] ; xx [ 586 ] = xx [ 1053 ] - xx [ 699 ] * xx [ 1054 ] + xx [
459 ] * xx [ 1043 ] + xx [ 915 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
489 , xx + 584 , xx + 913 ) ; pm_math_Vector3_cross_ra ( xx + 589 , xx + 624
, xx + 584 ) ; xx [ 589 ] = xx [ 913 ] + xx [ 584 ] ; xx [ 590 ] = xx [ 914 ]
+ xx [ 585 ] ; xx [ 591 ] = xx [ 915 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 589 , xx + 584 ) ; xx [
589 ] = xx [ 1025 ] - ( pm_math_Vector3_dot_ra ( xx + 830 , xx + 910 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ) ; xx [ 590 ] = xx [ 578 ] *
xx [ 589 ] - xx [ 580 ] * ( xx [ 1026 ] - ( pm_math_Vector3_dot_ra ( xx + 905
, xx + 910 ) + pm_math_Vector3_dot_ra ( xx + 581 , xx + 584 ) ) ) ; xx [ 584
] = xx [ 1072 ] - xx [ 1076 ] * xx [ 442 ] ; xx [ 585 ] = xx [ 1073 ] - xx [
1076 ] * xx [ 458 ] ; xx [ 586 ] = xx [ 1074 ] - xx [ 1076 ] * xx [ 459 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 910 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 913 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 1003 ) ; xx [ 584 ] =
xx [ 1078 ] + xx [ 1076 ] * xx [ 655 ] - xx [ 1075 ] * xx [ 442 ] + xx [ 1003
] ; xx [ 585 ] = xx [ 1079 ] - xx [ 1076 ] * xx [ 656 ] - xx [ 1075 ] * xx [
458 ] + xx [ 1004 ] ; xx [ 586 ] = xx [ 1080 ] + xx [ 1076 ] * xx [ 699 ] -
xx [ 1075 ] * xx [ 459 ] + xx [ 1005 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 489 , xx + 584 , xx + 1003 ) ; pm_math_Vector3_cross_ra ( xx + 910 , xx
+ 624 , xx + 584 ) ; xx [ 910 ] = xx [ 1003 ] + xx [ 584 ] ; xx [ 911 ] = xx
[ 1004 ] + xx [ 585 ] ; xx [ 912 ] = xx [ 1005 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 584 ) ; xx [
591 ] = pm_math_Vector3_dot_ra ( xx + 830 , xx + 913 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ; xx [ 592 ] = xx [ 580 ] * (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 913 ) + pm_math_Vector3_dot_ra ( xx
+ 581 , xx + 584 ) ) - xx [ 591 ] * xx [ 578 ] ; xx [ 584 ] = xx [ 1088 ] -
xx [ 1092 ] * xx [ 442 ] ; xx [ 585 ] = xx [ 1089 ] - xx [ 1092 ] * xx [ 458
] ; xx [ 586 ] = xx [ 1090 ] - xx [ 1092 ] * xx [ 459 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 910 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 913 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 1003 ) ; xx [ 584 ] =
xx [ 1094 ] + xx [ 1092 ] * xx [ 655 ] - xx [ 1091 ] * xx [ 442 ] + xx [ 1003
] ; xx [ 585 ] = xx [ 1095 ] - xx [ 1092 ] * xx [ 656 ] - xx [ 1091 ] * xx [
458 ] + xx [ 1004 ] ; xx [ 586 ] = xx [ 1096 ] + xx [ 1092 ] * xx [ 699 ] -
xx [ 1091 ] * xx [ 459 ] + xx [ 1005 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 489 , xx + 584 , xx + 1003 ) ; pm_math_Vector3_cross_ra ( xx + 910 , xx
+ 624 , xx + 584 ) ; xx [ 910 ] = xx [ 1003 ] + xx [ 584 ] ; xx [ 911 ] = xx
[ 1004 ] + xx [ 585 ] ; xx [ 912 ] = xx [ 1005 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 584 ) ; xx [
611 ] = pm_math_Vector3_dot_ra ( xx + 830 , xx + 913 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ; xx [ 615 ] = xx [ 580 ] * (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 913 ) + pm_math_Vector3_dot_ra ( xx
+ 581 , xx + 584 ) ) - xx [ 611 ] * xx [ 578 ] ; xx [ 584 ] = xx [ 1114 ] +
xx [ 442 ] * xx [ 1108 ] ; xx [ 585 ] = xx [ 1115 ] + xx [ 458 ] * xx [ 1108
] ; xx [ 586 ] = xx [ 1116 ] + xx [ 459 ] * xx [ 1108 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 910 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 913 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 1003 ) ; xx [ 584 ] =
xx [ 1111 ] - xx [ 655 ] * xx [ 1108 ] + xx [ 442 ] * xx [ 1106 ] + xx [ 1003
] ; xx [ 585 ] = xx [ 1112 ] + xx [ 656 ] * xx [ 1108 ] + xx [ 458 ] * xx [
1106 ] + xx [ 1004 ] ; xx [ 586 ] = xx [ 1113 ] - xx [ 699 ] * xx [ 1108 ] +
xx [ 459 ] * xx [ 1106 ] + xx [ 1005 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 489 , xx + 584 , xx + 1003 ) ; pm_math_Vector3_cross_ra ( xx + 910 , xx
+ 624 , xx + 584 ) ; xx [ 910 ] = xx [ 1003 ] + xx [ 584 ] ; xx [ 911 ] = xx
[ 1004 ] + xx [ 585 ] ; xx [ 912 ] = xx [ 1005 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 584 ) ; xx [
617 ] = pm_math_Vector3_dot_ra ( xx + 830 , xx + 913 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ; xx [ 618 ] = xx [ 580 ] * (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 913 ) + pm_math_Vector3_dot_ra ( xx
+ 581 , xx + 584 ) ) - xx [ 617 ] * xx [ 578 ] ; xx [ 584 ] = xx [ 1130 ] -
xx [ 1134 ] * xx [ 442 ] ; xx [ 585 ] = xx [ 1131 ] - xx [ 1134 ] * xx [ 458
] ; xx [ 586 ] = xx [ 1132 ] - xx [ 1134 ] * xx [ 459 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 584 , xx + 910 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 913 ) ;
pm_math_Vector3_cross_ra ( xx + 584 , xx + 648 , xx + 1003 ) ; xx [ 584 ] =
xx [ 1136 ] + xx [ 1134 ] * xx [ 655 ] - xx [ 1133 ] * xx [ 442 ] + xx [ 1003
] ; xx [ 585 ] = xx [ 1137 ] - xx [ 1134 ] * xx [ 656 ] - xx [ 1133 ] * xx [
458 ] + xx [ 1004 ] ; xx [ 586 ] = xx [ 1138 ] + xx [ 1134 ] * xx [ 699 ] -
xx [ 1133 ] * xx [ 459 ] + xx [ 1005 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 489 , xx + 584 , xx + 1003 ) ; pm_math_Vector3_cross_ra ( xx + 910 , xx
+ 624 , xx + 584 ) ; xx [ 910 ] = xx [ 1003 ] + xx [ 584 ] ; xx [ 911 ] = xx
[ 1004 ] + xx [ 585 ] ; xx [ 912 ] = xx [ 1005 ] + xx [ 586 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 910 , xx + 584 ) ; xx [
620 ] = pm_math_Vector3_dot_ra ( xx + 830 , xx + 913 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 584 ) ; xx [ 663 ] = xx [ 580 ] * (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 913 ) + pm_math_Vector3_dot_ra ( xx
+ 581 , xx + 584 ) ) - xx [ 620 ] * xx [ 578 ] ; xx [ 584 ] = xx [ 337 ] ; xx
[ 585 ] = xx [ 339 ] ; xx [ 586 ] = xx [ 341 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1159 , xx + 910 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 910 , xx + 913 ) ; xx [
1003 ] = xx [ 343 ] ; xx [ 1004 ] = xx [ 345 ] ; xx [ 1005 ] = xx [ 347 ] ;
pm_math_Vector3_cross_ra ( xx + 1159 , xx + 784 , xx + 1007 ) ; xx [ 1025 ] =
xx [ 420 ] + xx [ 1007 ] ; xx [ 1026 ] = xx [ 421 ] + xx [ 1008 ] ; xx [ 1027
] = xx [ 1164 ] + xx [ 1009 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 236
, xx + 1025 , xx + 1007 ) ; pm_math_Vector3_cross_ra ( xx + 910 , xx + 736 ,
xx + 1025 ) ; xx [ 910 ] = xx [ 1007 ] + xx [ 1025 ] ; xx [ 911 ] = xx [ 1008
] + xx [ 1026 ] ; xx [ 912 ] = xx [ 1009 ] + xx [ 1027 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 910 , xx + 1007 ) ; xx [
910 ] = xx [ 338 ] ; xx [ 911 ] = xx [ 340 ] ; xx [ 912 ] = xx [ 342 ] ; xx [
337 ] = xx [ 344 ] ; xx [ 338 ] = xx [ 346 ] ; xx [ 339 ] = xx [ 348 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1165 , xx + 340 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 340 , xx + 343 ) ;
pm_math_Vector3_cross_ra ( xx + 1165 , xx + 784 , xx + 346 ) ; xx [ 1025 ] =
xx [ 734 ] + xx [ 346 ] ; xx [ 1026 ] = xx [ 735 ] + xx [ 347 ] ; xx [ 1027 ]
= xx [ 1170 ] + xx [ 348 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 236 ,
xx + 1025 , xx + 346 ) ; pm_math_Vector3_cross_ra ( xx + 340 , xx + 736 , xx
+ 1025 ) ; xx [ 340 ] = xx [ 346 ] + xx [ 1025 ] ; xx [ 341 ] = xx [ 347 ] +
xx [ 1026 ] ; xx [ 342 ] = xx [ 348 ] + xx [ 1027 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 340 , xx + 346 ) ; xx [
340 ] = xx [ 330 ] * ( pm_math_Vector3_dot_ra ( xx + 910 , xx + 343 ) +
pm_math_Vector3_dot_ra ( xx + 337 , xx + 346 ) ) - ( pm_math_Vector3_dot_ra (
xx + 584 , xx + 343 ) + pm_math_Vector3_dot_ra ( xx + 1003 , xx + 346 ) ) *
xx [ 329 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1171 , xx +
341 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 341 , xx + 344 )
; pm_math_Vector3_cross_ra ( xx + 1171 , xx + 784 , xx + 1025 ) ; xx [ 1033 ]
= xx [ 763 ] + xx [ 1025 ] ; xx [ 1034 ] = xx [ 764 ] + xx [ 1026 ] ; xx [
1035 ] = xx [ 1176 ] + xx [ 1027 ] ; pm_math_Quaternion_inverseXform_ra ( xx
+ 236 , xx + 1033 , xx + 763 ) ; pm_math_Vector3_cross_ra ( xx + 341 , xx +
736 , xx + 1025 ) ; xx [ 341 ] = xx [ 763 ] + xx [ 1025 ] ; xx [ 342 ] = xx [
764 ] + xx [ 1026 ] ; xx [ 343 ] = xx [ 765 ] + xx [ 1027 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 341 , xx + 763 ) ; xx [
341 ] = xx [ 330 ] * ( pm_math_Vector3_dot_ra ( xx + 910 , xx + 344 ) +
pm_math_Vector3_dot_ra ( xx + 337 , xx + 763 ) ) - ( pm_math_Vector3_dot_ra (
xx + 584 , xx + 344 ) + pm_math_Vector3_dot_ra ( xx + 1003 , xx + 763 ) ) *
xx [ 329 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1177 , xx +
342 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 342 , xx + 345 )
; pm_math_Vector3_cross_ra ( xx + 1177 , xx + 784 , xx + 763 ) ; xx [ 1025 ]
= xx [ 1157 ] + xx [ 763 ] ; xx [ 1026 ] = xx [ 1158 ] + xx [ 764 ] ; xx [
1027 ] = xx [ 1182 ] + xx [ 765 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
236 , xx + 1025 , xx + 763 ) ; pm_math_Vector3_cross_ra ( xx + 342 , xx + 736
, xx + 1025 ) ; xx [ 342 ] = xx [ 763 ] + xx [ 1025 ] ; xx [ 343 ] = xx [ 764
] + xx [ 1026 ] ; xx [ 344 ] = xx [ 765 ] + xx [ 1027 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 342 , xx + 763 ) ; xx [
342 ] = xx [ 1067 ] - ( pm_math_Vector3_dot_ra ( xx + 584 , xx + 345 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 763 ) ) ; xx [ 343 ] = xx [ 329 ] *
xx [ 342 ] - xx [ 330 ] * ( xx [ 1068 ] - ( pm_math_Vector3_dot_ra ( xx + 910
, xx + 345 ) + pm_math_Vector3_dot_ra ( xx + 337 , xx + 763 ) ) ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1183 , xx + 344 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 344 , xx + 763 ) ;
pm_math_Vector3_cross_ra ( xx + 1183 , xx + 784 , xx + 1025 ) ; xx [ 1033 ] =
xx [ 1155 ] + xx [ 1025 ] ; xx [ 1034 ] = xx [ 1156 ] + xx [ 1026 ] ; xx [
1035 ] = xx [ 1188 ] + xx [ 1027 ] ; pm_math_Quaternion_inverseXform_ra ( xx
+ 236 , xx + 1033 , xx + 1025 ) ; pm_math_Vector3_cross_ra ( xx + 344 , xx +
736 , xx + 1033 ) ; xx [ 344 ] = xx [ 1025 ] + xx [ 1033 ] ; xx [ 345 ] = xx
[ 1026 ] + xx [ 1034 ] ; xx [ 346 ] = xx [ 1027 ] + xx [ 1035 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 344 , xx + 1025 ) ; xx [
344 ] = xx [ 1082 ] - ( pm_math_Vector3_dot_ra ( xx + 584 , xx + 763 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 1025 ) ) ; xx [ 345 ] = xx [ 329 ]
* xx [ 344 ] - xx [ 330 ] * ( xx [ 1083 ] - ( pm_math_Vector3_dot_ra ( xx +
910 , xx + 763 ) + pm_math_Vector3_dot_ra ( xx + 337 , xx + 1025 ) ) ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1189 , xx + 346 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 346 , xx + 763 ) ;
pm_math_Vector3_cross_ra ( xx + 1189 , xx + 784 , xx + 1025 ) ; xx [ 1033 ] =
xx [ 1180 ] + xx [ 1025 ] ; xx [ 1034 ] = xx [ 1181 ] + xx [ 1026 ] ; xx [
1035 ] = xx [ 1194 ] + xx [ 1027 ] ; pm_math_Quaternion_inverseXform_ra ( xx
+ 236 , xx + 1033 , xx + 1025 ) ; pm_math_Vector3_cross_ra ( xx + 346 , xx +
736 , xx + 1033 ) ; xx [ 346 ] = xx [ 1025 ] + xx [ 1033 ] ; xx [ 347 ] = xx
[ 1026 ] + xx [ 1034 ] ; xx [ 348 ] = xx [ 1027 ] + xx [ 1035 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 346 , xx + 1025 ) ; xx [
346 ] = pm_math_Vector3_dot_ra ( xx + 584 , xx + 763 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 1025 ) ; xx [ 347 ] = xx [ 330 ] *
( pm_math_Vector3_dot_ra ( xx + 910 , xx + 763 ) + pm_math_Vector3_dot_ra (
xx + 337 , xx + 1025 ) ) - xx [ 346 ] * xx [ 329 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 1195 , xx + 763 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 763 , xx + 1025 ) ;
pm_math_Vector3_cross_ra ( xx + 1195 , xx + 784 , xx + 1033 ) ; xx [ 1036 ] =
xx [ 1193 ] + xx [ 1033 ] ; xx [ 1037 ] = xx [ 1198 ] + xx [ 1034 ] ; xx [
1038 ] = xx [ 1200 ] + xx [ 1035 ] ; pm_math_Quaternion_inverseXform_ra ( xx
+ 236 , xx + 1036 , xx + 1033 ) ; pm_math_Vector3_cross_ra ( xx + 763 , xx +
736 , xx + 1036 ) ; xx [ 763 ] = xx [ 1033 ] + xx [ 1036 ] ; xx [ 764 ] = xx
[ 1034 ] + xx [ 1037 ] ; xx [ 765 ] = xx [ 1035 ] + xx [ 1038 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 763 , xx + 1033 ) ; xx [
348 ] = pm_math_Vector3_dot_ra ( xx + 584 , xx + 1025 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 1033 ) ; xx [ 420 ] = xx [ 330 ] *
( pm_math_Vector3_dot_ra ( xx + 910 , xx + 1025 ) + pm_math_Vector3_dot_ra (
xx + 337 , xx + 1033 ) ) - xx [ 348 ] * xx [ 329 ] ; xx [ 421 ] = xx [ 939 ]
* xx [ 1054 ] + xx [ 958 ] * xx [ 589 ] ; xx [ 734 ] = - ( xx [ 939 ] * xx [
1076 ] + xx [ 958 ] * xx [ 591 ] ) ; xx [ 735 ] = - ( xx [ 939 ] * xx [ 1092
] + xx [ 958 ] * xx [ 611 ] ) ; xx [ 763 ] = xx [ 939 ] * xx [ 1108 ] - xx [
958 ] * xx [ 617 ] ; xx [ 764 ] = - ( xx [ 939 ] * xx [ 1134 ] + xx [ 958 ] *
xx [ 620 ] ) ; xx [ 765 ] = - ( xx [ 1042 ] * xx [ 1076 ] + xx [ 1006 ] * xx
[ 591 ] ) ; xx [ 591 ] = - ( xx [ 1042 ] * xx [ 1092 ] + xx [ 1006 ] * xx [
611 ] ) ; xx [ 611 ] = xx [ 1042 ] * xx [ 1108 ] - xx [ 1006 ] * xx [ 617 ] ;
xx [ 617 ] = - ( xx [ 1042 ] * xx [ 1134 ] + xx [ 1006 ] * xx [ 620 ] ) ; xx
[ 620 ] = xx [ 1061 ] * xx [ 344 ] ; xx [ 808 ] = - ( xx [ 346 ] * xx [ 1061
] ) ; xx [ 809 ] = - ( xx [ 348 ] * xx [ 1061 ] ) ; xx [ 820 ] = - ( xx [ 346
] * xx [ 1081 ] ) ; xx [ 346 ] = - ( xx [ 348 ] * xx [ 1081 ] ) ; xx [ 348 ]
= - ( xx [ 1105 ] * xx [ 1134 ] ) ; xx [ 1247 ] = xx [ 63 ] * ( xx [ 129 ] -
( pm_math_Vector3_dot_ra ( xx + 172 , xx + 268 ) + pm_math_Vector3_dot_ra (
xx + 271 , xx + 274 ) ) ) - xx [ 64 ] * ( xx [ 130 ] - (
pm_math_Vector3_dot_ra ( xx + 367 , xx + 268 ) + pm_math_Vector3_dot_ra ( xx
+ 370 , xx + 274 ) ) ) + xx [ 81 ] * ( xx [ 96 ] - ( xx [ 201 ] * xx [ 374 ]
+ xx [ 207 ] * xx [ 375 ] ) ) ; xx [ 1248 ] = xx [ 158 ] ; xx [ 1249 ] = xx [
392 ] ; xx [ 1250 ] = xx [ 572 ] ; xx [ 1251 ] = xx [ 613 ] ; xx [ 1252 ] =
xx [ 669 ] ; xx [ 1253 ] = xx [ 803 ] ; xx [ 1254 ] = xx [ 884 ] ; xx [ 1255
] = xx [ 909 ] ; xx [ 1256 ] = xx [ 923 ] ; xx [ 1257 ] = xx [ 1029 ] ; xx [
1258 ] = xx [ 1044 ] ; xx [ 1259 ] = xx [ 1077 ] ; xx [ 1260 ] = xx [ 1093 ]
; xx [ 1261 ] = xx [ 1107 ] ; xx [ 1262 ] = xx [ 1135 ] ; xx [ 1263 ] = xx [
158 ] ; xx [ 1264 ] = xx [ 97 ] * ( xx [ 102 ] + pm_math_Vector3_dot_ra ( xx
+ 1139 , xx + 1144 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 1150 ) ) ;
xx [ 1265 ] = xx [ 41 ] ; xx [ 1266 ] = xx [ 55 ] ; xx [ 1267 ] = xx [ 62 ] ;
xx [ 1268 ] = xx [ 106 ] ; xx [ 1269 ] = xx [ 139 ] ; xx [ 1270 ] = xx [ 203
] ; xx [ 1271 ] = xx [ 211 ] ; xx [ 1272 ] = xx [ 433 ] ; xx [ 1273 ] = xx [
448 ] ; xx [ 1274 ] = xx [ 518 ] ; xx [ 1275 ] = xx [ 616 ] ; xx [ 1276 ] =
xx [ 621 ] ; xx [ 1277 ] = xx [ 1062 ] ; xx [ 1278 ] = xx [ 1123 ] ; xx [
1279 ] = xx [ 392 ] ; xx [ 1280 ] = xx [ 41 ] ; xx [ 1281 ] = xx [ 167 ] * xx
[ 373 ] ; xx [ 1282 ] = xx [ 1201 ] ; xx [ 1283 ] = xx [ 1202 ] ; xx [ 1284 ]
= xx [ 1203 ] ; xx [ 1285 ] = xx [ 1204 ] ; xx [ 1286 ] = xx [ 1205 ] ; xx [
1287 ] = xx [ 1206 ] ; xx [ 1288 ] = xx [ 1207 ] ; xx [ 1289 ] = xx [ 1208 ]
; xx [ 1290 ] = xx [ 1209 ] ; xx [ 1291 ] = xx [ 1210 ] ; xx [ 1292 ] = xx [
1211 ] ; xx [ 1293 ] = xx [ 1212 ] ; xx [ 1294 ] = xx [ 1213 ] ; xx [ 1295 ]
= xx [ 572 ] ; xx [ 1296 ] = xx [ 55 ] ; xx [ 1297 ] = xx [ 1201 ] ; xx [
1298 ] = xx [ 444 ] * xx [ 520 ] ; xx [ 1299 ] = xx [ 1214 ] ; xx [ 1300 ] =
xx [ 1215 ] ; xx [ 1301 ] = xx [ 1216 ] ; xx [ 1302 ] = xx [ 1217 ] ; xx [
1303 ] = xx [ 1218 ] ; xx [ 1304 ] = xx [ 1219 ] ; xx [ 1305 ] = xx [ 1220 ]
; xx [ 1306 ] = xx [ 1221 ] ; xx [ 1307 ] = xx [ 1222 ] ; xx [ 1308 ] = xx [
1223 ] ; xx [ 1309 ] = xx [ 1224 ] ; xx [ 1310 ] = xx [ 1225 ] ; xx [ 1311 ]
= xx [ 613 ] ; xx [ 1312 ] = xx [ 62 ] ; xx [ 1313 ] = xx [ 1202 ] ; xx [
1314 ] = xx [ 1214 ] ; xx [ 1315 ] = xx [ 576 ] * xx [ 622 ] + xx [ 577 ] *
xx [ 612 ] ; xx [ 1316 ] = xx [ 1226 ] ; xx [ 1317 ] = xx [ 654 ] ; xx [ 1318
] = xx [ 668 ] ; xx [ 1319 ] = xx [ 801 ] ; xx [ 1320 ] = xx [ 802 ] ; xx [
1321 ] = xx [ 843 ] ; xx [ 1322 ] = xx [ 844 ] ; xx [ 1323 ] = xx [ 1227 ] ;
xx [ 1324 ] = xx [ 1228 ] ; xx [ 1325 ] = xx [ 1229 ] ; xx [ 1326 ] = xx [
1230 ] ; xx [ 1327 ] = xx [ 669 ] ; xx [ 1328 ] = xx [ 106 ] ; xx [ 1329 ] =
xx [ 1203 ] ; xx [ 1330 ] = xx [ 1215 ] ; xx [ 1331 ] = xx [ 1226 ] ; xx [
1332 ] = xx [ 57 ] * ( xx [ 634 ] + pm_math_Vector3_dot_ra ( xx + 1231 , xx +
1234 ) ) ; xx [ 1333 ] = xx [ 179 ] ; xx [ 1334 ] = xx [ 184 ] ; xx [ 1335 ]
= xx [ 635 ] ; xx [ 1336 ] = xx [ 640 ] ; xx [ 1337 ] = xx [ 641 ] ; xx [
1338 ] = xx [ 793 ] ; xx [ 1339 ] = xx [ 794 ] ; xx [ 1340 ] = xx [ 795 ] ;
xx [ 1341 ] = xx [ 810 ] ; xx [ 1342 ] = xx [ 819 ] ; xx [ 1343 ] = xx [ 803
] ; xx [ 1344 ] = xx [ 139 ] ; xx [ 1345 ] = xx [ 1204 ] ; xx [ 1346 ] = xx [
1216 ] ; xx [ 1347 ] = xx [ 654 ] ; xx [ 1348 ] = xx [ 179 ] ; xx [ 1349 ] =
xx [ 761 ] * ( xx [ 799 ] - ( pm_math_Vector3_dot_ra ( xx + 775 , xx + 1237 )
+ pm_math_Vector3_dot_ra ( xx + 1240 , xx + 724 ) ) ) - xx [ 760 ] * ( xx [
800 ] - ( pm_math_Vector3_dot_ra ( xx + 191 , xx + 1237 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 724 ) ) ) ; xx [ 1350 ] = xx [ 418 ]
; xx [ 1351 ] = xx [ 419 ] ; xx [ 1352 ] = xx [ 733 ] ; xx [ 1353 ] = xx [
762 ] ; xx [ 1354 ] = xx [ 1153 ] ; xx [ 1355 ] = xx [ 1154 ] ; xx [ 1356 ] =
xx [ 1175 ] ; xx [ 1357 ] = xx [ 1192 ] ; xx [ 1358 ] = xx [ 1126 ] ; xx [
1359 ] = xx [ 884 ] ; xx [ 1360 ] = xx [ 203 ] ; xx [ 1361 ] = xx [ 1205 ] ;
xx [ 1362 ] = xx [ 1217 ] ; xx [ 1363 ] = xx [ 668 ] ; xx [ 1364 ] = xx [ 184
] ; xx [ 1365 ] = xx [ 418 ] ; xx [ 1366 ] = xx [ 198 ] * xx [ 197 ] + xx [
59 ] * xx [ 111 ] - xx [ 181 ] * xx [ 424 ] + xx [ 838 ] * ( xx [ 839 ] + xx
[ 165 ] * xx [ 822 ] + xx [ 195 ] * xx [ 823 ] ) ; xx [ 1367 ] = xx [ 212 ] ;
xx [ 1368 ] = xx [ 209 ] ; xx [ 1369 ] = xx [ 222 ] ; xx [ 1370 ] = xx [ 422
] ; xx [ 1371 ] = xx [ 423 ] ; xx [ 1372 ] = xx [ 425 ] ; xx [ 1373 ] = xx [
426 ] ; xx [ 1374 ] = xx [ 440 ] ; xx [ 1375 ] = xx [ 909 ] ; xx [ 1376 ] =
xx [ 211 ] ; xx [ 1377 ] = xx [ 1206 ] ; xx [ 1378 ] = xx [ 1218 ] ; xx [
1379 ] = xx [ 801 ] ; xx [ 1380 ] = xx [ 635 ] ; xx [ 1381 ] = xx [ 419 ] ;
xx [ 1382 ] = xx [ 212 ] ; xx [ 1383 ] = xx [ 578 ] * ( xx [ 903 ] - (
pm_math_Vector3_dot_ra ( xx + 830 , xx + 1000 ) + pm_math_Vector3_dot_ra ( xx
+ 1030 , xx + 888 ) ) ) - xx [ 580 ] * ( xx [ 904 ] - (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 1000 ) + pm_math_Vector3_dot_ra ( xx
+ 581 , xx + 888 ) ) ) ; xx [ 1384 ] = xx [ 452 ] ; xx [ 1385 ] = xx [ 588 ]
; xx [ 1386 ] = xx [ 590 ] ; xx [ 1387 ] = xx [ 592 ] ; xx [ 1388 ] = xx [
615 ] ; xx [ 1389 ] = xx [ 618 ] ; xx [ 1390 ] = xx [ 663 ] ; xx [ 1391 ] =
xx [ 923 ] ; xx [ 1392 ] = xx [ 433 ] ; xx [ 1393 ] = xx [ 1207 ] ; xx [ 1394
] = xx [ 1219 ] ; xx [ 1395 ] = xx [ 802 ] ; xx [ 1396 ] = xx [ 640 ] ; xx [
1397 ] = xx [ 733 ] ; xx [ 1398 ] = xx [ 209 ] ; xx [ 1399 ] = xx [ 452 ] ;
xx [ 1400 ] = xx [ 329 ] * ( xx [ 921 ] - ( pm_math_Vector3_dot_ra ( xx + 584
, xx + 913 ) + pm_math_Vector3_dot_ra ( xx + 1003 , xx + 1007 ) ) ) - xx [
330 ] * ( xx [ 922 ] - ( pm_math_Vector3_dot_ra ( xx + 910 , xx + 913 ) +
pm_math_Vector3_dot_ra ( xx + 337 , xx + 1007 ) ) ) ; xx [ 1401 ] = xx [ 340
] ; xx [ 1402 ] = xx [ 341 ] ; xx [ 1403 ] = xx [ 343 ] ; xx [ 1404 ] = xx [
345 ] ; xx [ 1405 ] = xx [ 347 ] ; xx [ 1406 ] = xx [ 420 ] ; xx [ 1407 ] =
xx [ 1029 ] ; xx [ 1408 ] = xx [ 448 ] ; xx [ 1409 ] = xx [ 1208 ] ; xx [
1410 ] = xx [ 1220 ] ; xx [ 1411 ] = xx [ 843 ] ; xx [ 1412 ] = xx [ 641 ] ;
xx [ 1413 ] = xx [ 762 ] ; xx [ 1414 ] = xx [ 222 ] ; xx [ 1415 ] = xx [ 588
] ; xx [ 1416 ] = xx [ 340 ] ; xx [ 1417 ] = xx [ 939 ] * xx [ 1024 ] + xx [
958 ] * xx [ 587 ] ; xx [ 1418 ] = xx [ 421 ] ; xx [ 1419 ] = xx [ 734 ] ; xx
[ 1420 ] = xx [ 735 ] ; xx [ 1421 ] = xx [ 763 ] ; xx [ 1422 ] = xx [ 764 ] ;
xx [ 1423 ] = xx [ 1044 ] ; xx [ 1424 ] = xx [ 518 ] ; xx [ 1425 ] = xx [
1209 ] ; xx [ 1426 ] = xx [ 1221 ] ; xx [ 1427 ] = xx [ 844 ] ; xx [ 1428 ] =
xx [ 793 ] ; xx [ 1429 ] = xx [ 1153 ] ; xx [ 1430 ] = xx [ 422 ] ; xx [ 1431
] = xx [ 590 ] ; xx [ 1432 ] = xx [ 341 ] ; xx [ 1433 ] = xx [ 421 ] ; xx [
1434 ] = xx [ 1042 ] * xx [ 1054 ] + xx [ 1006 ] * xx [ 589 ] ; xx [ 1435 ] =
xx [ 765 ] ; xx [ 1436 ] = xx [ 591 ] ; xx [ 1437 ] = xx [ 611 ] ; xx [ 1438
] = xx [ 617 ] ; xx [ 1439 ] = xx [ 1077 ] ; xx [ 1440 ] = xx [ 616 ] ; xx [
1441 ] = xx [ 1210 ] ; xx [ 1442 ] = xx [ 1222 ] ; xx [ 1443 ] = xx [ 1227 ]
; xx [ 1444 ] = xx [ 794 ] ; xx [ 1445 ] = xx [ 1154 ] ; xx [ 1446 ] = xx [
423 ] ; xx [ 1447 ] = xx [ 592 ] ; xx [ 1448 ] = xx [ 343 ] ; xx [ 1449 ] =
xx [ 734 ] ; xx [ 1450 ] = xx [ 765 ] ; xx [ 1451 ] = xx [ 1061 ] * xx [ 342
] ; xx [ 1452 ] = xx [ 620 ] ; xx [ 1453 ] = xx [ 808 ] ; xx [ 1454 ] = xx [
809 ] ; xx [ 1455 ] = xx [ 1093 ] ; xx [ 1456 ] = xx [ 621 ] ; xx [ 1457 ] =
xx [ 1211 ] ; xx [ 1458 ] = xx [ 1223 ] ; xx [ 1459 ] = xx [ 1228 ] ; xx [
1460 ] = xx [ 795 ] ; xx [ 1461 ] = xx [ 1175 ] ; xx [ 1462 ] = xx [ 425 ] ;
xx [ 1463 ] = xx [ 615 ] ; xx [ 1464 ] = xx [ 345 ] ; xx [ 1465 ] = xx [ 735
] ; xx [ 1466 ] = xx [ 591 ] ; xx [ 1467 ] = xx [ 620 ] ; xx [ 1468 ] = xx [
1081 ] * xx [ 344 ] ; xx [ 1469 ] = xx [ 820 ] ; xx [ 1470 ] = xx [ 346 ] ;
xx [ 1471 ] = xx [ 1107 ] ; xx [ 1472 ] = xx [ 1062 ] ; xx [ 1473 ] = xx [
1212 ] ; xx [ 1474 ] = xx [ 1224 ] ; xx [ 1475 ] = xx [ 1229 ] ; xx [ 1476 ]
= xx [ 810 ] ; xx [ 1477 ] = xx [ 1192 ] ; xx [ 1478 ] = xx [ 426 ] ; xx [
1479 ] = xx [ 618 ] ; xx [ 1480 ] = xx [ 347 ] ; xx [ 1481 ] = xx [ 763 ] ;
xx [ 1482 ] = xx [ 611 ] ; xx [ 1483 ] = xx [ 808 ] ; xx [ 1484 ] = xx [ 820
] ; xx [ 1485 ] = xx [ 1105 ] * xx [ 1108 ] ; xx [ 1486 ] = xx [ 348 ] ; xx [
1487 ] = xx [ 1135 ] ; xx [ 1488 ] = xx [ 1123 ] ; xx [ 1489 ] = xx [ 1213 ]
; xx [ 1490 ] = xx [ 1225 ] ; xx [ 1491 ] = xx [ 1230 ] ; xx [ 1492 ] = xx [
819 ] ; xx [ 1493 ] = xx [ 1126 ] ; xx [ 1494 ] = xx [ 440 ] ; xx [ 1495 ] =
xx [ 663 ] ; xx [ 1496 ] = xx [ 420 ] ; xx [ 1497 ] = xx [ 764 ] ; xx [ 1498
] = xx [ 617 ] ; xx [ 1499 ] = xx [ 809 ] ; xx [ 1500 ] = xx [ 346 ] ; xx [
1501 ] = xx [ 348 ] ; xx [ 1502 ] = xx [ 1110 ] * xx [ 1199 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 46 , xx + 268 ) ; xx [
41 ] = xx [ 442 ] * state [ 15 ] ; xx [ 55 ] = xx [ 268 ] + xx [ 41 ] ; xx [
62 ] = xx [ 458 ] * state [ 15 ] ; xx [ 96 ] = xx [ 269 ] + xx [ 62 ] ; xx [
102 ] = xx [ 459 ] * state [ 15 ] ; xx [ 106 ] = xx [ 270 ] + xx [ 102 ] ; xx
[ 274 ] = xx [ 55 ] ; xx [ 275 ] = xx [ 96 ] ; xx [ 276 ] = xx [ 106 ] ; xx [
340 ] = xx [ 55 ] * xx [ 707 ] ; xx [ 341 ] = xx [ 96 ] * xx [ 633 ] ; xx [
342 ] = xx [ 106 ] * xx [ 643 ] ; pm_math_Vector3_cross_ra ( xx + 274 , xx +
340 , xx + 343 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 274 ,
xx + 340 ) ; xx [ 111 ] = xx [ 460 ] * inputDot [ 1 ] ; xx [ 129 ] = xx [ 340
] - xx [ 111 ] ; xx [ 130 ] = xx [ 483 ] * inputDot [ 1 ] ; xx [ 139 ] = xx [
341 ] + xx [ 130 ] ; xx [ 158 ] = xx [ 484 ] * inputDot [ 1 ] ; xx [ 179 ] =
xx [ 342 ] + xx [ 158 ] ; xx [ 346 ] = xx [ 129 ] ; xx [ 347 ] = xx [ 139 ] ;
xx [ 348 ] = xx [ 179 ] ; xx [ 373 ] = xx [ 129 ] * xx [ 708 ] ; xx [ 374 ] =
xx [ 139 ] * xx [ 627 ] ; xx [ 375 ] = xx [ 179 ] * xx [ 628 ] ;
pm_math_Vector3_cross_ra ( xx + 346 , xx + 373 , xx + 418 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 512 , xx + 346 , xx + 373 ) ; xx [
184 ] = xx [ 235 ] * inputDot [ 2 ] ; xx [ 197 ] = xx [ 373 ] - xx [ 184 ] ;
xx [ 203 ] = xx [ 256 ] * inputDot [ 2 ] ; xx [ 209 ] = xx [ 374 ] + xx [ 203
] ; xx [ 211 ] = xx [ 488 ] * inputDot [ 2 ] ; xx [ 212 ] = xx [ 375 ] + xx [
211 ] ; xx [ 421 ] = xx [ 197 ] ; xx [ 422 ] = xx [ 209 ] ; xx [ 423 ] = xx [
212 ] ; xx [ 424 ] = xx [ 197 ] * xx [ 709 ] ; xx [ 425 ] = xx [ 209 ] * xx [
710 ] ; xx [ 426 ] = xx [ 212 ] * xx [ 711 ] ; pm_math_Vector3_cross_ra ( xx
+ 421 , xx + 424 , xx + 587 ) ; xx [ 421 ] = - 3.865146756434337e-8 ; xx [
422 ] = - 5.289307885790401e-3 ; xx [ 423 ] = - 3.071851004035524e-7 ; xx [
222 ] = 0.3301830145835319 ; xx [ 392 ] = 8.736479765048067e-7 ; xx [ 424 ] =
xx [ 392 ] * input [ 4 ] ; xx [ 425 ] = 0.9439169332518749 ; xx [ 426 ] =
3.787473687847524e-7 ; xx [ 433 ] = xx [ 426 ] * input [ 4 ] ; xx [ 440 ] =
xx [ 2 ] * ( xx [ 222 ] * xx [ 424 ] - xx [ 425 ] * xx [ 433 ] ) ; xx [ 448 ]
= input [ 4 ] - ( xx [ 426 ] * xx [ 433 ] + xx [ 392 ] * xx [ 424 ] ) * xx [
2 ] ; xx [ 452 ] = ( xx [ 425 ] * xx [ 424 ] + xx [ 222 ] * xx [ 433 ] ) * xx
[ 2 ] ; xx [ 590 ] = xx [ 440 ] ; xx [ 591 ] = xx [ 448 ] ; xx [ 592 ] = xx [
452 ] ; pm_math_Vector3_cross_ra ( xx + 421 , xx + 590 , xx + 611 ) ; xx [
590 ] = - xx [ 184 ] ; xx [ 591 ] = xx [ 203 ] ; xx [ 592 ] = xx [ 211 ] ;
pm_math_Vector3_cross_ra ( xx + 373 , xx + 590 , xx + 615 ) ; xx [ 424 ] = xx
[ 615 ] - xx [ 235 ] * inputDdot [ 2 ] ; xx [ 433 ] = xx [ 616 ] + xx [ 256 ]
* inputDdot [ 2 ] ; xx [ 518 ] = xx [ 617 ] + xx [ 488 ] * inputDdot [ 2 ] ;
xx [ 615 ] = xx [ 587 ] - xx [ 611 ] + xx [ 424 ] * xx [ 709 ] ; xx [ 616 ] =
xx [ 588 ] - xx [ 612 ] + xx [ 433 ] * xx [ 710 ] ; xx [ 617 ] = xx [ 589 ] -
xx [ 613 ] + xx [ 518 ] * xx [ 711 ] ; pm_math_Quaternion_xform_ra ( xx + 512
, xx + 615 , xx + 587 ) ; xx [ 611 ] = xx [ 373 ] + xx [ 197 ] ; xx [ 612 ] =
xx [ 374 ] + xx [ 209 ] ; xx [ 613 ] = xx [ 375 ] + xx [ 212 ] ; xx [ 197 ] =
2.971385142511557e-7 ; xx [ 209 ] = 1.158326953518261e-13 ; xx [ 212 ] =
3.938183989403888e-8 ; xx [ 373 ] = xx [ 197 ] * inputDot [ 2 ] ; xx [ 374 ]
= xx [ 209 ] * inputDot [ 2 ] ; xx [ 375 ] = - ( xx [ 212 ] * inputDot [ 2 ]
) ; pm_math_Vector3_cross_ra ( xx + 611 , xx + 373 , xx + 615 ) ;
pm_math_Vector3_cross_ra ( xx + 346 , xx + 636 , xx + 373 ) ;
pm_math_Vector3_cross_ra ( xx + 346 , xx + 373 , xx + 611 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 512 , xx + 611 , xx + 373 ) ; xx [
611 ] = ( xx [ 615 ] + xx [ 373 ] + xx [ 197 ] * inputDdot [ 2 ] ) * xx [ 232
] - xx [ 440 ] ; xx [ 612 ] = ( xx [ 616 ] + xx [ 374 ] + xx [ 209 ] *
inputDdot [ 2 ] ) * xx [ 232 ] - xx [ 448 ] ; xx [ 613 ] = ( xx [ 617 ] + xx
[ 375 ] - xx [ 212 ] * inputDdot [ 2 ] ) * xx [ 232 ] - xx [ 452 ] ;
pm_math_Quaternion_xform_ra ( xx + 512 , xx + 611 , xx + 373 ) ;
pm_math_Vector3_cross_ra ( xx + 636 , xx + 373 , xx + 611 ) ; xx [ 440 ] = xx
[ 418 ] + xx [ 587 ] + xx [ 611 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
545 , xx + 346 , xx + 615 ) ; xx [ 448 ] = xx [ 30 ] * state [ 19 ] ; xx [
452 ] = xx [ 615 ] + xx [ 448 ] ; xx [ 488 ] = xx [ 511 ] * state [ 19 ] ; xx
[ 520 ] = xx [ 616 ] - xx [ 488 ] ; xx [ 572 ] = xx [ 531 ] * state [ 19 ] ;
xx [ 618 ] = xx [ 617 ] - xx [ 572 ] ; xx [ 620 ] = xx [ 452 ] ; xx [ 621 ] =
xx [ 520 ] ; xx [ 622 ] = xx [ 618 ] ; xx [ 634 ] = xx [ 452 ] * xx [ 639 ] ;
xx [ 635 ] = xx [ 520 ] * xx [ 642 ] ; xx [ 636 ] = xx [ 618 ] * xx [ 644 ] ;
pm_math_Vector3_cross_ra ( xx + 620 , xx + 634 , xx + 724 ) ; xx [ 620 ] = xx
[ 448 ] ; xx [ 621 ] = - xx [ 488 ] ; xx [ 622 ] = - xx [ 572 ] ;
pm_math_Vector3_cross_ra ( xx + 615 , xx + 620 , xx + 634 ) ; xx [ 627 ] = xx
[ 724 ] + xx [ 639 ] * xx [ 634 ] ; xx [ 733 ] = xx [ 30 ] ; xx [ 734 ] = -
xx [ 511 ] ; xx [ 735 ] = - xx [ 531 ] ; xx [ 628 ] = xx [ 725 ] + xx [ 642 ]
* xx [ 635 ] ; xx [ 637 ] = xx [ 726 ] + xx [ 644 ] * xx [ 636 ] ; xx [ 724 ]
= xx [ 627 ] ; xx [ 725 ] = xx [ 628 ] ; xx [ 726 ] = xx [ 637 ] ; xx [ 762 ]
= xx [ 615 ] + xx [ 452 ] ; xx [ 763 ] = xx [ 616 ] + xx [ 520 ] ; xx [ 764 ]
= xx [ 617 ] + xx [ 618 ] ; xx [ 615 ] = xx [ 569 ] * state [ 19 ] + xx [ 30
] * state [ 20 ] ; xx [ 616 ] = xx [ 570 ] * state [ 19 ] - xx [ 511 ] *
state [ 20 ] ; xx [ 617 ] = - ( xx [ 571 ] * state [ 19 ] + xx [ 531 ] *
state [ 20 ] ) ; pm_math_Vector3_cross_ra ( xx + 762 , xx + 615 , xx + 569 )
; pm_math_Vector3_cross_ra ( xx + 346 , xx + 624 , xx + 615 ) ;
pm_math_Vector3_cross_ra ( xx + 346 , xx + 615 , xx + 762 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 762 , xx + 346 ) ; xx [
452 ] = ( xx [ 569 ] + xx [ 346 ] ) * xx [ 258 ] ; xx [ 520 ] = ( xx [ 570 ]
+ xx [ 347 ] ) * xx [ 258 ] ; xx [ 346 ] = ( xx [ 571 ] + xx [ 348 ] ) * xx [
258 ] ; xx [ 569 ] = xx [ 452 ] ; xx [ 570 ] = xx [ 520 ] ; xx [ 571 ] = xx [
346 ] ; xx [ 347 ] = pm_math_Vector3_dot_ra ( xx + 733 , xx + 724 ) +
pm_math_Vector3_dot_ra ( xx + 891 , xx + 569 ) ; xx [ 348 ] =
pm_math_Vector3_dot_ra ( xx + 733 , xx + 569 ) ; xx [ 569 ] = - xx [ 347 ] ;
xx [ 570 ] = - xx [ 348 ] ; solveSymmetricPosDef ( xx + 560 , xx + 569 , 2 ,
1 , xx + 615 , xx + 617 ) ; xx [ 569 ] = xx [ 627 ] + xx [ 296 ] * xx [ 615 ]
; xx [ 570 ] = xx [ 628 ] - xx [ 532 ] * xx [ 615 ] ; xx [ 571 ] = xx [ 637 ]
- xx [ 535 ] * xx [ 615 ] ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 569
, xx + 724 ) ; xx [ 569 ] = xx [ 452 ] + xx [ 439 ] * xx [ 615 ] + xx [ 564 ]
* xx [ 616 ] ; xx [ 570 ] = xx [ 520 ] + xx [ 565 ] * xx [ 615 ] - xx [ 566 ]
* xx [ 616 ] ; xx [ 571 ] = xx [ 346 ] - xx [ 567 ] * xx [ 615 ] - xx [ 568 ]
* xx [ 616 ] ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 569 , xx + 733 )
; pm_math_Vector3_cross_ra ( xx + 624 , xx + 733 , xx + 569 ) ; xx [ 762 ] =
- xx [ 111 ] ; xx [ 763 ] = xx [ 130 ] ; xx [ 764 ] = xx [ 158 ] ;
pm_math_Vector3_cross_ra ( xx + 340 , xx + 762 , xx + 793 ) ; xx [ 617 ] = xx
[ 793 ] - xx [ 460 ] * inputDdot [ 1 ] ; xx [ 618 ] = xx [ 794 ] + xx [ 483 ]
* inputDdot [ 1 ] ; xx [ 638 ] = xx [ 795 ] + xx [ 484 ] * inputDdot [ 1 ] ;
xx [ 793 ] = xx [ 617 ] ; xx [ 794 ] = xx [ 618 ] ; xx [ 795 ] = xx [ 638 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 739 , xx + 793 , xx + 799 ) ; xx [ 739 ] =
xx [ 340 ] + xx [ 129 ] ; xx [ 740 ] = xx [ 341 ] + xx [ 139 ] ; xx [ 741 ] =
xx [ 342 ] + xx [ 179 ] ; xx [ 129 ] = 1.439701931685288e-6 ; xx [ 139 ] =
0.01494182793334928 ; xx [ 179 ] = 1.536145228271068e-6 ; xx [ 340 ] = xx [
129 ] * inputDot [ 1 ] ; xx [ 341 ] = xx [ 139 ] * inputDot [ 1 ] ; xx [ 342
] = xx [ 179 ] * inputDot [ 1 ] ; pm_math_Vector3_cross_ra ( xx + 739 , xx +
340 , xx + 742 ) ; pm_math_Vector3_cross_ra ( xx + 274 , xx + 648 , xx + 340
) ; pm_math_Vector3_cross_ra ( xx + 274 , xx + 340 , xx + 739 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 739 , xx + 274 ) ; xx [
340 ] = xx [ 742 ] + xx [ 274 ] + xx [ 129 ] * inputDdot [ 1 ] ; xx [ 341 ] =
xx [ 743 ] + xx [ 275 ] + xx [ 139 ] * inputDdot [ 1 ] ; xx [ 274 ] = xx [
744 ] + xx [ 276 ] + xx [ 179 ] * inputDdot [ 1 ] ; xx [ 739 ] = xx [ 340 ] ;
xx [ 740 ] = xx [ 341 ] ; xx [ 741 ] = xx [ 274 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 536 , xx + 739 , xx + 742 ) ; xx [ 275 ] =
xx [ 799 ] + xx [ 742 ] ; xx [ 276 ] = xx [ 419 ] + xx [ 588 ] + xx [ 612 ] ;
xx [ 342 ] = xx [ 800 ] + xx [ 743 ] ; xx [ 418 ] = xx [ 420 ] + xx [ 589 ] +
xx [ 613 ] ; xx [ 419 ] = xx [ 801 ] + xx [ 744 ] ; xx [ 587 ] = xx [ 440 ] +
xx [ 724 ] + xx [ 569 ] + xx [ 275 ] ; xx [ 588 ] = xx [ 276 ] + xx [ 725 ] +
xx [ 570 ] + xx [ 342 ] ; xx [ 589 ] = xx [ 418 ] + xx [ 726 ] + xx [ 571 ] +
xx [ 419 ] ; pm_math_Quaternion_xform_ra ( xx + 489 , xx + 587 , xx + 569 ) ;
pm_math_Matrix3x3_transposeXform_ra ( xx + 536 , xx + 793 , xx + 587 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 593 , xx + 739 , xx + 536 ) ; xx [ 420 ] =
xx [ 587 ] + xx [ 536 ] ; xx [ 539 ] = xx [ 588 ] + xx [ 537 ] ; xx [ 536 ] =
xx [ 589 ] + xx [ 538 ] ; xx [ 540 ] = xx [ 373 ] + xx [ 733 ] + xx [ 420 ] ;
xx [ 541 ] = xx [ 374 ] + xx [ 734 ] + xx [ 539 ] ; xx [ 542 ] = xx [ 375 ] +
xx [ 735 ] + xx [ 536 ] ; pm_math_Quaternion_xform_ra ( xx + 489 , xx + 540 ,
xx + 587 ) ; pm_math_Vector3_cross_ra ( xx + 648 , xx + 587 , xx + 540 ) ; xx
[ 593 ] = xx [ 41 ] ; xx [ 594 ] = xx [ 62 ] ; xx [ 595 ] = xx [ 102 ] ;
pm_math_Vector3_cross_ra ( xx + 268 , xx + 593 , xx + 596 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 602 , xx + 596 , xx + 599 ) ; xx [ 602 ] =
xx [ 268 ] + xx [ 55 ] ; xx [ 603 ] = xx [ 269 ] + xx [ 96 ] ; xx [ 604 ] =
xx [ 270 ] + xx [ 106 ] ; xx [ 268 ] = xx [ 442 ] * state [ 16 ] - xx [ 655 ]
* state [ 15 ] ; xx [ 269 ] = xx [ 656 ] * state [ 15 ] + xx [ 458 ] * state
[ 16 ] ; xx [ 270 ] = xx [ 459 ] * state [ 16 ] - xx [ 699 ] * state [ 15 ] ;
pm_math_Vector3_cross_ra ( xx + 602 , xx + 268 , xx + 605 ) ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 651 , xx + 268 ) ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 268 , xx + 602 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 602 , xx + 268 ) ; xx [
55 ] = xx [ 605 ] + xx [ 268 ] ; xx [ 96 ] = xx [ 606 ] + xx [ 269 ] ; xx [
106 ] = xx [ 607 ] + xx [ 270 ] ; xx [ 268 ] = xx [ 55 ] ; xx [ 269 ] = xx [
96 ] ; xx [ 270 ] = xx [ 106 ] ; pm_math_Matrix3x3_xform_ra ( xx + 675 , xx +
268 , xx + 602 ) ; xx [ 537 ] = xx [ 599 ] + xx [ 602 ] ; xx [ 538 ] = xx [
343 ] + xx [ 569 ] + xx [ 540 ] + xx [ 537 ] ; xx [ 543 ] = xx [ 600 ] + xx [
603 ] ; xx [ 544 ] = xx [ 344 ] + xx [ 570 ] + xx [ 541 ] + xx [ 543 ] ; xx [
540 ] = xx [ 601 ] + xx [ 604 ] ; xx [ 541 ] = xx [ 345 ] + xx [ 571 ] + xx [
542 ] + xx [ 540 ] ; xx [ 569 ] = xx [ 538 ] ; xx [ 570 ] = xx [ 544 ] ; xx [
571 ] = xx [ 541 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 675 , xx +
596 , xx + 599 ) ; pm_math_Matrix3x3_xform_ra ( xx + 690 , xx + 268 , xx +
602 ) ; xx [ 268 ] = xx [ 599 ] + xx [ 602 ] ; xx [ 269 ] = xx [ 587 ] + xx [
268 ] ; xx [ 270 ] = xx [ 600 ] + xx [ 603 ] ; xx [ 542 ] = xx [ 588 ] + xx [
270 ] ; xx [ 587 ] = xx [ 601 ] + xx [ 604 ] ; xx [ 588 ] = xx [ 589 ] + xx [
587 ] ; xx [ 599 ] = xx [ 269 ] ; xx [ 600 ] = xx [ 542 ] ; xx [ 601 ] = xx [
588 ] ; xx [ 602 ] = - ( pm_math_Vector3_dot_ra ( xx + 684 , xx + 569 ) +
pm_math_Vector3_dot_ra ( xx + 700 , xx + 599 ) ) ; xx [ 603 ] = -
pm_math_Vector3_dot_ra ( xx + 684 , xx + 599 ) ; solveSymmetricPosDef ( xx +
664 , xx + 602 , 2 , 1 , xx + 569 , xx + 599 ) ; xx [ 599 ] = xx [ 39 ] ; xx
[ 600 ] = xx [ 40 ] ; xx [ 601 ] = xx [ 53 ] ; xx [ 39 ] = ( xx [ 2 ] * xx [
156 ] + xx [ 153 ] ) * xx [ 135 ] ; xx [ 40 ] = ( xx [ 2 ] * xx [ 157 ] + xx
[ 154 ] ) * xx [ 135 ] ; xx [ 152 ] = xx [ 56 ] ; xx [ 153 ] = xx [ 39 ] ; xx
[ 154 ] = xx [ 40 ] ; xx [ 53 ] = pm_math_Vector3_dot_ra ( xx + 599 , xx +
152 ) ; xx [ 135 ] = xx [ 53 ] / xx [ 178 ] ; xx [ 152 ] = xx [ 56 ] - xx [
148 ] * xx [ 135 ] ; xx [ 153 ] = xx [ 39 ] - xx [ 180 ] * xx [ 135 ] ; xx [
154 ] = xx [ 40 ] - xx [ 182 ] * xx [ 135 ] ; pm_math_Quaternion_xform_ra (
xx + 125 , xx + 152 , xx + 155 ) ; pm_math_Quaternion_inverseXform_ra ( xx +
1243 , xx + 46 , xx + 152 ) ; xx [ 571 ] = 3.974634836311137e-3 ; xx [ 589 ]
= xx [ 571 ] * state [ 34 ] ; xx [ 599 ] = 0.9999921011077627 ; xx [ 600 ] =
xx [ 599 ] * state [ 34 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 449 ,
xx + 601 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 601 , xx + 604 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 604 , xx + 601 ) ; xx [
604 ] = ( xx [ 2 ] * ( xx [ 154 ] * xx [ 589 ] - xx [ 153 ] * xx [ 600 ] ) +
xx [ 601 ] ) * xx [ 161 ] ; xx [ 605 ] = xx [ 161 ] * ( xx [ 602 ] + xx [ 2 ]
* xx [ 152 ] * xx [ 600 ] ) ; xx [ 600 ] = ( xx [ 603 ] - xx [ 2 ] * xx [ 152
] * xx [ 589 ] ) * xx [ 161 ] ; xx [ 589 ] = xx [ 571 ] * xx [ 605 ] + xx [
599 ] * xx [ 600 ] ; xx [ 571 ] = xx [ 589 ] / xx [ 164 ] ; xx [ 601 ] = xx [
604 ] ; xx [ 602 ] = xx [ 605 ] - xx [ 162 ] * xx [ 571 ] ; xx [ 603 ] = xx [
600 ] - xx [ 168 ] * xx [ 571 ] ; pm_math_Quaternion_xform_ra ( xx + 1243 ,
xx + 601 , xx + 606 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx +
46 , xx + 601 ) ; xx [ 609 ] = xx [ 599 ] * state [ 32 ] ; xx [ 610 ] =
3.974634836312298e-3 ; xx [ 611 ] = xx [ 610 ] * state [ 32 ] ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 204 , xx + 675 ) ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 675 , xx + 678 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 678 , xx + 675 ) ; xx [
612 ] = ( xx [ 2 ] * ( xx [ 602 ] * xx [ 609 ] - xx [ 603 ] * xx [ 611 ] ) +
xx [ 675 ] ) * xx [ 161 ] ; xx [ 613 ] = xx [ 161 ] * ( xx [ 676 ] - xx [ 2 ]
* xx [ 601 ] * xx [ 609 ] ) ; xx [ 609 ] = ( xx [ 2 ] * xx [ 601 ] * xx [ 611
] + xx [ 677 ] ) * xx [ 161 ] ; xx [ 161 ] = xx [ 610 ] * xx [ 613 ] + xx [
599 ] * xx [ 609 ] ; xx [ 610 ] = xx [ 161 ] / xx [ 163 ] ; xx [ 675 ] = xx [
612 ] ; xx [ 676 ] = xx [ 613 ] - xx [ 200 ] * xx [ 610 ] ; xx [ 677 ] = xx [
609 ] - xx [ 168 ] * xx [ 610 ] ; pm_math_Quaternion_xform_ra ( xx + 384 , xx
+ 675 , xx + 678 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 46
, xx + 675 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 675 , xx
+ 681 ) ; xx [ 611 ] = xx [ 228 ] * inputDot [ 0 ] ; xx [ 640 ] = xx [ 681 ]
- xx [ 611 ] ; xx [ 641 ] = xx [ 230 ] * inputDot [ 0 ] ; xx [ 654 ] = xx [
682 ] + xx [ 641 ] ; xx [ 663 ] = xx [ 231 ] * inputDot [ 0 ] ; xx [ 668 ] =
xx [ 683 ] + xx [ 663 ] ; xx [ 690 ] = xx [ 640 ] ; xx [ 691 ] = xx [ 654 ] ;
xx [ 692 ] = xx [ 668 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 262 , xx
+ 690 , xx + 693 ) ; xx [ 669 ] = xx [ 235 ] * inputDot [ 3 ] ; xx [ 696 ] =
xx [ 693 ] - xx [ 669 ] ; xx [ 697 ] = xx [ 256 ] * inputDot [ 3 ] ; xx [ 698
] = xx [ 694 ] + xx [ 697 ] ; xx [ 708 ] = xx [ 257 ] * inputDot [ 3 ] ; xx [
724 ] = xx [ 695 ] + xx [ 708 ] ; xx [ 733 ] = xx [ 693 ] + xx [ 696 ] ; xx [
734 ] = xx [ 694 ] + xx [ 698 ] ; xx [ 735 ] = xx [ 695 ] + xx [ 724 ] ; xx [
725 ] = 2.971385144603623e-7 ; xx [ 726 ] = 1.158326948378051e-13 ; xx [ 739
] = 3.938183964273754e-8 ; xx [ 740 ] = xx [ 725 ] * inputDot [ 3 ] ; xx [
741 ] = xx [ 726 ] * inputDot [ 3 ] ; xx [ 742 ] = - ( xx [ 739 ] * inputDot
[ 3 ] ) ; pm_math_Vector3_cross_ra ( xx + 733 , xx + 740 , xx + 743 ) ;
pm_math_Vector3_cross_ra ( xx + 690 , xx + 325 , xx + 733 ) ;
pm_math_Vector3_cross_ra ( xx + 690 , xx + 733 , xx + 740 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 262 , xx + 740 , xx + 733 ) ; xx [
740 ] = xx [ 392 ] * input [ 5 ] ; xx [ 741 ] = xx [ 426 ] * input [ 5 ] ; xx
[ 742 ] = xx [ 2 ] * ( xx [ 222 ] * xx [ 740 ] - xx [ 425 ] * xx [ 741 ] ) ;
xx [ 746 ] = input [ 5 ] - ( xx [ 426 ] * xx [ 741 ] + xx [ 392 ] * xx [ 740
] ) * xx [ 2 ] ; xx [ 392 ] = ( xx [ 425 ] * xx [ 740 ] + xx [ 222 ] * xx [
741 ] ) * xx [ 2 ] ; xx [ 793 ] = ( xx [ 743 ] + xx [ 733 ] + xx [ 725 ] *
inputDdot [ 3 ] ) * xx [ 232 ] - xx [ 742 ] ; xx [ 794 ] = ( xx [ 744 ] + xx
[ 734 ] + xx [ 726 ] * inputDdot [ 3 ] ) * xx [ 232 ] - xx [ 746 ] ; xx [ 795
] = ( xx [ 745 ] + xx [ 735 ] - xx [ 739 ] * inputDdot [ 3 ] ) * xx [ 232 ] -
xx [ 392 ] ; pm_math_Quaternion_xform_ra ( xx + 262 , xx + 793 , xx + 733 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 690 , xx + 743 ) ; xx [
222 ] = xx [ 261 ] * state [ 29 ] ; xx [ 232 ] = xx [ 743 ] + xx [ 222 ] ; xx
[ 425 ] = xx [ 282 ] * state [ 29 ] ; xx [ 426 ] = xx [ 744 ] - xx [ 425 ] ;
xx [ 740 ] = xx [ 283 ] * state [ 29 ] ; xx [ 741 ] = xx [ 745 ] - xx [ 740 ]
; xx [ 793 ] = xx [ 743 ] + xx [ 232 ] ; xx [ 794 ] = xx [ 744 ] + xx [ 426 ]
; xx [ 795 ] = xx [ 745 ] + xx [ 741 ] ; xx [ 799 ] = xx [ 20 ] * state [ 29
] + xx [ 261 ] * state [ 30 ] ; xx [ 800 ] = xx [ 202 ] * state [ 29 ] - xx [
282 ] * state [ 30 ] ; xx [ 801 ] = - ( xx [ 328 ] * state [ 29 ] + xx [ 283
] * state [ 30 ] ) ; pm_math_Vector3_cross_ra ( xx + 793 , xx + 799 , xx +
808 ) ; pm_math_Vector3_cross_ra ( xx + 690 , xx + 736 , xx + 793 ) ;
pm_math_Vector3_cross_ra ( xx + 690 , xx + 793 , xx + 799 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 799 , xx + 793 ) ; xx [
20 ] = ( xx [ 808 ] + xx [ 793 ] ) * xx [ 258 ] ; xx [ 799 ] = xx [ 261 ] ;
xx [ 800 ] = - xx [ 282 ] ; xx [ 801 ] = - xx [ 283 ] ; xx [ 819 ] = xx [ 232
] ; xx [ 820 ] = xx [ 426 ] ; xx [ 821 ] = xx [ 741 ] ; xx [ 822 ] = xx [ 232
] * xx [ 639 ] ; xx [ 823 ] = xx [ 426 ] * xx [ 642 ] ; xx [ 824 ] = xx [ 741
] * xx [ 644 ] ; pm_math_Vector3_cross_ra ( xx + 819 , xx + 822 , xx + 888 )
; xx [ 819 ] = xx [ 222 ] ; xx [ 820 ] = - xx [ 425 ] ; xx [ 821 ] = - xx [
740 ] ; pm_math_Vector3_cross_ra ( xx + 743 , xx + 819 , xx + 822 ) ; xx [
202 ] = xx [ 888 ] + xx [ 639 ] * xx [ 822 ] ; xx [ 232 ] = xx [ 889 ] + xx [
642 ] * xx [ 823 ] ; xx [ 328 ] = xx [ 890 ] + xx [ 644 ] * xx [ 824 ] ; xx [
743 ] = xx [ 202 ] ; xx [ 744 ] = xx [ 232 ] ; xx [ 745 ] = xx [ 328 ] ; xx [
426 ] = ( xx [ 809 ] + xx [ 794 ] ) * xx [ 258 ] ; xx [ 639 ] = ( xx [ 810 ]
+ xx [ 795 ] ) * xx [ 258 ] ; xx [ 793 ] = xx [ 20 ] ; xx [ 794 ] = xx [ 426
] ; xx [ 795 ] = xx [ 639 ] ; xx [ 258 ] = pm_math_Vector3_dot_ra ( xx + 799
, xx + 743 ) + pm_math_Vector3_dot_ra ( xx + 985 , xx + 793 ) ; xx [ 642 ] =
pm_math_Vector3_dot_ra ( xx + 799 , xx + 793 ) ; xx [ 743 ] = - xx [ 258 ] ;
xx [ 744 ] = - xx [ 642 ] ; solveSymmetricPosDef ( xx + 312 , xx + 743 , 2 ,
1 , xx + 793 , xx + 799 ) ; xx [ 743 ] = xx [ 20 ] + xx [ 284 ] * xx [ 793 ]
+ xx [ 320 ] * xx [ 794 ] ; xx [ 744 ] = xx [ 426 ] + xx [ 321 ] * xx [ 793 ]
- xx [ 322 ] * xx [ 794 ] ; xx [ 745 ] = xx [ 639 ] - xx [ 323 ] * xx [ 793 ]
- xx [ 324 ] * xx [ 794 ] ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 743
, xx + 799 ) ; xx [ 743 ] = - xx [ 611 ] ; xx [ 744 ] = xx [ 641 ] ; xx [ 745
] = xx [ 663 ] ; pm_math_Vector3_cross_ra ( xx + 681 , xx + 743 , xx + 808 )
; xx [ 644 ] = xx [ 808 ] - xx [ 228 ] * inputDdot [ 0 ] ; xx [ 741 ] = xx [
809 ] + xx [ 230 ] * inputDdot [ 0 ] ; xx [ 747 ] = xx [ 810 ] + xx [ 231 ] *
inputDdot [ 0 ] ; xx [ 808 ] = xx [ 644 ] ; xx [ 809 ] = xx [ 741 ] ; xx [
810 ] = xx [ 747 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 286 , xx +
808 , xx + 888 ) ; xx [ 891 ] = xx [ 681 ] + xx [ 640 ] ; xx [ 892 ] = xx [
682 ] + xx [ 654 ] ; xx [ 893 ] = xx [ 683 ] + xx [ 668 ] ; xx [ 681 ] =
1.439701935798469e-6 ; xx [ 682 ] = 0.01494182793334727 ; xx [ 683 ] =
1.53614523658561e-6 ; xx [ 913 ] = xx [ 681 ] * inputDot [ 0 ] ; xx [ 914 ] =
xx [ 682 ] * inputDot [ 0 ] ; xx [ 915 ] = xx [ 683 ] * inputDot [ 0 ] ;
pm_math_Vector3_cross_ra ( xx + 891 , xx + 913 , xx + 919 ) ;
pm_math_Vector3_cross_ra ( xx + 675 , xx + 784 , xx + 891 ) ;
pm_math_Vector3_cross_ra ( xx + 675 , xx + 891 , xx + 913 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 913 , xx + 891 ) ; xx [
765 ] = xx [ 919 ] + xx [ 891 ] + xx [ 681 ] * inputDdot [ 0 ] ; xx [ 795 ] =
xx [ 920 ] + xx [ 892 ] + xx [ 682 ] * inputDdot [ 0 ] ; xx [ 802 ] = xx [
921 ] + xx [ 893 ] + xx [ 683 ] * inputDdot [ 0 ] ; xx [ 891 ] = xx [ 765 ] ;
xx [ 892 ] = xx [ 795 ] ; xx [ 893 ] = xx [ 802 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 349 , xx + 891 , xx + 913 ) ; xx [ 349 ] =
xx [ 888 ] + xx [ 913 ] ; xx [ 350 ] = xx [ 889 ] + xx [ 914 ] ; xx [ 351 ] =
xx [ 890 ] + xx [ 915 ] ; xx [ 352 ] = xx [ 733 ] + xx [ 799 ] + xx [ 349 ] ;
xx [ 353 ] = xx [ 734 ] + xx [ 800 ] + xx [ 350 ] ; xx [ 354 ] = xx [ 735 ] +
xx [ 801 ] + xx [ 351 ] ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 352 ,
xx + 355 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 675 , xx +
352 ) ; xx [ 803 ] = xx [ 335 ] * state [ 25 ] ; xx [ 836 ] = xx [ 352 ] + xx
[ 803 ] ; xx [ 839 ] = xx [ 362 ] * state [ 25 ] ; xx [ 841 ] = xx [ 353 ] +
xx [ 839 ] ; xx [ 843 ] = xx [ 363 ] * state [ 25 ] ; xx [ 844 ] = xx [ 354 ]
+ xx [ 843 ] ; xx [ 888 ] = xx [ 352 ] + xx [ 836 ] ; xx [ 889 ] = xx [ 353 ]
+ xx [ 841 ] ; xx [ 890 ] = xx [ 354 ] + xx [ 844 ] ; xx [ 913 ] = xx [ 335 ]
* state [ 26 ] - xx [ 757 ] * state [ 25 ] ; xx [ 914 ] = xx [ 758 ] * state
[ 25 ] + xx [ 362 ] * state [ 26 ] ; xx [ 915 ] = xx [ 363 ] * state [ 26 ] -
xx [ 759 ] * state [ 25 ] ; pm_math_Vector3_cross_ra ( xx + 888 , xx + 913 ,
xx + 757 ) ; pm_math_Vector3_cross_ra ( xx + 675 , xx + 813 , xx + 888 ) ;
pm_math_Vector3_cross_ra ( xx + 675 , xx + 888 , xx + 913 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 913 , xx + 888 ) ; xx [
884 ] = ( xx [ 757 ] + xx [ 888 ] ) * xx [ 334 ] ; xx [ 913 ] = xx [ 335 ] ;
xx [ 914 ] = xx [ 362 ] ; xx [ 915 ] = xx [ 363 ] ; xx [ 919 ] = xx [ 836 ] ;
xx [ 920 ] = xx [ 841 ] ; xx [ 921 ] = xx [ 844 ] ; xx [ 985 ] = xx [ 836 ] *
xx [ 707 ] ; xx [ 986 ] = xx [ 841 ] * xx [ 633 ] ; xx [ 987 ] = xx [ 844 ] *
xx [ 643 ] ; pm_math_Vector3_cross_ra ( xx + 919 , xx + 985 , xx + 1000 ) ;
xx [ 919 ] = xx [ 803 ] ; xx [ 920 ] = xx [ 839 ] ; xx [ 921 ] = xx [ 843 ] ;
pm_math_Vector3_cross_ra ( xx + 352 , xx + 919 , xx + 985 ) ; xx [ 352 ] = xx
[ 1000 ] + xx [ 707 ] * xx [ 985 ] ; xx [ 353 ] = xx [ 1001 ] + xx [ 633 ] *
xx [ 986 ] ; xx [ 354 ] = xx [ 1002 ] + xx [ 643 ] * xx [ 987 ] ; xx [ 633 ]
= ( xx [ 758 ] + xx [ 889 ] ) * xx [ 334 ] ; xx [ 643 ] = ( xx [ 759 ] + xx [
890 ] ) * xx [ 334 ] ; xx [ 757 ] = xx [ 884 ] ; xx [ 758 ] = xx [ 633 ] ; xx
[ 759 ] = xx [ 643 ] ; xx [ 334 ] = pm_math_Vector3_dot_ra ( xx + 913 , xx +
352 ) + pm_math_Vector3_dot_ra ( xx + 778 , xx + 757 ) ; xx [ 707 ] =
pm_math_Vector3_dot_ra ( xx + 913 , xx + 757 ) ; xx [ 757 ] = - xx [ 334 ] ;
xx [ 758 ] = - xx [ 707 ] ; solveSymmetricPosDef ( xx + 393 , xx + 757 , 2 ,
1 , xx + 778 , xx + 888 ) ; xx [ 757 ] = xx [ 884 ] - xx [ 398 ] * xx [ 778 ]
+ xx [ 364 ] * xx [ 779 ] ; xx [ 758 ] = xx [ 633 ] + xx [ 399 ] * xx [ 778 ]
+ xx [ 400 ] * xx [ 779 ] ; xx [ 759 ] = xx [ 643 ] - xx [ 401 ] * xx [ 778 ]
+ xx [ 402 ] * xx [ 779 ] ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 757
, xx + 888 ) ; xx [ 1043 ] = xx [ 412 ] ; xx [ 1044 ] = xx [ 414 ] ; xx [
1045 ] = xx [ 435 ] ; xx [ 1046 ] = xx [ 429 ] ; xx [ 1047 ] = xx [ 430 ] ;
xx [ 1048 ] = xx [ 441 ] ; xx [ 1049 ] = xx [ 436 ] ; xx [ 1050 ] = xx [ 437
] ; xx [ 1051 ] = xx [ 443 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 845
, xx + 435 ) ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 435 , xx + 757 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 757 , xx + 435 ) ; xx [
412 ] = xx [ 427 ] * state [ 22 ] ; xx [ 414 ] = xx [ 435 ] - xx [ 2 ] * xx [
677 ] * xx [ 412 ] ; xx [ 429 ] = xx [ 413 ] * state [ 22 ] ; xx [ 430 ] = xx
[ 2 ] * xx [ 677 ] * xx [ 429 ] + xx [ 436 ] ; xx [ 435 ] = xx [ 2 ] * ( xx [
675 ] * xx [ 412 ] - xx [ 676 ] * xx [ 429 ] ) + xx [ 437 ] ; xx [ 757 ] = xx
[ 414 ] ; xx [ 758 ] = xx [ 430 ] ; xx [ 759 ] = xx [ 435 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 1043 , xx + 757 , xx + 913 ) ; xx [ 412 ] =
xx [ 355 ] + xx [ 888 ] + xx [ 913 ] ; xx [ 429 ] = xx [ 356 ] + xx [ 889 ] +
xx [ 914 ] ; xx [ 436 ] = ( xx [ 412 ] * xx [ 413 ] + xx [ 429 ] * xx [ 427 ]
) / xx [ 432 ] ; xx [ 1000 ] = xx [ 412 ] - xx [ 428 ] * xx [ 436 ] ; xx [
1001 ] = xx [ 429 ] - xx [ 431 ] * xx [ 436 ] ; xx [ 1002 ] = xx [ 357 ] + xx
[ 890 ] + xx [ 915 ] - xx [ 438 ] * xx [ 436 ] ; pm_math_Quaternion_xform_ra
( xx + 388 , xx + 1000 , xx + 1007 ) ; xx [ 1000 ] = xx [ 269 ] + xx [ 706 ]
* xx [ 569 ] + xx [ 660 ] * xx [ 570 ] ; xx [ 1001 ] = xx [ 542 ] + xx [ 523
] * xx [ 569 ] + xx [ 661 ] * xx [ 570 ] ; xx [ 1002 ] = xx [ 588 ] + xx [
524 ] * xx [ 569 ] + xx [ 570 ] * xx [ 662 ] ; pm_math_Quaternion_xform_ra (
xx + 464 , xx + 1000 , xx + 1022 ) ; xx [ 1000 ] = state [ 7 ] ; xx [ 1001 ]
= state [ 8 ] ; xx [ 1002 ] = state [ 9 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 16 , xx + 1000 , xx + 1025 ) ; xx [
1000 ] = 1.533017838243546e-3 ; xx [ 1001 ] = 1.220598588933962e-3 ; xx [
1002 ] = 8.957592155301676e-3 ; pm_math_Vector3_cross_ra ( xx + 46 , xx +
1000 , xx + 1033 ) ; xx [ 1000 ] = xx [ 1025 ] + xx [ 1033 ] ; xx [ 1001 ] =
xx [ 1026 ] + xx [ 1034 ] ; xx [ 1002 ] = xx [ 1027 ] + xx [ 1035 ] ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 1000 , xx + 1033 ) ; xx [ 1000 ] =
- xx [ 1025 ] ; xx [ 1001 ] = - xx [ 1026 ] ; xx [ 1002 ] = - xx [ 1027 ] ;
pm_math_Vector3_cross_ra ( xx + 46 , xx + 1000 , xx + 1025 ) ; xx [ 269 ] =
xx [ 1033 ] + xx [ 1025 ] ; xx [ 412 ] = xx [ 1034 ] + xx [ 1026 ] ; xx [ 429
] = xx [ 1035 ] + xx [ 1027 ] ; xx [ 1000 ] = xx [ 269 ] ; xx [ 1001 ] = xx [
412 ] ; xx [ 1002 ] = xx [ 429 ] ; pm_math_Matrix3x3_xform_ra ( xx + 748 , xx
+ 1000 , xx + 1025 ) ; xx [ 437 ] = xx [ 85 ] + xx [ 91 ] + xx [ 94 ] + xx [
133 ] ; xx [ 441 ] = xx [ 86 ] + xx [ 92 ] + xx [ 95 ] + xx [ 134 ] ; xx [
748 ] = xx [ 0 ] + xx [ 155 ] + xx [ 606 ] + xx [ 678 ] + xx [ 1007 ] + xx [
1022 ] + xx [ 1025 ] ; xx [ 749 ] = xx [ 437 ] + xx [ 156 ] + xx [ 607 ] + xx
[ 679 ] + xx [ 1008 ] + xx [ 1023 ] + xx [ 1026 ] ; xx [ 750 ] = xx [ 441 ] +
xx [ 157 ] + xx [ 608 ] + xx [ 680 ] + xx [ 1009 ] + xx [ 1024 ] + xx [ 1027
] ; xx [ 751 ] = 4.106972712955291e-4 * xx [ 46 ] ; xx [ 752 ] =
7.605948409860241e-4 * xx [ 47 ] ; xx [ 753 ] = 4.251591882645771e-4 * xx [
48 ] ; pm_math_Vector3_cross_ra ( xx + 46 , xx + 751 , xx + 754 ) ; xx [ 46 ]
= xx [ 87 ] - xx [ 44 ] * xx [ 6 ] ; xx [ 47 ] = xx [ 88 ] + xx [ 998 ] * xx
[ 14 ] * state [ 44 ] ; xx [ 48 ] = xx [ 89 ] - xx [ 999 ] * xx [ 13 ] *
state [ 44 ] ; pm_math_Quaternion_xform_ra ( xx + 75 , xx + 46 , xx + 87 ) ;
pm_math_Vector3_cross_ra ( xx + 445 , xx + 84 , xx + 46 ) ; xx [ 84 ] = xx [
118 ] + xx [ 44 ] * xx [ 12 ] ; xx [ 85 ] = xx [ 119 ] - xx [ 998 ] * xx [ 83
] * state [ 42 ] ; xx [ 86 ] = xx [ 120 ] + xx [ 999 ] * xx [ 82 ] * state [
42 ] ; pm_math_Quaternion_xform_ra ( xx + 107 , xx + 84 , xx + 118 ) ;
pm_math_Vector3_cross_ra ( xx + 71 , xx + 90 , xx + 82 ) ; xx [ 90 ] = xx [
115 ] + xx [ 44 ] * xx [ 15 ] ; xx [ 91 ] = xx [ 116 ] - xx [ 998 ] * xx [ 68
] * state [ 40 ] ; xx [ 92 ] = xx [ 117 ] + xx [ 999 ] * xx [ 67 ] * state [
40 ] ; pm_math_Quaternion_xform_ra ( xx + 121 , xx + 90 , xx + 115 ) ;
pm_math_Vector3_cross_ra ( xx + 103 , xx + 93 , xx + 90 ) ; xx [ 93 ] = xx [
145 ] + xx [ 44 ] * xx [ 31 ] ; xx [ 94 ] = xx [ 146 ] - xx [ 998 ] * xx [
113 ] * state [ 38 ] ; xx [ 95 ] = xx [ 147 ] + xx [ 999 ] * xx [ 112 ] *
state [ 38 ] ; pm_math_Quaternion_xform_ra ( xx + 98 , xx + 93 , xx + 145 ) ;
pm_math_Vector3_cross_ra ( xx + 136 , xx + 132 , xx + 93 ) ; xx [ 13 ] = xx [
754 ] + xx [ 87 ] + xx [ 46 ] + xx [ 118 ] + xx [ 82 ] + xx [ 115 ] + xx [ 90
] + xx [ 145 ] + xx [ 93 ] ; xx [ 132 ] = 2.989111964725376e-5 * xx [ 149 ] ;
xx [ 133 ] = 9.37560211669326e-5 * xx [ 150 ] ; xx [ 134 ] =
9.694819254541645e-5 * xx [ 151 ] ; pm_math_Vector3_cross_ra ( xx + 149 , xx
+ 132 , xx + 751 ) ; pm_math_Quaternion_xform_ra ( xx + 125 , xx + 751 , xx +
132 ) ; pm_math_Vector3_cross_ra ( xx + 140 , xx + 155 , xx + 149 ) ; xx [ 14
] = 1.726034543427056e-7 ; xx [ 44 ] = 2.688648778104647e-7 ; xx [ 67 ] =
1.78804166109968e-7 ; xx [ 155 ] = xx [ 14 ] * xx [ 152 ] ; xx [ 156 ] = xx [
44 ] * xx [ 153 ] ; xx [ 157 ] = xx [ 67 ] * xx [ 154 ] ;
pm_math_Vector3_cross_ra ( xx + 152 , xx + 155 , xx + 751 ) ;
pm_math_Quaternion_xform_ra ( xx + 1243 , xx + 751 , xx + 152 ) ;
pm_math_Vector3_cross_ra ( xx + 449 , xx + 606 , xx + 155 ) ; xx [ 606 ] = xx
[ 14 ] * xx [ 601 ] ; xx [ 607 ] = xx [ 44 ] * xx [ 602 ] ; xx [ 608 ] = xx [
67 ] * xx [ 603 ] ; pm_math_Vector3_cross_ra ( xx + 601 , xx + 606 , xx + 751
) ; pm_math_Quaternion_xform_ra ( xx + 384 , xx + 751 , xx + 601 ) ;
pm_math_Vector3_cross_ra ( xx + 204 , xx + 678 , xx + 606 ) ; xx [ 678 ] = xx
[ 42 ] * xx [ 675 ] ; xx [ 679 ] = xx [ 60 ] * xx [ 676 ] ; xx [ 680 ] = xx [
65 ] * xx [ 677 ] ; pm_math_Vector3_cross_ra ( xx + 675 , xx + 678 , xx + 751
) ; xx [ 675 ] = xx [ 640 ] * xx [ 34 ] ; xx [ 676 ] = xx [ 654 ] * xx [ 37 ]
; xx [ 677 ] = xx [ 668 ] * xx [ 38 ] ; pm_math_Vector3_cross_ra ( xx + 690 ,
xx + 675 , xx + 678 ) ; xx [ 675 ] = xx [ 696 ] ; xx [ 676 ] = xx [ 698 ] ;
xx [ 677 ] = xx [ 724 ] ; xx [ 690 ] = xx [ 696 ] * xx [ 709 ] ; xx [ 691 ] =
xx [ 698 ] * xx [ 710 ] ; xx [ 692 ] = xx [ 724 ] * xx [ 711 ] ;
pm_math_Vector3_cross_ra ( xx + 675 , xx + 690 , xx + 997 ) ; xx [ 675 ] = xx
[ 742 ] ; xx [ 676 ] = xx [ 746 ] ; xx [ 677 ] = xx [ 392 ] ;
pm_math_Vector3_cross_ra ( xx + 421 , xx + 675 , xx + 690 ) ; xx [ 421 ] = -
xx [ 669 ] ; xx [ 422 ] = xx [ 697 ] ; xx [ 423 ] = xx [ 708 ] ;
pm_math_Vector3_cross_ra ( xx + 693 , xx + 421 , xx + 675 ) ; xx [ 14 ] = xx
[ 675 ] - xx [ 235 ] * inputDdot [ 3 ] ; xx [ 34 ] = xx [ 676 ] + xx [ 256 ]
* inputDdot [ 3 ] ; xx [ 37 ] = xx [ 677 ] + xx [ 257 ] * inputDdot [ 3 ] ;
xx [ 675 ] = xx [ 997 ] - xx [ 690 ] + xx [ 14 ] * xx [ 709 ] ; xx [ 676 ] =
xx [ 998 ] - xx [ 691 ] + xx [ 34 ] * xx [ 710 ] ; xx [ 677 ] = xx [ 999 ] -
xx [ 692 ] + xx [ 37 ] * xx [ 711 ] ; pm_math_Quaternion_xform_ra ( xx + 262
, xx + 675 , xx + 690 ) ; pm_math_Vector3_cross_ra ( xx + 325 , xx + 733 , xx
+ 675 ) ; xx [ 38 ] = xx [ 678 ] + xx [ 690 ] + xx [ 675 ] ; xx [ 325 ] = xx
[ 202 ] + xx [ 316 ] * xx [ 793 ] ; xx [ 326 ] = xx [ 232 ] - xx [ 318 ] * xx
[ 793 ] ; xx [ 327 ] = xx [ 328 ] - xx [ 319 ] * xx [ 793 ] ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 325 , xx + 693 ) ;
pm_math_Vector3_cross_ra ( xx + 736 , xx + 799 , xx + 325 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 766 , xx + 808 , xx + 709 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 286 , xx + 891 , xx + 766 ) ; xx [ 42 ] =
xx [ 709 ] + xx [ 766 ] ; xx [ 44 ] = xx [ 679 ] + xx [ 691 ] + xx [ 676 ] ;
xx [ 60 ] = xx [ 710 ] + xx [ 767 ] ; xx [ 65 ] = xx [ 680 ] + xx [ 692 ] +
xx [ 677 ] ; xx [ 67 ] = xx [ 711 ] + xx [ 768 ] ; xx [ 286 ] = xx [ 38 ] +
xx [ 693 ] + xx [ 325 ] + xx [ 42 ] ; xx [ 287 ] = xx [ 44 ] + xx [ 694 ] +
xx [ 326 ] + xx [ 60 ] ; xx [ 288 ] = xx [ 65 ] + xx [ 695 ] + xx [ 327 ] +
xx [ 67 ] ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 286 , xx + 289 ) ;
pm_math_Vector3_cross_ra ( xx + 784 , xx + 355 , xx + 286 ) ; xx [ 292 ] = xx
[ 352 ] + xx [ 376 ] * xx [ 778 ] ; xx [ 293 ] = xx [ 353 ] + xx [ 379 ] * xx
[ 778 ] ; xx [ 294 ] = xx [ 354 ] + xx [ 397 ] * xx [ 778 ] ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 292 , xx + 325 ) ;
pm_math_Vector3_cross_ra ( xx + 813 , xx + 888 , xx + 292 ) ; xx [ 766 ] = xx
[ 403 ] ; xx [ 767 ] = xx [ 404 ] ; xx [ 768 ] = xx [ 406 ] ; xx [ 769 ] = xx
[ 407 ] ; xx [ 770 ] = xx [ 408 ] ; xx [ 771 ] = xx [ 410 ] ; xx [ 772 ] = xx
[ 411 ] ; xx [ 773 ] = xx [ 792 ] ; xx [ 774 ] = xx [ 826 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 766 , xx + 757 , xx + 355 ) ; xx [ 406 ] =
xx [ 751 ] + xx [ 289 ] + xx [ 286 ] + xx [ 325 ] + xx [ 292 ] + xx [ 355 ] -
xx [ 405 ] * xx [ 436 ] ; xx [ 407 ] = xx [ 752 ] + xx [ 290 ] + xx [ 287 ] +
xx [ 326 ] + xx [ 293 ] + xx [ 356 ] - xx [ 409 ] * xx [ 436 ] ; xx [ 408 ] =
xx [ 753 ] + xx [ 291 ] + xx [ 288 ] + xx [ 327 ] + xx [ 294 ] + xx [ 357 ] -
xx [ 825 ] * xx [ 436 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 406 ,
xx + 286 ) ; pm_math_Vector3_cross_ra ( xx + 845 , xx + 1007 , xx + 289 ) ;
xx [ 292 ] = xx [ 538 ] + xx [ 525 ] * xx [ 569 ] + xx [ 657 ] * xx [ 570 ] ;
xx [ 293 ] = xx [ 544 ] + xx [ 526 ] * xx [ 569 ] + xx [ 658 ] * xx [ 570 ] ;
xx [ 294 ] = xx [ 541 ] + xx [ 519 ] * xx [ 569 ] + xx [ 570 ] * xx [ 659 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 292 , xx + 325 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 1022 , xx + 292 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 1000 , xx + 406 ) ; xx [ 68 ] =
xx [ 755 ] + xx [ 88 ] + xx [ 47 ] + xx [ 119 ] + xx [ 83 ] + xx [ 116 ] + xx
[ 91 ] + xx [ 146 ] + xx [ 94 ] ; xx [ 46 ] = xx [ 756 ] + xx [ 89 ] + xx [
48 ] + xx [ 120 ] + xx [ 84 ] + xx [ 117 ] + xx [ 92 ] + xx [ 147 ] + xx [ 95
] ; xx [ 82 ] = xx [ 13 ] + xx [ 132 ] + xx [ 149 ] + xx [ 152 ] + xx [ 155 ]
+ xx [ 601 ] + xx [ 606 ] + xx [ 286 ] + xx [ 289 ] + xx [ 325 ] + xx [ 292 ]
+ xx [ 406 ] ; xx [ 83 ] = xx [ 68 ] + xx [ 133 ] + xx [ 150 ] + xx [ 153 ] +
xx [ 156 ] + xx [ 602 ] + xx [ 607 ] + xx [ 287 ] + xx [ 290 ] + xx [ 326 ] +
xx [ 293 ] + xx [ 407 ] ; xx [ 84 ] = xx [ 46 ] + xx [ 134 ] + xx [ 151 ] +
xx [ 154 ] + xx [ 157 ] + xx [ 603 ] + xx [ 608 ] + xx [ 288 ] + xx [ 291 ] +
xx [ 327 ] + xx [ 294 ] + xx [ 408 ] ; xx [ 85 ] = - pm_math_Vector3_dot_ra (
xx + 26 , xx + 748 ) ; xx [ 86 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx +
748 ) ; xx [ 87 ] = - pm_math_Vector3_dot_ra ( xx + 703 , xx + 748 ) ; xx [
88 ] = - ( pm_math_Vector3_dot_ra ( xx + 935 , xx + 82 ) +
pm_math_Vector3_dot_ra ( xx + 944 , xx + 748 ) ) ; xx [ 89 ] = - (
pm_math_Vector3_dot_ra ( xx + 954 , xx + 82 ) + pm_math_Vector3_dot_ra ( xx +
963 , xx + 748 ) ) ; xx [ 90 ] = - ( pm_math_Vector3_dot_ra ( xx + 973 , xx +
82 ) + pm_math_Vector3_dot_ra ( xx + 982 , xx + 748 ) ) ;
solveSymmetricPosDef ( xx + 848 , xx + 85 , 6 , 1 , xx + 115 , xx + 286 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 26 , xx + 82 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 672 , xx + 85 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 924 , xx + 703 , xx + 88 ) ; xx [ 1150 ] =
xx [ 82 ] ; xx [ 1151 ] = xx [ 85 ] ; xx [ 1152 ] = xx [ 88 ] ; xx [ 1153 ] =
xx [ 43 ] ; xx [ 1154 ] = xx [ 49 ] ; xx [ 1155 ] = xx [ 54 ] ; xx [ 1156 ] =
xx [ 83 ] ; xx [ 1157 ] = xx [ 86 ] ; xx [ 1158 ] = xx [ 89 ] ; xx [ 1159 ] =
xx [ 51 ] ; xx [ 1160 ] = xx [ 50 ] ; xx [ 1161 ] = xx [ 61 ] ; xx [ 1162 ] =
xx [ 84 ] ; xx [ 1163 ] = xx [ 87 ] ; xx [ 1164 ] = xx [ 90 ] ; xx [ 1165 ] =
xx [ 45 ] ; xx [ 1166 ] = xx [ 52 ] ; xx [ 1167 ] = xx [ 69 ] ; xx [ 1168 ] =
xx [ 645 ] ; xx [ 1169 ] = xx [ 687 ] ; xx [ 1170 ] = xx [ 721 ] ; xx [ 1171
] = xx [ 950 ] ; xx [ 1172 ] = xx [ 969 ] ; xx [ 1173 ] = xx [ 988 ] ; xx [
1174 ] = xx [ 646 ] ; xx [ 1175 ] = xx [ 688 ] ; xx [ 1176 ] = xx [ 722 ] ;
xx [ 1177 ] = xx [ 951 ] ; xx [ 1178 ] = xx [ 970 ] ; xx [ 1179 ] = xx [ 989
] ; xx [ 1180 ] = xx [ 647 ] ; xx [ 1181 ] = xx [ 689 ] ; xx [ 1182 ] = xx [
723 ] ; xx [ 1183 ] = xx [ 938 ] ; xx [ 1184 ] = xx [ 957 ] ; xx [ 1185 ] =
xx [ 976 ] ; solveSymmetricPosDef ( xx + 848 , xx + 1150 , 6 , 6 , xx + 1186
, xx + 47 ) ; xx [ 47 ] = xx [ 1209 ] ; xx [ 48 ] = xx [ 1215 ] ; xx [ 49 ] =
xx [ 1221 ] ; xx [ 43 ] = 9.806649999999999 ; xx [ 45 ] = xx [ 43 ] * xx [ 17
] ; xx [ 50 ] = xx [ 43 ] * xx [ 18 ] ; xx [ 51 ] = xx [ 2 ] * ( xx [ 19 ] *
xx [ 45 ] - xx [ 16 ] * xx [ 50 ] ) ; xx [ 52 ] = ( xx [ 16 ] * xx [ 45 ] +
xx [ 19 ] * xx [ 50 ] ) * xx [ 2 ] ; xx [ 16 ] = ( xx [ 17 ] * xx [ 45 ] + xx
[ 18 ] * xx [ 50 ] ) * xx [ 2 ] ; xx [ 17 ] = xx [ 51 ] ; xx [ 18 ] = xx [ 52
] ; xx [ 19 ] = xx [ 43 ] - xx [ 16 ] ; xx [ 45 ] = pm_math_Vector3_dot_ra (
xx + 47 , xx + 17 ) ; xx [ 47 ] = xx [ 120 ] - xx [ 45 ] ; xx [ 48 ] = xx [
1207 ] ; xx [ 49 ] = xx [ 1213 ] ; xx [ 50 ] = xx [ 1219 ] ; xx [ 54 ] =
pm_math_Vector3_dot_ra ( xx + 48 , xx + 17 ) ; xx [ 48 ] = xx [ 118 ] - xx [
54 ] ; xx [ 82 ] = xx [ 1208 ] ; xx [ 83 ] = xx [ 1214 ] ; xx [ 84 ] = xx [
1220 ] ; xx [ 49 ] = pm_math_Vector3_dot_ra ( xx + 82 , xx + 17 ) ; xx [ 50 ]
= xx [ 119 ] - xx [ 49 ] ; xx [ 82 ] = xx [ 959 ] * xx [ 47 ] - ( xx [ 671 ]
* xx [ 48 ] + xx [ 940 ] * xx [ 50 ] ) ; xx [ 83 ] = xx [ 933 ] * xx [ 48 ] -
xx [ 952 ] * xx [ 50 ] + xx [ 971 ] * xx [ 47 ] ; xx [ 84 ] = xx [ 953 ] * xx
[ 50 ] - xx [ 934 ] * xx [ 48 ] + xx [ 972 ] * xx [ 47 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 82 , xx + 85 ) ; xx [ 88
] = xx [ 1204 ] ; xx [ 89 ] = xx [ 1210 ] ; xx [ 90 ] = xx [ 1216 ] ; xx [ 61
] = pm_math_Vector3_dot_ra ( xx + 88 , xx + 17 ) ; xx [ 69 ] = xx [ 115 ] -
xx [ 61 ] ; xx [ 88 ] = xx [ 1205 ] ; xx [ 89 ] = xx [ 1211 ] ; xx [ 90 ] =
xx [ 1217 ] ; xx [ 91 ] = pm_math_Vector3_dot_ra ( xx + 88 , xx + 17 ) ; xx [
88 ] = xx [ 116 ] - xx [ 91 ] ; xx [ 92 ] = xx [ 1206 ] ; xx [ 93 ] = xx [
1212 ] ; xx [ 94 ] = xx [ 1218 ] ; xx [ 89 ] = pm_math_Vector3_dot_ra ( xx +
92 , xx + 17 ) ; xx [ 17 ] = xx [ 117 ] - xx [ 89 ] ; xx [ 18 ] = xx [ 51 ] +
xx [ 3 ] * xx [ 69 ] + xx [ 196 ] * xx [ 88 ] + xx [ 522 ] * xx [ 17 ] + xx [
941 ] * xx [ 48 ] - xx [ 960 ] * xx [ 50 ] + xx [ 979 ] * xx [ 47 ] + xx [
269 ] ; pm_math_Vector3_cross_ra ( xx + 82 , xx + 651 , xx + 92 ) ; xx [ 19 ]
= xx [ 52 ] + xx [ 22 ] * xx [ 69 ] + xx [ 21 ] * xx [ 88 ] + xx [ 23 ] * xx
[ 17 ] + xx [ 942 ] * xx [ 48 ] + xx [ 961 ] * xx [ 50 ] - xx [ 980 ] * xx [
47 ] + xx [ 412 ] ; xx [ 90 ] = xx [ 25 ] * xx [ 69 ] - xx [ 16 ] + xx [ 434
] * xx [ 88 ] + xx [ 1 ] * xx [ 17 ] - xx [ 943 ] * xx [ 48 ] - xx [ 962 ] *
xx [ 50 ] - xx [ 981 ] * xx [ 47 ] + xx [ 43 ] + xx [ 429 ] ; xx [ 115 ] = xx
[ 18 ] + xx [ 92 ] ; xx [ 116 ] = xx [ 19 ] + xx [ 93 ] ; xx [ 117 ] = xx [
90 ] + xx [ 94 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 115 ,
xx + 92 ) ; xx [ 17 ] = xx [ 570 ] - ( pm_math_Vector3_dot_ra ( xx + 367 , xx
+ 85 ) + pm_math_Vector3_dot_ra ( xx + 370 , xx + 92 ) ) ; xx [ 47 ] = xx [
461 ] * xx [ 41 ] - xx [ 102 ] * xx [ 453 ] ; xx [ 48 ] = xx [ 62 ] * xx [
453 ] - xx [ 41 ] * xx [ 456 ] ; xx [ 50 ] = xx [ 102 ] * xx [ 456 ] - xx [
461 ] * xx [ 62 ] ; xx [ 69 ] = xx [ 41 ] * xx [ 468 ] - xx [ 462 ] * xx [
102 ] ; xx [ 88 ] = xx [ 462 ] * xx [ 62 ] - xx [ 41 ] * xx [ 454 ] ; xx [ 95
] = xx [ 102 ] * xx [ 454 ] - xx [ 62 ] * xx [ 468 ] ; xx [ 112 ] = xx [ 41 ]
* xx [ 455 ] - xx [ 102 ] * xx [ 469 ] ; xx [ 113 ] = xx [ 62 ] * xx [ 469 ]
- xx [ 457 ] * xx [ 41 ] ; xx [ 115 ] = xx [ 457 ] * xx [ 102 ] - xx [ 62 ] *
xx [ 455 ] ; xx [ 286 ] = xx [ 102 ] * xx [ 47 ] - xx [ 62 ] * xx [ 48 ] ; xx
[ 287 ] = xx [ 41 ] * xx [ 48 ] - xx [ 102 ] * xx [ 50 ] ; xx [ 288 ] = xx [
62 ] * xx [ 50 ] - xx [ 41 ] * xx [ 47 ] ; xx [ 289 ] = xx [ 102 ] * xx [ 69
] - xx [ 62 ] * xx [ 88 ] ; xx [ 290 ] = xx [ 41 ] * xx [ 88 ] - xx [ 102 ] *
xx [ 95 ] ; xx [ 291 ] = xx [ 62 ] * xx [ 95 ] - xx [ 41 ] * xx [ 69 ] ; xx [
292 ] = xx [ 102 ] * xx [ 112 ] - xx [ 62 ] * xx [ 113 ] ; xx [ 293 ] = xx [
41 ] * xx [ 113 ] - xx [ 102 ] * xx [ 115 ] ; xx [ 294 ] = xx [ 62 ] * xx [
115 ] - xx [ 41 ] * xx [ 112 ] ; xx [ 115 ] = 3.734971681878978e-4 ; xx [ 116
] = 0.2610715860287156 ; xx [ 117 ] = xx [ 459 ] ; pm_math_Matrix3x3_xform_ra
( xx + 286 , xx + 115 , xx + 118 ) ; pm_math_Matrix3x3_xform_ra ( xx + 286 ,
xx + 169 , xx + 145 ) ; xx [ 149 ] = - 9.509201674619905e-5 ; xx [ 150 ] = -
6.255740505362065e-3 ; xx [ 151 ] = 1.691908047920296e-3 ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 149 , xx + 155 ) ; xx [ 41 ] =
state [ 15 ] * state [ 15 ] ; xx [ 47 ] = xx [ 155 ] * xx [ 41 ] ; xx [ 48 ]
= xx [ 156 ] * xx [ 41 ] ; xx [ 50 ] = xx [ 157 ] * xx [ 41 ] ; xx [ 149 ] =
xx [ 145 ] + xx [ 47 ] ; xx [ 150 ] = xx [ 146 ] + xx [ 48 ] ; xx [ 151 ] =
xx [ 147 ] + xx [ 50 ] ; pm_math_Vector3_cross_ra ( xx + 593 , xx + 169 , xx
+ 145 ) ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 145 , xx + 155 ) ; xx
[ 145 ] = - ( xx [ 166 ] * state [ 32 ] + xx [ 155 ] + xx [ 175 ] * state [
15 ] - xx [ 208 ] * state [ 16 ] ) ; xx [ 146 ] = - ( xx [ 183 ] * state [ 32
] + xx [ 156 ] + xx [ 176 ] * state [ 15 ] - xx [ 210 ] * state [ 16 ] ) ; xx
[ 147 ] = xx [ 199 ] * state [ 32 ] - ( xx [ 157 ] + xx [ 177 ] * state [ 15
] + xx [ 521 ] * state [ 16 ] ) ; pm_math_Vector3_cross_ra ( xx + 593 , xx +
115 , xx + 155 ) ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 155 , xx +
115 ) ; xx [ 41 ] = xx [ 569 ] - ( pm_math_Vector3_dot_ra ( xx + 172 , xx +
85 ) + pm_math_Vector3_dot_ra ( xx + 271 , xx + 92 ) ) ;
pm_math_Vector3_cross_ra ( xx + 82 , xx + 204 , xx + 155 ) ; xx [ 169 ] = xx
[ 18 ] + xx [ 155 ] ; xx [ 170 ] = xx [ 19 ] + xx [ 156 ] ; xx [ 171 ] = xx [
90 ] + xx [ 157 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 169
, xx + 155 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 82 , xx +
169 ) ; pm_math_Vector3_cross_ra ( xx + 82 , xx + 845 , xx + 175 ) ; xx [ 325
] = xx [ 18 ] + xx [ 175 ] ; xx [ 326 ] = xx [ 19 ] + xx [ 176 ] ; xx [ 327 ]
= xx [ 90 ] + xx [ 177 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx
+ 325 , xx + 175 ) ; xx [ 62 ] = xx [ 436 ] + pm_math_Vector3_dot_ra ( xx +
1139 , xx + 169 ) + pm_math_Vector3_dot_ra ( xx + 1147 , xx + 175 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 286 , xx + 495 , xx + 687 ) ; xx [ 69 ] =
xx [ 486 ] * xx [ 111 ] + xx [ 158 ] * xx [ 463 ] ; xx [ 88 ] = xx [ 130 ] *
xx [ 463 ] + xx [ 111 ] * xx [ 481 ] ; xx [ 95 ] = xx [ 158 ] * xx [ 481 ] -
xx [ 486 ] * xx [ 130 ] ; xx [ 102 ] = xx [ 111 ] * xx [ 493 ] + xx [ 487 ] *
xx [ 158 ] ; xx [ 112 ] = xx [ 487 ] * xx [ 130 ] + xx [ 111 ] * xx [ 479 ] ;
xx [ 113 ] = xx [ 158 ] * xx [ 479 ] - xx [ 130 ] * xx [ 493 ] ; xx [ 155 ] =
xx [ 111 ] * xx [ 480 ] + xx [ 158 ] * xx [ 494 ] ; xx [ 166 ] = xx [ 130 ] *
xx [ 494 ] + xx [ 482 ] * xx [ 111 ] ; xx [ 183 ] = xx [ 482 ] * xx [ 158 ] -
xx [ 130 ] * xx [ 480 ] ; xx [ 493 ] = - ( xx [ 158 ] * xx [ 69 ] + xx [ 130
] * xx [ 88 ] ) ; xx [ 494 ] = - ( xx [ 111 ] * xx [ 88 ] + xx [ 158 ] * xx [
95 ] ) ; xx [ 495 ] = xx [ 130 ] * xx [ 95 ] - xx [ 111 ] * xx [ 69 ] ; xx [
496 ] = - ( xx [ 158 ] * xx [ 102 ] + xx [ 130 ] * xx [ 112 ] ) ; xx [ 497 ]
= - ( xx [ 111 ] * xx [ 112 ] + xx [ 158 ] * xx [ 113 ] ) ; xx [ 498 ] = xx [
130 ] * xx [ 113 ] - xx [ 111 ] * xx [ 102 ] ; xx [ 499 ] = - ( xx [ 158 ] *
xx [ 155 ] + xx [ 130 ] * xx [ 166 ] ) ; xx [ 500 ] = - ( xx [ 111 ] * xx [
166 ] + xx [ 158 ] * xx [ 183 ] ) ; xx [ 501 ] = xx [ 130 ] * xx [ 183 ] - xx
[ 111 ] * xx [ 155 ] ; pm_math_Matrix3x3_compose_ra ( xx + 470 , xx + 493 ,
xx + 766 ) ; xx [ 69 ] = xx [ 629 ] * xx [ 629 ] ; xx [ 88 ] = xx [ 630 ] *
xx [ 631 ] ; xx [ 95 ] = xx [ 629 ] * xx [ 632 ] ; xx [ 102 ] = xx [ 630 ] *
xx [ 632 ] ; xx [ 112 ] = xx [ 629 ] * xx [ 631 ] ; xx [ 113 ] = xx [ 631 ] *
xx [ 632 ] ; xx [ 155 ] = xx [ 629 ] * xx [ 630 ] ; xx [ 468 ] = ( xx [ 69 ]
+ xx [ 630 ] * xx [ 630 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 469 ] = xx [ 2 ] *
( xx [ 88 ] - xx [ 95 ] ) ; xx [ 470 ] = ( xx [ 102 ] + xx [ 112 ] ) * xx [ 2
] ; xx [ 471 ] = ( xx [ 88 ] + xx [ 95 ] ) * xx [ 2 ] ; xx [ 472 ] = ( xx [
69 ] + xx [ 631 ] * xx [ 631 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 473 ] = xx [ 2
] * ( xx [ 113 ] - xx [ 155 ] ) ; xx [ 474 ] = xx [ 2 ] * ( xx [ 102 ] - xx [
112 ] ) ; xx [ 475 ] = ( xx [ 113 ] + xx [ 155 ] ) * xx [ 2 ] ; xx [ 476 ] =
( xx [ 69 ] + xx [ 632 ] * xx [ 632 ] ) * xx [ 2 ] - xx [ 11 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 593 , xx + 325 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 468 , xx + 325 , xx + 493 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 493 , xx + 762 , xx + 922 ) ; xx [ 493
] = xx [ 687 ] + xx [ 766 ] + xx [ 2 ] * xx [ 922 ] ; xx [ 494 ] = xx [ 688 ]
+ xx [ 767 ] + xx [ 2 ] * xx [ 923 ] ; xx [ 495 ] = xx [ 689 ] + xx [ 768 ] +
xx [ 2 ] * xx [ 924 ] ; xx [ 496 ] = xx [ 690 ] + xx [ 769 ] + xx [ 2 ] * xx
[ 925 ] ; xx [ 497 ] = xx [ 691 ] + xx [ 770 ] + xx [ 2 ] * xx [ 926 ] ; xx [
498 ] = xx [ 692 ] + xx [ 771 ] + xx [ 2 ] * xx [ 927 ] ; xx [ 499 ] = xx [
693 ] + xx [ 772 ] + xx [ 2 ] * xx [ 928 ] ; xx [ 500 ] = xx [ 694 ] + xx [
773 ] + xx [ 2 ] * xx [ 929 ] ; xx [ 501 ] = xx [ 695 ] + xx [ 774 ] + xx [ 2
] * xx [ 930 ] ; pm_math_Matrix3x3_xform_ra ( xx + 493 , xx + 715 , xx + 453
) ; xx [ 461 ] = - 7.972274099557475e-18 ; xx [ 462 ] = 2.220446049250313e-15
; xx [ 463 ] = - 1.107241468650821e-16 ; pm_math_Quaternion_xform_ra ( xx +
629 , xx + 461 , xx + 477 ) ; pm_math_Matrix3x3_xform_ra ( xx + 493 , xx +
730 , xx + 461 ) ; pm_math_Matrix3x3_xform_ra ( xx + 286 , xx + 648 , xx +
480 ) ; xx [ 606 ] = - 1.099342233073701e-3 ; xx [ 607 ] =
1.637907624476696e-6 ; xx [ 608 ] = - 0.01490133116181705 ;
pm_math_Quaternion_xform_ra ( xx + 489 , xx + 606 , xx + 645 ) ; xx [ 69 ] =
inputDot [ 1 ] * inputDot [ 1 ] ; xx [ 606 ] = xx [ 645 ] * xx [ 69 ] ; xx [
607 ] = xx [ 646 ] * xx [ 69 ] ; xx [ 608 ] = xx [ 647 ] * xx [ 69 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 606 , xx + 645 ) ; xx [ 606 ] =
xx [ 129 ] ; xx [ 607 ] = xx [ 139 ] ; xx [ 608 ] = xx [ 179 ] ;
pm_math_Quaternion_xform_ra ( xx + 489 , xx + 606 , xx + 675 ) ; xx [ 606 ] =
xx [ 675 ] * inputDot [ 1 ] ; xx [ 607 ] = xx [ 676 ] * inputDot [ 1 ] ; xx [
608 ] = xx [ 677 ] * inputDot [ 1 ] ; pm_math_Vector3_cross_ra ( xx + 593 ,
xx + 606 , xx + 678 ) ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 678 ,
xx + 593 ) ; xx [ 606 ] = xx [ 461 ] + xx [ 480 ] + xx [ 47 ] + xx [ 645 ] +
xx [ 2 ] * xx [ 593 ] ; xx [ 607 ] = xx [ 462 ] + xx [ 481 ] + xx [ 48 ] + xx
[ 646 ] + xx [ 2 ] * xx [ 594 ] ; xx [ 608 ] = xx [ 463 ] + xx [ 482 ] + xx [
50 ] + xx [ 647 ] + xx [ 2 ] * xx [ 595 ] ; xx [ 461 ] = -
1.447435834859361e-6 ; xx [ 462 ] = - 0.01744954274156846 ; xx [ 463 ] =
1.376418986481207e-6 ; pm_math_Quaternion_xform_ra ( xx + 629 , xx + 461 , xx
+ 480 ) ; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 675 , xx + 461 ) ; xx
[ 593 ] = xx [ 480 ] + xx [ 461 ] ; xx [ 594 ] = xx [ 481 ] + xx [ 462 ] ; xx
[ 595 ] = xx [ 482 ] + xx [ 463 ] ; pm_math_Vector3_cross_ra ( xx + 82 , xx +
140 , xx + 461 ) ; xx [ 480 ] = xx [ 18 ] + xx [ 461 ] ; xx [ 481 ] = xx [ 19
] + xx [ 462 ] ; xx [ 482 ] = xx [ 90 ] + xx [ 463 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 480 , xx + 461 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 169 , xx + 480 ) ; xx [
47 ] = xx [ 175 ] - xx [ 62 ] * xx [ 413 ] + xx [ 414 ] ;
pm_math_Vector3_cross_ra ( xx + 169 , xx + 813 , xx + 645 ) ; xx [ 48 ] = xx
[ 176 ] - xx [ 62 ] * xx [ 427 ] + xx [ 430 ] ; xx [ 50 ] = xx [ 177 ] + xx [
435 ] ; xx [ 175 ] = xx [ 47 ] + xx [ 645 ] ; xx [ 176 ] = xx [ 48 ] + xx [
646 ] ; xx [ 177 ] = xx [ 50 ] + xx [ 647 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 175 , xx + 645 ) ; xx [
69 ] = xx [ 779 ] - ( pm_math_Vector3_dot_ra ( xx + 191 , xx + 480 ) +
pm_math_Vector3_dot_ra ( xx + 415 , xx + 645 ) ) ; xx [ 88 ] = xx [ 233 ] *
xx [ 611 ] + xx [ 663 ] * xx [ 224 ] ; xx [ 95 ] = xx [ 641 ] * xx [ 224 ] +
xx [ 611 ] * xx [ 227 ] ; xx [ 102 ] = xx [ 663 ] * xx [ 227 ] - xx [ 233 ] *
xx [ 641 ] ; xx [ 112 ] = xx [ 611 ] * xx [ 240 ] + xx [ 234 ] * xx [ 663 ] ;
xx [ 113 ] = xx [ 234 ] * xx [ 641 ] + xx [ 611 ] * xx [ 225 ] ; xx [ 129 ] =
xx [ 663 ] * xx [ 225 ] - xx [ 641 ] * xx [ 240 ] ; xx [ 139 ] = xx [ 611 ] *
xx [ 226 ] + xx [ 663 ] * xx [ 241 ] ; xx [ 155 ] = xx [ 641 ] * xx [ 241 ] +
xx [ 229 ] * xx [ 611 ] ; xx [ 166 ] = xx [ 229 ] * xx [ 663 ] - xx [ 641 ] *
xx [ 226 ] ; xx [ 687 ] = - ( xx [ 663 ] * xx [ 88 ] + xx [ 641 ] * xx [ 95 ]
) ; xx [ 688 ] = - ( xx [ 611 ] * xx [ 95 ] + xx [ 663 ] * xx [ 102 ] ) ; xx
[ 689 ] = xx [ 641 ] * xx [ 102 ] - xx [ 611 ] * xx [ 88 ] ; xx [ 690 ] = - (
xx [ 663 ] * xx [ 112 ] + xx [ 641 ] * xx [ 113 ] ) ; xx [ 691 ] = - ( xx [
611 ] * xx [ 113 ] + xx [ 663 ] * xx [ 129 ] ) ; xx [ 692 ] = xx [ 641 ] * xx
[ 129 ] - xx [ 611 ] * xx [ 112 ] ; xx [ 693 ] = - ( xx [ 663 ] * xx [ 139 ]
+ xx [ 641 ] * xx [ 155 ] ) ; xx [ 694 ] = - ( xx [ 611 ] * xx [ 155 ] + xx [
663 ] * xx [ 166 ] ) ; xx [ 695 ] = xx [ 641 ] * xx [ 166 ] - xx [ 611 ] * xx
[ 139 ] ; xx [ 175 ] = - 0.9972896958009461 ; xx [ 176 ] =
8.852854518104358e-5 ; xx [ 177 ] = 0.07357482457984132 ;
pm_math_Matrix3x3_xform_ra ( xx + 687 , xx + 175 , xx + 224 ) ; xx [ 233 ] =
- 9.045674901471507e-5 ; xx [ 234 ] = - 2.965461248705281e-3 ; xx [ 235 ] =
0.01377564771951815 ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 233 , xx
+ 675 ) ; xx [ 678 ] = - 0.01609995664951487 ; xx [ 679 ] =
2.933657308331041e-6 ; xx [ 680 ] = - 0.0137731566582556 ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 678 , xx + 709 ) ; xx [ 715 ] =
xx [ 675 ] - xx [ 811 ] + xx [ 670 ] - ( xx [ 709 ] - xx [ 787 ] + xx [ 331 ]
) ; xx [ 716 ] = xx [ 676 ] - xx [ 812 ] + xx [ 790 ] - ( xx [ 710 ] - xx [
788 ] + xx [ 332 ] ) ; xx [ 717 ] = xx [ 677 ] + xx [ 791 ] - ( xx [ 711 ] -
xx [ 789 ] + xx [ 333 ] ) ; pm_math_Matrix3x3_xform_ra ( xx + 687 , xx + 678
, xx + 331 ) ; xx [ 675 ] = - 1.099342233105492e-3 ; xx [ 676 ] =
1.637907633074165e-6 ; xx [ 677 ] = - 0.0149013311618127 ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 675 , xx + 709 ) ; xx [ 88 ] =
inputDot [ 0 ] * inputDot [ 0 ] ; xx [ 675 ] = xx [ 331 ] + xx [ 709 ] * xx [
88 ] ; xx [ 676 ] = xx [ 332 ] + xx [ 710 ] * xx [ 88 ] ; xx [ 677 ] = xx [
333 ] + xx [ 711 ] * xx [ 88 ] ; xx [ 88 ] = xx [ 365 ] * xx [ 803 ] - xx [
843 ] * xx [ 336 ] ; xx [ 95 ] = xx [ 839 ] * xx [ 336 ] - xx [ 803 ] * xx [
360 ] ; xx [ 102 ] = xx [ 843 ] * xx [ 360 ] - xx [ 365 ] * xx [ 839 ] ; xx [
112 ] = xx [ 803 ] * xx [ 377 ] - xx [ 366 ] * xx [ 843 ] ; xx [ 113 ] = xx [
366 ] * xx [ 839 ] - xx [ 803 ] * xx [ 358 ] ; xx [ 129 ] = xx [ 843 ] * xx [
358 ] - xx [ 839 ] * xx [ 377 ] ; xx [ 139 ] = xx [ 803 ] * xx [ 359 ] - xx [
843 ] * xx [ 378 ] ; xx [ 155 ] = xx [ 839 ] * xx [ 378 ] - xx [ 361 ] * xx [
803 ] ; xx [ 166 ] = xx [ 361 ] * xx [ 843 ] - xx [ 839 ] * xx [ 359 ] ; xx [
766 ] = xx [ 843 ] * xx [ 88 ] - xx [ 839 ] * xx [ 95 ] ; xx [ 767 ] = xx [
803 ] * xx [ 95 ] - xx [ 843 ] * xx [ 102 ] ; xx [ 768 ] = xx [ 839 ] * xx [
102 ] - xx [ 803 ] * xx [ 88 ] ; xx [ 769 ] = xx [ 843 ] * xx [ 112 ] - xx [
839 ] * xx [ 113 ] ; xx [ 770 ] = xx [ 803 ] * xx [ 113 ] - xx [ 843 ] * xx [
129 ] ; xx [ 771 ] = xx [ 839 ] * xx [ 129 ] - xx [ 803 ] * xx [ 112 ] ; xx [
772 ] = xx [ 843 ] * xx [ 139 ] - xx [ 839 ] * xx [ 155 ] ; xx [ 773 ] = xx [
803 ] * xx [ 155 ] - xx [ 843 ] * xx [ 166 ] ; xx [ 774 ] = xx [ 839 ] * xx [
166 ] - xx [ 803 ] * xx [ 139 ] ; pm_math_Matrix3x3_xform_ra ( xx + 766 , xx
+ 233 , xx + 331 ) ; xx [ 358 ] = - 9.509201674601914e-5 ; xx [ 359 ] = -
6.255740505361236e-3 ; xx [ 360 ] = 1.691908047920082e-3 ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 358 , xx + 709 ) ; xx [ 88 ] =
state [ 25 ] * state [ 25 ] ; xx [ 95 ] = xx [ 709 ] * xx [ 88 ] ; xx [ 102 ]
= xx [ 710 ] * xx [ 88 ] ; xx [ 112 ] = xx [ 711 ] * xx [ 88 ] ; xx [ 358 ] =
xx [ 331 ] + xx [ 95 ] ; xx [ 359 ] = xx [ 332 ] + xx [ 102 ] ; xx [ 360 ] =
xx [ 333 ] + xx [ 112 ] ; pm_math_Vector3_cross_ra ( xx + 919 , xx + 233 , xx
+ 331 ) ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 331 , xx + 233 ) ; xx
[ 88 ] = xx [ 781 ] * state [ 25 ] ; pm_math_Vector3_cross_ra ( xx + 743 , xx
+ 678 , xx + 331 ) ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 331 , xx +
678 ) ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 681 , xx + 331 ) ; xx [
113 ] = xx [ 782 ] * state [ 25 ] ; xx [ 129 ] = xx [ 783 ] * state [ 25 ] -
state [ 26 ] ; xx [ 681 ] = xx [ 233 ] + xx [ 88 ] - ( xx [ 678 ] + xx [ 331
] * inputDot [ 0 ] ) ; xx [ 682 ] = xx [ 234 ] + xx [ 113 ] - ( xx [ 679 ] +
xx [ 332 ] * inputDot [ 0 ] ) ; xx [ 683 ] = xx [ 235 ] + xx [ 129 ] - ( xx [
680 ] + xx [ 333 ] * inputDot [ 0 ] ) ; pm_math_Vector3_cross_ra ( xx + 743 ,
xx + 175 , xx + 233 ) ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 233 ,
xx + 175 ) ; xx [ 139 ] = xx [ 778 ] - ( pm_math_Vector3_dot_ra ( xx + 775 ,
xx + 480 ) + pm_math_Vector3_dot_ra ( xx + 1240 , xx + 645 ) ) ; xx [ 233 ] =
4.103874629447085e-18 ; xx [ 234 ] = 2.081668171172169e-16 ; xx [ 235 ] =
5.538917848685321e-17 ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 233 ,
xx + 480 ) ; xx [ 233 ] = - 1.435160843344562e-6 ; xx [ 234 ] = -
0.0149203787001616 ; xx [ 235 ] = - 1.500400464949372e-6 ;
pm_math_Quaternion_xform_ra ( xx + 236 , xx + 233 , xx + 645 ) ; xx [ 233 ] =
xx [ 645 ] + xx [ 331 ] ; xx [ 234 ] = xx [ 646 ] + xx [ 332 ] ; xx [ 235 ] =
xx [ 647 ] + xx [ 333 ] ; pm_math_Matrix3x3_compose_ra ( xx + 213 , xx + 766
, xx + 922 ) ; xx [ 213 ] = 3.734971681886328e-4 ; xx [ 214 ] =
0.2610715860287173 ; xx [ 215 ] = xx [ 363 ] ; pm_math_Matrix3x3_xform_ra (
xx + 922 , xx + 213 , xx + 216 ) ; pm_math_Matrix3x3_xform_ra ( xx + 922 , xx
+ 816 , xx + 219 ) ; xx [ 331 ] = xx [ 95 ] ; xx [ 332 ] = xx [ 102 ] ; xx [
333 ] = xx [ 112 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 331 , xx +
645 ) ; xx [ 331 ] = xx [ 219 ] + xx [ 645 ] ; xx [ 332 ] = xx [ 220 ] + xx [
646 ] ; xx [ 333 ] = xx [ 221 ] + xx [ 647 ] ; pm_math_Vector3_cross_ra ( xx
+ 919 , xx + 816 , xx + 219 ) ; pm_math_Quaternion_xform_ra ( xx + 804 , xx +
219 , xx + 645 ) ; xx [ 219 ] = xx [ 88 ] ; xx [ 220 ] = xx [ 113 ] ; xx [
221 ] = xx [ 129 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 219 , xx +
678 ) ; xx [ 219 ] = - ( xx [ 143 ] * state [ 34 ] + xx [ 645 ] + xx [ 837 ]
* state [ 22 ] + xx [ 678 ] ) ; xx [ 220 ] = - ( xx [ 160 ] * state [ 34 ] +
xx [ 646 ] + xx [ 679 ] - xx [ 840 ] * state [ 22 ] ) ; xx [ 221 ] = xx [ 194
] * state [ 34 ] - ( xx [ 647 ] + xx [ 842 ] * state [ 22 ] + xx [ 680 ] ) ;
pm_math_Vector3_cross_ra ( xx + 919 , xx + 213 , xx + 645 ) ;
pm_math_Quaternion_xform_ra ( xx + 804 , xx + 645 , xx + 213 ) ;
pm_math_Vector3_cross_ra ( xx + 82 , xx + 449 , xx + 645 ) ; xx [ 82 ] = xx [
18 ] + xx [ 645 ] ; xx [ 83 ] = xx [ 19 ] + xx [ 646 ] ; xx [ 84 ] = xx [ 90
] + xx [ 647 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 82 ,
xx + 645 ) ; xx [ 82 ] = xx [ 85 ] + xx [ 442 ] * xx [ 41 ] + xx [ 596 ] ; xx
[ 83 ] = xx [ 86 ] + xx [ 458 ] * xx [ 41 ] + xx [ 597 ] ; xx [ 84 ] = xx [
87 ] + xx [ 459 ] * xx [ 41 ] + xx [ 598 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 82 , xx + 85 ) ; xx [
678 ] = xx [ 85 ] + xx [ 617 ] ; xx [ 679 ] = xx [ 86 ] + xx [ 618 ] ; xx [
680 ] = xx [ 87 ] + xx [ 638 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
545 , xx + 678 , xx + 85 ) ; pm_math_Vector3_cross_ra ( xx + 82 , xx + 648 ,
xx + 709 ) ; xx [ 82 ] = xx [ 92 ] - xx [ 655 ] * xx [ 41 ] + xx [ 442 ] * xx
[ 17 ] + xx [ 55 ] + xx [ 709 ] ; xx [ 83 ] = xx [ 93 ] + xx [ 656 ] * xx [
41 ] + xx [ 458 ] * xx [ 17 ] + xx [ 96 ] + xx [ 710 ] ; xx [ 84 ] = xx [ 94
] - xx [ 699 ] * xx [ 41 ] + xx [ 459 ] * xx [ 17 ] + xx [ 106 ] + xx [ 711 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 82 , xx + 92 ) ;
pm_math_Vector3_cross_ra ( xx + 678 , xx + 624 , xx + 82 ) ; xx [ 678 ] = xx
[ 92 ] + xx [ 340 ] + xx [ 82 ] ; xx [ 679 ] = xx [ 93 ] + xx [ 341 ] + xx [
83 ] ; xx [ 680 ] = xx [ 94 ] + xx [ 274 ] + xx [ 84 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 678 , xx + 82 ) ; xx [
18 ] = xx [ 533 ] * xx [ 448 ] + xx [ 572 ] * xx [ 508 ] ; xx [ 19 ] = xx [
488 ] * xx [ 508 ] + xx [ 448 ] * xx [ 529 ] ; xx [ 88 ] = xx [ 533 ] * xx [
488 ] - xx [ 572 ] * xx [ 529 ] ; xx [ 90 ] = xx [ 448 ] * xx [ 549 ] + xx [
534 ] * xx [ 572 ] ; xx [ 92 ] = xx [ 534 ] * xx [ 488 ] + xx [ 448 ] * xx [
527 ] ; xx [ 93 ] = xx [ 488 ] * xx [ 549 ] - xx [ 572 ] * xx [ 527 ] ; xx [
94 ] = xx [ 448 ] * xx [ 528 ] + xx [ 572 ] * xx [ 550 ] ; xx [ 95 ] = xx [
488 ] * xx [ 550 ] + xx [ 530 ] * xx [ 448 ] ; xx [ 102 ] = xx [ 488 ] * xx [
528 ] - xx [ 530 ] * xx [ 572 ] ; xx [ 766 ] = - ( xx [ 572 ] * xx [ 18 ] +
xx [ 488 ] * xx [ 19 ] ) ; xx [ 767 ] = xx [ 572 ] * xx [ 88 ] - xx [ 448 ] *
xx [ 19 ] ; xx [ 768 ] = - ( xx [ 488 ] * xx [ 88 ] + xx [ 448 ] * xx [ 18 ]
) ; xx [ 769 ] = - ( xx [ 572 ] * xx [ 90 ] + xx [ 488 ] * xx [ 92 ] ) ; xx [
770 ] = xx [ 572 ] * xx [ 93 ] - xx [ 448 ] * xx [ 92 ] ; xx [ 771 ] = - ( xx
[ 488 ] * xx [ 93 ] + xx [ 448 ] * xx [ 90 ] ) ; xx [ 772 ] = - ( xx [ 572 ]
* xx [ 94 ] + xx [ 488 ] * xx [ 95 ] ) ; xx [ 773 ] = xx [ 572 ] * xx [ 102 ]
- xx [ 448 ] * xx [ 95 ] ; xx [ 774 ] = - ( xx [ 488 ] * xx [ 102 ] + xx [
448 ] * xx [ 94 ] ) ; xx [ 18 ] = 0.9999868227746884 ; xx [ 92 ] = -
3.059547932262596e-4 ; xx [ 93 ] = xx [ 18 ] ; xx [ 94 ] =
5.124516430675907e-3 ; pm_math_Matrix3x3_xform_ra ( xx + 766 , xx + 92 , xx +
486 ) ; pm_math_Matrix3x3_xform_ra ( xx + 766 , xx + 894 , xx + 527 ) ; xx [
678 ] = 1.428773235307968e-6 ; xx [ 679 ] = - 1.252118193655733e-9 ; xx [ 680
] = 3.296392424220902e-7 ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 678
, xx + 709 ) ; xx [ 19 ] = state [ 19 ] * state [ 19 ] ; xx [ 678 ] = xx [
527 ] + xx [ 709 ] * xx [ 19 ] ; xx [ 679 ] = xx [ 528 ] + xx [ 710 ] * xx [
19 ] ; xx [ 680 ] = xx [ 529 ] + xx [ 711 ] * xx [ 19 ] ; xx [ 19 ] = xx [
509 ] * xx [ 184 ] + xx [ 211 ] * xx [ 485 ] ; xx [ 88 ] = xx [ 203 ] * xx [
485 ] + xx [ 184 ] * xx [ 506 ] ; xx [ 90 ] = xx [ 211 ] * xx [ 506 ] - xx [
509 ] * xx [ 203 ] ; xx [ 95 ] = xx [ 184 ] * xx [ 516 ] + xx [ 510 ] * xx [
211 ] ; xx [ 102 ] = xx [ 510 ] * xx [ 203 ] + xx [ 184 ] * xx [ 504 ] ; xx [
112 ] = xx [ 211 ] * xx [ 504 ] - xx [ 203 ] * xx [ 516 ] ; xx [ 113 ] = xx [
184 ] * xx [ 505 ] + xx [ 211 ] * xx [ 517 ] ; xx [ 129 ] = xx [ 203 ] * xx [
517 ] + xx [ 507 ] * xx [ 184 ] ; xx [ 143 ] = xx [ 507 ] * xx [ 211 ] - xx [
203 ] * xx [ 505 ] ; xx [ 502 ] = - ( xx [ 211 ] * xx [ 19 ] + xx [ 203 ] *
xx [ 88 ] ) ; xx [ 503 ] = - ( xx [ 184 ] * xx [ 88 ] + xx [ 211 ] * xx [ 90
] ) ; xx [ 504 ] = xx [ 203 ] * xx [ 90 ] - xx [ 184 ] * xx [ 19 ] ; xx [ 505
] = - ( xx [ 211 ] * xx [ 95 ] + xx [ 203 ] * xx [ 102 ] ) ; xx [ 506 ] = - (
xx [ 184 ] * xx [ 102 ] + xx [ 211 ] * xx [ 112 ] ) ; xx [ 507 ] = xx [ 203 ]
* xx [ 112 ] - xx [ 184 ] * xx [ 95 ] ; xx [ 508 ] = - ( xx [ 211 ] * xx [
113 ] + xx [ 203 ] * xx [ 129 ] ) ; xx [ 509 ] = - ( xx [ 184 ] * xx [ 129 ]
+ xx [ 211 ] * xx [ 143 ] ) ; xx [ 510 ] = xx [ 203 ] * xx [ 143 ] - xx [ 184
] * xx [ 113 ] ; pm_math_Matrix3x3_xform_ra ( xx + 502 , xx + 885 , xx + 527
) ; xx [ 502 ] = - 3.938183989418748e-8 ; xx [ 503 ] = 5.589510653689871e-13
; xx [ 504 ] = - 2.971385142506328e-7 ; pm_math_Quaternion_xform_ra ( xx +
512 , xx + 502 , xx + 505 ) ; xx [ 19 ] = inputDot [ 2 ] * inputDot [ 2 ] ;
xx [ 502 ] = xx [ 527 ] + xx [ 505 ] * xx [ 19 ] ; xx [ 503 ] = xx [ 528 ] +
xx [ 506 ] * xx [ 19 ] ; xx [ 504 ] = xx [ 529 ] + xx [ 507 ] * xx [ 19 ] ;
pm_math_Vector3_cross_ra ( xx + 590 , xx + 885 , xx + 505 ) ;
pm_math_Quaternion_xform_ra ( xx + 512 , xx + 505 , xx + 508 ) ; xx [ 505 ] =
xx [ 197 ] ; xx [ 506 ] = xx [ 209 ] ; xx [ 507 ] = - xx [ 212 ] ;
pm_math_Quaternion_xform_ra ( xx + 512 , xx + 505 , xx + 208 ) ;
pm_math_Vector3_cross_ra ( xx + 620 , xx + 894 , xx + 505 ) ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 505 , xx + 527 ) ; xx [ 505 ] =
xx [ 508 ] + xx [ 208 ] * inputDot [ 2 ] - ( xx [ 527 ] + xx [ 897 ] * state
[ 19 ] + xx [ 223 ] * state [ 20 ] ) ; xx [ 506 ] = xx [ 509 ] + xx [ 209 ] *
inputDot [ 2 ] - ( xx [ 528 ] + xx [ 898 ] * state [ 19 ] + xx [ 579 ] *
state [ 20 ] ) ; xx [ 507 ] = xx [ 510 ] + xx [ 210 ] * inputDot [ 2 ] - ( xx
[ 529 ] + xx [ 899 ] * state [ 19 ] + xx [ 623 ] * state [ 20 ] ) ;
pm_math_Vector3_cross_ra ( xx + 620 , xx + 92 , xx + 508 ) ;
pm_math_Quaternion_xform_ra ( xx + 545 , xx + 508 , xx + 92 ) ; xx [ 19 ] =
xx [ 615 ] - ( pm_math_Vector3_dot_ra ( xx + 830 , xx + 85 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 82 ) ) ; xx [ 508 ] = -
2.971385142194091e-7 ; xx [ 509 ] = - 1.158326953740249e-13 ; xx [ 510 ] =
3.938183990803402e-8 ; pm_math_Quaternion_xform_ra ( xx + 512 , xx + 508 , xx
+ 527 ) ; xx [ 508 ] = xx [ 527 ] + xx [ 208 ] ; xx [ 509 ] = xx [ 528 ] + xx
[ 209 ] ; xx [ 510 ] = xx [ 529 ] + xx [ 210 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 169 , xx + 208 ) ; xx [
527 ] = xx [ 208 ] + xx [ 644 ] ; xx [ 528 ] = xx [ 209 ] + xx [ 741 ] ; xx [
529 ] = xx [ 210 ] + xx [ 747 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
299 , xx + 527 , xx + 208 ) ; pm_math_Vector3_cross_ra ( xx + 169 , xx + 784
, xx + 590 ) ; xx [ 169 ] = xx [ 47 ] + xx [ 590 ] ; xx [ 170 ] = xx [ 48 ] +
xx [ 591 ] ; xx [ 171 ] = xx [ 50 ] + xx [ 592 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 169 , xx + 590 ) ;
pm_math_Vector3_cross_ra ( xx + 527 , xx + 736 , xx + 169 ) ; xx [ 527 ] = xx
[ 590 ] + xx [ 765 ] + xx [ 169 ] ; xx [ 528 ] = xx [ 591 ] + xx [ 795 ] + xx
[ 170 ] ; xx [ 529 ] = xx [ 592 ] + xx [ 802 ] + xx [ 171 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 527 , xx + 169 ) ; xx [
47 ] = xx [ 285 ] * xx [ 222 ] + xx [ 740 ] * xx [ 277 ] ; xx [ 48 ] = xx [
425 ] * xx [ 277 ] + xx [ 222 ] * xx [ 280 ] ; xx [ 50 ] = xx [ 285 ] * xx [
425 ] - xx [ 740 ] * xx [ 280 ] ; xx [ 88 ] = xx [ 222 ] * xx [ 297 ] + xx [
295 ] * xx [ 740 ] ; xx [ 90 ] = xx [ 295 ] * xx [ 425 ] + xx [ 222 ] * xx [
278 ] ; xx [ 95 ] = xx [ 425 ] * xx [ 297 ] - xx [ 740 ] * xx [ 278 ] ; xx [
102 ] = xx [ 222 ] * xx [ 279 ] + xx [ 740 ] * xx [ 298 ] ; xx [ 112 ] = xx [
425 ] * xx [ 298 ] + xx [ 281 ] * xx [ 222 ] ; xx [ 113 ] = xx [ 425 ] * xx [
279 ] - xx [ 281 ] * xx [ 740 ] ; xx [ 885 ] = - ( xx [ 740 ] * xx [ 47 ] +
xx [ 425 ] * xx [ 48 ] ) ; xx [ 886 ] = xx [ 740 ] * xx [ 50 ] - xx [ 222 ] *
xx [ 48 ] ; xx [ 887 ] = - ( xx [ 425 ] * xx [ 50 ] + xx [ 222 ] * xx [ 47 ]
) ; xx [ 888 ] = - ( xx [ 740 ] * xx [ 88 ] + xx [ 425 ] * xx [ 90 ] ) ; xx [
889 ] = xx [ 740 ] * xx [ 95 ] - xx [ 222 ] * xx [ 90 ] ; xx [ 890 ] = - ( xx
[ 425 ] * xx [ 95 ] + xx [ 222 ] * xx [ 88 ] ) ; xx [ 891 ] = - ( xx [ 740 ]
* xx [ 102 ] + xx [ 425 ] * xx [ 112 ] ) ; xx [ 892 ] = xx [ 740 ] * xx [ 113
] - xx [ 222 ] * xx [ 112 ] ; xx [ 893 ] = - ( xx [ 425 ] * xx [ 113 ] + xx [
222 ] * xx [ 102 ] ) ; xx [ 277 ] = - 3.059547932248718e-4 ; xx [ 278 ] = xx
[ 18 ] ; xx [ 279 ] = xx [ 531 ] ; pm_math_Matrix3x3_xform_ra ( xx + 885 , xx
+ 277 , xx + 527 ) ; pm_math_Matrix3x3_xform_ra ( xx + 885 , xx + 966 , xx +
590 ) ; xx [ 709 ] = 1.428773235466369e-6 ; xx [ 710 ] = -
1.252118195191106e-9 ; xx [ 711 ] = 3.296392427306233e-7 ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 709 , xx + 721 ) ; xx [ 18 ] =
state [ 29 ] * state [ 29 ] ; xx [ 709 ] = xx [ 590 ] + xx [ 721 ] * xx [ 18
] ; xx [ 710 ] = xx [ 591 ] + xx [ 722 ] * xx [ 18 ] ; xx [ 711 ] = xx [ 592
] + xx [ 723 ] * xx [ 18 ] ; xx [ 18 ] = xx [ 259 ] * xx [ 669 ] + xx [ 708 ]
* xx [ 251 ] ; xx [ 47 ] = xx [ 697 ] * xx [ 251 ] + xx [ 669 ] * xx [ 254 ]
; xx [ 48 ] = xx [ 708 ] * xx [ 254 ] - xx [ 259 ] * xx [ 697 ] ; xx [ 50 ] =
xx [ 669 ] * xx [ 266 ] + xx [ 260 ] * xx [ 708 ] ; xx [ 88 ] = xx [ 260 ] *
xx [ 697 ] + xx [ 669 ] * xx [ 252 ] ; xx [ 90 ] = xx [ 708 ] * xx [ 252 ] -
xx [ 697 ] * xx [ 266 ] ; xx [ 95 ] = xx [ 669 ] * xx [ 253 ] + xx [ 708 ] *
xx [ 267 ] ; xx [ 102 ] = xx [ 697 ] * xx [ 267 ] + xx [ 255 ] * xx [ 669 ] ;
xx [ 112 ] = xx [ 255 ] * xx [ 708 ] - xx [ 697 ] * xx [ 253 ] ; xx [ 1043 ]
= - ( xx [ 708 ] * xx [ 18 ] + xx [ 697 ] * xx [ 47 ] ) ; xx [ 1044 ] = - (
xx [ 669 ] * xx [ 47 ] + xx [ 708 ] * xx [ 48 ] ) ; xx [ 1045 ] = xx [ 697 ]
* xx [ 48 ] - xx [ 669 ] * xx [ 18 ] ; xx [ 1046 ] = - ( xx [ 708 ] * xx [ 50
] + xx [ 697 ] * xx [ 88 ] ) ; xx [ 1047 ] = - ( xx [ 669 ] * xx [ 88 ] + xx
[ 708 ] * xx [ 90 ] ) ; xx [ 1048 ] = xx [ 697 ] * xx [ 90 ] - xx [ 669 ] *
xx [ 50 ] ; xx [ 1049 ] = - ( xx [ 708 ] * xx [ 95 ] + xx [ 697 ] * xx [ 102
] ) ; xx [ 1050 ] = - ( xx [ 669 ] * xx [ 102 ] + xx [ 708 ] * xx [ 112 ] ) ;
xx [ 1051 ] = xx [ 697 ] * xx [ 112 ] - xx [ 669 ] * xx [ 95 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 1043 , xx + 916 , xx + 251 ) ; xx [ 590 ] =
- 3.938183964288614e-8 ; xx [ 591 ] = 5.589510653062229e-13 ; xx [ 592 ] = -
2.971385144598395e-7 ; pm_math_Quaternion_xform_ra ( xx + 262 , xx + 590 , xx
+ 668 ) ; xx [ 18 ] = inputDot [ 3 ] * inputDot [ 3 ] ; xx [ 590 ] = xx [ 251
] + xx [ 668 ] * xx [ 18 ] ; xx [ 591 ] = xx [ 252 ] + xx [ 669 ] * xx [ 18 ]
; xx [ 592 ] = xx [ 253 ] + xx [ 670 ] * xx [ 18 ] ; pm_math_Vector3_cross_ra
( xx + 421 , xx + 916 , xx + 251 ) ; pm_math_Quaternion_xform_ra ( xx + 262 ,
xx + 251 , xx + 421 ) ; xx [ 251 ] = xx [ 725 ] ; xx [ 252 ] = xx [ 726 ] ;
xx [ 253 ] = - xx [ 739 ] ; pm_math_Quaternion_xform_ra ( xx + 262 , xx + 251
, xx + 668 ) ; pm_math_Vector3_cross_ra ( xx + 819 , xx + 966 , xx + 251 ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 251 , xx + 696 ) ; xx [ 251 ] =
xx [ 421 ] + xx [ 668 ] * inputDot [ 3 ] - ( xx [ 696 ] + xx [ 993 ] * state
[ 29 ] + xx [ 24 ] * state [ 30 ] ) ; xx [ 252 ] = xx [ 422 ] + xx [ 669 ] *
inputDot [ 3 ] - ( xx [ 697 ] + xx [ 994 ] * state [ 29 ] + xx [ 614 ] *
state [ 30 ] ) ; xx [ 253 ] = xx [ 423 ] + xx [ 670 ] * inputDot [ 3 ] - ( xx
[ 698 ] + xx [ 995 ] * state [ 29 ] + xx [ 619 ] * state [ 30 ] ) ;
pm_math_Vector3_cross_ra ( xx + 819 , xx + 277 , xx + 421 ) ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 421 , xx + 277 ) ; xx [ 18 ] =
xx [ 793 ] - ( pm_math_Vector3_dot_ra ( xx + 584 , xx + 208 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 169 ) ) ; xx [ 421 ] = -
2.971385141348656e-7 ; xx [ 422 ] = - 1.158326949952305e-13 ; xx [ 423 ] =
3.938183974928171e-8 ; pm_math_Quaternion_xform_ra ( xx + 262 , xx + 421 , xx
+ 696 ) ; xx [ 421 ] = xx [ 696 ] + xx [ 668 ] ; xx [ 422 ] = xx [ 697 ] + xx
[ 669 ] ; xx [ 423 ] = xx [ 698 ] + xx [ 670 ] ; pm_math_Matrix3x3_compose_ra
( xx + 493 , xx + 551 , xx + 1043 ) ; pm_math_Matrix3x3_compose_ra ( xx + 468
, xx + 766 , xx + 493 ) ; xx [ 24 ] = xx [ 1018 ] * xx [ 1018 ] ; xx [ 47 ] =
xx [ 1019 ] * xx [ 1020 ] ; xx [ 48 ] = xx [ 1018 ] * xx [ 1021 ] ; xx [ 50 ]
= xx [ 1019 ] * xx [ 1021 ] ; xx [ 88 ] = xx [ 1018 ] * xx [ 1020 ] ; xx [ 90
] = xx [ 1020 ] * xx [ 1021 ] ; xx [ 95 ] = xx [ 1018 ] * xx [ 1019 ] ; xx [
468 ] = ( xx [ 24 ] + xx [ 1019 ] * xx [ 1019 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 469 ] = xx [ 2 ] * ( xx [ 47 ] - xx [ 48 ] ) ; xx [ 470 ] = ( xx [ 50 ] +
xx [ 88 ] ) * xx [ 2 ] ; xx [ 471 ] = ( xx [ 47 ] + xx [ 48 ] ) * xx [ 2 ] ;
xx [ 472 ] = ( xx [ 24 ] + xx [ 1020 ] * xx [ 1020 ] ) * xx [ 2 ] - xx [ 11 ]
; xx [ 473 ] = xx [ 2 ] * ( xx [ 90 ] - xx [ 95 ] ) ; xx [ 474 ] = xx [ 2 ] *
( xx [ 50 ] - xx [ 88 ] ) ; xx [ 475 ] = ( xx [ 90 ] + xx [ 95 ] ) * xx [ 2 ]
; xx [ 476 ] = ( xx [ 24 ] + xx [ 1021 ] * xx [ 1021 ] ) * xx [ 2 ] - xx [ 11
] ; xx [ 549 ] = xx [ 325 ] - xx [ 111 ] ; xx [ 550 ] = xx [ 326 ] + xx [ 130
] ; xx [ 551 ] = xx [ 327 ] + xx [ 158 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 545 , xx + 549 , xx + 111 ) ; pm_math_Matrix3x3_postCross_ra ( xx +
468 , xx + 111 , xx + 549 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 549 , xx
+ 620 , xx + 468 ) ; xx [ 549 ] = xx [ 1043 ] + xx [ 493 ] + xx [ 2 ] * xx [
468 ] ; xx [ 550 ] = xx [ 1044 ] + xx [ 494 ] + xx [ 2 ] * xx [ 469 ] ; xx [
551 ] = xx [ 1045 ] + xx [ 495 ] + xx [ 2 ] * xx [ 470 ] ; xx [ 552 ] = xx [
1046 ] + xx [ 496 ] + xx [ 2 ] * xx [ 471 ] ; xx [ 553 ] = xx [ 1047 ] + xx [
497 ] + xx [ 2 ] * xx [ 472 ] ; xx [ 554 ] = xx [ 1048 ] + xx [ 498 ] + xx [
2 ] * xx [ 473 ] ; xx [ 555 ] = xx [ 1049 ] + xx [ 499 ] + xx [ 2 ] * xx [
474 ] ; xx [ 556 ] = xx [ 1050 ] + xx [ 500 ] + xx [ 2 ] * xx [ 475 ] ; xx [
557 ] = xx [ 1051 ] + xx [ 501 ] + xx [ 2 ] * xx [ 476 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 549 , xx + 1012 , xx + 111 ) ; xx [ 325 ] =
- xx [ 460 ] ; xx [ 326 ] = xx [ 483 ] ; xx [ 327 ] = xx [ 484 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 325 , xx + 468 ) ;
pm_math_Vector3_cross_ra ( xx + 468 , xx + 1012 , xx + 325 ) ;
pm_math_Quaternion_xform_ra ( xx + 1018 , xx + 325 , xx + 468 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 687 , xx + 303 , xx + 493 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 242 , xx + 885 , xx + 303 ) ; xx [ 24 ] =
xx [ 1056 ] * xx [ 1056 ] ; xx [ 47 ] = xx [ 1057 ] * xx [ 1058 ] ; xx [ 48 ]
= xx [ 1056 ] * xx [ 1059 ] ; xx [ 50 ] = xx [ 1057 ] * xx [ 1059 ] ; xx [ 88
] = xx [ 1056 ] * xx [ 1058 ] ; xx [ 90 ] = xx [ 1058 ] * xx [ 1059 ] ; xx [
95 ] = xx [ 1056 ] * xx [ 1057 ] ; xx [ 240 ] = ( xx [ 24 ] + xx [ 1057 ] *
xx [ 1057 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 241 ] = xx [ 2 ] * ( xx [ 47 ] -
xx [ 48 ] ) ; xx [ 242 ] = ( xx [ 50 ] + xx [ 88 ] ) * xx [ 2 ] ; xx [ 243 ]
= ( xx [ 47 ] + xx [ 48 ] ) * xx [ 2 ] ; xx [ 244 ] = ( xx [ 24 ] + xx [ 1058
] * xx [ 1058 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 245 ] = xx [ 2 ] * ( xx [ 90
] - xx [ 95 ] ) ; xx [ 246 ] = xx [ 2 ] * ( xx [ 50 ] - xx [ 88 ] ) ; xx [
247 ] = ( xx [ 90 ] + xx [ 95 ] ) * xx [ 2 ] ; xx [ 248 ] = ( xx [ 24 ] + xx
[ 1059 ] * xx [ 1059 ] ) * xx [ 2 ] - xx [ 11 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 743 , xx + 325 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 240 , xx + 325 , xx + 549 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 549 , xx + 819 , xx + 240 ) ; xx [ 549
] = xx [ 493 ] + xx [ 303 ] + xx [ 2 ] * xx [ 240 ] ; xx [ 550 ] = xx [ 494 ]
+ xx [ 304 ] + xx [ 2 ] * xx [ 241 ] ; xx [ 551 ] = xx [ 495 ] + xx [ 305 ] +
xx [ 2 ] * xx [ 242 ] ; xx [ 552 ] = xx [ 496 ] + xx [ 306 ] + xx [ 2 ] * xx
[ 243 ] ; xx [ 553 ] = xx [ 497 ] + xx [ 307 ] + xx [ 2 ] * xx [ 244 ] ; xx [
554 ] = xx [ 498 ] + xx [ 308 ] + xx [ 2 ] * xx [ 245 ] ; xx [ 555 ] = xx [
499 ] + xx [ 309 ] + xx [ 2 ] * xx [ 246 ] ; xx [ 556 ] = xx [ 500 ] + xx [
310 ] + xx [ 2 ] * xx [ 247 ] ; xx [ 557 ] = xx [ 501 ] + xx [ 311 ] + xx [ 2
] * xx [ 248 ] ; xx [ 240 ] = - 0.03229790576921281 ; xx [ 241 ] = -
5.131724710788964e-3 ; xx [ 242 ] = 0.9994651122897766 ;
pm_math_Matrix3x3_xform_ra ( xx + 549 , xx + 240 , xx + 243 ) ; xx [ 245 ] =
- xx [ 228 ] ; xx [ 246 ] = xx [ 230 ] ; xx [ 247 ] = xx [ 231 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 245 , xx + 227 ) ;
pm_math_Vector3_cross_ra ( xx + 227 , xx + 240 , xx + 245 ) ;
pm_math_Quaternion_xform_ra ( xx + 1056 , xx + 245 , xx + 227 ) ; xx [ 229 ]
= 1.81142804615305e-4 ; xx [ 230 ] = - xx [ 1109 ] ; xx [ 231 ] =
0.2610715346452743 ; pm_math_Matrix3x3_xform_ra ( xx + 286 , xx + 229 , xx +
240 ) ; xx [ 229 ] = 0.9999999138435713 ; xx [ 230 ] = 7.735117927198942e-5 ;
xx [ 231 ] = - 4.078353164850834e-4 ; pm_math_Matrix3x3_xform_ra ( xx + 922 ,
xx + 229 , xx + 245 ) ; xx [ 916 ] = xx [ 64 ] * xx [ 17 ] - (
pm_math_Vector3_dot_ra ( xx + 118 , xx + 185 ) - pm_math_Vector3_dot_ra ( xx
+ 149 , xx + 188 ) + pm_math_Vector3_dot_ra ( xx + 145 , xx + 115 ) * xx [ 2
] + xx [ 63 ] * xx [ 41 ] ) + xx [ 81 ] * ( xx [ 610 ] + xx [ 201 ] * xx [
156 ] + xx [ 207 ] * xx [ 157 ] ) ; xx [ 917 ] = - ( xx [ 62 ] * xx [ 97 ] )
; xx [ 918 ] = - ( pm_math_Vector3_dot_ra ( xx + 453 , xx + 718 ) + xx [ 167
] * xx [ 41 ] + pm_math_Vector3_dot_ra ( xx + 477 , xx + 718 ) * inputDdot [
1 ] ) ; xx [ 919 ] = - ( pm_math_Vector3_dot_ra ( xx + 453 , xx + 727 ) + xx
[ 444 ] * xx [ 41 ] + pm_math_Vector3_dot_ra ( xx + 477 , xx + 727 ) *
inputDdot [ 1 ] ) ; xx [ 920 ] = - ( pm_math_Vector3_dot_ra ( xx + 606 , xx +
712 ) + xx [ 576 ] * xx [ 41 ] + xx [ 577 ] * xx [ 17 ] +
pm_math_Vector3_dot_ra ( xx + 593 , xx + 712 ) * inputDdot [ 1 ] ) ; xx [ 921
] = - ( ( xx [ 135 ] + pm_math_Vector3_dot_ra ( xx + 1231 , xx + 461 ) ) * xx
[ 57 ] ) ; xx [ 922 ] = xx [ 760 ] * xx [ 69 ] - ( pm_math_Vector3_dot_ra (
xx + 224 , xx + 715 ) - pm_math_Vector3_dot_ra ( xx + 675 , xx + 796 ) +
pm_math_Vector3_dot_ra ( xx + 358 , xx + 796 ) + pm_math_Vector3_dot_ra ( xx
+ 681 , xx + 175 ) * xx [ 2 ] + xx [ 761 ] * xx [ 139 ] ) - (
pm_math_Vector3_dot_ra ( xx + 480 , xx + 715 ) - pm_math_Vector3_dot_ra ( xx
+ 233 , xx + 796 ) ) * inputDdot [ 0 ] ; xx [ 923 ] = xx [ 181 ] * xx [ 69 ]
- ( pm_math_Vector3_dot_ra ( xx + 216 , xx + 833 ) - pm_math_Vector3_dot_ra (
xx + 331 , xx + 827 ) + pm_math_Vector3_dot_ra ( xx + 219 , xx + 213 ) * xx [
2 ] + xx [ 198 ] * xx [ 62 ] + xx [ 59 ] * xx [ 139 ] ) - xx [ 838 ] * ( xx [
571 ] + xx [ 165 ] * xx [ 646 ] + xx [ 195 ] * xx [ 647 ] ) ; xx [ 924 ] = xx
[ 580 ] * ( xx [ 616 ] - ( pm_math_Vector3_dot_ra ( xx + 905 , xx + 85 ) +
pm_math_Vector3_dot_ra ( xx + 581 , xx + 82 ) ) ) - ( pm_math_Vector3_dot_ra
( xx + 486 , xx + 900 ) - pm_math_Vector3_dot_ra ( xx + 678 , xx + 573 ) +
pm_math_Vector3_dot_ra ( xx + 502 , xx + 573 ) + pm_math_Vector3_dot_ra ( xx
+ 505 , xx + 92 ) * xx [ 2 ] + xx [ 578 ] * xx [ 19 ] ) -
pm_math_Vector3_dot_ra ( xx + 508 , xx + 573 ) * inputDdot [ 2 ] ; xx [ 925 ]
= xx [ 330 ] * ( xx [ 794 ] - ( pm_math_Vector3_dot_ra ( xx + 910 , xx + 208
) + pm_math_Vector3_dot_ra ( xx + 337 , xx + 169 ) ) ) - (
pm_math_Vector3_dot_ra ( xx + 527 , xx + 990 ) - pm_math_Vector3_dot_ra ( xx
+ 709 , xx + 947 ) + pm_math_Vector3_dot_ra ( xx + 590 , xx + 947 ) +
pm_math_Vector3_dot_ra ( xx + 251 , xx + 277 ) * xx [ 2 ] + xx [ 329 ] * xx [
18 ] ) - pm_math_Vector3_dot_ra ( xx + 421 , xx + 947 ) * inputDdot [ 3 ] ;
xx [ 926 ] = - ( pm_math_Vector3_dot_ra ( xx + 111 , xx + 1015 ) + xx [ 939 ]
* xx [ 41 ] + pm_math_Vector3_dot_ra ( xx + 468 , xx + 1015 ) * inputDdot [ 1
] + xx [ 958 ] * xx [ 19 ] ) ; xx [ 927 ] = - ( pm_math_Vector3_dot_ra ( xx +
111 , xx + 1039 ) + xx [ 1042 ] * xx [ 41 ] + pm_math_Vector3_dot_ra ( xx +
468 , xx + 1039 ) * inputDdot [ 1 ] + xx [ 1006 ] * xx [ 19 ] ) ; xx [ 928 ]
= - ( xx [ 1055 ] * xx [ 243 ] + xx [ 1060 ] * xx [ 244 ] + ( xx [ 1055 ] *
xx [ 227 ] + xx [ 1060 ] * xx [ 228 ] ) * inputDdot [ 0 ] + xx [ 1061 ] * xx
[ 18 ] ) ; xx [ 929 ] = - ( xx [ 1055 ] * xx [ 244 ] - xx [ 1060 ] * xx [ 243
] + ( xx [ 1055 ] * xx [ 228 ] - xx [ 1060 ] * xx [ 227 ] ) * inputDdot [ 0 ]
+ xx [ 1081 ] * xx [ 18 ] ) ; xx [ 930 ] = - ( pm_math_Vector3_dot_ra ( xx +
240 , xx + 1097 ) + xx [ 1105 ] * xx [ 41 ] ) ; xx [ 931 ] = - (
pm_math_Vector3_dot_ra ( xx + 245 , xx + 1118 ) + xx [ 1110 ] * xx [ 139 ] )
; memcpy ( xx + 1503 , xx + 1247 , 256 * sizeof ( double ) ) ;
factorAndSolveSymmetric ( xx + 1503 , 16 , xx + 240 , ii + 0 , xx + 916 , xx
+ 208 , xx + 1759 ) ; xx [ 2 ] = ( xx [ 57 ] * xx [ 213 ] + xx [ 53 ] ) / xx
[ 178 ] ; xx [ 17 ] = xx [ 56 ] - xx [ 148 ] * xx [ 2 ] ; xx [ 18 ] = xx [ 39
] - xx [ 180 ] * xx [ 2 ] ; xx [ 19 ] = xx [ 40 ] - xx [ 182 ] * xx [ 2 ] ;
pm_math_Quaternion_xform_ra ( xx + 125 , xx + 17 , xx + 39 ) ; xx [ 11 ] = (
xx [ 838 ] * xx [ 215 ] + xx [ 589 ] ) / xx [ 164 ] ; xx [ 17 ] = xx [ 604 ]
; xx [ 18 ] = xx [ 605 ] - xx [ 162 ] * xx [ 11 ] ; xx [ 19 ] = xx [ 600 ] -
xx [ 168 ] * xx [ 11 ] ; pm_math_Quaternion_xform_ra ( xx + 1243 , xx + 17 ,
xx + 82 ) ; xx [ 17 ] = ( xx [ 81 ] * xx [ 208 ] - xx [ 161 ] ) / xx [ 163 ]
; xx [ 85 ] = xx [ 612 ] ; xx [ 86 ] = xx [ 613 ] + xx [ 200 ] * xx [ 17 ] ;
xx [ 87 ] = xx [ 609 ] + xx [ 168 ] * xx [ 17 ] ; pm_math_Quaternion_xform_ra
( xx + 384 , xx + 85 , xx + 92 ) ; xx [ 18 ] = xx [ 217 ] * xx [ 329 ] + xx [
1061 ] * xx [ 220 ] + xx [ 1081 ] * xx [ 221 ] - xx [ 258 ] ; xx [ 19 ] = - (
xx [ 642 ] + xx [ 330 ] * xx [ 217 ] ) ; solveSymmetricPosDef ( xx + 312 , xx
+ 18 , 2 , 1 , xx + 47 , xx + 56 ) ; xx [ 85 ] = xx [ 20 ] + xx [ 284 ] * xx
[ 47 ] + xx [ 320 ] * xx [ 48 ] ; xx [ 86 ] = xx [ 426 ] + xx [ 321 ] * xx [
47 ] - xx [ 322 ] * xx [ 48 ] ; xx [ 87 ] = xx [ 639 ] - xx [ 323 ] * xx [ 47
] - xx [ 324 ] * xx [ 48 ] ; pm_math_Quaternion_xform_ra ( xx + 299 , xx + 85
, xx + 18 ) ; xx [ 85 ] = xx [ 733 ] + xx [ 18 ] + xx [ 349 ] ; xx [ 86 ] =
xx [ 734 ] + xx [ 19 ] + xx [ 350 ] ; xx [ 87 ] = xx [ 735 ] + xx [ 20 ] + xx
[ 351 ] ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 85 , xx + 111 ) ; xx
[ 56 ] = xx [ 761 ] * xx [ 214 ] + xx [ 215 ] * xx [ 59 ] + xx [ 1110 ] * xx
[ 223 ] - xx [ 334 ] ; xx [ 57 ] = - ( xx [ 707 ] + xx [ 214 ] * xx [ 760 ] +
xx [ 181 ] * xx [ 215 ] ) ; solveSymmetricPosDef ( xx + 393 , xx + 56 , 2 , 1
, xx + 85 , xx + 87 ) ; xx [ 115 ] = xx [ 884 ] - xx [ 398 ] * xx [ 85 ] + xx
[ 364 ] * xx [ 86 ] ; xx [ 116 ] = xx [ 633 ] + xx [ 399 ] * xx [ 85 ] + xx [
400 ] * xx [ 86 ] ; xx [ 117 ] = xx [ 643 ] - xx [ 401 ] * xx [ 85 ] + xx [
402 ] * xx [ 86 ] ; pm_math_Quaternion_xform_ra ( xx + 380 , xx + 115 , xx +
118 ) ; xx [ 24 ] = xx [ 111 ] + xx [ 118 ] + xx [ 913 ] ; xx [ 50 ] = xx [
112 ] + xx [ 119 ] + xx [ 914 ] ; xx [ 53 ] = ( xx [ 97 ] * xx [ 209 ] + xx [
198 ] * xx [ 215 ] + xx [ 24 ] * xx [ 413 ] + xx [ 50 ] * xx [ 427 ] ) / xx [
432 ] ; xx [ 115 ] = xx [ 24 ] - xx [ 428 ] * xx [ 53 ] ; xx [ 116 ] = xx [
50 ] - xx [ 431 ] * xx [ 53 ] ; xx [ 117 ] = xx [ 113 ] + xx [ 120 ] + xx [
915 ] - xx [ 438 ] * xx [ 53 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx
+ 115 , xx + 145 ) ; xx [ 56 ] = xx [ 216 ] * xx [ 578 ] + xx [ 958 ] * xx [
218 ] + xx [ 1006 ] * xx [ 219 ] - xx [ 347 ] ; xx [ 57 ] = - ( xx [ 348 ] +
xx [ 580 ] * xx [ 216 ] ) ; solveSymmetricPosDef ( xx + 560 , xx + 56 , 2 , 1
, xx + 87 , xx + 115 ) ; xx [ 115 ] = xx [ 452 ] + xx [ 439 ] * xx [ 87 ] +
xx [ 564 ] * xx [ 88 ] ; xx [ 116 ] = xx [ 520 ] + xx [ 565 ] * xx [ 87 ] -
xx [ 566 ] * xx [ 88 ] ; xx [ 117 ] = xx [ 346 ] - xx [ 567 ] * xx [ 87 ] -
xx [ 568 ] * xx [ 88 ] ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 115 ,
xx + 148 ) ; xx [ 115 ] = xx [ 373 ] + xx [ 148 ] + xx [ 420 ] ; xx [ 116 ] =
xx [ 374 ] + xx [ 149 ] + xx [ 539 ] ; xx [ 117 ] = xx [ 375 ] + xx [ 150 ] +
xx [ 536 ] ; pm_math_Quaternion_xform_ra ( xx + 489 , xx + 115 , xx + 155 ) ;
xx [ 24 ] = xx [ 155 ] + xx [ 268 ] ; xx [ 115 ] = xx [ 627 ] + xx [ 296 ] *
xx [ 87 ] ; xx [ 116 ] = xx [ 628 ] - xx [ 532 ] * xx [ 87 ] ; xx [ 117 ] =
xx [ 637 ] - xx [ 535 ] * xx [ 87 ] ; pm_math_Quaternion_xform_ra ( xx + 545
, xx + 115 , xx + 160 ) ; pm_math_Vector3_cross_ra ( xx + 624 , xx + 148 , xx
+ 115 ) ; xx [ 148 ] = xx [ 440 ] + xx [ 160 ] + xx [ 115 ] + xx [ 275 ] ; xx
[ 149 ] = xx [ 276 ] + xx [ 161 ] + xx [ 116 ] + xx [ 342 ] ; xx [ 150 ] = xx
[ 418 ] + xx [ 162 ] + xx [ 117 ] + xx [ 419 ] ; pm_math_Quaternion_xform_ra
( xx + 489 , xx + 148 , xx + 115 ) ; pm_math_Vector3_cross_ra ( xx + 648 , xx
+ 155 , xx + 148 ) ; xx [ 50 ] = xx [ 343 ] + xx [ 115 ] + xx [ 148 ] + xx [
537 ] ; xx [ 56 ] = xx [ 344 ] + xx [ 116 ] + xx [ 149 ] + xx [ 543 ] ; xx [
57 ] = xx [ 345 ] + xx [ 117 ] + xx [ 150 ] + xx [ 540 ] ; xx [ 115 ] = xx [
50 ] ; xx [ 116 ] = xx [ 56 ] ; xx [ 117 ] = xx [ 57 ] ; xx [ 59 ] = xx [ 156
] + xx [ 270 ] ; xx [ 62 ] = xx [ 157 ] + xx [ 587 ] ; xx [ 148 ] = xx [ 24 ]
; xx [ 149 ] = xx [ 59 ] ; xx [ 150 ] = xx [ 62 ] ; xx [ 129 ] = xx [ 208 ] *
xx [ 63 ] + xx [ 167 ] * xx [ 210 ] + xx [ 444 ] * xx [ 211 ] + xx [ 576 ] *
xx [ 212 ] + xx [ 939 ] * xx [ 218 ] + xx [ 1042 ] * xx [ 219 ] + xx [ 1105 ]
* xx [ 222 ] - ( pm_math_Vector3_dot_ra ( xx + 684 , xx + 115 ) +
pm_math_Vector3_dot_ra ( xx + 700 , xx + 148 ) ) ; xx [ 130 ] = xx [ 577 ] *
xx [ 212 ] - xx [ 64 ] * xx [ 208 ] - pm_math_Vector3_dot_ra ( xx + 684 , xx
+ 148 ) ; solveSymmetricPosDef ( xx + 664 , xx + 129 , 2 , 1 , xx + 63 , xx +
115 ) ; xx [ 115 ] = xx [ 24 ] + xx [ 706 ] * xx [ 63 ] + xx [ 660 ] * xx [
64 ] ; xx [ 116 ] = xx [ 59 ] + xx [ 523 ] * xx [ 63 ] + xx [ 661 ] * xx [ 64
] ; xx [ 117 ] = xx [ 62 ] + xx [ 524 ] * xx [ 63 ] + xx [ 64 ] * xx [ 662 ]
; pm_math_Quaternion_xform_ra ( xx + 464 , xx + 115 , xx + 148 ) ; xx [ 115 ]
= xx [ 0 ] + xx [ 39 ] + xx [ 82 ] + xx [ 92 ] + xx [ 145 ] + xx [ 148 ] + xx
[ 1025 ] ; xx [ 116 ] = xx [ 437 ] + xx [ 40 ] + xx [ 83 ] + xx [ 93 ] + xx [
146 ] + xx [ 149 ] + xx [ 1026 ] ; xx [ 117 ] = xx [ 441 ] + xx [ 41 ] + xx [
84 ] + xx [ 94 ] + xx [ 147 ] + xx [ 150 ] + xx [ 1027 ] ;
pm_math_Vector3_cross_ra ( xx + 140 , xx + 39 , xx + 155 ) ;
pm_math_Vector3_cross_ra ( xx + 449 , xx + 82 , xx + 39 ) ;
pm_math_Vector3_cross_ra ( xx + 204 , xx + 92 , xx + 81 ) ; xx [ 92 ] = xx [
202 ] + xx [ 316 ] * xx [ 47 ] ; xx [ 93 ] = xx [ 232 ] - xx [ 318 ] * xx [
47 ] ; xx [ 94 ] = xx [ 328 ] - xx [ 319 ] * xx [ 47 ] ;
pm_math_Quaternion_xform_ra ( xx + 299 , xx + 92 , xx + 160 ) ;
pm_math_Vector3_cross_ra ( xx + 736 , xx + 18 , xx + 92 ) ; xx [ 18 ] = xx [
38 ] + xx [ 160 ] + xx [ 92 ] + xx [ 42 ] ; xx [ 19 ] = xx [ 44 ] + xx [ 161
] + xx [ 93 ] + xx [ 60 ] ; xx [ 20 ] = xx [ 65 ] + xx [ 162 ] + xx [ 94 ] +
xx [ 67 ] ; pm_math_Quaternion_xform_ra ( xx + 236 , xx + 18 , xx + 92 ) ;
pm_math_Vector3_cross_ra ( xx + 784 , xx + 111 , xx + 18 ) ; xx [ 111 ] = xx
[ 352 ] + xx [ 376 ] * xx [ 85 ] ; xx [ 112 ] = xx [ 353 ] + xx [ 379 ] * xx
[ 85 ] ; xx [ 113 ] = xx [ 354 ] + xx [ 397 ] * xx [ 85 ] ;
pm_math_Quaternion_xform_ra ( xx + 380 , xx + 111 , xx + 160 ) ;
pm_math_Vector3_cross_ra ( xx + 813 , xx + 118 , xx + 111 ) ; xx [ 118 ] = xx
[ 751 ] + xx [ 92 ] + xx [ 18 ] + xx [ 160 ] + xx [ 111 ] + xx [ 355 ] - xx [
405 ] * xx [ 53 ] ; xx [ 119 ] = xx [ 752 ] + xx [ 93 ] + xx [ 19 ] + xx [
161 ] + xx [ 112 ] + xx [ 356 ] - xx [ 409 ] * xx [ 53 ] ; xx [ 120 ] = xx [
753 ] + xx [ 94 ] + xx [ 20 ] + xx [ 162 ] + xx [ 113 ] + xx [ 357 ] - xx [
825 ] * xx [ 53 ] ; pm_math_Quaternion_xform_ra ( xx + 388 , xx + 118 , xx +
18 ) ; pm_math_Vector3_cross_ra ( xx + 845 , xx + 145 , xx + 92 ) ; xx [ 111
] = xx [ 50 ] + xx [ 525 ] * xx [ 63 ] + xx [ 657 ] * xx [ 64 ] ; xx [ 112 ]
= xx [ 56 ] + xx [ 526 ] * xx [ 63 ] + xx [ 658 ] * xx [ 64 ] ; xx [ 113 ] =
xx [ 57 ] + xx [ 519 ] * xx [ 63 ] + xx [ 64 ] * xx [ 659 ] ;
pm_math_Quaternion_xform_ra ( xx + 464 , xx + 111 , xx + 118 ) ;
pm_math_Vector3_cross_ra ( xx + 651 , xx + 148 , xx + 111 ) ; xx [ 145 ] = xx
[ 13 ] + xx [ 132 ] + xx [ 155 ] + xx [ 152 ] + xx [ 39 ] + xx [ 601 ] + xx [
81 ] + xx [ 18 ] + xx [ 92 ] + xx [ 118 ] + xx [ 111 ] + xx [ 406 ] ; xx [
146 ] = xx [ 68 ] + xx [ 133 ] + xx [ 156 ] + xx [ 153 ] + xx [ 40 ] + xx [
602 ] + xx [ 82 ] + xx [ 19 ] + xx [ 93 ] + xx [ 119 ] + xx [ 112 ] + xx [
407 ] ; xx [ 147 ] = xx [ 46 ] + xx [ 134 ] + xx [ 157 ] + xx [ 154 ] + xx [
41 ] + xx [ 603 ] + xx [ 83 ] + xx [ 20 ] + xx [ 94 ] + xx [ 120 ] + xx [ 113
] + xx [ 408 ] ; xx [ 148 ] = - pm_math_Vector3_dot_ra ( xx + 26 , xx + 115 )
; xx [ 149 ] = - pm_math_Vector3_dot_ra ( xx + 672 , xx + 115 ) ; xx [ 150 ]
= - pm_math_Vector3_dot_ra ( xx + 703 , xx + 115 ) ; xx [ 151 ] = - (
pm_math_Vector3_dot_ra ( xx + 935 , xx + 145 ) + pm_math_Vector3_dot_ra ( xx
+ 944 , xx + 115 ) ) ; xx [ 152 ] = - ( pm_math_Vector3_dot_ra ( xx + 954 ,
xx + 145 ) + pm_math_Vector3_dot_ra ( xx + 963 , xx + 115 ) ) ; xx [ 153 ] =
- ( pm_math_Vector3_dot_ra ( xx + 973 , xx + 145 ) + pm_math_Vector3_dot_ra (
xx + 982 , xx + 115 ) ) ; solveSymmetricPosDef ( xx + 848 , xx + 148 , 6 , 1
, xx + 115 , xx + 166 ) ; xx [ 0 ] = xx [ 115 ] - xx [ 61 ] ; xx [ 13 ] = xx
[ 116 ] - xx [ 91 ] ; xx [ 18 ] = xx [ 117 ] - xx [ 89 ] ; xx [ 19 ] = xx [
118 ] - xx [ 54 ] ; xx [ 20 ] = xx [ 119 ] - xx [ 49 ] ; xx [ 24 ] = xx [ 120
] - xx [ 45 ] ; xx [ 26 ] = xx [ 959 ] * xx [ 24 ] - ( xx [ 671 ] * xx [ 19 ]
+ xx [ 940 ] * xx [ 20 ] ) ; xx [ 27 ] = xx [ 933 ] * xx [ 19 ] - xx [ 952 ]
* xx [ 20 ] + xx [ 971 ] * xx [ 24 ] ; xx [ 28 ] = xx [ 953 ] * xx [ 20 ] -
xx [ 934 ] * xx [ 19 ] + xx [ 972 ] * xx [ 24 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx + 26 , xx + 38 ) ; xx [ 41
] = xx [ 51 ] + xx [ 3 ] * xx [ 0 ] + xx [ 196 ] * xx [ 13 ] + xx [ 522 ] *
xx [ 18 ] + xx [ 941 ] * xx [ 19 ] - xx [ 960 ] * xx [ 20 ] + xx [ 979 ] * xx
[ 24 ] + xx [ 269 ] ; pm_math_Vector3_cross_ra ( xx + 26 , xx + 651 , xx + 44
) ; xx [ 3 ] = xx [ 52 ] + xx [ 22 ] * xx [ 0 ] + xx [ 21 ] * xx [ 13 ] + xx
[ 23 ] * xx [ 18 ] + xx [ 942 ] * xx [ 19 ] + xx [ 961 ] * xx [ 20 ] - xx [
980 ] * xx [ 24 ] + xx [ 412 ] ; xx [ 21 ] = xx [ 25 ] * xx [ 0 ] - xx [ 16 ]
+ xx [ 434 ] * xx [ 13 ] + xx [ 1 ] * xx [ 18 ] - xx [ 943 ] * xx [ 19 ] - xx
[ 962 ] * xx [ 20 ] - xx [ 981 ] * xx [ 24 ] + xx [ 43 ] + xx [ 429 ] ; xx [
49 ] = xx [ 41 ] + xx [ 44 ] ; xx [ 50 ] = xx [ 3 ] + xx [ 45 ] ; xx [ 51 ] =
xx [ 21 ] + xx [ 46 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 464 , xx +
49 , xx + 42 ) ; xx [ 1 ] = xx [ 63 ] - ( pm_math_Vector3_dot_ra ( xx + 172 ,
xx + 38 ) + pm_math_Vector3_dot_ra ( xx + 271 , xx + 42 ) ) ; xx [ 16 ] = xx
[ 64 ] - ( pm_math_Vector3_dot_ra ( xx + 367 , xx + 38 ) +
pm_math_Vector3_dot_ra ( xx + 370 , xx + 42 ) ) ; xx [ 49 ] = xx [ 38 ] + xx
[ 442 ] * xx [ 1 ] + xx [ 596 ] ; xx [ 50 ] = xx [ 39 ] + xx [ 458 ] * xx [ 1
] + xx [ 597 ] ; xx [ 51 ] = xx [ 40 ] + xx [ 459 ] * xx [ 1 ] + xx [ 598 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 49 , xx + 38 ) ; xx [ 22
] = xx [ 38 ] + xx [ 617 ] ; xx [ 23 ] = xx [ 39 ] + xx [ 618 ] ; xx [ 25 ] =
xx [ 40 ] + xx [ 638 ] ; xx [ 38 ] = xx [ 22 ] ; xx [ 39 ] = xx [ 23 ] ; xx [
40 ] = xx [ 25 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 38 ,
xx + 59 ) ; pm_math_Vector3_cross_ra ( xx + 49 , xx + 648 , xx + 62 ) ; xx [
67 ] = xx [ 42 ] - xx [ 655 ] * xx [ 1 ] + xx [ 442 ] * xx [ 16 ] + xx [ 55 ]
+ xx [ 62 ] ; xx [ 68 ] = xx [ 43 ] + xx [ 656 ] * xx [ 1 ] + xx [ 458 ] * xx
[ 16 ] + xx [ 96 ] + xx [ 63 ] ; xx [ 69 ] = xx [ 44 ] - xx [ 699 ] * xx [ 1
] + xx [ 459 ] * xx [ 16 ] + xx [ 106 ] + xx [ 64 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 489 , xx + 67 , xx + 42 ) ;
pm_math_Vector3_cross_ra ( xx + 38 , xx + 624 , xx + 54 ) ; xx [ 62 ] = xx [
42 ] + xx [ 340 ] + xx [ 54 ] ; xx [ 63 ] = xx [ 43 ] + xx [ 341 ] + xx [ 55
] ; xx [ 64 ] = xx [ 44 ] + xx [ 274 ] + xx [ 56 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 545 , xx + 62 , xx + 42 ) ; xx [ 45
] = xx [ 87 ] - ( pm_math_Vector3_dot_ra ( xx + 830 , xx + 59 ) +
pm_math_Vector3_dot_ra ( xx + 1030 , xx + 42 ) ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 26 , xx + 54 ) ;
pm_math_Vector3_cross_ra ( xx + 26 , xx + 845 , xx + 62 ) ; xx [ 67 ] = xx [
41 ] + xx [ 62 ] ; xx [ 68 ] = xx [ 3 ] + xx [ 63 ] ; xx [ 69 ] = xx [ 21 ] +
xx [ 64 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 388 , xx + 67 , xx + 62
) ; xx [ 46 ] = xx [ 53 ] + pm_math_Vector3_dot_ra ( xx + 1139 , xx + 54 ) +
pm_math_Vector3_dot_ra ( xx + 1147 , xx + 62 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 380 , xx + 54 , xx + 67 ) ; xx [ 52
] = xx [ 62 ] - xx [ 413 ] * xx [ 46 ] + xx [ 414 ] ;
pm_math_Vector3_cross_ra ( xx + 54 , xx + 813 , xx + 81 ) ; xx [ 53 ] = xx [
63 ] - xx [ 427 ] * xx [ 46 ] + xx [ 430 ] ; xx [ 57 ] = xx [ 64 ] + xx [ 435
] ; xx [ 62 ] = xx [ 52 ] + xx [ 81 ] ; xx [ 63 ] = xx [ 53 ] + xx [ 82 ] ;
xx [ 64 ] = xx [ 57 ] + xx [ 83 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
380 , xx + 62 , xx + 81 ) ; xx [ 62 ] = xx [ 85 ] - ( pm_math_Vector3_dot_ra
( xx + 775 , xx + 67 ) + pm_math_Vector3_dot_ra ( xx + 1240 , xx + 81 ) ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 236 , xx + 54 , xx + 63 ) ; xx [ 89
] = xx [ 63 ] + xx [ 644 ] ; xx [ 90 ] = xx [ 64 ] + xx [ 741 ] ; xx [ 91 ] =
xx [ 65 ] + xx [ 747 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx +
89 , xx + 63 ) ; pm_math_Vector3_cross_ra ( xx + 54 , xx + 784 , xx + 92 ) ;
xx [ 95 ] = xx [ 52 ] + xx [ 92 ] ; xx [ 96 ] = xx [ 53 ] + xx [ 93 ] ; xx [
97 ] = xx [ 57 ] + xx [ 94 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 236
, xx + 95 , xx + 92 ) ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 736 , xx +
95 ) ; xx [ 111 ] = xx [ 92 ] + xx [ 765 ] + xx [ 95 ] ; xx [ 112 ] = xx [ 93
] + xx [ 795 ] + xx [ 96 ] ; xx [ 113 ] = xx [ 94 ] + xx [ 802 ] + xx [ 97 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 299 , xx + 111 , xx + 92 ) ; xx [
52 ] = xx [ 47 ] - ( pm_math_Vector3_dot_ra ( xx + 584 , xx + 63 ) +
pm_math_Vector3_dot_ra ( xx + 1003 , xx + 92 ) ) ; pm_math_Vector3_cross_ra (
xx + 26 , xx + 204 , xx + 95 ) ; xx [ 111 ] = xx [ 41 ] + xx [ 95 ] ; xx [
112 ] = xx [ 3 ] + xx [ 96 ] ; xx [ 113 ] = xx [ 21 ] + xx [ 97 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 111 , xx + 95 ) ;
pm_math_Vector3_cross_ra ( xx + 26 , xx + 449 , xx + 111 ) ; xx [ 115 ] = xx
[ 41 ] + xx [ 111 ] ; xx [ 116 ] = xx [ 3 ] + xx [ 112 ] ; xx [ 117 ] = xx [
21 ] + xx [ 113 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 1243 , xx + 115
, xx + 111 ) ; pm_math_Vector3_cross_ra ( xx + 26 , xx + 140 , xx + 115 ) ;
xx [ 118 ] = xx [ 41 ] + xx [ 115 ] ; xx [ 119 ] = xx [ 3 ] + xx [ 116 ] ; xx
[ 120 ] = xx [ 21 ] + xx [ 117 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
125 , xx + 118 , xx + 115 ) ; pm_math_Vector3_cross_ra ( xx + 26 , xx + 136 ,
xx + 118 ) ; xx [ 132 ] = xx [ 41 ] + xx [ 118 ] ; xx [ 133 ] = xx [ 3 ] + xx
[ 119 ] ; xx [ 134 ] = xx [ 21 ] + xx [ 120 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx + 132 , xx + 118 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 98 , xx + 26 , xx + 132 ) ;
pm_math_Vector3_cross_ra ( xx + 26 , xx + 103 , xx + 98 ) ; xx [ 101 ] = xx [
41 ] + xx [ 98 ] ; xx [ 102 ] = xx [ 3 ] + xx [ 99 ] ; xx [ 103 ] = xx [ 21 ]
+ xx [ 100 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 101 , xx
+ 98 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 26 , xx + 101 )
; pm_math_Vector3_cross_ra ( xx + 26 , xx + 71 , xx + 102 ) ; xx [ 71 ] = xx
[ 41 ] + xx [ 102 ] ; xx [ 72 ] = xx [ 3 ] + xx [ 103 ] ; xx [ 73 ] = xx [ 21
] + xx [ 104 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 107 , xx + 71 , xx
+ 102 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 107 , xx + 26 , xx + 71 )
; pm_math_Quaternion_inverseXform_ra ( xx + 75 , xx + 26 , xx + 105 ) ;
pm_math_Vector3_cross_ra ( xx + 26 , xx + 445 , xx + 106 ) ; xx [ 109 ] = xx
[ 41 ] + xx [ 106 ] ; xx [ 110 ] = xx [ 3 ] + xx [ 107 ] ; xx [ 111 ] = xx [
21 ] + xx [ 108 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 75 , xx + 109 ,
xx + 106 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 384 , xx + 26 , xx +
75 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 464 , xx + 384 , xx + 121
) ; pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 49 , xx + 109 ) ; xx
[ 121 ] = xx [ 66 ] ; xx [ 122 ] = xx [ 70 ] ; xx [ 123 ] = xx [ 74 ] ; xx [
124 ] = xx [ 79 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 54 ,
xx + 49 ) ; xx [ 53 ] = xx [ 26 ] - xx [ 49 ] ; xx [ 54 ] = xx [ 27 ] - xx [
50 ] ; xx [ 55 ] = xx [ 28 ] - xx [ 51 ] ; xx [ 49 ] = - 0.7077022283680725 ;
xx [ 50 ] = - 9.268355963184496e-3 ; xx [ 51 ] = xx [ 144 ] ; xx [ 72 ] = -
xx [ 629 ] ; xx [ 73 ] = - xx [ 630 ] ; xx [ 74 ] = - xx [ 631 ] ; xx [ 75 ]
= - xx [ 632 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 72 , xx + 26 , xx
+ 121 ) ; xx [ 72 ] = xx [ 22 ] - xx [ 121 ] ; xx [ 73 ] = xx [ 23 ] - xx [
122 ] ; xx [ 74 ] = xx [ 25 ] - xx [ 123 ] ; xx [ 21 ] = xx [ 460 ] ; xx [ 22
] = xx [ 159 ] ; xx [ 23 ] = - 0.0735748245777017 ; xx [ 121 ] = xx [ 4 ] ;
xx [ 122 ] = xx [ 5 ] ; xx [ 123 ] = xx [ 35 ] ; xx [ 124 ] = xx [ 36 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 125 , xx + 26 , xx + 3 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 121 , xx + 3 , xx + 125 ) ; xx [ 3
] = xx [ 26 ] - xx [ 125 ] ; xx [ 4 ] = xx [ 27 ] - xx [ 126 ] ; xx [ 5 ] =
xx [ 28 ] - xx [ 127 ] ; xx [ 121 ] = - 0.7077023583577413 ; xx [ 122 ] = -
9.268340058974256e-3 ; xx [ 123 ] = 0.7064499061132662 ; xx [ 25 ] = xx [ 67
] + xx [ 335 ] * xx [ 62 ] + xx [ 985 ] ;
pm_math_Quaternion_inverseCompose_ra ( xx + 236 , xx + 380 , xx + 124 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 124 , xx + 89 , xx + 128 ) ; xx [
35 ] = xx [ 68 ] + xx [ 362 ] * xx [ 62 ] + xx [ 986 ] ; xx [ 36 ] = xx [ 69
] + xx [ 363 ] * xx [ 62 ] + xx [ 987 ] ; xx [ 124 ] = xx [ 25 ] - xx [ 128 ]
; xx [ 125 ] = xx [ 35 ] - xx [ 129 ] ; xx [ 126 ] = xx [ 36 ] - xx [ 130 ] ;
xx [ 127 ] = - 3.734971681712052e-4 ; xx [ 128 ] = - 0.2610715860287324 ; xx
[ 129 ] = - 0.965319370710185 ; pm_math_Quaternion_inverseXform_ra ( xx +
1243 , xx + 26 , xx + 133 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 804
, xx + 1243 , xx + 136 ) ; xx [ 26 ] = xx [ 25 ] ; xx [ 27 ] = xx [ 35 ] ; xx
[ 28 ] = xx [ 36 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 136 , xx + 26
, xx + 140 ) ; xx [ 25 ] = xx [ 133 ] - xx [ 140 ] ; xx [ 26 ] = xx [ 134 ] -
xx [ 141 ] ; xx [ 27 ] = xx [ 135 ] - xx [ 142 ] ; xx [ 133 ] =
1.845352212151208e-7 ; xx [ 134 ] = 3.974634836310293e-3 ; xx [ 135 ] =
0.9999921011077457 ; pm_math_Quaternion_inverseXform_ra ( xx + 512 , xx + 38
, xx + 136 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 545 , xx + 512 ,
xx + 38 ) ; xx [ 139 ] = xx [ 59 ] + xx [ 30 ] * xx [ 45 ] + xx [ 634 ] ; xx
[ 140 ] = xx [ 60 ] - xx [ 511 ] * xx [ 45 ] + xx [ 635 ] ; xx [ 141 ] = xx [
61 ] - xx [ 531 ] * xx [ 45 ] + xx [ 636 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 38 , xx + 139 , xx + 142 ) ; xx [
38 ] = xx [ 136 ] + xx [ 424 ] - xx [ 142 ] ; xx [ 39 ] = xx [ 137 ] + xx [
433 ] - xx [ 143 ] ; xx [ 40 ] = xx [ 138 ] + xx [ 518 ] - xx [ 144 ] ; xx [
136 ] = - 1.380846652021539e-7 ; xx [ 137 ] = 0.9999999999981868 ; xx [ 138 ]
= 1.899414133421651e-6 ; pm_math_Quaternion_inverseXform_ra ( xx + 262 , xx +
89 , xx + 139 ) ; pm_math_Quaternion_inverseCompose_ra ( xx + 299 , xx + 262
, xx + 142 ) ; xx [ 89 ] = xx [ 63 ] + xx [ 261 ] * xx [ 52 ] + xx [ 822 ] ;
xx [ 90 ] = xx [ 64 ] - xx [ 282 ] * xx [ 52 ] + xx [ 823 ] ; xx [ 91 ] = xx
[ 65 ] - xx [ 283 ] * xx [ 52 ] + xx [ 824 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 142 , xx + 89 , xx + 146 ) ; xx [
89 ] = xx [ 139 ] + xx [ 14 ] - xx [ 146 ] ; xx [ 90 ] = xx [ 140 ] + xx [ 34
] - xx [ 147 ] ; xx [ 91 ] = xx [ 141 ] + xx [ 37 ] - xx [ 148 ] ; xx [ 34 ]
= 1.380846645915312e-7 ; xx [ 35 ] = - xx [ 256 ] ; xx [ 36 ] = -
1.899414134864941e-6 ; deriv [ 0 ] = state [ 7 ] ; deriv [ 1 ] = state [ 8 ]
; deriv [ 2 ] = state [ 9 ] ; deriv [ 3 ] = xx [ 7 ] ; deriv [ 4 ] = xx [ 8 ]
; deriv [ 5 ] = xx [ 9 ] ; deriv [ 6 ] = xx [ 10 ] ; deriv [ 7 ] = xx [ 0 ] ;
deriv [ 8 ] = xx [ 13 ] ; deriv [ 9 ] = xx [ 18 ] ; deriv [ 10 ] = xx [ 19 ]
; deriv [ 11 ] = xx [ 20 ] ; deriv [ 12 ] = xx [ 24 ] ; deriv [ 13 ] = state
[ 15 ] ; deriv [ 14 ] = state [ 16 ] ; deriv [ 15 ] = xx [ 1 ] ; deriv [ 16 ]
= xx [ 16 ] ; deriv [ 17 ] = state [ 19 ] ; deriv [ 18 ] = state [ 20 ] ;
deriv [ 19 ] = xx [ 45 ] ; deriv [ 20 ] = xx [ 88 ] - (
pm_math_Vector3_dot_ra ( xx + 905 , xx + 59 ) + pm_math_Vector3_dot_ra ( xx +
581 , xx + 42 ) ) ; deriv [ 21 ] = state [ 22 ] ; deriv [ 22 ] = - xx [ 46 ]
; deriv [ 23 ] = state [ 25 ] ; deriv [ 24 ] = state [ 26 ] ; deriv [ 25 ] =
xx [ 62 ] ; deriv [ 26 ] = xx [ 86 ] - ( pm_math_Vector3_dot_ra ( xx + 191 ,
xx + 67 ) + pm_math_Vector3_dot_ra ( xx + 415 , xx + 81 ) ) ; deriv [ 27 ] =
state [ 29 ] ; deriv [ 28 ] = state [ 30 ] ; deriv [ 29 ] = xx [ 52 ] ; deriv
[ 30 ] = xx [ 48 ] - ( pm_math_Vector3_dot_ra ( xx + 910 , xx + 63 ) +
pm_math_Vector3_dot_ra ( xx + 337 , xx + 92 ) ) ; deriv [ 31 ] = state [ 32 ]
; deriv [ 32 ] = xx [ 17 ] - ( xx [ 201 ] * xx [ 96 ] + xx [ 207 ] * xx [ 97
] ) ; deriv [ 33 ] = state [ 34 ] ; deriv [ 34 ] = xx [ 11 ] + xx [ 165 ] *
xx [ 112 ] + xx [ 195 ] * xx [ 113 ] ; deriv [ 35 ] = state [ 36 ] ; deriv [
36 ] = - ( xx [ 2 ] + pm_math_Vector3_dot_ra ( xx + 1231 , xx + 115 ) ) ;
deriv [ 37 ] = state [ 38 ] ; deriv [ 38 ] = - ( xx [ 31 ] + xx [ 131 ] * xx
[ 120 ] - xx [ 33 ] * xx [ 132 ] ) ; deriv [ 39 ] = state [ 40 ] ; deriv [ 40
] = - ( xx [ 15 ] + xx [ 114 ] * xx [ 100 ] - xx [ 29 ] * xx [ 101 ] ) ;
deriv [ 41 ] = state [ 42 ] ; deriv [ 42 ] = - ( xx [ 12 ] + xx [ 80 ] * xx [
104 ] - xx [ 32 ] * xx [ 71 ] ) ; deriv [ 43 ] = state [ 44 ] ; deriv [ 44 ]
= - ( xx [ 6 ] + xx [ 996 ] * xx [ 105 ] - xx [ 58 ] * xx [ 108 ] ) ; deriv [
45 ] = state [ 46 ] ; deriv [ 46 ] = 3.974634836311581e-3 * ( xx [ 76 ] - xx
[ 110 ] ) + xx [ 599 ] * ( xx [ 77 ] - xx [ 111 ] ) ; deriv [ 47 ] = state [
48 ] ; deriv [ 48 ] = pm_math_Vector3_dot_ra ( xx + 53 , xx + 49 ) ; deriv [
49 ] = state [ 50 ] ; deriv [ 50 ] = pm_math_Vector3_dot_ra ( xx + 72 , xx +
21 ) ; deriv [ 51 ] = state [ 52 ] ; deriv [ 52 ] = pm_math_Vector3_dot_ra (
xx + 3 , xx + 121 ) ; deriv [ 53 ] = state [ 54 ] ; deriv [ 54 ] =
pm_math_Vector3_dot_ra ( xx + 124 , xx + 127 ) ; deriv [ 55 ] = state [ 56 ]
; deriv [ 56 ] = pm_math_Vector3_dot_ra ( xx + 25 , xx + 133 ) ; deriv [ 57 ]
= state [ 58 ] ; deriv [ 58 ] = pm_math_Vector3_dot_ra ( xx + 38 , xx + 136 )
; deriv [ 59 ] = state [ 60 ] ; deriv [ 60 ] = pm_math_Vector3_dot_ra ( xx +
89 , xx + 34 ) ; errorResult [ 0 ] = xx [ 317 ] ; return NULL ; }
