#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6894033735385027323) {
   out_6894033735385027323[0] = delta_x[0] + nom_x[0];
   out_6894033735385027323[1] = delta_x[1] + nom_x[1];
   out_6894033735385027323[2] = delta_x[2] + nom_x[2];
   out_6894033735385027323[3] = delta_x[3] + nom_x[3];
   out_6894033735385027323[4] = delta_x[4] + nom_x[4];
   out_6894033735385027323[5] = delta_x[5] + nom_x[5];
   out_6894033735385027323[6] = delta_x[6] + nom_x[6];
   out_6894033735385027323[7] = delta_x[7] + nom_x[7];
   out_6894033735385027323[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3944300214870265721) {
   out_3944300214870265721[0] = -nom_x[0] + true_x[0];
   out_3944300214870265721[1] = -nom_x[1] + true_x[1];
   out_3944300214870265721[2] = -nom_x[2] + true_x[2];
   out_3944300214870265721[3] = -nom_x[3] + true_x[3];
   out_3944300214870265721[4] = -nom_x[4] + true_x[4];
   out_3944300214870265721[5] = -nom_x[5] + true_x[5];
   out_3944300214870265721[6] = -nom_x[6] + true_x[6];
   out_3944300214870265721[7] = -nom_x[7] + true_x[7];
   out_3944300214870265721[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7899367608552472341) {
   out_7899367608552472341[0] = 1.0;
   out_7899367608552472341[1] = 0;
   out_7899367608552472341[2] = 0;
   out_7899367608552472341[3] = 0;
   out_7899367608552472341[4] = 0;
   out_7899367608552472341[5] = 0;
   out_7899367608552472341[6] = 0;
   out_7899367608552472341[7] = 0;
   out_7899367608552472341[8] = 0;
   out_7899367608552472341[9] = 0;
   out_7899367608552472341[10] = 1.0;
   out_7899367608552472341[11] = 0;
   out_7899367608552472341[12] = 0;
   out_7899367608552472341[13] = 0;
   out_7899367608552472341[14] = 0;
   out_7899367608552472341[15] = 0;
   out_7899367608552472341[16] = 0;
   out_7899367608552472341[17] = 0;
   out_7899367608552472341[18] = 0;
   out_7899367608552472341[19] = 0;
   out_7899367608552472341[20] = 1.0;
   out_7899367608552472341[21] = 0;
   out_7899367608552472341[22] = 0;
   out_7899367608552472341[23] = 0;
   out_7899367608552472341[24] = 0;
   out_7899367608552472341[25] = 0;
   out_7899367608552472341[26] = 0;
   out_7899367608552472341[27] = 0;
   out_7899367608552472341[28] = 0;
   out_7899367608552472341[29] = 0;
   out_7899367608552472341[30] = 1.0;
   out_7899367608552472341[31] = 0;
   out_7899367608552472341[32] = 0;
   out_7899367608552472341[33] = 0;
   out_7899367608552472341[34] = 0;
   out_7899367608552472341[35] = 0;
   out_7899367608552472341[36] = 0;
   out_7899367608552472341[37] = 0;
   out_7899367608552472341[38] = 0;
   out_7899367608552472341[39] = 0;
   out_7899367608552472341[40] = 1.0;
   out_7899367608552472341[41] = 0;
   out_7899367608552472341[42] = 0;
   out_7899367608552472341[43] = 0;
   out_7899367608552472341[44] = 0;
   out_7899367608552472341[45] = 0;
   out_7899367608552472341[46] = 0;
   out_7899367608552472341[47] = 0;
   out_7899367608552472341[48] = 0;
   out_7899367608552472341[49] = 0;
   out_7899367608552472341[50] = 1.0;
   out_7899367608552472341[51] = 0;
   out_7899367608552472341[52] = 0;
   out_7899367608552472341[53] = 0;
   out_7899367608552472341[54] = 0;
   out_7899367608552472341[55] = 0;
   out_7899367608552472341[56] = 0;
   out_7899367608552472341[57] = 0;
   out_7899367608552472341[58] = 0;
   out_7899367608552472341[59] = 0;
   out_7899367608552472341[60] = 1.0;
   out_7899367608552472341[61] = 0;
   out_7899367608552472341[62] = 0;
   out_7899367608552472341[63] = 0;
   out_7899367608552472341[64] = 0;
   out_7899367608552472341[65] = 0;
   out_7899367608552472341[66] = 0;
   out_7899367608552472341[67] = 0;
   out_7899367608552472341[68] = 0;
   out_7899367608552472341[69] = 0;
   out_7899367608552472341[70] = 1.0;
   out_7899367608552472341[71] = 0;
   out_7899367608552472341[72] = 0;
   out_7899367608552472341[73] = 0;
   out_7899367608552472341[74] = 0;
   out_7899367608552472341[75] = 0;
   out_7899367608552472341[76] = 0;
   out_7899367608552472341[77] = 0;
   out_7899367608552472341[78] = 0;
   out_7899367608552472341[79] = 0;
   out_7899367608552472341[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4703570443703936485) {
   out_4703570443703936485[0] = state[0];
   out_4703570443703936485[1] = state[1];
   out_4703570443703936485[2] = state[2];
   out_4703570443703936485[3] = state[3];
   out_4703570443703936485[4] = state[4];
   out_4703570443703936485[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4703570443703936485[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4703570443703936485[7] = state[7];
   out_4703570443703936485[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8270162387682653102) {
   out_8270162387682653102[0] = 1;
   out_8270162387682653102[1] = 0;
   out_8270162387682653102[2] = 0;
   out_8270162387682653102[3] = 0;
   out_8270162387682653102[4] = 0;
   out_8270162387682653102[5] = 0;
   out_8270162387682653102[6] = 0;
   out_8270162387682653102[7] = 0;
   out_8270162387682653102[8] = 0;
   out_8270162387682653102[9] = 0;
   out_8270162387682653102[10] = 1;
   out_8270162387682653102[11] = 0;
   out_8270162387682653102[12] = 0;
   out_8270162387682653102[13] = 0;
   out_8270162387682653102[14] = 0;
   out_8270162387682653102[15] = 0;
   out_8270162387682653102[16] = 0;
   out_8270162387682653102[17] = 0;
   out_8270162387682653102[18] = 0;
   out_8270162387682653102[19] = 0;
   out_8270162387682653102[20] = 1;
   out_8270162387682653102[21] = 0;
   out_8270162387682653102[22] = 0;
   out_8270162387682653102[23] = 0;
   out_8270162387682653102[24] = 0;
   out_8270162387682653102[25] = 0;
   out_8270162387682653102[26] = 0;
   out_8270162387682653102[27] = 0;
   out_8270162387682653102[28] = 0;
   out_8270162387682653102[29] = 0;
   out_8270162387682653102[30] = 1;
   out_8270162387682653102[31] = 0;
   out_8270162387682653102[32] = 0;
   out_8270162387682653102[33] = 0;
   out_8270162387682653102[34] = 0;
   out_8270162387682653102[35] = 0;
   out_8270162387682653102[36] = 0;
   out_8270162387682653102[37] = 0;
   out_8270162387682653102[38] = 0;
   out_8270162387682653102[39] = 0;
   out_8270162387682653102[40] = 1;
   out_8270162387682653102[41] = 0;
   out_8270162387682653102[42] = 0;
   out_8270162387682653102[43] = 0;
   out_8270162387682653102[44] = 0;
   out_8270162387682653102[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8270162387682653102[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8270162387682653102[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8270162387682653102[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8270162387682653102[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8270162387682653102[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8270162387682653102[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8270162387682653102[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8270162387682653102[53] = -9.8000000000000007*dt;
   out_8270162387682653102[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8270162387682653102[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8270162387682653102[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8270162387682653102[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8270162387682653102[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8270162387682653102[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8270162387682653102[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8270162387682653102[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8270162387682653102[62] = 0;
   out_8270162387682653102[63] = 0;
   out_8270162387682653102[64] = 0;
   out_8270162387682653102[65] = 0;
   out_8270162387682653102[66] = 0;
   out_8270162387682653102[67] = 0;
   out_8270162387682653102[68] = 0;
   out_8270162387682653102[69] = 0;
   out_8270162387682653102[70] = 1;
   out_8270162387682653102[71] = 0;
   out_8270162387682653102[72] = 0;
   out_8270162387682653102[73] = 0;
   out_8270162387682653102[74] = 0;
   out_8270162387682653102[75] = 0;
   out_8270162387682653102[76] = 0;
   out_8270162387682653102[77] = 0;
   out_8270162387682653102[78] = 0;
   out_8270162387682653102[79] = 0;
   out_8270162387682653102[80] = 1;
}
void h_25(double *state, double *unused, double *out_2875940558495848619) {
   out_2875940558495848619[0] = state[6];
}
void H_25(double *state, double *unused, double *out_437677348284835849) {
   out_437677348284835849[0] = 0;
   out_437677348284835849[1] = 0;
   out_437677348284835849[2] = 0;
   out_437677348284835849[3] = 0;
   out_437677348284835849[4] = 0;
   out_437677348284835849[5] = 0;
   out_437677348284835849[6] = 1;
   out_437677348284835849[7] = 0;
   out_437677348284835849[8] = 0;
}
void h_24(double *state, double *unused, double *out_6108302598805820632) {
   out_6108302598805820632[0] = state[4];
   out_6108302598805820632[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8781001539355520542) {
   out_8781001539355520542[0] = 0;
   out_8781001539355520542[1] = 0;
   out_8781001539355520542[2] = 0;
   out_8781001539355520542[3] = 0;
   out_8781001539355520542[4] = 1;
   out_8781001539355520542[5] = 0;
   out_8781001539355520542[6] = 0;
   out_8781001539355520542[7] = 0;
   out_8781001539355520542[8] = 0;
   out_8781001539355520542[9] = 0;
   out_8781001539355520542[10] = 0;
   out_8781001539355520542[11] = 0;
   out_8781001539355520542[12] = 0;
   out_8781001539355520542[13] = 0;
   out_8781001539355520542[14] = 1;
   out_8781001539355520542[15] = 0;
   out_8781001539355520542[16] = 0;
   out_8781001539355520542[17] = 0;
}
void h_30(double *state, double *unused, double *out_71785431859636342) {
   out_71785431859636342[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4090018981842772349) {
   out_4090018981842772349[0] = 0;
   out_4090018981842772349[1] = 0;
   out_4090018981842772349[2] = 0;
   out_4090018981842772349[3] = 0;
   out_4090018981842772349[4] = 1;
   out_4090018981842772349[5] = 0;
   out_4090018981842772349[6] = 0;
   out_4090018981842772349[7] = 0;
   out_4090018981842772349[8] = 0;
}
void h_26(double *state, double *unused, double *out_3439617997365578500) {
   out_3439617997365578500[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3303825970589220375) {
   out_3303825970589220375[0] = 0;
   out_3303825970589220375[1] = 0;
   out_3303825970589220375[2] = 0;
   out_3303825970589220375[3] = 0;
   out_3303825970589220375[4] = 0;
   out_3303825970589220375[5] = 0;
   out_3303825970589220375[6] = 0;
   out_3303825970589220375[7] = 1;
   out_3303825970589220375[8] = 0;
}
void h_27(double *state, double *unused, double *out_6144602853944627823) {
   out_6144602853944627823[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6264782293643197260) {
   out_6264782293643197260[0] = 0;
   out_6264782293643197260[1] = 0;
   out_6264782293643197260[2] = 0;
   out_6264782293643197260[3] = 1;
   out_6264782293643197260[4] = 0;
   out_6264782293643197260[5] = 0;
   out_6264782293643197260[6] = 0;
   out_6264782293643197260[7] = 0;
   out_6264782293643197260[8] = 0;
}
void h_29(double *state, double *unused, double *out_5987416185593498678) {
   out_5987416185593498678[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3579787637528380165) {
   out_3579787637528380165[0] = 0;
   out_3579787637528380165[1] = 1;
   out_3579787637528380165[2] = 0;
   out_3579787637528380165[3] = 0;
   out_3579787637528380165[4] = 0;
   out_3579787637528380165[5] = 0;
   out_3579787637528380165[6] = 0;
   out_3579787637528380165[7] = 0;
   out_3579787637528380165[8] = 0;
}
void h_28(double *state, double *unused, double *out_3107816802500295138) {
   out_3107816802500295138[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8662186654597910739) {
   out_8662186654597910739[0] = 1;
   out_8662186654597910739[1] = 0;
   out_8662186654597910739[2] = 0;
   out_8662186654597910739[3] = 0;
   out_8662186654597910739[4] = 0;
   out_8662186654597910739[5] = 0;
   out_8662186654597910739[6] = 0;
   out_8662186654597910739[7] = 0;
   out_8662186654597910739[8] = 0;
}
void h_31(double *state, double *unused, double *out_2718753890144719474) {
   out_2718753890144719474[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3930034072822571851) {
   out_3930034072822571851[0] = 0;
   out_3930034072822571851[1] = 0;
   out_3930034072822571851[2] = 0;
   out_3930034072822571851[3] = 0;
   out_3930034072822571851[4] = 0;
   out_3930034072822571851[5] = 0;
   out_3930034072822571851[6] = 0;
   out_3930034072822571851[7] = 0;
   out_3930034072822571851[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6894033735385027323) {
  err_fun(nom_x, delta_x, out_6894033735385027323);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3944300214870265721) {
  inv_err_fun(nom_x, true_x, out_3944300214870265721);
}
void car_H_mod_fun(double *state, double *out_7899367608552472341) {
  H_mod_fun(state, out_7899367608552472341);
}
void car_f_fun(double *state, double dt, double *out_4703570443703936485) {
  f_fun(state,  dt, out_4703570443703936485);
}
void car_F_fun(double *state, double dt, double *out_8270162387682653102) {
  F_fun(state,  dt, out_8270162387682653102);
}
void car_h_25(double *state, double *unused, double *out_2875940558495848619) {
  h_25(state, unused, out_2875940558495848619);
}
void car_H_25(double *state, double *unused, double *out_437677348284835849) {
  H_25(state, unused, out_437677348284835849);
}
void car_h_24(double *state, double *unused, double *out_6108302598805820632) {
  h_24(state, unused, out_6108302598805820632);
}
void car_H_24(double *state, double *unused, double *out_8781001539355520542) {
  H_24(state, unused, out_8781001539355520542);
}
void car_h_30(double *state, double *unused, double *out_71785431859636342) {
  h_30(state, unused, out_71785431859636342);
}
void car_H_30(double *state, double *unused, double *out_4090018981842772349) {
  H_30(state, unused, out_4090018981842772349);
}
void car_h_26(double *state, double *unused, double *out_3439617997365578500) {
  h_26(state, unused, out_3439617997365578500);
}
void car_H_26(double *state, double *unused, double *out_3303825970589220375) {
  H_26(state, unused, out_3303825970589220375);
}
void car_h_27(double *state, double *unused, double *out_6144602853944627823) {
  h_27(state, unused, out_6144602853944627823);
}
void car_H_27(double *state, double *unused, double *out_6264782293643197260) {
  H_27(state, unused, out_6264782293643197260);
}
void car_h_29(double *state, double *unused, double *out_5987416185593498678) {
  h_29(state, unused, out_5987416185593498678);
}
void car_H_29(double *state, double *unused, double *out_3579787637528380165) {
  H_29(state, unused, out_3579787637528380165);
}
void car_h_28(double *state, double *unused, double *out_3107816802500295138) {
  h_28(state, unused, out_3107816802500295138);
}
void car_H_28(double *state, double *unused, double *out_8662186654597910739) {
  H_28(state, unused, out_8662186654597910739);
}
void car_h_31(double *state, double *unused, double *out_2718753890144719474) {
  h_31(state, unused, out_2718753890144719474);
}
void car_H_31(double *state, double *unused, double *out_3930034072822571851) {
  H_31(state, unused, out_3930034072822571851);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
