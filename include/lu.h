#ifndef H_LU
#define H_LU

int gsl_linalg_LU_svx_float (const gsl_matrix_float * LU, const gsl_permutation * p, gsl_vector_float * x);
int gsl_linalg_LU_decomp_float (gsl_matrix_float * A, gsl_permutation * p, int *signum);
int gsl_linalg_LU_solve_float (const gsl_matrix_float * LU, const gsl_permutation * p, const gsl_vector_float * b, gsl_vector_float * x);
int gsl_linalg_LU_svx_float (const gsl_matrix_float * LU, const gsl_permutation * p, gsl_vector_float * x);
int gsl_linalg_LU_refine_float (const gsl_matrix_float * A, const gsl_matrix_float * LU, const gsl_permutation * p, const gsl_vector_float * b,
gsl_vector_float * x, gsl_vector_float * residual);
int gsl_linalg_LU_invert_float (const gsl_matrix_float * LU, const gsl_permutation * p, gsl_matrix_float * inverse);
float gsl_linalg_LU_det_float (gsl_matrix_float * LU, int signum);
float gsl_linalg_LU_lndet_float (gsl_matrix_float * LU);
int gsl_linalg_LU_sgndet_float (gsl_matrix_float * LU, int signum);

#endif
