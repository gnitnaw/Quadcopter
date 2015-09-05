#include <stdio.h>
#include <math.h>
#include <gsl/gsl_blas.h>

static double Euler[3], Angle[3], alpha;
static double norm;
static double magNord[3], acclNorm[3];
static double rotateAll[9];
static double q[4];
static int i, j, k;
double getNorm(double* val) {
    return sqrt(pow(val[0],2)+pow(val[1],2)+pow(val[2],2)) ;
}

double getAngle(double *val) {
    return acos(val[1]/sqrt(pow(val[0],2) + pow(val[1],2)));
}


void Euler_init(double* accl, double* magn) {
    norm = getNorm(accl);
    for (i=0; i<3; ++i) {
	acclNorm[i] = accl[i] / norm;
	Euler[i] = acos(acclNorm[i]);
    }

    alpha = sqrt((acclNorm[2]+1)/2);

//    q[0] = alpha;
//    q[1] = sqrt(1-pow(alpha,2));
//    for (i=2; i<4; ++i) q[i] = q[1];
//    for (i=1; i<4; ++i) q[i] *= acclNorm[i];

//    Euler[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]) , 1-2*(q[1]*q[1]+q[2]*q[2]));
//    Euler[1] = asin(2*(q[0]*q[2]-q[1]*q[3]));
//    Euler[2] = atan2(2*(q[0]*q[3]+q[2]*q[1]) , 1-2*(q[2]*q[2]+q[3]*q[3]));


//    Euler[2] = getAngle(magn);

    double rotate[4][9] = { {1,0,0, 0, cos(Euler[0]), -sin(Euler[0]) , 0, sin(Euler[0]), cos(Euler[0]) } ,
                {cos(Euler[1]), 0, -sin(Euler[1]) , 0,1,0 , sin(Euler[1]), 0, cos(Euler[1])},
                {cos(Euler[2]), -sin(Euler[2]), 0 , sin(Euler[2]), cos(Euler[2]), 0 , 0,0,1},
                {0,0,0,0,0,0,0,0,0}};
/*

    double rotate[4][9] = { {1,0,0, 0, cos(Euler[0]), sin(Euler[0]) , 0, -sin(Euler[0]), cos(Euler[0]) } ,
                {cos(Euler[1]), 0, sin(Euler[1]) , 0,1,0 , -sin(Euler[1]), 0, cos(Euler[1])},
                {cos(Euler[2]), sin(Euler[2]), 0 , -sin(Euler[2]), cos(Euler[2]), 0 , 0,0,1},
                {0,0,0,0,0,0,0,0,0}};
*/
    gsl_matrix_float_view A = gsl_matrix_float_view_array(&rotate[0][0], 3, 3);
    gsl_matrix_float_view B = gsl_matrix_float_view_array(&rotate[1][0], 3, 3);
    gsl_matrix_float_view C = gsl_matrix_float_view_array(&rotate[2][0], 3, 3);
    gsl_matrix_float_view D = gsl_matrix_float_view_array(&rotate[3][0], 3, 3);
    gsl_matrix_float_view E = gsl_matrix_float_view_array(rotateAll, 3, 3);
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &B.matrix, &A.matrix, 0.0, &D.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &D.matrix, &C.matrix, 0.0, &E.matrix);

//    norm = getNorm(magn);
    for (i=0; i<3; ++i) magNord[i] = 0;
    magNord[2] = norm;

    gsl_vector_float_view mm = gsl_vector_float_view_array(magNord, 3);
    gsl_vector_float_view aa = gsl_vector_float_view_array(Angle, 3);

    gsl_blas_sgemv (CblasNoTrans, 1.0, &E.matrix, &mm.vector, 0.0, &aa.vector);

/*
    magNord[0] = (cos(Euler[1])*cos(Euler[0]) * magn[0] + (sin(Euler[2])*sin(Euler[1])*cos(Euler[0])-cos(Euler[2])*sin(Euler[0])) * magn[1]
		+ (cos(Euler[2])*sin(Euler[1])*cos(Euler[0])+sin(Euler[2])*sin(Euler[0])) * magn[2])/norm;
    magNord[1] = (cos(Euler[1])*sin(Euler[0]) * magn[0] + (sin(Euler[2])*sin(Euler[1])*sin(Euler[0])-cos(Euler[2])*cos(Euler[0])) * magn[1]
		+ (cos(Euler[2])*sin(Euler[1])*sin(Euler[0])-sin(Euler[2])*cos(Euler[0])) * magn[2])/norm;
    magNord[2] = (-sin(Euler[1]) * magn[0] + sin(Euler[2])*cos(Euler[1]) * magn[1] + cos(Euler[2])*cos(Euler[1]) * magn[2])/norm;

    magNord[0] = (cos(Euler[0])*cos(Euler[1]) - cos(Euler[2])*sin(Euler[0])*sin(Euler[1])) * magn[0] +
 		 (sin(Euler[0])*cos(Euler[1]) + cos(Euler[2])*cos(Euler[0])*sin(Euler[1])) * magn[1] +
		  sin(Euler[2])*sin(Euler[1]) * magn[2];

    magNord[1] = (-cos(Euler[0])*sin(Euler[1]) - cos(Euler[2])*sin(Euler[0])*cos(Euler[1])) * magn[0] +
                 (-sin(Euler[0])*sin(Euler[1]) + cos(Euler[2])*cos(Euler[0])*cos(Euler[1])) * magn[1] +
                  sin(Euler[2])*cos(Euler[1]) * magn[2];

    magNord[2] = sin(Euler[2])*sin(Euler[0]) * magn[0] - sin(Euler[2])*cos(Euler[0])*magn[1] + cos(Euler[2]) * magn[2];
*/
/*
    for (i=0; i<3; ++i) {
	magNord[i] = 0;
	for (j=0; j=3; ++j) {
	    magNord[i] += 
	}
        magNord[i] /= norm;
    }
*/
    printf("%f, %f, %f\t",accl[0],accl[1],accl[2]);
    printf("%f, %f, %f\t",magNord[0],magNord[1],magNord[2]);
    printf("%f, %f, %f\n", Angle[0],Angle[1],Angle[2]);
}
