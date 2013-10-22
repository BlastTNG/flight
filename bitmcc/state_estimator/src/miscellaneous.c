#include <math.h>
#include <stdio.h>
#include <gsl/gsl_blas.h>

#include "miscellaneous.h"

#define pi 3.1415926535897932384626433832795

// read out the m vector to console
void print_vector(const gsl_vector *vec)
{
    int m = vec->size;
    int i;
    for (i=0; i<m; i++)
    {
        printf("%4.6g\n",gsl_vector_get(vec,i));
    }
    printf("\n");
    return;
}

// read out the m by n matrix to console
void print_matrix(const gsl_matrix *mat)
{
    int m = mat->size1;
    int n = mat->size2;
    int i,j;

    for (i=0; i<m; i++)
    {
        for (j=0; j<n; j++)
        {
        	printf("%4.6g, ",gsl_matrix_get(mat,i,j));
        }
        printf("\n");
    }
    printf("\n");
    return;
}

// rotation matrix about x axis by x
void C_x(gsl_matrix * m, const double x)
{
    gsl_matrix_set_zero(m);
    gsl_matrix_set(m,0,0,1);
    gsl_matrix_set(m,1,1,cos(x));
    gsl_matrix_set(m,1,2,sin(x));
    gsl_matrix_set(m,2,1,-sin(x));
    gsl_matrix_set(m,2,2,cos(x));
    return;
}

// rotation matrix about y axis by x
void C_y(gsl_matrix * m, const double x)
{
    gsl_matrix_set_zero(m);
    gsl_matrix_set(m,1,1,1);
    gsl_matrix_set(m,0,0,cos(x));
    gsl_matrix_set(m,0,2,-sin(x));
    gsl_matrix_set(m,2,0,sin(x));
    gsl_matrix_set(m,2,2,cos(x));
    return;
}

// rotation matrix about z axis by x
void C_z(gsl_matrix * m, const double x)
{
    gsl_matrix_set_zero(m);
    gsl_matrix_set(m,2,2,1);
    gsl_matrix_set(m,0,0,cos(x));
    gsl_matrix_set(m,0,1,sin(x));
    gsl_matrix_set(m,1,0,-sin(x));
    gsl_matrix_set(m,1,1,cos(x));
    return;
}

// cross product matrix for vector v
void xmat(gsl_matrix *m, const gsl_vector *v)
{
    gsl_matrix_set_zero(m);
    gsl_matrix_set(m,0,1,-gsl_vector_get(v,2));
    gsl_matrix_set(m,1,0,gsl_vector_get(v,2));
    gsl_matrix_set(m,0,2,gsl_vector_get(v,1));
    gsl_matrix_set(m,2,0,-gsl_vector_get(v,1));
    gsl_matrix_set(m,1,2,-gsl_vector_get(v,0));
    gsl_matrix_set(m,2,1,gsl_vector_get(v,0));
    return;
}

// un-crosses a matrix M to retrieve the original vector v
// (only works for skew-symmetric 3x3 matrices with zeros on diagonal
void unxmat(gsl_vector * v, const gsl_matrix * M)
{
    gsl_vector_set(v,0,(gsl_matrix_get(M,2,1)-gsl_matrix_get(M,1,2))/2.0);
    gsl_vector_set(v,1,(gsl_matrix_get(M,0,2)-gsl_matrix_get(M,2,0))/2.0);
    gsl_vector_set(v,2,(gsl_matrix_get(M,1,0)-gsl_matrix_get(M,0,1))/2.0);
    return;
}

// convert axis-angle representation to rotation matrix
void axis2rot(gsl_matrix *m, const gsl_vector *v, const double phi)
{
    double cosa = cos(phi);
    double sina = sin(phi);
    double mij;
    double a[] = {gsl_vector_get(v, 0),gsl_vector_get(v, 1),gsl_vector_get(v, 2)};
    double sign = 1.0;
    
    gsl_matrix_set_zero(m);
    int i,j;
    
    for (i=0; i<3; i++)
    {
        for (j=i; j<3; j++)
        {
            mij = (1-cosa)*a[i]*a[j];
            if (i == j) 
            {
                mij += cosa;
                gsl_matrix_set(m,i,j,mij);
            }
            else
            {
                gsl_matrix_set(m,i,j,mij+sign*sina*a[3-i-j]);
                gsl_matrix_set(m,j,i,mij-sign*sina*a[3-i-j]);
                sign = sign*-1.0;
            }
        }
    }
    
    return;
}

// concatenates two matrices M and N horizontally and stores the result in MAT
void horiz_concat(gsl_matrix * MAT, const gsl_matrix * M, const gsl_matrix * N)
{
    if ((M->size1) != (N->size1))
    {
    	printf("horiz_concat: Invalid dimensions\n");
        return;
    }
    int h = M->size1; // height
    int A = M->size2;
    int B = N->size2;
    
    gsl_matrix_view aview = gsl_matrix_submatrix(MAT, 0, 0, h, A);
    gsl_matrix_view bview = gsl_matrix_submatrix(MAT, 0, A, h, B);
    
    gsl_matrix_memcpy(&aview.matrix, M);
    gsl_matrix_memcpy(&bview.matrix, N);

    return;
}

// concatenates two matrices M and N vertically and stores the result in MAT
void vert_concat(gsl_matrix * MAT, const gsl_matrix * M, const gsl_matrix * N)
{
    if ((M->size2) != (N->size2))
    {
    	printf("vert_concat: Invalid dimensions\n");
        return;
    }
    int w = M->size2; // width
    int A = M->size1;
    int B = N->size1;
    
    gsl_matrix_view aview = gsl_matrix_submatrix(MAT, 0, 0, A, w);
    gsl_matrix_view bview = gsl_matrix_submatrix(MAT, A, 0, B, w);
    
    gsl_matrix_memcpy(&aview.matrix, M);
    gsl_matrix_memcpy(&bview.matrix, N);
    
    return;
}
// combines two vectos u and v and stores the result in vec
void vec_concat(gsl_vector * vec, const gsl_vector * u, const gsl_vector * v)
{
	int i;
    for (i=0; i<((u->size)+(v->size)); i++)
    {
        if (i<(u->size))
        {
            gsl_vector_set(vec,i,gsl_vector_get(u,i));
        }
        else
        {
            gsl_vector_set(vec,i,gsl_vector_get(v,i-(u->size)));
        }
    }
    return;
}

// performs the multiplication C = alpha*A*M*A'+beta*C for M > 0 and C > 0
void quadratic_mult_pd(const double alpha, gsl_matrix * A, gsl_matrix * M, const double beta, gsl_matrix * C)
{
    double sum, a[A->size2];
    gsl_vector_view col, row, temp;
    temp = gsl_vector_view_array(a,A->size2);
    
    int i,j;

    for (i=0; i<(C->size1); i++)
    {
        col = gsl_matrix_row(A,i);
        gsl_blas_dsymv(CblasUpper, 1.0, M, &col.vector, 0.0, &temp.vector);
        for (j=i; j<(C->size2); j++)
        {
            sum = 0;
            row = gsl_matrix_row(A,j);
            gsl_blas_ddot(&temp.vector,&row.vector,&sum);
            sum += beta*gsl_matrix_get(C,i,j);
            gsl_matrix_set(C,i,j,sum);
            gsl_matrix_set(C,j,i,sum);
        }
    }
    
    return;
}


// computes the 3-1-2 Euler paramterization phi from rotation matrix C
void parameterize_312_rotation(gsl_vector * phi, const gsl_matrix * C)
{

    double phix,phiy,phiz;
    phix = asin(gsl_matrix_get(C,1,2));
    phiy = atan2(-gsl_matrix_get(C,0,2),gsl_matrix_get(C,2,2));
    phiz = atan2(-gsl_matrix_get(C,1,0),gsl_matrix_get(C,1,1));
    gsl_vector_set(phi,0,phix);
    gsl_vector_set(phi,1,phiy);
    gsl_vector_set(phi,2,phiz);

    return;
}

// computes the rotation matrix for a 3-1-2 Euler parameterizations from phi
void compute_312_rotation_matrix(gsl_matrix * C, const gsl_vector * phi)
{
    double cos1, cos2, cos3, sin1, sin2, sin3;
    double p1 = gsl_vector_get(phi,0);
    double p2 = gsl_vector_get(phi,1);
    double p3 = gsl_vector_get(phi,2);

    cos1 = cos(p1);
    cos2 = cos(p2);
    cos3 = cos(p3);
    sin1 = sin(p1);
    sin2 = sin(p2);
    sin3 = sin(p3);

    gsl_matrix_set(C,0,0,cos2*cos3-sin1*sin2*sin3);
    gsl_matrix_set(C,0,1,cos2*sin3+sin1*sin2*cos3);
    gsl_matrix_set(C,0,2,-cos1*sin2);

    gsl_matrix_set(C,1,0,-cos1*sin3);
    gsl_matrix_set(C,1,1,cos1*cos3);
    gsl_matrix_set(C,1,2,sin1);

    gsl_matrix_set(C,2,0,sin2*cos3+sin1*cos2*sin3);
    gsl_matrix_set(C,2,1,sin2*sin3-sin1*cos2*cos3);
    gsl_matrix_set(C,2,2,cos1*cos2);

    return;

    /*
    gsl_matrix * temp = gsl_matrix_alloc(3,3);
    gsl_matrix * C_n = gsl_matrix_alloc(3,3);

    C_z(C,gsl_vector_get(phi,2));
    gsl_matrix_memcpy(temp,C);
    C_x(C_n,gsl_vector_get(phi,0));
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,C_n,temp,0.0,C);
    gsl_matrix_memcpy(temp,C);
    C_y(C_n,gsl_vector_get(phi,1));
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,C_n,temp,0.0,C);

    gsl_matrix_free(temp);
    gsl_matrix_free(C_n);
    return;
    */
}
