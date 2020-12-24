#include "oneplusone.h"
#include "stdio.h"
#include <stdlib.h>

double * mytest;
double ** matrix;

int oneplusoneplus(int val) {
    printf("Hello! one plus one\n");
    printf("your value = %d\n", val);
    return 1 + 1 + val;
}

int show(int *val, int size){
    int sum = 0;
    for (int i=0; i<size; i++) {
        printf("%d\n", val[i]);
        sum = sum + val[i];
    }
    return sum;
}

void print_hello() {
    printf("Hello!!!!! \n");
}

void set_matrix2(int n_rows, int n_cols, double (*mat)[n_cols] ) {
    for (int i=0; i<n_rows; i++) {
        for (int j=0; j<n_cols; j++) {
            printf("%d, %d, %f\n", i, j, mat[i][j]);
        }
    }
}

void set_matrix_from_julia(double ** cmat, double * jmat, int n_rows, int n_cols) {
    for (int i=0; i<n_rows; i++) {
        for (int j=0; j<n_cols; j++) {
            cmat[i][j] = jmat[i + j * n_rows];
        }
    }    
}

void set_matrix(double * mat, int n_rows, int n_cols) {
    printf("n_rows=%d, n_cols=%d \n", n_rows, n_cols);
    
    matrix = malloc(n_rows * sizeof(double *));
    for (int i=0; i<n_rows; i++) {
        matrix[i] = malloc(n_cols * sizeof(double));
    }
    printf("Matrix received from Julia:\n");

    for (int i=0; i<n_rows; i++) {
        for (int j=0; j<n_cols; j++) {
            printf("%3.0f ", mat[i+j*n_rows]);
        }
        printf("\n");
    }    

    set_matrix_from_julia(matrix, mat, n_rows, n_cols);

    printf("Matrix set in C:\n");

    for (int i=0; i<n_rows; i++) {
        for (int j=0; j<n_cols; j++) {
            printf("%3.0f ", matrix[i][j]);
        }
        printf("\n");
    }    
}

double ** get_matrix() {
    printf("getting matrix done!\n");

    return matrix;
}

void set_test(double * val, int len) {
    print_hello();
    mytest = malloc(sizeof(double) * len);
    for (int i=0; i<len; i++) {
        mytest[i] = val[i];
    }
}

double * get_test() {
    print_hello();
    return mytest;
}

