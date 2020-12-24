#define MAX_NODES 128 
#define LEN(x) sizeof(x) / sizeof(x[0])

double * mytest;
double ** matrix;


int oneplusoneplus(int val);
int show(int * val, int size);
void set_test(double * val, int len);
double * get_test();
void set_matrix(double * mat, int n_rows, int n_cols);
double ** get_matrix();
void set_matrix2(int n_rows, int n_cols, double (*mat)[n_cols]);