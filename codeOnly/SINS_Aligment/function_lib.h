

extern "C" int __declspec(dllexport) main();

void   AddingVector(int, double *,double *,double *);
double ScalarProduct(int, double *,double *);
void   MatrixOnVector(int,int, double *,double *,double *);
void   MatrixOnMatrix(int,int,int, double *,double *,double *);
void   EquatingOfVectors(int, double *,double *);
void   EquatingOfMatrix(int,int, double *,double *);
void   NullingOfArray(int, double*);
void   PrintOfArray(int, int, char, double *, int);
void   PrintOfArrayEasy(int, int, double *, int );
int	   IsolationSubMatrix(int, int, double *, double *, int, int);

void   func(double,double *,double *,int);
double rnd_gauss(double,double);
void   dgq0b(double *,double *,double *,double *,double *,double *,int,int);
void   dg0b(double *,double *,double *,double *,double *,int);
void   f0b(double,double *,double *,double *,double,double *,double *,double *,int);
void PrintInFile (int , int , double *);
void TranspMatrix (int , double * , double * );


void RoughAligment	(double, double , double *, double *, double *, double *, double *, double *, double *, double *, double *, double *, double *, FILE    *, double , FILE *, double *, FILE *);
void InitialOfMatrix (double *, double *, double , double , double , double );
void IntegrationOfOrientationMatrix (double *, double *, double *, double , double , double );
void KalmanFilter (int , int , int  , double , double *, double *, double *, double *, double *, double *, double *, double , double , double *, double *, double *, double *, double *, double , double *, double , double , double *, FILE *, double *, double);
void KalmanFilter_2 (int , int , int  , double , double *, double *, double *, double *, double *, double *, double *, double , double , double *, double *, double *, double *, double *, double , double *, double , double , double *, FILE *, double *, double);

