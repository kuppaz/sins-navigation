class StochasticModel
{

double *X_,*P_, *Sq_, *Q1_, *Q_, *W;
int M, Mq;

public:
       double *X, *F, *P,*Bq, *Q, *Q1, *Sq;

//---------------------------------------------------------------

StochasticModel(int iM,int iMq)
{

StochasticModel::M=iM;      StochasticModel::Mq=iMq;

X    = new double[iM];           P   = new double[iM*iM];
X_   = new double[iM];           P_  = new double[iM*iM];

F    = new double[iM*iM];
Q1_ = new double[iM];

if(Mq)
{
Bq   =  new double[iM*iMq];      Q   = new double[iMq*iMq];
Sq   = new double[iMq*iMq];      Sq_ = new double[iM*iMq];
Q1   = new double[iMq];          
}

W   = new double[iM];            Q_  = new double[iM*iM];

return;
}

//---------------------------------------------------------------

~StochasticModel()
{

delete [] X ;         delete [] P ;
delete [] X_;         delete [] P_;
delete [] F ;
delete [] Q1_;
delete [] Q_;

if(Mq)
{
delete [] Sq;         
delete [] Sq_;         
delete [] Q;         
delete [] Q1;         
delete [] Bq;
}
delete [] W;

return;
}


//--------------------------------------------------------------

void reset()
{
double *pd;
pd = X_;    X_ = X;    X = pd;
pd = P_;    P_ = P;    P = pd;

return;
}


//------------------------------------------------------------

void Propagation(void)
{

if(Mq)
  {

   MatrixOnVector(M, Mq, Sq_, Q1, Q1_ );
  }

 reset();

return;
}


};    // END CLASS StochasticModel

