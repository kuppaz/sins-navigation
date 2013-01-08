#include <math.h>
#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void AddingVector(int Dimension, double *VectorA,double *VectorB,double *VectorExit)
//��������� ������� VectorA � VectorB, ������� ������ VectorExit
{
      int i;
      for(i=0; i<Dimension; i++) 
		  VectorExit[i] = VectorA[i]+VectorB[i];
}

void TranspMatrix (int Dimension, double *Matrix, double *MatrixExit )
{
	int i,j;
	for (i=0; i< Dimension; i++)
	{
		for (j=0; j< Dimension; j++)
		{
			MatrixExit[j*Dimension+i] = Matrix[i*Dimension+j];
		}
	}
}

double ScalarProduct(int Dimension, double *VectorA,double *VectorB)
//��������� ������������ �������� VectorA � Vector B, �� ������ ScalarExit
{
    int i;
    double ScalarExit = .0;
    for(i=0; i<Dimension; i++)
		ScalarExit+=VectorA[i]*VectorB[i];
    return(ScalarExit);
}

void MatrixOnVector(int DimensionExit,int DimensionVector, double *MatrixA, double *VectorB, double *VectorExit)
//��������� ������� m*n �� ������ ����������� n(DimensionVector), ��������� � �������� ������� VectorExit
{
   int i,j;
   double y;
     for(i=0; i<DimensionExit; i++)
     {
       y=.0;
       for(j=0; j<DimensionVector; j++) 
		   y += MatrixA[i*DimensionVector+j]*VectorB[j];
	   VectorExit[i] = y;
     }
}

void PrintInFile (int n, int m, double *Matrix)
{
	FILE    *PrintOfMatrix;
		     PrintOfMatrix = fopen("D://�����//Postgraduate//1#_Scientific work//1#_Software//1#_Mine//Motion Imitator//MovingImitator//SINS motion processing_new data//PrintOfMatrix.txt", "w");
	int i,j;
	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			fprintf (PrintOfMatrix, "%.10f\t", Matrix[m*i+j]);
		}
		fprintf (PrintOfMatrix, "\n");
	}
	fclose (PrintOfMatrix);
	system ("pause");
}

void MatrixOnMatrix(int n1,int n2,int n3, double *MatrixA,double *MatrixB,double *MatrixExit)
//��������� ������� MatrixA ����������� n1*n2 �� ������� MatrixB ����������� n2*n3, ��������� � �������� ������� VectorExit
{
   int i,j,k;
   double y;
   for(i=0; i<n1; i++)
   {
		for(j=0; j<n3; j++)
		{
			y=.0;
			for(k=0; k<n2; k++) 
				y += MatrixA[i*n1+k]*MatrixB[k*n2+j];
        MatrixExit[i*n1+j] = y;
		}
   }
}

void EquatingOfVectors(int Dimension, double *VectorA, double *VectorB)
//������������� �������� ������� VectorA ������� VectorB
{
       int i;
       for (i=0; i<Dimension; i++) 
		   VectorB[i]=VectorA[i];
}

void EquatingOfMatrix(int m, int n, double *MatrixA, double *MatrixB)
//������������� �������� ������� MatrixA ������� MatrixB
{
      int i,j;
      for(i=0; i<m; i++)
      {
          for(j=0; j<n; j++)
          {
             MatrixB[i*n+j]=MatrixA[i*n+j];
          }
      }
}
void NullingOfArray(int Dimention, double* Array)
//��������� �������
{
     int i;
     for(i=0; i<Dimention; i++)
              Array[i] = .0;
}
void PrintOfArray(int Strings, int Columns, char nat, double *Array, int pause)
//������� �� ����� ������ ����������� String*Columns � 'nat' ����������� ������ ����� �������
{
	int i,j;
	char a[10]=" %.";
	sprintf(a+3,"%c",nat);
	sprintf(a+strlen(a),"f");
	printf ("\n");
	for(i=0; i<Strings ; i++)
    {
             for (j=0 ; j<Columns; j++)
                 printf((const char*) a, Array[i*Columns+j]);
             printf ("\n");
    }
	if(pause == 1)
		system("PAUSE");
}

void PrintOfArrayEasy(int Strings, int Columns, double *Array, int pause)
//������� �� ����� ������ ����������� String*Columns
{
	int i,j;
	printf ("\n");
	for(i=0; i<Strings ; i++)
    {
             for (j=0 ; j<Columns; j++)
                 printf("%.3f\t", Array[i*Columns+j]);
             printf ("\n");
    }
	if(pause == 1)
		system("PAUSE");
}

int IsolationSubMatrix(int DimensionMatrix, int DimensionSubMatrix, double *Matrix, double *SubMatrix, int n, int m)
{
	int i,j;
	if ((DimensionSubMatrix+m-1) > DimensionMatrix || (DimensionSubMatrix+n-1) > DimensionMatrix)
	{
		printf ("\n\t Error");
		system("pause");
		return 0;
	}
	for (i=0; i<DimensionSubMatrix; i++)
	{
		for(j=0; j<DimensionSubMatrix; j++)
			SubMatrix[i*DimensionSubMatrix+j] = Matrix[(n-1+i)*DimensionMatrix+(m-1)+j];
	}
	return 0;
}

double rnd_gauss(double m,double s)
{
	int  i;
	double x=0.0;
	for(i=0; i < 12; i++) 
		x+=(double)rand()/32767.0-0.5; 
	return(s*x+m);
}

/*
*
*_________________________________________________________________
*
*                 *********
*                 *  f0b  *
*                 *********
*
*   f0b(z,xm,sm,h,r,xp,sp,kf,m) - ������������         ����������
*                                 �������� �������   �������   ��
*                                 �����    ���������    ���������.
*                                 (������   ����    ��������������
*                                 �����������).
*
*   ������� ���������:
*
*                m  - ����������� ������� (������� ���������);
*                z  - ���������  z (������);         _
*                xm - ������������� ��������� ������ �(-) (m x 1);
*                sm - �������������      ����������     �������
*                     ����������� ����� S(-) (m x m), ���������� � ����� �������;
*                h  - ������������� ������� ��������� h (m x 1);
*                r  - �������������  ��������� ���� ���������  z
*                     (������).
*
*   �������� ���������:
*                                                             _
*                  xp - ������������� �������������  ������   �(+)
*                       (m x 1);
*                  sp - �������������   ��������������    �������
*                       ����������� ����� S(+) (m x m), ���������� � ����� �������;
*                  kf - �������������  �������������  ������������
*                       �������� (m x 1).
*
*   ��������!!!
*   ��������!!!   ���  ��������   sm   �   sp   ��������� �������
*   ��������!!!   ������ �� ����������������� �������.
*   ��������!!!
*
*
*/

void f0b(double z,double *xm,double *sm,double *h,
         double r,double *xp,double *sp,double *kf,int m)
  {
   int i,j,ji,ii,m1;
   double y1,c,y,zm,al,al1,a;
     m1=m+1;
     y1=*sm;
     c=*h;
     y=c*y1;
     zm=c**xm;
     al=r+y*y;
     *kf=y1*y;
     *sp=y1*sqrt(r/al);
       for(i=1;i < m;i++)
         {
          y=*(h+i);
          zm+=y**(xm+i);
          ii=m1*i;
          y*=*(sm+ii);
            for(j=0;j < i;j++) y+=*(sm+j*m+i)**(h+j);
          al1=al+y*y;
          a=1.0/sqrt(al*al1);
          c=y*a;
          a*=al;
          al=al1;
            for(j=0;j < i;j++)
              {
               y1=*(kf+j);
               ji=j*m+i;
               al1=*(sm+ji);
               *(sp+ji)=al1*a-y1*c;
               *(kf+j)=y1+al1*y;
               }
               y1=*(sm+ii);
               *(sp+ii)=y1*a;
               *(kf+i)=y1*y;
       }
        y=1.0/al;
        y1=(z-zm)*y;
          for(i=0;i < m;i++)
            {
             a=*(kf+i);
             *(xp+i)=*(xm+i)+a*y1;
             *(kf+i)=a*y;
            }
  }


/*
*
*_________________________________________________________________
*
*                 **********
*                 *  dg0b  *
*                 **********
*
*   dg0b(xp,sp,f,xm,sm,m,w) - ������������   �����������  ��������
*                             �������  ������� ��� ����� ��������.
*                             (������     ����      ��������������
*                             �����������).
*                             ������    ���������������  ���������
*                             ���������� ��������� ���������������
*                             ������-������.
*
*   ��������!!!
*   ��������!!!   � ������ ������ ��� �������  q=B*u  �����������.
*   ��������!!!
*
*   ������� ���������:
*
*                m  - ����������� ������� (������� ���������);_
*                xp - ������������� ������ ������� ���������  �(+)
*                     (m x 1) �� ��������;
*                sp - �������������  ��������  ����������� ����
*                     S(+) (m x m) ��  ��������, ���������� � ����� �������;
*                f  - ������������� ���������� ������� �(m x m),
*                     ���������� � ����� �������.
*
*   �������� ���������:
*                                              _
*                  xm - �������������  ������  �(-) (m x 1)  �����
*                       ��������;
*                  sm - ������������� ��������  �����������  ����
*                       S(-)  (m x m) �����  ��������, ���������� � ����� �������.
*
*   ��������� ����������:
*
*                  w  - ���������� (m x 1) ������.
*
*   ��������!!!
*   ��������!!!   ���  ��������   sm   �   sp   ��������� �������
*   ��������!!!   ������ �� ����������������� �������.
*   ��������!!!
*
*/

void dg0b(double *xp,double *sp,double *f,
          double *xm,double *sm,int m)
  {
   int i,j,k,ij,im,jm,m1;
   double c,y,y1,dy;
   double *w = new double [m];
     for(i=0;i < m;i++)  w[i] = 0.0;
     m1=m+1;
     for(i=0;i < m;i++)
       {
        c=0.0;
        im=i*m;
          for(j=0;j < m;j++)
            {
             y=0.0;
             jm=j*m;
               for(k=0;k <= i;k++) y+=*(sp+k*m+i)**(f+jm+k);
             ij=im+j;
             c+=*(f+ij)**(xp+j);
             *(sm+ij)=y;
            }
         *(xm+i)=c;
       }
         for(k=m-1;k > 0;k--)
           {
            y=0.0;

              for(i=0;i < m;i++) w[i] = 0.0;

              for(i=0;i < m;i++)
                {
                 y1=*(sm+i*m+k);
                 y+=y1*y1;
                }
             y=sqrt(y);
             dy=1.0/y;
               for(j=0;j < k;j++)
                 {
                  y1=0.0;
                    for(i=0;i < m;i++)
                      {
                       im=i*m;
                       y1+=*(sm+im+j)**(sm+im+k);
                      }
                  y1*=dy;
                  *(w+j)=y1;
                  c=y1*dy;
             for(i=0;i < m;i++)
               {
                im=i*m;
                *(sm+im+j)-=(*(sm+im+k))*c;
               }
                }
                for(i=0;i < k;i++) *(sm+i*m+k)=*(w+i);
                   *(sm+m1*k)=y;
                  }
          c=0.0;
             for(i=0;i < m;i++)
               {
                y=*(sm+i*m);
                c+=y*y;
               }
           *sm=sqrt(c);
delete [] w;
   }

/*
*
*_________________________________________________________________
*
*                 ***********
*                 *  dgq0b  *
*                 ***********
*
*   dgq0b(xp,sp,f,sq,xm,sm,m,w) - ������������         ����������
*                                 ��������  �������   �������  ��
*                                 ����� ��������.
*                                 (������   ����    ��������������
*                                 �����������).
*                                 ������ ��������������� ���������
*                                 ����������             ���������
*                                 ���������������  ������-������.
*
*
*   ��������!!!
*   ��������!!!   � ������ ������ ��� �������  q=B*u ������������.
*   ��������!!!
*
*   ������� ���������:
*
*                m  - ����������� ������� (������� ���������);
*                xp - ������������� ������ ������� ���������  �(+)
*                     (m x 1) �� ��������;
*                sp - �������������  ��������  ����������� ����
*                     S(+) (m x m) ��  ��������, ���������� � ����� �������;
*                f  - ������������� ���������� ������� �(m x m), ���������� � ����� �������.
*                sq - �������������    �����   sq(m x mq)     ��
*                     ��������������   �������  Q(m x m)    ����
*                     ������� q(j)=B(j)*u(j), ���������� � ����� �������:
*                                                       T
*                       sq(j) = B(j)*Sq(j) : sq(j)*sq(j) = Q(j).
*
*   �������� ���������:
*
*                  xm - �������������  ������  �(-) (m x 1)  �����
*                       ��������;
*                  sm - ������������� ��������  �����������  ����
*                       S(-)  (m x m) �����  ��������, ���������� � ����� �������.
*
*
*   ��������� ����������:
*
*                  w  - ���������� (m x 1) ������.
*
*   ��������!!!
*   ��������!!!   ���  ��������   sm   �   sp   ��������� �������
*   ��������!!!   ������ �� ����������������� �������.
*   ��������!!!
*
*   ��������!!!
*   ��������!!!   �  ��������   ������    ������������  ������  sq
*   ��������!!!   ��������.
*   ��������!!!
*
*/
void dgq0b(double *xp,double *sp,double *f,double *sq,
           double *xm,double *sm,int m,int mq)
{
		int i,j,k,ij,im,jq,kq,m1;
		double c,y,y1,dy;
		double *w = new double [m];
		for(i=0;i < m;i++)  
			w[i] = 0.0;
		m1=m+1;
		for(i=0;i < m;i++)
		{
			c=0.0;
			im=i*m;
			for(j=0;j < m;j++)
			{
				y=0.0;
				jq=j*m;
				for(k=0;k <= i;k++) 
					y+=*(sp+k*m+i)**(f+jq+k);
				ij=im+j;
				c+=*(f+ij)**(xp+j);
				*(sm+ij)=y;
			}
			*(xm+i)=c;
		}
		for(k=m-1;k > 0;k--)
		{
			y=0.0;
			kq=k*mq;

			for(i=0;i < m;i++) 
				w[i] = 0.0;

			for(i=0;i < m;i++)
			{
				y1=*(sm+i*m+k);
				y+=y1*y1;
			}
			for(i=0;i < mq;i++)
			{
				y1=*(sq+kq+i);
				y+=y1*y1;
			}
			y=sqrt(y);
			dy=1.0/y;
			for(j=0;j < k;j++)
			{
				y1=0.0;
				jq=j*mq;
				for(i=0;i < m;i++)
				{
					im=i*m;
					y1+=*(sm+im+j)**(sm+im+k);
				}
				for(i=0;i < mq;i++) 
					y1+=*(sq+jq+i)**(sq+kq+i);
				y1*=dy;
				*(w+j)=y1;
				c=y1*dy;
				for(i=0;i < m;i++)
				{
					im=i*m;
					*(sm+im+j)-=(*(sm+im+k))*c;
				}
				for(i=0;i < mq;i++) 
					*(sq+jq+i)-=(*(sq+kq+i))*c;
			}
			for(i=0;i < k;i++) 
				*(sm+i*m+k)=*(w+i);
			*(sm+m1*k)=y;
		}
		c=0.0;
		for(i=0;i < m;i++)
		{
			y=*(sm+i*m);
			c+=y*y;
		}
		for(i=0;i < mq;i++)
		{
			y=*(sq+i);
			c+=y*y;
		}
		*sm=sqrt(c);
		delete [] w;
}
/*
*
*_________________________________________________________________
*
*                 **********
*                 *  func  *
*                 **********
*
*   func(dt,a,f,m) - ������������ ���������� ���������� �������  f
*                    (m x m)  ��������  ������������   �������  ��
*                    �������� �������   a(m x m)   ���������������
*                    ����������� �������.
*                    (������������   ���������    �����    �������
*                    �������).
*
*
*                    f = E + (dt * a) + 0.5 ( dt * dt * a * a)
*
*                    (E - ��������� �������)
*
*
*   ������� ���������:
*
*                 m  - ����������� ������  a  �  f;
*                 dt - ����������� ������� �������;
*                 a  - ���������   ������  (m x m)   -     �������
*                      ����������� ������ �������, ���������� � ����� �������.
*
*   �������� ���������:
*
*                 f  - ���������   ������   (m x m)  -  ���������
*                      ������� �������, ���������� � ����� �������.
*
*/

void func(double dt,double *a,double *f,int m)
  {
   int i,j,k,ij,im;
   double c,y;
          c=0.5*dt*dt;
          for(i=0;i < m;i++)
           {
            im=i*m;
            for(j=0;j < m;j++)
              {
               y=0.0;
                 for(k=0;k < m;k++) y+=*(a+im+k)**(a+k*m+j);
               ij=im+j;
               y=dt**(a+ij)+c*y;
               if(i == j) y=1.0+y;
               *(f+ij)=y;
              }
            }
  }