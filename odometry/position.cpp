#include<iostream>
#include<iomanip>
#include<cstdlib>
#include<cmath>
#include<fstream>
#define _USE_MATH_DEFINES
using namespace std;

#define DEBUG false

template <class T>
void printMat(T **nMat, int nRow, int nCol)
{
  for(int i = 0; i < nRow; i++)
    {
      for(int j = 0; j < nCol; j++)
	cout<<nMat[i][j]<<"\t";
      cout<<endl;
    }
}

template <class T>
void printArray(T *nAr, int nSize)
{
  for(int i = 0; i < nSize; i++)
    cout<<nAr[i]<<" ";
  cout<<endl;
}

ifstream accIn, orientIn;

void parse(double *timeStep, double acc[3], double orient[3])
{
  char nGarbage;
  accIn>>(*timeStep)>>nGarbage>>acc[0]>>nGarbage>>acc[1]>>nGarbage>>acc[2]>>nGarbage;
  orientIn>>(*timeStep)>>nGarbage>>orient[0]>>nGarbage>>orient[1]>>nGarbage>>orient[2]>>nGarbage;  
}

void fillRotationMatrix(double rotMat[3][3], double orient[3])
{
  double alpha = orient[0]*M_PI/180;
  double gamma = orient[1]*M_PI/180;
  double beta = orient[2]*M_PI/180;
  rotMat[0][0] = cos(alpha)*cos(beta);
  rotMat[0][1] = cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma);
  rotMat[0][2] = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
  rotMat[1][0] = sin(alpha)*cos(beta);
  rotMat[1][1] = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
  rotMat[1][2] = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
  rotMat[2][0] = -1*sin(beta);
  rotMat[2][1] = cos(beta)*sin(gamma);
  rotMat[2][2] = cos(beta)*cos(gamma);  
}

double* transformAcc(double rotMat[3][3], double acc[3])
{
  double *newAcc = new double[3];
  for(int i = 0; i < 3; i++)
    {
      newAcc[i] = 0;
      for(int j = 0; j < 3; j++)
	newAcc[i] += rotMat[i][j]*acc[j];
    }
  return newAcc;
}

void calcPos(char *argv[])
{
  double timeStep, prevTimeStep = 0.0, acc[3], orient[3], rotMat[3][3];  
  double posX, posY, posZ, *newAcc, deltaT;
  posZ = posY = posZ = 0.0;  
  accIn.open(argv[1]);
  orientIn.open(argv[2]);  
  
  if(!accIn || !orientIn)
    {
      cout<<"File opening error\n";
      exit(0);
    }   
  cout<<setprecision(20)<<endl;  
  while(!accIn.eof())
    {
      parse(&timeStep, acc, orient);

      if(DEBUG){
	cout<<"Acceleration: "<<timeStep<<"\t"<<acc[0]<<"\t"<<acc[1]<<"\t"<<acc[2]<<endl;
	cout<<"Orientation: "<<timeStep<<"\t"<<orient[0]<<"\t"<<orient[1]<<"\t"<<orient[2]<<endl;
      }

      fillRotationMatrix(rotMat, orient);
      newAcc = transformAcc(rotMat, acc);

      if(DEBUG)
	cout<<"Transformed Acceleration: "<<newAcc[0]<<"\t"<<newAcc[1]<<"\t"<<newAcc[2]<<endl;

      deltaT = timeStep - prevTimeStep;
      deltaT *= deltaT;
      posX += 0.5*newAcc[0]*deltaT;
      posY += 0.5*newAcc[1]*deltaT;
      posZ += 0.5*newAcc[2]*deltaT;
      //cout<<"Current Position: "<<posX<<"\t"<<posY<<"\t"<<posZ<<endl;
      //cout<<"*****************\n";
      cout<<posX<<"\t"<<posY<<"\t"<<posZ<<endl;
      prevTimeStep = timeStep;
    }
  cout<<"Current Position: "<<posX<<"\t"<<posY<<"\t"<<posZ<<endl;
}

/*
int main(int argc, char *argv[])
{
  if(argc < 2)
    {
      cout<<"Arguments missing\n";
      cout<<"<Acceleration-file> <Orientation-file>\n";
      exit(0);
    }
  calcPos(argv);  
  return 0;
}
*/
