#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Camera.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define TIME_STEP 32
using namespace webots;
using namespace std;

//Toa do
typedef struct 
{
  double x,z,dx,dz;
}Coord;

/*CHECKPOINT A VA CHECKPOINT B*/
Coord CP_A[10] = {{2.08, -3.25 ,0.081, 0.02},{1.98, 0.921, 0.081, 0.02},{1.58, 3.7, 0.02, 0.081},{4.14, 4.14, 0.081, 0.02},
{4.16, 1.71, 0.02, 0.081},{5.15, 1.31, 0.081, 0.02},{-5.03, 0.3, 0.081, 0.02},{-1.64, 4.42, 0.02, 0.081},
{0.657, 4.4, 0.081, 0.02},{-0.361, 2.44, 0.02, 0.081}};
Coord CP_B[10] = {{-4.49, 4.1, 0.081, 0.02},{-4.09, 1.7, 0.02, 0.081},{-0.91, 1.7, 0.081, 0.03},{1.82, 4.14, 0.081, 0.03},
{-4.68, -2.61, 0.081, 0.02},{-1.21, -3.75, 0.081, 0.02},{1.69, -2.49, 0.081, 0.03},{3.83, -2.49, 0.081, 0.02},
{5.15, -0.052, 0.02, 0.081},{-5.03, -0.882, 0.02, 0.081}};

/*CHECKLINE VA LINETYPE A*/
//checkline A
Coord CL_A[54] = {{4.79, -4.5722,0.081,0.02},//0
{3.24, -4.5722,0.081,0.03},//1
{3.26, -3.9122, 0.081, 0.03},//2
{3.31, -3.25 ,0.081, 0.03},//3
{2.08, -3.25 ,0.081, 0.02},//4
{2.082, -1.9011, 0.02, 0.081},//5
{2.28, -1.9, 0.081, 0.02},//6
{2.28, -1.42, 0.02, 0.081},//7
{1.86, -1.42, 0.081, 0.03},//8
{1.882, -1.0411, 0.081, 0.03},//9
{1.88, -0.661, 0.081, 0.03},//10
{1.23, -0.659, 0.081, 0.03},//11
{1.25, 0.921, 0.081, 0.03},//12
{1.98, 0.921, 0.081, 0.02},//13
{1.98, 1.37, 0.02, 0.081},//14
{1.78, 1.39, 0.35, 0.081},//15
{1.78, 2.33, 0.35, 0.081},//16
{1.58, 2.34, 0.02, 0.081},//17
{1.58, 3.89, 0.03, 0.081},//18
{1.82, 4.14, 0.081, 0.03},//19
{2.88, 4.14, 0.081, 0.03},//20
{3.63, 4.14, 0.081, 0.03},//21
{4.14, 4.14, 0.081, 0.02},//22
{4.14, 3.23, 0.02, 0.081},//23
{4.34, 3.03, 0.03, 0.081},//24
{4.13, 2.82, 0.03, 0.081},//25
{4.33, 2.6, 0.03, 0.081},//26
{4.13, 2.39, 0.03, 0.081},//27
{4.35, 2.17, 0.03, 0.081},//28
{4.16, 1.97, 0.03, 0.081},//29
{4.16, 1.71, 0.02, 0.081},//30
{3.31, 1.71, 0.081, 0.02},//31
{3.31, 1.3, 0.02, 0.081},//32
{5.15, 1.31, 0.081, 0.02},//33
{5.15, 0.3, 0.02, 0.081},//34
{-5.03, 0.3, 0.081, 0.02},//35
{-5.03, 0.85, 0.02, 0.081},//36
{-1.64, 0.85, 0.081, 0.02},//37
{-1.64, 2.35, 0.03, 0.081},//38
{-1.64, 3.09, 0.03, 0.081},//39
{-1.64, 4.42, 0.02, 0.081},//40
{-1.36, 4.42, 0.081, 0.02},//41
{-1.16, 4.61, 0.081, 0.03},//42
{-0.928, 4.39, 0.081, 0.03},//43
{-0.718, 4.6, 0.081, 0.03},//44
{-0.508, 4.4, 0.081, 0.03},//45
{-0.298, 4.6, 0.081, 0.03},//46
{-0.0876, 4.4, 0.081, 0.02},//47
{0.657, 4.4, 0.081, 0.02},//48
{0.661, 3.17, 0.03, 0.081},//49
{0.409, 2.92, 0.081, 0.03},//50
{-0.361, 2.92, 0.081, 0.02},//51
{-0.361, 2.44, 0.02, 0.081},//52
{-0.361, 2.14, 0.02, 0.081},//53
};
//Line type A
int LineTypeA[53] = {0,1,2,0,0,0,0,0,3,4,0,5,
                     0,0,0,-1,0,0,6,0,7,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,0,0,
                     0,0,8,0,0,0,0,0,0,0,0,0,
                     0,9,0,0,0};
/*CHECKLINE VA LINETYPE B*/
//Check line B
Coord CL_B[53] = {{-4.38, 4.72, 0.081, 0.02},//0
{-2.49, 4.72, 0.081, 0.02},//1
{-2.49, 4.1, 0.02, 0.081},//2
{-3.27, 4.1, 0.081, 0.02},//3
{-3.27, 3.43, 0.02, 0.081},//4
{-3.67, 3.43, 0.081, 0.02},//5
{-3.67, 4.1, 0.02, 0.081},//6
{-4.49, 4.1, 0.081, 0.02},//7
{-4.49, 3.71, 0.02, 0.081},//8
{-4.29, 3.7, 0.35, 0.081},//9
{-4.29, 2.75, 0.35, 0.081},//10
{-4.09, 2.74, 0.02, 0.081},//11
{-4.09, 1.7, 0.02, 0.081},//12
{-3.43, 1.7, 0.081, 0.03},//13
{-2.66, 1.7, 0.03, 0.081},//14
{-0.91, 1.7, 0.081, 0.03},//15
{-0.92, 1.32, 0.081, 0.03},//16
{-0.92, 0.94, 0.081, 0.03},//17
{-0.92, -0.64, 0.081, 0.03},//18
{-1.69, -0.64, 0.081, 0.03},//19
{-1.69, -1.4, 0.02, 0.081},//20
{-3.12, -1.4, 0.081, 0.03},//21
{-3.36, -1.64, 0.03, 0.081},//22
{-3.36, -2.37, 0.03, 0.081},//23
{-3.6, -2.61, 0.081, 0.03},//24
{-4.68, -2.61, 0.081, 0.02},//25
{-4.68, -3.75, 0.02, 0.081},//26
{-3.56, -3.75, 0.081, 0.03},//27
{-2.84, -3.75, 0.081, 0.03},//28
{-1.21, -3.75, 0.081, 0.02},//29
{-1.02, -3.95, 0.081, 0.03},//30
{-0.796, -3.73, 0.081, 0.03},//31
{-0.576, -3.93, 0.081, 0.03},//32
{-0.366, -3.73, 0.081, 0.03},//33
{-0.156, -3.93, 0.02, 0.081},//34
{0.044, -3.73, 0.081, 0.03},//35
{0.238, -3.73, 0.081, 0.02},//36
{0.238, -2.47, 0.02, 0.081},//37
{0.424, -2.47, 0.081, 0.02},//38
{0.634, -2.672, 0.081, 0.03},//39
{0.844, -2.47, 0.081, 0.03},//40
{1.05, -2.67, 0.081, 0.03},//41
{1.27, -2.47, 0.081, 0.03},//42
{1.49, -2.68, 0.081, 0.03},//43
{1.69, -2.49, 0.081, 0.03},//44
{3.83, -2.49, 0.081, 0.02},//45
{3.83, -0.992, 0.03, 0.081},//46
{4.49, -1.01, 0.03, 0.081},//47
{5.15, -1.01, 0.03, 0.081},//48
{5.15, -0.052, 0.02, 0.081},//49
{-5.03, -0.052, 0.081, 0.02},//50
{-5.03, -0.882, 0.02, 0.081},//51
{-5.03, -1.18, 0.02, 0.081},//52
};
//Line type B
int LineTypeB[52] = {0,0,0,0,0,0,0,0,0,-1,0,0,
                     0,1,0,2,3,4,0,0,0,5,0,6,
                     0,0,0,7,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,8,9,
                     0,0,0,0};

/*KIEM TRA CHECKLINE VA CHECKPOINT*/
bool ForwardLine(Coord trans, Coord point)
{ 
  
  if(trans.x>=(point.x-point.dx) && trans.x<=(point.x+point.dx) && trans.z>=(point.z-point.dz) && trans.z<=(point.z+point.dz))
  {
    return true;
  }else return false;
}

/*GOC QUAY CUA DAU XE*/
Coord Rotation(const double *trans, Coord coord, const double *rotat)
{
  Coord Result;
  if(rotat[1]<0){
      Result.x =  trans[0]+ coord.x*cos(rotat[3]) - coord.z*sin(rotat[3]);
      Result.z =  trans[2]+ coord.x*sin(rotat[3]) + coord.z*cos(rotat[3]);
    }
    else
    {
      Result.x =  trans[0]+ coord.x*cos(-rotat[3]) - coord.z*sin(-rotat[3]);
      Result.z =  trans[2]+ coord.x*sin(-rotat[3]) + coord.z*cos(-rotat[3]);
    }
  return Result;
}

/*KIEM TRA OUTLINE TREN DUONG THANG A*/
bool StraightLineA(int index, Coord L, Coord R, double d)
{
  double a = CL_A[index-1].z - CL_A[index].z;
  double b = CL_A[index].x - CL_A[index-1].x;
  double c = -CL_A[index-1].x*a - CL_A[index-1].z*b;
  if((abs(a*L.x+b*L.z+c))/sqrt(a*a + b*b) > d || (abs(a*R.x+b*R.z+c)/sqrt(a*a+b*b)>d))
  {
    return true;
  }else return false;
}
/*KIEM TRA OUTLINE TREN DUONG THANG B*/
bool StraightLineB(int index, Coord L, Coord R, double d)
{
  double a = CL_B[index-1].z - CL_B[index].z;
  double b = CL_B[index].x - CL_B[index-1].x;
  double c = -CL_B[index-1].x*a - CL_B[index-1].z*b;
  if((abs(a*L.x+b*L.z+c))/sqrt(a*a + b*b) > d || (abs(a*R.x+b*R.z+c)/sqrt(a*a+b*b)>d))
  {
    return true;
  }else return false;
}

/*KIEM TRA OUTLINE O DUONG CONG*/
bool CurveLine(Coord L, Coord R, double I1, double I2, double R1, double R2)
{
  if((L.x-I1)*(L.x-I1) + (L.z-I2)*(L.z-I2) > R1*R1 
  && (L.x-I1)*(L.x-I1) + (L.z-I2)*(L.z-I2) < R2*R2 
  && (R.x-I1)*(R.x-I1) + (R.z-I2)*(R.z-I2) > R1*R1 
  && (R.x-I1)*(R.x-I1) + (R.z-I2)*(R.z-I2) < R2*R2)  
  {  
      return false;
  }
  else return true;
}

/*HAM CHECK OUTLINE A*/
bool CheckOutLineA(Coord L, Coord R, int numline)
{
  // cout<<LineTypeA[numline-1]<<endl;
  switch(LineTypeA[numline-1]){
    case -1:
      return StraightLineA(numline, L, R, 0.35);
    case 0:
      return StraightLineA(numline, L, R, 0.15);
    case 1: 
      return CurveLine(L, R, 3.29, -4.24, 0.18, 0.48);
    case 2: 
      return CurveLine(L, R, 3.27, -3.58, 0.18, 0.48);
    case 3:
      return CurveLine(L, R, 1.9, -1.23, 0.04, 0.36);
    case 4: 
      return CurveLine(L, R, 1.86, -0.85, 0.04, 0.36);
    case 5: 
      return CurveLine(L, R, 1.31, 0.13, 0.63, 0.94);
    case 6:
      return CurveLine(L, R, 1.58, 4.14, 0.09, 0.39);
    case 7:
      return CurveLine(L, R, 3.27, 4.14, 0.2, 0.5);
    case 8:
      return CurveLine(L, R, -1.64, 2.73, 0.2, 0.5);
    case 9:
      return CurveLine(L, R, 0.66, 2.92, 0.09, 0.39);
    default: return false;
    }
}

/*HAM CHECK OUTLINE B*/
bool CheckOutLineB(Coord L, Coord R, int numline)
{
  switch(LineTypeB[numline-1]){
    case -1:
      return StraightLineB(numline, L, R, 0.35);
    case 0:
      return StraightLineB(numline, L, R, 0.15);
    case 1: 
      return CurveLine(L, R, -3.01, 1.696, 0.2, 0.5);
    case 2: 
      return CurveLine(L, R, -0.95, 1.51, 0.04, 0.37);
    case 3:
      return CurveLine(L, R, -0.9, 1.14, 0.04, 0.36);
    case 4: 
      return CurveLine(L, R, -0.91, 0.15, 0.63, 0.94);
    case 5: 
      return CurveLine(L, R, -3.36, -1.4, 0.09, 0.39);
    case 6:
      return CurveLine(L, R, -3.36, -2.61, 0.09, 0.39);
    case 7:
      return CurveLine(L, R, -3.19, -3.75, 0.2, 0.5);
    case 8:
      return CurveLine(L, R, 4.16, -1.04, 0.18, 0.48);
    case 9: 
      return CurveLine(L, R, 4.82, -1.02, 0.18, 0.48);
    default: return false;

    }
}
int main(int argc, char **argv) {
  Supervisor *supervisor = new Supervisor();
  
  /*CAMERA*/
  Camera* cm1;
  cm1 = supervisor->getCamera("camera1");
  cm1->enable(TIME_STEP);
  Camera* cm2;
  cm2 = supervisor->getCamera("camera2");
  cm2->enable(TIME_STEP);
  const string argument[10] = {"c1", "c2", "c3", "c4", "c5", "c6","cam1","cam2","cam3","cam4"};
  /*INSTANCE CHO ROBOT A VA ROBOT B*/
  
  Coord LPoint0A, RPoint0A, MPoint0A, LPoint0B, RPoint0B, MPoint0B;
  double startTimeA = 0.0;//KHAI BAO THOI GIAN BAT DAU SIMULATION
  double startTimeB = 0.0;
  /*INDEX CHO CHECKLINE VA CHECKPOINT A, B*/
  int CLine_A = 1;
  int CLine_B = 1;
  int CPoint_A = 0;
  int CPoint_B = 0;

  /*LAY DATA TU CAC NODE CUA XE A*/
  Node *R_A = supervisor->getFromDef("R_A");
  Node *L_A = supervisor->getFromDef("L_A");
  Node *M_A = supervisor->getFromDef("M_A");
  Node *Car_A = supervisor->getFromDef("Speed_Line_Follower_Robot_V4_A");
  //Get translation of car and solid
  Field *R_trans_field_A = R_A->getField("translation");
  Field *L_trans_field_A = L_A->getField("translation");
  Field *M_trans_field_A = M_A->getField("translation");
  Field *Car_trans_field_A = Car_A->getField("translation");
  Field *CarControllerA = Car_A->getField("controller");
  //Get rotation of Car
  Field *Car_rot_field_A = Car_A->getField("rotation");

  /*LAY DATA TU CAC NODE CUA XE B*/
  Node *R_B = supervisor->getFromDef("R_B");
  Node *L_B = supervisor->getFromDef("L_B");
  Node *M_B = supervisor->getFromDef("M_B");
  Node *Car_B = supervisor->getFromDef("Speed_Line_Follower_Robot_V4_B");
  Node *cam1 = supervisor->getFromDef("camera1");
  //Get translation of car and solid
  Field *R_trans_field_B = R_B->getField("translation");
  Field *L_trans_field_B = L_B->getField("translation");
  Field *M_trans_field_B = M_B->getField("translation");
  Field *Car_trans_field_B = Car_B->getField("translation");
  Field *CarControllerB = Car_B->getField("controller");
  //Get rotation of Car
  Field *Car_rot_field_B = Car_B->getField("rotation");
  const double HomeA[3] = {1.09, 0.017, -5.2};
  const double HomeB[3] = {-2.85498, 0.0169847, 1.05892};

  /*ROTATION AND TRANSLATION OF CAMERA*/
  Field *CamTrans1 = cam1->getField("translation");
  Field *CamRots1 = cam1->getField("rotation");
  double CamTranslation1[3] = {0, 0, 0};
  double CamRotation1[4] = {0, 0, 0, 0};

  while (supervisor->step(TIME_STEP) != -1) {
    supervisor->wwiSendText("T|" + to_string(supervisor->getTime())); //GUI TIN HIEU BAT DAU TIN THOI GIAN CHO PLUGIN
    string a = supervisor->wwiReceiveText();
    cout<<a<<endl;
    for(int i = 0; i<10; i++){
     if(a == argument[6])
      {
        CamTranslation1[0] = 3.1;
        CamTranslation1[1] = 5.12;
        CamTranslation1[2] = -2.79;
        CamRotation1[0] = -1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = 1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[7])
      {
        CamTranslation1[0] = 3.1;
        CamTranslation1[1] = 3.91;
        CamTranslation1[2] = 2.7;
        CamRotation1[0] = -1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = 1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[8])
      {
        CamTranslation1[0] = -1.22;
        CamTranslation1[1] = 3.64;
        CamTranslation1[2] = -0.11;
        CamRotation1[0] = -1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = 1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[9])
      {
        CamTranslation1[0] = -0.88;
        CamTranslation1[1] = 3.49;
        CamTranslation1[2] = 3.21;
        CamRotation1[0] = -1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = 1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      // if(a == argument[0])
      // {
        // CarAController->setSFString("");
        // CarControllerB->setSFString("");
      // }
      // if(a == argument[1])
      // {
        // CarControllerA->setSFString("");
        // CarControllerB->setSFString("");
        // break;
      // }
      // if(a == argument[2])
      // {
        // CarControllerA->setSFString("");
        // CarControllerB->setSFString("");
        // break;
      // }
      // if(a == argument[3])
      // {
        // CarControllerA->setSFString("");
        // CarControllerB->setSFString("");
        // break;
      // }
    }

  /*CAMERA*/
    cm1->getImage();
    cm2->getImage();
  /*XU LY ROTATION DAU XE A*/
    const double *Car_rot_A = Car_rot_field_A->getSFRotation();
    const double *Car_trans_A = Car_trans_field_A->getSFVec3f();
    //On the R
    LPoint0A.x = -0.04;
    LPoint0A.z = -0.14;
    Coord LPointA = Rotation(Car_trans_A,LPoint0A,Car_rot_A);
    const double LTranslationA[3] = {LPointA.x, Car_trans_A[1], LPointA.z};
    L_trans_field_A->setSFVec3f(LTranslationA);
    //On the L
    RPoint0A.x = 0.04;
    RPoint0A.z = -0.14;
    Coord RPointA = Rotation(Car_trans_A,RPoint0A,Car_rot_A);
    const double RTranslationA[3] = {RPointA.x, Car_trans_A[1], RPointA.z};
    R_trans_field_A->setSFVec3f(RTranslationA);
    //In the M
    MPoint0A.x = 0;
    MPoint0A.z = -0.14;
    Coord MPointA = Rotation(Car_trans_A,MPoint0A,Car_rot_A);
    const double MTranslationA[3] = {MPointA.x, Car_trans_A[1], MPointA.z};
    M_trans_field_A->setSFVec3f(MTranslationA);

  /*XU LY ROTATION DAU XE B*/
    const double *Car_rot_B = Car_rot_field_B->getSFRotation();
    const double *Car_trans_B = Car_trans_field_B->getSFVec3f();
    //On the R
    LPoint0B.x = -0.04;
    LPoint0B.z = -0.14;
    Coord LPointB = Rotation(Car_trans_B,LPoint0B,Car_rot_B);
    const double LTranslationB[3] = {LPointB.x, Car_trans_B[1], LPointB.z};
    L_trans_field_B->setSFVec3f(LTranslationB);
    //On the L
    RPoint0B.x = 0.04;
    RPoint0B.z = -0.14;
    Coord RPointB = Rotation(Car_trans_B,RPoint0B,Car_rot_B);
    const double RTranslationB[3] = {RPointB.x, Car_trans_B[1], RPointB.z};
    R_trans_field_B->setSFVec3f(RTranslationB);
    //In the M
    MPoint0B.x = 0;
    MPoint0B.z = -0.14;
    Coord MPointB = Rotation(Car_trans_B,MPoint0B,Car_rot_B);
    const double MTranslationB[3] = {MPointB.x, Car_trans_B[1], MPointB.z};
    M_trans_field_B->setSFVec3f(MTranslationB);


    if(ForwardLine(MPointA, CL_A[CLine_A])){
       cout<<CLine_A<<endl;
       CLine_A++;
    }
    
    if(ForwardLine(MPointB, CL_B[CLine_B])){
       cout<<CLine_B<<endl;
       CLine_B++;
    }
    if (CheckOutLineA(LPointA, RPointA, CLine_A))
    { 
      supervisor->wwiSendText("O.A|" + to_string(supervisor->getTime()));
      //supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE); 
      CarControllerA->setSFString("forceStop"); 
      Car_trans_field_A->setSFVec3f(HomeA);
    }

    if (CheckOutLineB(LPointB, RPointB, CLine_B))
    { 
      supervisor->wwiSendText("O.B|" + to_string(supervisor->getTime()));
      //supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE); 
      CarControllerB->setSFString("forceStop");
      Car_trans_field_B->setSFVec3f(HomeB);  
    }
    
    /*KIEM TRA CHECKPOINT A*/
    if (ForwardLine(MPointA, CP_A[CPoint_A]))
    {
      if(CPoint_A<4)
      {
        supervisor->wwiSendText("C.A." + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint_A++;
      }
    }
    /*KIEM TRA CHECKPOINT B*/
    if (ForwardLine(MPointB, CP_B[CPoint_B]))
    {
      if(CPoint_B<4)
      {
        supervisor->wwiSendText("C.B." + to_string(CPoint_B+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint_B++;
      }
    }

    /*DUNG 5 GIAY XE A*/
    if(CLine_A >= 53)
    {          
      Coord xeduaA;
      xeduaA.x = 0;
      xeduaA.z = 0;
      Coord Car_coordA = Rotation(Car_trans_A,xeduaA,Car_rot_A);
      if(ForwardLine(Car_coordA, CL_A[52]))
      {
        if(CPoint_A == 4)
        {
          if(startTimeA == 0.0)
          {
            startTimeA = supervisor->getTime();
          }
          cout<<startTimeA;
          if (supervisor->getTime() - startTimeA >= 05.00) {
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            supervisor->wwiSendText("C.A.0" + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
            CarControllerA->setSFString("forceStop"); 
          }
          if (CLine_A == 54)
          {
            supervisor->wwiSendText("O.A|" + to_string(supervisor->getTime()));
            cout<<"\nOutline";
            CarControllerA->setSFString("forceStop"); 
            Car_trans_field_A->setSFVec3f(HomeA);
          }
        }       
      }
    }

    /*DUNG 5 GIAY XE B*/
    if(CLine_B >= 52)
    {          
      Coord xeduaB;
      xeduaB.x = 0;
      xeduaB.z = 0;
      Coord Car_coordB = Rotation(Car_trans_B,xeduaB,Car_rot_B);
      if(ForwardLine(Car_coordB, CL_B[51]))
      {
        if(CPoint_B == 4)
        {
          if(startTimeB == 0.0)
          {
            startTimeB = supervisor->getTime();
          }
          cout<<startTimeB;
          if (supervisor->getTime() - startTimeB >= 05.00) {
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            supervisor->wwiSendText("C.B.0" + to_string(CPoint_B+1) + "|" + to_string(supervisor->getTime()));
            CarControllerB->setSFString("forceStop"); 
          }
          if (CLine_B == 53)
          {
            supervisor->wwiSendText("O.B|" + to_string(supervisor->getTime()));
            cout<<"\nOutline";
            CarControllerB->setSFString("forceStop"); 
            Car_trans_field_B->setSFVec3f(HomeB);
          }
        }       
      }
    }
    // if(CLine_A == 1)
    // {
      // CamTranslation1[0] = 3.58573;
      // CamTranslation1[1] = 2.31174;
      // CamTranslation1[2] = -1.42357;
      // CamRotation1[0] = 0.965404;
      // CamRotation1[1] = -0.240458;
      // CamRotation1[2] = -0.100876;
      // CamRotation1[3] = -0.555965;
      // CamTrans1->setSFVec3f(CamTranslation1);
      // CamRots1->setSFRotation(CamRotation1);
    // }
  }
  delete supervisor;
  return 0;
}