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
Coord CP_A[9] = {{2.947, 2.544 ,0.081, 0.03},{5.304, 1.004, 0.081, 0.03},{-4.476, 0.304, 0.081, 0.03},{-0.006, 1.362, 0.081, 0.02},
{2.939, 1.363, 0.081, 0.03},{0.559, -0.777, 0.081, 0.03},{1.899, -2.677, 0.02, 0.081},{3.799, -3.377, 0.03, 0.081},{2.759, -4.077, 0.081, 0.03}};
Coord CP_B[9] = {{-1.0187, -2.743, 0.081, 0.03},{1.5413, -1.848, 0.081, 0.02},{5.302, -1.848, 0.081, 0.03},{-4.478, -0.048, 0.081, 0.03},
{-2.198, -1.948, 0.081, 0.03},{-0.618, -0.408, 0.081, 0.03},{-0.668, 3.429, 0.081, 0.03},{-2.538, 4.329, 0.03, 0.081},{0.792, 4.329, 0.081, 0.03}};


/*CHECKLINE VA LINETYPE A*/
//checkline A
Coord CL_A[44] = {{4.1913, 3.704,0.081,0.03},//0
{2.247, 3.704,0.081,0.03},//1
{2.247, 2.544, 0.03, 0.081},//2
{2.947, 2.544 ,0.081, 0.03},//3
{2.947, 1.964 ,0.03, 0.081},//4
{3.264, 1.964, 0.081, 0.03},//5
{3.264, 1.644, 0.03, 0.081},//6
{3.574, 1.644, 0.081, 0.03},//7
{3.764, 1.464, 0.03, 0.081},//8
{3.764, 1.134, 0.03, 0.081},//9
{3.934, 1.004, 0.081, 0.03},//10
{5.304, 1.004, 0.081, 0.03},//11
{5.304, 0.304, 0.03, 0.081},//12
{-4.476, 0.304, 0.081, 0.03},//13
{-4.476, 1.004, 0.03, 0.081},//14
{-4.116, 1.362, 0.081, 0.03},//15
{-0.006, 1.362, 0.081, 0.03},//16
{0.014, 1.562, 0.081, 0.35},//17
{0.804, 1.562, 0.081, 0.35},//18
{0.824, 1.762, 0.081, 0.03},//19
{1.804, 1.762, 0.081, 0.03},//20
{1.816, 1.562, 0.081, 0.35},//21
{2.599, 1.563, 0.081, 0.35},//22
{2.609, 1.363, 0.081, 0.03},//23
{2.939, 1.363, 0.081, 0.03},//24
{2.939, 0.663, 0.081, 0.03},//25
{1.089, 0.663, 0.081, 0.03},//26
{1.089, -0.437, 0.081, 0.03},//27
{1.109, -0.777, 0.081, 0.03},//28
{0.559, -0.777, 0.081, 0.03},//29
{0.199, -1.137, 0.081, 0.03},//30
{0.199, -3.117, 0.03, 0.081},//31
{1.899, -3.117, 0.03, 0.081},//32
{1.899, -2.677, 0.03, 0.081},//33
{2.549, -2.677, 0.081, 0.03},//34
{3.259, -2.677, 0.03, 0.081},//35
{3.799, -2.677, 0.081, 0.03},//36
{3.799, -3.377, 0.03, 0.081},//37
{4.169, -3.377, 0.081, 0.03},//38
{4.169, -4.077, 0.081, 0.03},//39
{3.809, -4.077, 0.081, 0.03},//40
{3.049, -4.077, 0.081, 0.03},//41
{2.759, -4.077, 0.081, 0.03},//42
{2.449, -4.077, 0.081, 0.03},//43
};
//Line type A
int LineTypeA[43] = {0,0,0,0,0,0,0,1,0,2,0,0,
                     0,0,3,0,0,-1,0,0,0,-1,0,0,
                     4,0,5,6,0,7,0,8,0,0,9,0,
                     0,0,10,0,11,0,0};
/*CHECKLINE VA LINETYPE B*/
//Check line B
Coord CL_B[43] = {{0.8913, -4.446, 0.081, 0.03},//0
{-1.3087, -4.443, 0.081, 0.03},//1
{-1.3087, -2.743, 0.081, 0.03},//2
{-1.0187, -2.743, 0.081, 0.03},//3
{-0.6587, -2.393, 0.081, 0.03},//4
{-0.6587, -1.848, 0.03, 0.081},//5
{1.5413, -1.848, 0.081, 0.03},//6
{1.5513, -1.648, 0.081, 0.35},//7
{2.337, -1.648, 0.081, 0.35},//8
{2.347, -1.448, 0.081, 0.03},//9
{3.347, -1.448, 0.081, 0.03},//10
{3.357, -1.648, 0.081, 0.35},//11
{4.137, -1.648, 0.081, 0.35},//12
{4.147, -1.848, 0.081, 0.03},//13
{5.302, -1.848, 0.081, 0.03},//14
{5.302, -0.048, 0.03, 0.081},//15
{-4.478, -0.048, 0.081, 0.03},//16
{-4.478, -0.698, 0.03, 0.081},//17
{-4.478, -1.408, 0.081, 0.03},//18
{-4.478, -1.948, 0.03, 0.081},//19
{-2.198, -1.948, 0.081, 0.03},//20
{-2.198, -1.368, 0.03, 0.081},//21
{-1.878, -1.368, 0.081, 0.03},//22
{-1.878, -1.048, 0.03, 0.081},//23
{-1.558, -1.048, 0.081, 0.03},//24
{-1.378, -0.898, 0.03, 0.081},//25
{-1.378, -0.588, 0.03, 0.081},//26
{-1.208, -0.408, 0.081, 0.03},//27
{-0.618, -0.408, 0.081, 0.03},//28
{-0.678, 0.692, 0.081, 0.03},//29
{-1.038, 0.692, 0.081, 0.03},//30
{-1.038, 1.692, 0.03, 0.081},//31
{-0.688, 1.692, 0.081, 0.03},//32
{-0.698, 2.032, 0.081, 0.03},//33
{-0.698, 2.732, 0.081, 0.03},//34
{-0.668, 3.429, 0.081, 0.03},//35
{-2.178, 3.429, 0.081, 0.03},//36
{-2.538, 3.779, 0.081, 0.03},//37
{-2.538, 4.329, 0.03, 0.081},//38
{-0.268, 4.329, 0.081, 0.03},//39
{0.492, 4.329, 0.03, 0.081},//40
{0.792, 4.329, 0.081, 0.03},//41
{1.092, 4.329, 0.081, 0.03}//42
};
//Line type B
int LineTypeB[42] = {0,1,0,2,0,0,0,-1,0,0,0,
                    -1,0,0,0,0,0,3,0,0,0,0,0,
                     0,4,0,5,0,6,0,0,0,7,8,9,
                     0,10,0,0,11,0,0};

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
      return CurveLine(L, R, 3.55, 1.43, 0.065, 0.38);
    case 2: 
      return CurveLine(L, R, 3.98, 1.22, 0.065, 0.38);
    case 3:
      return CurveLine(L, R, -4.475, 1.363, 0.2, 0.5);
    case 4: 
      return CurveLine(L, R, 2.915, 1.013, 0.2, 0.5);
    case 5: 
      return CurveLine(L, R, 1.1, 0.113, 0.399, 0.7);
    case 6:
      return CurveLine(L, R, 1.085, -0.607, 0.02, 0.34);
    case 7:
      return CurveLine(L, R, 0.201, -0.778, 0.2, 0.5);
    case 8:
      return CurveLine(L, R, 1.051, -3.028, 0.7, 1.01);
    case 9:
      return CurveLine(L, R, 2.901, -2.678, 0.2, 0.5);
    case 10:
      return CurveLine(L, R, 4.151, -3.728, 0.2, 0.5);
    case 11: 
      return CurveLine(L, R, 3.404, -4.078, 0.2, 0.5);
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
      return CurveLine(L, R, -1.295, -3.592, 0.7, 1.01);
    case 2: 
      return CurveLine(L, R, -0.659, -2.742, 0.2, 0.5);
    case 3:
      return CurveLine(L, R, -4.479, -1.049, 0.2, 0.5);
    case 4: 
      return CurveLine(L, R, -1.573, -0.849, 0.05, 0.36);
    case 5: 
      return CurveLine(L, R, -1.183, -0.609, 0.05, 0.36);
    case 6:
      return CurveLine(L, R, -0.694, 0.141, 0.4, 0.7);
    case 7:
      return CurveLine(L, R, -0.695, 1.863, 0.023, 0.322);
    case 8:
      return CurveLine(L, R, -0.695, 2.383, 0.2, 0.5);
    case 9: 
      return CurveLine(L, R, -0.691, 3.081, 0.2, 0.5);
    case 10: 
      return CurveLine(L, R, -2.536, 3.431, 0.2, 0.5);
    case 11: 
      return CurveLine(L, R, 0.144, 4.331, 0.2, 0.5);
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
  const string argument[12] = {"1_ClassicCar_1 3_NHH_STEM_Rhyder", "3_NHH_STEM_Rhyder 1_ClassicCar_1", "3_NH_SmurfCat_2 4_NH_Team_Ki_Niem_1", "4_NH_Team_Ki_Niem_1 3_NH_SmurfCat_2","cam1","cam2","cam3","cam4","cam5","cam6","cam7","cam8"};
  /*INSTANCE CHO ROBOT A VA ROBOT B*/
  
  Coord LPoint0A, RPoint0A, MPoint0A, LPoint0B, RPoint0B, MPoint0B;
  double startTimeA = 0.0;//KHAI BAO THOI GIAN BAT DAU SIMULATION
  double startTimeB = 0.0;
  /*INDEX CHO CHECKLINE VA CHECKPOINT A, B*/
  int CLine_A = 1;
  int CLine_B = 1;
  int CPoint_A = 0;
  int CPoint_B = 0;
  int isFinishA = 0;
  int isFinishB = 0;
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
  Node *cam2 = supervisor->getFromDef("camera2");
  //Get translation of car and solid
  Field *R_trans_field_B = R_B->getField("translation");
  Field *L_trans_field_B = L_B->getField("translation");
  Field *M_trans_field_B = M_B->getField("translation");
  Field *Car_trans_field_B = Car_B->getField("translation");
  Field *CarControllerB = Car_B->getField("controller");
  //Get rotation of Car
  Field *Car_rot_field_B = Car_B->getField("rotation");
  const double HomeA[3] = {4.29567, 0.0160194, 3.70168};
  const double HomeB[3] = {0.995301, 0.0139667, -4.44321};

  /*ROTATION AND TRANSLATION OF CAMERA*/
  Field *CamTrans1 = cam1->getField("translation");
  Field *CamRots1 = cam1->getField("rotation");
  double CamTranslation1[3] = {0, 0, 0};
  double CamRotation1[4] = {0, 0, 0, 0};
  
  Field *CamTrans2 = cam2->getField("translation");
  Field *CamRots2 = cam2->getField("rotation");
  double CamTranslation2[3] = {0, 0, 0};
  double CamRotation2[4] = {0, 0, 0, 0};

  while (supervisor->step(TIME_STEP) != -1) {
    supervisor->wwiSendText("T|" + to_string(supervisor->getTime())); //GUI TIN HIEU BAT DAU TIN THOI GIAN CHO PLUGIN
    string a = supervisor->wwiReceiveText();
    cout<<a<<endl;
    for(int i = 0; i<12; i++){
      if(a == argument[0])
      {
        CarControllerA->setSFString("1_ClassicCar_1");
        CarControllerB->setSFString("3_NHH_STEM_Rhyder");
      }
      if(a == argument[1])
      {
        CarControllerA->setSFString("3_NHH_STEM_Rhyder");
        CarControllerB->setSFString("1_ClassicCar_1");
        break;
      }
      if(a == argument[2])
      {
        CarControllerA->setSFString("3_NH_SmurfCat_2");
        CarControllerB->setSFString("4_NH_Team_Ki_Niem_1");
        break;
      }
      if(a == argument[3])
      {
        CarControllerA->setSFString("4_NH_Team_Ki_Niem_1");
        CarControllerB->setSFString("3_NH_SmurfCat_2");
        break;
      }
     if(a == argument[4])
      {
        CamTranslation1[0] = 4.29;
        CamTranslation1[1] =3.36;
        CamTranslation1[2] = 2.18;
        CamRotation1[0] = 1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = -1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[5])
      {
        CamTranslation1[0] = -0.214694;
        CamTranslation1[1] = 3.24936;
        CamTranslation1[2] = 0.430006;
        CamRotation1[0] = -0.447212;
        CamRotation1[1] = 0.774597;
        CamRotation1[2] = 0.447215;
        CamRotation1[3] = 1.82348;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[6])
      {
        CamTranslation1[0] = 1.22531;
        CamTranslation1[1] = 3.21936;
        CamTranslation1[2] = 1.28001;
        CamRotation1[0] = 1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] =-1.5708;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[7])
      {
        CamTranslation1[0] = 1.40531;
        CamTranslation1[1] = 3.22144;
        CamTranslation1[2] = -0.103655;
        CamRotation1[0] = -1;
        CamRotation1[1] = 0;
        CamRotation1[2] = 0;
        CamRotation1[3] = 1.0472;
        CamTrans1->setSFVec3f(CamTranslation1);
        CamRots1->setSFRotation(CamRotation1);
      }
      if(a == argument[8])
      {
        CamTranslation2[0] = -0.08;
        CamTranslation2[1] = 3.24;
        CamTranslation2[2] = -3.03;
        CamRotation2[0] = 1;
        CamRotation2[1] = 0;
        CamRotation2[2] = 0;
        CamRotation2[3] = -1.5708;
        CamTrans2->setSFVec3f(CamTranslation2);
        CamRots2->setSFRotation(CamRotation2);
      }
      if(a == argument[9])
      {
        CamTranslation2[0] = -1.48085;
        CamTranslation2[1] = 3.88368;
        CamTranslation2[2] = -0.220006;
        CamRotation2[0] = 0.519992;
        CamRotation2[1] = -0.677662;
        CamRotation2[2] = -0.519983;
        CamRotation2[3] = -1.95045;
        CamTrans2->setSFVec3f(CamTranslation2);
        CamRots2->setSFRotation(CamRotation2);
      }
      if(a == argument[10])
      {
        CamTranslation2[0] = -0.870851;
        CamTranslation2[1] = 3.17368;
        CamTranslation2[2] = 3.31999;
        CamRotation2[0] = -6.14069e-07;
        CamRotation2[1] = 0.707112;
        CamRotation2[2] = 0.707102;
        CamRotation2[3] = -3.14159;
        CamTrans2->setSFVec3f(CamTranslation2);
        CamRots2->setSFRotation(CamRotation2);
      }
      if(a == argument[11])
      {
        CamTranslation2[0] = 1.00854;
        CamTranslation2[1] = 1.288;
        CamTranslation2[2] = 5.20474;
        CamRotation2[0] = -0.678605;
        CamRotation2[1] = 0.678592;
        CamRotation2[2] = 0.281085;
        CamRotation2[3] = 1.09608;
        CamTrans2->setSFVec3f(CamTranslation2);
        CamRots2->setSFRotation(CamRotation2);
      }
   

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
       cout<<CLine_B<<" "<<LineTypeB[CLine_B]<<endl;
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
      if(CPoint_A<8)
      {
        supervisor->wwiSendText("C.A." + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint_A++;
      }
    }
    /*KIEM TRA CHECKPOINT B*/
    if (ForwardLine(MPointB, CP_B[CPoint_B]))
    {
      if(CPoint_B<8)
      {
        supervisor->wwiSendText("C.B." + to_string(CPoint_B+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint_B++;
      }
    }

    /*DUNG 5 GIAY XE A*/
    if(CLine_A >= 43)
    {          
      Coord xeduaA;
      xeduaA.x = 0;
      xeduaA.z = 0.04;
      Coord Car_coordA = Rotation(Car_trans_A,xeduaA,Car_rot_A);
      if(ForwardLine(Car_coordA, CL_A[42]))
      {
        if(CPoint_A == 8)
        {
          if(startTimeA == 0.0)
          {
            startTimeA = supervisor->getTime();
          }
          cout<<startTimeA<<endl;
          if (supervisor->getTime() - startTimeA >= 05.00) {
            if(isFinishA == 0)
            {
              supervisor->wwiSendText("C.A.0" + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
              CarControllerA->setSFString("forceStop");
              isFinishA++; 
            } 
          }
          if (CLine_A == 45)
          {
            supervisor->wwiSendText("O.A|" + to_string(supervisor->getTime()));
            CarControllerA->setSFString("forceStop"); 
            Car_trans_field_A->setSFVec3f(HomeA);
          }
        }       
      }
    }

    /*DUNG 5 GIAY XE B*/
    if(CLine_B >= 42)
       {          
          Coord xeduaB;
          xeduaB.x = 0;
          xeduaB.z = 0.04;
          Coord Car_coordB = Rotation(Car_trans_B,xeduaB,Car_rot_B);
          if(ForwardLine(Car_coordB, CL_B[41]))
          {
            if(CPoint_B == 8)
            {
               if(startTimeB == 0.0)
              {
                startTimeB = supervisor->getTime();
              }
              if (supervisor->getTime() - startTimeB >= 05.00) {
                if(isFinishB == 0)
                {
                  supervisor->wwiSendText("C.B.0" + to_string(CPoint_B+1) + "|" + to_string(supervisor->getTime()));
                  CarControllerB->setSFString("forceStop"); 
                  isFinishB++;
                }
              }
              if (CLine_B == 44)
              {
                supervisor->wwiSendText("O.B|" + to_string(supervisor->getTime()));
                CarControllerB->setSFString("forceStop");
                Car_trans_field_B->setSFVec3f(HomeB); 
              }
            }
            
          }
   
        }
    if(CLine_A == 11)
    {
      CamTranslation1[0] = 5.54012;
      CamTranslation1[1] = 2.73853;
      CamTranslation1[2] = 0.33;
      CamRotation1[0] = -0.447213;
      CamRotation1[1] = 0.774596;
      CamRotation1[2] = 0.447214;
      CamRotation1[3] = 1.82348;
      CamTrans1->setSFVec3f(CamTranslation1);
      CamRots1->setSFRotation(CamRotation1);
    }
    if(CLine_A == 26)
    {
      CamTranslation1[0] = 0.82;
      CamTranslation1[1] = 0.433029;
      CamTranslation1[2] = -1.4972;
      CamRotation1[0] = 6.60461e-07;
      CamRotation1[1] = 0.965926;
      CamRotation1[2] = 0.25882;
      CamRotation1[3] = 3.14159;
      CamTrans1->setSFVec3f(CamTranslation1);
      CamRots1->setSFRotation(CamRotation1);
   }
    if(CLine_A == 34)
    {
      CamTranslation1[0] = 3.48531;
      CamTranslation1[1] = 2.18217;
      CamTranslation1[2] = -4.12193;
      CamRotation1[0] = 1.71501e-07;
      CamRotation1[1] = 0.793353;
      CamRotation1[2] = 0.608762;
      CamRotation1[3] = 3.14159;
      CamTrans1->setSFVec3f(CamTranslation1);
      CamRots1->setSFRotation(CamRotation1);
    }
   
  
    if(CLine_B == 7)
    {
      CamTranslation2[0] = 3.24;
      CamTranslation2[1] = 3.34;
      CamTranslation2[2] = -1.49;
      CamRotation2[0] = 1;
      CamRotation2[1] = 0;
      CamRotation2[2] = 0;
      CamRotation2[3] = -1.5708;
      CamTrans2->setSFVec3f(CamTranslation2);
      CamRots2->setSFRotation(CamRotation2);
    }
    if(CLine_B == 28)
    {
      CamTranslation2[0] = -0.273771;
      CamTranslation2[1] = 0.591986;
      CamTranslation2[2] = -1.65222;
      CamRotation2[0] = 7.84271e-07;
      CamRotation2[1] = 0.991445;
      CamRotation2[2] = 0.130527;
      CamRotation2[3] = 3.14159;
      CamTrans2->setSFVec3f(CamTranslation2);
      CamRots2->setSFRotation(CamRotation2);
    }
    if(CLine_B == 49)
    {
      CamTranslation2[0] = 4.9787;
      CamTranslation2[1] = 1.88675;
      CamTranslation2[2] = -0.444567;
      CamRotation2[0] = 0.517725;
      CamRotation2[1] = -0.67637;
      CamRotation2[2] = -0.523912;
      CamRotation2[3] = -1.94572;
      CamTrans2->setSFVec3f(CamTranslation2);
      CamRots2->setSFRotation(CamRotation2);
    }
    if(CLine_B == 50)
    {
      CamTranslation2[0] = 4.9787;
      CamTranslation2[1] = 1.77057;
      CamTranslation2[2] = -1.81064;
      CamRotation2[0] = 1.12382e-06;
      CamRotation2[1] = -0.922126;
      CamRotation2[2] = -0.386889;
      CamRotation2[3] = 3.14159;
      CamTrans2->setSFVec3f(CamTranslation2);
      CamRots2->setSFRotation(CamRotation2);
    }
  }
  delete supervisor;
  return 0;
}
