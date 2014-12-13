#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox      dsDrawBoxD
#endif
#define NUM 6              // 出現する物体の数

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject rlink[NUM];                   // リンク

/*                   |-0-| |-1-| |-2-| |-3-| |-4-| |-5-|*/
dReal l_x[NUM]    = { 0.20, 0.04, 0.04, 0.04, 0.00, 0.00 };  // 長さ x
dReal l_y[NUM]    = { 0.20, 0.04, 0.04, 0.04, 0.00, 0.00 };  // 長さ y
dReal l_z[NUM]    = { 0.10, 0.60, 0.30, 0.20, 0.00, 0.00 };  // 長さ z
dReal x[NUM]      = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.30 };  // 重心 x
dReal y[NUM]      = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.30 };  // 重心 y
dReal z[NUM]      = { 0.05, 0.40, 0.85, 1.10, 1.20, 0.30 };  // 重心 z

dReal length[NUM] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };  // 円柱長さ
dReal weight[NUM] = { 1.00, 4.00, 2.50, 0.50, 0.20, 0.10 };  // 質量
dReal r[NUM]      = { 0.00, 0.00, 0.00, 0.00, 0.03, 0.03 };  // 半径

dReal c_x[NUM]    = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };  // 関節中心点 x
dReal c_y[NUM]    = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };  // 関節中心点 y
dReal c_z[NUM]    = { 0.00, 1.00, 0.70, 1.00, 1.20, 0.00 };  // 関節中心点 z
dReal axis_x[NUM] = {    0,    0,    0,    0,    0,    0 };  // 関節回転軸 x
dReal axis_y[NUM] = {    0,    0,    1,    1,    0,    0 };  // 関節回転軸 y
dReal axis_z[NUM] = {    1,    1,    0,    0,    1,    1 };  // 関節回転軸 z

int flg_init = 0;
dReal q_desire[3] = { 0.0, 0.0, 0.0 };
dReal p_desire[3] = { 0.0, 0.0, 1.2 };


/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass; // 質量パラメータ

  // リンクの生成

  // 0:ベース-3:第3リンク
  for (int i = 0; i < 4; i++){
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,weight[i],l_x[i],l_y[i],l_z[i]);
    dBodySetMass(rlink[0].body, &mass);
    rlink[i].geom = dCreateBox(space,l_x[i],l_y[i],l_z[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }
  // 4:手先マーカ
    rlink[4].body = dBodyCreate(world);
    dBodySetPosition(rlink[4].body, x[4], y[4], z[4]);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass,weight[4],r[4]);
    dBodySetMass(rlink[4].body, &mass);
    rlink[4].geom = dCreateSphere(space,r[4]);
    dGeomSetBody(rlink[4].geom,rlink[4].body);
  // 5:ターゲットマーカ
    rlink[5].body = dBodyCreate(world);
    dBodySetPosition(rlink[5].body, x[5], y[5], z[5]);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass,weight[5],r[5]);
    dBodySetMass(rlink[5].body, &mass);
    rlink[5].geom = dCreateSphere(space,r[5]);
    dGeomSetBody(rlink[5].geom,rlink[5].body);


  // ジョイントの生成とリンクへの取り付け

  // 0:ベース
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  // 1:第1リンク-4:マーカ
  for (int j = 1; j < 5; j++){
    joint[j] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
  }
  // 5:ターゲットマーカ
  for (int j = 5; j < 6; j++){
    joint[j] = dJointCreateFixed(world, 0);  // 固定ジョイント
    dJointAttach(joint[j], rlink[j].body, 0);
    dJointSetFixed(joint[j]);
  }

}

/*** ロボットアームの描画 ***/
void drawArm()
{
  dReal r;
  dReal side[3];

  for (int i = 0; i < 4 ; i++){
    // 0:ベース-9:第6リンク
    side[0] = l_x[i]; side[1] = l_y[i]; side[2] = l_z[i];
    dsDrawBox(dGeomGetPosition(rlink[i].geom),dGeomGetRotation(rlink[i].geom),side);
  }
  // 4:手先マーカ
  dsSetColor(1.0, 0.0, 0.0);
  r = dGeomSphereGetRadius(rlink[4].geom);
  dsDrawSphereD(dBodyGetPosition(rlink[4].body),dBodyGetRotation(rlink[4].body),r);
  // 5:ターゲットマーカ
  dsSetColor(0.0, 0.0, 1.0);
  r = dGeomSphereGetRadius(rlink[5].geom);
  dsDrawSphereD(dBodyGetPosition(rlink[5].body),dBodyGetRotation(rlink[5].body),r);
}

/*** 動摩擦＆静止摩擦による外乱を再現 ***/
void disturbance( dReal DA_data[], dReal qdot[] )
{
  /* 動摩擦によるパワーロス */
  double Fric = 1.0;  // 動摩擦係数
  for(int i=0; i<3; i++){
    DA_data[i] = DA_data[i] - Fric*qdot[i]; 
  }

  /* 静止摩擦によるパワーロス&停止 */
  double sFric_qdot = 0.001;
  double sFric = 0.30;
  for(int i=0; i<3; i++){
    if(qdot[i]<sFric_qdot){
      if(DA_data[i]<sFric){
        DA_data[i] = 0.0;
      }
    }
  }
}

/*** ヤコビ行列の計算 ***/
void jacobian( dReal J[], dReal q[] )
{
  J[0] = -1*l_z[2]*sin(q[0])*sin(q[1]) - l_z[3]*sin(q[0])*sin(q[1]+q[2]);
  J[1] =    l_z[2]*cos(q[0])*cos(q[2]) - l_z[3]*cos(q[0])*cos(q[1]+q[2]);
  J[2] =                                 l_z[3]*cos(q[0])*cos(q[1]+q[2]);
  J[3] =    l_z[2]*cos(q[0])*sin(q[1]) + l_z[3]*cos(q[0])*sin(q[1]+q[2]);
  J[4] =    l_z[2]*sin(q[0])*cos(q[1]) + l_z[3]*sin(q[0])*cos(q[1]+q[2]);
  J[5] =                                 l_z[3]*sin(q[0])*cos(q[1]+q[2]);
  J[6] = 0.0;
  J[7] = -1*l_z[2]*sin(q[1]) - l_z[3]*sin(q[1]+q[2]);
  J[8] = -1*l_z[3]*sin(q[1]+q[2]);
}

/*** 制御 ***/
void control()
{
  /* 変数・ゲイン定義 */
  dReal KPe[3] = { 10.0, 10.0, 10.0 };
  dReal KVe[3] = { 2.0, 2.0, 2.0 };
  dReal KPc[3] = { 0.0, 0.0, 0.0 };

  dReal q[3]  = { 0.0 };
  dReal qdot[3] = { 0.0 };

  dReal J[9] = { 0.0 };
  dReal DA_data[3] = { 0.0 };


  /* 初期化 */
  if(flg_init=0){
    // 初期化シーケンスを記述する．
    flg_init++;
  }

  /* 関節角度・角速度計測 */
  for(int i = 0; i < 3 ; i++){
    q[i] = dJointGetHingeAngle(joint[i+1]);
    qdot[i] = dJointGetHingeAngleRate(joint[i+1]);
  }

  /* ヤコビ行列計算 */
  jacobian( J, q );

  /* 位置偏差（位置誤差）計測 */
  const dReal *p_M;
  p_M = dBodyGetPosition(rlink[4].body);
  dReal diff_x = p_desire[0] - p_M[0];
  dReal diff_y = p_desire[1] - p_M[1];
  dReal diff_z = p_desire[2] - p_M[2];

  /*[出力トルク] = [PD角度制御] + [転置ヤコビによる位置制御]  (好みによってゲインを設定して変更可)*/
  DA_data[0] = KPe[0]*(q_desire[0]-q[0])-KVe[0]*qdot[0] + KPc[0]*(J[0]*diff_x + J[3]*diff_y + J[6]*diff_z);
  DA_data[1] = KPe[1]*(q_desire[1]-q[1])-KVe[1]*qdot[1] + KPc[1]*(J[1]*diff_x + J[4]*diff_y + J[7]*diff_z);
  DA_data[2] = KPe[2]*(q_desire[2]-q[2])-KVe[2]*qdot[2] + KPc[2]*(J[2]*diff_x + J[5]*diff_y + J[8]*diff_z);

  disturbance(DA_data, qdot);

  /* トルク制御 */
/*  for (int i = 1; i < 4; i++){
    dJointAddHingeTorque( joint[i], DA_data[i-1] );
  }
*/  /* 関節角度制御 */
  for (int i = 1; i < 4; i++) {
    dReal fMax = 10.0;
    dJointSetHingeParam(joint[i],dParamVel, DA_data[i-1]);
    dJointSetHingeParam(joint[i],dParamFMax,fMax);
  }
  /* 不要な関節をロック */
  for (int i = 4; i < 5; i++) {
    dReal k =  10.0, fMax = 100.0;
    dReal tmp = dJointGetHingeAngle(joint[i]);
    dJointSetHingeParam(joint[i],dParamVel, -k*tmp);
    dJointSetHingeParam(joint[i],dParamFMax,fMax);
  }

  printf("%3.3lf,%3.3lf,%3.3lf¥r",q[0]/3.14159265*180.0,q[1]/3.14159265*180.0,q[2]/3.14159265*180.0);

  FILE *fp_0; fp_0 = fopen("data.csv","a");
  fprintf(fp_0,"%3.9lf,%3.9lf¥n", q[0],q[1],q[2]);
  fclose(fp_0);
}

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = { 1.2f, 0.0f, 0.8f };          // 視点[m]
  float hpr[3] = { 180.0f, 0.0f, 0.0f };        // 視線[°]
  dsSetViewpoint(xyz, hpr);                     // 視点と視線の設定
}

/*** キー入力関数 ***/
void command(int cmd)
{
  switch(cmd){
  case 'h':
    // 操作説明
    printf("Press key¥n");
    printf("1, q : joint[0] UP, DOWN¥n");
    printf("2, w : joint[1] UP, DOWN¥n");
    printf("3, e : joint[2] UP, DOWN¥n");
    printf("a, z : axis_X UP, DOWN¥n");
    printf("s, x : axis_Y UP, DOWN¥n");
    printf("d, c : axis_Z UP, DOWN¥n");
    printf("r    : all joint position RESET¥n¥n¥n");
    break;
  case '1':
    q_desire[0] = q_desire[0] + 3.14159265/180.0;
    break;
  case 'q':
    q_desire[0] = q_desire[0] - 3.14159265/180.0;
    break;
  case '2':
    q_desire[1] = q_desire[1] + 3.14159265/180.0;
    break;
  case 'w':
    q_desire[1] = q_desire[1] - 3.14159265/180.0;
    break;
  case '3':
    q_desire[2] = q_desire[2] + 3.14159265/180.0;
    break;
  case 'e':
    q_desire[2] = q_desire[2] - 3.14159265/180.0;
    break;
  case 'r':
    q_desire[0] = 0.0;
    q_desire[1] = 0.0;
    q_desire[2] = 0.0;
    p_desire[0] = 0.0;
    p_desire[1] = 0.0;
    p_desire[2] = 1.2;
    break;
  case 'a':
    p_desire[0] = p_desire[0] + 0.01;
    break;
  case 'z':
    p_desire[0] = p_desire[0] - 0.01;
    break;
  case 's':
    p_desire[1] = p_desire[1] + 0.01;
    break;
  case 'x':
    p_desire[1] = p_desire[1] - 0.01;
    break;
  case 'd':
    p_desire[2] = p_desire[2] + 0.01;
    break;
  case 'c':
    p_desire[2] = p_desire[2] - 0.01;
    break;
  default:
    break;
  }

}


/*** シミュレーションループ ***/
void simLoop(int pause)
{

  control();                                  // P制御
  dWorldStep(world, 0.01);
  drawArm();                                   // ロボットの描画
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command;                       // command関数
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char **argv)
{
  dInitODE(); // ODEの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, 0.0);            // 重力の設定

  makeArm();                                      // アームの生成
  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ

  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE();
  return 0; // ODEの終了

}
