#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

dWorldID world;             // 動力学計算用ワールド
dSpaceID space;             // 衝突検出用スペース
dGeomID  ground;            // 地面
dsFunctions fn;             // ODE関数
dMass mass;                 // 物体質量
dMatrix3 R;                 // 回転行列

typedef struct {          // Object構造体
  dBodyID body;           // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;           // ジオメトリのID番号(衝突検出計算用）
  dJointID joint;         // 関節
  dReal l = 0;           // 長さ[m], 半径[m], 質量[kg]
  dReal r = 0;
  dReal m = 0;
  dReal lx[3] = {};      // Boxの長さ(x,y,z)
  int longDir = 3;        // 円柱長手方向
  dReal color[4] = {};   // オブジェクトの色(r,g,b->0-1)
  dReal cx[3] = {};      // 中心座標
  dReal ax[4] = {};      // 中心座標での回転軸ベクトル+回転確度[rad]
  dReal jcx[3] = {};     // 関節中心
  dReal jax[3] = {};     // 関節回転軸ベクトル（Hinge）
  dReal ujax[2][3] = {}; // 関節回転軸ベクトル（Universal）
  dReal q = 0.0;       // 関節角度
  dReal qdot = 0.0;    // 関節角速度
  dReal q_d = 0.0;     // 目標角度
  dReal acc[3] = {};   // オブジェクトの加速度情報
  dReal fmax = 0.0;      // 出力上限[Nm]
  dReal output = 0.0;    // 出力（トルクの場合も、角速度の場合もある→DAの電圧値）
} Object;

#define NO_WEIGHT 0.0001  // 重量なしの場合の値
#define WORDL_OFFSET 0.2  // オフセット
#define ARM_NUM 4         // アームのパーツ点数
#define MOTOR_NUM 3       // モータケース点数
#define LINK_NUM 7        // リンクを構成するすべてのパーツ点数（1+3+3）
#define BASE_NUM 4        // ベースを形成する要素

static Object arm[ARM_NUM];
static Object motor[MOTOR_NUM];
static Object link[LINK_NUM];
static Object ball;

static void generateObject(Object* obj, const char* type) {
  obj -> body = dBodyCreate(world);
  dBodySetPosition(obj->body, obj->cx[0], obj->cx[1], obj->cx[2]);
  dMassSetZero(&mass);

  if(!strcmp(type,"Cylinder")){
    dMassSetCylinderTotal(&mass, obj->m, obj->longDir, obj->r, obj->l);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateCylinder(space, obj->r, obj->l);

  }else if(!strcmp(type,"Box")){
    dMassSetBoxTotal(&mass, obj->m, obj->lx[0], obj->lx[1], obj->lx[2]);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateBox(space, obj->lx[0], obj->lx[1], obj->lx[2]);

  }else if(!strcmp(type,"Capsule")){
    dMassSetCapsuleTotal(&mass, obj->m, obj->longDir, obj->r, obj->l);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateCapsule(space, obj->r, obj->l);

  }else if(!strcmp(type,"Sphere")){
    dMassSetSphereTotal(&mass, obj->m, obj->r);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateSphere(space, obj->r);

  }else{
    printf("No type Error @ generateObject\n");
  }
  dGeomSetBody(obj->geom, obj->body);
  dRFromAxisAndAngle(R, obj->ax[0], obj->ax[1], obj->ax[2], obj->ax[3]);
  dBodySetRotation(obj->body, R);
}
static void generateHingeJoint(Object* obj, Object* tar) {
  obj->joint = dJointCreateHinge(world, 0);
  if(tar != 0){
    dJointAttach(obj->joint, obj->body, tar->body);
  }else{
    dJointAttach(obj->joint, obj->body, 0);
  }
  dJointSetHingeAnchor(obj->joint, obj->jcx[0], obj->jcx[1], obj->jcx[2]);
  dJointSetHingeAxis(obj->joint, obj->jax[0], obj->jax[1], obj->jax[2]);
}
static void generateUniversalJoint(Object* obj, Object* tar) {
  obj->joint = dJointCreateUniversal(world, 0);
  dJointAttach(obj->joint, obj->body, tar->body);
  dJointSetUniversalAnchor(obj->joint, obj->jcx[0], obj->jcx[1], obj->jcx[2]);
  dJointSetUniversalAxis1(obj->joint, obj->ujax[0][0], obj->ujax[0][1], obj->ujax[0][2]);
  dJointSetUniversalAxis2(obj->joint, obj->ujax[1][0], obj->ujax[1][1], obj->ujax[1][2]);
}
static void generateFixJoint(Object* obj, Object* tar) {
  obj->joint = dJointCreateFixed(world, 0);
  if(tar != 0){
    dJointAttach(obj->joint, obj->body, tar->body);
  }else{
    dJointAttach(obj->joint, obj->body, 0);
  }
  dJointSetFixed(obj->joint);
}
static void drawObject(Object* obj, const char* type) {
  const dReal *pos, *rot;
  dReal side[3];

  pos = dBodyGetPosition(obj->body);
  rot = dBodyGetRotation(obj->body);
  dsSetColorAlpha(obj->color[0], obj->color[1], obj->color[2], obj->color[3]);
  if(!strcmp(type,"Cylinder")){
    dsDrawCylinder(pos, rot, obj->l, obj->r);
  }else if(!strcmp(type,"Box")){
    for(int i=0; i<3; i++){
      side[i] = obj->lx[i];
    }
    dsDrawBox(pos, rot, side);
  }else if(!strcmp(type,"Capsule")){
    dsDrawCapsule(pos, rot, obj->l, obj->r);
  }else if(!strcmp(type,"Sphere")){
    dsDrawSphereD(pos, rot, obj->r);
  }else{
    printf("No type Error\n");
  }
  dsSetColorAlpha(0.0,0.0,0.0,1.0); //色情報をリセット
}
double setPIto360(double rad){
  double id_pi = fmod(rad,(2.0*M_PI));
  double deg = 0.0;
  deg = id_pi * 180.0 / M_PI;
  if(signbit(rad)){
    // 不感帯をつくる→値の丸め込み
    //deg = 360.0 - deg;
  }
  return deg;
}
static void setParamColor(Object* obj, double r, double g, double b, double alpha) {
  obj->color[0] = r;
  obj->color[1] = g;
  obj->color[2] = b;
  obj->color[3] = alpha;
}
static void setParamCenter(Object* obj, double x, double y, double z) {
  obj->cx[0] = x;
  obj->cx[1] = y;
  obj->cx[2] = z;
}
static void setParamAxis(Object* obj, double ax, double ay, double az, double axis_rad) {
  obj->ax[0] = ax;
  obj->ax[1] = ay;
  obj->ax[2] = az;
  obj->ax[3] = axis_rad;
}
static void setParamCylinder(Object* obj, double m, double l, int longDir, double r) {
  obj->l = l;
  obj->r = r;
  obj->longDir = longDir;
  obj->m = m;
}
static void setParamBox(Object* obj, double m, double lx, double ly, double lz) {
  obj->lx[0] = lx;
  obj->lx[1] = ly;
  obj->lx[2] = lz;
  obj->m = m;
}
static void setParamSphere(Object* obj, double m, double r) {
  obj->r = r;
  obj->m = m;
}
static void setParamCapsule(Object* obj, double m, double l, double r) {
  obj->l = l;
  obj->r = r;
  obj->m = m;
}
static void setParamJointCenter(Object* obj, double jx, double jy, double jz) {
  obj->jcx[0] = jx;
  obj->jcx[1] = jy;
  obj->jcx[2] = jz;
}
static void setParamJointHingeAxis(Object* obj, double jax, double jay, double jaz) {
  obj->jax[0] = jax;
  obj->jax[1] = jay;
  obj->jax[2] = jaz;
}
static void setParamJointUniversalAxis(Object* obj, double jax1, double jay1, double jaz1, double jax2, double jay2, double jaz2) {
  obj->ujax[0][0] = jax1;
  obj->ujax[0][1] = jay1;
  obj->ujax[0][2] = jaz1;
  obj->ujax[1][0] = jax2;
  obj->ujax[1][1] = jay2;
  obj->ujax[1][2] = jaz2;
}
static void setParamActuator(Object* obj, double fmax) {
  obj->fmax = fmax;
}

// エンコーダ値取得
static void getHingeAngleInfo(Object* obj) {
  obj->q = dJointGetHingeAngle(obj->joint);
  obj->qdot = dJointGetHingeAngleRate(obj->joint);
}

// モータ制御
static void setHingeOutput(Object* obj, const char* type) {
  if(!strcmp(type,"torque")){
    dJointAddHingeTorque(obj->joint, obj->output);
  }else if(!strcmp(type,"speed")){
    dJointSetHingeParam(obj->joint,dParamVel, obj->output);
    dJointSetHingeParam(obj->joint,dParamFMax, obj->fmax);
  }
}

// 関節制御
static void setHingeAngle360(Object* obj, double q_d){
  double q = dJointGetHingeAngle(obj->joint);
  double q_desire = setPIto360(q_d);
  q = setPIto360(q);
  double f = 10.0 * (q_desire - q);
  dJointSetHingeParam(obj->joint,dParamVel, f);
  dJointSetHingeParam(obj->joint,dParamFMax, 100.0);
}

static void setUniversalAngle(Object* obj, double q1_d, double q2_d){
  double q1 = dJointGetUniversalAngle1(obj->joint);
  double q2 = dJointGetUniversalAngle2(obj->joint);
  double f1 = 30.0 * (q1_d - q1);
  double f2 = 30.0 * (q2_d - q2);
  dJointSetUniversalParam(obj->joint,dParamVel, f1);
  dJointSetUniversalParam(obj->joint,dParamVel2, f2);
  dJointSetUniversalParam(obj->joint,dParamFMax, 100.0);
  dJointSetUniversalParam(obj->joint,dParamFMax2, 100.0);
}

static void createArm() {
  dReal m = NO_WEIGHT;
  dReal l = 0.04;
  dReal r = 0.011;
  dReal cx[ARM_NUM][3] = {
    { 0.00, 0.00, 0.02 }, { 0.00, 0.00, 0.084 }, { 0.00, 0.00, 0.151 }, { 0.00, 0.00, 0.199 } };
  dReal ax[ARM_NUM][4] = {
    { 0.0, 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, M_PI/2.0 }, { 1.0, 0.0, 0.0, M_PI/2.0 }, { 0.0, 0.0, 0.0, 0.0 } };
  dReal color[ARM_NUM][4] = {
    { 0.00, 0.00, 0.00, 1.0 }, { 0.00, 0.00, 0.00, 1.0 }, { 0.00, 0.00, 0.00, 1.0 }, { 0.00, 0.00, 0.00, 1.0 } };
  dReal jcx[ARM_NUM][3] =  {
    { 0.00, 0.00, 0.02 }, { 0.00, 0.00, 0.084 }, { 0.00, 0.00, 0.151 }, { 0.00, 0.00, 0.199 } };
  dReal jax[ARM_NUM][3] =  {
    { 0.00, 0.00, 1.00 }, { 0.00, 1.00, 0.00 }, { 0.00, 1.00, 0.00 }, { 0.00, 0.00, 1.00 } };
  for(int i=0; i<ARM_NUM; i++){
    setParamCylinder(&arm[i], m, l, 3, r);
    setParamCenter(&arm[i], cx[i][0], cx[i][1], cx[i][2]+WORDL_OFFSET);
    setParamAxis(&arm[i], ax[i][0], ax[i][1], ax[i][2], ax[i][3]);
    setParamColor(&arm[i], color[i][0], color[i][1], color[i][2], color[i][3]);
    setParamJointCenter(&arm[i], jcx[i][0], jcx[i][1], jcx[i][2]+WORDL_OFFSET);
    setParamJointHingeAxis(&arm[i], jax[i][0], jax[i][1], jax[i][2]);
    generateObject(&arm[i], "Cylinder");
    if(i==0){
      generateHingeJoint(&arm[i], 0);
    }else if(i==3){
      generateFixJoint(&arm[i], &arm[i-1]);
    }else{
      generateHingeJoint(&arm[i], &arm[i-1]);
    }
  }
}

static void createMotor() {
  dReal m = 0.0546;
  dReal size[3] = { 0.024, 0.032, 0.050 };   // D: 0.024 W: 0.032 H: 0.050
  dReal cx[MOTOR_NUM][3] = {
    { 0.00, 0.014, 0.02 }, { 0.00, 0.00, 0.07 }, { 0.00, 0.00, 0.137 } };
  dReal ax[MOTOR_NUM][4] = {
    { 1.0, 0.0, 0.0, -M_PI/2.0 }, { 1.0, 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, 0.0 } };
  dReal color[MOTOR_NUM][4] = {
    { 0.20, 0.20, 0.20, 1.0 }, { 0.20, 0.20, 0.20, 1.0 }, { 0.20, 0.20, 0.20, 1.0 } };
  dReal jcx[MOTOR_NUM][3] =  {
    { 0.00, 0.014, 0.02 }, { 0.00, 0.00, 0.07 }, { 0.00, 0.00, 0.137 } };
  dReal jax[MOTOR_NUM][3] =  {
    { 0.00, 0.00, 1.00 }, { 0.00, 0.00, 1.00 }, { 0.00, 0.00, 1.00 } };
  for(int i=0; i<MOTOR_NUM; i++){
    setParamBox(&motor[i], m, size[0], size[1], size[2]);
    setParamCenter(&motor[i], cx[i][0], cx[i][1], cx[i][2]+WORDL_OFFSET);
    setParamAxis(&motor[i], ax[i][0], ax[i][1], ax[i][2], ax[i][3]);
    setParamColor(&motor[i], color[i][0], color[i][1], color[i][2], color[i][3]);
    setParamJointCenter(&motor[i], jcx[i][0], jcx[i][1], jcx[i][2]+WORDL_OFFSET);
    setParamJointHingeAxis(&motor[i], jax[i][0], jax[i][1], jax[i][2]);
    generateObject(&motor[i], "Box");
    if(i==0){
      generateFixJoint(&motor[i], 0);
    }else{
      generateFixJoint(&motor[i], &arm[i-1]);
    }
  }
}

static void createLink() {
  int belong[LINK_NUM] = { 0, 1, 1, 1, 2, 2, 2 };   // モータ軸に接続する
  dReal m = 0.005;
  dReal size[LINK_NUM][3] = {
    { 0.024, 0.034, 0.005 }, { 0.024, 0.003, 0.028 }, { 0.024, 0.040, 0.005 }, { 0.024, 0.003, 0.028 },
    { 0.024, 0.003, 0.028 }, { 0.024, 0.040, 0.005 }, { 0.024, 0.003, 0.028 }  };
  dReal cx[LINK_NUM][3] = {
    { 0.000, 0.00, 0.0425 }, { 0.00,-0.0215, 0.098 }, { 0.000, 0.00, 0.1095 }, { 0.00, 0.0215, 0.098 },
    { 0.00,-0.0215, 0.165 }, { 0.000, 0.00, 0.1765 }, { 0.00, 0.0215, 0.165 }  };
  dReal ax[LINK_NUM][4] = {
    { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 }  };
  dReal color[4] = { 0.80, 0.80, 0.80, 1.0 };
  dReal jcx[LINK_NUM][3] = {
    { 0.000, 0.00, 0.0425 }, { 0.00,-0.0215, 0.098 }, { 0.000, 0.00, 0.1095 }, { 0.00, 0.0215, 0.098 },
    { 0.00,-0.0215, 0.165 }, { 0.000, 0.00, 0.1765 }, { 0.00, 0.0215, 0.165 }  };
  dReal jax[LINK_NUM][3] =  {
    { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 },
    { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }  };
  for(int i=0; i<LINK_NUM; i++){
    setParamBox(&link[i], m, size[i][0], size[i][1], size[i][2]);
    setParamCenter(&link[i], cx[i][0], cx[i][1], cx[i][2]+WORDL_OFFSET);
    setParamAxis(&link[i], ax[i][0], ax[i][1], ax[i][2], ax[i][3]);
    setParamColor(&link[i], color[0], color[1], color[2], color[3]);   // colorはリンクで共通化するため色分けなし
    setParamJointCenter(&link[i], jcx[i][0], jcx[i][1], jcx[i][2]+WORDL_OFFSET);
    setParamJointHingeAxis(&link[i], jax[i][0], jax[i][1], jax[i][2]);
    generateObject(&link[i], "Box");
    generateFixJoint(&link[i], &arm[belong[i]]);
  }
}

static void createBase() {
  dReal m = 0.0546;
  dReal size[3] = { 0.024, 0.032, 0.050 };   // D: 0.024 W: 0.032 H: 0.050
  dReal cx[MOTOR_NUM][3] = {
    { 0.00, 0.014, 0.02 }, { 0.00, 0.00, 0.07 }, { 0.00, 0.00, 0.137 } };
  dReal ax[MOTOR_NUM][4] = {
    { 1.0, 0.0, 0.0, -M_PI/2.0 }, { 1.0, 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, 0.0 } };
  dReal color[MOTOR_NUM][4] = {
    { 0.20, 0.20, 0.20, 1.0 }, { 0.20, 0.20, 0.20, 1.0 }, { 0.20, 0.20, 0.20, 1.0 } };
  dReal jcx[MOTOR_NUM][3] =  {
    { 0.00, 0.014, 0.02 }, { 0.00, 0.00, 0.07 }, { 0.00, 0.00, 0.137 } };
  dReal jax[MOTOR_NUM][3] =  {
    { 0.00, 0.00, 1.00 }, { 0.00, 0.00, 1.00 }, { 0.00, 0.00, 1.00 } };
  for(int i=0; i<MOTOR_NUM; i++){
    setParamBox(&motor[i], m, size[0], size[1], size[2]);
    setParamCenter(&motor[i], cx[i][0], cx[i][1], cx[i][2]+WORDL_OFFSET);
    setParamAxis(&motor[i], ax[i][0], ax[i][1], ax[i][2], ax[i][3]);
    setParamColor(&motor[i], color[i][0], color[i][1], color[i][2], color[i][3]);
    setParamJointCenter(&motor[i], jcx[i][0], jcx[i][1], jcx[i][2]+WORDL_OFFSET);
    setParamJointHingeAxis(&motor[i], jax[i][0], jax[i][1], jax[i][2]);
    generateObject(&motor[i], "Box");
    generateFixJoint(&motor[i], 0);
    generateFixJoint(&motor[i], &arm[i-1]);
  }
}

static void createBall() {
  dReal m = NO_WEIGHT;
  dReal r = 0.011;
  dReal cx[3] = { 0.0, 0.0, 0.219 };
  dReal ax[4] = { 1.0, 0.0, 0.0, 0.0 };
  dReal color[4] = { 0.0, 0.90, 0.0, 1.0 };
  dReal jcx[3] =  { 0.0, 0.0, 0.219 };
  dReal jax[3] =  { 0.00, 0.00, 1.0 };
  setParamSphere(&ball, m, r);
  setParamCenter(&ball, cx[0], cx[1], cx[2]+WORDL_OFFSET);
  setParamAxis(&ball, ax[0], ax[1], ax[2], ax[3]);
  setParamColor(&ball, color[0], color[1], color[2], color[3]);
  setParamJointCenter(&ball, jcx[0], jcx[1], jcx[2]+WORDL_OFFSET);
  setParamJointHingeAxis(&ball, jax[0], jax[1], jax[2]);
  generateObject(&ball, "Sphere");
  generateFixJoint(&ball, 0);
}



// ロボット構成
static void create() {
  createArm();
  createMotor();
  createLink();
  createBase();
  createBall();
}


// ロボット描画
static void draw() {
  for(int i=0; i<ARM_NUM; i++) {
    drawObject(&arm[i], "Cylinder");
  }
  for(int i=0; i<MOTOR_NUM; i++) {
    drawObject(&motor[i], "Box");
  }
  for(int i=0; i<LINK_NUM; i++) {
    drawObject(&link[i], "Box");
  }
  drawObject(&ball, "Sphere");
}

// ロボット制御
static void control() {

}


void command(int cmd) {
  switch(cmd){
  case '0':
    break;
  default:
    break;
  }
}


// シミュレーションループ関数
static void simLoop(int pause) {
  control();
  dWorldStep(world, 0.001);
  draw();
}

// 視点・視線の設定
static void setView(float x,float y,float z,float h,float p,float r) {
  static float xyz[3] = {x,y,z};
  static float hpr[3] = {h,p,r};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

static void start() {
  setView(0.3,0.3,0.3,-135.0,-10.0,0.0);
}

// 描画関数の設定
void setDrawStuff() {
  fn.version = DS_VERSION;          // ドロースタッフのバージョン
  fn.start   = &start;              // 前処理 start関数のポインタ
  fn.step    = &simLoop;            // simLoop関数のポインタ
  fn.command = &command;            // command関数
  fn.path_to_textures = "textures"; // テクスチャ
}

int main (int argc, char *argv[]) {
  dInitODE();
  setDrawStuff();
  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);

  dWorldSetGravity(world, 0.0, 0.0, -9.8);
  dWorldSetERP(world,1.0);          // ERPの設定
  dWorldSetCFM(world,0.0);          // CFMの設定
  ground = dCreatePlane(space,0,0,1,0);
  create();
  dsSimulationLoop(argc,argv,500,500,&fn);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}