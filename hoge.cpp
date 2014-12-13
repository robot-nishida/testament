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

#define BASE_NUM  4       // ベース / スマホ
#define MOTOR_NUM 2       // サーボユニット（ケース/軸）
#define LINK_NUM  5       // リンク
#define TAP_NUM   2       // 先端ユニット

#define SERVO_NUM 3       // サーボの個数

static Object base[BASE_NUM];
static Object motor[SERVO_NUM][MOTOR_NUM];
static Object link[SERVO_NUM][LINK_NUM];
static Object tap[TAP_NUM];

#define SP_H 0.160
#define SP_W 0.080
#define SP_T 0.008

#define NO_M 0.0001

dReal base_size[3] = { 0.08, 0.1, 0.02 };
dReal sp_size[3] = { SP_W, SP_H, SP_T };
dReal sp_screen_size[3] = { 0.9*SP_W, 0.7*SP_H, SP_T };
dReal sp_button_r = 0.7*SP_H;
dReal sp_button_l = SP_T;
dReal base_center[BASE_NUM][3] = {
  { 0.04, 0.0, 0.01 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }
};
dReal base_color[BASE_NUM][4] = {
  { 1.0, 1.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 0.5, 0.5, 0.5, 1.0 }
};
dReal base_axis[BASE_NUM][4] = {
  { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }
};

dReal motor_size[3] = { 0.032, 0.040, 0.050 };
dReal motor_m = 0.054;
dReal motor_l = 0.048;
dReal motor_r = 0.030;
dReal motor_center[SERVO_NUM][MOTOR_NUM][3] = {
  { { 0.0, 0.0, 0.025 }, { 0.0, 0.0, 0.044 } },
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } },
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } }
};
dReal motor_color[MOTOR_NUM][4] = {
  { 1.0, 1.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0, 1.0 }
};
dReal motor_axis[MOTOR_NUM][4] = {
  { 1.0, 1.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0, 1.0 }
};


dReal link_size[3][3] = {{ 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }};
dReal link_m = 0.005;
dReal link_center[SERVO_NUM][LINK_NUM][3] = {
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } },
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } },
  { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } }
};
dReal link_color[LINK_NUM][4] = {
  { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }
}
dReal link_axis[LINK_NUM][4] = {
  { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }
}

dReal tap_m = 0.020;
dReal tap_l = 0.040;
dReal tap_r = 0.008;
dReal tap_center[TAP_NUM][3] = {
  { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }
}
dReal tap_color[TAP_NUM][4] = {
  { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }
}
dReal tap_axis[TAP_NUM][4] = {
  { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }
}

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



// ロボット構成
static void create() {
  /* ベース/環境 */
  for(int i=0; i<BASE_NUM; i++){
    switch(i){
      case '0':
        setParamBox(&base[i], NO_M, base_size[0], base_size[1], base_size[2]);  break;
      case '1':
        setParamBox(&base[i], NO_M, sp_size[0], sp_size[1], sp_size[2]);  break;
      case '2':
        setParamBox(&base[i], NO_M, sp_screen_size[0], sp_screen_size[1], sp_screen_size[2]);  break;
      case '3':
        setParamCylinder(&base[i], NO_M, sp_button_l, 3, sp_button_r);  break;
      default:  break;
    }
    setParamCenter(&base[i], base[i][0], base[i][1], base[i][2]);
    setParamColor(&base[i], base_color[i][0], base_color[i][1], base_color[i][2], base_color[i][3]);
    setParamAxis(&base[i], base_axis[i][0], base_axis[i][1], base_axis[i][2], base_axis[i][3]);
    if(i==3){
      generateObject(&base[i], "Cylinder");
    }else{
      generateObject(&base[i], "Box");
    }
  }

  /* モータ */
  for(int i=0; i<SERVO_NUM; i++){
    setParamBox(&motor[i][0], motor_m, motor_size[0], motor_size[1], motor_size[2]);
    setParamCylinder(&motor[i][1], NO_M, motor_l, 3, motor_r);

    for(int j=0; j<MOTOR_NUM; j++){
      setParamCenter(&motor[i][j], motor_center[i][j][0], motor_center[i][j][1], motor_center[i][j][2]);
      setParamColor(&motor[i][j], motor_color[j][0], motor_color[j][1], motor_color[j][2], motor_color[j][3]);
      setParamAxis(&motor[i][j], motor_axis[j][0], motor_axis[j][1], motor_axis[j][2], motor_axis[j][3]);
    }
    generateObject(&motor[i][0], "Box");
    generateObject(&motor[i][1], "Cylinder");
  }

  /* リンク */
  for(int i=0; i<SERVO_NUM; i++){
    for(int j=0; j<LINK_NUM; j++){
      setParamBox(&link[i][j], link_m, link_size[j][0], link_size[j][1], link_size[j][2]);
      setParamCenter(&link[i][j], 0.0, 0.0, 0.0);
      setParamColor(&link[i][j], 0.0, 0.0, 0.0, 1.0);
      setParamAxis(&link[i][j], 0.0, 0.0, 0.0, 0.0);
      generateObject(&link[i][j], "Box");
    }
  }

  /* タップフィンガー */
  setParamCylinder(&tap[0], tap_m, tap_l, 3, tap_r);
  setParamSphere(&tap[1], tap_m, tap_r);

  for(int j=0; j<TAP_NUM; j++){
    setParamCenter(&tap[i], 0.0, 0.0, 0.0);
    setParamColor(&tap[i], 0.0, 0.0, 0.0, 1.0);
    setParamAxis(&tap[i], 0.0, 0.0, 0.0, 0.0);
  }
  generateObject(&tap[0], "Cylinder");
  generateObject(&tap[1], "Sphere");
}

// ロボットの関節を接続
static void connect() {
  for(int i=0; i<BASE_NUM; i++){
    setParamJointCenter(&base[i], 0.0, 0.0, 0.0);
    setParamJointHingeAxis(&base[i], 0.0, 0.0, 0.0);
    generateFixJoint(&base[i], 0);
  }
  for(int i=0; i<SERVO_NUM; i++){
    for(j=0; j<MOTOR_NUM; j++){
      setParamJointCenter(&motor[i][j], 0.0, 0.0, 0.0);
      setParamJointHingeAxis(&motor[i][j], 0.0, 0.0, 0.0);
      if(i==0){
        generateHingeJoint(&motor[i][0], &base[0]);
      }else{
        generateHingeJoint(&motor[i][j], &link[i-1][0]);
      }
      generateHingeJoint(&motor[i][1], &motor[i][0]);
    }
    for(j=0; j<LINK_NUM; j++){
      setParamJointCenter(&link[i][j], 0.0, 0.0, 0.0);
      setParamJointHingeAxis(&link[i][j], 0.0, 0.0, 0.0);
      generateHingeJoint(&link[i][j], &motor[i][1]);
    }
  }
  for(int i=0; i<TAP_NUM; i++){
    setParamJointCenter(&tap[i], 0.0, 0.0, 0.0);
    setParamJointHingeAxis(&tap[i], 0.0, 0.0, 0.0);
    generateFixJoint(&tap[i], link[SERVO_NUM-1][0]);
  }
}

// ロボット描画
static void draw() {
  /* ベース/環境 */
  for(int i=0; i<BASE_NUM; i++){
    if(i==3){
      drawObject(&base[i], "Cylinder");
    }else{
      drawObject(&base[i], "Box");
    }
  }
  /* モータ *//* リンク */
  for(int i=0; i<SERVO_NUM; i++){
    drawObject(&motor[i][0], "Box");
    drawObject(&motor[i][1], "Cylinder");
    if(j=0; j<LINK_NUM; j++){
      drawObject(&motor[i][j], "Box");
    }
  }
  /* タップフィンガー */
  drawObject(&tap[0], "Cylinder");
  drawObject(&tap[1], "Sphere");
}

// ロボット制御
static void control() {

}


void command(int cmd)
{
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
  setView(0.9,0.9,0.6,-135.0,-10.0,0.0);
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

  dWorldSetGravity(world,0.0,0.0,0.0);
  dWorldSetERP(world,1.0);          // ERPの設定
  dWorldSetCFM(world,0.0);          // CFMの設定
  ground = dCreatePlane(space,0,0,1,0);
  create();
  connect();
  dsSimulationLoop(argc,argv,500,500,&fn);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}
