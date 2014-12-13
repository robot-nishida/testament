参考URL：http://www.ssi-robot.com/?p=389

インストール先は
/opt
ではなく
/Sodtware
にしました。

drawstuff：
 R : ~/Software/src/ode-0.13 $ mkdir ~/Software/local/include/drawstuff
 R : ~/Software/src/ode-0.13 $ cp include/drawstuff/*.h ~/Software/local/include/drawstuff/
 R : ~/Software/src/ode-0.13 $ cp drawstuff//src/libdrawstuff.la ~/Software/local/lib/
 R : ~/Software/src/ode-0.13 $ cp drawstuff/src/.libs/libdrawstuff.a ~/Software/local/lib/
 R : ~/Software/src/ode-0.13 $ mkdir ~/Software/local/share/drawstuff

これ、必要だったかな？いらないかも
 R : ~/Software/src/ode-0.13 $ mkdir -p ~/Software/local/share/drawstuff
 R : ~/Software/src/ode-0.13 $ mkdir -p ~/Software/local/share/drawstuff/textures
 R : ~/Software/src/ode-0.13 $ cp drawstuff/textures/* ~/Software/local/share/drawstuff/textures/

デモ：
$ ode/demo/demo_dball

ビルド：
g++ hoge.cpp -I ~/Software/local/include/ `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL

ode-configへのpathを追加→bash_profile
export PATH=$HOME/Software/local/bin:$PATH

シェルスクリプト：
./ode.sh <ソースファイル名>

