#!/bin/sh

src=$1
name=`echo $src | cut -d "." -f 1`
dst=`echo "${name}.out"`
echo ${dst}
g++ -o $dst $1 -I ~/Software/local/include/ `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL
