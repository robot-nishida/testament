# Makefile

testament: testament.o
	g++ -w -o exe testament.o `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL

testament.o: testament.cpp
	g++ -w -c testament.cpp -I ~/Software/local/include/

.PHONY: clean
clean:
	rm -f testament.o