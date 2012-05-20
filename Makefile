all: build make

make: build/Makefile
	(cd .build; make)

build/Makefile: build
	(cd .build; cmake ..)

build:
	mkdir -p .build

clean:
	rm -f  -r .build
	rm -f  -r bin