all:
	python3 setup.py build
	python3 setup.py install

install:
	mkdir -p /usr/include/motionlib
	cp c/geometry.h /usr/include/motionlib/
	cp c/vectorops.h /usr/include/motionlib/
	cp c/utils.h /usr/include/motionlib/
	cp c/so3.h /usr/include/motionlib/
	cp c/se3.h /usr/include/motionlib/

.PHONY: clean
clean:
	rm -rf build
