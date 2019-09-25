ip ?= 10.42.0.108

build_dir = build-$(shell qibuild config | sed -n '4p' | xargs)
sdk_dir = $(shell qitoolchain info | sed -n '5p' | sed 's/^ *in *//g')

ifeq ($(shell uname), Linux)
export LD_LIBRARY_PATH = $(sdk_dir)/lib
else ifeq ($(shell uname), Darwin)
export DYLD_LIBRARY_PATH = $(sdk_dir)/lib
endif

export PYTHONPATH = $(build_dir)/naoqipy:.

.PHONY: all config run runclient testpy clean

all:
	@qibuild make

config:
	@qibuild configure

run:
	@qibuild run main $(ip)

runclient:
	@qibuild run client

runclientpy:
	@python3 pyexample/client.py

testpy:
	@python3 pyexample/test.py

clean:
	@qibuild clean -f
