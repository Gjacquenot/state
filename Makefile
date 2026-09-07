all: build run

build:
	docker build -f Dockerfile -t conversion .
.PHONY: build

run:
	@echo hello
	docker run conversion
	docker run --entrypoint /bin/bash conversion -c "/usr/local/bin/vessel_test"
.PHONY: run

clean:
	@rm -rf build
.PHONY: clean
