all: docker_build build compile test

docker_build:
	docker build -f Dockerfile -t conversion .

build:
	docker run -u $(shell id -u):$(shell id -g) --rm -v $(shell pwd):/src -w /src conversion \
		/bin/bash -c "cmake -B build -S . -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)"
.PHONY: build

compile:
	docker run -u $(shell id -u):$(shell id -g) --rm -v $(shell pwd):/src -w /src conversion \
		/bin/bash -c "make -C build"

test:
	docker run -u $(shell id -u):$(shell id -g) --rm -v $(shell pwd):/src -w /src conversion \
		/bin/bash -c "build/vessel_test"
.PHONY: test

clean:
	@rm -rf build
.PHONY: clean
