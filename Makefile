all: build run

build:
	docker build -f Dockerfile -t conversion .

run:
	@echo hello
	docker run conversion