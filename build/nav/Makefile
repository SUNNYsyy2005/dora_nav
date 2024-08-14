.PHONY: run

install:
	g++ -o main main.cc -I/usr/local/include -L/usr/local/lib -llz4 -lm -std=c++17
debug:
	g++ -g -o main main.cc -I/usr/local/include -L/usr/local/lib -llz4 -lm -std=c++17

run: ARGS=$(filter-out $@,$(MAKECMDGOALS))
run:
	rm -rf ../teb/path.csv
	./main $(ARGS)
%:
	@:

run2: ARGS
run2:
	rm -rf ../teb/path.csv
	./main $(ARGS)
