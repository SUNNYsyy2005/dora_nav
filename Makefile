slam:
	rm -rf build/slam/laser_data.dat
	dora start slamflow.yml
create:ARGS1=$(filter-out $@,$(MAKECMDGOALS))
create:
	cd ${HOME}/dora_nav/build/slam && make run
	cd ${HOME}/dora_nav/build/nav && make run $(ARGS1)
start:
	dora start dataflow.yml
clear:
	rm -rf out
	rm -rf amcl.txt
	rm -rf teb.txt
	clear