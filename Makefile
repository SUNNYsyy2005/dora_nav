slam:
	rm -rf build/slam/laser_data.dat
	dora start slamflow.yml
create:
	cd /home/sunny/dora_nav/build/slam && make run
	cd /home/sunny/dora_nav/build/nav && make run 400 450 400 400
start:
	dora start dataflow.yml
