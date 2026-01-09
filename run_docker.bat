@echo off
echo === Starting Courier Robot Container ===
echo.
echo Container will be available at: http://localhost:6080
echo Once inside, run: ./start_mission.sh
echo.

docker run -it --rm -p 6080:80 --gpus all -v %cd%\ros2_ws:/home/ubuntu/ros2_ws --name courier_robot courier-robot:latest
