@echo off
echo === Building Courier Robot Docker Image ===
echo.

docker build -t courier-robot:nav2 .

echo.
echo === Build Complete ===
echo.
echo To run the container:
echo   docker run -it --rm -p 6080:80 --gpus all -v %cd%\ros2_ws:/home/ubuntu/ros2_ws --name courier_robot courier-robot:nav2
echo.
pause
