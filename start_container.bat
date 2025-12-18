@echo off
echo ============================================
echo   Courier Robot - Quick Start
echo ============================================
echo.
echo Container will be available at: http://localhost:6080
echo.

:: Check if image exists
docker image inspect courier-robot:nav2 >nul 2>&1
if errorlevel 1 (
    echo [WARNING] Image not found! Run 'start_first_time.bat' first.
    pause
    exit /b 1
)

docker run -it --rm ^
    -p 6080:80 ^
    --gpus all ^
    -v "%cd%\ros2_ws:/home/ubuntu/ros2_ws" ^
    --name courier_robot ^
    courier-robot:nav2
