@echo off
setlocal enabledelayedexpansion

echo ============================================
echo   Courier Robot - First Time Setup
echo ============================================
echo.

:: Check if Docker is running
docker info >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker is not running!
    echo Please start Docker Desktop and try again.
    pause
    exit /b 1
)

:: Check if image already exists
docker image inspect courier-robot:nav2 >nul 2>&1
if errorlevel 1 (
    echo [INFO] Docker image 'courier-robot:nav2' not found.
    echo [INFO] Building custom image with Nav2 pre-installed...
    echo.
    echo This may take 10-15 minutes on first run...
    echo.
    
    docker build -t courier-robot:nav2 .
    
    if errorlevel 1 (
        echo.
        echo [ERROR] Docker build failed!
        pause
        exit /b 1
    )
    
    echo.
    echo [SUCCESS] Docker image built successfully!
) else (
    echo [INFO] Docker image 'courier-robot:nav2' already exists.
    echo [INFO] Skipping build. To rebuild, run: docker build -t courier-robot:nav2 .
)

echo.
echo ============================================
echo   Starting Container
echo ============================================
echo.
echo Container will be available at: http://localhost:6080
echo.
echo Once inside the container, run:
echo   cd /home/ubuntu/ros2_ws
echo   colcon build --symlink-install
echo   source install/setup.bash
echo   ./start_nav2.sh
echo.
echo ============================================
echo.

:: Run the container
docker run -it --rm ^
    -p 6080:80 ^
    --gpus all ^
    -v "%cd%\ros2_ws:/home/ubuntu/ros2_ws" ^
    --name courier_robot ^
    courier-robot:nav2

echo.
echo Container stopped.
pause
