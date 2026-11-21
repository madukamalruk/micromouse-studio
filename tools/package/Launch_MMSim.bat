@echo off
rem MMSim launcher: starts the simulator on a bundled maze with the Flood-Fill solver.
rem Pick other mazes and solvers from the menus at the top once it is open.
cd /d "%~dp0"
start "" "%~dp0mmsim.exe" --maze "%~dp0mazes\test\gen16_loops_a.txt" --firmware demo_floodfill
