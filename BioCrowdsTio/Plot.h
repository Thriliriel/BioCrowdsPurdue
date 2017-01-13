#pragma once

#include <GL\freeglut.h>

class Plot
{
	//public attributes
	///////////////////////

	//private attributes
	private:
		std::string windowName;
		int windowSizeX;
		int windowSizeY;

	//public methods
	public:
		Plot();
		Plot(std::string newWindowName, int newWindowSizeX, int newWindowSizeY, int argcp, char **argv);
		~Plot();
		void MainLoop();

	//private methods
	private:
		void Init();
		//void Display();
		//void DrawObject();
		void CenterOnScreen();
};