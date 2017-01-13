#include "stdafx.h"
#include "Plot.h"

//-------------------------------------------------------------------------
//  Draws our object.
//-------------------------------------------------------------------------
void DrawObject()
{
	//  Draw Icosahedron
	glutWireIcosahedron();
}

//-------------------------------------------------------------------------
//  This function is passed to glutDisplayFunc in order to display 
//  OpenGL contents on the window.
//-------------------------------------------------------------------------
void Display()
{
	//  Clear the window or more specifically the frame buffer...
	//  This happens by replacing all the contents of the frame
	//  buffer by the clear color (black in our case)
	glClear(GL_COLOR_BUFFER_BIT);

	//  Draw object
	DrawObject();

	//  Swap contents of backward and forward frame buffers
	glutSwapBuffers();
}

Plot::Plot()
{
}

Plot::Plot(std::string newWindowName, int newWindowSizeX, int newWindowSizeY, int argcp, char **argv)
{
	windowName = newWindowName;
	windowSizeX = newWindowSizeX;
	windowSizeY = newWindowSizeY;

	//open window
	glutInit(&argcp, argv);
	CenterOnScreen();
	glutInitWindowSize(windowSizeX, windowSizeY);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	const char* newName = windowName.c_str();
	glutCreateWindow(newName);

	//  Set OpenGL program initial state.
	Init();

	// Set the callback functions
	glutDisplayFunc(Display);	
}

Plot::~Plot()
{
}

//call main loop
void Plot::MainLoop() {
	//  Start GLUT event processing loop (just 1 iteration to not lock the code)
	glutMainLoopEvent();
}

//-------------------------------------------------------------------------
//  Set OpenGL program initial state.
//-------------------------------------------------------------------------
void Plot::Init()
{
	//  Set the frame buffer clear color to black. 
	glClearColor(0.0, 0.0, 0.0, 0.0);
}

//-------------------------------------------------------------------------
//  This function sets the window x and y coordinates
//  such that the window becomes centered
//-------------------------------------------------------------------------
void Plot::CenterOnScreen()
{
	windowSizeX = (glutGet(GLUT_SCREEN_WIDTH) - windowSizeX) / 2;
	windowSizeY = (glutGet(GLUT_SCREEN_HEIGHT) - windowSizeY) / 2;
}
