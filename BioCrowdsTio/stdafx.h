// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

//threads @TODO
/*#include <QDebug>
#include <QThread>
#include <QString>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QApplication>*/

#include <stdio.h>
#include <string>
#include <tchar.h>
#include <iostream>
#include <io.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <time.h>
#include "Vector3.h"

//node struct - Vector3 position, triangle vertices position
typedef struct { Vector3 position; float v1X, v1Z, v2X, v2Z, v3X, v3Z; } Node;

#include "Goal.h"
#include "Sign.h"
#include "Marker.h"
#include "Cell.h"
#include "Hofstede.h"
#include "Agent.h"
#include "AgentGroup.h"
#include "delaunay.h"
#include "AStar.h"
#include "AStarSearchNode.h"
#include <array>
#include <SFML/Graphics.hpp>
#include "Simulation.h"
