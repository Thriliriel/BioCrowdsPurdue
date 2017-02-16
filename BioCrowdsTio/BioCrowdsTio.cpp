// BioCrowdsTio.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

//test function for threads
/*void myRunFunction(QString name)
{
	for (int i = 0; i <= 5; i++)
	{
		qDebug() << name << " " << i <<
			"from" << QThread::currentThread();
	}
}*/

int main(int argcp, char **argv)
{
	srand(42);

	//configure scenario
	Simulation newSimulation(Vector3(100, 0, 100), 1, argcp, argv);
	//Simulation newSimulation(Vector3(1000, 0, 1000), 5, argcp, argv);
	//Simulation newSimulation(Vector3(30, 0, 20), 1, argcp, argv);

	//system("PAUSE");

    return 0;

	/*QCoreApplication a(argcp, argv);
	
	QFuture<void> t1 = QtConcurrent::run(myRunFunction, QString("A"));
	QFuture<void> t2 = QtConcurrent::run(myRunFunction, QString("B"));
	QFuture<void> t3 = QtConcurrent::run(myRunFunction, QString("C"));

	t1.waitForFinished();
	t2.waitForFinished();
	t3.waitForFinished();

	return a.exec();*/
}

