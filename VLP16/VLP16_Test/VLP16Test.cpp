// VLP16Test.cpp : Defines the entry point for the console application.


#include "stdafx.h"
#include "App.h"

int main(int argc, char ** argv)
{
	try {
		App app;
		app.init();
		app.run();
	}
	catch (std::exception &ex)
	{
		cout << ex.what() << endl;
		return -1;
	}
	return 0;
}