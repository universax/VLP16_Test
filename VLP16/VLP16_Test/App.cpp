#include "stdafx.h"
#include "App.h"
#include "VLP16.h"

using namespace boost::property_tree;
using namespace boost::property_tree::xml_parser;

HRESULT App::init() {
	HRESULT hr = S_OK;
	_running = false;

	if (SUCCEEDED(hr))
	{
		_running = true;
	}
	return hr;
}

void App::run() {
	//VLP
	VLP16::VLP16 vlp;
	vlp.Start("192.168.3.201", 2368);
	vlp.Run();
}