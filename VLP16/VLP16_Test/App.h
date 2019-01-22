#pragma once
class App
{
public:
	App() {}
	~App() {}

	HRESULT init();
	void run();

private:
	//Event
	bool _running;
};

