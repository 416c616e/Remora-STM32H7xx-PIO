#include "module.h"

#include <cstdio>

Module::Module()
{
	this->counter = 0;
	this->updateCount = 1;
	printf("Creating a std module\n\r");
}


Module::Module(int32_t threadFreq, int32_t slowUpdateFreq) :
	threadFreq(threadFreq),
	slowUpdateFreq(slowUpdateFreq)
{
	this->counter = 0;
	this->updateCount = this->threadFreq / this->slowUpdateFreq;
	printf("Creating a slower module, updating every %ld thread cycles\n\r",this->updateCount);
}

Module::~Module(){}


void Module::runModule()
{
	++this->counter;

	if (this->counter >= this->updateCount)
	{
		this->slowUpdate();
		this->counter = 0;
	}

	this->update();
}


void Module::runModulePost()
{
	this->updatePost();
}

void Module::update(){}
void Module::updatePost(){}
void Module::slowUpdate(){}
void Module::configure(){}
