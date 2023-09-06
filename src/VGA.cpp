#include "VGA.h"

VGA::VGA()
{
	bufferCount = 1;
}

VGA::~VGA()
{
	bits = 0;
}

bool VGA::init(const PinConfig pins, const Mode mode, int bits)
{
	this->mode = mode;
	this->bits = bits;
	
	return false;
}

bool VGA::start()
{
	return false;
}

bool VGA::show()
{
	return false;
}

void VGA::dot(int x, int y, int r, int g, int b)
{
}

void VGA::dot(int x, int y, int rgb)
{
}

void VGA::dotdit(int x, int y, int r, int g, int b)
{
}

void VGA::dotdit(int x, int y, int rgb)
{
}

int VGA::rgb(int r, int g, int b)
{
	return 0;
}