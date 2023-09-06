#ifndef VGA_H
#define VGA_H

#include "PinConfig.h"
#include "Mode.h"

class VGA
{
	public:
	Mode mode;
	int bufferCount;
	int bits;
	PinConfig pins;
	
	public:
	VGA();
	~VGA();
	bool init(const PinConfig pins, const Mode mode, int bits);
	bool start();
	bool show();
	void dot(int x, int y, int r, int g, int b);
	void dot(int x, int y, int rgb);
	void dotdit(int x, int y, int r, int g, int b);
	void dotdit(int x, int y, int rgb);
	int rgb(int r, int g, int b);
};

#endif //VGA_h