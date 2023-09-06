//bitluni 2023

#pragma once
#include "Adafruit_GFX.h"

template<class Base>
class GfxWrapper : public Adafruit_GFX
{
  public:
	Base &base;
	GfxWrapper(Base &vga, const int xres, const int yres)
		:
		Adafruit_GFX(xres, yres),
		base(vga)
	{
	}

	virtual void drawPixel(int16_t x, int16_t y, uint16_t color)
	{
		base.dot(x, y, base.rgb((color >> 8) & 0b11111000, (color >> 3) & 0b11111100, (color << 3) & 0b11111000));
	}
};
