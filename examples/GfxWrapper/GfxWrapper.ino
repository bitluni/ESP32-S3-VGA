//This example shows how to use the GfxWrapper to be able to use the Adafruit GFX library with VGA
//bitluni

#include "ESP32S3VGA.h"
#include <GfxWrapper.h>
#include <Fonts/FreeMonoBoldOblique24pt7b.h>
#include <Fonts/FreeSerif24pt7b.h>

//                   r,r,r,r,r,  g,g, g, g, g, g,   b, b, b, b, b,   h,v
const PinConfig pins(4,5,6,7,8,  9,10,11,12,13,14,  15,16,17,18,21,  1,2);

//VGA Device
VGA vga;
Mode mode = Mode::MODE_320x240x60;
GfxWrapper<VGA> gfx(vga, mode.hRes, mode.vRes);

//initial setup
void setup()
{
	vga.bufferCount = 2;
	if(!vga.init(pins, mode, 16)) while(1) delay(1);

	vga.start();
}

//the loop is done every frame
void loop()
{
	static int x = 0;
	vga.clear(vga.rgb(0x80, 0, 0));
	//using adafruit gfx
	gfx.setFont(&FreeMonoBoldOblique24pt7b);
	gfx.setCursor(100 + x, 100);
	gfx.print("Hello");
	gfx.setFont(&FreeSerif24pt7b);
	gfx.setCursor(100, 200);
	gfx.print("World!");
	vga.show();
	x = (x + 1) & 255;
}
