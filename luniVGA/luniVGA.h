#pragma once
/*************************************************
 * 
 * 		author: bitluni / parts from esp-idf
 * 		pls attribute me. k thx bye
 * 
 * ***********************************************/
#include <stdlib.h>
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_hal.h>
#include <rom/cache.h>
#include <soc/lcd_cam_struct.h>
#include "spiram.h"

#include "DMAVideoBuffer.h"

//borrowed from esp code
#define HAL_FORCE_MODIFY_U32_REG_FIELD(base_reg, reg_field, field_val)    \
{                                                           \
	uint32_t temp_val = base_reg.val;                       \
	typeof(base_reg) temp_reg;                              \
	temp_reg.val = temp_val;                                \
	temp_reg.reg_field = (field_val);                       \
	(base_reg).val = temp_reg.val;                          \
}

extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

class PinConfig
{
	public:
		int r[5];
		int g[6];
		int b[5];
		int hSync, vSync;

	PinConfig(
		int r0, int r1, int r2, int r3, int r4, 
		int g0, int g1, int g2, int g3, int g4, int g5, 
		int b0, int b1, int b2, int b3, int b4,
		int hSync, int vSync)
		{
			r[0] = r0; r[1] = r1; r[2] = r2; r[3] = r3; r[4] = r4;
			g[0] = g0; g[1] = g1; g[2] = g2; g[3] = g3; g[4] = g4; g[5] = g5;
			b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; b[4] = b4;
			this->hSync = hSync;
			this->vSync = vSync;
		}
};

class VGAMode
{
	public:
	uint32_t hFront, hSync, hBack, hRes, hPol;
	uint32_t vFront, vSync, vBack, vRes, vPol, vClones;
	uint32_t frequency;
	VGAMode()
	{
	}

	VGAMode(const VGAMode &m)
	{
		this->hFront = m.hFront;
		this->hSync = m.hSync;
		this->hBack = m.hBack;
		this->hRes = m.hRes;
		this->hPol = m.hPol;
		this->vFront = m.vFront;
		this->vSync = m.vSync;
		this->vBack = m.vBack;
		this->vRes = m.vRes;
		this->vPol = m.vPol;
		this->frequency = m.frequency;
		this->vClones = m.vClones;
	}

	VGAMode(int hFront, int hSync, int hBack, int hRes, int vFront, int vSync, int vBack, int vRes, int frequency, 
		int hPol = 1, int vPol = 1, int vClones = 1)
	{
		this->hFront = hFront;
		this->hSync = hSync;
		this->hBack = hBack;
		this->hRes = hRes;
		this->hPol = hPol;
		this->vFront = vFront;
		this->vSync = vSync;
		this->vBack = vBack;
		this->vRes = vRes;
		this->vPol = vPol;
		this->frequency = frequency;
		this->vClones = vClones;
	}

	int totalHorizontal() const
	{
		return hFront + hSync + hBack + hRes;
	}

	int totalVertical() const
	{
		return vFront + vSync + vBack + vRes * vClones;
	}

	int blankHorizontal() const
	{
		return hFront + hSync + hBack;
	}

	int blankVertical() const
	{
		return vFront + vSync + vBack;
	}
};

//VESA-DMT-1.13.pdf 
//r=reduced blanking
//f=fake mode
const VGAMode MODE_640x400x70(16, 96, 48, 640, 12, 2, 35, 400, 25175000);
const VGAMode MODE_320x200x70(8, 48, 24, 320, 12, 2, 35, 200, 12587500, 0, 0, 2);
const VGAMode MODE_640x480x60(16, 96, 48, 640, 10, 2, 33, 480, 25175000);
const VGAMode MODE_800x600x56(24, 72, 128, 800, 1, 2, 22, 600, 36000000);
const VGAMode MODE_800x600x60(40, 128, 88, 800, 1, 4, 23, 600, 40000000);
const VGAMode MODE_1024x768x43(8, 176, 56, 1024, 0, 4, 20, 768, 44900000);
const VGAMode MODE_1024x768x60(24, 136, 160, 1024, 3, 6, 29, 768, 65000000);
const VGAMode MODE_1280x720x60(110, 40, 220, 1280, 5, 5, 20, 720, 74250000);
const VGAMode MODE_1024x768x40f(8, 32, 100, 1024, 1, 4, 19, 768, 40000000);


gdma_channel_handle_t dma_chan;
int bitCount = 0;
DMAVideoBuffer *dmaBuffer = 0;
const VGAMode *mode;
const int bufferCount = 1;

int backBuffer = 0;

void attachPinToSignal(int pin, int signal)
{
	esp_rom_gpio_connect_out_signal(pin, signal, false, false);
	gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
	gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
}

bool vgaInit(const PinConfig &pinConfig, const VGAMode &vgaMode, int bits = 8, bool usePsram = true)
{
	mode = &vgaMode;
	bitCount = bits;
	backBuffer = 0;
	dmaBuffer = new DMAVideoBuffer(vgaMode.vRes, vgaMode.hRes * (bits / 8), vgaMode.vClones, true, usePsram, bufferCount);
	if(!dmaBuffer->isValid())
	{
		delete dmaBuffer;
		return false;
	}

	periph_module_enable(PERIPH_LCD_CAM_MODULE);
	periph_module_reset(PERIPH_LCD_CAM_MODULE);
	LCD_CAM.lcd_user.lcd_reset = 1;
	esp_rom_delay_us(100);


	// f=240000000/n
	// n=240000000/f;
	int N = round(240000000.0/(double)vgaMode.frequency);
	if(N < 2) N = 2;
	// clk = source / (N + b/a) --integer--> clk = source / N
	LCD_CAM.lcd_clock.clk_en = 1;
	LCD_CAM.lcd_clock.lcd_clk_sel = 2;			// PLL240M
	// - For integer divider, LCD_CAM_LCD_CLKM_DIV_A and LCD_CAM_LCD_CLKM_DIV_B are cleared.
	// - For fractional divider, the value of LCD_CAM_LCD_CLKM_DIV_B should be less than the value of LCD_CAM_LCD_CLKM_DIV_A.
	LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
	LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
	LCD_CAM.lcd_clock.lcd_clkm_div_num = N; 	// 0 => 256; 1 => 2; 14 compfy
	LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;		
	LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
	LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;


	LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 1;
	LCD_CAM.lcd_user.lcd_2byte_en = (bits==8)?0:1;
    LCD_CAM.lcd_user.lcd_cmd = 0;
    LCD_CAM.lcd_user.lcd_dummy = 0;
    LCD_CAM.lcd_user.lcd_dout = 1;
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;
    LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;//-1;
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 0;
	LCD_CAM.lcd_user.lcd_always_out_en = 1;
    LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = vgaMode.hPol ^ 1;
    LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = vgaMode.vPol ^ 1;
    LCD_CAM.lcd_ctrl2.lcd_de_idle_pol = 1;	

	LCD_CAM.lcd_misc.lcd_bk_en = 1;	
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;

	LCD_CAM.lcd_ctrl2.lcd_hsync_width = vgaMode.hSync - 1;				//7 bit
    LCD_CAM.lcd_ctrl.lcd_hb_front = vgaMode.blankHorizontal() - 1;		//11 bit
    LCD_CAM.lcd_ctrl1.lcd_ha_width = vgaMode.hRes - 1;					//12 bit
    LCD_CAM.lcd_ctrl1.lcd_ht_width = vgaMode.totalHorizontal();			//12 bit

	LCD_CAM.lcd_ctrl2.lcd_vsync_width = vgaMode.vSync - 1;				//7bit
    HAL_FORCE_MODIFY_U32_REG_FIELD(LCD_CAM.lcd_ctrl1, lcd_vb_front, vgaMode.vSync + vgaMode.vBack - 1);		//8bit
    LCD_CAM.lcd_ctrl.lcd_va_height = vgaMode.vRes * vgaMode.vClones - 1;					//10 bit
    LCD_CAM.lcd_ctrl.lcd_vt_height = vgaMode.totalVertical() - 1;		//10 bit

	LCD_CAM.lcd_ctrl2.lcd_hs_blank_en = 1;
	HAL_FORCE_MODIFY_U32_REG_FIELD(LCD_CAM.lcd_ctrl2, lcd_hsync_position, 0);//vgaMode.hFront);

	LCD_CAM.lcd_misc.lcd_next_frame_en = 1; //?? limitation

	if(bits == 8)
	{
		int pins[8] = {
			pinConfig.r[2], pinConfig.r[3], pinConfig.r[4],
			pinConfig.g[3], pinConfig.g[4], pinConfig.g[5],
			pinConfig.b[3], pinConfig.b[4]
		};
		for (int i = 0; i < bits; i++) 
			if (pins[i] >= 0) 
				attachPinToSignal(pins[i], LCD_DATA_OUT0_IDX + i);
	}
	else if(bits == 16)
	{
		int pins[16] = {
			pinConfig.r[0], pinConfig.r[1], pinConfig.r[2], pinConfig.r[3], pinConfig.r[4],
			pinConfig.g[0], pinConfig.g[1], pinConfig.g[2], pinConfig.g[3], pinConfig.g[4], pinConfig.g[5],
			pinConfig.b[0], pinConfig.b[1], pinConfig.b[2], pinConfig.b[3], pinConfig.b[4]
		};
		for (int i = 0; i < bits; i++) 
			if (pins[i] >= 0) 
				attachPinToSignal(pins[i], LCD_DATA_OUT0_IDX + i);
	}
	attachPinToSignal(pinConfig.hSync, LCD_H_SYNC_IDX);
	attachPinToSignal(pinConfig.vSync, LCD_V_SYNC_IDX);
  
	gdma_channel_alloc_config_t dma_chan_config = 
	{
		.direction = GDMA_CHANNEL_DIRECTION_TX,
	};
	gdma_new_channel(&dma_chan_config, &dma_chan);
	gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
	gdma_transfer_ability_t ability = 
	{
        .sram_trans_align = 4,
        .psram_trans_align = 64,
    };
    gdma_set_transfer_ability(dma_chan, &ability);
	return true;
}

void vgaDot(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b)
{
	int v = y;
	int h = x;
	if(x >= mode->hRes || y >= mode->vRes) return;
	if(bitCount == 8)
		dmaBuffer->getLineAddr8(y, backBuffer)[x] = (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
	else if(bitCount == 16)
		dmaBuffer->getLineAddr16(y, backBuffer)[x] = (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);
}

void vgaDotDit(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b)
{
	if(x >= mode->hRes || y >= mode->vRes) return;
	if(bitCount == 8)
	{
		r = min((rand() & 31) + r, 255);
		g = min((rand() & 31) + g, 255);
		b = min((rand() & 63) + b, 255);
		dmaBuffer->getLineAddr8(y, backBuffer)[x] = (r >> 5) | ((g >> 5) << 3) | (b & 0b11000000);
	}
	else
	if(bitCount == 16)
	{
		r = min((rand() & 7) + r, 255);
		g = min((rand() & 3) + g, 255); 
		b = min((rand() & 7) + b, 255);
		dmaBuffer->getLineAddr16(y, backBuffer)[x] = (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);
	}
}

void vgaShow()
{
	dmaBuffer->flush(backBuffer);
	if(bufferCount <= 1) 
		return;
	dmaBuffer->attachBuffer(backBuffer);
	backBuffer = (backBuffer + 1) % bufferCount;
}

void vgaStart()
{
	//very delicate... dma might be late for peripheral
	gdma_reset(dma_chan);
    esp_rom_delay_us(1);	
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update = 1;
	esp_rom_delay_us(1);
	LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_user.lcd_update = 1;
	gdma_start(dma_chan, (intptr_t)dmaBuffer->getDescriptor());
    esp_rom_delay_us(1);
    LCD_CAM.lcd_user.lcd_update = 1;
	LCD_CAM.lcd_user.lcd_start = 1;
}

