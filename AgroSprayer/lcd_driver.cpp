#include "lcd_driver.h"
#include "board_config.h"

#include "esp_lcd_panel_rgb.h"
#include "esp_err.h"

static esp_lcd_panel_handle_t s_panel = nullptr;

esp_lcd_panel_handle_t lcd_begin()
{
    if (s_panel) {
        return s_panel;
    }

    esp_lcd_rgb_panel_config_t cfg = {};

    cfg.clk_src = LCD_CLK_SRC_DEFAULT;

    // Zachowujemy sprawdzone 20 MHz.
    cfg.timings.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    cfg.timings.h_res = LCD_H_RES;
    cfg.timings.v_res = LCD_V_RES;

    cfg.timings.hsync_pulse_width = 162;
    cfg.timings.hsync_back_porch  = 152;
    cfg.timings.hsync_front_porch = 48;

    cfg.timings.vsync_pulse_width = 45;
    cfg.timings.vsync_back_porch  = 13;
    cfg.timings.vsync_front_porch = 3;

    cfg.timings.flags.pclk_active_neg = 1;

    cfg.data_width = LCD_DATA_WIDTH;
    cfg.bits_per_pixel = LCD_BITS_PER_PIXEL;

    // KLUCZOWA ZMIANA:
    // Jeden framebuffer w PSRAM zamiast dwóch.
    // Dwa pełne framebuffer'y zwiększają ruch w PSRAM.
    cfg.num_fbs = 1;

    // Dwa bounce-buffery są automatycznie tworzone przez driver
    // w pamięci wewnętrznej.
    cfg.bounce_buffer_size_px =
        LCD_H_RES * LCD_BOUNCE_LINES;

    // DMA burst 64 B - stabilne ustawienie dla RGB/PSRAM.
    cfg.dma_burst_size = 64;

    cfg.sram_trans_align = 64;
    cfg.psram_trans_align = 64;

    cfg.hsync_gpio_num = LCD_GPIO_HSYNC;
    cfg.vsync_gpio_num = LCD_GPIO_VSYNC;
    cfg.de_gpio_num    = LCD_GPIO_DE;
    cfg.pclk_gpio_num  = LCD_GPIO_PCLK;
    cfg.disp_gpio_num  = GPIO_NUM_NC;

    cfg.data_gpio_nums[0]  = LCD_GPIO_B0;
    cfg.data_gpio_nums[1]  = LCD_GPIO_B1;
    cfg.data_gpio_nums[2]  = LCD_GPIO_B2;
    cfg.data_gpio_nums[3]  = LCD_GPIO_B3;
    cfg.data_gpio_nums[4]  = LCD_GPIO_B4;

    cfg.data_gpio_nums[5]  = LCD_GPIO_G0;
    cfg.data_gpio_nums[6]  = LCD_GPIO_G1;
    cfg.data_gpio_nums[7]  = LCD_GPIO_G2;
    cfg.data_gpio_nums[8]  = LCD_GPIO_G3;
    cfg.data_gpio_nums[9]  = LCD_GPIO_G4;
    cfg.data_gpio_nums[10] = LCD_GPIO_G5;

    cfg.data_gpio_nums[11] = LCD_GPIO_R0;
    cfg.data_gpio_nums[12] = LCD_GPIO_R1;
    cfg.data_gpio_nums[13] = LCD_GPIO_R2;
    cfg.data_gpio_nums[14] = LCD_GPIO_R3;
    cfg.data_gpio_nums[15] = LCD_GPIO_R4;

    // Framebuffer w PSRAM, ale DMA czyta przez wewnętrzne bounce-buffery.
    cfg.flags.fb_in_psram = 1;

    // Nie używamy double_fb.
    cfg.flags.double_fb = 0;

    // Normalne ciągłe odświeżanie.
    cfg.flags.refresh_on_demand = 0;

    ESP_ERROR_CHECK(
        esp_lcd_new_rgb_panel(
            &cfg,
            &s_panel
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_init(
            s_panel
        )
    );

    return s_panel;
}
