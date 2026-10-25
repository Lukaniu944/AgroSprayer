#include "lvgl_port.h"
#include "board_config.h"
#include "gt911.h"

#include <lvgl.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_lcd_panel_rgb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static esp_lcd_panel_handle_t s_panel = nullptr;
static lv_display_t *s_display = nullptr;

static SemaphoreHandle_t s_mutex = nullptr;
static SemaphoreHandle_t s_flush_done = nullptr;

static esp_timer_handle_t s_tick = nullptr;

static uint8_t *s_buf1 = nullptr;
static uint8_t *s_buf2 = nullptr;

#define LVGL_BUF_LINES 20

// Keep UI objects and software drawing allocations away from scarce DMA RAM.
// Split extra memory into pools no larger than the configured TLSF limit.
#if LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN
static uint8_t *s_lvgl_extra_memory = nullptr;
static bool add_lvgl_psram_pools()
{
    const size_t total_bytes = 256U * 1024U;
    const size_t chunk_bytes = LV_MEM_SIZE < 64U * 1024U
        ? LV_MEM_SIZE : 64U * 1024U;
    if (chunk_bytes < 4096U) {
        Serial.println("LVGL: configured memory pool is too small");
        return false;
    }
    s_lvgl_extra_memory = (uint8_t *)heap_caps_malloc(
        total_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_lvgl_extra_memory) {
        Serial.println("LVGL: cannot reserve 256 KiB in PSRAM");
        return false;
    }
    for (size_t offset = 0; offset < total_bytes; offset += chunk_bytes) {
        const size_t remaining = total_bytes - offset;
        const size_t bytes = remaining < chunk_bytes ? remaining : chunk_bytes;
        if (!lv_mem_add_pool(s_lvgl_extra_memory + offset, bytes)) {
            // Registered pools own slices of this block until device reset.
            // Do not free it while LVGL holds those slices.
            Serial.println("LVGL: cannot register extra memory pool");
            return false;
        }
    }
    Serial.println("LVGL: added 256 KiB PSRAM for UI and drawing");
    return true;
}
#endif

static void tick_cb(void *)
{
    lv_tick_inc(2);
}

static bool IRAM_ATTR on_color_trans_done(
    esp_lcd_panel_handle_t,
    const esp_lcd_rgb_panel_event_data_t *,
    void *
)
{
    BaseType_t hp_task_woken = pdFALSE;

    if (s_flush_done) {
        xSemaphoreGiveFromISR(
            s_flush_done,
            &hp_task_woken
        );
    }

    return hp_task_woken == pdTRUE;
}

static void flush_cb(
    lv_display_t *disp,
    const lv_area_t *area,
    uint8_t *px_map
)
{
    if (s_flush_done) {
        xSemaphoreTake(
            s_flush_done,
            0
        );
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(
        s_panel,
        area->x1,
        area->y1,
        area->x2 + 1,
        area->y2 + 1,
        px_map
    );

    if (err != ESP_OK) {
        Serial.printf(
            "LVGL FLUSH ERR: %d\n",
            (int)err
        );

        lv_display_flush_ready(disp);
        return;
    }

    // Core 3.3.12:
    // czekamy tutaj na prawdziwe zakonczenie transferu RGB.
    // Nie uzywamy lv_display_set_flush_wait_cb().
    if (xSemaphoreTake(
        s_flush_done,
        pdMS_TO_TICKS(100)
    ) != pdTRUE) {
        // RGB DMA may still be reading px_map. Keep the LVGL draw buffer
        // owned by the driver until the completion callback signals us.
        Serial.println("LVGL: RGB flush timeout; waiting for DMA completion");
        xSemaphoreTake(s_flush_done, portMAX_DELAY);
    }

    lv_display_flush_ready(disp);
}

static void touch_cb(
    lv_indev_t *,
    lv_indev_data_t *data
)
{
    TouchPoint p = gt911_read();

    if (p.pressed) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = p.x;
        data->point.y = p.y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

bool lvgl_lock(uint32_t timeout_ms)
{
    if (!s_mutex) return false;

    TickType_t ticks =
        timeout_ms == UINT32_MAX
            ? portMAX_DELAY
            : pdMS_TO_TICKS(timeout_ms);

    return xSemaphoreTakeRecursive(
        s_mutex,
        ticks
    ) == pdTRUE;
}

void lvgl_unlock()
{
    if (s_mutex) {
        xSemaphoreGiveRecursive(s_mutex);
    }
}

static void lvgl_task(void *)
{
    while (true) {
        uint32_t wait_ms = 5;

        if (lvgl_lock(100)) {
            wait_ms = lv_timer_handler();
            lvgl_unlock();
        }

        if (wait_ms < 2)  wait_ms = 2;
        if (wait_ms > 20) wait_ms = 20;

        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

bool lvgl_port_begin(
    esp_lcd_panel_handle_t panel
)
{
    s_panel = panel;

    if (!s_panel) {
        Serial.println("LVGL: brak panelu");
        return false;
    }

    lv_init();
#if LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN
    if (!add_lvgl_psram_pools()) return false;
#else
    Serial.println("LVGL: uses external allocator; PSRAM pools not added");
#endif

    s_mutex = xSemaphoreCreateRecursiveMutex();

    if (!s_mutex) {
        Serial.println("LVGL: brak mutex");
        return false;
    }

    s_flush_done = xSemaphoreCreateBinary();

    if (!s_flush_done) {
        Serial.println("LVGL: brak flush semaphore");
        return false;
    }

    const size_t buf_bytes =
        LCD_H_RES *
        LVGL_BUF_LINES *
        sizeof(uint16_t);

    s_buf1 = (uint8_t *)heap_caps_malloc(
        buf_bytes,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA
    );

    s_buf2 = (uint8_t *)heap_caps_malloc(
        buf_bytes,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA
    );

    if (!s_buf1 || !s_buf2) {
        Serial.printf(
            "LVGL: brak DMA RAM %p %p %u\n",
            s_buf1,
            s_buf2,
            (unsigned)buf_bytes
        );
        return false;
    }

    s_display = lv_display_create(
        LCD_H_RES,
        LCD_V_RES
    );

    if (!s_display) {
        Serial.println("LVGL: display create error");
        return false;
    }

    lv_display_set_color_format(
        s_display,
        LV_COLOR_FORMAT_RGB565
    );

    lv_display_set_buffers(
        s_display,
        s_buf1,
        s_buf2,
        buf_bytes,
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    lv_display_set_flush_cb(
        s_display,
        flush_cb
    );

    esp_lcd_rgb_panel_event_callbacks_t cbs = {};
    cbs.on_color_trans_done = on_color_trans_done;

    esp_err_t cb_err =
        esp_lcd_rgb_panel_register_event_callbacks(
            s_panel,
            &cbs,
            nullptr
        );

    if (cb_err != ESP_OK) {
        Serial.printf(
            "LVGL: callback error %d\n",
            (int)cb_err
        );
        return false;
    }

    lv_indev_t *indev = lv_indev_create();

    if (!indev) {
        Serial.println("LVGL: indev error");
        return false;
    }

    lv_indev_set_type(
        indev,
        LV_INDEV_TYPE_POINTER
    );

    lv_indev_set_read_cb(
        indev,
        touch_cb
    );

    esp_timer_create_args_t tick_args = {};
    tick_args.callback = tick_cb;
    tick_args.name = "lvgl_tick";

    if (esp_timer_create(
        &tick_args,
        &s_tick
    ) != ESP_OK) {
        Serial.println("LVGL: timer create error");
        return false;
    }

    if (esp_timer_start_periodic(
        s_tick,
        2000
    ) != ESP_OK) {
        Serial.println("LVGL: timer start error");
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        lvgl_task,
        "lvgl",
        8192,
        nullptr,
        2,
        nullptr,
        1
    );

    if (ok != pdPASS) {
        Serial.println("LVGL: task error");
        return false;
    }

    Serial.println(
        "LVGL 9.6 / CORE 3.3.12 FIX2 OK"
    );

    return true;
}
