#include "valve_test.h"
#include "pressure_sensor.h"
#include "ui.h"
#include "board_config.h"
#include "sprayer_state.h"
#include "aog.h"
#include "flowmeter.h"

#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <initializer_list>
#include <Preferences.h>
#include "tank_counter.h"
#include "job_counter.h"
#include "nozzle_model.h"
#include "auto_regulator.h"
#include "valve_ui_interlock.h"
#include <cmath>

extern "C" { LV_FONT_DECLARE(dashboard_font_48); LV_FONT_DECLARE(home_font_18); LV_FONT_DECLARE(home_font_24); LV_FONT_DECLARE(home_font_32); }

// Avoid allocating label buffers and invalidating widgets for unchanged data.
static void set_label_text_if_changed(lv_obj_t *label, const char *text)
{
    if (label && text && strcmp(lv_label_get_text(label), text) != 0) {
        lv_label_set_text(label, text);
    }
}

static void set_text_color_if_changed(lv_obj_t *obj, lv_color_t color,
                                      lv_style_selector_t selector)
{
    if (obj && !lv_color_eq(lv_obj_get_style_text_color(obj, LV_PART_MAIN), color)) {
        lv_obj_set_style_text_color(obj, color, selector);
    }
}

static lv_obj_t *lblWifi;
static lv_obj_t *lblAog;
static lv_obj_t *lblUdp;
static lv_obj_t *lblSpeed;
static lv_obj_t *lblRate;
static lv_obj_t *lblActualRate;
static lv_obj_t *lblWorkStatus;
static lv_obj_t *lblFlow;
static lv_obj_t *lblTarget;
static lv_obj_t *lblPressure;
static lv_obj_t *valvePanel = nullptr, *valveStatus = nullptr;
static lv_obj_t *valvePressure = nullptr;
static lv_obj_t *valveAutoButton = nullptr, *valveAutoStatus = nullptr;
static void create_valve_test();
static void valve_refresh();
static lv_obj_t *lblTank;
static lv_obj_t *infoPanel=nullptr,*infoLeft=nullptr,*infoRight=nullptr,*infoNote=nullptr;
static bool infoIsTank=false;
static void diagnostics_open_cb(lv_event_t *);
static void tank_open_cb(lv_event_t *);
static void info_refresh();
static lv_obj_t *jobPanel=nullptr,*jobValues[4]={},*jobStatus=nullptr,*jobPauseButton=nullptr,*jobNewButton=nullptr;
static bool jobConfirm=false;
static void job_open_cb(lv_event_t *);
static void job_refresh();
static lv_obj_t *lblTankPct;
static lv_obj_t *lblReg;
static lv_obj_t *lblErr;
static lv_obj_t *lblTotal;
static lv_obj_t *lblMode;
static lv_obj_t *lblBoomWidth;

static lv_obj_t *sectionEditorPanel = nullptr;
static lv_obj_t *sectionEditorTitle = nullptr;
static lv_obj_t *sectionEditorValue = nullptr;
static lv_obj_t *sectionEditorTotal = nullptr;
static lv_obj_t *detailMsg = nullptr;
static lv_obj_t *detailMsg2 = nullptr;

static int sectionEditIndex = 0;
static lv_obj_t *flowCalPanel = nullptr;
static lv_obj_t *flowCalStatus = nullptr;
static lv_obj_t *flowCalPulses = nullptr;
static lv_obj_t *flowCalVolume = nullptr;
static lv_obj_t *flowCalCurrent = nullptr;
static lv_obj_t *flowCalResult = nullptr;
static lv_obj_t *flowCalTime = nullptr;
static lv_obj_t *flowCalAverage = nullptr;

static lv_obj_t *flowCalLiters = nullptr;
static lv_obj_t *flowCalInstant = nullptr;
static lv_obj_t *flowCalInputPanel = nullptr;
static lv_obj_t *flowCalInputLabel = nullptr;
static char flowCalInput[12] = "";
static const char *flowCalMessage = "GOTOWY";
static bool flowCalSaved = false;
static uint32_t flowCalRefreshMs = 0;
static bool flowCalRunning = false;
static uint32_t flowCalStartPulses = 0;
static uint32_t flowCalCapturedPulses = 0;
static uint32_t flowCalStartMs = 0;
static uint32_t flowCalElapsedMs = 0;
static int flowCalVolumeX10 = 1000; // 100.0 L
static uint32_t flowCalCandidateX100 = 0;

static float sectionWidthM[SECTION_COUNT] = {2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f};
static Preferences sectionPrefs;

static lv_obj_t *btnSections[SECTION_COUNT];
static lv_obj_t *sprayerSections[SECTION_COUNT];

static bool prevSectionState[SECTION_COUNT] = {false, false, false, false, false, false};
static bool prevSectionStateValid = false;
static bool previousAutoMode = true;

static lv_obj_t *btnAuto;
static lv_obj_t *btnMan;
static lv_obj_t *btnValveOpen=nullptr, *btnValveClose=nullptr;
static lv_obj_t *valveDirectionButton=nullptr;

static lv_obj_t *scrHome = nullptr;
static lv_obj_t *scrSettings = nullptr;
static lv_obj_t *scrNavigation = nullptr;
static void load_navigation_cb(lv_event_t *);
static lv_obj_t *scrSettingsDetail = nullptr;
static lv_obj_t *lblSettingsDetailTitle = nullptr;
static lv_obj_t *settingsDetailPanel = nullptr;
static void create_section_editor();
static void create_flow_calibration();
static void release_detail_editors();
static void close_quick_input();
static void quick_settings_refresh();
static void create_bench_test();
static void bench_refresh();
static void bench_end_cb(lv_event_t *);
static lv_obj_t *benchPanel = nullptr;
static lv_obj_t *benchStatus = nullptr;
static lv_obj_t *benchSpeed = nullptr;
static lv_obj_t *benchFlow = nullptr;
static lv_obj_t *benchTarget = nullptr;
static lv_obj_t *benchReg = nullptr;
static lv_obj_t *benchAuto = nullptr;
static lv_obj_t *regulatorPanel=nullptr, *regulatorValues[3]={}, *regulatorMessage=nullptr;
static RegulatorSettings regulatorDraft;
static void create_regulator_settings();
static void create_valve_test();
static lv_obj_t *benchSections[SECTION_COUNT] = {};
static lv_obj_t *simulationBar = nullptr;
static const char *benchMessage = "Nacisnij START TESTU";

static lv_color_t C_BG()      { return lv_color_hex(0x06131D); }
static lv_color_t C_PANEL()   { return lv_color_hex(0x142A39); }
static lv_color_t C_BORDER()  { return lv_color_hex(0x38586C); }
static lv_color_t C_GREEN()   { return lv_color_hex(0x46E58A); }
static lv_color_t C_BLUE()    { return lv_color_hex(0x46E58A); }
static lv_color_t C_TEXT()    { return lv_color_hex(0xF2F7FA); }
static lv_color_t C_MUTED()   { return lv_color_hex(0x90A4B0); }
static lv_color_t C_RED()     { return lv_color_hex(0xFF4B55); }
static lv_color_t C_ORANGE()  { return lv_color_hex(0xFFB121); }
static lv_color_t C_YELLOW()  { return lv_color_hex(0xF6D33B); }

static lv_obj_t *panel(
    lv_obj_t *parent,
    int x, int y, int w, int h,
    lv_color_t border
)
{
    lv_obj_t *o = lv_obj_create(parent);

    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_scrollable(o, false);

    lv_obj_set_style_radius(o, 10, 0);
    lv_obj_set_style_bg_color(o, C_PANEL(), 0);
    lv_obj_set_style_bg_grad_color(o, lv_color_hex(0x081A27), 0);
    lv_obj_set_style_bg_grad_dir(o, LV_GRAD_DIR_NONE, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(o, border, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_pad_all(o, 0, 0);

    return o;
}

static lv_obj_t *label(
    lv_obj_t *parent,
    const char *text,
    int x, int y,
    lv_color_t color
)
{
    lv_obj_t *l = lv_label_create(parent);
    set_label_text_if_changed(l, text);
    lv_obj_set_pos(l, x, y);
    set_text_color_if_changed(l, color, 0);
    return l;
}


static void font16(lv_obj_t *obj)
{
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, 0);
}

static void font20(lv_obj_t *obj)
{
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_20, 0);
}

static void font24(lv_obj_t *obj)
{
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, 0);
}

static void font28(lv_obj_t *obj)
{
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_28, 0);
}

static lv_obj_t *button(
    lv_obj_t *parent,
    int x, int y, int w, int h,
    const char *text,
    lv_event_cb_t cb,
    void *user = nullptr
)
{
    lv_obj_t *b = lv_button_create(parent);

    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);

    lv_obj_set_style_radius(b, 9, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x172C3A), 0);
    lv_obj_set_style_border_color(b, C_BORDER(), 0);
    lv_obj_set_style_border_width(b, 2, 0);
    lv_obj_set_style_pad_all(b, 4, 0);

    lv_obj_t *l = lv_label_create(b);
    set_label_text_if_changed(l, text);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    font16(l);
    lv_obj_center(l);

    if (cb) {
        lv_obj_add_event_cb(
            b,
            cb,
            LV_EVENT_CLICKED,
            user
        );
    }

    return b;
}

static void set_button_state(lv_obj_t *b, bool active)
{
    if (!b) return;
    const bool mode = b == btnAuto || b == btnMan;
    bool section = false;
    for (int i=0; i<SECTION_COUNT; ++i)
        if (b==btnSections[i] || b==sprayerSections[i]) section=true;
    const bool manual = b==btnMan || b==btnValveOpen || b==btnValveClose || (section && !sprayer_display_state().autoMode);
    const lv_color_t top = active ? lv_color_hex(manual ? (mode ? 0xB89216 : 0x806619) : (mode ? 0x1BC76A : 0x138A47)) : lv_color_hex(0x172C3A);
    const lv_color_t bottom = active ? lv_color_hex(manual ? 0x705513 : (mode ? 0x0B8844 : 0x07562D)) : lv_color_hex(0x10212E);
    const lv_color_t border = active ? (manual ? C_YELLOW() : C_GREEN()) : C_BORDER();
    if (lv_obj_get_child_count(b) > 1) {
        lv_obj_t *dot = lv_obj_get_child(b, 1);
        const lv_color_t dotColor = active ? (manual ? C_YELLOW() : lv_color_hex(0x62FF86)) : lv_color_hex(0x607585);
        if (!lv_color_eq(lv_obj_get_style_bg_color(dot, LV_PART_MAIN), dotColor))
            lv_obj_set_style_bg_color(dot, dotColor, 0);
    }
    if (lv_color_eq(lv_obj_get_style_bg_color(b, LV_PART_MAIN), top) &&
        lv_color_eq(lv_obj_get_style_bg_grad_color(b, LV_PART_MAIN), bottom) &&
        lv_color_eq(lv_obj_get_style_border_color(b, LV_PART_MAIN), border)) return;
    lv_obj_set_style_bg_color(b, top, 0);
    lv_obj_set_style_bg_grad_color(b, bottom, 0);
    lv_obj_set_style_bg_grad_dir(b, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(b, border, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
}

static void rate_minus_cb(lv_event_t *)
{
    sprayer_display_state().rateLHa -= 10;
    if (sprayer_display_state().rateLHa < 10) {
        sprayer_display_state().rateLHa = 10;
    }
    ui_update();
}

static void rate_plus_cb(lv_event_t *)
{
    sprayer_display_state().rateLHa += 10;
    if (sprayer_display_state().rateLHa > 1000) {
        sprayer_display_state().rateLHa = 1000;
    }
    ui_update();
}

static void auto_cb(lv_event_t *)
{
    if (bench_test_active()) return;
    auto_regulator_stop();
    valve_test_stop();
    for (int i=0; i<SECTION_COUNT; ++i) {
        if (sprayer_display_state().sections[i]) {
            nozzle_section_change_at(millis());
            break;
        }
    }
    for (int i=0; i<SECTION_COUNT; ++i) sprayer_display_state().sections[i]=false;
    sprayer_display_state().autoMode = true;
    prevSectionStateValid=false;
    aog_send_mode_now();
    ui_update();
}

static void man_cb(lv_event_t *)
{
    if (bench_test_active()) return;
    auto_regulator_stop();
    valve_test_stop();
    for (int i=0; i<SECTION_COUNT; ++i) {
        if (sprayer_display_state().sections[i]) {
            nozzle_section_change_at(millis());
            break;
        }
    }
    for (int i=0; i<SECTION_COUNT; ++i) sprayer_display_state().sections[i]=false;
    sprayer_display_state().autoMode = false;
    prevSectionStateValid=false;
    aog_send_mode_now();
    ui_update();
}

static void manual_valve_cb(lv_event_t *e)
{
    if(bench_test_active() || sprayer_display_state().autoMode || lv_screen_active()!=scrHome)return;
    valve_manual_pulse((int)(intptr_t)lv_event_get_user_data(e));
    ui_update();
}
static void manual_valve_stop_cb(lv_event_t *) { auto_regulator_stop();valve_test_stop();ui_update(); }

static void section_cb(lv_event_t *e)
{
    if (sprayer_display_state().autoMode) {
        return;
    }

    int i =
        (int)(intptr_t)
        lv_event_get_user_data(e);

    if (i < 0 || i >= SECTION_COUNT) {
        return;
    }

    nozzle_section_change_at(millis());
    sprayer_display_state().sections[i] = !sprayer_display_state().sections[i];

    // Natychmiastowa reakcja wizualna - bez czekania na ui_update().
    if (btnSections[i]) {
        set_button_state(btnSections[i], sprayer_display_state().sections[i]);
    }

    if (sprayerSections[i]) {
        set_button_state(sprayerSections[i], sprayer_display_state().sections[i]);
    }

    prevSectionState[i] = sprayer_display_state().sections[i];
    prevSectionStateValid = true;

    aog_send_switch_now();
}


static void sprayer_section_cb(lv_event_t *e)
{
    if (sprayer_display_state().autoMode) {
        return;
    }

    int i =
        (int)(intptr_t)
        lv_event_get_user_data(e);

    if (i < 0 || i >= SECTION_COUNT) {
        return;
    }

    sprayer_display_state().sections[i] = !sprayer_display_state().sections[i];

    // Natychmiastowa reakcja wizualna - bez czekania na ui_update().
    if (sprayerSections[i]) {
        set_button_state(sprayerSections[i], sprayer_display_state().sections[i]);
    }

    if (btnSections[i]) {
        set_button_state(btnSections[i], sprayer_display_state().sections[i]);
    }

    prevSectionState[i] = sprayer_display_state().sections[i];
    prevSectionStateValid = true;

    aog_send_switch_now();
}

static void all_cb(lv_event_t *)
{
    if (sprayer_display_state().autoMode) {
        return;
    }

    bool anyOff = false;

    for (int i = 0; i < SECTION_COUNT; i++) {
        if (!sprayer_display_state().sections[i]) {
            anyOff = true;
        }
    }

    // All-on or all-off changes at least one section.
    nozzle_section_change_at(millis());
    for (int i = 0; i < SECTION_COUNT; i++) {
        sprayer_display_state().sections[i] = anyOff;

        if (btnSections[i]) {
            set_button_state(btnSections[i], anyOff);
        }

        if (sprayerSections[i]) {
            set_button_state(sprayerSections[i], anyOff);
        }

        prevSectionState[i] = anyOff;
    }

    prevSectionStateValid = true;

    aog_send_switch_now();
}



static float section_total_width_m()
{
    float total = 0.0f;

    for (int i = 0; i < SECTION_COUNT; i++) {
        total += sectionWidthM[i];
    }

    return total;
}

static lv_obj_t *sectionWidthLabels[SECTION_COUNT] = {};

static void section_editor_refresh()
{
    char buf[48];

    if (sectionEditorTitle) {
        snprintf(
            buf,
            sizeof(buf),
            "SEKCJA %d",
            sectionEditIndex + 1
        );
        set_label_text_if_changed(sectionEditorTitle, buf);
    }

    if (sectionEditorValue) {
        snprintf(
            buf,
            sizeof(buf),
            "%.1f m",
            sectionWidthM[sectionEditIndex]
        );
        set_label_text_if_changed(sectionEditorValue, buf);
    }

    const float total = section_total_width_m();

    if (sectionEditorTotal) {
        snprintf(
            buf,
            sizeof(buf),
            "SZEROKOSC BELKI: %.1f m",
            total
        );
        set_label_text_if_changed(sectionEditorTotal, buf);
    }

    if (lblBoomWidth) {
        snprintf(
            buf,
            sizeof(buf),
            "BELKA %.1f m",
            total
        );
        set_label_text_if_changed(lblBoomWidth, buf);
    }

    // Aktualizuj napisy na przyciskach sekcji na ekranie glownym:
    // numer sekcji + aktualna szerokosc z ustawien.
    for (int i = 0; i < SECTION_COUNT; i++) {
        if (!btnSections[i]) {
            continue;
        }

        lv_obj_t *btnLabel = lv_obj_get_child(btnSections[i], 0);

        if (!btnLabel) {
            continue;
        }

        if (sectionWidthLabels[i]) {
            snprintf(buf, sizeof(buf), "%d", i+1);
            set_label_text_if_changed(btnLabel, buf);
            snprintf(buf, sizeof(buf), "%.1f m", sectionWidthM[i]);
            set_label_text_if_changed(sectionWidthLabels[i], buf);
        } else {
            snprintf(buf, sizeof(buf), "%d\n%.1f m", i+1, sectionWidthM[i]);
            set_label_text_if_changed(btnLabel, buf);
        }
    }
}

static void section_settings_load()
{
    sectionPrefs.begin("agrosec", true);

    for (int i = 0; i < SECTION_COUNT; i++) {
        char key[8];
        snprintf(key, sizeof(key), "sec%d", i + 1);

        float v = sectionPrefs.getFloat(key, sectionWidthM[i]);

        if (v >= 0.1f && v <= 20.0f) {
            sectionWidthM[i] = v;
        }
    }

    sectionPrefs.end();
}

static void section_prev_cb(lv_event_t *)
{
    sectionEditIndex--;

    if (sectionEditIndex < 0) {
        sectionEditIndex = SECTION_COUNT - 1;
    }

    section_editor_refresh();
}

static void section_next_cb(lv_event_t *)
{
    sectionEditIndex++;

    if (sectionEditIndex >= SECTION_COUNT) {
        sectionEditIndex = 0;
    }

    section_editor_refresh();
}

static void section_minus_len_cb(lv_event_t *)
{
    sectionWidthM[sectionEditIndex] -= 0.1f;

    if (sectionWidthM[sectionEditIndex] < 0.1f) {
        sectionWidthM[sectionEditIndex] = 0.1f;
    }

    sectionWidthM[sectionEditIndex] =
        ((int)(sectionWidthM[sectionEditIndex] * 10.0f + 0.5f)) / 10.0f;

    section_editor_refresh();
}

static void section_plus_len_cb(lv_event_t *)
{
    sectionWidthM[sectionEditIndex] += 0.1f;

    if (sectionWidthM[sectionEditIndex] > 20.0f) {
        sectionWidthM[sectionEditIndex] = 20.0f;
    }

    sectionWidthM[sectionEditIndex] =
        ((int)(sectionWidthM[sectionEditIndex] * 10.0f + 0.5f)) / 10.0f;

    section_editor_refresh();
}

static void section_defaults_cb(lv_event_t *)
{
    const float defaults[SECTION_COUNT] = {
        2.5f, 4.0f, 4.0f, 4.0f, 4.0f, 2.5f
    };

    for (int i = 0; i < SECTION_COUNT; i++) {
        sectionWidthM[i] = defaults[i];
    }

    section_editor_refresh();
}

static void section_save_cb(lv_event_t *)
{
    sectionPrefs.begin("agrosec", false);

    for (int i = 0; i < SECTION_COUNT; i++) {
        char key[8];
        snprintf(key, sizeof(key), "sec%d", i + 1);
        sectionPrefs.putFloat(key, sectionWidthM[i]);
    }

    sectionPrefs.end();
    section_editor_refresh();
}


static void flow_cal_refresh()
{
    char buf[80];

    if (flowCalCurrent) {
        uint32_t ppl =
            flowmeter_get_pulses_per_liter_x100();

        snprintf(
            buf,
            sizeof(buf),
            "AKTUALNIE: %lu.%02lu imp/L",
            (unsigned long)(ppl / 100),
            (unsigned long)(ppl % 100)
        );

        set_label_text_if_changed(flowCalCurrent, buf);
    }

    if (flowCalVolume) {
        snprintf(
            buf,
            sizeof(buf),
            "OBJETOSC TESTU: %d.%d L",
            flowCalVolumeX10 / 10,
            abs(flowCalVolumeX10 % 10)
        );

        set_label_text_if_changed(flowCalVolume, buf);
    }

    uint32_t pulses = flowCalCapturedPulses;
    uint32_t elapsedMs = flowCalElapsedMs;

    if (flowCalRunning) {
        uint32_t nowPulses =
            flowmeter_get_total_pulses();

        pulses =
            nowPulses -
            flowCalStartPulses;

        elapsedMs =
            millis() -
            flowCalStartMs;
    }

    if (flowCalPulses) {
        snprintf(
            buf,
            sizeof(buf),
            "IMPULSY: %lu",
            (unsigned long)pulses
        );

        set_label_text_if_changed(flowCalPulses, buf);
    }

    if (flowCalTime) {
        uint32_t seconds = elapsedMs / 1000UL;

        snprintf(
            buf,
            sizeof(buf),
            "CZAS: %lu:%02lu",
            (unsigned long)(seconds / 60UL),
            (unsigned long)(seconds % 60UL)
        );

        set_label_text_if_changed(flowCalTime, buf);
    }

    if (flowCalAverage) {
        if (!flowCalRunning && elapsedMs > 0 && pulses > 0) {
            // volume L = volumeX10 / 10
            // avg L/min x10 = volumeX10 * 60000 / elapsedMs
            uint32_t avgX10 =
                (uint32_t)(
                    ((uint64_t)flowCalVolumeX10 *
                    60000ULL + elapsedMs / 2) /
                    (uint64_t)elapsedMs
                );

            snprintf(
                buf,
                sizeof(buf),
                "SREDNI PRZEPLYW: %lu.%01lu L/min",
                (unsigned long)(avgX10 / 10UL),
                (unsigned long)(avgX10 % 10UL)
            );
        } else {
            snprintf(
                buf,
                sizeof(buf),
                "SREDNI PRZEPLYW: --.- L/min"
            );
        }

        set_label_text_if_changed(flowCalAverage, buf);
    }

    if (flowCalLiters) {
        const uint32_t ppl = flowmeter_get_pulses_per_liter_x100();
        const uint64_t litersX10 = ppl ? ((uint64_t)pulses * 1000ULL + ppl / 2) / ppl : 0;
        snprintf(buf, sizeof(buf), "LITRY WG KALIBRACJI: %lu.%lu L",
                 (unsigned long)(litersX10 / 10), (unsigned long)(litersX10 % 10));
        set_label_text_if_changed(flowCalLiters, buf);
    }
    if (flowCalInstant) {
        snprintf(buf, sizeof(buf), "PRZEPLYW: %d.%d L/min",
                 sprayer_display_state().actualFlowX10 / 10, abs(sprayer_display_state().actualFlowX10 % 10));
        set_label_text_if_changed(flowCalInstant, buf);
    }
    if (flowCalStatus) {
        set_label_text_if_changed(flowCalStatus, flowCalRunning ? "TEST TRWA..." : flowCalMessage);
        set_text_color_if_changed(flowCalStatus,
            (flowCalRunning || flowCalSaved) ? C_GREEN() : C_ORANGE(), 0);
    }

    if (flowCalResult) {
        if (flowCalCandidateX100 > 0) {
            snprintf(
                buf,
                sizeof(buf),
                "WYNIK: %lu.%02lu imp/L",
                (unsigned long)(flowCalCandidateX100 / 100),
                (unsigned long)(flowCalCandidateX100 % 100)
            );
        } else {
            snprintf(
                buf,
                sizeof(buf),
                "WYNIK: --.-- imp/L"
            );
        }

        set_label_text_if_changed(flowCalResult, buf);
    }
}

static void flow_cal_start_cb(lv_event_t *)
{
    if (bench_test_active()) return;
    if (flowCalRunning) return;
    flowCalSaved = false;
    flowCalMessage = "GOTOWY";
    flowCalStartPulses =
        flowmeter_get_total_pulses();

    flowCalCapturedPulses = 0;
    flowCalCandidateX100 = 0;
    flowCalElapsedMs = 0;
    flowCalStartMs = millis();
    flowCalRunning = true;

    flow_cal_refresh();
}

static void flow_cal_stop_cb(lv_event_t *)
{
    if (!flowCalRunning) {
        return;
    }

    uint32_t now =
        flowmeter_get_total_pulses();

    flowCalCapturedPulses =
        now -
        flowCalStartPulses;

    flowCalElapsedMs =
        millis() -
        flowCalStartMs;

    flowCalRunning = false;
    flowCalMessage = flowCalCapturedPulses ? "WPISZ LITRY" : "BRAK IMPULSOW";
    flow_cal_refresh();
}

static void flow_cal_set_volume(int volumeX10)
{
    if (flowCalRunning) return;
    flowCalVolumeX10 = volumeX10;
    flowCalCandidateX100 = 0;
    flowCalSaved = false;
    flowCalMessage = "GOTOWY";
    flow_cal_refresh();
}

static void flow_cal_preset_cb(lv_event_t *e)
{
    flow_cal_set_volume((int)(intptr_t)lv_event_get_user_data(e));
}

static void flow_cal_input_open_cb(lv_event_t *)
{
    if (flowCalRunning) return;
    flowCalInput[0] = '\0';
    set_label_text_if_changed(flowCalInputLabel, "WPISZ RZECZYWISTE LITRY");
    lv_obj_set_hidden(flowCalInputPanel, false);
}

static void flow_cal_input_cancel_cb(lv_event_t *)
{
    lv_obj_set_hidden(flowCalInputPanel, true);
}

static void flow_cal_input_key_cb(lv_event_t *e)
{
    lv_obj_t *matrix = (lv_obj_t *)lv_event_get_target(e);
    const uint32_t id = lv_buttonmatrix_get_selected_button(matrix);
    if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;
    const char *key = lv_buttonmatrix_get_button_text(matrix, id);
    if (!key) return;
    size_t n = strlen(flowCalInput);
    if (strcmp(key, "USUN") == 0) {
        if (n) flowCalInput[n - 1] = '\0';
    } else if (n < sizeof(flowCalInput) - 1) {
        if (strcmp(key, ".") == 0 && strchr(flowCalInput, '.')) return;
        flowCalInput[n] = key[0];
        flowCalInput[n + 1] = '\0';
    }
    set_label_text_if_changed(flowCalInputLabel, n || flowCalInput[0] ? flowCalInput : "WPISZ LITRY");
}

static void flow_cal_input_accept_cb(lv_event_t *)
{
    char *end = nullptr;
    const double liters = strtod(flowCalInput, &end);
    if (end == flowCalInput || *end || !(liters >= 0.1 && liters <= 10000.0)) {
        set_label_text_if_changed(flowCalInputLabel, "ZAKRES: 0.1 - 10000 L");
        return;
    }
    flow_cal_set_volume((int)(liters * 10.0 + 0.5));
    lv_obj_set_hidden(flowCalInputPanel, true);
}

static void flow_cal_save_cb(lv_event_t *)
{
    if (flowCalRunning) return;
    flowCalSaved = false;
    flowCalCandidateX100 = 0;
    if (!flowCalCapturedPulses || !flowCalElapsedMs || flowCalVolumeX10 <= 0) {
        flowCalMessage = "BRAK TESTU";
        flow_cal_refresh();
        return;
    }
    const uint64_t candidate = ((uint64_t)flowCalCapturedPulses * 1000ULL +
                               flowCalVolumeX10 / 2) / flowCalVolumeX10;
    if (candidate < 10 || candidate > 100000) {
        flowCalMessage = "K POZA ZAKRESEM";
        flow_cal_refresh();
        return;
    }
    flowCalCandidateX100 = (uint32_t)candidate;
    flowCalSaved = flowmeter_set_pulses_per_liter_x100(flowCalCandidateX100);
    flowCalMessage = flowCalSaved ? "ZAPISANO" : "BLAD ZAPISU";
    flow_cal_refresh();
}

static void load_home_cb(lv_event_t *)
{
    if (scrHome) {
        lv_screen_load(scrHome);
        release_detail_editors();
        ui_update();
    }
}

static void load_settings_cb(lv_event_t *)
{
    if (scrSettings) {
        lv_screen_load(scrSettings);
        release_detail_editors();
    }
}

static void settings_detail_back_cb(lv_event_t *)
{
    if (scrSettings) {
        lv_screen_load(scrSettings);
        release_detail_editors();
    }
}

static void settings_detail_cb(lv_event_t *e)
{
    const char *title =
        (const char *)lv_event_get_user_data(e);

    if (lblSettingsDetailTitle && title) {
        set_label_text_if_changed(lblSettingsDetailTitle, title);
    }

    const bool benchMode = title && strcmp(title, "TEST NA BIURKU") == 0;
    const bool valveMode = title && strcmp(title, "CISNIENIE / REGULATOR") == 0;
    if (bench_test_active() && !benchMode) {
        release_detail_editors();
        set_label_text_if_changed(detailMsg, "Najpierw nacisnij ZAKONCZ TEST w pasku SYMULACJA.");
        set_label_text_if_changed(detailMsg2, "Kalibracja i zapis ustawien wymagaja zakonczenia testu.");
        lv_obj_set_hidden(detailMsg, false);
        lv_obj_set_hidden(detailMsg2, false);
        lv_screen_load(scrSettingsDetail);
        return;
    }
    set_label_text_if_changed(detailMsg, "Panel gotowy do podlaczenia funkcji kalibracji.");
    set_label_text_if_changed(detailMsg2, "Kolejny krok: dodamy realne pola, +/- oraz zapis do pamieci.");

    const bool sectionMode =
        title && strcmp(title, "SEKCJE ROBOCZE") == 0;

    const bool flowMode =
        title && strcmp(title, "PRZEPLYWOMIERZ") == 0;

    if (valveMode && !valvePanel) {
        release_detail_editors();
        create_valve_test();
    } else if (benchMode && !benchPanel) {
        release_detail_editors();
        create_bench_test();
    } else if (sectionMode && !sectionEditorPanel) {
        release_detail_editors();
        create_section_editor();
    } else if (flowMode && !flowCalPanel) {
        release_detail_editors();
        create_flow_calibration();
    } else if (!sectionMode && !flowMode && !benchMode && !valveMode) {
        release_detail_editors();
    }

    if (sectionEditorPanel) {
        if (sectionMode) {
            lv_obj_set_hidden(sectionEditorPanel, false);
        } else {
            lv_obj_set_hidden(sectionEditorPanel, true);
        }
    }

    if (flowCalPanel) {
        if (flowMode) {
            lv_obj_set_hidden(flowCalPanel, false);
        } else {
            lv_obj_set_hidden(flowCalPanel, true);
        }
    }

    if (detailMsg) {
        if (sectionMode || flowMode || benchMode || valveMode) {
            lv_obj_set_hidden(detailMsg, true);
        } else {
            lv_obj_set_hidden(detailMsg, false);
        }
    }

    if (detailMsg2) {
        if (sectionMode || flowMode || benchMode || valveMode) {
            lv_obj_set_hidden(detailMsg2, true);
        } else {
            lv_obj_set_hidden(detailMsg2, false);
        }
    }

    if (sectionMode) {
        section_editor_refresh();
    }

    if (flowMode) {
        flow_cal_refresh();
    }

    if (benchMode) bench_refresh();
    if (valveMode) valve_refresh();

    if (scrSettingsDetail) {
        lv_screen_load(scrSettingsDetail);
    }
}

static lv_obj_t *nav_button(
    lv_obj_t *parent,
    int x,
    int w,
    const char *text,
    bool active,
    lv_event_cb_t cb
)
{
    lv_obj_t *b = lv_button_create(parent);

    lv_obj_set_pos(b, x, 5);
    lv_obj_set_size(b, w, 40);

    lv_obj_set_style_radius(b, 8, 0);
    lv_obj_set_style_bg_color(
        b,
        active
            ? lv_color_hex(0x0C5F8A)
            : lv_color_hex(0x06131C),
        0
    );

    lv_obj_set_style_border_color(
        b,
        active
            ? C_BLUE()
            : lv_color_hex(0x164F69),
        0
    );

    lv_obj_set_style_border_width(
        b,
        active ? 1 : 0,
        0
    );

    lv_obj_set_style_pad_all(b, 0, 0);

    lv_obj_t *l = lv_label_create(b);
    set_label_text_if_changed(l, text);
    font16(l);
    lv_obj_center(l);

    if (cb) {
        lv_obj_add_event_cb(
            b,
            cb,
            LV_EVENT_CLICKED,
            nullptr
        );
    }

    return b;
}

static lv_obj_t *settings_tile(
    lv_obj_t *parent,
    int x, int y,
    int w, int h,
    const char *title,
    const char *subtitle,
    lv_color_t accent
)
{
    lv_obj_t *b = lv_button_create(parent);

    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);

    lv_obj_set_style_radius(b, 14, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x081A25), 0);
    lv_obj_set_style_border_color(b, C_BORDER(), 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_pad_all(b, 0, 0);

    const char *icon = strstr(title, "SEKCJE") ? LV_SYMBOL_LIST :
                       strstr(title, "PRZEPLYW") ? LV_SYMBOL_TINT :
                       strstr(title, "ZBIORNIKA") ? LV_SYMBOL_CHARGE : LV_SYMBOL_SETTINGS;
    lv_obj_t *badge = panel(b, 20, 20, 50, 50, C_BORDER());
    lv_obj_t *glyph = label(badge, icon, 0, 0, accent);
    font24(glyph); lv_obj_center(glyph);
    lv_obj_t *t = label(
        b,
        title,
        84, 30,
        C_TEXT()
    );
    font24(t);

    lv_obj_t *s = label(
        b,
        subtitle,
        20, 76,
        C_MUTED()
    );
    font16(s);

    lv_obj_set_width(s, w - 40);
    lv_label_set_long_mode(s, LV_LABEL_LONG_WRAP);
    lv_obj_t *open = label(b, "OTWORZ  " LV_SYMBOL_RIGHT, 20, h - 40, accent);
    font16(open);
    lv_obj_add_event_cb(
        b,
        settings_detail_cb,
        LV_EVENT_CLICKED,
        (void *)title
    );

    return b;
}

// All geometry stays within the fixed 1024 x 600 viewport.
static lv_obj_t *flowActualBar = nullptr, *flowTargetBar = nullptr;
static lv_obj_t *lblFlowCompare = nullptr, *lblActiveWidth = nullptr;
static int previousActualBar = -1, previousTargetBar = -1;

static lv_obj_t *dashboard_bar(lv_obj_t *parent, int x, int y, lv_color_t color)
{
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_pos(bar, x, y);
    lv_obj_set_size(bar, 232, 14);
    lv_bar_set_range(bar, 0, 1000);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x203644), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);
    return bar;
}

// One drawing widget avoids allocating an LVGL object for every nozzle and brace.
static lv_obj_t *sprayerScene = nullptr;
static bool sceneStates[SECTION_COUNT] = {};
static float sceneWidths[SECTION_COUNT] = {};
static bool sceneStateValid = false;
static bool sceneManual = false;

static void scene_line(lv_layer_t *layer, int x, int y, int x2, int y2,
                       lv_color_t color, int width = 1)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.p1.x = x; d.p1.y = y; d.p2.x = x2; d.p2.y = y2;
    d.color = color; d.width = width;
    lv_draw_line(layer, &d);
}

static void scene_rect(lv_layer_t *layer, int x, int y, int w, int h,
                       uint32_t fill, int radius = 2)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(fill); d.bg_opa = LV_OPA_COVER;
    d.radius = radius;
    d.border_width = 1; d.border_color = lv_color_hex(0xA4BCCB);
    lv_area_t area = {x, y, x + w - 1, y + h - 1};
    lv_draw_rect(layer, &d, &area);
}

static void sprayer_scene_draw(lv_event_t *e)
{
    lv_layer_t *layer=lv_event_get_layer(e);
    lv_area_t a;lv_obj_get_coords(sprayerScene,&a);
    const int x=a.x1,y=a.y1,w=lv_obj_get_width(sprayerScene),cx=x+w/2;
    float total=0;for(int i=0;i<SECTION_COUNT;++i)total+=sceneWidths[i];
    if(total<=0)return;
    // Two tapered steel wings. Section colour is an accent on the spray rail.
    float sum=0;int left=6;
    const lv_color_t active=sceneManual?C_YELLOW():C_GREEN();
    const int mid=w/2;
    auto upper=[&](int pos){return y+20+abs(pos-mid)*10/(mid-6);};
    for(int i=0;i<SECTION_COUNT;++i){
        sum+=sceneWidths[i];const int right=6+(int)((w-12)*sum/total+.5f);
        const lv_color_t accent=sceneStates[i]?active:lv_color_hex(0x415968);
        const lv_color_t steel=lv_color_hex(sceneStates[i]?0x94B4C4:0x4F6878);
        auto wing=[&](int l,int r){
            if(r-l<6)return;
            // Dark edge under the steel gives a distinct outer profile.
            scene_line(layer,x+l,upper(l),x+r,upper(r),lv_color_hex(0x243E4D),4);
            scene_line(layer,x+l,upper(l),x+r,upper(r),steel,2);
            scene_line(layer,x+l,y+38,x+r,y+38,lv_color_hex(0x243E4D),4);
            scene_line(layer,x+l,y+38,x+r,y+38,accent,2);
            for(int cell=0;cell<4;++cell){
                const int l0=l+(r-l)*cell/4,r0=l+(r-l)*(cell+1)/4;
                scene_line(layer,x+l0,y+37,x+r0,upper(r0)+1,steel);
                scene_line(layer,x+l0,upper(l0)+1,x+r0,y+37,steel);
            }
            // A folding joint separates each working segment.
            scene_line(layer,x+l,upper(l),x+l,y+38,lv_color_hex(0xBCD0DA),2);
            scene_rect(layer,x+l-2,y+33,5,7,0x39586B,2);
        };
        // The central cradle hides the rails behind the tank and tyres.
        wing(left,right<mid-53?right-2:mid-53);
        wing(left>mid+53?left:mid+53,right-2);
        for(int n=0;n<3;++n){
            const int pos=left+(right-left)*(2*n+1)/6;
            if(abs(pos-mid)<54)continue;
            const int nx=x+pos;
            scene_line(layer,nx,y+39,nx,y+42,lv_color_hex(0xBCD0DA),2);
            scene_rect(layer,nx-2,y+42,5,3,0x4D7F98,1);
            if(sceneStates[i]){
                lv_draw_triangle_dsc_t cone;lv_draw_triangle_dsc_init(&cone);
                const int half=(right-left)/16<8?(right-left)/16:8;
                cone.p[0]={nx,y+45};cone.p[1]={nx-half,y+52};cone.p[2]={nx+half,y+52};
                cone.color=active;cone.opa=LV_OPA_30;lv_draw_triangle(layer,&cone);
            }
        }
        left=right;
    }
    // Bracing links and reinforced root joints attach both wings to the cradle.
    scene_line(layer,cx-54,y+23,cx-111,y+34,lv_color_hex(0x6A8EA4),3);
    scene_line(layer,cx+54,y+23,cx+111,y+34,lv_color_hex(0x6A8EA4),3);
    scene_rect(layer,cx-59,y+29,7,13,0x42697F,2);
    scene_rect(layer,cx+52,y+29,7,13,0x42697F,2);
    // Rear view with a tall rounded tank, distinct tyres and a visible axle.
    // Recessed tyre centres and restrained highlights avoid a flat box shape.
    scene_rect(layer,cx-52,y+17,18,36,0x08141D,7);
    scene_rect(layer,cx+34,y+17,18,36,0x08141D,7);
    for(int n=0;n<5;++n){
        scene_line(layer,cx-49,y+23+n*6,cx-39,y+20+n*6,lv_color_hex(0x425966),2);
        scene_line(layer,cx+39,y+20+n*6,cx+49,y+23+n*6,lv_color_hex(0x425966),2);
    }
    scene_rect(layer,cx-41,y+34,82,13,0x27485B,4);
    scene_rect(layer,cx-35,y+4,70,37,0x6FADBF,17);
    scene_rect(layer,cx-29,y+7,58,12,0xBDE0E5,9);
    scene_line(layer,cx-27,y+28,cx+27,y+28,lv_color_hex(0x4C859B),2);
    scene_line(layer,cx-29,y+33,cx+29,y+33,lv_color_hex(0x3F728B),3);
    scene_rect(layer,cx-10,y+1,20,5,0x32586B,2);
    // Two bands retain the tank on its steel cradle.
    scene_line(layer,cx-20,y+7,cx-20,y+40,lv_color_hex(0x3A677B),3);
    scene_line(layer,cx+20,y+7,cx+20,y+40,lv_color_hex(0x3A677B),3);
    scene_line(layer,cx-26,y+41,cx+26,y+41,lv_color_hex(0x89B3C3),2);
    // Central rear valve block, plumbing and small tail reflectors.
    scene_rect(layer,cx-8,y+39,16,10,0x2C536A,3);
    scene_rect(layer,cx-4,y+41,8,5,0x8DC0D0,2);
    scene_line(layer,cx-19,y+44,cx-9,y+44,lv_color_hex(0x7FA3B7),2);
    scene_line(layer,cx+9,y+44,cx+19,y+44,lv_color_hex(0x7FA3B7),2);
    scene_rect(layer,cx-31,y+43,7,4,0xE9994B,1);
    scene_rect(layer,cx+24,y+43,7,4,0xE9994B,1);
    scene_line(layer,cx-25,y+50,cx+25,y+50,lv_color_hex(0x496E85),2);
}

static void refresh_sprayer_scene()
{
    if (!sprayerScene) return;
    const bool manual = !sprayer_display_state().autoMode;
    bool changed = !sceneStateValid || sceneManual!=manual;
    sceneManual=manual;
    for (int i=0;i<SECTION_COUNT;++i) {
        const bool on = sprayer_display_state().sections[i];
        if (sceneStates[i] != on || sceneWidths[i] != sectionWidthM[i]) changed = true;
        sceneStates[i] = on; sceneWidths[i] = sectionWidthM[i];
    }
    sceneStateValid = true;
    if (changed) lv_obj_invalidate(sprayerScene);
}

static lv_obj_t *home_text(lv_obj_t *parent, const char *text, int x, int y,
                           lv_color_t color, const lv_font_t *font)
{
    lv_obj_t *o = label(parent, text, x, y, color);
    lv_obj_set_style_text_font(o, font, 0);
    return o;
}
static bool flowWithinTarget = false;
static lv_obj_t *flowStatusIcon = nullptr;

static void home_icon_draw(lv_event_t *e)
{
    lv_obj_t *o = (lv_obj_t *)lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t a; lv_obj_get_coords(o, &a);
    const int x=a.x1, y=a.y1, n=lv_obj_get_width(o);
    const int kind=(int)(intptr_t)lv_event_get_user_data(e);
    lv_draw_arc_dsc_t arc; lv_draw_arc_dsc_init(&arc);
    arc.center={x+n/2,y+n/2}; arc.radius=n/2-2; arc.width=2;
    arc.color=C_GREEN(); arc.rounded=1;
    if (kind==5) {
        arc.start_angle=0; arc.end_angle=360;
        arc.color=flowWithinTarget ? C_GREEN() : C_ORANGE();
        lv_draw_arc(layer,&arc);
        if (flowWithinTarget) {
            scene_line(layer,x+n/4,y+n/2,x+n*2/5,y+n*2/3,C_GREEN(),3);
            scene_line(layer,x+n*2/5,y+n*2/3,x+n*3/4,y+n/3,C_GREEN(),3);
        } else {
            scene_line(layer,x+n/2,y+n/4,x+n/2,y+n*3/5,C_ORANGE(),3);
            scene_rect(layer,x+n/2-1,y+n*3/4,3,3,0xFFB121,1);
        }
    } else if (kind==2) { // Gauge.
        arc.start_angle=155;arc.end_angle=385;lv_draw_arc(layer,&arc);
        scene_line(layer,x+n/2,y+n/2,x+n*3/4,y+n/4,C_GREEN(),3);
        scene_rect(layer,x+n/2-2,y+n/2-2,4,4,0x46E58A,2);
    } else if (kind==4) { // Tank.
        scene_rect(layer,x+n/4,y+2,n/2,n-4,0x436377,4);
        scene_line(layer,x+n/3,y,x+n*2/3,y,C_MUTED(),2);
        scene_line(layer,x+n/2,y,x+n/2,y+3,C_MUTED(),2);
    } else if (kind==3) { // Folded map.
        scene_line(layer,x+2,y+6,x+n/3,y+2,C_GREEN(),2);
        scene_line(layer,x+n/3,y+2,x+n*2/3,y+6,C_GREEN(),2);
        scene_line(layer,x+n*2/3,y+6,x+n-2,y+2,C_GREEN(),2);
        scene_line(layer,x+2,y+6,x+2,y+n-3,C_GREEN(),2);
        scene_line(layer,x+n/3,y+2,x+n/3,y+n-7,C_GREEN(),2);
        scene_line(layer,x+n*2/3,y+6,x+n*2/3,y+n-3,C_GREEN(),2);
        scene_line(layer,x+n-2,y+2,x+n-2,y+n-7,C_GREEN(),2);
        scene_line(layer,x+2,y+n-3,x+n/3,y+n-7,C_GREEN(),2);
        scene_line(layer,x+n/3,y+n-7,x+n*2/3,y+n-3,C_GREEN(),2);
        scene_line(layer,x+n*2/3,y+n-3,x+n-2,y+n-7,C_GREEN(),2);
    } else {
        lv_draw_triangle_dsc_t t;lv_draw_triangle_dsc_init(&t);
        t.color=C_GREEN();
        if(kind==1) { // Drop.
            t.p[0]={x+n/2,y+1}; t.p[1]={x+3,y+n*2/3};t.p[2]={x+n-3,y+n*2/3};
            lv_draw_triangle(layer,&t);
            scene_rect(layer,x+3,y+n/2,n-6,n/2,0x46E58A,n/3);
            scene_line(layer,x+6,y+n*2/3,x+8,y+n*4/5,lv_color_hex(0x99FFBE),2);
        } else { // Twin leaves and stem.
            t.p[0]={x+n/2,y+n-4};t.p[1]={x+n/2,y+n/3};t.p[2]={x+n-1,y+1};lv_draw_triangle(layer,&t);
            t.color=lv_color_hex(0x34BC73);
            t.p[0]={x+n/2,y+n-4};t.p[1]={x+1,y+n/3};t.p[2]={x+4,y+n*3/4};lv_draw_triangle(layer,&t);
            scene_line(layer,x+n/2,y+n/2,x+n/2,y+n,C_GREEN(),2);
        }
    }
}
static lv_obj_t *home_icon(lv_obj_t *parent,int x,int y,int size,int kind)
{
    lv_obj_t *o=panel(parent,x,y,size,size,C_PANEL());
    lv_obj_set_style_bg_opa(o,LV_OPA_TRANSP,0);lv_obj_set_style_border_width(o,0,0);
    lv_obj_set_clickable(o,false);
    lv_obj_add_event_cb(o,home_icon_draw,LV_EVENT_DRAW_MAIN,(void *)(intptr_t)kind);
    return o;
}
static lv_obj_t *home_control(lv_obj_t *parent,int x,int y,int w,int h,
                              const char *text,lv_event_cb_t cb)
{
    lv_obj_t *b=button(parent,x,y,w,h,text,cb);
    lv_obj_set_style_text_font(lv_obj_get_child(b,0),&home_font_24,0);
    lv_obj_set_style_bg_grad_dir(b,LV_GRAD_DIR_NONE,0);
    lv_obj_set_style_bg_grad_color(b,lv_color_hex(0x10212E),0);
    lv_obj_set_style_border_width(b,1,0);
    return b;
}
static void fit_home_number(lv_obj_t *obj, const char *text)
{
    int width=0;
    const lv_font_t *font=&dashboard_font_48;
    for(const char *p=text;*p;++p) width+=lv_font_get_glyph_width(font,*p,*(p+1));
    const int available=lv_obj_get_style_width(obj,LV_PART_MAIN);
    if(available>0 && width>available) font=&home_font_32;
    if(lv_obj_get_style_text_font(obj,LV_PART_MAIN)!=font) lv_obj_set_style_text_font(obj,font,0);
}

static void dashboard_metric(lv_obj_t *scr, int x, const char *title,
                             lv_obj_t **value, const char *unit, const char *)
{
    const int width = x==14 ? 212 : x==232 ? 198 : 194;
    lv_obj_t *card=panel(scr,x,46,width,122,C_BORDER());
    home_icon(card,14,13,26,strstr(title,"DAWKA") ? 0 : strstr(title,"PRZEPLYW") ? 1 : 2);
    home_text(card,title,46,x==14 ? 12 : 16,C_MUTED(),&lv_font_montserrat_16);
    *value=home_text(card,"--",0,x==14 ? 38 : 32,C_TEXT(),&dashboard_font_48);
    lv_obj_set_width(*value,width-16);
    lv_obj_set_height(*value,x==14 ? 54 : 60);
    lv_label_set_long_mode(*value,LV_LABEL_LONG_CLIP);
    lv_obj_set_x(*value,8);lv_obj_set_style_text_align(*value,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_t *units=home_text(card,unit,0,94,C_MUTED(),&home_font_18);
    lv_obj_set_width(units,width-2);lv_obj_set_style_text_align(units,LV_TEXT_ALIGN_CENTER,0);
}

static void load_navigation_cb(lv_event_t *)
{
    if (scrNavigation) {
        lv_screen_load(scrNavigation);
        release_detail_editors();
    }
}

static void dashboard_navigation(lv_obj_t *scr, int selected)
{
    lv_obj_t *dock = panel(scr, 14, selected==0 ? 548 : 536, 996, selected==0 ? 42 : 54, C_BORDER());
    const char *names[] = {LV_SYMBOL_HOME "  GLOWNA", LV_SYMBOL_GPS "  NAWIGACJA",
                           LV_SYMBOL_SETTINGS "  USTAWIENIA"};
    lv_event_cb_t actions[] = {load_home_cb, load_navigation_cb, load_settings_cb};
    for (int i = 0; i < 3; ++i) {
        lv_obj_t *b = button(dock, i * 332 + 4, 2, 328, selected==0 ? 36 : 48, names[i], actions[i]);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_set_style_text_font(lv_obj_get_child(b,0), &home_font_18, 0);
        if (selected == i) { lv_obj_t *line=panel(dock,i*332+22,selected==0 ? 38 : 50,288,2,C_GREEN()); lv_obj_set_style_bg_color(line,C_GREEN(),0); }
        lv_obj_set_style_bg_color(b, selected == i ? lv_color_hex(0x163D31) : C_PANEL(), 0);
        lv_obj_set_style_text_color(b, selected == i ? C_GREEN() : C_MUTED(), 0);
    }
}

static lv_obj_t *quickWidths[SECTION_COUNT] = {};
static lv_obj_t *quickTotal = nullptr, *quickMessage = nullptr;
static lv_obj_t *quickInputs[32] = {}; static int quickInputCount = 0;
static bool quickWidthsDirty = false;
static const char *quickStatus = nullptr;
static uint32_t quickStatusUntil = 0;
static lv_obj_t *nozzlePanel=nullptr,*nozzleValues[8]={},*nozzleSelect=nullptr,*liquidSelect=nullptr,*nozzleMessage=nullptr,*homeProfile=nullptr;
static lv_obj_t *nozzleHomeInfo=nullptr;
static NozzleSettings nozzleDraft;
static bool nozzleDirty=false;
static void nozzle_editor_refresh();
static void nozzle_open_cb(lv_event_t *);
static lv_obj_t *quickModal = nullptr, *quickText = nullptr, *quickError = nullptr;
static int quickField = -1;
static char quickEntry[8] = {};
static char quickInitial[24] = {};

static void close_quick_input()
{
    if (quickModal) lv_obj_delete(quickModal);
    quickModal = quickText = quickError = nullptr;
}
static void quick_settings_refresh()
{
    char text[48];
    for (int i=0; i<SECTION_COUNT; ++i) {
        snprintf(text,sizeof(text),"%.1f",sectionWidthM[i]);
        set_label_text_if_changed(quickWidths[i],text);
    }
    snprintf(text,sizeof(text),"BELKA: %.1f m",section_total_width_m());
    set_label_text_if_changed(quickTotal,text);
    set_label_text_if_changed(quickMessage,bench_test_active() ? "Zakoncz symulacje, aby zmieniac ustawienia." :
        quickStatus && (int32_t)(quickStatusUntil-millis())>0 ? quickStatus :
        quickWidthsDirty ? "Zmienione szerokosci - nacisnij ZAPISZ." : "Dotknij liczby, aby wpisac wartosc.");
    for(int i=0;i<quickInputCount;++i) {
        const bool disabled=bench_test_active();
        if(lv_obj_has_state(quickInputs[i],LV_STATE_DISABLED)!=disabled) {
            if(disabled) lv_obj_add_state(quickInputs[i],LV_STATE_DISABLED);
            else lv_obj_remove_state(quickInputs[i],LV_STATE_DISABLED);
        }
    }
}
static void quick_commit_value(float value)
{
    quickStatus=nullptr;
    if(quickField>=0 && quickField<SECTION_COUNT) {
        sectionWidthM[quickField]=((int)(value*10+.5f))/10.0f;
        quickWidthsDirty=true;
        section_editor_refresh(); refresh_sprayer_scene();
    } else if(quickField==SECTION_COUNT) gState.rateLHa=(int)(value+.5f);
    quick_settings_refresh();
}
static void quick_accept_cb(lv_event_t *)
{
    if(bench_test_active()) {close_quick_input();return;}
    char text[32];snprintf(text,sizeof(text),"%s",quickEntry);
    for(char *p=text;*p;++p) if(*p==',') *p='.';
    char *end=nullptr;float value=strtof(text,&end);
    if(quickField>=100) {
        const int i=quickField-100;
        const float minimum=i==0?.1f:i==1?.8f:1;
        const float maximum=i==0?5:i==1?1.6f:40;
        if(end==text||*end||!std::isfinite(value)||value<minimum||value>maximum||(i>=2&&value!=(int)value)) {
            set_label_text_if_changed(quickError,i==0?"Zakres: 0.10 - 5.00 L/min wody przy 3 bar":i==1?"Zakres gestosci: 0.80 - 1.60 kg/L":"Liczba koncowek: 1 - 40, bez ulamkow");return;
        }
        if(i==0)nozzleDraft.waterQ3[nozzleDraft.nozzle]=value;
        else if(i==1){nozzleDraft.density=value;nozzleDraft.liquid=4;}
        else nozzleDraft.counts[i-2]=(uint8_t)value;
        nozzleDirty=true;close_quick_input();nozzle_editor_refresh();return;
    }
    const float lo=quickField>SECTION_COUNT ? 0 : quickField==SECTION_COUNT ? 10.0f : .1f;
    const float hi=quickField>SECTION_COUNT ? 20000 : quickField==SECTION_COUNT ? 1000.0f : 20.0f;
    if(end==text || *end || !(value>=lo && value<=hi)) {
        set_label_text_if_changed(quickError,quickField>SECTION_COUNT ? "Zakres: 0 - 20000 L" : quickField==SECTION_COUNT ? "Zakres: 10 - 1000 L/ha" : "Zakres: 0.1 - 20.0 m");return;
    }
    if(quickField>SECTION_COUNT) {
        if(!tank_counter_set(value,quickField==SECTION_COUNT+2)) {
            set_label_text_if_changed(quickError,"Blad zapisu lub suma ponad 20000 L. Najpierw ustaw ilosc.");return;
        }
    } else quick_commit_value(value);
    close_quick_input();ui_update();
}
static void quick_cancel_cb(lv_event_t *) { close_quick_input(); }
static void quick_key_cb(lv_event_t *e)
{
    if (!quickText || bench_test_active()) return;
    const int key=(int)(intptr_t)lv_event_get_user_data(e);
    size_t len=strlen(quickEntry);
    if (key==-1) { if (len) quickEntry[--len]=0; }
    else if (key==-2) quickEntry[0]=0;
    else if (len<sizeof(quickEntry)-1 &&
             (key!='.' || !strchr(quickEntry,'.'))) {
        if (key=='.' && len==0) quickEntry[len++]='0';
        if (len<sizeof(quickEntry)-1) { quickEntry[len++]=(char)key;quickEntry[len]=0; }
    }
    set_label_text_if_changed(quickText,quickEntry[0] ? quickEntry : quickInitial);
    set_text_color_if_changed(quickText,quickEntry[0] ? C_TEXT() : C_MUTED(),0);
    set_label_text_if_changed(quickError,quickField==100 ? "Podaj wydatek WODY; korekta RSM stosowana jest automatycznie." : "Wpisz wartosc i zatwierdz przyciskiem OK.");
}
static void quick_open_input_cb(lv_event_t *e)
{
    if(bench_test_active())return;
    close_quick_input();
    quickField=(int)(intptr_t)lv_event_get_user_data(e);
    if(quickField<0 || (quickField>SECTION_COUNT+2 && (quickField<100||quickField>107)))return;
    quickModal=panel(lv_layer_top(),110,62,804,460,C_GREEN());
    home_text(quickModal,quickField==100 ? "KALIBRACJA WODY - 1 KONCOWKA PRZY 3 bar" : quickField==101 ? "GESTOSC CIECZY (kg/L)" : quickField>=102 ? "LICZBA KONCOWEK NA SEKCJI" : quickField==SECTION_COUNT+1 ? "USTAW AKTUALNA ILOSC W ZBIORNIKU (L)" : quickField==SECTION_COUNT+2 ? "DOLEJ DO ZBIORNIKA (L)" : quickField==SECTION_COUNT ? "DAWKA ZADANA (L/ha)" : "SZEROKOSC SEKCJI (m)",24,20,C_TEXT(),&home_font_24);
    lv_obj_t *field=panel(quickModal,24,65,756,60,C_BORDER());
    quickEntry[0]=0;
    if(quickField>=100)snprintf(quickInitial,sizeof(quickInitial),"%.2f",quickField==100?nozzleDraft.waterQ3[nozzleDraft.nozzle]:quickField==101?nozzleDraft.density:(float)nozzleDraft.counts[quickField-102]);
    else if(quickField>SECTION_COUNT) snprintf(quickInitial,sizeof(quickInitial),"%.1f",quickField==SECTION_COUNT+2 ? 0.0f : tank_counter_read().liters);
    else if(quickField==SECTION_COUNT) snprintf(quickInitial,sizeof(quickInitial),"%d",gState.rateLHa);
    else snprintf(quickInitial,sizeof(quickInitial),"%.1f",sectionWidthM[quickField]);
    quickText=home_text(field,quickInitial,16,14,C_MUTED(),&home_font_24);
    quickError=home_text(quickModal,"Wpisz wartosc i zatwierdz przyciskiem OK.",24,137,C_MUTED(),&lv_font_montserrat_16);
    static const char *keys[]={"7","8","9","COFNIJ","4","5","6","WYCZYSC","1","2","3",".","0","ANULUJ","OK"};
    static const int codes[]={'7','8','9',-1,'4','5','6',-2,'1','2','3','.', '0',0,0};
    for(int i=0;i<15;++i) {
        int col=i%4, row=i/4, width=180;
        if(i>=12) {col=i==12 ? 0 : i==13 ? 2 : 3; if(i==12) width=372;}
        lv_event_cb_t cb=i==13 ? quick_cancel_cb : i==14 ? quick_accept_cb : quick_key_cb;
        lv_obj_t *key=button(quickModal,24+col*192,180+row*64,width,56,keys[i],cb,(void *)(intptr_t)codes[i]);
        lv_obj_set_style_text_font(lv_obj_get_child(key,0),&home_font_24,0);
        if(i==14) set_button_state(key,true);
    }

}
static void quick_width_step_cb(lv_event_t *e)
{
    if(bench_test_active())return;
    const int code=(int)(intptr_t)lv_event_get_user_data(e);
    const int i=abs(code)-1;if(i<0 || i>=SECTION_COUNT)return;
    quickField=i;
    const int tenths=(int)(sectionWidthM[i]*10+.5f)+(code>0 ? 1 : -1);
    quick_commit_value((tenths<1 ? 1 : tenths>200 ? 200 : tenths)/10.0f);
}
static void quick_save_cb(lv_event_t *)
{
    if(bench_test_active())return;
    bool ok=sectionPrefs.begin("agrosec",false);
    if(ok) {
        for(int i=0;i<SECTION_COUNT;++i) {
            char key[8];snprintf(key,sizeof(key),"sec%d",i+1);
            if(sectionPrefs.putFloat(key,sectionWidthM[i])!=sizeof(float)) ok=false;
        }
        sectionPrefs.end();
    }
    if(ok)quickWidthsDirty=false;
    quickStatus=ok ? "Szerokosci zapisane." : "Blad zapisu. Sprobuj ponownie.";
    quickStatusUntil=millis()+3000;
    quick_settings_refresh();
}
static lv_obj_t *quick_button(lv_obj_t *parent,int x,int y,int w,int h,const char *text,
                              lv_event_cb_t cb,int data)
{
    lv_obj_t *b=button(parent,x,y,w,h,text,cb,(void *)(intptr_t)data);
    lv_obj_set_style_text_font(lv_obj_get_child(b,0),&home_font_24,0);
    if(quickInputCount<32)quickInputs[quickInputCount++]=b;
    return b;
}
static void info_refresh() {
    if(!infoPanel)return;
    char text[320];
    if(infoIsTank) {
        const auto t=tank_counter_read();
        if(t.configured)snprintf(text,sizeof(text),"POZOSTALO (WYLICZONE)\n\n%.1f L\n\nZuzyto od ustawienia: %.1f L",t.liters,t.usedLiters);
        else snprintf(text,sizeof(text),"ZBIORNIK NIEUSTAWIONY\n\nWpisz aktualna ilosc cieczy.");
        set_label_text_if_changed(infoLeft,text);
        set_label_text_if_changed(infoNote,!t.saveOk?"BLAD ZAPISU - sprawdz ilosc przed wylaczeniem.":bench_test_active()?"SYMULACJA - licznik zbiornika wstrzymany.":"Szacunek z cisnienia i czynnych koncowek. Sprawdz stan po zmianie modelu.");
    } else {
        char pressure[32];if(pressure_sensor_valid())snprintf(pressure,sizeof(pressure),"%.1f bar",pressure_sensor_bar_x10()/10.0f);else snprintf(pressure,sizeof(pressure),"-- (brak odczytu)");
        snprintf(text,sizeof(text),"Profil: %s / %s\nWydatek oblicz.: %.1f L/min\nCisnienie: %s\nSygnal czujnika: %d mV",
            nozzle_name(),nozzle_liquid_name(),gState.actualFlowX10/10.0f,
            pressure,pressure_sensor_signal_mv());
        set_label_text_if_changed(infoLeft,text);
        char age[32];if(gState.rxPackets)snprintf(age,sizeof(age),"%lu ms",(unsigned long)(millis()-gState.lastAogMs));else snprintf(age,sizeof(age),"brak pakietow");
        const int direction=valve_test_direction();
        snprintf(text,sizeof(text),"Wi-Fi: %s | AOG: %s\nOstatnie dane: %s\nUDP RX: %lu / TX: %lu\nZawor - polecenie: %s\nTryb: %s",gState.wifiOk?"OK":"BRAK",gState.aogOk?"OK":"BRAK",age,
            (unsigned long)gState.rxPackets,(unsigned long)gState.txPackets,direction>0?"A":direction<0?"B":"STOP",gState.autoMode?"AUTO":"MAN");
        set_label_text_if_changed(infoRight,text);
        snprintf(text,sizeof(text),"Sekcje (polecenia): 1:%s  2:%s  3:%s  4:%s  5:%s  6:%s\nBrak potwierdzenia polozenia zaworow%s",
            gState.sections[0]?"ON":"OFF",gState.sections[1]?"ON":"OFF",gState.sections[2]?"ON":"OFF",
            gState.sections[3]?"ON":"OFF",gState.sections[4]?"ON":"OFF",gState.sections[5]?"ON":"OFF",bench_test_active()?" | SYMULACJA":"");
        set_label_text_if_changed(infoNote,text);
    }
}
static void info_open(bool tank) {
    release_detail_editors();infoIsTank=tank;
    lv_obj_set_hidden(detailMsg,true);lv_obj_set_hidden(detailMsg2,true);
    set_label_text_if_changed(lblSettingsDetailTitle,tank?"ZBIORNIK":"DIAGNOSTYKA - DANE RZECZYWISTE");
    infoPanel=panel(settingsDetailPanel,12,12,952,396,C_BORDER());
    if(tank) {
        infoLeft=home_text(infoPanel,"--",24,24,C_TEXT(),&home_font_24);
        home_control(infoPanel,540,30,384,64,"USTAW ILOSC",nullptr);
        lv_obj_t *set=lv_obj_get_child(infoPanel,1);
        lv_obj_add_event_cb(set,quick_open_input_cb,LV_EVENT_CLICKED,(void *)(intptr_t)(SECTION_COUNT+1));
        lv_obj_t *add=home_control(infoPanel,540,110,384,64,"DOLEJ",nullptr);
        lv_obj_add_event_cb(add,quick_open_input_cb,LV_EVENT_CLICKED,(void *)(intptr_t)(SECTION_COUNT+2));
        home_text(infoPanel,"USTAW: aktualna zawartosc zbiornika.\nDOLEJ: dodaje wpisane litry.\n\nStan zapisujemy przy wpisaniu ilosci\ni co 60 s podczas zuzycia. Po naglym\nodcieciu zasilania sprawdz pozostale litry.",540,194,C_MUTED(),&lv_font_montserrat_16);
    } else {
        home_text(infoPanel,"CZUJNIKI",24,20,C_GREEN(),&home_font_24);
        home_text(infoPanel,"LACZNOSC I WYJSCIA",480,20,C_GREEN(),&home_font_24);
        infoLeft=home_text(infoPanel,"--",24,70,C_TEXT(),&lv_font_montserrat_20);
        infoRight=home_text(infoPanel,"--",480,70,C_TEXT(),&lv_font_montserrat_20);
        lv_obj_set_style_text_line_space(infoLeft,16,0);lv_obj_set_style_text_line_space(infoRight,16,0);
    }
    infoNote=home_text(infoPanel,"--",24,340,C_MUTED(),&lv_font_montserrat_16);
    lv_obj_set_width(infoNote,900);
    info_refresh();lv_screen_load(scrSettingsDetail);
}
static void diagnostics_open_cb(lv_event_t *){info_open(false);}
static void tank_open_cb(lv_event_t *){info_open(true);}
static void job_refresh() {
    if(!jobPanel)return;
    const auto j=job_counter_read();char text[96];
    snprintf(text,sizeof(text),"%.1f",j.liters);set_label_text_if_changed(jobValues[0],text);
    snprintf(text,sizeof(text),"%.3f",j.hectares);set_label_text_if_changed(jobValues[1],text);
    const uint64_t sec=j.sprayMs/1000;
    snprintf(text,sizeof(text),"%02lu:%02lu:%02lu",(unsigned long)(sec/3600),(unsigned long)((sec/60)%60),(unsigned long)(sec%60));
    set_label_text_if_changed(jobValues[2],text);
    if(j.hectares>=0.001f)snprintf(text,sizeof(text),"%.0f",j.areaLiters/j.hectares);else snprintf(text,sizeof(text),"--");
    set_label_text_if_changed(jobValues[3],text);
    set_label_text_if_changed(jobStatus,jobConfirm?"Nacisnij ZERUJ I START, aby skasowac poprzednie podsumowanie.":
        !j.saveOk?"BLAD ZAPISU - dane pozostaja w pamieci do restartu.":bench_test_active()?"SYMULACJA - rzeczywisty zabieg nie jest zliczany.":
        !j.exists?"Nacisnij NOWY ZABIEG, aby rozpoczac zliczanie.":j.running?"ZLICZANIE AKTYWNE":"PAUZA - nacisnij WZNOW, aby kontynuowac.");
    set_label_text_if_changed(lv_obj_get_child(jobPauseButton,0),j.running?"PAUZA":"WZNOW");
    set_label_text_if_changed(lv_obj_get_child(jobNewButton,0),jobConfirm?"ZERUJ I START":"NOWY ZABIEG");
    if(!j.exists||bench_test_active())lv_obj_add_state(jobPauseButton,LV_STATE_DISABLED);else lv_obj_remove_state(jobPauseButton,LV_STATE_DISABLED);
    if(bench_test_active())lv_obj_add_state(jobNewButton,LV_STATE_DISABLED);else lv_obj_remove_state(jobNewButton,LV_STATE_DISABLED);
}
static void job_new_cb(lv_event_t *) {
    if(bench_test_active())return;
    if(job_counter_read().exists&&!jobConfirm){jobConfirm=true;job_refresh();return;}
    jobConfirm=false;job_counter_new();job_refresh();
}
static void job_pause_cb(lv_event_t *) {
    jobConfirm=false;job_counter_pause(job_counter_read().running);job_refresh();
}
static void job_open_cb(lv_event_t *) {
    release_detail_editors();
    lv_obj_set_hidden(detailMsg,true);lv_obj_set_hidden(detailMsg2,true);
    set_label_text_if_changed(lblSettingsDetailTitle,"PODSUMOWANIE ZABIEGU");
    jobPanel=panel(settingsDetailPanel,12,12,952,396,C_BORDER());
    const char *names[]={"ZUZYTA CIECZ","POWIERZCHNIA","CZAS OPRYSKU","SREDNIA DAWKA"};
    const char *units[]={"L - obliczone","ha - szacowana","godz : min : sek","L/ha - odcinki z danymi"};
    for(int i=0;i<4;i++) {
        auto *tile=panel(jobPanel,20+i*230,20,222,150,C_BORDER());
        home_text(tile,names[i],10,12,C_MUTED(),&lv_font_montserrat_16);
        jobValues[i]=home_text(tile,"--",10,52,C_TEXT(),i==2?&home_font_24:&home_font_32);
        lv_obj_set_width(jobValues[i],202);lv_label_set_long_mode(jobValues[i],LV_LABEL_LONG_CLIP);
        home_text(tile,units[i],10,115,C_MUTED(),&lv_font_montserrat_16);
    }
    jobStatus=home_text(jobPanel,"--",20,190,C_GREEN(),&lv_font_montserrat_16);
    home_text(jobPanel,"Litry: cisnienie, profil cieczy i liczba czynnych koncowek. Wartosci obliczone.\nSrednia: tylko odcinki z poprawna predkoscia i powierzchnia. Bez korekty nakladek.\nZapis co 60 s i przy pauzie. Po restarcie zabieg pozostaje w pauzie.",20,224,C_MUTED(),&lv_font_montserrat_16);
    jobNewButton=home_control(jobPanel,20,315,440,56,"NOWY ZABIEG",job_new_cb);
    jobPauseButton=home_control(jobPanel,480,315,452,56,"WZNOW",job_pause_cb);
    job_refresh();lv_screen_load(scrSettingsDetail);
}
static void nozzle_editor_refresh(){
    if(!nozzlePanel)return;
    char b[128];
    set_label_text_if_changed(lv_obj_get_child(nozzleSelect,0),nozzleDraft.nozzle?"KONCOWKA: 04RSM  >":"KONCOWKA: 11003  >");
    const char *names[]={"WODA","RSM 28","RSM 30","RSM 32","WLASNA"};
    snprintf(b,sizeof(b),"CIECZ: %s  >",names[nozzleDraft.liquid]);set_label_text_if_changed(lv_obj_get_child(liquidSelect,0),b);
    for(int i=0;i<8;i++){
        if(i==0)snprintf(b,sizeof(b),"%.2f L/min",nozzleDraft.waterQ3[nozzleDraft.nozzle]);
        else if(i==1)snprintf(b,sizeof(b),"%.2f kg/L",nozzleDraft.density);
        else snprintf(b,sizeof(b),"%d szt.",nozzleDraft.counts[i-2]);
        set_label_text_if_changed(nozzleValues[i],b);
    }
    int n=0;for(auto count:nozzleDraft.counts)n+=count;
    if(!nozzle_profile_valid(nozzleDraft))
        snprintf(b,sizeof(b),"NIEZGODNY PROFIL: 11003=WODA/WLASNA, 04RSM=RSM 28/30/32");
    else
        snprintf(b,sizeof(b),"%s | %d koncowek | Przy 3 bar: %.2f L/min na koncowke",nozzleDirty?"NIEZAPISANE":"ZAPISANE",n,nozzle_flow(3,nozzleDraft));
    set_label_text_if_changed(nozzleMessage,b);
}
static void nozzle_cycle_cb(lv_event_t *e){
    if(bench_test_active())return;
    if((intptr_t)lv_event_get_user_data(e)==0)nozzleDraft.nozzle^=1;
    else {nozzleDraft.liquid=(nozzleDraft.liquid+1)%5;const float rho[]={1,1.28f,1.30f,1.32f};if(nozzleDraft.liquid<4)nozzleDraft.density=rho[nozzleDraft.liquid];}
    nozzleDirty=true;nozzle_editor_refresh();
}
static void nozzle_save_cb(lv_event_t *){
    if(!nozzle_profile_valid(nozzleDraft)){
        set_label_text_if_changed(nozzleMessage,"NIEZGODNY PROFIL: 11003=WODA/WLASNA, 04RSM=RSM 28/30/32");
        return;
    }
    if(nozzle_save(nozzleDraft)){nozzleDirty=false;nozzle_editor_refresh();ui_update();}
    else set_label_text_if_changed(nozzleMessage,"BLAD ZAPISU - ustawienia nie zostaly zastosowane.");
}
static void nozzle_cancel_cb(lv_event_t *){nozzleDraft=nozzle_settings();nozzleDirty=false;nozzle_editor_refresh();}
static void nozzle_open_cb(lv_event_t *){
    if(bench_test_active())return;
    release_detail_editors();nozzleDraft=nozzle_settings();nozzleDirty=false;
    lv_obj_set_hidden(detailMsg,true);lv_obj_set_hidden(detailMsg2,true);
    set_label_text_if_changed(lblSettingsDetailTitle,"KONCOWKI I CIECZ - EDYCJA");
    nozzlePanel=panel(settingsDetailPanel,12,12,952,396,C_BORDER());
    nozzleSelect=home_control(nozzlePanel,20,16,446,50,"--",nozzle_cycle_cb);
    liquidSelect=home_control(nozzlePanel,482,16,450,50,"--",nullptr);
    lv_obj_add_event_cb(liquidSelect,nozzle_cycle_cb,LV_EVENT_CLICKED,(void*)1);
    const char *names[]={"WYDATEK WODY PRZY 3 bar","GESTOSC CIECZY","SEKCJA 1","SEKCJA 2","SEKCJA 3","SEKCJA 4","SEKCJA 5","SEKCJA 6"};
    for(int i=0;i<8;i++){
        int x=i<2?20+i*462:20+(i-2)*154;
        int y=i<2?78:182,w=i<2?446:142;
        home_text(nozzlePanel,names[i],x,y,C_MUTED(),&lv_font_montserrat_16);
        auto *b=home_control(nozzlePanel,x,y+24,w,48,"--",nullptr);
        lv_obj_add_event_cb(b,quick_open_input_cb,LV_EVENT_CLICKED,(void*)(intptr_t)(100+i));
        nozzleValues[i]=lv_obj_get_child(b,0);
    }
    home_text(nozzlePanel,"Kalibracja wydatku: wpisz pomiar WODA przy 3 bar. Korekta RSM jest automatyczna.",20,158,C_MUTED(),&lv_font_montserrat_16);
    nozzleMessage=home_text(nozzlePanel,"--",20,270,C_GREEN(),&lv_font_montserrat_16);
    home_text(nozzlePanel,"Dawka i litry sa obliczane. Zmiany dzialaja po ZAPISZ; wyjscie odrzuca edycje.",20,297,C_MUTED(),&lv_font_montserrat_16);
    home_control(nozzlePanel,20,330,446,50,"COFNIJ ZMIANY",nozzle_cancel_cb);
    auto *save=home_control(nozzlePanel,482,330,450,50,"ZAPISZ",nozzle_save_cb);set_button_state(save,true);
    nozzle_editor_refresh();lv_screen_load(scrSettingsDetail);
}
static void create_quick_settings()
{
    lv_obj_t *heading=panel(scrSettings,12,8,1000,56,C_BORDER());
    home_text(heading,"USTAWIENIA",18,12,C_TEXT(),&home_font_24);
    home_text(heading,"Zmiana wartosci bez przewijania i wybierania sekcji",350,20,C_MUTED(),&lv_font_montserrat_16);
    lv_obj_t *widths=panel(scrSettings,12,76,600,368,C_BORDER());
    home_text(widths,"SZEROKOSCI SEKCJI (m)",16,12,C_TEXT(),&home_font_18);
    for(int i=0;i<SECTION_COUNT;++i) {
        const int x=12+(i%2)*292, y=44+(i/2)*88;
        lv_obj_t *row=panel(widths,x,y,284,80,C_BORDER());
        char title[24];snprintf(title,sizeof(title),"SEKCJA %d",i+1);
        home_text(row,title,12,7,C_MUTED(),&lv_font_montserrat_16);
        quick_button(row,8,28,48,46,"-",quick_width_step_cb,-(i+1));
        lv_obj_t *value=quick_button(row,64,28,154,46,"--",quick_open_input_cb,i);
        quickWidths[i]=lv_obj_get_child(value,0);
        quick_button(row,226,28,48,46,"+",quick_width_step_cb,i+1);
    }
    quickTotal=home_text(widths,"BELKA: 21.0 m",16,322,C_GREEN(),&home_font_18);
    quick_button(widths,352,312,230,48,"ZAPISZ",quick_save_cb,0);
    quickMessage=home_text(scrSettings,"Dotknij liczby, aby wpisac wartosc.",26,458,C_MUTED(),&lv_font_montserrat_16);
    button(scrSettings,24,482,576,42,"PODSUMOWANIE ZABIEGU",job_open_cb);
    lv_obj_t *flow=button(scrSettings,624,76,388,104,"KONCOWKI I CIECZ\nWybor / edycja / kalibracja",nozzle_open_cb);
    lv_obj_set_style_text_font(lv_obj_get_child(flow,0),&home_font_18,0);
    lv_obj_t *pressure=button(scrSettings,624,196,388,104,"CISNIENIE\nOdczyt czujnika / test zaworu",settings_detail_cb,(void *)"CISNIENIE / REGULATOR");
    lv_obj_set_style_text_font(lv_obj_get_child(pressure,0),&home_font_18,0);
    lv_obj_t *bench=button(scrSettings,624,316,388,104,"TEST NA BIURKU\nSymulacja predkosci i przeplywu",settings_detail_cb,(void *)"TEST NA BIURKU");
    lv_obj_set_style_text_font(lv_obj_get_child(bench,0),&home_font_18,0);
    button(scrSettings,624,432,188,76,"DIAGNOSTYKA",diagnostics_open_cb);
    button(scrSettings,824,432,188,76,"ZBIORNIK",tank_open_cb);
    quick_settings_refresh();dashboard_navigation(scrSettings,2);
}

void ui_create()
{
    lv_obj_t *scr = lv_screen_active();
    scrHome = scr;

    section_settings_load();
    regulator_settings_load();
    nozzle_begin();
    tank_counter_begin();
    job_counter_begin();

    lv_obj_set_style_bg_color(scr, C_BG(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(scr, false);

    // Geometry follows the approved 1024 x 600 dashboard reference.
    lv_obj_t *top = panel(scr, 14, 4, 996, 36, C_BG());
    lv_obj_set_style_border_width(top, 0, 0);
    home_icon(top, 0, 2, 30, 0);
    home_text(top, "AgroSprayer", 40, 2, C_TEXT(), &home_font_24);
    lv_obj_t *modePill = panel(top, 244, 4, 78, 28, C_GREEN());
    lv_obj_set_style_radius(modePill, 16, 0);
    lblMode = home_text(modePill, "AUTO", 0, 1, C_GREEN(), &home_font_18);
    lv_obj_set_width(lblMode, 76); lv_obj_set_style_text_align(lblMode, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t *wifiPill = panel(top, 736, 1, 108, 32, C_BORDER());
    lblWifi = home_text(wifiPill, LV_SYMBOL_WIFI " Wi-Fi", 8, 6, C_GREEN(), &home_font_18);
    auto *profileButton=home_control(top,334,1,320,32,"--",nozzle_open_cb);
    homeProfile=lv_obj_get_child(profileButton,0);lv_obj_set_style_text_font(homeProfile,&lv_font_montserrat_16,0);
    lv_obj_t *aogPill = panel(top, 852, 1, 142, 32, C_BORDER());
    lblAog = home_text(aogPill, LV_SYMBOL_GPS " AgOpenGPS", 8, 8, C_GREEN(), &lv_font_montserrat_16);
    lblUdp = label(top, "UDP", 0, 0, C_MUTED()); lv_obj_set_hidden(lblUdp, true);

    dashboard_metric(scr, 14, "DAWKA", &lblActualRate, "L/ha", LV_SYMBOL_EDIT);
    lv_obj_t *doseCard=lv_obj_get_parent(lblActualRate);
    home_text(doseCard,"ZADANA",122,4,C_MUTED(),&lv_font_montserrat_16);
    lblRate=home_text(doseCard,"200 L/ha",112,24,C_TEXT(),&lv_font_montserrat_16);
    lv_obj_set_size(lblRate,92,20);
    lv_label_set_long_mode(lblRate,LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(lblRate,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_set_clickable(doseCard,true);
    lv_obj_add_event_cb(doseCard,quick_open_input_cb,LV_EVENT_CLICKED,(void *)(intptr_t)SECTION_COUNT);
    for(uint32_t i=0;i<lv_obj_get_child_count(doseCard);++i)
        lv_obj_set_clickable(lv_obj_get_child(doseCard,i),false);
    dashboard_metric(scr, 232, "WYDATEK", &lblFlow, "L/min", LV_SYMBOL_TINT);
    dashboard_metric(scr, 438, "CISNIENIE", &lblPressure, "bar", LV_SYMBOL_SETTINGS);
    home_text(doseCard,"oblicz.",12,102,C_MUTED(),&lv_font_montserrat_16);
    // A small pencil at the edge makes direct editing discoverable.
    home_text(doseCard,LV_SYMBOL_EDIT,188,99,C_GREEN(),&lv_font_montserrat_16);

    lv_obj_t *compare = panel(scr, 14, 176, 618, 96, C_BORDER());
    home_icon(compare, 16, 5, 20, 1);
    home_text(compare, "WYDATEK / CEL", 44, 6, C_TEXT(), &home_font_18);
    home_text(compare, "Oblicz.", 12, 43, C_MUTED(), &lv_font_montserrat_16);
    home_text(compare, "Cel", 12, 74, C_MUTED(), &lv_font_montserrat_16);
    flowActualBar = dashboard_bar(compare, 85, 46, C_GREEN());
    flowTargetBar = dashboard_bar(compare, 85, 77, C_GREEN());
    lv_obj_set_size(flowActualBar, 285, 12); lv_obj_set_size(flowTargetBar, 285, 12);
    home_text(compare,"L/min",390,6,C_MUTED(),&lv_font_montserrat_16);
    lblFlowCompare = home_text(compare, "--", 390, 43, C_TEXT(), &lv_font_montserrat_16);
    lblTarget = home_text(compare, "--", 390, 74, C_TEXT(), &lv_font_montserrat_16);
    flowStatusIcon = home_icon(compare, 552, 31, 28, 5);
    lblReg = home_text(compare, "POSTOJ", 503, 72, C_GREEN(), &lv_font_montserrat_16);
    lv_obj_set_width(lblReg, 112);
    lv_obj_set_style_text_align(lblReg, LV_TEXT_ALIGN_CENTER, 0); lv_label_set_long_mode(lblReg, LV_LABEL_LONG_WRAP);
    lblErr = home_text(compare, "--", 496, 8, C_MUTED(), &lv_font_montserrat_16);

    lv_obj_t *speed = panel(scr, 14, 280, 298, 64, C_BORDER());
    home_icon(speed, 14, 28, 26, 2);
    home_text(speed, "PREDKOSC", 12, 6, C_MUTED(), &lv_font_montserrat_16);
    lblSpeed = home_text(speed, "0.0", 56, 23, C_TEXT(), &home_font_32);
    lv_obj_set_size(lblSpeed,170,38);lv_label_set_long_mode(lblSpeed,LV_LABEL_LONG_CLIP);lv_obj_set_style_text_align(lblSpeed,LV_TEXT_ALIGN_RIGHT,0);
    home_text(speed, "km/h", 236, 35, C_MUTED(), &lv_font_montserrat_16);
    lv_obj_t *width = panel(scr, 320, 280, 312, 64, C_BORDER());
    home_text(width, "SZEROKOSC AKTYWNA", 12, 6, C_MUTED(), &lv_font_montserrat_16);
    lblActiveWidth = home_text(width, "0.0", 12, 23, C_TEXT(), &home_font_32);
    lv_obj_set_size(lblActiveWidth,252,38);lv_label_set_long_mode(lblActiveWidth,LV_LABEL_LONG_CLIP);lv_obj_set_style_text_align(lblActiveWidth,LV_TEXT_ALIGN_RIGHT,0);
    home_text(width, "m", 276, 35, C_MUTED(), &lv_font_montserrat_16);

    lv_obj_t *map = panel(scr, 640, 46, 370, 235, C_BORDER());
    home_icon(map, 12, 8, 24, 3);
    home_text(map, "NAWIGACJA", 44, 10, C_TEXT(), &home_font_18);
    lblWorkStatus=home_text(map,"POSTOJ",190,10,C_MUTED(),&lv_font_montserrat_16);
    lv_obj_set_width(lblWorkStatus,164);
    lv_obj_set_style_text_align(lblWorkStatus,LV_TEXT_ALIGN_RIGHT,0);
    lv_obj_t *mapHint=home_text(map,"Brak danych mapy\nz AgOpenGPS",16,118,C_MUTED(),&home_font_18);
    lv_obj_set_width(mapHint,338);lv_obj_set_style_text_align(mapHint,LV_TEXT_ALIGN_CENTER,0);
    nozzleHomeInfo=home_text(map,"--",16,177,C_MUTED(),&lv_font_montserrat_16);
    lv_obj_set_width(nozzleHomeInfo,338);lv_obj_set_style_text_align(nozzleHomeInfo,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_t *tank = panel(scr, 640, 289, 370, 55, C_BORDER());
    lv_obj_add_event_cb(tank,tank_open_cb,LV_EVENT_CLICKED,nullptr);
    home_icon(tank, 16, 10, 28, 4);
    home_text(tank, "ZBIORNIK", 58, 18, C_MUTED(), &home_font_18);
    lblTank = home_text(tank, "--- L", 206, 7, C_TEXT(), &home_font_18);
    lblTankPct = home_text(tank, "Wyliczone", 206, 33, C_MUTED(), &lv_font_montserrat_16);

    lv_obj_t *sections = panel(scr, 14, 352, 996, 128, C_BORDER());
    home_text(sections, LV_SYMBOL_LIST "  SEKCJE ROBOCZE", 20, 10, C_TEXT(), &home_font_18);
    lblBoomWidth = home_text(sections, "BELKA 21.0 m", 792, 14, C_MUTED(), &lv_font_montserrat_16);
    sprayerScene = panel(sections, 16, 8, 822, 54, C_PANEL());
    lv_obj_set_style_bg_opa(sprayerScene, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(sprayerScene, 0, 0);
    lv_obj_set_clickable(sprayerScene, false);
    lv_obj_add_event_cb(sprayerScene, sprayer_scene_draw, LV_EVENT_DRAW_MAIN, nullptr);
    refresh_sprayer_scene();
    for (int i = 0; i < SECTION_COUNT; ++i) {
        btnSections[i] = button(sections, 16+i*139, 64, 135, 40, "--", section_cb, (void *)(intptr_t)i);
        lv_obj_set_style_radius(btnSections[i], 4, 0);
        lv_obj_set_style_pad_all(btnSections[i], 0, 0);
        lv_obj_set_style_text_font(lv_obj_get_child(btnSections[i], 0), &home_font_24, 0);
        lv_obj_t *number = lv_obj_get_child(btnSections[i], 0);
        lv_obj_align(number, LV_ALIGN_CENTER, 6, 0);
        lv_obj_t *dot = panel(btnSections[i], 36, 13, 12, 12, C_GREEN());
        lv_obj_set_style_border_width(dot, 0, 0); lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x607585), 0);
        lv_obj_set_style_bg_grad_dir(dot, LV_GRAD_DIR_NONE, 0);
        lv_obj_set_clickable(dot, false);
        sectionWidthLabels[i] = home_text(sections, "-- m", 16+i*139, 106, C_MUTED(), &lv_font_montserrat_16);
        lv_obj_set_width(sectionWidthLabels[i], 135);
        lv_obj_set_style_text_align(sectionWidthLabels[i], LV_TEXT_ALIGN_CENTER, 0);
        sprayerSections[i] = nullptr;
    }
    lv_obj_t *allButton = home_control(sections, 856, 64, 128, 40, "WSZYSTKIE", all_cb);
    lv_obj_set_style_text_font(lv_obj_get_child(allButton,0), &home_font_18, 0);
    section_editor_refresh();

    // Use the same filled panels and spacing as the dashboard cards above.
    lv_obj_t *modeFrame=panel(scr,14,484,268,62,C_BORDER());
    lv_obj_t *doseFrame=panel(scr,290,484,280,62,C_BORDER());
    lv_obj_t *valveFrame=panel(scr,578,484,432,62,C_BORDER());
    for(lv_obj_t *frame : {modeFrame,doseFrame,valveFrame})
        lv_obj_set_clickable(frame,false);
    home_text(modeFrame,"TRYB PRACY",8,4,C_MUTED(),&lv_font_montserrat_16);
    home_text(doseFrame,"KOREKTA DAWKI",8,4,C_MUTED(),&lv_font_montserrat_16);
    home_text(valveFrame,"ZAWOR REGULACYJNY | impuls 0,25 s",8,4,C_MUTED(),&lv_font_montserrat_16);
    btnAuto = home_control(modeFrame,6,22,124,34,"AUTO",auto_cb);
    btnMan = home_control(modeFrame,138,22,124,34,"MAN",man_cb);
    lv_obj_t *rateDown=home_control(doseFrame,6,22,130,34,"-10 L/ha",rate_minus_cb);
    lv_obj_t *rateUp=home_control(doseFrame,144,22,130,34,"+10 L/ha",rate_plus_cb);
    btnValveOpen=home_control(valveFrame,6,22,134,34,"OTWORZ",nullptr);
    lv_obj_add_event_cb(btnValveOpen,manual_valve_cb,LV_EVENT_CLICKED,(void *)(intptr_t)1);
    lv_obj_t *valveStop=home_control(valveFrame,148,22,134,34,"STOP RUCHU",manual_valve_stop_cb);
    lv_obj_set_style_border_color(valveStop,C_RED(),0);
    btnValveClose=home_control(valveFrame,290,22,134,34,"ZAMKNIJ",nullptr);
    lv_obj_add_event_cb(btnValveClose,manual_valve_cb,LV_EVENT_CLICKED,(void *)(intptr_t)-1);
    for(lv_obj_t *b : {rateDown,rateUp,btnValveOpen,valveStop,btnValveClose})
        lv_obj_set_style_text_font(lv_obj_get_child(b,0),&home_font_18,0);
    lv_obj_set_style_text_font(lv_obj_get_child(valveStop,0),&lv_font_montserrat_16,0);
    lblTotal = label(scr, "0.0 L", 0, 0, C_MUTED()); lv_obj_set_hidden(lblTotal, true);
    dashboard_navigation(scr, 0);


    // ==========================================================
    // EKRAN USTAWIENIA - 4 KAFELKI 2x2
    // ==========================================================
    scrSettings = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scrSettings, C_BG(), 0);
    lv_obj_set_style_bg_opa(scrSettings, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(scrSettings, false);

    create_quick_settings();

    // ==========================================================
    // STRONA SZCZEGOLOW USTAWIEN
    // ==========================================================
    scrSettingsDetail = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scrSettingsDetail, C_BG(), 0);
    lv_obj_set_style_bg_opa(scrSettingsDetail, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(scrSettingsDetail, false);

    lv_obj_t *detailTop = panel(
        scrSettingsDetail,
        4, 4,
        1016, 62,
        lv_color_hex(0x155C79)
    );

    lv_obj_t *backBtn = button(
        detailTop,
        12, 9,
        92, 44,
        "< WROC",
        settings_detail_back_cb
    );

    (void)backBtn;

    lblSettingsDetailTitle =
        label(
            detailTop,
            "USTAWIENIA",
            128, 16,
            C_TEXT()
        );
    font24(lblSettingsDetailTitle);

    lv_obj_t *detailPanel = panel(
        scrSettingsDetail,
        24, 92,
        976, 420,
        lv_color_hex(0x176786)
    );

    settingsDetailPanel = detailPanel;

    detailMsg =
        label(
            detailPanel,
            "Panel gotowy do podlaczenia funkcji kalibracji.",
            28, 34,
            C_MUTED()
        );
    font20(detailMsg);

    detailMsg2 =
        label(
            detailPanel,
            "Kolejny krok: dodamy realne pola, +/- oraz zapis do pamieci.",
            28, 76,
            C_TEXT()
        );
    font16(detailMsg2);


    dashboard_navigation(scrSettingsDetail, 2);

    scrNavigation = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scrNavigation, C_BG(), 0);
    lv_obj_set_scrollable(scrNavigation, false);
    lv_obj_t *navPanel = panel(scrNavigation, 12, 12, 1000, 526, C_BORDER());
    font28(label(navPanel, LV_SYMBOL_GPS "  NAWIGACJA", 24, 20, C_GREEN()));
    font24(label(navPanel, "Podglad mapy wymaga integracji z AgOpenGPS.", 24, 212, C_TEXT()));
    font16(label(navPanel, "Dane predkosci i stanow sekcji sa obslugiwane na ekranie glownym.", 24, 258, C_MUTED()));
    dashboard_navigation(scrNavigation, 1);

    simulationBar = panel(lv_layer_top(), 556, 0, 464, 56, C_ORANGE());
    lv_obj_set_style_bg_color(simulationBar, lv_color_hex(0x4A2600), 0);
    lv_obj_t *simulationLabel = label(simulationBar, "SYMULACJA", 14, 17, C_ORANGE());
    font20(simulationLabel);
    button(simulationBar, 216, 5, 236, 44, "ZAKONCZ TEST", bench_end_cb);
    lv_obj_set_hidden(simulationBar, true);

}







static void release_detail_editors()
{
    // Navigating away from a panel must never re-energize the motor.
    auto_regulator_stop();
    close_quick_input();
    if(nozzlePanel)lv_obj_delete(nozzlePanel);
    nozzlePanel=nozzleSelect=liquidSelect=nozzleMessage=nullptr;for(auto &v:nozzleValues)v=nullptr;
    if(jobPanel)lv_obj_delete(jobPanel);
    jobPanel=jobStatus=jobPauseButton=jobNewButton=nullptr;
    for(auto &value:jobValues)value=nullptr;jobConfirm=false;
    if(infoPanel)lv_obj_delete(infoPanel);
    infoPanel=infoLeft=infoRight=infoNote=nullptr;
    valve_test_stop();
    if(regulatorPanel)lv_obj_delete(regulatorPanel);
    regulatorPanel=regulatorMessage=nullptr;
    for(auto &value:regulatorValues)value=nullptr;
    if (valvePanel) {
        valve_test_stop();
        lv_obj_delete(valvePanel);
    }
    valvePanel = valveStatus = valvePressure = valveDirectionButton = nullptr;
    valveAutoButton = valveAutoStatus = nullptr;
    if (benchPanel) lv_obj_delete(benchPanel);
    benchPanel = benchStatus = benchSpeed = benchFlow = benchTarget = benchReg = benchAuto = nullptr;
    for (int i = 0; i < SECTION_COUNT; ++i) benchSections[i] = nullptr;
    if (sectionEditorPanel) lv_obj_delete(sectionEditorPanel);
    sectionEditorPanel = nullptr;
    sectionEditorTitle = sectionEditorValue = sectionEditorTotal = nullptr;
    if (flowCalPanel) lv_obj_delete(flowCalPanel);
    flowCalPanel = nullptr;
    flowCalStatus = flowCalPulses = flowCalVolume = flowCalCurrent = nullptr;
    flowCalResult = flowCalTime = flowCalAverage = nullptr;
    flowCalLiters = flowCalInstant = flowCalInputPanel = flowCalInputLabel = nullptr;
}

static void create_section_editor()
{
    // Lekki edytor jednej sekcji naraz.
    // Created only while this editor is open.
    sectionEditorPanel = panel(
        settingsDetailPanel,
        110, 120,
        756, 250,
        C_BLUE()
    );
    lv_obj_set_hidden(sectionEditorPanel, true);

    sectionEditorTitle =
        label(
            sectionEditorPanel,
            "SEKCJA 1",
            300, 20,
            C_TEXT()
        );
    font24(sectionEditorTitle);

    button(
        sectionEditorPanel,
        54, 82,
        96, 64,
        "<",
        section_prev_cb
    );

    button(
        sectionEditorPanel,
        184, 82,
        112, 64,
        "- 0.1",
        section_minus_len_cb
    );

    sectionEditorValue =
        label(
            sectionEditorPanel,
            "2.5 m",
            330, 100,
            C_GREEN()
        );
    font28(sectionEditorValue);

    button(
        sectionEditorPanel,
        460, 82,
        112, 64,
        "+ 0.1",
        section_plus_len_cb
    );

    button(
        sectionEditorPanel,
        606, 82,
        96, 64,
        ">",
        section_next_cb
    );

    sectionEditorTotal =
        label(
            sectionEditorPanel,
            "SZEROKOSC BELKI: 21.0 m",
            224, 166,
            C_BLUE()
        );
    font20(sectionEditorTotal);

    button(
        sectionEditorPanel,
        144, 202,
        210, 40,
        "DOMYSLNE",
        section_defaults_cb
    );

    lv_obj_t *sectionSaveBtn = button(
        sectionEditorPanel,
        402, 202,
        210, 40,
        "ZAPISZ",
        section_save_cb
    );
    set_button_state(sectionSaveBtn, true);

    section_editor_refresh();


}

static void create_flow_calibration()
{
    // ==========================================================
    // KALIBRACJA PRZEPLYWOMIERZA - TEST 100 L / CZAS
    // ==========================================================
    flowCalPanel = panel(
        settingsDetailPanel,
        70, 54,
        836, 340,
        C_GREEN()
    );
    lv_obj_set_hidden(flowCalPanel, true);

    flowCalCurrent = label(flowCalPanel, "AKTUALNIE: -- imp/L", 24, 14, C_TEXT());
    font20(flowCalCurrent);
    flowCalStatus = label(flowCalPanel, "GOTOWY", 555, 16, C_ORANGE());
    font16(flowCalStatus);
    flowCalVolume = label(flowCalPanel, "OBJETOSC TESTU: 100.0 L", 24, 50, C_TEXT());
    font20(flowCalVolume);
    flowCalPulses = label(flowCalPanel, "IMPULSY: 0", 24, 86, C_BLUE());
    font20(flowCalPulses);
    flowCalTime = label(flowCalPanel, "CZAS: 0:00", 445, 86, C_TEXT());
    font20(flowCalTime);
    flowCalLiters = label(flowCalPanel, "LITRY WG KALIBRACJI: 0.0 L", 24, 120, C_BLUE());
    font20(flowCalLiters);
    flowCalInstant = label(flowCalPanel, "PRZEPLYW: --.- L/min", 445, 120, C_BLUE());
    font20(flowCalInstant);
    flowCalAverage = label(flowCalPanel, "SREDNI PRZEPLYW: --.- L/min", 24, 154, C_TEXT());
    font20(flowCalAverage);
    flowCalResult = label(flowCalPanel, "WYNIK: --.-- imp/L", 445, 154, C_GREEN());
    font20(flowCalResult);
    button(flowCalPanel, 24, 193, 130, 48, "START", flow_cal_start_cb);
    button(flowCalPanel, 166, 193, 130, 48, "STOP", flow_cal_stop_cb);
    button(flowCalPanel, 308, 193, 184, 48, "WPISZ LITRY", flow_cal_input_open_cb);
    lv_obj_t *flowSaveBtn = button(flowCalPanel, 504, 193, 302, 48,
                                   "OBLICZ I ZAPISZ", flow_cal_save_cb);
    set_button_state(flowSaveBtn, true);
    button(flowCalPanel, 24, 253, 130, 44, "100 L", flow_cal_preset_cb, (void *)(intptr_t)1000);
    button(flowCalPanel, 166, 253, 130, 44, "200 L", flow_cal_preset_cb, (void *)(intptr_t)2000);
    button(flowCalPanel, 308, 253, 130, 44, "300 L", flow_cal_preset_cb, (void *)(intptr_t)3000);
    lv_obj_t *flowHint = label(flowCalPanel,
        "Napelnij przewody. START -> oprysk -> STOP. Wpisz faktyczne litry.", 24, 310, C_MUTED());
    font16(flowHint);

    // Persistent keypad: created once, no screen rebuilding during a test.
    flowCalInputPanel = panel(flowCalPanel, 0, 0, 836, 340, C_BLUE());
    lv_obj_set_clickable(flowCalInputPanel, true);
    flowCalInputLabel = label(flowCalInputPanel, "WPISZ RZECZYWISTE LITRY", 24, 35, C_TEXT());
    font20(flowCalInputLabel);
    label(flowCalInputPanel, "Litry: dokladnosc 0.1 L", 24, 90, C_MUTED());
    button(flowCalInputPanel, 24, 150, 260, 60, "ZATWIERDZ", flow_cal_input_accept_cb);
    button(flowCalInputPanel, 24, 230, 260, 60, "ANULUJ", flow_cal_input_cancel_cb);
    static const char *keys[] = {"1", "2", "3", "\n", "4", "5", "6", "\n",
                                 "7", "8", "9", "\n", ".", "0", "USUN", ""};
    lv_obj_t *matrix = lv_buttonmatrix_create(flowCalInputPanel);
    lv_buttonmatrix_set_map(matrix, keys);
    lv_buttonmatrix_set_button_ctrl_all(matrix,
        (lv_buttonmatrix_ctrl_t)(LV_BUTTONMATRIX_CTRL_CLICK_TRIG | LV_BUTTONMATRIX_CTRL_NO_REPEAT));
    lv_obj_set_pos(matrix, 470, 18);
    lv_obj_set_size(matrix, 324, 300);
    lv_obj_set_style_text_font(matrix, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_add_event_cb(matrix, flow_cal_input_key_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_set_hidden(flowCalInputPanel, true);

    flow_cal_refresh();

}

static void bench_refresh()
{
    if (!benchPanel) return;
    const bool running = bench_test_active();
    char buf[96];
    set_label_text_if_changed(benchStatus, running ? "SYMULACJA REGULATORA - silnik wylaczony" : benchMessage);
    const SprayerState &state = sprayer_display_state();
    if (running) {
        snprintf(buf, sizeof(buf), "PREDKOSC: %d.%d km/h", state.speedX10 / 10, state.speedX10 % 10);
        set_label_text_if_changed(benchSpeed, buf);
        snprintf(buf, sizeof(buf), "PRZEPLYW: %d.%d L/min", state.actualFlowX10 / 10, state.actualFlowX10 % 10);
        set_label_text_if_changed(benchFlow, buf);
        const int target = targetFlowX10();
        snprintf(buf, sizeof(buf), "CEL: %d.%d L/min / %d L/ha", target / 10, target % 10, state.rateLHa);
        set_label_text_if_changed(benchTarget, buf);
        snprintf(buf, sizeof(buf), "ZAWOR: %d%% | %s", bench_test_open_percent(), regulatorText());
        set_label_text_if_changed(benchReg, buf);
    } else {
        set_label_text_if_changed(benchSpeed, "PREDKOSC: -- km/h");
        set_label_text_if_changed(benchFlow, "PRZEPLYW: -- L/min");
        set_label_text_if_changed(benchTarget, "CEL: -- L/min");
        set_label_text_if_changed(benchReg, "REGULATOR: --");
    }
    set_button_state(benchAuto,running && state.autoMode);
    if(benchAuto) {
        set_label_text_if_changed(lv_obj_get_child(benchAuto,0),state.autoMode ? "SIM AUTO" : "SIM MAN");
        if(running)lv_obj_remove_state(benchAuto,LV_STATE_DISABLED);
        else lv_obj_add_state(benchAuto,LV_STATE_DISABLED);
    }
    for (int i = 0; i < SECTION_COUNT; ++i) {
        set_button_state(benchSections[i], running && state.sections[i]);
        if (running) lv_obj_remove_state(benchSections[i], LV_STATE_DISABLED);
        else lv_obj_add_state(benchSections[i], LV_STATE_DISABLED);
    }
}

static void bench_start_cb(lv_event_t *)
{
    if (flowCalRunning) {
        benchMessage = "Najpierw zatrzymaj kalibracje przeplywomierza";
        bench_refresh();
        return;
    }
    bench_test_begin();
    prevSectionStateValid = false;
    ui_update();
}

static void bench_end_cb(lv_event_t *)
{
    bench_test_end();
    benchMessage = "TEST ZAKONCZONY - biezace dane sa przywrocone";
    prevSectionStateValid = false;
    ui_update();
}

static void bench_speed_cb(lv_event_t *e)
{
    if (!bench_test_active()) return;
    bench_test_set_speed_x10(sprayer_display_state().speedX10 + (int)(intptr_t)lv_event_get_user_data(e));
    ui_update();
}

static void bench_speed_preset_cb(lv_event_t *e)
{
    bench_test_set_speed_x10((int)(intptr_t)lv_event_get_user_data(e));
    ui_update();
}

static void bench_flow_cb(lv_event_t *e)
{
    if (!bench_test_active()) return;
    bench_test_set_flow_x10(sprayer_display_state().actualFlowX10 + (int)(intptr_t)lv_event_get_user_data(e));
    ui_update();
}

static void bench_flow_preset_cb(lv_event_t *e)
{
    if (!bench_test_active()) return;
    const int percent = (int)(intptr_t)lv_event_get_user_data(e);
    bench_test_set_flow_x10((targetFlowX10() * percent + 50) / 100);
    ui_update();
}

static void bench_section_cb(lv_event_t *e)
{
    if (!bench_test_active()) return;
    const int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (index < 0 || index >= SECTION_COUNT) return;
    SprayerState &state = sprayer_display_state();
    state.sections[index] = !state.sections[index];
    ui_update();
}

static void bench_auto_cb(lv_event_t *) {
    bench_test_set_auto(!sprayer_display_state().autoMode);
    ui_update();
}
static void create_bench_test()
{
    benchPanel = panel(settingsDetailPanel, 12, 12, 952, 396, C_ORANGE());
    benchStatus = label(benchPanel, benchMessage, 24, 14, C_ORANGE());
    font20(benchStatus);
    benchSpeed = label(benchPanel, "PREDKOSC: -- km/h", 24, 54, C_TEXT());
    benchFlow = label(benchPanel, "PRZEPLYW: -- L/min", 490, 54, C_BLUE());
    font24(benchSpeed);
    font24(benchFlow);
    button(benchPanel, 24, 95, 120, 44, "- 0.5 km/h", bench_speed_cb, (void *)(intptr_t)-5);
    button(benchPanel, 156, 95, 120, 44, "+ 0.5 km/h", bench_speed_cb, (void *)(intptr_t)5);
    button(benchPanel, 24, 147, 100, 44, "5 km/h", bench_speed_preset_cb, (void *)(intptr_t)50);
    button(benchPanel, 136, 147, 100, 44, "8 km/h", bench_speed_preset_cb, (void *)(intptr_t)80);
    button(benchPanel, 248, 147, 100, 44, "10 km/h", bench_speed_preset_cb, (void *)(intptr_t)100);
    benchAuto=button(benchPanel,360,95,112,96,"AUTO / MAN",bench_auto_cb);
    font16(lv_obj_get_child(benchAuto,0));
    button(benchPanel, 490, 95, 120, 44, "- 1 L/min", bench_flow_cb, (void *)(intptr_t)-10);
    button(benchPanel, 622, 95, 120, 44, "+ 1 L/min", bench_flow_cb, (void *)(intptr_t)10);
    button(benchPanel, 490, 147, 104, 44, "CEL -20%", bench_flow_preset_cb, (void *)(intptr_t)80);
    button(benchPanel, 606, 147, 104, 44, "= CEL", bench_flow_preset_cb, (void *)(intptr_t)100);
    button(benchPanel, 722, 147, 104, 44, "CEL +20%", bench_flow_preset_cb, (void *)(intptr_t)120);
    benchTarget = label(benchPanel, "CEL: -- L/min", 24, 205, C_GREEN());
    benchReg = label(benchPanel, "REGULATOR: --", 490, 205, C_ORANGE());
    font20(benchTarget);
    font20(benchReg);
    for (int i = 0; i < SECTION_COUNT; ++i) {
        char name[20];
        snprintf(name, sizeof(name), "SEKCJA %d", i + 1);
        benchSections[i] = button(benchPanel, 24 + i * 148, 244, 136, 48,
            name, bench_section_cb, (void *)(intptr_t)i);
    }
    button(benchPanel, 24, 310, 270, 52, "START AUTO", bench_start_cb);
    button(benchPanel, 310, 310, 270, 52, "ZAKONCZ TEST", bench_end_cb);
    button(benchPanel, 596, 310, 306, 52, "POKAZ EKRAN GLOWNY", load_home_cb);
    bench_refresh();
}

static int configured_target_flow_x10()
{
    float activeWidthM = 0.0f;

    for (int i = 0; i < SECTION_COUNT; i++) {
        if (sprayer_display_state().sections[i]) {
            activeWidthM += sectionWidthM[i];
        }
    }

    if (activeWidthM <= 0.0f ||
        sprayer_display_state().speedX10 <= 0 ||
        sprayer_display_state().rateLHa <= 0) {
        return 0;
    }

    // Q [L/min] = dawka [L/ha] * predkosc [km/h] * szerokosc [m] / 600
    // Poniewaz speedX10 = km/h * 10, wynik od razu liczymy jako L/min * 10.
    float targetX10 =
        ((float)sprayer_display_state().rateLHa *
         (float)sprayer_display_state().speedX10 *
         activeWidthM) / 600.0f;

    if (targetX10 < 0.0f) {
        targetX10 = 0.0f;
    }

    if (targetX10 > 9999.0f) {
        targetX10 = 9999.0f;
    }

    return (int)(targetX10 + 0.5f);
}

static int configured_flow_error_percent(int targetX10)
{
    if (targetX10 <= 0) {
        return 0;
    }

    long diff =
        (long)sprayer_display_state().actualFlowX10 -
        (long)targetX10;

    long err =
        (diff * 100L) /
        (long)targetX10;

    if (err > 999) {
        err = 999;
    }

    if (err < -999) {
        err = -999;
    }

    return (int)err;
}

float ui_get_section_width_m(int index)
{
    if (index < 0 || index >= SECTION_COUNT) {
        return 0.0f;
    }

    return sectionWidthM[index];
}

float ui_get_total_boom_width_m()
{
    return section_total_width_m();
}

float ui_get_active_boom_width_m()
{
    float total = 0.0f;

    for (int i = 0; i < SECTION_COUNT; i++) {
        if (sprayer_display_state().sections[i]) {
            total += sectionWidthM[i];
        }
    }

    return total;
}

int ui_get_target_flow_x10()
{
    return configured_target_flow_x10();
}

static void valve_refresh()
{
    if (!valvePanel) return;
    const int dir = valve_test_direction();
    const char *status = !valve_test_ready() ? "BLAD INICJALIZACJI - TEST ZABLOKOWANY" :
        dir == 1 ? "KIERUNEK A - IMPULS 1 s" :
        dir == -1 ? "KIERUNEK B - IMPULS 1 s" :
        valve_test_armed() ? "GOTOWY - WYJSCIE ZATRZYMANE" : "TEST ZABLOKOWANY";
    set_label_text_if_changed(valveStatus, status);
    if(valveAutoButton) set_label_text_if_changed(lv_obj_get_child(valveAutoButton,0),
        auto_regulator_armed() ? "AUTO REG: WYLACZ" : "AUTO REG: WLACZ");
    if(valveAutoStatus) set_label_text_if_changed(valveAutoStatus,auto_regulator_status());
    if(valveDirectionButton) set_label_text_if_changed(lv_obj_get_child(valveDirectionButton,0),
        valve_open_direction()==1 ? "OTWORZ = A | ZAMIEN KIERUNKI" : "OTWORZ = B | ZAMIEN KIERUNKI");
    char text[96];
    if (pressure_sensor_valid()) {
        const int p = pressure_sensor_bar_x10();
        snprintf(text, sizeof(text), "CISNIENIE: %d.%d bar | SYGNAL: %d mV",
                 p / 10, p % 10, pressure_sensor_signal_mv());
    } else snprintf(text, sizeof(text), "CISNIENIE: BRAK POPRAWNEGO ODCZYTU");
    set_label_text_if_changed(valvePressure, text);
}
static void valve_direction_cb(lv_event_t *) {
    if(bench_test_active())return;
    auto_regulator_stop();
    if(!valve_toggle_open_direction()) {set_label_text_if_changed(valveStatus,"BLAD ZAPISU KIERUNKU");return;}
    valve_refresh();
}
static void valve_arm_cb(lv_event_t *) {
    if (!bench_test_active() && !auto_regulator_armed()) valve_test_arm();
    valve_refresh();
}
static void valve_stop_cb(lv_event_t *) { auto_regulator_stop(); valve_test_stop(); valve_refresh(); }
static void valve_pulse_cb(lv_event_t *e) {
    if (!bench_test_active() && !auto_regulator_armed()) valve_test_pulse((int)(intptr_t)lv_event_get_user_data(e));
    valve_refresh();
}
static void regulator_editor_refresh() {
    if(!regulatorPanel)return;
    char text[48];
    snprintf(text,sizeof(text),"%lu ms",(unsigned long)regulatorDraft.pulseMs);
    set_label_text_if_changed(regulatorValues[0],text);
    snprintf(text,sizeof(text),"%lu ms",(unsigned long)regulatorDraft.pauseMs);
    set_label_text_if_changed(regulatorValues[1],text);
    snprintf(text,sizeof(text),"+/- %lu%%",(unsigned long)regulatorDraft.tolerancePercent);
    set_label_text_if_changed(regulatorValues[2],text);
}
static void regulator_adjust_cb(lv_event_t *e) {
    const int code=(int)(intptr_t)lv_event_get_user_data(e);
    const int index=abs(code)-1, sign=code>0?1:-1;
    if(index<0 || index>2)return;
    uint32_t *value=index==0?&regulatorDraft.pulseMs:index==1?&regulatorDraft.pauseMs:&regulatorDraft.tolerancePercent;
    const int step=index==0?50:index==1?100:1;
    const int low=index==0?50:index==1?100:1, high=index==0?1000:index==1?3000:20;
    const int next=(int)*value+sign*step;
    *value=next<low?low:next>high?high:next;
    set_label_text_if_changed(regulatorMessage,"Niezapisane zmiany - nacisnij ZAPISZ.");
    regulator_editor_refresh();
}
static void regulator_defaults_cb(lv_event_t *) {
    regulatorDraft=RegulatorSettings{};
    set_label_text_if_changed(regulatorMessage,"Przywrocono domyslne - nacisnij ZAPISZ.");
    regulator_editor_refresh();
}
static void regulator_save_cb(lv_event_t *) {
    if(!regulator_settings_save(regulatorDraft)) {
        set_label_text_if_changed(regulatorMessage,"BLAD ZAPISU - poprzednie ustawienia pozostaja aktywne.");return;
    }
    set_label_text_if_changed(regulatorMessage,"ZAPISANO - ustawienia dzialaja w symulacji.");
}
static void regulator_back_cb(lv_event_t *) {release_detail_editors();create_valve_test();}
static void regulator_open_cb(lv_event_t *) {
    if(bench_test_active())return;
    auto_regulator_stop();
    release_detail_editors();create_regulator_settings();
}
static void create_regulator_settings() {
    regulatorDraft=regulator_settings();
    regulatorPanel=panel(settingsDetailPanel,12,12,952,396,C_BORDER());
    home_text(regulatorPanel,"REGULATOR DAWKI",20,12,C_TEXT(),&home_font_24);
    home_text(regulatorPanel,"Parametry symulacji | automatyczne sterowanie silnikiem jeszcze wylaczone",20,42,C_MUTED(),&lv_font_montserrat_16);
    const char *titles[]={"IMPULS ZAWORU","PRZERWA PO IMPULSIE","TOLERANCJA DAWKI"};
    const char *notes[]={"50-1000 ms | krok 50 ms","100-3000 ms | krok 100 ms","1-20% | krok 1%"};
    for(int i=0;i<3;++i) {
        lv_obj_t *tile=panel(regulatorPanel,20+i*306,76,288,184,C_BORDER());
        home_text(tile,titles[i],12,12,C_MUTED(),&lv_font_montserrat_16);
        regulatorValues[i]=home_text(tile,"--",12,43,C_TEXT(),&home_font_32);
        lv_obj_t *minus=home_control(tile,12,94,128,48,"-",nullptr);
        lv_obj_add_event_cb(minus,regulator_adjust_cb,LV_EVENT_CLICKED,(void *)(intptr_t)(-i-1));
        lv_obj_t *plus=home_control(tile,148,94,128,48,"+",nullptr);
        lv_obj_add_event_cb(plus,regulator_adjust_cb,LV_EVENT_CLICKED,(void *)(intptr_t)(i+1));
        home_text(tile,notes[i],12,155,C_MUTED(),&lv_font_montserrat_16);
    }
    regulatorMessage=home_text(regulatorPanel,"Zmiany zaczna dzialac po nacisnieciu ZAPISZ.",20,278,C_MUTED(),&lv_font_montserrat_16);
    home_control(regulatorPanel,20,315,288,56,"DOMYSLNE",regulator_defaults_cb);
    lv_obj_t *save=home_control(regulatorPanel,326,315,288,56,"ZAPISZ",regulator_save_cb);
    lv_obj_set_style_border_color(save,C_GREEN(),0);
    home_control(regulatorPanel,632,315,288,56,"WROC",regulator_back_cb);
    regulator_editor_refresh();
}
static void valve_auto_toggle_cb(lv_event_t *) {
    if(bench_test_active())return;
    auto_regulator_toggle_request();
}
static void create_valve_test()
{
    // The manual diagnostic screen takes control only after automatic mode is OFF.
    auto_regulator_stop();
    valve_test_stop();
    valvePanel = panel(settingsDetailPanel, 12, 12, 952, 396, C_ORANGE());
    valveStatus = label(valvePanel, "TEST ZABLOKOWANY", 24, 18, C_ORANGE());
    font24(valveStatus);
    valvePressure = label(valvePanel, "CISNIENIE: --.- bar", 24, 65, C_TEXT());
    font20(valvePressure);
    lv_obj_t *regSettings=button(valvePanel,24,105,876,62,"USTAWIENIA REGULATORA | impuls / przerwa / tolerancja",regulator_open_cb);
    font20(lv_obj_get_child(regSettings,0));
    valveAutoButton=home_control(valvePanel,24,178,432,46,"AUTO REG: WLACZ",valve_auto_toggle_cb);
    lv_obj_set_style_text_font(lv_obj_get_child(valveAutoButton,0),&home_font_18,0);
    valveAutoStatus=home_text(valvePanel,"WYLACZONY",472,190,C_ORANGE(),&lv_font_montserrat_16);
    lv_obj_set_width(valveAutoStatus,428);
    button(valvePanel, 24, 235, 276, 56, "KIERUNEK A - 1 s", valve_pulse_cb, (void *)(intptr_t)1);
    button(valvePanel, 324, 235, 276, 56, "KIERUNEK B - 1 s", valve_pulse_cb, (void *)(intptr_t)-1);
    button(valvePanel, 624, 235, 276, 56, "STOP / BLOKADA", valve_stop_cb);
    button(valvePanel, 24, 315, 400, 56, "ODBLOKUJ TEST", valve_arm_cb);
    valveDirectionButton=button(valvePanel,452,315,448,56,"ZAMIEN OTWORZ / ZAMKNIJ",valve_direction_cb);
    font16(lv_obj_get_child(valveDirectionButton,0));
    valve_refresh();
}

void ui_update_model()
{
    // No widget updates here. Run at 100 ms, even if visual refresh is not due.
    nozzle_update();
    bench_test_tick();
    tank_counter_update();
    job_counter_update();
}

void ui_update()
{
    job_refresh();
    info_refresh();
    char buf[96];
    snprintf(buf,sizeof(buf),"%s | %s  >",nozzle_name(),nozzle_liquid_name());set_label_text_if_changed(homeProfile,buf);
    if (simulationBar) {
        if (bench_test_active()) lv_obj_set_hidden(simulationBar, false);
        else lv_obj_set_hidden(simulationBar, true);
    }
    if (lv_screen_active()==scrSettings) quick_settings_refresh();
    if (benchPanel) bench_refresh();
    if (valvePanel) valve_refresh();

    if (flowCalRunning && (uint32_t)(millis() - flowCalRefreshMs) >= 250) {
        flowCalRefreshMs = millis();
        flow_cal_refresh();
    }

    const TankReading tankReading=tank_counter_read();
    if(tankReading.configured)snprintf(buf,sizeof(buf),"%.1f L",tankReading.liters);
    else snprintf(buf,sizeof(buf),"--- L");
    set_label_text_if_changed(lblTank,buf);

    set_label_text_if_changed(lblTankPct,tankReading.configured ? (tankReading.saveOk?"Wyliczone":"Blad zapisu") : "Dotknij: ustaw");
    const float wantedPressure=nozzle_target_bar();
    // Match the displayed AUTO target to pressure_control_core.h: 2.0-4.8 bar.
    // The nozzle table may calculate higher RSM pressures, but AUTO rejects them.
    if(std::isfinite(wantedPressure)&&wantedPressure>=2.0f&&wantedPressure<=4.8f)
        snprintf(buf,sizeof(buf),"%s\nCel cisnienia: %.2f bar",nozzle_status(),wantedPressure);
    else if(std::isfinite(wantedPressure))
        snprintf(buf,sizeof(buf),"%s\nCel poza zakresem AUTO (2.0-4.8 bar)",nozzle_status());
    else
        snprintf(buf,sizeof(buf),"%s\n%s",nozzle_status(),targetFlowX10()>0?"Cel poza zakresem cisnienia":"Cel cisnienia: --");
    set_label_text_if_changed(nozzleHomeInfo,buf);
    // Detail panels above keep updating; hidden home widgets wait until return.
    if (lv_screen_active() != scrHome) return;

    set_label_text_if_changed(
        lblWifi,
        gState.wifiOk
            ? LV_SYMBOL_WIFI " Wi-Fi"
            : LV_SYMBOL_WIFI " BRAK"
    );

    set_text_color_if_changed(
        lblWifi,
        gState.wifiOk
            ? C_GREEN()
            : C_RED(),
        0
    );

    set_label_text_if_changed(
        lblAog,
        gState.aogOk
            ? LV_SYMBOL_GPS " AgOpenGPS"
            : LV_SYMBOL_GPS " AOG BRAK"
    );

    set_text_color_if_changed(
        lblAog,
        gState.aogOk
            ? C_GREEN()
            : C_RED(),
        0
    );

    set_label_text_if_changed(
        lblUdp,
        gState.udpOk
            ? "UDP 8888"
            : "UDP --"
    );

    set_text_color_if_changed(
        lblUdp,
        gState.udpOk
            ? C_GREEN()
            : C_MUTED(),
        0
    );

    snprintf(
        buf,
        sizeof(buf),
        "%d.%d",
        sprayer_display_state().speedX10 / 10,
        abs(sprayer_display_state().speedX10 % 10)
    );

    set_label_text_if_changed(
        lblSpeed,
        buf
    );
    if (lv_obj_get_style_text_font(lblSpeed, LV_PART_MAIN) != (strlen(buf) > 4 ? &home_font_24 : &home_font_32))
        lv_obj_set_style_text_font(lblSpeed, strlen(buf) > 4 ? &home_font_24 : &home_font_32, 0);

    snprintf(
        buf,
        sizeof(buf),
        "%d L/ha",
        sprayer_display_state().rateLHa
    );

    set_label_text_if_changed(
        lblRate,
        buf
    );
    // The setpoint stays smaller than the measured rate in the shared card.


    set_text_color_if_changed(
        lblRate,
        C_TEXT(),
        0
    );

    int flowDisplayX10 = sprayer_display_state().actualFlowX10;
    if (flowDisplayX10 < 0 || flowDisplayX10 > 2500) {
        flowDisplayX10 = 0;
    }

    snprintf(
        buf,
        sizeof(buf),
        "%d.%d",
        flowDisplayX10 / 10,
        abs(flowDisplayX10 % 10)
    );

    set_label_text_if_changed(
        lblFlow,
        buf
    );
    fit_home_number(lblFlow,buf);

    set_text_color_if_changed(
        lblFlow,
        C_TEXT(),
        0
    );

    const float activeWidth=ui_get_active_boom_width_m();
    const auto &display=sprayer_display_state();
    const bool rateValid=display.speedX10>0 && activeWidth>0 &&
                         (bench_test_active() || (gState.aogOk && nozzle_reading_valid()));
    if(rateValid){
        const int actualRate=(int)(flowDisplayX10*600.0f/(display.speedX10*activeWidth)+.5f);
        snprintf(buf,sizeof(buf),"%d",actualRate);
        set_label_text_if_changed(lblActualRate,buf);fit_home_number(lblActualRate,buf);
        const bool inRange=abs(actualRate-display.rateLHa)*100<=display.rateLHa*(int)regulator_settings().tolerancePercent;
        set_text_color_if_changed(lblActualRate,inRange?C_GREEN():C_ORANGE(),0);
    }else{
        set_label_text_if_changed(lblActualRate,"--");
        set_text_color_if_changed(lblActualRate,C_MUTED(),0);
    }
    if(!bench_test_active()&&!nozzle_reading_valid()){
        set_label_text_if_changed(lblFlow,"--");set_label_text_if_changed(lblFlowCompare,"--");
    }
    const bool working=display.speedX10>0 && activeWidth>0;
    const char *status=bench_test_active()?"SYMULACJA":
        display.autoMode&&!gState.aogOk?"BRAK LACZNOSCI":
        !working?"POSTOJ":display.autoMode?"PRACA AUTO":"PRACA MAN";
    set_label_text_if_changed(lblWorkStatus,status);
    set_text_color_if_changed(lblWorkStatus,display.autoMode&&!gState.aogOk?C_RED():
                              working?(display.autoMode?C_GREEN():C_YELLOW()):C_MUTED(),0);

    int target =
        configured_target_flow_x10();

    snprintf(buf, sizeof(buf), "%d.%d", flowDisplayX10 / 10,
             abs(flowDisplayX10 % 10));
    set_label_text_if_changed(lblFlowCompare, !bench_test_active()&&!nozzle_reading_valid()?"--":buf);
    // Shared scale: whichever value is larger occupies 90 percent.
    const int scale = (flowDisplayX10 > target ? flowDisplayX10 : target);
    const int actualBar = scale > 0 ? (int)((int64_t)flowDisplayX10 * 900 / scale) : 0;
    const int targetBar = scale > 0 ? (int)((int64_t)target * 900 / scale) : 0;
    if (actualBar != previousActualBar) {
        lv_bar_set_value(flowActualBar, actualBar, LV_ANIM_OFF);
        previousActualBar = actualBar;
    }
    if (targetBar != previousTargetBar) {
        lv_bar_set_value(flowTargetBar, targetBar, LV_ANIM_OFF);
        previousTargetBar = targetBar;
    }
    snprintf(buf, sizeof(buf), "%.1f", ui_get_active_boom_width_m());
    set_label_text_if_changed(lblActiveWidth, buf);
    if (lv_obj_get_style_text_font(lblActiveWidth, LV_PART_MAIN) != (strlen(buf) > 4 ? &home_font_24 : &home_font_32))
        lv_obj_set_style_text_font(lblActiveWidth, strlen(buf) > 4 ? &home_font_24 : &home_font_32, 0);

    snprintf(
        buf,
        sizeof(buf),
        "%d.%d",
        target / 10,
        abs(target % 10)
    );

    set_label_text_if_changed(
        lblTarget,
        buf
    );

    const int error = configured_flow_error_percent(target);
    const char *reg = !bench_test_active()&&!nozzle_reading_valid()?"BRAK DANYCH": target < REGULATOR_MIN_TARGET_X10 ? "POSTOJ" :
                      abs(error) <= (int)regulator_settings().tolerancePercent ? "W CELU" :
                      error < 0 ? "ZA MALY" : "ZA DUZY";
    const bool within = target >= REGULATOR_MIN_TARGET_X10 && abs(error) <= (int)regulator_settings().tolerancePercent;
    if (flowWithinTarget != within) { flowWithinTarget=within; lv_obj_invalidate(flowStatusIcon); }
    lv_obj_set_hidden(flowStatusIcon, target < REGULATOR_MIN_TARGET_X10);


    set_label_text_if_changed(
        lblReg,
        reg
    );

    set_text_color_if_changed(
        lblReg,
        strcmp(reg, "W CELU") == 0
            ? C_GREEN()
            : C_ORANGE(),
        0
    );

    if (target < REGULATOR_MIN_TARGET_X10) {
        set_label_text_if_changed(
            lblErr,
            "--"
        );
    } else {
        snprintf(
            buf,
            sizeof(buf),
            "%+d%%",
            configured_flow_error_percent(target)
        );

        set_label_text_if_changed(
            lblErr,
            buf
        );
    }

    int totalDisplayX10 = sprayer_display_state().totalLitersX10;
    if (totalDisplayX10 < 0 || totalDisplayX10 > 500000) {
        totalDisplayX10 = 0;
    }

    snprintf(
        buf,
        sizeof(buf),
        "%d.%d L",
        totalDisplayX10 / 10,
        abs(totalDisplayX10 % 10)
    );

    set_label_text_if_changed(
        lblTotal,
        buf
    );

    // Pressure is always the real sensor reading, including bench mode.
    if (pressure_sensor_valid()) {
        const int pressureX10 = pressure_sensor_bar_x10();
        snprintf(buf, sizeof(buf), "%d.%d", pressureX10 / 10, pressureX10 % 10);
        set_label_text_if_changed(lblPressure, buf);
        fit_home_number(lblPressure,buf);
        set_text_color_if_changed(lblPressure, C_TEXT(), 0);
    } else {
        set_label_text_if_changed(lblPressure, "--.-");
        set_text_color_if_changed(lblPressure, C_ORANGE(), 0);
    }

    set_label_text_if_changed(
        lblMode,
        bench_test_active() ? "TEST" : sprayer_display_state().autoMode
            ? "AUTO"
            : "MAN"
    );

    set_text_color_if_changed(
        lblMode,
        sprayer_display_state().autoMode
            ? C_GREEN()
            : C_YELLOW(),
        0
    );

    lv_obj_t *modeBadge=lv_obj_get_parent(lblMode);
    const lv_color_t modeColor=sprayer_display_state().autoMode ? C_GREEN() : C_YELLOW();
    const lv_color_t modeFill=lv_color_hex(sprayer_display_state().autoMode ? 0x163D31 : 0x3B3216);
    if(!lv_color_eq(lv_obj_get_style_border_color(modeBadge,LV_PART_MAIN),modeColor))
        lv_obj_set_style_border_color(modeBadge,modeColor,0);
    if(!lv_color_eq(lv_obj_get_style_bg_color(modeBadge,LV_PART_MAIN),modeFill))
        lv_obj_set_style_bg_color(modeBadge,modeFill,0);

    if (bench_test_active()) {
        lv_obj_add_state(btnAuto, LV_STATE_DISABLED);
        lv_obj_add_state(btnMan, LV_STATE_DISABLED);
    } else {
        lv_obj_remove_state(btnAuto, LV_STATE_DISABLED);
        lv_obj_remove_state(btnMan, LV_STATE_DISABLED);
    }

    if (agrovalve::shouldStopMotorForUi(sprayer_display_state().autoMode,
                                           bench_test_active(), auto_regulator_armed(),
                                           valve_test_direction()!=0)) valve_test_stop();
    const bool valveDisabled=sprayer_display_state().autoMode || bench_test_active() || !valve_test_ready();
    for(lv_obj_t *b : {btnValveOpen,btnValveClose}) {
        if(lv_obj_has_state(b,LV_STATE_DISABLED)!=valveDisabled) {
            if(valveDisabled)lv_obj_add_state(b,LV_STATE_DISABLED);else lv_obj_remove_state(b,LV_STATE_DISABLED);
        }
    }
    const int valveDirection=valve_test_direction()*valve_open_direction();
    set_button_state(btnValveOpen,valveDirection==1);
    set_button_state(btnValveClose,valveDirection==-1);

    set_button_state(
        btnAuto,
        sprayer_display_state().autoMode
    );

    set_button_state(
        btnMan,
        !sprayer_display_state().autoMode
    );

    
    if(previousAutoMode!=sprayer_display_state().autoMode) {
        previousAutoMode=sprayer_display_state().autoMode;
        prevSectionStateValid=false;
    }
    // Sekcje: aktualizuj tylko po rzeczywistej zmianie stanu.
    // Ogranicza niepotrzebne invalidacje i przyspiesza reakcje dotyku.
    for (int i = 0; i < SECTION_COUNT; i++) {
        const bool now = sprayer_display_state().sections[i];

        if (!prevSectionStateValid || prevSectionState[i] != now) {
            prevSectionState[i] = now;

            if (sprayerSections[i]) {
                set_button_state(sprayerSections[i], now);
            }

            if (btnSections[i]) {
                set_button_state(btnSections[i], now);
            }
        }
    }
    prevSectionStateValid = true;
    refresh_sprayer_scene();

}
