#include <Arduino.h>
#include <AudioTools.h> // clone into Documents/Arduino/libraries/ https://github.com/pschatzmann/arduino-audio-tools
#include <BluetoothA2DPSink.h> // clone into Documents/Arduino/libraries/ https://github.com/pschatzmann/ESP32-A2DP
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "driver/i2s.h"
#include <Wire.h>
#include <lvgl.h>
#include <M5Unified.h>
#include <map>
#include <mutex>
std::mutex lvgl_callback_mutex;

#define SYS_I2C_SDA_PIN  21
#define SYS_I2C_SCL_PIN  22
#define SYS_I2S_MCLK_PIN 0
#define SYS_I2S_SCLK_PIN 19
#define SYS_I2S_LRCK_PIN 27
#define SYS_I2S_DOUT_PIN 2
#define SYS_I2S_DIN_PIN  34
#define SYS_SPI_MISO_PIN 38
#define SYS_SPI_MOSI_PIN 23
#define SYS_SPI_CLK_PIN  18
#define SYS_SPI_CS_PIN   4

// I2S and Bluetooth sink
ES8388 es8388(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN);
I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

#define TFT_HOR_RES   320
#define TFT_VER_RES   240

typedef struct {
    M5GFX * tft;
} lv_tft_espi_t;

lv_tft_espi_t * dsc;

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

lv_obj_t * ui_system_status_symbols;
lv_obj_t * ui_volume_slider;
lv_obj_t * ui_pause_play;
lv_obj_t * ui_pause_play_icon;
lv_obj_t * ui_next_button;
lv_obj_t * ui_next_button_icon;
lv_obj_t * ui_prev_button;
lv_obj_t * ui_prev_button_icon;
lv_obj_t * ui_metadata_readout;
lv_timer_t * system_status_timer;
String last_system_status = "";

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    lv_draw_sw_rgb565_swap((lv_color_t *)px_map, w * h);
    dsc->tft->startWrite();
    dsc->tft->setAddrWindow(area->x1, area->y1, w, h);
    dsc->tft->pushImage(area->x1, area->y1, w, h, (uint16_t *)px_map);
    dsc->tft->endWrite();
    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    auto pos = M5.Touch.getTouchPointRaw();
    auto detail = M5.Touch.getDetail();
    bool touched = detail.isPressed();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } 
    else {
        data->state = LV_INDEV_STATE_PR; 
        /*Set the coordinates*/
        data->point.x = pos.x;
        data->point.y = pos.y;
        Serial.printf("Touch: %d %d\n", data->point.x, data->point.y);
    }
}

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    if (level == LV_LOG_LEVEL_ERROR) {
        Serial.print(buf);
    }
}
#endif

void status_icon_update(lv_timer_t * timer) {
    String status = "";

    if (a2dp_sink.is_connected()) {
        lv_obj_set_state(ui_pause_play, LV_STATE_DISABLED, false);
        lv_obj_set_state(ui_next_button, LV_STATE_DISABLED, false);
        lv_obj_set_state(ui_prev_button, LV_STATE_DISABLED, false);
        lv_obj_set_state(ui_volume_slider, LV_STATE_DISABLED, false);
        status += a2dp_sink.get_peer_name();
        status += " ";
        status += LV_SYMBOL_BLUETOOTH;
    }
    else {
        lv_obj_set_state(ui_pause_play, LV_STATE_DISABLED, true);
        lv_obj_set_state(ui_next_button, LV_STATE_DISABLED, true);
        lv_obj_set_state(ui_prev_button, LV_STATE_DISABLED, true);
        lv_obj_set_state(ui_volume_slider, LV_STATE_DISABLED, true);
    }

    bool charging = M5.Power.getBatteryCurrent() > 1;
    if (charging) {
        status += " ";
        status += LV_SYMBOL_CHARGE;
    }

    int battery_level = M5.Power.getBatteryLevel();
    status += " ";
    if (battery_level > 75) status += LV_SYMBOL_BATTERY_FULL;
    else if (battery_level > 50) status += LV_SYMBOL_BATTERY_3;
    else if (battery_level > 25) status += LV_SYMBOL_BATTERY_2;
    else if (battery_level > 10) status += LV_SYMBOL_BATTERY_1;
    else status += LV_SYMBOL_BATTERY_EMPTY;

    lv_label_set_text(ui_system_status_symbols, status.c_str());

    last_system_status = status;
}

void ui_event_volume_slider(lv_event_t *e) {
    int volume_value = lv_slider_get_value(ui_volume_slider);
    es8388.setDACVolume(volume_value);
}

void ui_event_pause_play(lv_event_t *e) {
    bool checked = lv_obj_has_state(ui_pause_play, LV_STATE_CHECKED);
    if (checked) {
        a2dp_sink.pause();
        lv_label_set_text(ui_pause_play_icon, LV_SYMBOL_PLAY);
    }
    else {
        a2dp_sink.play();
        lv_label_set_text(ui_pause_play_icon, LV_SYMBOL_PAUSE);
    }
}

void ui_event_next_button(lv_event_t *e) {
    a2dp_sink.next();
}

void ui_event_prev_button(lv_event_t *e) {
    a2dp_sink.previous();
}

/*
typedef enum {
    ESP_AVRC_MD_ATTR_TITLE = 0x1,                 // title of the playing track
    ESP_AVRC_MD_ATTR_ARTIST = 0x2,                // track artist 
    ESP_AVRC_MD_ATTR_ALBUM = 0x4,                 // album name 
    ESP_AVRC_MD_ATTR_TRACK_NUM = 0x8,             // track position on the album 
    ESP_AVRC_MD_ATTR_NUM_TRACKS = 0x10,           // number of tracks on the album 
    ESP_AVRC_MD_ATTR_GENRE = 0x20,                // track genre 
    ESP_AVRC_MD_ATTR_PLAYING_TIME = 0x40          // total album playing time in miliseconds 
} esp_avrc_md_attr_mask_t;
*/
typedef struct {
    String title = ""; // ESP_AVRC_MD_ATTR_TITLE
    String artist = ""; // ESP_AVRC_MD_ATTR_ARTIST
    String album = ""; // ESP_AVRC_MD_ATTR_ALBUM
    uint32_t track_num = 0; // ESP_AVRC_MD_ATTR_TRACK_NUM
    uint32_t num_tracks = 0; // ESP_AVRC_MD_ATTR_PLAYING_TIME
    uint32_t genre = 0; // ESP_AVRC_MD_ATTR_GENRE
    uint32_t playtime_ms = 0; // ESP_AVRC_MD_ATTR_PLAYING_TIME
    uint32_t current_play_position_ms = 0; // from callback 
} avrc_metadata_t;

avrc_metadata_t current_metadata;

uint32_t avrc_metadata_to_uint32_t(const uint8_t *metadata_data) {
    union {
        uint8_t i[4];
        uint32_t li;
    } converter;
    converter.i[0] = metadata_data[3];
    converter.i[1] = metadata_data[2];
    converter.i[2] = metadata_data[1];
    converter.i[3] = metadata_data[0];
    return converter.li;
}

float avrc_metadata_to_float(const uint8_t *metadata_data) {
    union {
        uint8_t i[4];
        float f;
    } converter;
    converter.i[0] = metadata_data[0];
    converter.i[1] = metadata_data[1];
    converter.i[2] = metadata_data[2];
    converter.i[3] = metadata_data[3];
    return converter.f;
}

void avrc_metadata_callback(uint8_t metadata_attr, const uint8_t * metadata_data) {
    std::lock_guard<std::mutex> lock(lvgl_callback_mutex);
    if (metadata_attr == ESP_AVRC_MD_ATTR_TITLE) {
        current_metadata.title = String((char*)metadata_data);
    }
    else if (metadata_attr == ESP_AVRC_MD_ATTR_ARTIST) {
        current_metadata.artist = String((char*)metadata_data);
    }
    else if (metadata_attr == ESP_AVRC_MD_ATTR_ALBUM) {
        current_metadata.album = String((char*)metadata_data);
    }
    else if (metadata_attr >= ESP_AVRC_MD_ATTR_TRACK_NUM) {
        String metadata_attr_str = (metadata_attr == ESP_AVRC_MD_ATTR_TRACK_NUM  ? "track num"  : (metadata_attr == ESP_AVRC_MD_ATTR_NUM_TRACKS ? "num tracks" : (metadata_attr == ESP_AVRC_MD_ATTR_GENRE ? "genre" : (metadata_attr == ESP_AVRC_MD_ATTR_PLAYING_TIME ? "play time" : ""))));
        Serial.printf("recieved metadata %s: ", metadata_attr_str);
        for (int i = 0; i < sizeof(metadata_data); i++) {
            Serial.printf("%d,", metadata_data[i]);
        }
        Serial.println();
    }
    bool not_provided = current_metadata.title.indexOf("Not Provided") >= 0;
    if (not_provided) {
        lv_label_set_text(ui_metadata_readout, "");
    }
    else {
        uint32_t play_pos_seconds = (current_metadata.current_play_position_ms / 1000);
        uint32_t play_pos_minutes = (play_pos_seconds / 60);
        lv_label_set_text_fmt(ui_metadata_readout, "%s by %s\n%s\n%02d:%02d", current_metadata.title.c_str(), current_metadata.artist.c_str(), current_metadata.album.c_str(), play_pos_minutes % 60, play_pos_seconds % 60);
    }
}

void avrc_playback_state_callback(esp_avrc_playback_stat_t playback) {
    std::lock_guard<std::mutex> lock(lvgl_callback_mutex);
    if (playback == ESP_AVRC_PLAYBACK_PAUSED) {
        lv_obj_set_state(ui_pause_play, LV_STATE_CHECKED, true);
        lv_label_set_text(ui_pause_play_icon, LV_SYMBOL_PLAY);
    }
    else if (playback == ESP_AVRC_PLAYBACK_PLAYING) {
        lv_obj_set_state(ui_pause_play, LV_STATE_CHECKED, false);
        lv_label_set_text(ui_pause_play_icon, LV_SYMBOL_PAUSE);
    }
    else if (playback == ESP_AVRC_PLAYBACK_STOPPED) {
        lv_label_set_text_fmt(ui_metadata_readout, "");
        avrc_metadata_t new_metadata;
        current_metadata = new_metadata;
    }
}

void avrc_play_pos_callback(uint32_t play_pos) {
    std::lock_guard<std::mutex> lock(lvgl_callback_mutex);
    current_metadata.current_play_position_ms = play_pos;
    uint32_t play_pos_seconds = (current_metadata.current_play_position_ms / 1000);
    uint32_t play_pos_minutes = (play_pos_seconds / 60);
    lv_label_set_text_fmt(ui_metadata_readout, "%s by %s\n%s\n%02d:%02d", current_metadata.title.c_str(), current_metadata.artist.c_str(), current_metadata.album.c_str(), play_pos_minutes % 60, play_pos_seconds % 60);
}

void avrc_track_change_callback(uint8_t * id) {
    lv_label_set_text_fmt(ui_metadata_readout, "");
    avrc_metadata_t new_metadata;
    current_metadata = new_metadata;
}

// Playback state
bool isPlaying = false;

void setup() {
    Serial.begin(115200);

    // Init M5 and display
    M5.begin();

    M5.Power.setVibration(128);
    delay(500);
    M5.Power.setVibration(0);
    M5.Lcd.init();

    lv_init(); // M5.Lcd.print* will not work after this point

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(millis);

    /* register print function for debugging */
    lv_log_register_print_cb( my_print );

    dsc = (lv_tft_espi_t *)lv_malloc_zeroed(sizeof(lv_tft_espi_t));
    LV_ASSERT_MALLOC(dsc);

    lv_display_t * disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

    dsc->tft = &M5.Lcd;
    lv_display_set_driver_data(disp, (void *)dsc);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x606060), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *ui_Screen1 = lv_obj_create(lv_scr_act());
    lv_obj_remove_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x606060), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_size(ui_Screen1, TFT_HOR_RES, TFT_VER_RES);

    ui_system_status_symbols = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_system_status_symbols, "");
    lv_obj_set_style_text_font(ui_system_status_symbols, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_system_status_symbols, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_system_status_symbols, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_width(ui_system_status_symbols, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_system_status_symbols, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_system_status_symbols, LV_ALIGN_TOP_RIGHT);
    // lv_obj_add_flag(ui_system_status_symbols, LV_OBJ_FLAG_HIDDEN);     /// Flags

    lv_timer_create( status_icon_update, 1000, NULL );

    ui_volume_slider = lv_slider_create(ui_Screen1);
    lv_slider_set_range(ui_volume_slider, 0, 80);
    lv_obj_set_align(ui_volume_slider, LV_ALIGN_CENTER);
    lv_obj_set_width(ui_volume_slider, TFT_HOR_RES -70);
    lv_obj_set_pos(ui_volume_slider, 0, 90);

    lv_obj_add_event_cb(ui_volume_slider, ui_event_volume_slider, LV_EVENT_VALUE_CHANGED, NULL);

    ui_pause_play = lv_button_create(ui_Screen1);
    lv_obj_set_align(ui_pause_play, LV_ALIGN_CENTER);
    lv_obj_set_width(ui_pause_play, 70);
    lv_obj_set_height(ui_pause_play, 55);
    lv_obj_set_pos(ui_pause_play, 0, 35);
    lv_obj_add_flag(ui_pause_play, LV_OBJ_FLAG_CHECKABLE);

    lv_obj_add_event_cb(ui_pause_play, ui_event_pause_play, LV_EVENT_CLICKED, NULL);

    ui_pause_play_icon = lv_label_create(ui_pause_play);
    lv_obj_set_align(ui_pause_play_icon, LV_ALIGN_CENTER);
    lv_label_set_text(ui_pause_play_icon, LV_SYMBOL_PAUSE);

    ui_next_button = lv_button_create(ui_Screen1);
    lv_obj_set_align(ui_next_button, LV_ALIGN_CENTER);
    lv_obj_set_width(ui_next_button, 70);
    lv_obj_set_height(ui_next_button, 55);
    lv_obj_set_pos(ui_next_button, 80, 35);

    lv_obj_add_event_cb(ui_next_button, ui_event_next_button, LV_EVENT_CLICKED, NULL);

    ui_next_button_icon = lv_label_create(ui_next_button);
    lv_obj_set_align(ui_next_button_icon, LV_ALIGN_CENTER);
    lv_label_set_text(ui_next_button_icon, LV_SYMBOL_NEXT);

    ui_prev_button = lv_button_create(ui_Screen1);
    lv_obj_set_align(ui_prev_button, LV_ALIGN_CENTER);
    lv_obj_set_width(ui_prev_button, 70);
    lv_obj_set_height(ui_prev_button, 55);
    lv_obj_set_pos(ui_prev_button, -80, 35);

    lv_obj_add_event_cb(ui_prev_button, ui_event_prev_button, LV_EVENT_CLICKED, NULL);

    ui_prev_button_icon = lv_label_create(ui_prev_button);
    lv_obj_set_align(ui_prev_button_icon, LV_ALIGN_CENTER);
    lv_label_set_text(ui_prev_button_icon, LV_SYMBOL_PREV);

    ui_metadata_readout = lv_label_create(ui_Screen1);
    lv_obj_set_align(ui_metadata_readout, LV_ALIGN_CENTER);
    lv_obj_set_width(ui_metadata_readout, TFT_HOR_RES - 10);
    lv_obj_set_height(ui_metadata_readout, 75);
    lv_obj_set_pos(ui_metadata_readout, 0, -45);
    lv_label_set_text(ui_metadata_readout, "");
    lv_obj_set_style_text_align(ui_metadata_readout, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(ui_metadata_readout, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Configure I2S pins for i2s stream object
    auto i2s_cfg = i2s.defaultConfig();
    i2s_cfg.pin_mck = SYS_I2S_MCLK_PIN;
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;  // BCLK pin
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;   // LRC pin
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN; 
    i2s_cfg.pin_data_rx = SYS_I2S_DIN_PIN; // DIN pin

    i2s_cfg.rx_tx_mode = audio_tools::RXTX_MODE;
    i2s_cfg.sample_rate = 48000;
    i2s_cfg.bits_per_sample = 16;

    i2s.begin(i2s_cfg);

    if (!es8388.init()) Serial.println("Init Fail");
    es8388.setADCInput(ADC_INPUT_LINPUT1_RINPUT1);
    es8388.setMicGain(MIC_GAIN_24DB);
    es8388.setADCVolume(100);
    // The volume output should not exceed 40, otherwise there will be noise or current sounds
    es8388.setDACVolume(lv_slider_get_max_value(ui_volume_slider));
    es8388.setDACOutput(DAC_OUTPUT_ALL);
    es8388.setBitsSample(ES_MODULE_ADC, BIT_LENGTH_16BITS);
    es8388.setSampleRate(SAMPLE_RATE_48K);

    // Start Bluetooth sink
    a2dp_sink.start("M5 Speaker");
    a2dp_sink.set_volume(100);
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_sink.set_avrc_rn_playstatus_callback(avrc_playback_state_callback);
    a2dp_sink.set_avrc_rn_play_pos_callback(avrc_play_pos_callback, 1);
    a2dp_sink.set_avrc_rn_track_change_callback(avrc_track_change_callback);
    a2dp_sink.set_avrc_rn_events({ESP_AVRC_RN_PLAY_STATUS_CHANGE , ESP_AVRC_RN_TRACK_CHANGE ,
                                    ESP_AVRC_RN_TRACK_REACHED_END , ESP_AVRC_RN_TRACK_REACHED_START ,
                                    ESP_AVRC_RN_PLAY_POS_CHANGED , ESP_AVRC_RN_BATTERY_STATUS_CHANGE ,
                                    ESP_AVRC_RN_SYSTEM_STATUS_CHANGE , ESP_AVRC_RN_APP_SETTING_CHANGE ,
                                    ESP_AVRC_RN_NOW_PLAYING_CHANGE , ESP_AVRC_RN_AVAILABLE_PLAYERS_CHANGE ,
                                    ESP_AVRC_RN_ADDRESSED_PLAYER_CHANGE ,
                                    ESP_AVRC_RN_UIDS_CHANGE,ESP_AVRC_RN_VOLUME_CHANGE});
    lv_slider_set_value(ui_volume_slider, lv_slider_get_max_value(ui_volume_slider), LV_ANIM_OFF);
}

unsigned long last_screen_update = 0;

void loop() {
    unsigned long now = millis();
    if ((now - last_screen_update) > 50) {
        std::lock_guard<std::mutex> lock(lvgl_callback_mutex);
        M5.update();
        lv_task_handler(); /* let the GUI do its work */
        last_screen_update = now;
    }
}