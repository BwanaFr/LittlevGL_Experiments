#include <Arduino.h>
#include <ESP32_TFT.h>
#include <lvgl.h>
#include <Ticker.h>

#define LVGL_TICK_PERIOD 5
#define RA8875_SCLK    15
#define RA8875_CS      14
#define RA8875_MISO    13
#define RA8875_MOSI    25


#define LV_BUFFER_SIZE LV_HOR_RES_MAX * 100
Ticker tick; /* timer for interrupt handler */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_BUFFER_SIZE];

RA8875 tft = RA8875(RA8875_CS, 255);

lv_obj_t * slider_label;
lv_obj_t * slider;
int screenWidth = 800;
int screenHeight = 480;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  tft.setActiveWindow(area->x1, area->x2, area->y1, area->y2);
  //tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  int xLen = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
  /*for (int y = area->y1; y <= area->y2; y++) {
    int xLen = area->x2 - area->x1 + 1;
      //c = color_p->full;    
      color_p+=xLen;
  }*/
  tft.drawPixels((uint16_t*)color_p, xLen, area->x1, area->y1);
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/* Interrupt driven periodic handler */
static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}

/* Reading input device (simulated encoder here) */
bool read_encoder(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
  static int32_t last_diff = 0;
  int32_t diff = 0; /* Dummy - no movement */
  int btn_state = LV_INDEV_STATE_REL; /* Dummy - no press */

  data->enc_diff = diff - last_diff;;
  data->state = btn_state;

  last_diff = diff;

  return false;
}


void setup() {
  Serial.begin(115200);
  lv_init();

  Serial.println("TFT start");
  tft.begin(RA8875_800x480);
  
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_BUFFER_SIZE);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 800;
  disp_drv.ver_res = 480;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

/*Initialize the touch pad*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_ENCODER;
  indev_drv.read_cb = read_encoder;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  //Set the theme..
  /*lv_theme_t * th = lv_theme_night_init(210, NULL);     //Set a HUE value and a Font for the Night Theme
  lv_theme_set_current(th);*/

  lv_obj_t * scr = lv_cont_create(NULL, NULL);
  lv_disp_load_scr(scr);

  //lv_obj_t * tv = lv_tabview_create(scr, NULL);
  //lv_obj_set_size(tv, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));

  /* Create simple label */
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "Hello Arduino! (V6.1)");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, -50);

  /* Create a slider in the center of the display */
  slider = lv_slider_create(lv_scr_act(), NULL);
  lv_obj_set_width(slider, 100);                        /*Set the width*/
  lv_obj_set_height(slider, 50);
  lv_obj_align(slider, NULL, LV_ALIGN_CENTER, 0, 0);    /*Align to the center of the parent (screen)*/
  //lv_obj_set_event_cb(slider, slider_event_cb);         /*Assign an event function*/

  /* Create a label below the slider */
  slider_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(slider_label, "0");
  lv_obj_set_auto_realign(slider, true);
  lv_obj_align(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

}


void loop() {
  static long lastUpdate = 0;  
  lv_task_handler(); /* let the GUI do its work */
  long now = millis();
  if((now - lastUpdate) > 100){
    lastUpdate = now;
    int16_t value = lv_slider_get_value(slider)+1;
    if(value > 100){
      value = 0;
    }
    lv_slider_set_value(slider, value, true);
    String s = "";
    s += value;
    lv_label_set_text(slider_label, s.c_str());
  }
}