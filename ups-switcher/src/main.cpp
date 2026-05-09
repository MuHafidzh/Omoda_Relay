#include <Arduino.h>
#include "RSoftSerial.hpp"

#define LED_BUILTIN 13

/***** OUTPUT *****/
#define REL_AC_A  6
#define REL_AC_B  5
#define BUZZER    4
#define REL_DC_A  PIN_A0

/***** INPUT *****/
#define BTN_A     2
#define AC_DET    PIN_A3
#define DC_DET    PIN_A4

RSoftSerial db_sub(3);

int main_state = 0;

// Time for waiting camera to bootup
const unsigned long pc_on_delay_s = 60;

// Duration waiting ecoflow to turn on
const unsigned long init_wait_s = 5;

// Time for waiting PC to fully off
const unsigned long pc_poff_wait_s = 60;

// Time for waiting PC to bootup
const unsigned long pc_boot_timeout_s = 30;

// Timeout for ecoflow to turn on
const unsigned long ecoflow_timeout_s = 60;

// Duration for DC to remain active
const unsigned long dc_gone_timeout_s = 30;

// Wait timeout for PC to send shutdown command
const unsigned long pc_poff_timeout_s = 300;

// Time of long button press
const unsigned long button_long_press_ms = 2000;

// Time of short button press
const unsigned long button_short_press_ms = 100;

// Minimum number of AC detections to consider AC is on
const int ac_det_min_cnt = 10;

void bripip();
void bupup();
void beep();
void led_blink(int cnt);
bool is_button_long_pressed();
bool is_button_short_pressed();
bool check_serial(const String &str);
bool is_dc_on();
bool is_ac_on();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REL_AC_A, OUTPUT);
  pinMode(REL_AC_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(REL_DC_A, OUTPUT);

  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(AC_DET, INPUT);
  pinMode(DC_DET, INPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(REL_AC_A, LOW);
  digitalWrite(REL_AC_B, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(REL_DC_A, LOW);

  Serial.begin(9600);
  db_sub.begin(9600);

  // while(1) {
  //   if(db_sub.available()) {
  //     auto str = db_sub.readStringUntil('\n');
  //     Serial.print(str);
  //   }
  //   if(Serial.available()) {
  //     auto str = Serial.readStringUntil('\n');
  //     Serial.print(str);
  //   }
  //   delay(10);
  // }

}

void loop() {

  static auto last_time = millis();
  static unsigned long timer_sec = 0; 
  static int fail_ctr = 0;

  static char send_code = '0';

  static bool force_on = false;

  led_blink(main_state);
  
  auto timer_ms = millis() - last_time;
  if(timer_ms > 1000) {
    last_time = millis();
    timer_sec += 1;
  }

  auto reset_timer = []() {
    last_time = millis();
    timer_sec = 0;
  };

  
  if(is_button_long_pressed()) {
    if(main_state < 7) {
      main_state = 7;
      digitalWrite(REL_AC_A, HIGH);
      digitalWrite(REL_AC_B, HIGH);
      digitalWrite(REL_DC_A, HIGH);
      beep();
      force_on = true;
    }
    else {
      main_state = 10;
      fail_ctr = 0;
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(REL_AC_A, LOW);
      digitalWrite(REL_AC_B, LOW);
      digitalWrite(REL_DC_A, HIGH);
      bupup();

      force_on = false;
    }
    reset_timer();
  }

  switch(main_state) {
  case -1: // Error
    ; // Do nothing
    if(is_button_short_pressed())
    {
      digitalWrite(REL_AC_A, LOW);
      digitalWrite(REL_AC_B, LOW);
      digitalWrite(REL_DC_A, HIGH);
      send_code = '0';
      fail_ctr = 0;
      reset_timer();
      main_state = 0;
    }
    break;

  case 0: // Initialize
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(REL_AC_A, LOW);
    digitalWrite(REL_AC_B, LOW);
    digitalWrite(REL_DC_A, HIGH);
    
    send_code = '0';

    if(!is_dc_on()) {
      break;
    }

    if(fail_ctr < 5) {
      beep();
      main_state = 1;
      reset_timer();
    }
    else {
      for(int i=0; i<3; i++) {
        bupup();
        delay(1000);
      }
      main_state = -1;
    }
    break;

  case 1: // Wait for init delay
    if(timer_ms > 200) {
      main_state = 2;
      digitalWrite(REL_DC_A, HIGH);
      digitalWrite(REL_AC_A, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      reset_timer();
    };
    break;

  case 2: // Wait few seconds for ecoflow to on
    if(timer_sec > init_wait_s) {
      main_state = 3;
      reset_timer();
    }
    break;

  case 3: // Check if AC is on
    {
      static int ac_det_ctr = 0;
      if(is_ac_on()) {
        ac_det_ctr += 1;
      }

      if(ac_det_ctr > ac_det_min_cnt) {
        main_state = 4;
        ac_det_ctr = 0;
        reset_timer();
      }

      if(timer_sec > ecoflow_timeout_s) {
        digitalWrite(REL_DC_A, HIGH);
        main_state = 10;
        fail_ctr += 5;
      }
    }
    break;

  case 4: // Wait few seconds for camera bootup
    if(timer_sec > pc_on_delay_s) {
      main_state = 5;
    }
    break;

  case 5: // Switch on AC for PC
    digitalWrite(REL_AC_B, HIGH);
    reset_timer();
    main_state = 6;
    break;

  case 6: // Wait PC to bootup
    if(check_serial("its0")) {
      main_state = 7;
      bripip();
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if(timer_sec > pc_boot_timeout_s) {
      // main_state = 0;
      // fail_ctr += 1;

      // bypass
      main_state = 7;
    }
    break;

  case 7: // Idle
    
    // Check if DC A is on
    if(is_dc_on() || force_on) {
      reset_timer();
      send_code = '0';
    }

    if(timer_sec > dc_gone_timeout_s) {
      main_state = 8;
      send_code = '1';
      reset_timer();
    }

    if(check_serial("its1")) {
      main_state = 8;
      send_code = '1';
      reset_timer();
    }   
    else if(check_serial("its9")) {
      force_on = true;
    }

    break;

  case 8: // wait PC to process the shutdown request
    if(check_serial("its1")) {
      main_state = 9;
      bripip();
      digitalWrite(LED_BUILTIN, LOW);
      reset_timer();
      send_code = '2';
    } 
    else if(check_serial("its0")) {
      main_state = 7;
      reset_timer();
      send_code = '0';
    }

    if(timer_sec > pc_poff_timeout_s) {
      main_state = 9;
      fail_ctr += 1;
      send_code = '2';
    }
    break;

  case 9: 
    if(timer_sec > pc_poff_wait_s) {
      main_state = 10;
      digitalWrite(REL_AC_A, LOW);
      digitalWrite(REL_AC_B, LOW);
      digitalWrite(REL_DC_A, HIGH);

      // Should not execute until here
      reset_timer();
      send_code = '3';

      force_on = false;
    }
    break;

  case 10: // Wait for capacitor to discharge
    if(timer_sec > dc_gone_timeout_s) {
      main_state = 0;
    }
    break;

  default:
    main_state = 0;
    fail_ctr += 1;
    break;

  } /***** END OF CASE *****/

  static unsigned long send_last_time = millis();

  if((millis() - send_last_time) > 1000) {
    send_last_time = millis();
    Serial.print("its");
    Serial.print(send_code);
    Serial.write('\n');
  }

  delay(10);
}

void bripip() {
  for(int i=0; i<6; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(20);
    digitalWrite(BUZZER, LOW);
    delay(20);
  }
}

void bupup() {
  for(int i=0; i<4; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
}

void led_blink(int cnt) {
  static int led_seq_state = 0;
  static int led_cnt = 0;
  static unsigned long led_last_time = millis();
  switch(led_seq_state) {
  case 0: 
    led_cnt = cnt;
    led_seq_state = 1;
    break;
  case 1:
    if(led_cnt > 0) {
      digitalWrite(LED_BUILTIN, HIGH);
      led_last_time = millis();
      led_seq_state = 2;
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
      led_seq_state = 4;
    }
    break;
  case 2:
    if((millis() - led_last_time) > 100) {
      digitalWrite(LED_BUILTIN, LOW);
      led_last_time = millis();
      led_seq_state = 3;
      led_cnt -= 1;
    }
    break;
  case 3:
    if((millis() - led_last_time) > 100) {
      led_seq_state = 1;
    }
    break;
  case 4:
    if((millis() - led_last_time) > 1000) {
      led_seq_state = 0;
    }
    break;
  default:
    led_seq_state = 0;
    break;
  }
}

bool is_button_long_pressed() {
  static unsigned long btn_last_time = millis();
  static bool btn_state = 0;

  if(digitalRead(BTN_A) == LOW) {
    if(millis() - btn_last_time > button_long_press_ms) {
      if(btn_state == 0) {
        beep();
        btn_state = 1;
        return true; // Button pressed for more than 1 second
      }
    }
  }
  else {
    btn_state = 0;
    btn_last_time = millis();
  }

  return false;
}

bool is_button_short_pressed() {
  static unsigned long btn_last_time = millis();
  static bool btn_state = 0;

  if(digitalRead(BTN_A) == LOW) {
      btn_state = true;
  }
  else if(digitalRead(BTN_A) == HIGH && btn_state) {
    btn_state = false;
    auto duration = millis() - btn_last_time;
    if(duration > button_short_press_ms && duration < button_long_press_ms)
      return true;
  }
  else {
    btn_state = 0;
    btn_last_time = millis();
  }

  return false;
}

bool check_serial(const String &str) {
  if(Serial.available()) {
    auto str = Serial.readStringUntil('\n');
      // str.remove(str.length() - 1);
      if(str.length() < 4) return false;
      if(str.endsWith(str)) {
        Serial.flush();
        db_sub.flush();
        return true;
      }
  }
  else if(db_sub.available()) {
    auto str = db_sub.readStringUntil('\n');
      // str.remove(str.length() - 1);
      if(str.length() < 4) return false;
      if(str.endsWith(str)) {
        Serial.flush();
        db_sub.flush();
        return true;
      }
  }
  return false;
}

bool is_dc_on() {
  if(analogRead(DC_DET) > 300) {
    return true; // DC is on
  }
  return false;
}

bool is_ac_on() {
  if(analogRead(AC_DET) > 300) {
    return true; // DC is on
  }
  return false;
}