// ===========================================================
// CONFIG: เลือกโหมด PID + โหมดสถาปัตยกรรม
// ===========================================================

#define PID_SIMPLE 0
#define PID_TUSTIN 1
const byte pidMode = PID_SIMPLE;    // เปลี่ยนเป็น PID_TUSTIN เพื่อเทียบอัลกอริทึม

#define ARCH_PAPER      1           // แบบ paper: Timer1 COMPB → ADC → ADC ISR
#define ARCH_MY_TIMER   0           // แบบเรา: Timer1 COMPA ISR ครอบ ADC
const byte archMode = ARCH_PAPER;   // เปลี่ยนเป็น ARCH_MY_TIMER เพื่อเทียบสถาปัตยกรรม

// ===========================================================
// I/O Pins
// ===========================================================
const byte U_PWMP   = 3;  // PWM output u(t)
const byte Out_PWMP = 6;  // PWM output error (optional)
const byte adcPin   = 0;  // A0 – analog input y(t)

// ===========================================================
// Sampling parameters
// ===========================================================
const float fs = 976.165f;      // sampling frequency (Hz)
const float Ts = 1.0f / fs;     // sampling period (s)

// ===========================================================
// PID parameters (ใช้ร่วมกันทั้ง SIMPLE และ TUSTIN)
// ===========================================================
const float Kc = 0.8f;  // 0.8f; 1.2f;
const float Ti = 2.0f;  // 3.0f; 0.5f;
const float Td = 0.01f; // 0.01f; 0.05f;

// สำหรับ Simple PID ตาม AN964
const float Kp = Kc;
const float Ki = Kc / Ti;
const float Kd = Kc * Td;

// Setpoint defined in normalized ADC scale:= 0.951 
//volatile float x = 973.0f / 1023.0f;

// Setpoint defined in normalized ADC scale:= 0.900
volatile float x = 921.0f / 1023.0f;

// Setpoint defined in normalized ADC scale:= 0.7 
//volatile float x = 716.0f / 1023.0f;

// Setpoint defined in normalized ADC scale:= 0.503 (ADC ≈ 515, ~2.5 V at Vref = 5 V)
//volatile float x = 515.0f / 1023.0f;

// Setpoint defined in normalized ADC scale:= 0.2 (1V)
//volatile float x = 205.0f / 1023.0f;

// ===========================================================
// State variables
// ===========================================================
volatile float y    = 0.0f;   // measured output (0..1)
volatile float u    = 0.0f;   // control output (0..1)

volatile float e    = 0.0f;   // error e(k)
volatile float e_1  = 0.0f;   // error e(k-1)

volatile float ui   = 0.0f;   // integral term
volatile float ui_1 = 0.0f;   // integral term (previous) สำหรับ Tustin
volatile float ud   = 0.0f;   // derivative term

volatile int   upwm = 0;      // PWM 0..255
volatile int   out  = 0;      // error mapping for PWM 0..255

// ===========================================================
// Serial debug variables
// ===========================================================
const uint16_t SERIAL_DECIMATION = 10;
volatile uint16_t isrCounter = 0;

volatile float dbg_x = 0.0f;
volatile float dbg_y = 0.0f;
volatile float dbg_e = 0.0f;
volatile float dbg_u = 0.0f;
volatile unsigned long dbg_n = 0;

volatile bool sampleReady = false;

// ===========================================================
// ฟังก์ชันรวม: คำนวณ PID (ทั้ง SIMPLE และ TUSTIN)
// ===========================================================
inline void computePID()
{
  e = x - y;   // error

  if (pidMode == PID_SIMPLE) {
    // -------- Simple PID (Euler / AN964) --------
    ui += Ki * Ts * e;
    if (ui > 1.0f)       ui = 1.0f;
    else if (ui < -1.0f) ui = -1.0f;

    ud = Kd * (e - e_1) / Ts;

    u = Kp * e + ui + ud;
  } else {
    // -------- Tustin PID (แบบ paper) ----------
    ui = ui_1 + Kc / Ti * Ts / 2.0f * (e + e_1);
    if (ui > 1.0f)       ui = 1.0f;
    else if (ui < -1.0f) ui = -1.0f;

    ud = Kc * Td / Ts * (e - e_1);

    u = Kc * e + ui + ud;

    ui_1 = ui;  // เก็บ integral เดิม
  }

  // optional feed-forward จาก setpoint (เหมือนโค้ด paper)
  // u = u + x;  //debug here!

  e_1 = e;      // เก็บ error รอบก่อน
}

// ===========================================================
// ARCH_PAPER: ADC ISR (Timer1 COMPB ยิง ADC → ADC_vect ทำ PID)
// ===========================================================
ISR(ADC_vect)
{
  if (archMode != ARCH_PAPER) return;   // ใช้เฉพาะโหมด paper

  digitalWrite(LED_BUILTIN, HIGH);

  // อ่าน ADC (10-bit → 0..1)
  //y = (float)ADC / 1024.0f;  // debug here!
  y = (float)ADC / 1023.0f;

  // คำนวณ PID ตามโหมดที่เลือก
  computePID();

  // จำกัด u แล้ว map ไป PWM
  if (u > 1.0f)      u = 1.0f;
  else if (u < 0.0f) u = 0.0f;

  upwm = (int)(u * 255.0f + 0.5f);
  out  = (int)((e + 1.0f) * 127.0f + 0.5f);

  analogWrite(U_PWMP,   upwm);
  analogWrite(Out_PWMP, out);

  // เตรียมข้อมูล Serial (decimation)
  isrCounter++;
  if (isrCounter >= SERIAL_DECIMATION) {
    isrCounter = 0;
    dbg_x = x; dbg_y = y; dbg_e = e; dbg_u = u;
    dbg_n++;
    sampleReady = true;
  }

  digitalWrite(LED_BUILTIN, LOW);
}

// Timer1 Compare B: ใช้เป็นตัว "ยิง" ADC ในสถาปัตยกรรม paper
ISR(TIMER1_COMPB_vect)
{
  if (archMode != ARCH_PAPER) return;
  ADCSRA |= bit(ADSC);   // start ADC conversion
}

// ===========================================================
// ARCH_MY_TIMER: Timer ISR ครอบ ADC (แบบที่อาจารย์เสนอ)
// ===========================================================
ISR(TIMER1_COMPA_vect)
{
  if (archMode != ARCH_MY_TIMER) return;

  digitalWrite(LED_BUILTIN, HIGH);

  // อ่านผล ADC รอบที่แล้ว
  //y = (float)ADC / 1024.0f;  // debug here
  y = (float)ADC / 1023.0f;

  // คำนวณ PID ตามโหมดที่เลือก
  computePID();

  // จำกัด u แล้ว map ไป PWM
  if (u > 1.0f)      u = 1.0f;
  else if (u < 0.0f) u = 0.0f;

  upwm = (int)(u * 255.0f + 0.5f);
  out  = (int)((e + 1.0f) * 127.0f + 0.5f);

  analogWrite(U_PWMP,   upwm);
  analogWrite(Out_PWMP, out);

  // Serial decimation
  isrCounter++;
  if (isrCounter >= SERIAL_DECIMATION) {
    isrCounter = 0;
    dbg_x = x; dbg_y = y; dbg_e = e; dbg_u = u;
    dbg_n++;
    sampleReady = true;
  }

  // สั่งเริ่ม ADC สำหรับรอบถัดไป
  ADCSRA |= bit(ADSC);

  digitalWrite(LED_BUILTIN, LOW);
}

// ===========================================================
// SETUP
// ===========================================================
void setup ()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(U_PWMP,  OUTPUT);
  pinMode(Out_PWMP, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("===== PID Architecture Test ====="));
  Serial.print(F("PID mode = "));
  Serial.println(pidMode == PID_SIMPLE ? F("SIMPLE") : F("TUSTIN"));
  Serial.print(F("Architecture = "));
  Serial.println(archMode == ARCH_PAPER ? F("PAPER_ADC_ISR") : F("MY_TIMER_ISR"));
  Serial.println(F("t_sec,x,y,e,u"));

  // ---------- Timer1: CTC @ fs ----------
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // CTC mode, prescaler 8
  TCCR1B = bit(CS11) | bit(WGM12);

  OCR1A = 2047;  // ~976 Hz

  // ให้ Compare B อยู่ที่คาบเดียวกัน (ใช้ใน ARCH_PAPER)
  OCR1B = 2047;  // ถ้า Tinkercad งอแง ลองเปลี่ยนเป็น 2046 ได้

  if (archMode == ARCH_PAPER) {
    TIMSK1 = bit(OCIE1B);      // ใช้ Timer1 COMPB interrupt
  } else { // ARCH_MY_TIMER
    TIMSK1 = bit(OCIE1A);      // ใช้ Timer1 COMPA interrupt
  }

  // ---------- ADC ----------
  ADMUX = bit(REFS0) | (adcPin & 7);   // AVcc, A0

  if (archMode == ARCH_PAPER) {
    // มี ADC interrupt, Timer1 COMPB เป็นตัวสั่ง ADSC ผ่าน ISR
    ADCSRA = bit(ADEN) | bit(ADIE) | bit(ADPS2); // Enable + ADC ISR + prescaler 16
    ADCSRA |= bit(ADSC);                         // start first conversion
  } else {
    // Manual ADC (อ่านใน Timer ISR, ไม่มี ADC ISR)
    ADCSRA = bit(ADEN) | bit(ADPS2);             // Enable ADC, no interrupt
    ADCSRA |= bit(ADSC);                         // start first conversion
  }
}

// ===========================================================
// LOOP: พิมพ์ค่า debug t_sec,x,y,e,u
// ===========================================================
void loop ()
{
  if (sampleReady) {
    float x_loc, y_loc, e_loc, u_loc;
    unsigned long n_loc;

    noInterrupts();
    x_loc = dbg_x;
    y_loc = dbg_y;
    e_loc = dbg_e;
    u_loc = dbg_u;
    n_loc = dbg_n;
    sampleReady = false;
    interrupts();

    float t = n_loc * Ts;

    Serial.print(t, 6);     Serial.print(',');
    Serial.print(x_loc, 4); Serial.print(',');
    Serial.print(y_loc, 4); Serial.print(',');
    Serial.print(e_loc, 4); Serial.print(',');
    Serial.println(u_loc, 4);
  }
}
