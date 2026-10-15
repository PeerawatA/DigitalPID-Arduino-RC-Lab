// ===========================================================
// PID_2order_RCRC_Arduino_paper_vs_Ts_Tustin_vs_Simple_v3_4_TIMING.ino
//
// v3.4 (แก้จาก v3.3):
//   - เพิ่ม CTRL_MODE = MODE_OPEN_LOOP สำหรับทดสอบ noise ของการวัด
//     u(t) คงที่ = U_OPEN (ไม่ใช้ผล PID แต่ยังคำนวณ PID เพื่อให้ภาระ ISR เท่าเดิม)
//     รัน OPEN_RUN_TIME_S วินาที
//   - เพิ่มสถิติค่า ADC ดิบของ y จาก "ทุกรอบ ISR" ช่วงสภาวะคงตัว
//     (หลัง SS_START วินาที): mean, std, min, max -> noise floor ของการวัด
//     (closed loop: หลัง 20 s, open loop: หลัง 2 s)
//
// v3.3 (แก้จาก v3.2):
//   - ก่อนเริ่มควบคุม ดึงขา PWM u(t) ลง LOW ค้างไว้ DISCHARGE_MS (1 s)
//     เพื่อคายประจุ C1, C2 ให้ระบบเริ่มจาก y(0) = 0 ทุกครั้ง
//     (v3.2 พบว่าถ้ากด Reset ขณะวงจรยังมีประจุจากรอบก่อน
//      ค่า y เริ่มต้นไม่เท่ากันในแต่ละไฟล์ -> ผลตอบสนองช่วงต้นต่างกัน)
//   - ค่าคงตัวเวลาช้าสุดของวงจร RC = 28.8 ms -> 1 s = ~35 เท่า (คายประจุหมด)
//
// v3.2 (แก้จาก v3.1):
//   - ระหว่างรันส่งข้อมูลเพียง 5 คอลัมน์ t,x,y,e,u (ทศนิยม 4 ตำแหน่ง)
//     เหมือนการทดลองเดิม -> ภาระ Serial/USB เท่าเดิม
//     (v3.1 ส่ง 11 คอลัมน์ และพบสัญญาณรบกวนสภาวะคงตัวสูงขึ้น ~65%)
//   - สถิติ jitter และ T_exec ยังเก็บจาก "ทุกรอบ ISR" ใน RAM
//     แล้วพิมพ์เป็นสรุปตอนจบ (บรรทัดขึ้นต้นด้วย '#')
//   - เพิ่ม histogram ของ T_exec ในส่วนสรุป
//
// ส่วนควบคุม (PID, ADC, PWM, Timer1) เหมือนการทดลองเดิมทุกประการ
//
// Serial output:
//   หัวไฟล์ (ข้อความ) -> ข้อมูล t_sec,x,y,e,u ทุก 10 รอบ ISR
//   -> ส่วนสรุป '#' -> "# END"
//
// วิธีบันทึกด้วย CoolTerm:
//   Start Capture -> กด Reset บนบอร์ด -> รอจนขึ้น "# END" -> Stop Capture
// วิเคราะห์ด้วย PID_Experimental_Data_Analysis_v2.sce
// ===========================================================

// ===========================================================
// CONFIG
// ===========================================================
#define PID_SIMPLE 0
#define PID_TUSTIN 1
const byte pidMode = PID_TUSTIN;    // PID_SIMPLE หรือ PID_TUSTIN

#define ARCH_PAPER      1           // Timer1 COMPB -> ADC -> ADC ISR (ADC-based)
#define ARCH_MY_TIMER   0           // Timer1 COMPA ISR ครอบ ADC (Timer-based)
const byte archMode = ARCH_PAPER;   // ARCH_PAPER หรือ ARCH_MY_TIMER

#define LOG_STREAM   0              // ส่ง t,x,y,e,u ทุก 10 รอบ + สรุปตอนจบ (ใช้เป็นหลัก)
#define LOG_SUMMARY  1              // ไม่ส่งระหว่างรัน, พิมพ์เฉพาะสรุปตอนจบ
const byte logMode = LOG_SUMMARY;    // LOG_STREAM หรือ LOG_SUMMARY

#define MODE_CLOSED_LOOP 0           // ควบคุมด้วย PID (การทดลองหลัก)
#define MODE_OPEN_LOOP   1           // u คงที่ = U_OPEN (ทดสอบ noise ของการวัด)
const byte ctrlMode = MODE_CLOSED_LOOP;   // MODE_CLOSED_LOOP หรือ MODE_OPEN_LOOP
const float U_OPEN           = 0.90f;     // u คงที่ในโหมด open loop (DC gain = 1 -> y ≈ 0.9)
const float OPEN_RUN_TIME_S  = 30.0f;     // ความยาวการทดสอบ open loop (s)

const float RUN_TIME_S = 100.0f;    // ความยาวการทดลอง (s) - ใช้ค่าเดียวกันทุกไฟล์
const unsigned long DISCHARGE_MS = 1000;  // คายประจุวงจร RC ก่อนเริ่ม (ms)

// ===========================================================
// I/O Pins
// ===========================================================
const byte U_PWMP   = 3;  // PWM output u(t)   -> OC2B (Timer2)
const byte Out_PWMP = 6;  // PWM output error  -> OC0A (Timer0)
const byte adcPin   = 0;  // A0 - analog input y(t)

// ===========================================================
// Sampling parameters
// ===========================================================
const float fs = 976.165f;      // ค่าที่ใช้ในสมการ PID (เหมือนการทดลองเดิม)
const float Ts = 1.0f / fs;

// คาบจริงของฮาร์ดแวร์: (OCR1A+1) x 0.5 us = 1024 us พอดี
const float Ts_hw = 1.024e-3f;
const unsigned long RUN_TICKS = (unsigned long)(
    ((ctrlMode == MODE_OPEN_LOOP) ? OPEN_RUN_TIME_S : RUN_TIME_S) / 1.024e-3f);
// เริ่มเก็บสถิติ noise ของ y หลังเวลานี้ (สภาวะคงตัว)
const unsigned long SS_START_TICKS = (unsigned long)(
    ((ctrlMode == MODE_OPEN_LOOP) ? 2.0f : 20.0f) / 1.024e-3f);

const float TICK_TO_US = 0.5f;  // Timer1 prescaler 8 @ 16 MHz

// ===========================================================
// PID parameters
// ===========================================================
const float Kc = 0.8f;
const float Ti = 2.0f;
const float Td = 0.01f;

const float Kp = Kc;
const float Ki = Kc / Ti;
const float Kd = Kc * Td;

// Setpoint (normalized ADC scale)
volatile float x = 921.0f / 1023.0f;     // = 0.900
//volatile float x = 973.0f / 1023.0f;   // = 0.951

// ===========================================================
// State variables
// ===========================================================
volatile float y    = 0.0f;
volatile float u    = 0.0f;
volatile float e    = 0.0f;
volatile float e_1  = 0.0f;
volatile float ui   = 0.0f;
volatile float ui_1 = 0.0f;
volatile float ud   = 0.0f;
volatile int   upwm = 0;
volatile int   out  = 0;

// ===========================================================
// Timing measurement (หน่วย: tick ของ Timer1 = 0.5 us)
//   L(k) = TCNT1 ณ ต้น ISR   (ค่าหน่วงนับจาก compare match)
//   J(k) = L(k) - L(k-1)     (= T_s(k) - T_s,nominal พอดี)
//   T_exec(k) = TCNT1 หลัง - TCNT1 ก่อน computePID()
// คำนวณใน ISR ด้วยจำนวนเต็มทั้งหมด
// ===========================================================
volatile unsigned long isrTickCount = 0;
volatile uint16_t lat_prev  = 0;
volatile bool     lat_first = true;

const uint8_t JHIST_BINS  = 17;   // |J| = 0..15 ticks และ >= 16 ticks (0.5 us/bin)
const uint8_t THIST_BINS  = 16;   // T_exec: <50, 50-55, ..., 115-120, >=120 us (5 us/bin)
const uint16_t THIST_LO   = 100;  // 50 us  = 100 ticks
const uint16_t THIST_STEP = 10;   // 5 us   = 10 ticks

volatile bool          runDone  = false;
volatile unsigned long tot_n    = 0;
volatile unsigned long tot_njit = 0;
volatile int32_t       tot_jsum = 0;
volatile uint32_t      tot_jsq  = 0;
volatile int16_t       tot_jmin = 0;
volatile int16_t       tot_jmax = 0;
volatile uint16_t      tot_lmin = 0xFFFF;
volatile uint16_t      tot_lmax = 0;
volatile uint32_t      tot_lsum = 0;
volatile unsigned long tot_tn   = 0;
volatile uint16_t      tot_tmin = 0xFFFF;
volatile uint16_t      tot_tmax = 0;
volatile uint32_t      tot_tsum = 0;
volatile uint64_t      tot_tsq  = 0;
volatile uint32_t      jhist[JHIST_BINS];

// สถิติค่า ADC ดิบของ y ช่วงสภาวะคงตัว (ทุกรอบ ISR)
volatile unsigned long adc_n   = 0;
volatile uint32_t      adc_sum = 0;
volatile uint64_t      adc_sq  = 0;
volatile uint16_t      adc_min = 0xFFFF;
volatile uint16_t      adc_max = 0;

inline void recordADC(uint16_t a)
{
  if (!runDone && isrTickCount >= SS_START_TICKS) {
    adc_n++;
    adc_sum += a;
    adc_sq  += (uint32_t)a * a;
    if (a < adc_min) adc_min = a;
    if (a > adc_max) adc_max = a;
  }
}
volatile uint32_t      thist[THIST_BINS];

inline void measureTiming(uint16_t tc)
{
  isrTickCount++;
  if (lat_first) {
    lat_first = false;
    lat_prev  = tc;
    return;
  }
  int16_t  d = (int16_t)(uint16_t)(tc - lat_prev);
  uint16_t a = (d < 0) ? (uint16_t)(-d) : (uint16_t)d;
  lat_prev = tc;

  if (!runDone) {
    tot_n++;
    if (a) tot_njit++;
    tot_jsum += d;
    tot_jsq  += (uint32_t)a * a;
    if (d < tot_jmin) tot_jmin = d;
    if (d > tot_jmax) tot_jmax = d;
    if (tc < tot_lmin) tot_lmin = tc;
    if (tc > tot_lmax) tot_lmax = tc;
    tot_lsum += tc;
    jhist[(a < JHIST_BINS - 1) ? a : (JHIST_BINS - 1)]++;
  }
}

inline void recordTexec(uint16_t dt)
{
  if (!runDone) {
    tot_tn++;
    if (dt < tot_tmin) tot_tmin = dt;
    if (dt > tot_tmax) tot_tmax = dt;
    tot_tsum += dt;
    tot_tsq  += (uint32_t)dt * dt;

    uint8_t b;
    if (dt < THIST_LO) {
      b = 0;
    } else {
      uint16_t k = (dt - THIST_LO) / THIST_STEP + 1;
      b = (k < THIST_BINS - 1) ? (uint8_t)k : (THIST_BINS - 1);
    }
    thist[b]++;
  }
}

// ===========================================================
// Serial debug variables
// ===========================================================
const uint16_t SERIAL_DECIMATION = 10;
volatile uint16_t isrCounter = 0;

volatile float dbg_x = 0.0f, dbg_y = 0.0f, dbg_e = 0.0f, dbg_u = 0.0f;
volatile unsigned long dbg_n = 0;

volatile bool sampleReady  = false;
volatile bool summaryReady = false;

// ===========================================================
// PID (เหมือนการทดลองเดิม)
// ===========================================================
inline void computePID()
{
  e = x - y;

  if (pidMode == PID_SIMPLE) {
    ui += Ki * Ts * e;
    if (ui > 1.0f)       ui = 1.0f;
    else if (ui < -1.0f) ui = -1.0f;

    ud = Kd * (e - e_1) / Ts;
    u  = Kp * e + ui + ud;

  } else {
    ui = ui_1 + Kc / Ti * Ts / 2.0f * (e + e_1);
    if (ui > 1.0f)       ui = 1.0f;
    else if (ui < -1.0f) ui = -1.0f;

    ud   = Kc * Td / Ts * (e - e_1);
    u    = Kc * e + ui + ud;
    ui_1 = ui;
  }

  e_1 = e;
}

// ===========================================================
// ส่วนร่วมหลัง computePID(): clamp, PWM, decimation, จบการทดลอง
// ===========================================================
inline void outputAndLog()
{
  if (ctrlMode == MODE_OPEN_LOOP) u = U_OPEN;   // ทดสอบ open loop: ไม่ใช้ผล PID

  if (u > 1.0f)      u = 1.0f;
  else if (u < 0.0f) u = 0.0f;

  upwm = (int)(u * 255.0f + 0.5f);
  out  = (int)((e + 1.0f) * 127.0f + 0.5f);

  analogWrite(U_PWMP,   upwm);
  analogWrite(Out_PWMP, out);

  isrCounter++;
  if (isrCounter >= SERIAL_DECIMATION) {
    isrCounter = 0;
    if (!runDone && logMode == LOG_STREAM) {
      dbg_x = x; dbg_y = y; dbg_e = e; dbg_u = u;
      dbg_n = isrTickCount;
      sampleReady = true;
    }
  }

  if (!runDone && isrTickCount >= RUN_TICKS) {
    runDone      = true;     // หยุดสะสมสถิติและหยุดส่งข้อมูล (การควบคุมยังทำงานต่อ)
    summaryReady = true;
  }
}

// ===========================================================
// ARCH_PAPER: ADC ISR (Timer1 COMPB สั่ง ADC -> ADC_vect ทำ PID)
// ===========================================================
ISR(ADC_vect)
{
  if (archMode != ARCH_PAPER) return;

  measureTiming(TCNT1);            // ต้องเป็นคำสั่งแรกหลัง guard

  digitalWrite(LED_BUILTIN, HIGH);

  uint16_t adc_raw = ADC;
  recordADC(adc_raw);
  y = (float)adc_raw / 1023.0f;

  uint16_t t0 = TCNT1;
  computePID();
  uint16_t t1 = TCNT1;
  recordTexec((uint16_t)(t1 - t0));

  outputAndLog();

  digitalWrite(LED_BUILTIN, LOW);
}

ISR(TIMER1_COMPB_vect)
{
  if (archMode != ARCH_PAPER) return;
  ADCSRA |= bit(ADSC);
}

// ===========================================================
// ARCH_MY_TIMER: Timer1 COMPA ISR ครอบ ADC
//   อ่านผล ADC ที่สั่งแปลงไว้ท้ายรอบก่อน (หน่วง 1 คาบโดยโครงสร้าง)
// ===========================================================
ISR(TIMER1_COMPA_vect)
{
  if (archMode != ARCH_MY_TIMER) return;

  measureTiming(TCNT1);            // ต้องเป็นคำสั่งแรกหลัง guard

  digitalWrite(LED_BUILTIN, HIGH);

  uint16_t adc_raw = ADC;
  recordADC(adc_raw);
  y = (float)adc_raw / 1023.0f;

  uint16_t t0 = TCNT1;
  computePID();
  uint16_t t1 = TCNT1;
  recordTexec((uint16_t)(t1 - t0));

  outputAndLog();

  ADCSRA |= bit(ADSC);

  digitalWrite(LED_BUILTIN, LOW);
}

// ===========================================================
// SETUP
// ===========================================================
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(U_PWMP,   OUTPUT);
  pinMode(Out_PWMP, OUTPUT);

  // คายประจุวงจร RC: u(t) = 0 V ค้างไว้ก่อนเริ่มการทดลอง -> y(0) = 0
  digitalWrite(U_PWMP, LOW);
  digitalWrite(Out_PWMP, LOW);

  for (uint8_t i = 0; i < JHIST_BINS; i++) jhist[i] = 0;
  for (uint8_t i = 0; i < THIST_BINS; i++) thist[i] = 0;

  Serial.begin(115200);
  Serial.println(F("===== PID Architecture Test v3.4 (TIMING) ====="));
  Serial.print(F("PID mode          = "));
  Serial.println(pidMode == PID_SIMPLE ? F("SIMPLE") : F("TUSTIN"));
  Serial.print(F("Architecture      = "));
  Serial.println(archMode == ARCH_PAPER ? F("PAPER_ADC_ISR") : F("MY_TIMER_ISR"));
  Serial.print(F("Control mode      = "));
  Serial.println(ctrlMode == MODE_CLOSED_LOOP ? F("CLOSED_LOOP") : F("OPEN_LOOP"));
  if (ctrlMode == MODE_OPEN_LOOP) {
    Serial.print(F("U_OPEN            = "));
    Serial.println(U_OPEN, 4);
  }
  Serial.print(F("Log mode          = "));
  Serial.println(logMode == LOG_STREAM ? F("STREAM") : F("SUMMARY_ONLY"));
  Serial.print(F("Setpoint x        = "));
  Serial.println(x, 4);
  Serial.print(F("Run time (s)      = "));
  Serial.println((ctrlMode == MODE_OPEN_LOOP) ? OPEN_RUN_TIME_S : RUN_TIME_S, 1);
  Serial.print(F("SERIAL_DECIMATION = "));
  Serial.println(SERIAL_DECIMATION);
  Serial.print(F("Discharge (ms)    = "));
  Serial.println(DISCHARGE_MS);
  Serial.flush();
  delay(DISCHARGE_MS);               // รอคายประจุก่อนเริ่มควบคุม (Timer1 ยังไม่ทำงาน)
  if (logMode == LOG_STREAM) {
    Serial.println(F("t_sec,x,y,e,u"));
  } else {
    Serial.println(F("# running (no streaming) ..."));
  }
  Serial.flush();

  // ---------- Timer1: CTC @ 976.5625 Hz ----------
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B = bit(CS11) | bit(WGM12);   // CTC, prescaler 8

  OCR1A = 2047;
  OCR1B = 2047;

  if (archMode == ARCH_PAPER) {
    TIMSK1 = bit(OCIE1B);
  } else {
    TIMSK1 = bit(OCIE1A);
  }

  // ---------- ADC (prescaler 16 -> 1 MHz, เหมือนการทดลองเดิม) ----------
  ADMUX = bit(REFS0) | (adcPin & 7);

  if (archMode == ARCH_PAPER) {
    ADCSRA = bit(ADEN) | bit(ADIE) | bit(ADPS2);
  } else {
    ADCSRA = bit(ADEN) | bit(ADPS2);
  }
  ADCSRA |= bit(ADSC);
}

// ===========================================================
// พิมพ์สรุปสถิติทั้งการทดลอง (เรียกครั้งเดียวตอนจบ)
// ===========================================================
void printSummary()
{
  unsigned long n, njit, tn;
  int32_t  jsum;
  uint32_t jsq, lsum, tsum, hj[JHIST_BINS], ht[THIST_BINS];
  uint64_t tsq, asq;
  unsigned long an;
  uint32_t asum;
  uint16_t amin, amax;
  int16_t  jmin, jmax;
  uint16_t lmin, lmax, tmin, tmax;

  noInterrupts();                    // runDone = true แล้ว ค่าไม่เปลี่ยนอีก
  n = tot_n; njit = tot_njit; jsum = tot_jsum; jsq = tot_jsq;
  jmin = tot_jmin; jmax = tot_jmax;
  lmin = tot_lmin; lmax = tot_lmax; lsum = tot_lsum;
  tn = tot_tn; tmin = tot_tmin; tmax = tot_tmax; tsum = tot_tsum; tsq = tot_tsq;
  for (uint8_t i = 0; i < JHIST_BINS; i++) hj[i] = jhist[i];
  for (uint8_t i = 0; i < THIST_BINS; i++) ht[i] = thist[i];
  an = adc_n; asum = adc_sum; asq = adc_sq; amin = adc_min; amax = adc_max;
  interrupts();

  float jmean = (float)jsum / (float)n;
  float jvar  = (float)jsq / (float)n - jmean * jmean;
  float tmean = (float)tsum / (float)tn;
  float tvar  = (float)tsq / (float)tn - tmean * tmean;
  if (jvar < 0) jvar = 0;
  if (tvar < 0) tvar = 0;

  Serial.println(F("# ===== SUMMARY (all ISR cycles) ====="));
  Serial.print(F("# cycles_J,"));            Serial.println(n);
  Serial.print(F("# cycles_Texec,"));        Serial.println(tn);
  Serial.print(F("# J_mean_us,"));           Serial.println(jmean * TICK_TO_US, 3);
  Serial.print(F("# J_std_us,"));            Serial.println(sqrt(jvar) * TICK_TO_US, 3);
  Serial.print(F("# J_rms_us,"));            Serial.println(sqrt((float)jsq / (float)n) * TICK_TO_US, 3);
  Serial.print(F("# J_min_us,"));            Serial.println(jmin * TICK_TO_US, 1);
  Serial.print(F("# J_max_us,"));            Serial.println(jmax * TICK_TO_US, 1);
  Serial.print(F("# J_nonzero_frac,"));      Serial.println((float)njit / (float)n, 5);
  Serial.print(F("# L_mean_us,"));           Serial.println((float)lsum / (float)n * TICK_TO_US, 3);
  Serial.print(F("# L_min_us,"));            Serial.println(lmin * TICK_TO_US, 1);
  Serial.print(F("# L_max_us,"));            Serial.println(lmax * TICK_TO_US, 1);
  Serial.print(F("# Texec_mean_us,"));       Serial.println(tmean * TICK_TO_US, 3);
  Serial.print(F("# Texec_std_us,"));        Serial.println(sqrt(tvar) * TICK_TO_US, 3);
  Serial.print(F("# Texec_min_us,"));        Serial.println(tmin * TICK_TO_US, 1);
  Serial.print(F("# Texec_max_us,"));        Serial.println(tmax * TICK_TO_US, 1);

  // noise ของค่า ADC (y) ช่วงสภาวะคงตัว จากทุกรอบ ISR
  if (an > 0) {
    float am = (float)asum / (float)an;
    // ลบค่าเฉลี่ยเป็นจำนวนเต็มก่อน เพื่อลดความคลาดเคลื่อนของ float
    float am0 = (float)(asum / an);
    float d   = am - am0;
    float var = (float)(asq - 2ULL * (uint64_t)(asum / an) * asum
                         + (uint64_t)(asum / an) * (asum / an) * an) / (float)an - d * d;
    if (var < 0) var = 0;
    Serial.print(F("# y_ss_samples,"));      Serial.println(an);
    Serial.print(F("# y_ss_mean_lsb,"));     Serial.println(am, 3);
    Serial.print(F("# y_ss_std_lsb,"));      Serial.println(sqrt(var), 3);
    Serial.print(F("# y_ss_min_lsb,"));      Serial.println(amin);
    Serial.print(F("# y_ss_max_lsb,"));      Serial.println(amax);
    Serial.print(F("# y_ss_mean,"));         Serial.println(am / 1023.0f, 5);
    Serial.print(F("# y_ss_std,"));          Serial.println(sqrt(var) / 1023.0f, 5);
  }

  // |J| histogram: bin i = |J| ของ i x 0.5 us (bin สุดท้าย >= 8.0 us)
  for (uint8_t i = 0; i < JHIST_BINS; i++) {
    Serial.print(F("# J_hist,"));
    Serial.print(i * TICK_TO_US, 1);
    Serial.print(',');
    Serial.println(hj[i]);
  }
  // T_exec histogram: bin 0 = < 50 us, bin k = [45+5k, 50+5k) us, bin สุดท้าย >= 120 us
  //   ค่าที่พิมพ์คือขอบล่างของ bin (us); bin 0 พิมพ์เป็น 0
  for (uint8_t i = 0; i < THIST_BINS; i++) {
    Serial.print(F("# Texec_hist,"));
    if (i == 0) Serial.print(0.0f, 1);
    else        Serial.print((float)(THIST_LO + (i - 1) * THIST_STEP) * TICK_TO_US, 1);
    Serial.print(',');
    Serial.println(ht[i]);
  }
  Serial.println(F("# END"));
}

// ===========================================================
// LOOP
// ===========================================================
void loop()
{
  if (sampleReady) {
    float x_loc, y_loc, e_loc, u_loc;
    unsigned long n_loc;

    noInterrupts();
    x_loc = dbg_x; y_loc = dbg_y; e_loc = dbg_e; u_loc = dbg_u;
    n_loc = dbg_n;
    sampleReady = false;
    interrupts();

    float t = (float)n_loc * Ts_hw;    // เวลาจริงตามคาบฮาร์ดแวร์ 1024 us

    Serial.print(t, 6);     Serial.print(',');
    Serial.print(x_loc, 4); Serial.print(',');
    Serial.print(y_loc, 4); Serial.print(',');
    Serial.print(e_loc, 4); Serial.print(',');
    Serial.println(u_loc, 4);
  }

  if (summaryReady) {
    summaryReady = false;
    printSummary();
  }
}
