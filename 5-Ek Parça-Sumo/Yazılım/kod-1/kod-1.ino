// =============================
// MİNİ SUMO ROBOT - ANA KOD
// =============================

// --- PIN TANIMLARI ---
// HC-SR04
#define TRIG_PIN   3
#define ECHO_PIN   2

// Motor B (Sol)
#define MOT_B_A    5
#define MOT_B_B    6

// Motor A (Sağ)
#define MOT_A_A    10
#define MOT_A_B    11

// Start Modülü
#define START_PIN  7

// --- SABITLER ---
#define ESIK_MESAFE  40    // cm - gerekirse değiştir
#define TAM_HIZ      255
#define DONME_HIZI   180

// --- FONKSİYONLAR ---

long mesafeOku() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long sure = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  long cm = sure / 58;
  return cm;
}

void dur() {
  analogWrite(MOT_B_A, 0);
  analogWrite(MOT_B_B, 0);
  analogWrite(MOT_A_A, 0);
  analogWrite(MOT_A_B, 0);
}

void sagaDon() {
  // Sol → İleri, Sağ → Geri (yerinde dönüş)
  analogWrite(MOT_B_A, DONME_HIZI);
  analogWrite(MOT_B_B, 0);
  analogWrite(MOT_A_A, 0);
  analogWrite(MOT_A_B, DONME_HIZI);
}

void saldir() {
  // İkisi de tam ileri
  analogWrite(MOT_B_A, TAM_HIZ);
  analogWrite(MOT_B_B, 0);
  analogWrite(MOT_A_A, TAM_HIZ);
  analogWrite(MOT_A_B, 0);
}

// --- SETUP ---
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MOT_B_A, OUTPUT);
  pinMode(MOT_B_B, OUTPUT);
  pinMode(MOT_A_A, OUTPUT);
  pinMode(MOT_A_B, OUTPUT);

  pinMode(START_PIN, INPUT);

  dur(); // Başlangıçta motorlar kapalı
}

// --- ANA DÖNGÜ ---
void loop() {

  // START SİNYALİ BEKLE
  // Start modülü ON = 0V, OFF = 5V
  // Sinyal gelmeden (pin HIGH) motorlar çalışmaz
  if (digitalRead(START_PIN) == HIGH) {
    dur();
    return;
  }

  // SENSÖR OKU
  long mesafe = mesafeOku();

  // ALGORİTMA
  if (mesafe > 0 && mesafe < ESIK_MESAFE) {
    saldir();       // Rakip VAR → tam gaz üstüne git
  } else {
    sagaDon();      // Rakip YOK → yerinde sağa dön, ara
  }
}