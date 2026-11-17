/*
 * ============================================================================
 *  code_esp32_gateway.ino
 *  Passerelle / emulateur de telemetrie pour le simulateur MATLAB avion_fpga_pro
 * ============================================================================
 *
 *  Carte      : ESP32 Dev Module (ESP32 "normale")
 *  Liaison PC : USB (Serial, UART0)            -> MATLAB, 115200 bauds
 *  Liaison FPGA : UART2 (RX=GPIO16, TX=GPIO17) <- DE0-Nano, 115200 bauds, 3.3 V
 *
 *  Format de sortie STRICT (ASCII) :   "ax,ay,az\n"      valeurs en g
 *
 *  MODES
 *    TEST_MODE = true  : EMULATEUR. Genere une trajectoire de vol (virages
 *                        inclines en huit + tangage + turbulences), calcule
 *                        l'accelerometre theorique (ax,ay,az) en g, ajoute du
 *                        bruit blanc gaussien, des vibrations et de rares
 *                        chocs (pour exercer le filtre adaptatif de MATLAB).
 *    TEST_MODE = false : PASSERELLE TRANSPARENTE. Tout octet recu sur UART2
 *                        (DE0-Nano) est retransmis tel quel sur l'USB.
 *
 *  CABLAGE (mode passerelle)
 *      DE0-Nano GPIO (TX)  ---->  ESP32 GPIO16 (RX2)
 *      DE0-Nano GND        ----   ESP32 GND            (masse commune obligatoire)
 *      Niveaux logiques 3.3 V des deux cotes : liaison directe possible.
 *      Ne JAMAIS relier une sortie 5 V a une broche de l'ESP32.
 *
 *  Conventions (identiques a MATLAB) :
 *      roll  = atan2(ay, az)
 *      pitch = atan2(-ax, sqrt(ay^2 + az^2))
 *      Roll+ = aile droite basse, Pitch+ = nez en haut.
 *
 *  Code non bloquant : aucune fonction delay(), cadencement par millis().
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
//  CONFIGURATION
// ---------------------------------------------------------------------------
#define TEST_MODE          false    // true = emulateur, false = passerelle DE0-Nano (version finale)
#define TELEMETRY_HZ       50       // 50 ou 100 (cadence d'emission en mode emulateur)
#define OUTPUT_RAW_LSB     false    // true : envoie des LSB bruts (256 LSB/g) au lieu de g

#define USB_BAUD           115200   // vers MATLAB (doit correspondre a MATLAB)
#define FPGA_BAUD          115200   // depuis la DE0-Nano (doit correspondre au module UART FPGA)
#define FPGA_RX_PIN        16       // RX2 <- TX de la DE0-Nano
#define FPGA_TX_PIN        17       // TX2 -> (optionnel) RX de la DE0-Nano
#define FPGA_RX_BUF_SIZE   2048

#define LED_PIN            2        // LED embarquee (activite)

// Parametres de l'emulateur
#define NOISE_SIGMA_G      0.015f   // bruit blanc gaussien par axe [g]
#define VIBRATION_G        0.010f   // vibration moteur [g]
#define VIBRATION_HZ       23.0f
#define SHOCK_PROB         0.004f   // probabilite d'un choc par echantillon
#define SHOCK_G            0.30f    // amplitude des chocs [g]
#define ENABLE_TURBULENCE  true

// ---------------------------------------------------------------------------
//  VARIABLES
// ---------------------------------------------------------------------------
static const uint32_t PERIOD_MS = 1000UL / TELEMETRY_HZ;
static uint32_t nextTick = 0;

// Turbulence : processus d'Ornstein-Uhlenbeck sur roll et pitch [deg]
static float turbRoll  = 0.0f;
static float turbPitch = 0.0f;

// Activite LED (non bloquant)
static uint32_t ledOffAt = 0;

// ---------------------------------------------------------------------------
//  OUTILS
// ---------------------------------------------------------------------------

// Nombre aleatoire uniforme dans ]0,1[ a partir du generateur materiel de l'ESP32.
static inline float uniform01() {
  return ((float)esp_random() + 1.0f) / 4294967297.0f;
}

// Bruit blanc gaussien N(0,1) par la transformation de Box-Muller.
static float gaussian() {
  static bool  haveSpare = false;
  static float spare = 0.0f;
  if (haveSpare) {
    haveSpare = false;
    return spare;
  }
  const float u1 = uniform01();
  const float u2 = uniform01();
  const float r  = sqrtf(-2.0f * logf(u1));
  const float th = 2.0f * (float)M_PI * u2;
  spare = r * sinf(th);
  haveSpare = true;
  return r * cosf(th);
}

static inline void pulseLed(uint32_t now) {
  digitalWrite(LED_PIN, HIGH);
  ledOffAt = now + 15;
}

// Emission d'une ligne "ax,ay,az\n" sans printf bloquant ni String dynamique.
static void sendSample(float ax, float ay, float az) {
  char buf[64];
  int n;
#if OUTPUT_RAW_LSB
  n = snprintf(buf, sizeof(buf), "%d,%d,%d\n",
               (int)lroundf(ax * 256.0f), (int)lroundf(ay * 256.0f), (int)lroundf(az * 256.0f));
#else
  n = snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f\n", ax, ay, az);
#endif
  if (n > 0) {
    Serial.write(reinterpret_cast<const uint8_t*>(buf), (size_t)n);
  }
}

// ---------------------------------------------------------------------------
//  EMULATEUR DE VOL
// ---------------------------------------------------------------------------
static void emulateFlight(uint32_t nowMs) {
  const float t  = nowMs * 0.001f;
  const float dt = 1.0f / (float)TELEMETRY_HZ;
  const float d2r = (float)M_PI / 180.0f;

  // Trajectoire en huit : virages alternes a gauche / droite, avec tangage couple.
  float rollDeg  = 38.0f * sinf(2.0f * (float)M_PI * t / 14.0f)
                 +  3.0f * sinf(2.0f * (float)M_PI * 1.3f * t);
  float pitchDeg =  7.0f * sinf(2.0f * (float)M_PI * t / 7.0f + 0.8f)
                 +  2.5f * sinf(2.0f * (float)M_PI * 0.9f * t);

#if ENABLE_TURBULENCE
  // Ornstein-Uhlenbeck : dx = -x/tau*dt + sigma*sqrt(2dt/tau)*N(0,1)
  const float tau = 0.6f;
  const float k   = sqrtf(2.0f * dt / tau);
  turbRoll  += -turbRoll  / tau * dt + 2.2f * k * gaussian();
  turbPitch += -turbPitch / tau * dt + 1.2f * k * gaussian();
  rollDeg  += turbRoll;
  pitchDeg += turbPitch;
#endif

  const float r = rollDeg  * d2r;
  const float p = pitchDeg * d2r;

  // Gravite exprimee dans le repere corps (modele statique, comme en MATLAB).
  float ax = -sinf(p);
  float ay =  sinf(r) * cosf(p);
  float az =  cosf(r) * cosf(p);

  // Vibrations moteur + bruit blanc gaussien.
  const float vib = VIBRATION_G * sinf(2.0f * (float)M_PI * VIBRATION_HZ * t);
  ax += vib + NOISE_SIGMA_G * gaussian();
  ay += 0.7f * vib + NOISE_SIGMA_G * gaussian();
  az += 0.5f * vib + NOISE_SIGMA_G * gaussian();

  // Chocs rares : |a| s'ecarte de 1 g, le filtre adaptatif doit s'en mefier.
  if (uniform01() < SHOCK_PROB) {
    ax += SHOCK_G * gaussian();
    ay += SHOCK_G * gaussian();
    az += SHOCK_G * gaussian();
  }

  sendSample(ax, ay, az);
}

// ---------------------------------------------------------------------------
//  PASSERELLE TRANSPARENTE UART2 -> USB
// ---------------------------------------------------------------------------
static void bridgeTransparent(uint32_t nowMs) {
  int avail = Serial2.available();
  if (avail > 0) {
    uint8_t buf[256];
    if (avail > (int)sizeof(buf)) avail = sizeof(buf);
    const size_t n = Serial2.readBytes(buf, (size_t)avail);   // ne bloque pas : octets deja presents
    if (n > 0) {
      Serial.write(buf, n);                                    // aucune modification
      pulseLed(nowMs);
    }
  }
}

// ---------------------------------------------------------------------------
//  SETUP / LOOP
// ---------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(USB_BAUD);

  Serial2.setRxBufferSize(FPGA_RX_BUF_SIZE);
  Serial2.begin(FPGA_BAUD, SERIAL_8N1, FPGA_RX_PIN, FPGA_TX_PIN);
  Serial2.setTimeout(1);

  nextTick = millis() + PERIOD_MS;
}

void loop() {
  const uint32_t now = millis();

#if TEST_MODE
  // Cadencement strict sans derive : nextTick avance d'une periode fixe.
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick += PERIOD_MS;
    if ((int32_t)(now - nextTick) > (int32_t)(5 * PERIOD_MS)) {
      nextTick = now + PERIOD_MS;          // rattrapage si le systeme a pris du retard
    }
    emulateFlight(now);
    pulseLed(now);
  }
#else
  bridgeTransparent(now);
#endif

  // Extinction non bloquante de la LED d'activite.
  if (ledOffAt != 0 && (int32_t)(now - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }
}
