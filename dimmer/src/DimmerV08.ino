#define DEBUG 1
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// WLAN-Konfiguration
const char* ssid = "8";
const char* password = "RusCoj!2025wlan";

// Webserver
AsyncWebServer server(8080);

// Pins
const int TriacTriggerPin = 4;            // Gemeinsamer Pin für TRIAC und Optokoppler
const int SyncPin = 6;      // Pin, ansd dem das Taktsignal vom Arduino anliegt

// Globale Variablen
volatile int dim_value = 50;         // Startwert für den Dimmwert (50% bedeutet halbe Leistung)
volatile unsigned long phase_cut_time = 0;  // Phasenanschnitt-Zeit
volatile int isr_phase = 0;          // Phase der ISR-Steuerung
volatile uint64_t tim = 0; 
// Timer und Semaphore
hw_timer_t *timer = NULL;            // Timer für Phasensteuerung
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Mutex für Thread-Sicherheit

// Arduino-Takt-Interrupt
void   ISR_SyncPin() {
    isr_phase = 0;         // ISR-Phase zurücksetzen
    phase_cut_time = (10000UL * (100 - dim_value)) / 100; // Berechnung des Phasenanschnitts (µs)

    // Start des Timers für die erste Phase
    if (phase_cut_time<10) {
      digitalWrite(TriacTriggerPin, HIGH);
      timerAlarm(timer, 300, true, 0);
      isr_phase=2; 
    }
    else 
      timerAlarm(timer, phase_cut_time, true, 0);
    
   //timerAlarm(timer, 10, true, 0);// T
   tim=0; 
   timerStart(timer);
   
}

// ISR für Timer


//  void IRAM_ATTR ISR_Timer() {
    // portENTER_CRITICAL_ISR(&timerMux);
  
  tim = timerReadMicros(timer) - tim;
 
  if (isr_phase == 0) {
    // Erste Phase: Pin HIGH (Start Impuls)
    
    timerAlarm(timer, 300, true, 0);
    digitalWrite(TriacTriggerPin, HIGH
    );
    // 300 µs für den Impuls
    isr_phase = 1; // Wechsel zur nächsten Phase
  } 
  else if (isr_phase == 1) {
    // Zweite Phase: Pin LOW (Ende Impuls)
    digitalWrite(TriacTriggerPin, LOW);
    timerAlarm(timer, 10000 - 300, true, 0); // Wartezeit: 10 ms - Phasenanschnitt
    isr_phase = 2; // Wechsel zur nächsten Phase
  } 
  else if (isr_phase == 2) {
    // Dritte Phase: Pin HIGH (zweiter Impuls)

    digitalWrite(TriacTriggerPin, HIGH);
    timerAlarm(timer, 300, false, 0); // 300 µs für den zweiten Impuls
    isr_phase = 3; // Wechsel zur nächsten Phase
    } 
  else if (isr_phase == 3) {
    // Letzte Phase: Pin LOW (Ende Sequenz)
    digitalWrite(TriacTriggerPin, LOW);
    timerStop(timer); // Timer stoppen
    isr_phase = 4; 
     
  }
  else if (isr_phase == 4){
    log_printf("Warning: there is no phase 4");
  }


//  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
   
  Serial.begin(115200);
  
  // WLAN-Verbindung
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.print(".");
  // }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Webserver-Konfiguration
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body>";
    html += "<h1>ESP32 TRIAC Control</h1>";
    html += "<p>Dimmwert: " + String(dim_value) + " %</p>";
    html += "<form action=\"/setdim\" method=\"GET\">";
    html += "<label for=\"dimm\">Dimmwert (0-100 %): </label>";
    html += "<input type=\"number\" id=\"dimm\" name=\"dimm\" min=\"0\" max=\"10000\" value=\"" + String(dim_value) + "\">";
    html += "<input type=\"submit\" value=\"Set Dimmwert\">";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/setdim", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("dimm")) {
      String dimmParam = request->getParam("dimm")->value();
      int new_dim_value = dimmParam.toInt();
      if (new_dim_value >= 0 && new_dim_value <= 10000) {
        dim_value = new_dim_value;
        Serial.print("Dimmwert auf ");
        Serial.print(dim_value);
        Serial.println(" % gesetzt.");
      } else {
        Serial.println("Ungültiger Dimmwert! Bitte Wert zwischen 0 und 100 eingeben.");
      }
    }
    request->redirect("/");
  }) ;

  server.begin();
  Serial.println("Server gestartet");

  // Pins konfigurieren
  pinMode(TriacTriggerPin, OUTPUT);
  pinMode(SyncPin, INPUT_PULLDOWN);
  digitalWrite(TriacTriggerPin, LOW);

  // synch 50Hz 
  attachInterrupt(digitalPinToInterrupt(SyncPin), ISR_SyncPin, RISING);
  
  // Timer konfigurieren
  timer = timerBegin(1000000); // Timer allokieren, Prescaler 80 (1 µs Takt)
  timerAttachInterrupt(timer, &ISR_Timer);
}

void loop() {
  // Hauptschleife bleibt leer, die Steuerung erfolgt vollständig durch Interrupts
  #ifdef DEBUG
  delayMicroseconds(30000); 
  pinMode(SyncPin, INPUT_PULLUP); 
  pinMode(SyncPin, INPUT_PULLDOWN); 
  #endif
}

