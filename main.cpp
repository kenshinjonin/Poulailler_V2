#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <Timezone.h>
#include <Dusk2Dawn.h>

// Paramètres de connexion WiFi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// Objets pour se connecter au réseau WiFi
WiFiClient wifiClient;
WiFiUDP ntpUDP;

// Objet NTPClient pour récupérer l'heure via NTP
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");

RTC_DS3231 rtc; // Créer un objet RTC_DS3231 pour interagir avec le module RTC

// Définir le fuseau horaire (ajuster en fonction de votre emplacement géographique)
TimeChangeRule myDST = {"CEST", Last, Sun, Mar, 2, 120};    // Heure d'été (Central European Summer Time)
TimeChangeRule mySTD = {"CET", Last, Sun, Oct, 3, 60};       // Heure d'hiver (Central European Time)
Timezone myTZ(myDST, mySTD);  // Créer un objet Timezone avec les règles de changement d'heure

// Paramètres de l'emplacement géographique (à ajuster en fonction de votre emplacement)
const double latitude = 46.308352907180954;    // Latitude en degrés décimaux
const double longitude = 4.887889949495334;  // Longitude en degrés décimaux
int UTC_OFFSET = 1;

// Variables pour stocker l'heure du lever et du coucher du soleil
int leverSoleil = 0;  // Heure du lever du soleil (en minutes depuis minuit)
int coucherSoleil = 0;  // Heure du coucher du soleil (en minutes depuis minuit)

// Définition des broches utilisées
const int buttonPin = 2;       // Broche du bouton poussoir
const int motorEnablePin = 4;  // Broche d'activation du moteur (ENA sur L298N)
const int motorInput1 = 5;     // Broche d'entrée 1 du moteur (IN1 sur L298N)
const int motorInput2 = 6;     // Broche d'entrée 2 du moteur (IN2 sur L298N)
const int upperLimitSwitchPin = 7;  // Broche du capteur fin de course haut
const int lowerLimitSwitchPin = 8;  // Broche du capteur fin de course bas

// Variables d'état de la porte
bool isMoving = false;
bool previousDirectionUp = false;

// Constante pour définir le délai minimal entre deux appuis successifs sur le bouton (en millisecondes)
const unsigned long debounceDelay = 500;

// Variable pour suivre le moment du dernier appui sur le bouton
unsigned long lastButtonPressTime = 0;

// Durée de temporisation d'une seconde (en millisecondes)
const unsigned long stopDuration = 1000;

// Durée maximale de mouvement de la porte (en millisecondes)
const unsigned long maxMovementDuration = 22000; // 22 secondes

// Fonction pour calculer l'heure du lever du soleil et du coucher du soleil
void calculateSunriseSunset() {
  
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int dayOfWeek = now.dayOfTheWeek();

  // Vérifier si nous sommes dans la période de l'heure d'été (DST)
  if (month > 2 && month < 11) {
    if (month == 3 && dayOfWeek == 1 && day > 24) {
      // Dernier dimanche de mars, heure d'été commence
      UTC_OFFSET = 2;
    } else if (month == 10 && dayOfWeek == 1 && day > 24) {
      // Dernier dimanche d'octobre, heure d'été se termine
      UTC_OFFSET = 1;
    } else {
      // Pas de changement d'heure, utiliser UTC_OFFSET actuel
    }
  } else {
    UTC_OFFSET = 1;  // En dehors de la période d'heure d'été, utiliser UTC_OFFSET sur +1
  }

  // Définir la position et l'offset UTC pour Dusk2Dawn
  Dusk2Dawn myLocation(latitude, longitude, UTC_OFFSET * 60);
  
  // Obtenez l'heure du lever du soleil
  int sunriseTime = myLocation.sunrise(year, month, day, false);

  // Obtenez l'heure du coucher du soleil
  int sunsetTime = myLocation.sunset(year, month, day, false);

  // Convertir les heures locales en "HHhMM"
  time_t sunriseTime_t = sunriseTime * 60; // Convertir en secondes
  time_t sunsetTime_t = sunsetTime * 60;   // Convertir en secondes

  struct tm sunriseTm = *localtime(&sunriseTime_t);
  struct tm sunsetTm = *localtime(&sunsetTime_t);

  char leverFormate[6];
  char coucherFormate[6];

  strftime(leverFormate, sizeof(leverFormate), "%Hh%M", &sunriseTm);
  strftime(coucherFormate, sizeof(coucherFormate), "%Hh%M", &sunsetTm);

  // Afficher dans la console série
  Serial.print("Heure du lever du soleil : ");
  Serial.println(leverFormate);
  Serial.print("Heure du coucher du soleil : ");
  Serial.println(coucherFormate);
}

// Fonction pour se connecter au réseau WiFi
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connecté");
}

// Fonction pour mettre à jour l'heure du module DS3231 via NTP et gérer le passage automatique à l'heure d'été/heure d'hiver
void updateDS3231Time() {
  timeClient.update(); // Récupérer l'heure NTP
  time_t utcTime = timeClient.getEpochTime(); // Obtenir l'heure UTC en secondes depuis le 1er janvier 1970
  time_t localTime = myTZ.toLocal(utcTime); // Convertir l'heure UTC en heure locale
  rtc.adjust(DateTime(localTime)); // Mettre à jour l'heure du module RTC avec l'heure locale
}

// Tâche pour mettre à jour l'heure toutes les 24 heures
void updateTimeTask(void *parameter) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(86400000)); // Attendre 24 heures (en millisecondes)
    updateDS3231Time(); // Mettre à jour l'heure du module DS3231
    Serial.println("Mise à jour de l'heure");
  }
}

// Fonction de tâche pour recalculer l'heure du lever du soleil et du coucher du soleil tous les jours à 00h01
void calculateSunriseSunsetTask(void *parameter) {
  for (;;) {
    // Attendre jusqu'à 00h01 pour recalculer l'heure du lever du soleil et du coucher du soleil
    DateTime now = rtc.now();
    DateTime nextExecution = DateTime(now.year(), now.month(), now.day(), 0, 1, 0);
    nextExecution = nextExecution + TimeSpan(1, 0, 0, 0); // Ajouter un jour pour la prochaine exécution

    // Calculer le nombre de secondes jusqu'à la prochaine exécution
    time_t nowSecs = now.unixtime();
    time_t nextExecutionSecs = nextExecution.unixtime();
    long delaySeconds = nextExecutionSecs - nowSecs;

    if (delaySeconds < 0) {
      delaySeconds += 24 * 60 * 60; // Ajouter 24 heures si la prochaine exécution est passée
    }

    vTaskDelay(pdMS_TO_TICKS(delaySeconds * 1000));

    calculateSunriseSunset(); // Recalculer l'heure du lever du soleil et du coucher du soleil
  }
}

// Fonction pour arrêter le mouvement de la porte
void stopPorte() {
  isMoving = false;
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorEnablePin, LOW); // Désactiver le moteur
  Serial.println("Porte arrêtée");
}

// Fonction pour ouvrir la porte
void ouvrirPorte() {
  isMoving = true;
  Serial.println("Ouverture de la porte");
  previousDirectionUp = true; // Mettre à jour la direction précédente
  digitalWrite(motorEnablePin, HIGH); // Activer le moteur
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);

  unsigned long startTime = millis(); // Temps de départ pour suivre la durée de mouvement
  unsigned long elapsedTime = 0; // Temps écoulé depuis le début du mouvement

  while (digitalRead(upperLimitSwitchPin) == HIGH && elapsedTime < maxMovementDuration) {
    // Vérifier si suffisamment de temps s'est écoulé pour laisser d'autres tâches s'exécuter (éviter un blocage)
    delay(1); // Attente d'une milliseconde
    elapsedTime = millis() - startTime; // Mettre à jour le temps écoulé
  }

  // Attendre 1 seconde
  delay(1000);
  stopPorte(); // Arrêter la porte
  Serial.println("Porte ouverte");
}

// Fonction pour fermer la porte
void fermerPorte() {
  isMoving = true;
  Serial.println("Fermeture de la porte");
  previousDirectionUp = false; // Mettre à jour la direction précédente
  digitalWrite(motorEnablePin, HIGH); // Activer le moteur
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);

  unsigned long startTime = millis(); // Temps de départ pour suivre la durée de mouvement
  unsigned long elapsedTime = 0; // Temps écoulé depuis le début du mouvement

  while (digitalRead(lowerLimitSwitchPin) == HIGH && elapsedTime < maxMovementDuration) {
    // Vérifier si suffisamment de temps s'est écoulé pour laisser d'autres tâches s'exécuter (éviter un blocage)
    delay(1); // Attente d'une milliseconde
    elapsedTime = millis() - startTime; // Mettre à jour le temps écoulé
  }

  // Attendre 1 seconde
  delay(1000);
  stopPorte(); // Arrêter la porte
  Serial.println("Porte fermée");
}

// Fonction pour gérer l'interruption du bouton poussoir
void handleButtonPress() {
  // Lire l'état du bouton poussoir
  int buttonState = digitalRead(buttonPin);

  // Vérifier si le bouton est enfoncé
  if (buttonState == LOW) {
    // Vérifier si suffisamment de temps s'est écoulé depuis le dernier appui sur le bouton
    if (millis() - lastButtonPressTime >= debounceDelay) {
      // Mettre à jour le moment du dernier appui sur le bouton
      lastButtonPressTime = millis();
      Serial.println("Appui sur le bouton détecté");

      // Inverser l'état de la porte si elle est fixe (pas en mouvement)
      if (!isMoving) {
        // Vérifier l'état des broches du moteur pour déterminer la direction de mouvement
        bool motorInput1State = digitalRead(motorInput1);
        bool motorInput2State = digitalRead(motorInput2);

        // Arrêter la porte si elle est en mouvement dans une direction
        if (motorInput1State || motorInput2State) {
          stopPorte();
        } else {
          // Aucun mouvement en cours, décider de la direction en fonction des capteurs de fin de course
          if (digitalRead(upperLimitSwitchPin) == LOW && digitalRead(lowerLimitSwitchPin) == LOW) {
            // Les deux capteurs de fin de course sont inactifs, cela signifie que la porte est à mi-chemin,
            // on choisit alors la direction précédente du mouvement
            if (previousDirectionUp) {
              fermerPorte(); // On ferme la porte car la direction précédente était vers le haut
            } else {
              ouvrirPorte(); // On ouvre la porte car la direction précédente était vers le bas
            }
          } else if (digitalRead(upperLimitSwitchPin) == LOW) {
            ouvrirPorte(); // Le capteur du haut est actif, la porte est fermée, donc on l'ouvre
          } else if (digitalRead(lowerLimitSwitchPin) == LOW) {
            fermerPorte(); // Le capteur du bas est actif, la porte est ouverte, donc on la ferme
          }
        }
      }
    }
  }
}

// Fonction d'initialisation
void setup() {
  Serial.begin(115200);
  connectWiFi();  // Se connecter au réseau WiFi

  Wire.begin();         // Démarrer la communication I2C
  rtc.begin();          // Démarrer la communication avec le module RTC

  // Mettre à jour l'heure du module DS3231 au démarrage avec l'heure NTP
  updateDS3231Time();

  // Calculer l'heure du lever du soleil et du coucher du soleil au démarrage
  calculateSunriseSunset();

  // Démarrer une tâche pour la mise à jour de l'heure toutes les 24 heures
  xTaskCreatePinnedToCore(updateTimeTask, "updateTimeTask", 4096, NULL, 1, NULL, 0);
  // Démarrer une tâche pour recalculer l'heure du lever du soleil et du coucher du soleil toutes les 24 heures à 00h01
  xTaskCreatePinnedToCore(calculateSunriseSunsetTask, "calculateSunriseSunsetTask", 4096, NULL, 1, NULL, 0);

  // Configuration des broches en entrée/sortie
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(upperLimitSwitchPin, INPUT_PULLUP);
  pinMode(lowerLimitSwitchPin, INPUT_PULLUP);

  // Désactiver le moteur au démarrage
  digitalWrite(motorEnablePin, LOW);

  // Attachement des interruptions pour le bouton poussoir
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, CHANGE);
}

// Fonction de boucle principale
void loop() {
  // Le code principal de votre programme va ici...
}
