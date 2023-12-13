#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <Timezone.h>
#include <Dusk2Dawn.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// Paramètres de connexion WiFi
const char *ssid = "SSID";
const char *password = "PASSWORD";

// Objets pour se connecter au réseau WiFi
WiFiClient wifiClient;
WiFiUDP ntpUDP;

// Objet NTPClient pour récupérer l'heure via NTP
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");

// Créer un objet RTC_DS3231 pour interagir avec le module RTC
RTC_DS3231 rtc;

// Définir le fuseau horaire (ajuster en fonction de votre emplacement géographique)
TimeChangeRule myDST = {"CEST", Last, Sun, Mar, 2, 120}; // Heure d'été (Central European Summer Time)
TimeChangeRule mySTD = {"CET", Last, Sun, Oct, 3, 60};   // Heure d'hiver (Central European Time)
Timezone myTZ(myDST, mySTD);                             // Créer un objet Timezone avec les règles de changement d'heure

// Paramètres de l'emplacement géographique (à ajuster en fonction de votre emplacement)
const double latitude = 46.308352907180954; // Latitude en degrés décimaux
const double longitude = 4.887889949495334; // Longitude en degrés décimaux
int UTC_OFFSET = 1;

// Variables pour stocker l'heure du lever et du coucher du soleil
int sunriseTime = 0; // Heure du lever du soleil (en minutes depuis minuit)
int sunsetTime = 0;  // Heure du coucher du soleil (en minutes depuis minuit)

// Définition des broches utilisées
const uint8_t buttonPinNumber = 2;           // Broche du bouton poussoir
const uint8_t motorEnablePinNumber = 4;      // Broche d'activation du moteur (ENA sur L298N)
const uint8_t motorInput1PinNumber = 5;      // Broche d'entrée 1 du moteur (IN1 sur L298N)
const uint8_t motorInput2PinNumber = 6;      // Broche d'entrée 2 du moteur (IN2 sur L298N)
const uint8_t upperLimitSwitchPinNumber = 7; // Broche du capteur fin de course haut
const uint8_t lowerLimitSwitchPinNumber = 8; // Broche du capteur fin de course bas

// Variables d'état de la porte
bool isDoorMoving = false;
bool previousDoorDirectionUp = false;

// Constante pour définir le délai minimal entre deux appuis successifs sur le bouton (en millisecondes)
const unsigned long buttonDebounceDelay = 500;

// Variable pour suivre le moment du dernier appui sur le bouton
unsigned long lastButtonPressTimestamp = 0;

// Durée maximale de mouvement de la porte (en millisecondes)
const unsigned long maxDoorMovementDuration = 22000; // 22 secondes

// Définir les constantes pour les dates de changement d'heure d'été
const int DST_START_MONTH = 3; // Mars
const int DST_END_MONTH = 10;  // Octobre
const int DST_CHANGE_DAY = 25; // Après le 24ème jour

// Fonction pour formater le temps en "HHhMM" sans utiliser strftime
void formatTime(int time, char *buffer)
{
  int hours = time / 60;
  int minutes = time % 60;
  sprintf(buffer, "%02dh%02d", hours, minutes);
}

// Calculer l'heure du lever du soleil et du coucher du soleil
void calculateSunriseSunset()
{
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int dayOfWeek = now.dayOfTheWeek();

  // Vérifier si nous sommes dans la période de l'heure d'été (DST)
  if ((month > DST_START_MONTH && month < DST_END_MONTH) ||
      (month == DST_START_MONTH && (dayOfWeek == 1 && day >= DST_CHANGE_DAY)) ||
      (month == DST_END_MONTH && (dayOfWeek == 1 && day < DST_CHANGE_DAY)))
  {
    UTC_OFFSET = 2; // Heure d'été
  }
  else
  {
    UTC_OFFSET = 1; // Heure standard
  }

  // Créer l'objet Dusk2Dawn une seule fois si possible
  static Dusk2Dawn myLocation(latitude, longitude, UTC_OFFSET * 60);

  // Calculer le lever et le coucher du soleil
  int sunriseTime = myLocation.sunrise(year, month, day, false);
  int sunsetTime = myLocation.sunset(year, month, day, false);

  // Convertir les heures locales en "HHhMM"
  char sunriseTimeFormatted[6];
  char sunsetTimeFormatted[6];
  formatTime(sunriseTime, sunriseTimeFormatted);
  formatTime(sunsetTime, sunsetTimeFormatted);

  // Afficher dans la console série
  Serial.print("Heure du lever du soleil : ");
  Serial.println(sunriseTimeFormatted);
  Serial.print("Heure du coucher du soleil : ");
  Serial.println(sunsetTimeFormatted);
}

// Connexion au réseau WiFi
bool connectWiFi()
{
  unsigned long startTime = millis();
  unsigned long timeout = 30000; // Timeout de 30 secondes
  bool connected = false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Tentative de connexion WiFi...");

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout)
  {
    delay(1000);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    connected = true;
    Serial.println("");
    Serial.println("WiFi connecté avec succès à : ");
    Serial.println(ssid);
    Serial.print("Adresse IP : ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  else
  {
    Serial.println("");
    Serial.println("Échec de la connexion WiFi. Vérifiez vos identifiants ou votre réseau.");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }

  return connected;
}

// Fonction pour obtenir et afficher le RSSI en dBm
void printRSSI()
{
  long rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Mettre à jour l'heure du module DS3231 via NTP et gérer le passage automatique à l'heure d'été/heure d'hiver
bool updateDS3231Time()
{
  if (!timeClient.update())
  {               // Récupérer l'heure NTP et vérifier si la mise à jour a réussi
    return false; // La mise à jour NTP a échoué, donc on ne met pas à jour l'heure du DS3231
  }

  time_t utcTime = timeClient.getEpochTime(); // Obtenir l'heure UTC en secondes depuis le 1er janvier 1970
  if (utcTime == 0)
  {
    return false; // La récupération de l'heure UTC a échoué
  }

  time_t localTime = myTZ.toLocal(utcTime); // Convertir l'heure UTC en heure locale
  if (localTime == 0)
  {
    return false; // La conversion de l'heure locale a échoué
  }

  rtc.adjust(DateTime(localTime)); // Mettre à jour l'heure du module RTC avec l'heure locale
  return true;                     // L'heure a été mise à jour avec succès
}

// Tâche pour vérifier la connexion WIFI toutes les 15 minutes
TaskHandle_t WiFiReconnectTaskHandle = NULL; // Variable pour stocker le handle de la tâche WiFiReconnectTask
void WiFiReconnectTask(void *pvParameters)
{
  for (;;)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Reconnexion WiFi...");
      connectWiFi();
    }
    vTaskDelay(pdMS_TO_TICKS(15 * 60 * 1000)); // Attendre 15 minutes
  }
}

// Tâche pour afficher le RSSI toutes les 15 minutes
TaskHandle_t printRSSITaskHandle = NULL; // Variable pour stocker le handle de la tâche printRSSITask
void printRSSITask(void *parameter)
{
  for (;;)
  {
    printRSSI();                               // Appeler la fonction pour afficher le RSSI
    vTaskDelay(pdMS_TO_TICKS(15 * 60 * 1000)); // Attendre 15 minutes
  }
}

// Tâche pour mettre à jour l'heure toutes les 24 heures
TaskHandle_t updateTimeTaskHandle = NULL; // Variable pour stocker le handle de la tâche updateTimeTask
void updateTimeTask(void *parameter)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(86400000)); // Attendre 24 heures (en millisecondes)
    updateDS3231Time();                  // Mettre à jour l'heure du module DS3231
    Serial.println("Mise à jour de l'heure");
  }
}

// Fonction de tâche pour recalculer l'heure du lever du soleil et du coucher du soleil tous les jours à 00h01
TaskHandle_t calculateSunriseSunsetTaskHandle = NULL; // Variable pour stocker le handle de la tâche calculateSunriseSunsetTask
void calculateSunriseSunsetTask(void *parameter)
{
  const int secondsPerDay = 86400; // Nombre de secondes dans une journée
  const int targetHour = 0;        // Heure cible (00h)
  const int targetMinute = 1;      // Minute cible (01)

  for (;;)
  {
    DateTime now = rtc.now();
    DateTime nextExecution(now.year(), now.month(), now.day(), targetHour, targetMinute, 0);

    // Si l'heure actuelle est après 00h01, définir la prochaine exécution pour le jour suivant
    if (now >= nextExecution)
    {
      nextExecution = nextExecution + TimeSpan(1, 0, 0, 0);
    }

    // Calculer le délai en secondes jusqu'à la prochaine exécution
    long delaySeconds = (nextExecution.unixtime() - now.unixtime());

    // Convertir le délai en millisecondes et attendre
    vTaskDelay(pdMS_TO_TICKS(delaySeconds * 1000));

    calculateSunriseSunset(); // Recalculer l'heure du lever et du coucher du soleil
  }
}

// Arrêter le mouvement de la porte
void stopPorte()
{
  isDoorMoving = false;
  digitalWrite(motorInput1PinNumber, LOW);
  digitalWrite(motorInput2PinNumber, LOW);
  digitalWrite(motorEnablePinNumber, LOW); // Désactiver le moteur
  Serial.println("Porte arrêtée");
}

// Ouvrir la porte
void ouvrirPorte()
{
  isDoorMoving = true;
  Serial.println("Ouverture de la porte");
  previousDoorDirectionUp = true;           // Mettre à jour la direction précédente
  digitalWrite(motorEnablePinNumber, HIGH); // Activer le moteur
  digitalWrite(motorInput1PinNumber, HIGH);
  digitalWrite(motorInput2PinNumber, LOW);

  unsigned long startTime = millis(); // Temps de départ pour suivre la durée de mouvement
  unsigned long elapsedTime = 0;      // Temps écoulé depuis le début du mouvement

  while (digitalRead(upperLimitSwitchPinNumber) == HIGH && elapsedTime < maxDoorMovementDuration)
  {
    // Vérifier si suffisamment de temps s'est écoulé pour laisser d'autres tâches s'exécuter (éviter un blocage)
    delay(1);                           // Attente d'une milliseconde
    elapsedTime = millis() - startTime; // Mettre à jour le temps écoulé
  }

  // Attendre 1 seconde
  delay(1000);
  stopPorte(); // Arrêter la porte
  Serial.println("Porte ouverte");
}

// Fermer la porte
void fermerPorte()
{
  isDoorMoving = true;
  Serial.println("Fermeture de la porte");
  previousDoorDirectionUp = false;          // Mettre à jour la direction précédente
  digitalWrite(motorEnablePinNumber, HIGH); // Activer le moteur
  digitalWrite(motorInput1PinNumber, LOW);
  digitalWrite(motorInput2PinNumber, HIGH);

  unsigned long startTime = millis(); // Temps de départ pour suivre la durée de mouvement
  unsigned long elapsedTime = 0;      // Temps écoulé depuis le début du mouvement

  while (digitalRead(lowerLimitSwitchPinNumber) == HIGH && elapsedTime < maxDoorMovementDuration)
  {
    // Vérifier si suffisamment de temps s'est écoulé pour laisser d'autres tâches s'exécuter (éviter un blocage)
    delay(1);                           // Attente d'une milliseconde
    elapsedTime = millis() - startTime; // Mettre à jour le temps écoulé
  }

  // Attendre 1 seconde
  delay(1000);
  stopPorte(); // Arrêter la porte
  Serial.println("Porte fermée");
}

// Fonction pour gérer l'interruption du bouton poussoir
void handleButtonPress()
{
  // Lire l'état du bouton poussoir
  int buttonState = digitalRead(buttonPinNumber);

  // Vérifier si le bouton est enfoncé
  if (buttonState == LOW && millis() - lastButtonPressTimestamp >= buttonDebounceDelay)
  {
    // Mettre à jour le moment du dernier appui sur le bouton
    lastButtonPressTimestamp = millis();
    Serial.println("Appui sur le bouton détecté");

    // Vérifier si la porte n'est pas en mouvement
    if (!isDoorMoving)
    {
      // Obtenir l'état des broches du moteur
      bool motorInput1State = digitalRead(motorInput1PinNumber);
      bool motorInput2State = digitalRead(motorInput2PinNumber);

      // Arrêter la porte si elle est en mouvement
      if (motorInput1State || motorInput2State)
      {
        stopPorte();
      }
      else
      {
        // Déterminer la direction en fonction des capteurs de fin de course
        bool upperLimitActive = digitalRead(upperLimitSwitchPinNumber) == LOW;
        bool lowerLimitActive = digitalRead(lowerLimitSwitchPinNumber) == LOW;

        if (upperLimitActive && lowerLimitActive)
        {
          // Les deux capteurs de fin de course sont inactifs, la porte est à mi-chemin
          if (previousDoorDirectionUp)
          {
            fermerPorte();
          }
          else
          {
            ouvrirPorte();
          }
        }
        else if (upperLimitActive)
        {
          fermerPorte();
        }
        else if (lowerLimitActive)
        {
          ouvrirPorte();
        }
      }
    }
  }
}

// Initialisation de la porte
void initPorte()
{
  // Déterminer si la porte doit être ouverte ou fermée au démarrage
  DateTime now = rtc.now();
  int currentMinuteOfDay = now.hour() * 60 + now.minute();

  if (currentMinuteOfDay >= sunriseTime && currentMinuteOfDay < sunsetTime)
  {
    // L'heure actuelle est entre le lever et le coucher du soleil, ouvrir la porte
    ouvrirPorte();
  }
  else
  {
    // L'heure actuelle est entre le coucher du soleil et le lever du lendemain, fermer la porte
    fermerPorte();
  }
}

// Affichage de la liste des tâches
void printTaskList()
{
  char taskListBuffer[512];
  vTaskList(taskListBuffer);
  Serial.println("Task List:");
  Serial.println(taskListBuffer);
}

// Tâche d'affichage de la liste des tâches
TaskHandle_t printTaskListTaskHandle = NULL; // Variable pour stocker le handle de la tâche d'affichage
void printTaskListTask(void *parameter)
{
  for (;;)
  {
    // Appelez la fonction pour afficher la liste des tâches
    printTaskList();

    // Attente d'un certain intervalle (par exemple, 30 minutes)
    vTaskDelay(pdMS_TO_TICKS(30 * 60 * 1000)); // 30 minutes
  }
}

// Tâche de fermeture de porte
TaskHandle_t closeDoorTaskHandle = NULL; // Variable pour stocker le handle de la tâche de fermeture de porte
void closeDoorTask(void *parameter)
{
  for (;;)
  {
    fermerPorte();

    vTaskDelay(pdMS_TO_TICKS((sunsetTime + 30) * 60 * 1000));
  }
}

// Tâche d'ouverture de porte
TaskHandle_t openDoorTaskHandle = NULL; // Variable pour stocker le handle de la tâche d'ouverture de porte
void openDoorTask(void *parameter)
{
  for (;;)
  {
    ouvrirPorte();

    vTaskDelay(pdMS_TO_TICKS((sunriseTime + 5) * 60 * 1000));
  }
}

// Serveur WEB
AsyncWebServer server(80);

// Fonction d'initialisation
void setup()
{

  // Initialisation console série
  Serial.begin(115200);

  // Se connecter au réseau WiFi
  bool isWiFiConnected = connectWiFi();

  // Démarrer la communication I2C (module RTC)
  Wire.begin();

  // Démarrer la communication avec le module RTC
  rtc.begin();

  // Configuration des broches en entrée/sortie
  pinMode(buttonPinNumber, INPUT_PULLUP);
  pinMode(motorEnablePinNumber, OUTPUT);
  pinMode(motorInput1PinNumber, OUTPUT);
  pinMode(motorInput2PinNumber, OUTPUT);
  pinMode(upperLimitSwitchPinNumber, INPUT_PULLUP);
  pinMode(lowerLimitSwitchPinNumber, INPUT_PULLUP);

  // Désactiver le moteur au démarrage
  digitalWrite(motorEnablePinNumber, LOW);

  // Mettre à jour l'heure du module DS3231 au démarrage avec l'heure NTP
  updateDS3231Time();

  // Calculer l'heure du lever du soleil et du coucher du soleil au démarrage
  calculateSunriseSunset();

  // Appeler la fonction pour initialiser la porte
  initPorte();

  // Créer une tâche pour la reconnexion WiFi
  xTaskCreatePinnedToCore(WiFiReconnectTask, "WiFiReconnectTask", 4096, NULL, 1, &WiFiReconnectTaskHandle, 0);
  // Démarrer une tâche pour afficher le RSSI toutes les 15 minutes
  xTaskCreatePinnedToCore(printRSSITask, "printRSSITask", 4096, NULL, 1, &printRSSITaskHandle, 0);
  // Démarrer une tâche pour la mise à jour de l'heure toutes les 24 heures
  xTaskCreatePinnedToCore(updateTimeTask, "updateTimeTask", 4096, NULL, 1, &updateTimeTaskHandle, 0);
  // Démarrer une tâche pour recalculer l'heure du lever du soleil et du coucher du soleil toutes les 24 heures à 00h01
  xTaskCreatePinnedToCore(calculateSunriseSunsetTask, "calculateSunriseSunsetTask", 4096, NULL, 1, &calculateSunriseSunsetTaskHandle, 0);
  // Créer la tâche d'impression de la liste des tâches
  xTaskCreatePinnedToCore(printTaskListTask, "printTaskListTask", 4096, NULL, 1, &printTaskListTaskHandle, 0);
  // Créer la tâche de fermeture de porte
  xTaskCreatePinnedToCore(closeDoorTask, "closeDoorTask", 4096, NULL, 1, &closeDoorTaskHandle, 0);
  // Créer la tâche d'ouverture de porte
  xTaskCreatePinnedToCore(openDoorTask, "openDoorTask", 4096, NULL, 1, &openDoorTaskHandle, 0);

  // Attachement des interruptions pour le bouton poussoir
  attachInterrupt(digitalPinToInterrupt(buttonPinNumber), handleButtonPress, CHANGE);

  // Page WEB
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server started");
}

// Fonction de boucle principale
void loop()
{
  // Le code principal de votre programme va ici...
}
