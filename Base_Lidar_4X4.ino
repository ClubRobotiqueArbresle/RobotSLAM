// Base robot DAGU 4x4 sur Mega
// Version 1.2 avec controleur pour holowheel 19/11/2021
// Version 1.1 avec LCD I2C
// fonctions deplacemnt : OK
// Gestion interuption : Ok
// ROS Serial OK a tester
// 4/01/21 : Ajout IMU
// 4/01/21 : Ajout GPS
// 18/04/21 : correction de l'IMU
// 28/12/2021 : Ajout du parechoc avec capteurs US

// Roues de 125 mm

// 1 metre en 2s soit 0,5 m /sec ou 1,8 Km/h
// rapport Odometrie CM : 0,13542

// version ROS a commenter pour desactive ROS
//#define ROS

// Bib
// Pour le LCD
#include <LCD_I2C.h>

// MPU6050
#include "Wire.h"
#include <MPU6050_light.h>

// Broches Moteurs
const int IN1 = 4; // bleu
const int IN2 = 5; // vert
const int IN3 = 6; //jaune
const int IN4 = 7; // violet
const int ENBA = 8; //blanc
const int ENBB = 9; //gris

// Broches Capteurs US
const int CapUSDroite_Echo = 23;
const int CapUSDroite_Trig = 25;
const int CapUSCentre_Trig = 27;
const int CapUSCentre_Echo = 29;
const int CapUSGauche_Trig = 31;
const int CapUSGauche_Echo = 33;
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;
// Distance de detection d'obstacles
const int DistanceMini = 20; // en Cm
int TableauDistance[3][2] = { {1, 0}, {2, 0}, {3, 0} };

// Broches Odometrie
const int OdoG1 = 2;
// const int OdoG2 = 19;
const int OdoD1 = 3;
// const int OdoD2 = 3;

// Mega, Mega2560, MegaADK
// interrupt : 2, 3, 18, 19, 20, 21

// Globales Déplacement
long CompteurOdoG = 0;
long CompteurOdoD = 0;
unsigned long timer = 0;
int VitesseGauche = 0;
int VitesseDroite = 0;
const int VitesseMin = 40;
const int VitesseMinRecule = -40;
unsigned int Compteur = 0;
long Moyenne;
long Distance;
const float RapportOdo = 0.13542; // Rapport odometrie
float VitesseRobot = 0;

// Constructeurs
LCD_I2C lcd(0x27);
MPU6050 mpu(Wire);

#include "Deplacement.h"
#include "Fonctions_ros.h"

// Fonction gestion LCD
void Affiche_Ligne(int LigneDebut, int Colonne, String Chaine)
{
  for (int i = 0; i < Chaine.length(); i++)
  {
    lcd.setCursor(LigneDebut + i, Colonne);
    lcd.print(Chaine[i]);
  }
}

void OdoG()
{
  CompteurOdoG ++;
}

void OdoD()
{
  CompteurOdoD ++;
}

void AfficheOdo()
{
  String Message = "Odo : " + String(Moyenne) + "          Dst:" + String(Distance) + " Vit:" + String(VitesseRobot);
  Affiche_Ligne(0, 0, Message);
  // lcd.setCursor(0, 1); lcd.print(Message);
}

void AfficheIMU()
{
  String Message = "Dir:" + String(mpu.getAngleZ()) + "    ";
  // "X:" + String(mpu.getAngleX()) + "Y:" + String(mpu.getAngleY()) +
  Affiche_Ligne(0, 2, Message);
}

void AfficheVitesse()
{
  String Message = "G:" + String(VitesseGauche) + " D:" + String(VitesseDroite) + "    ";
  Affiche_Ligne(0, 2, Message);
}

int CapteurUSGauche()
{
  digitalWrite(CapUSGauche_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(CapUSGauche_Trig, LOW);
  long measure = pulseIn(CapUSGauche_Echo, HIGH, MEASURE_TIMEOUT);
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  return int(distance_mm / 10.0);
}

int CapteurUSDroite()
{
  digitalWrite(CapUSDroite_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(CapUSDroite_Trig, LOW);
  long measure = pulseIn(CapUSDroite_Echo, HIGH, MEASURE_TIMEOUT);
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  return int(distance_mm / 10.0);
}

int CapteurUSCentre()
{
  digitalWrite(CapUSCentre_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(CapUSCentre_Trig, LOW);
  long measure = pulseIn(CapUSCentre_Echo, HIGH, MEASURE_TIMEOUT);
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  return int(distance_mm / 10.0);
}

int DetectionObstacles()
{
  int Capteur = 0; // reset capteur
  // Reset du tableau des capteurs US
  TableauDistance[1][2] = 0;
  TableauDistance[2][2] = 0;
  TableauDistance[3][2] = 0;
  // Mise a jour des capteurs US
  TableauDistance[1][2] = CapteurUSGauche();
  TableauDistance[2][2] = CapteurUSCentre();
  TableauDistance[3][2] = CapteurUSDroite();

  if (TableauDistance[1][2] < DistanceMini ) // si Capteur Gauche inferieur a 5 cm
  {
    Capteur = 1;
  }
  if (TableauDistance[2][2] < DistanceMini ) // si Capteur Centre inferieur a 5 cm
  {
    Capteur = 2;
  }
  if (TableauDistance[3][2] < DistanceMini ) // si Capteur Droite inferieur a 5 cm
  {
    Capteur = 3;
  }
  return Capteur;
}

void setup() {
  // serial 2 17(RX), 16(TX)
  // serial 3 15(RX), 14(TX)
  Serial.begin(115200); // Pour console
  Serial.println("setup");
  // ROS
#ifdef ROS
  nh.initNode();
  nh.advertise(Odometrie);
  nh.advertise(imu);
  nh.advertise(gps);
  nh.subscribe(moteurs);
#endif

  // LCD
  lcd.begin();
  lcd.backlight();

  // MPU
  Wire.begin();
  byte status = mpu.begin();
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero

  // PIN moteurs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENBA, OUTPUT);
  pinMode(ENBB, OUTPUT);

  // PIN Capteurs US
  /* broches Capteur Centre */
  pinMode(CapUSCentre_Trig, OUTPUT);
  digitalWrite(CapUSCentre_Trig, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(CapUSCentre_Echo, INPUT);

  /* broches Capteur Droite */
  pinMode(CapUSDroite_Trig, OUTPUT);
  digitalWrite(CapUSDroite_Trig, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(CapUSDroite_Echo, INPUT);

  /* broches Capteur Gauche */
  pinMode(CapUSGauche_Trig, OUTPUT);
  digitalWrite(CapUSGauche_Trig, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(CapUSGauche_Echo, INPUT);

  // Pin Odometrie
  pinMode(OdoG1, INPUT_PULLUP);
  // pinMode(OdoG2, INPUT_PULLUP);
  pinMode(OdoD1, INPUT_PULLUP);
  // pinMode(OdoD2, INPUT_PULLUP);

  // Gestion des interruptions pour l'odometrie
  attachInterrupt(digitalPinToInterrupt(OdoG1), OdoG, RISING);
  attachInterrupt(digitalPinToInterrupt(OdoD1), OdoD, RISING);

}

void loop() {
  // lecture de l IMU
  mpu.update();

  Moyenne = (CompteurOdoG + CompteurOdoD) / 2;
  // Distance = Moyenne / 75  * 125 * PI; // Rapport de 1/75eme au niveau de la boite et 125 mm de roue
  Distance = Moyenne * RapportOdo; // rapport odemetrie vers reel en cm

  // Application des variables Moteurs
  //
  // pour test du nouveau controleur
  // Rafraichissment toutes les 100 ms

  if ((millis() - timer) > 100) {
    Compteur ++;
    Serial.println(Compteur);
    VitesseRobot = Distance / (Compteur * 10); // Calcul de vitesse reele Distance sur temps en m/s

    AfficheOdo(); // sur LCD Ligne 1 et 3
    // AfficheIMU(); // IMU sur LCD Ligne 2
    AfficheVitesse(); // Vitesse Moteurs Ligne 2

#ifdef ROS
    nh.spinOnce();
    PublishOdo(); // Publication ROS Odo
    PublishIMU(); // Publication ROS IMU
#endif
    
    Serial.println(Compteur);
    Serial.print("Distance : ");
    Serial.println(Distance);
    Serial.print("Odometrie : ");
    Serial.println(Moyenne);
    timer = millis();
  } // fin pulsation
  Serial.println("fin pulsation");
  // Detection d'obstacle
  int Obstacle = DetectionObstacles();

  Serial.println("Detection Obstacles");
  Serial.println(Obstacle);
  switch (Obstacle)
  {
    case 1: // gauche
      {
        Serial.println("Obstacle a Gauche tourne a droite");
        VirageDroite();
        break;
      }
      
    case 2:
      {
        Serial.println("Obstacle au Centre tourne a 90 degres");
        Stop();
        break;
      }
    case 3:
      {
        Serial.println("Obstacle a Droite tourne a gauche");
        VirageGauche();
        break;
      }
    default:
      ToutDroit();
  }

  Serial.print("Vitesse Gauche = "); Serial.println(VitesseGauche);
  Serial.print("Vitesse Droite = "); Serial.println(VitesseDroite);

  //Avance en fonction des valeurs
  Avance();
  
  /*
    if (Serial.available() > 0) {
      char Recu = Serial.read();
      // lcd.setCursor(0, 1); lcd.print(Recu);
      switch (Recu) {
        case 'A':
          Bouge(VitesseGauche, VitesseDroite);
          break;
        case 'P':
          VitessePlus();
          break;
        case 'M':
          VitesseMoins();
          break;
        case 'G':
          VirageGauche();
          break;
        case 'D':
          VirageDroite();
          break;
        case 'S':
          Stop();
          break;
        default:
          break;
      } // fin switch

    } // fin serial read
  */
  delay(500);
} // fin loop
