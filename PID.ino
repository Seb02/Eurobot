#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_PWMServoDriver.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Broches pour les encodeurs
int encodeur1_pinA = 2;
int encodeur1_pinB = 3;
int encodeur2_pinA = 4;
int encodeur2_pinB = 5;

// Variables pour le contrôleur PID
double vitesse = 0.1;    // Vitesse cible en m/s
double objectif = 40*vitesse/0.06;  // Vitesse cible en ticks par seconde
double kp = 0.1;    // Coefficient proportionnel
double ki = 0.01;   // Coefficient intégral
double kd = 0.02;   // Coefficient dérivé

// Variables pour le contrôle PID
int erreur_prec = 0;
double integrale = 0;

void setup() {
  // Initialisation du shield moteur
  AFMS.begin();

  // Configuration des broches des encodeurs en mode INPUT
  pinMode(encodeur1_pinA, INPUT);
  pinMode(encodeur1_pinB, INPUT);
  pinMode(encodeur2_pinA, INPUT);
  pinMode(encodeur2_pinB, INPUT);
}

void loop() {
  // Lire les valeurs des encodeurs
  int ticks_encodeur1 = lire_valeur_encodeur(encodeur1_pinA, encodeur1_pinB);
  int ticks_encodeur2 = lire_valeur_encodeur(encodeur2_pinA, encodeur2_pinB);

  // Calculer l'erreur de vitesse pour chaque moteur
  int erreur_moteur1 = objectif - ticks_encodeur1;
  int erreur_moteur2 = objectif - ticks_encodeur2;

  // Calculer la sortie PID pour chaque moteur
  double sortie_pid_moteur1 = calcul_pid(erreur_moteur1);
  double sortie_pid_moteur2 = calcul_pid(erreur_moteur2);

  // Définir la vitesse des moteurs en fonction de la sortie PID
  motor1->setSpeed(abs(sortie_pid_moteur1));
  motor2->setSpeed(abs(sortie_pid_moteur2));

  // Déterminer la direction des moteurs en fonction de la sortie PID
  if (sortie_pid_moteur1 >= 0) {
    motor1->run(FORWARD);
  } else {
    motor1->run(BACKWARD);
  }

  if (sortie_pid_moteur2 >= 0) {
    motor2->run(FORWARD);
  } else {
    motor2->run(BACKWARD);
  }

  // Actualiser les valeurs précédentes
  erreur_prec = erreur_moteur1;

  // Mettre à jour l'intégrale
  integrale += erreur_moteur1;

  // Appliquer les commandes de vitesse aux moteurs
  AFMS.getMotor(1)->run(FORWARD);
  AFMS.getMotor(2)->run(FORWARD);

  // Appliquer la fréquence de boucle appropriée
  delay(100);
}

  // Fonction pour lire la valeur de l'encodeur
int lire_valeur_encodeur(int pinA, int pinB) {
  static int position = 0;  // Cette variable statique conserve la position actuelle de l'encodeur, qui est initialement définie à zéro. Elle est mise à jour à mesure que l'encodeur tourne.
  static int lastEncoded = 0; // Cette variable statique conserve la valeur précédente de l'état de l'encodeur (combinaison binaire des signaux A et B).
  static unsigned long lastMillis = 0;  // Cette variable statique conserve le temps (en millisecondes) de la dernière mise à jour de l'encodeur.
  
  int MSB = digitalRead(pinA); // Lire l'état de la broche A
  int LSB = digitalRead(pinB); // Lire l'état de la broche B
  int encoded = (MSB << 1) | LSB; // Combinez les deux valeurs pour obtenir une valeur unique

  // Vérifiez si l'état a changé et assez de temps s'est écoulé pour éviter les rebonds
  if (encoded != lastEncoded && (millis() - lastMillis) > 10) {
    if ((lastEncoded == 0b00 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b00)) {
      // Sens horaire
      position++;
    } else if ((lastEncoded == 0b00 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b00)) {
      // Sens anti-horaire
      position--;
    }
    lastMillis = millis();
  }
  
  lastEncoded = encoded;
  
  return position;  // La fonction retourne la nouvelle position de l'encodeur après avoir mis à jour sa valeur en fonction des rotations détectées.
}

}
