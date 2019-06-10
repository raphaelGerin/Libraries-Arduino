#ifndef BASEROULANTE_H
#define BASEROULANTE_H

#include "Arduino.h"
#include<DCMotor.h>
#include <PID_v1.h>

#define BAUDRATE 1000000

class BaseRoulante
{
public:
    BaseRoulante(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues);
    void avancerTourner(float v, float theta);          // vitesseMoyenne, vitesseRotation
    void avancer(float vg, float vd); //vitesse roue gauche, vitesse roue droite
    bool tournerPrecis(float theta, float precision);       // angle, precision
    bool suivrePoint(float*, float precision);   //tableauCoordonnées, precision
    bool suivrePoint(float xCible, float yCible, float precision);   //x, y, precision
    void allerRetour(float x,float y); //x, y
    void actualiserPosition();
    void setPosition(double , double); //posX, posY
    void setPosition(double, double, double); //posX, posY et angle
    double getPosX();
    double getPosY();
    double getAngle();
    float getVg();
    float getVd();
    void setAcceleration(float acc);
    void setAngleCorrecteur(float kp, float ki, float kd);       //P, I, D
    void setDistanceCorrecteur(float kp, float ki, float kd);    //P, I, D
    void setAngleSvrPtCorrecteur(float kp, float ki, float kd);
    void setIntegralSaturation(float sat);
    void setPeriode(byte periode); // période de mise à jour pour les asservissements

    void setRapportAvancerTourner(float r); // plus l'angle vers la cible est grand, moins on va vite
    double modulo360(double angle); //permet d'avoir un angle entre -180° et +180°
    void debug(); //C'est ici que sont mis tous les prints relatifs au robot
    void setDebugMode(byte mode); // Permet de choisir une écriture allégée du débug


private :
    DCMotor* m_roueGauche;
    DCMotor* m_roueDroite;
    PS2MouseEgab* m_sourisGauche;
    PS2MouseEgab* m_sourisDroite;

    byte m_debugMode=1;
    bool m_debug_asservissement = false;

//----------Charactéristiques du robot---------------------
    float m_empattement;        //distance entre le centre de rotation du robot et une roue en mm !
    float m_rayon; //37.64         //rayon des roues en mm
    double m_angle; // Sotck l'angle du robot au cours du temps, il va quand mÃªme rester Ã  dÃ©finir un cotÃ© + et un -
    double m_posX; //
    double m_posY; // =
    float m_vg = 0;
    float m_vd = 0;
    float m_acc = 0.1;  //Float pour pouvoir faire des pentes très douces
    double m_encodeurPrecedentGauche;
    double m_encodeurPrecedentDroit;
//---------Asservissements----------------------------------
    int m_periode = 5;

    //En angle :
    double m_consigneAngle; // consigne pour le correcteur PID en angle
    double m_vitesseRotation; // sortie du PID pour régler l'angle absolu du robot
    float m_Kp = 0.55;
    float m_Ki = 0.8;
    float m_Kd = 0;

    //En suivi de trajectoire :
    double m_consigneAngleSvrPt; // consigne pour le correcteur PID en angle d'avancerTourner
    double m_vitesseRotationSvrPt; // sortie du PID pour régler l'angle absolu du robot
    float m_KpSvrPt = 0.25;
    float m_KiSvrPt = 0.25;
    float m_KdSvrPt = 0;
    bool m_modeSvrPt;

    //En distance :
    double m_consigneDistance = 0; //consigne pour le PID en distance
    double m_distance;   // entrée du PID
    double m_vitesseMoyenne; // sotie du PID
    double m_vitesseMoyenneMax = 150;
    float m_Kpd = 20;
    float m_Kid = 0.2;
    float m_Kdd = 0;

    float m_rapportAvancerTourner = 0;
    PID correctionAngle = PID(&m_angle, &m_vitesseRotation, &m_consigneAngle, m_Kp, m_Ki, m_Kd, true);
    PID correctionDistance = PID(&m_distance, &m_vitesseMoyenne, &m_consigneDistance, m_Kpd, m_Kid, m_Kdd, false);
    PID correctionAngleSvrPt = PID(&m_angle, &m_vitesseRotationSvrPt, &m_consigneAngleSvrPt, m_KpSvrPt, m_KiSvrPt, m_KdSvrPt, true);

//---------variables pour le maniement séquentiel du robot-----------------------
    int m_compteur = 0;
    int m_etape = 0; //permet de savoir où en est le robot dans la rÃ©alisation des taches
    bool m_setupAllerRetour = false;
    float xInit = m_posX;
    float yInit = m_posY;
    float angleInit = m_angle;
};

#endif // BASEROULANTE_H
