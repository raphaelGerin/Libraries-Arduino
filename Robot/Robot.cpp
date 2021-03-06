
#include <Robot.h>


//Constructor :
Robot::Robot(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues) // : rajouter un héritage ...  float empattement = 127.5, float rayonRoues = 49.5)
{
    Serial.begin(1000000);
//    Serial.print("battery level : ");
//    Serial.println(Robot::getBattery()/10);
    m_roueGauche = roueGauche;
    m_roueDroite = roueDroite;
    m_empattement = empattement;
    m_rayon = rayonRoues;
    m_angle = 0;                        // Sotck l'angle du robot au cours du temps, il va quand mÃªme rester Ã  dÃ©finir un cotÃ© + et un -
    m_posX = 0; //
    m_posY = 0; // =
    m_encodeurPrecedentGauche = m_roueGauche->getCodeur(); //normalement ils valent 0 au dÃ©but
    m_encodeurPrecedentDroit = m_roueDroite->getCodeur();

    pinMode(pinDemarrageOutput,OUTPUT);
    pinMode(pinDemarrageInput,INPUT_PULLUP);

    pinMode(pinInterrupteurPince,OUTPUT);
    pinMode(pinOuverturePince,INPUT_PULLUP);
    pinMode(pinSerragePince,INPUT_PULLUP);

    pinMode(ledON,OUTPUT);
    pinMode(ledJaune,OUTPUT);
    pinMode(ledArretUrgence,OUTPUT);
    pinMode(ledEtapeFinie,OUTPUT);
    pinMode(ledEtape1,OUTPUT);
    pinMode(ledEtape2,OUTPUT);
    pinMode(ledEtape3,OUTPUT);
    pinMode(ledEtape4,OUTPUT);

}

/*Robot::Robot(DCMotor *roueGauche, DCMotor *roueDroite, float empattement, float rayonRoues, PS2MouseEgab* sourisGache, PS2MouseEgab* sourisDroite, float angleInterSouris)
{
    Serial.begin(1000000);
    Serial.print("battery level : ");
//    Serial.println(Robot::getBattery()/10);
    m_roueGauche = roueGauche;
    m_roueDroite = roueDroite;
    m_empattement = empattement;
    m_rayon = rayonRoues;
    m_angle = 0;
    m_posX = 0;
    m_posY = 0;
    m_sourisGauche = sourisGache;
    m_sourisDroite = sourisDroite;
}
*/

void Robot::debug()
{
    if (!Serial.available()) Serial.begin(1000000);
    //Serial.print("temps\t");
    Serial.print(millis());
/*    Serial.print("\tX ");
    Serial.print(m_posX);
    Serial.print("\tY ");
    Serial.print(m_posY);
    Serial.print("\t° ");
    Serial.print(m_angle);
*/
/*
    Serial.print("\t\tC_rot° ");
    Serial.print(m_vitesseRotationSvrPt*m_#include "Arduino.h"+m_vitesseRotation*(1-m_#include "Arduino.h"));
    Serial.print("\tC_vit ");
    Serial.print(m_vitesseMoyenne);
    Serial.print("\tC_angle :");
    Serial.print(m_consigneAngleSvrPt*m_#include "Arduino.h"+m_consigneAngle*(1-m_#include "Arduino.h"));
    Serial.print("\td ");
    Serial.print(m_distance);
    Serial.print("\t#");
    Serial.print(m_etape);
*/
    Serial.print("\t");
    Serial.print(m_angleComputedSvrPt);
    Serial.print(m_angleComputedSvrPt);

    Serial.println();
}

void Robot::reglagePinceManuel()
{
    if (!digitalRead(pinSerragePince) and pinceDebloquee)
      {
        digitalWrite(ledEtape2,HIGH);   // led jaune debug
        digitalWrite(ledEtape4, LOW);
        sensMoteurPince(1);//sens qui serre la pince
        analogWrite(pinInputPince, 255);
        delay(50);
        //blocage moteur
        if (analogRead(pinCapteurPince) > 80)
        {
          digitalWrite(ledEtape1, HIGH); // allumage led verte
          pinceDebloquee = false;
          analogWrite(pinInputPince, 0);
        }
        else
        {
          digitalWrite(ledEtape1, LOW); // extinction de la led verte
        }
        deblocageOuverture = true;
      }
    else if (!digitalRead(pinOuverturePince) and deblocageOuverture)// on dessert la pince
      {
        digitalWrite(ledEtape2,LOW);
        pinceDebloquee = true;
        sensMoteurPince(-1);//sens qui desserre la pince
        analogWrite(pinInputPince, 255);
        delay(50);
        digitalWrite(ledEtape4, HIGH); // led bleue debug
        // blocage à l'ouverture
        if (analogRead(pinCapteurPince) > 70)
        {
          digitalWrite(ledEtape1, HIGH); // allumage led
          deblocageOuverture = false;
          analogWrite(pinInputPince, 0);
        }
        else
        {
          digitalWrite(ledEtape1, LOW); // extinction de la led
        }

      }
      else
      {
        digitalWrite(ledEtape2,LOW);
        digitalWrite(ledEtape4, LOW);
        analogWrite(pinInputPince, 0);
      }
}

void Robot::sensMoteurPince(int sens)
{
  if (sens == 1) //sens qui serre la pince
  {
    digitalWrite(pinIN1Pince, HIGH);
    digitalWrite(pinIN2Pince, LOW);
  }
  if (sens == -1) //sens qui desserre la pince
  {
    digitalWrite(pinIN1Pince, LOW);
    digitalWrite(pinIN2Pince, HIGH);
  }
}


void Robot::initLeds()
{
    byte tableauLed[8] = {ledON, ledJaune, ledArretUrgence, ledEtapeFinie, ledEtape1, ledEtape2, ledEtape3, ledEtape4};
    allumerLeds();
    delay(500);

    for (int i=0;i<8;i++)
    {
        digitalWrite(tableauLed[i],HIGH);
        delay(50);
    }
    delay(80);

    for (int i=7;i>=0;i--)
    {
        digitalWrite(tableauLed[i],LOW);
        delay(50);
    }
    delay(80);

    for (int i=0;i<8*3;i++)
    {
        digitalWrite(tableauLed[i%8],HIGH);
        digitalWrite(tableauLed[(i-3)%8], LOW);
        delay(50);
    }
    delay(50);
    eteindreLeds();
}

void Robot::jeuLeds()
{
    byte tableauLed[8] = {ledON, ledJaune, ledArretUrgence, ledEtapeFinie, ledEtape1, ledEtape2, ledEtape3, ledEtape4};

    allumerLeds();
    delay(800);
    eteindreLeds();
    delay(300);
    for (int i=0;i<8;i++)
    {
        digitalWrite(tableauLed[i],HIGH);
        delay(50);
    }
    delay(100);
    eteindreLeds();
    delay(400);

    for (int i=0;i<8;i++)
    {
        digitalWrite(tableauLed[i],HIGH);
        delay(50);
    }
    delay(100);

    for (int i=0;i<8;i++)
    {
        digitalWrite(tableauLed[i],LOW);
        delay(50);
    }
    delay(200);

    for (int i=0;i<8;i++)
    {
        digitalWrite(tableauLed[i],HIGH);
        delay(50);
    }
    delay(100);

    for (int i=7;i>=0;i--)
    {
        digitalWrite(tableauLed[i],LOW);
        delay(50);
    }

    for (int i=0;i<8*5;i++)
    {
        digitalWrite(tableauLed[i%8],HIGH);
        digitalWrite(tableauLed[(i-3)%8], LOW);
        delay(50);
    }
    delay(50);
    eteindreLeds();
}

void Robot::allerRetour(float x, float y)
{
    if (!m_setupAllerRetour)
    {
        xInit = m_posX;
        yInit = m_posY;
        angleInit = m_angle;
        m_setupAllerRetour = true;
    }
    switch (m_etape)
    {
    case 0 :
        if (Robot::suivrePoint(x,y,2))
        {
            m_compteur++;
            digitalWrite(ledEtape1,HIGH);
        }
        else
        {
            m_compteur = 0;
            digitalWrite(ledEtape1,LOW);
        }

        if (m_compteur>200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    case 1 :
        if (Robot::tournerPrecis(angleInit+180,1))
        {
            m_compteur++;
            digitalWrite(ledEtape2,HIGH);
        }
        else
        {
            m_compteur = 0;
            digitalWrite(ledEtape2,LOW);
        }
        if (m_compteur > 200)
        {
            m_compteur = 0;
            m_etape++;
            digitalWrite(ledEtape2,LOW);
        }
        break;
    case 2 :
        if (Robot::suivrePoint(xInit,yInit,2))
        {
            m_compteur++;
            digitalWrite(ledEtape3,HIGH);
        }
        else
        {
            m_compteur = 0;
            digitalWrite(ledEtape3,LOW);
        }
        if (m_compteur>200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
    case 3 :
        if (Robot::tournerPrecis(angleInit,1))
        {
            m_compteur++;
            digitalWrite(ledEtape4,HIGH);
        }
        else
        {
            m_compteur = 0;
            digitalWrite(ledEtape4,LOW);
        }
        if (m_compteur > 200)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
/*
    case 4 :
        if (Robot::suivrePoint(xInit,yInit,2))
        {
            m_compteur++;
            digitalWrite(ledEtape1,HIGH);
            digitalWrite(ledEtape2,HIGH);
            digitalWrite(ledEtape3,HIGH);
            digitalWrite(ledEtape4,HIGH);
        }
        else
        {
            m_compteur = 0;
            digitalWrite(ledEtape1,LOW);
            digitalWrite(ledEtape2,LOW);
            digitalWrite(ledEtape3,LOW);
            digitalWrite(ledEtape4,LOW);
        }
        if (m_compteur > 2000)
        {
            m_compteur = 0;
            m_etape++;
        }
        break;
*/
    default :
        m_etape = 0;
        break;
    }
    Robot::actualiserPosition();
}


bool Robot::tirageTirette()
{
    digitalWrite(pinDemarrageInput,HIGH);
    if (digitalRead(pinDemarrageInput))
    {
        digitalWrite(ledJaune,HIGH);
        return true;
    }
    else
    {
        digitalWrite(ledJaune,LOW);
        return false;
    }
}

void Robot::actualiserPosition() // tourne Ã  une visteese x1 pour le moteur 1 et idem pour le 2 : x1 = GAUCHE
{
  //#ifdef CODEURS
  double encodeurGauche = m_roueGauche->getCodeur();
  double encodeurDroit = m_roueDroite->getCodeur();
  int nbPasEncodeurGauche = encodeurGauche - m_encodeurPrecedentGauche;
  int nbPasEncodeurDroit = encodeurDroit - m_encodeurPrecedentDroit;
  m_encodeurPrecedentGauche = encodeurGauche;
  m_encodeurPrecedentDroit = encodeurDroit;

  //Pour évite les gros bugs non physiques
  if (abs(nbPasEncodeurGauche) + abs(nbPasEncodeurDroit) < 3000)
  {
    //dans un intervalle de temps :
    double distance_parcourue_roue_gauche = nbPasEncodeurGauche * 2 * PI * m_rayon / m_roueGauche->getReducteur(); //il faut surement rajouter le rapport de rÃ©duction du moteur                       // ici on calcule la distance parcourue. Pour cela, on estime que le temps d'exÃ©cution de l'algo est faible, et le seul facteur limitant est
    double distance_parcourue_roue_droite = nbPasEncodeurDroit * 2 * PI * m_rayon / m_roueDroite->getReducteur();
    double distance_parcourue = (distance_parcourue_roue_gauche + distance_parcourue_roue_droite) / 2;     // on constate avec un raisonmment physique que la distance parcourue par le ce,tre du robot est la moitiÃ© de la somme de celles parcourues par chacunes de ses roues
    m_angle = m_angle + 180/PI*atan((distance_parcourue_roue_gauche - distance_parcourue_roue_droite) / (2 * m_empattement));             // cf simple calme d'angles dans un triangle + ajout de la fonction arctan ðŸ™‚
    m_angle = modulo180(m_angle); // permet d'avoir un angle entre -180 et +180°
    m_posX += distance_parcourue * cos(PI/180*m_angle);
    m_posY += distance_parcourue * sin(PI/180*m_angle);
  }
  //#endif
  /*#ifdef SOURIS
  m_sourisGauche->actualiserPosition();
  m_sourisDroite->actualiserPosition();

  float x1=m_sourisGauche->getpositionX();//récupération de l'abscisse de la souris 1 dans son repère (celui de référence par convention).
  float y1=m_sourisGauche->getpositionY();//récupération de l'ordonnée de la souris 1 dans son repère (celui de référence par convention).

  float X2=m_sourisDroite->getpositionX();//récupération de l'abscisse de la souris 2 dans son repère.
  float X2=m_sourisDroite->getpositionY();//récupération de l'abscisse de la souris 2 dans son repère.

  float x2=X2*cos(m_angleInterSouris)-Y2*sin(m_angleInterSouris); //calcul des coordonées de la souris 2 dans le repère de la souris 1.
  float y2=X2*sin(m_angleInterSouris)+Y2*cos(m_angleInterSouris);

  m_posX+=(x1+x2)/2; // calcul de la position du centre du robot /!\ On considère sue le centre du robot est le milieu du segment reliant les deux souris
  m_posY+=(y1+y2)/2;
  if(x2==x1){
      m_angle=(2*(y2>y1)-1)*(PI/2);
  }
  else {
      m_angle=atan((y2-y1)/(x2-x1)); //calcul de la position angulaire du robot.
  }
  #endif
*/
}


bool Robot::tournerPrecis(float theta, float precision) // le robot vise l'angle theta en radians
{
  m_modeSvrPt= false;
  m_consigneAngle = modulo180(theta);
  correctionAngle.Compute();
  if (abs(theta - m_angle) <= precision) {
    avancerTourner(0, 0);
    digitalWrite(ledEtapeFinie, HIGH);
    return true;
  }
  digitalWrite(ledEtapeFinie, LOW);
  Robot::avancerTourner(0, -m_vitesseRotation);
  return false;
}

bool Robot::suivrePoint(float xCible, float yCible, float precision)
{
  m_modeSvrPt = true;
  /*
  int diff = millis() - m_lastTime;
  if (diff > 3*m_periode)  m_lastTime = millis() - m_periode;
  if(diff>=m_periode)
  */
  if (true)
  {
          m_distance = sqrt(pow((m_posX - xCible), 2) + pow((m_posY - yCible), 2));
          int sens = ((cos(PI/180*m_angle) * (xCible - m_posX) + sin(PI/180*m_angle) * (yCible - m_posY) ) > 0 ) * 2 - 1; // produit scalaire pour savoir si le robot a dépassé la cible
          m_distance = - sens * m_distance;
          m_distanceComputed = correctionDistance.Compute();

          m_consigneAngleSvrPt = 180/PI*atan((yCible - m_posY) / (xCible - m_posX));
          if(abs(modulo180(m_angle-m_consigneAngleSvrPt))>90) m_consigneAngleSvrPt=modulo180(180+m_consigneAngleSvrPt);
          m_angleComputedSvrPt = correctionAngleSvrPt.Compute();

          m_lastTime +=m_periode;
  }
  else
  {
      m_angleComputedSvrPt = false;
      m_distanceComputed = false;
  }
  if (abs(m_distance) >= precision) //vérifier que abs fonctionne tout le temps avec un float !
  {
      digitalWrite(ledEtapeFinie, LOW);
      Robot::avancerTourner(int(m_vitesseMoyenne / (1 + abs(m_vitesseRotationSvrPt)*m_rapportAvancerTourner)), -m_vitesseRotationSvrPt);
      return false;
  }
  else
  {
    digitalWrite(ledEtapeFinie, HIGH);
    Robot::avancerTourner(0, 0);
    return true;
  }
}


void Robot::avancerTourner(int v, int theta)
{
  theta = constrain(theta, -128, 127);
  v = constrain(v, -128 + abs(theta), 127 - abs(theta));
  //#if PONT_EN_H
  avancer(v-theta,v+theta);
  //#endif

  /*
  #if MD25
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(v);
  Wire.endTransmission();
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(theta);
  Wire.endTransmission();
  #endif
  */
}

void Robot::avancer(int vg, int vd)
{
    vg = constrain(vg,-128,127);
    vd = constrain(vd,-128,127);
    //#ifdef PONT_EN_H
    m_roueGauche->tourneRPM(vg);
    m_roueDroite->tourneRPM(vd);
    //#endif
}

void Robot::setAcceleration(byte acc)
{
  m_acceleration = acc;
}

void Robot::setPosition(double x, double y, double angle)
{
    m_posX = x;
    m_posY = y;
    m_angle = angle;
}

double Robot::getPosX()
{
    return m_posX;
}

double Robot::getPosY()
{
    return m_posY;
}

double Robot::getAngle()
{
    return m_angle;
}

void Robot::setRapportAvancerTourner(float r)
{
    m_rapportAvancerTourner = r;
}

float Robot::getRapportAvancerTourner()
{
    return m_rapportAvancerTourner;
}

void Robot::setAngleCorrecteur(float kp, float ki, float kd)
{
    correctionAngle.SetTunings(kp,ki,kd);
}

void Robot::setDistanceCorrecteur(float kp, float ki, float kd)
{
    correctionDistance.SetTunings(kp,ki,kd);
}

void Robot::setAngleSvrPtCorrecteur(float kp, float ki, float kd)
{
    correctionAngleSvrPt.SetTunings(kp,ki,kd);
}

float Robot::modulo180(float angle) //retourne un angle entre -180 et 180°
{
  return angle - 360 * floor((angle + 180) / 360);
}


void Robot::allumerLeds()
{
    digitalWrite(ledON,HIGH);
    digitalWrite(ledJaune,HIGH);
    digitalWrite(ledArretUrgence,HIGH);
    digitalWrite(ledEtapeFinie,HIGH);
    digitalWrite(ledEtape1,HIGH);
    digitalWrite(ledEtape2,HIGH);
    digitalWrite(ledEtape3,HIGH);
    digitalWrite(ledEtape4,HIGH);

}

void Robot::eteindreLeds()
{
    digitalWrite(ledON,LOW);
    digitalWrite(ledJaune,LOW);
    digitalWrite(ledArretUrgence,LOW);
    digitalWrite(ledEtapeFinie,LOW);
    digitalWrite(ledEtape1,LOW);
    digitalWrite(ledEtape2,LOW);
    digitalWrite(ledEtape3,LOW);
    digitalWrite(ledEtape4,LOW);

}

void Robot::setIntegralSaturation(float sat)
{
    correctionAngle.SetIntegralSaturation(sat);
    correctionDistance.SetIntegralSaturation(sat);
}

