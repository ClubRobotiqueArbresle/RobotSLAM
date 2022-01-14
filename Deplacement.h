
// Fonctions deplacement
void Avance()
{
  Serial.println("Avance");
  lcd.setCursor(5, 4); lcd.print("Avance   ");

  // LOW sens avance
  // HIGH recule

  // Determination sens roues gauches
  if (VitesseGauche != 0) // si vitesse non nulle
  {
    if (VitesseGauche > 0) // roues gauche en avant
    {
      digitalWrite(IN3, LOW); // sens roues <-Gauches
      analogWrite(IN2, VitesseGauche); // vitesse roues Gauche
    } else   { // si sens arriere
      digitalWrite(IN3, HIGH); // sens roues <-Gauches
      analogWrite(IN2, abs(VitesseGauche)-10); // vitesse roues <-Gauches
    }
  } else  {
    analogWrite(IN2, 0); // arret roues Gauche
  }
  // Determination sens roues droites
  if (VitesseGauche != 0) // si vitesse non nulle
  {
    if (VitesseDroite > 0)
    {
      digitalWrite(IN4, LOW); // sens roues -> Droites
      analogWrite(IN1, VitesseDroite); //  vitesse roues Droites
    } else   { // si sens arriere
      digitalWrite(IN4, HIGH); // sens roues -> Droites
      analogWrite(IN1, abs(VitesseDroite)-10); // vitesse roues Droites
    }
  } else  {
    analogWrite(IN1, 0); // arret roues Droite
  }
}

void Bouge(int VitesseGauche, int VitesseDroite)
{
  // Determination sens roues gauches
  if (VitesseGauche > 0) // roues gauche en avant
  {
    digitalWrite(IN3, LOW); // sens roues <-Gauches
    analogWrite(IN2, VitesseGauche); // roues Gauche
  } else   { // si sens arriere
    digitalWrite(IN3, HIGH); // sens roues <-Gauches
    analogWrite(IN2, abs(VitesseGauche));
  }

  // Determination sens roues droites
  if (VitesseDroite > 0)
  {
    digitalWrite(IN4, LOW); // sens roues -> Droites
    analogWrite(IN1, VitesseDroite); //  vitesse roues Droites
  } else   { // si sens arriere
    digitalWrite(IN4, HIGH); // sens roues -> Droites
    analogWrite(IN1, abs(VitesseDroite)); // vitesse roues Droites
  }
}

void Stop()
{
  VitesseGauche = 0;
  VitesseDroite = 0;
}

void VitessePlus()
{
  VitesseGauche += 10;
  VitesseDroite += 10;
  // Maxi
  if (VitesseGauche > 240) VitesseGauche = 240;
  if (VitesseDroite > 240) VitesseDroite = 240;
  // Maxi recule
  if (VitesseGauche < -240) VitesseGauche = -240;
  if (VitesseDroite < -240) VitesseDroite = -240;
  // Mini vitesse Gauche
  //if ((VitesseGauche < VitesseMin) && ( VitesseGauche > VitesseMinRecule ) && (VitesseGauche > 0)) VitesseGauche = VitesseMin;
  //if ((VitesseGauche < VitesseMin) && ( VitesseGauche > VitesseMinRecule ) && (VitesseGauche < 0)) VitesseGauche = VitesseMinRecule;
  // mini vitesse droit
  //if ((VitesseDroite < VitesseMin) && ( VitesseDroite > VitesseMinRecule ) && (VitesseDroite > 0)) VitesseDroite = VitesseMin;
  // if ((VitesseDroite < VitesseMin) && ( VitesseDroite > VitesseMinRecule ) && (VitesseDroite < 0)) VitesseDroite = VitesseMinRecule;
}

void VitesseMoins()
{
  VitesseGauche -= 10;
  VitesseDroite -= 10;
}

void VirageGauche()
{
  VitesseGauche = -60;
  VitesseDroite = 60;
}

void VirageDroite()
{
  VitesseGauche = 60;
  VitesseDroite = -60;
}
void ToutDroit()
{
  VitesseGauche = 60;
  VitesseDroite = 60;
}

void Recule()
{
  VitesseGauche = -60;
  VitesseDroite = -60;
}


void AvanceProg()
{
  Serial.println("Avance Progressive");
  lcd.setCursor(5, 0); lcd.print("Prog   ");
  int VitG = VitesseGauche;
	int VitD = VitesseDroite;
	VitesseGauche = VitesseMin;
	VitesseDroite = VitesseMin;
	Avance();
	while((VitG >= VitesseGauche)|| (VitD >= VitesseDroite))
	{
	if(VitG > VitesseGauche) VitesseGauche +=10;
	if(VitD > VitesseDroite) VitesseDroite +=10;
	Avance();
	delay(200); // delai mecanique
	}
}

