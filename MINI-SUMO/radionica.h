
void MiniSumo::radionica() {
  char simbol[10 + sizeof(char)];
  static int senzor;              // Pohrana vrijednosti čitanja senzora - lokalna varijabla

  if (setup()) {

  }

  senzor = digitalRead(16);       // 0 - CRNO ----- 1 - BIJELO

  sprintf(simbol, "%d", senzor);  // Ispis na mrm-8x8 display
  display(simbol);                //

  if (senzor == false) {
    go(100, 40);
  }
  else{
    stop();
  }

}
