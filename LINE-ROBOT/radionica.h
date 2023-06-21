
void LineRobot::radionica() {

  if (setup()) {

  }              

  if (lineAny()) {
    go(-60, -60);
    delayMs(150);
  }
  else{
    go(30, 30);
  }

}
