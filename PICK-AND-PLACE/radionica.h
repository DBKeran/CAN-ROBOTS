
void LineRobot::radionica() {

  if (setup()) {
    armBase();
  }
  delayMs(2000); 
  armUp();
  delayMs(2000);
  armCatchReady(); 
  delayMs(2000);
  armBase();            

}
