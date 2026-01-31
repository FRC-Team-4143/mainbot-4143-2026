package frc.robot.lib2026;

import edu.wpi.first.wpilibj.DriverStation;

public class HubActiveMonitor {
    public String gameData = DriverStation.getGameSpecificMessage();
    public boolean hubActive = false;
    {

        if( DriverStation.isAutonomous() == true){
                hubActive = true;
        }



    if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      break;
    case 'R' :
      //Red case code
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}
}
public void 
if(flipRegions == true) {

}







}
