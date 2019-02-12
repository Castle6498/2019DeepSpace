
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraVision{
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
public double getX(){
	return table.getEntry("tx").getDouble(0.0);
}

public double getY(){
	return table.getEntry("ty").getDouble(0.0);
}

public double getArea(){
	return table.getEntry("ta").getDouble(0.0);
}

//post to smart dashboard periodically

}