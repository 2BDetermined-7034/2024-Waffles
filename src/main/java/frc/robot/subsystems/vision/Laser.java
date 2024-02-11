package frc.robot.subsystems.vision;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan; //LaserCAN
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;
import frc.robot.Constants;

import java.io.ObjectInputFilter;
import java.util.Objects;
import java.util.stream.Collectors;

public class Laser extends SubsystemBase implements SubsystemLogging {

    private LaserCan lc;

    public Laser(){
        lc = new LaserCan(0); //takes in can_id
        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT); //Change to long for range up to 4m, short caps at 1.3m, but is more reliable ig.
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public int getRawDistance(){
        if (!(lc.getMeasurement() == null)) {
            return lc.getMeasurement().distance_mm;
        }else {
            return -1;
        }
    }

    private double previousPoint;
    public double getWeightedDistance() {
        previousPoint = previousPoint * 0.9 + getRawDistance() * 0.1; //Feel free to change weights :) -Henry
        return previousPoint;
    }

    public boolean noteIsPresent(double distance){
        return 0 <= distance && 100 >= distance;
    }


    @Override
    public void periodic() {
//		if (!(lc.getMeasurement() == null)) {
//			log("Distance (mm)", lc.getMeasurement().distance_mm);
//		}else {
//			log("No distance found", "null");
//		}
//		log("Raw Distance (MM):", getRawDistance());
//		log("Weighted Distance (MM):", getWeightedDistance(getRawDistance()));
    }

}
