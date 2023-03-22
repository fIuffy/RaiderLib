package frc.robot.raiderlib.builders;

import java.util.ArrayList;

import frc.robot.raiderlib.motor.struct.MotorToControllerSimple;


public class MovingPart {
    private final String pseudoName, desc;
    private ArrayList<MotorToControllerSimple> motors;
     /**
     * Constructor for creating a basic Moving Part.
     * @param psuedoName - Readable name of part.
     * @param desc - Readable description of part.
     */
    public MovingPart(String pseudoName, 
                        String desc) {
        this.pseudoName = pseudoName;
        this.desc = desc;
        init();
    }


    /**
     * Called when the MovingPart constructor is finished
     */
    public void init() {

    }

    /**
     * Called when Part is added to the RaiderLib Subsystem
     */
    public void onAddToList() {

    }

    /**
     * The super constructor is called specifically for the implementation of PID Exports.
     */
    public void periodic() {
        for(MotorToControllerSimple controller : motors) {
            if(controller.getMotor().exporting) {
                controller.getMotor().exportPeriodic();
            }
        }
    }

    public MovingPart addMotor(MotorToControllerSimple motor) { 
        motors.add(motor);
        return this;
    }

    public String getPseudoName() {
        return this.pseudoName;
    }

    public String getDesc() {
        return this.desc;
    }

    public ArrayList<MotorToControllerSimple> getMotors() {
        return this.motors;
    }
}
