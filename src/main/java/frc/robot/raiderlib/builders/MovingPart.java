package frc.robot.raiderlib.builders;

import java.util.ArrayList;

import frc.robot.raiderlib.motor.struct.MotorControllerSimple;


public class MovingPart {
    private final String pseudoName, desc;
    private ArrayList<MotorControllerSimple> motors;
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
        for(MotorControllerSimple controller : motors) {
            if(controller.getMotor().exporting) {
                controller.getMotor().exportPeriodic();
            }
        }
    }

    /**
     * Add a motor to the part's list of motors
     * @param motor MotorControllerSimple
     * @return this (MovingPart)
     */
    public MovingPart addMotor(MotorControllerSimple motor) { 
        motors.add(motor);
        return this;
    }

    /**
     * 
     * @return English readable name of part
     */
    public String getPseudoName() {
        return this.pseudoName;
    }

    /**
     * 
     * @return Description of part
     */
    public String getDesc() {
        return this.desc;
    }

    /**
     * 
     * @return ArrayList of MotorControllers
     */
    public ArrayList<MotorControllerSimple> getMotors() {
        return this.motors;
    }
}
