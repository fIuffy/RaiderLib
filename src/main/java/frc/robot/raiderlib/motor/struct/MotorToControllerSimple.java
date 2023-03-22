package frc.robot.raiderlib.motor.struct;

import frc.robot.raiderlib.motor.SPARKMotorSimple;
import frc.robot.raiderlib.motor.TFXMotorSimple;
import frc.robot.raiderlib.motor.TSRXMotorSimple;
import frc.robot.raiderlib.motor.VSPXMotorSimple;

public class MotorToControllerSimple {
    private CommonControllers controller;
    private int id;
    private boolean brushless;
    private String exportFileName;
    private MotorSimple motor;
    private double minDutyCycle;
    private boolean velocityControl;
    private double maxOut;


    /**
     * MotorToControllerSimple constructor which makes creating Motors based on different controllers much easier.
     * Made specifically to teach newer 1518 programmers as well as make PID control structure a lot more standard.
     * @param controller - CommonController enum (Which controller are we using).
     * @param id - ID on the CANbus.
     * @param brushless - Whether motor is brushless or not.
     * @param exportFileName - PIDExport file name.
     * @param minDutyCycle - Minimum ControlPercent input.
     * @param velocityControl - Use velocity or not (Only really needed for things that drive wheels).
     * @param maxOut - Maximum ControlPercent input.
     * @param useAbsolute - Whether we are attaching an AbsoluteEncoder or not.
     */
    private MotorToControllerSimple(CommonControllers controller, int id, boolean brushless, String exportFileName, double minDutyCycle, boolean velocityControl, double maxOut, boolean useAbsolute) {
        this.controller = controller;
        this.id = id;
        this.brushless = brushless;
        this.exportFileName = exportFileName;
        this.minDutyCycle = minDutyCycle;
        this.velocityControl = velocityControl;
        this.maxOut = maxOut;
        createSimpleController(useAbsolute);
    }

    public MotorSimple getMotor() {
        return this.motor;
    }
    
    /**
     * Set the motor field in terms of what controller we are using
     * @param useAbsolute
     */
    private void createSimpleController(boolean useAbsolute) {
        switch(controller) {
            case TALON_FX:
                this.motor = new TFXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
                break;
            case TALON_SRX:
                this.motor = new TSRXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
                break;
            case SPARK_MAX:
                this.motor = new SPARKMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
                break;
            case VICTOR_SPX:
                this.motor = new VSPXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
                break;
        }
        this.motor.setFeedback(useAbsolute);
    }

    /**
     * Common Motor Controllers I have seen since joining Raider Robotics (2022-Present)
     */
    public enum CommonControllers {
        TALON_SRX,
        TALON_FX,
        SPARK_MAX,
        VICTOR_SPX;
    }
}
