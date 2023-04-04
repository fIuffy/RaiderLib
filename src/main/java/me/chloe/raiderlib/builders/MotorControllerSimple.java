package me.chloe.raiderlib.builders;

import me.chloe.raiderlib.motor.CommonControllers;
import me.chloe.raiderlib.motor.MotorSimple;
import me.chloe.raiderlib.motor.SPARKMotorSimple;
import me.chloe.raiderlib.motor.TFXMotorSimple;
import me.chloe.raiderlib.motor.TSRXMotorSimple;
import me.chloe.raiderlib.motor.VSPXMotorSimple;

/**
 * Makes handling of motors through differently branded controllers easier.
 * Allows for PID control.
 */
public class MotorControllerSimple {
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
     * @param controller CommonController enum (Which controller are we using).
     * @param id ID on the CANbus.
     * @param brushless Whether motor is brushless or not.
     * @param exportFileName PIDExport file name.
     * @param minDutyCycle Minimum ControlPercent or Velocity input.
     * @param velocityControl Whether to use velocity or not for PIDExports.
     * @param maxOut Maximum ControlPercent or Velocity input.
     * @param useAbsolute Whether we are attaching an AbsoluteEncoder or not.
     */
    public MotorControllerSimple(CommonControllers controller, int id, boolean brushless, String exportFileName, double minDutyCycle, boolean velocityControl, double maxOut, boolean useAbsolute) {
        this.id = id;
        this.brushless = brushless;
        this.exportFileName = exportFileName;
        this.minDutyCycle = minDutyCycle;
        this.velocityControl = velocityControl;
        this.maxOut = maxOut;
        this.motor = getMotorSimple(controller, useAbsolute);
        if(this.motor != null) this.motor.useAbsoluteForFeedback(useAbsolute);
        if(this.motor != null) this.motor.setMotorStateController(this);
    }

    /**
     * Get the motor being used
     * @return MotorSimple
     */
    public MotorSimple getMotor() {
        return this.motor;
    }
    
    /**
     * Set the motor field in terms of what controller we are using
     * @param useAbsolute
     */
    private MotorSimple getMotorSimple(CommonControllers controller, boolean useAbsolute) {
        switch(controller) {
            case TALON_FX:
                return new TFXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
            case TALON_SRX:
                return new TSRXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
            case SPARK_MAX:
                return new SPARKMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
            case VICTOR_SPX:
                return new VSPXMotorSimple(this.id, this.brushless, this.exportFileName, this.minDutyCycle, this.velocityControl, this.maxOut);
        }
        return null;
    }
}
