package me.chloe.raiderlib.builders;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import me.chloe.raiderlib.motor.MotorSimple.MotorSimpleState;


/**
 * Constructor to make using PID control with multiple motors a lot more basic and standard.
 * A MovingPart instance allows for the ability to create static motor states that are all grouped under
 * a specific part.
 */
public class MovingPart {
    private final String pseudoName, desc;
    private HashMap<Integer, MotorControllerSimple> motors = new HashMap<>();
    private ArrayList<MovingPartState> movingPartStates = new ArrayList<>();

    private MovingPartState currentState;

     /**
     * Constructor for creating a basic Moving Part.
     * @param pseudoName - Readable name of part.
     * @param desc - Readable description of part.
     */
    public MovingPart(String pseudoName, 
                        String desc) {
        this.pseudoName = pseudoName;
        this.desc = desc;
        this.currentState = new MovingPartState("periodic");
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
     * Called on Teleop start.
     */
    public void onTeleopInit() {

    }

    /**
     * Called on Autonomous start.
     */
    public void onAutoInit() {

    }

    /**
     * The super constructor is called for the implementation of PID Exports and MovingPart states.
     */
    public void periodic() {
        MotorSimpleState[] states = new MotorSimpleState[motors.size()];
        int x = 0;
        for(Map.Entry<Integer, MotorControllerSimple> set : motors.entrySet()) {
            if(set.getValue().getMotor().exporting) {
                set.getValue().getMotor().exportPeriodic();
            }
            MotorSimpleState actualState = set.getValue().getMotor().getState();
            set.getValue().getMotor().updateState(set.getValue().getMotor().getMotorPositionConverted(), set.getValue().getMotor().getMotorVelocityConverted());
            states[x] = actualState;
            x++;
        }
        this.currentState.setCurrentMotorStates(states);
    }

    /**
     * Add a motor to the part's list of motors
     * @param motor MotorControllerSimple
     * @return this (MovingPart) to be used as a builder.
     */
    public MovingPart addMotor(MotorControllerSimple motor) { 
        motors.put(motor.getMotor().getCANID(), motor);
        return this;
    }

    /**
     * Add an Array of motors to the part's list of motors
     * @param ms Array of MotorControllerSimple separated by commas
     * @return this (MovingPart) to be used as a builder.
     */
    public MovingPart addMotors(MotorControllerSimple... ms) { 
        for(MotorControllerSimple m : ms) {
            motors.put(m.getMotor().getCANID(), m);
        }
        
        return this;
    }

    /**
     * Add states in the builder
     * @param stateName Name of state
     * @param motorSimpleStates Array of MotorSimpleStates separated by commas
     * @return this (MovingPart) to be used as builder.
     */
    public MovingPart addState(String stateName, MotorSimpleState... motorSimpleStates) {
        movingPartStates.add(new MovingPartState(stateName, motorSimpleStates));
        return this;
    }


    /**
     * Change the MovingPart to a state created in the builder.
     * @param stateName Desired state's name
     */
    public void setState(String stateName) {
        MovingPartState wantedState = null;
        for(MovingPartState state : movingPartStates) {
            if(state.getStateName().equalsIgnoreCase(stateName)) {
                wantedState = state;
            }
        }
        if(wantedState != null) {
            for(MotorSimpleState motorState : wantedState.motorSimpleStates) {
                double targetPos = motorState.getStatePosition();
                double targetSpeed = motorState.getStateSpeed();
                MotorControllerSimple targetMotor = motors.get(motorState.getController().getMotor().getCANID());
                if(targetPos != -1) {
                    targetMotor.getMotor().setMotorPositional(targetPos);
                }
                if(targetSpeed != -1) {
                    if(motorState.velocityControl) {
                        targetMotor.getMotor().setMotorVelocity(targetSpeed);
                    } else {
                        targetMotor.getMotor().setMotorControlPercent(targetSpeed);
                    }
                }
            }
        }
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

    public class MovingPartState {
        private final String stateName;
        private ArrayList<MotorSimpleState> motorSimpleStates;

        /**
         * Create a MovingPartState which is basically just a way to make it so that
         * specific motor positions and velocities correlate to a specific State name
         * @param stateName String
         * @param motorSimpleStates MotorSimpleState list separated by commas
         */
        public MovingPartState(String stateName, MotorSimpleState... motorSimpleStates) {
            this.stateName = stateName;
            this.motorSimpleStates = new ArrayList<>(Arrays.asList(motorSimpleStates));
        }

        /**
         * Get state name
         * @return String
         */
        public String getStateName() {
            return this.stateName;
        }

        /**
         * Get list of motor states
         * @return ArrayList of MotorSimpleStates
         */
        public ArrayList<MotorSimpleState> getMotorStates() {
            return this.motorSimpleStates;
        }

        /**
         * Change the current list of states
         * @param states MotorSimpleState[]
         */
        public void setCurrentMotorStates(MotorSimpleState[] states) {
            this.motorSimpleStates = new ArrayList<>(Arrays.asList(states));
        }
    }
}
