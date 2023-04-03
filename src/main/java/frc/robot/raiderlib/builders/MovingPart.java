package frc.robot.raiderlib.builders;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.raiderlib.motor.struct.MotorControllerSimple;
import frc.robot.raiderlib.motor.struct.MotorSimple.MotorSimpleState;


public class MovingPart {
    private final String pseudoName, desc;
    private HashMap<Integer, MotorControllerSimple> motors = new HashMap<>();
    private ArrayList<MovingPartState> movingPartStates = new ArrayList<>();

    private MovingPartState currentState;

     /**
     * Constructor for creating a basic Moving Part.
     * @param psuedoName - Readable name of part.
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

    public void onTeleopInit() {

    }

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
     * @return this (MovingPart)
     */
    public MovingPart addMotor(MotorControllerSimple motor) { 
        motors.put(motor.getMotor().getCANID(), motor);
        return this;
    }

    public MovingPart addState(MovingPartState state) {
        movingPartStates.add(state);
        return this;
    }


    public void changeToState(String stateName) {
        MovingPartState wantedState = null;
        for(MovingPartState state : movingPartStates) {
            if(state.getStateName().equalsIgnoreCase(stateName)) {
                wantedState = state;
            }
        }
        if(wantedState != null) {
            for(MotorSimpleState motorState : wantedState.motorSimpleStates) {
                MotorControllerSimple targetMotor = motors.get(motorState.getCANID());
                targetMotor.getMotor().setMotorPositional(motorState.getStatePosition());
                Commands.waitSeconds(0.5d);
                if(motorState.getStateVelocity() != 0.0d && Math.abs(targetMotor.getMotor().getMotorVelocity()) < 0.0375) targetMotor.getMotor().setMotorPositional(motorState.getStatePosition());
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

        public String getStateName() {
            return this.stateName;
        }

        public ArrayList<MotorSimpleState> getMotorStates() {
            return this.motorSimpleStates;
        }

        public void setCurrentMotorStates(MotorSimpleState[] states) {
            this.motorSimpleStates = new ArrayList<>(Arrays.asList(states));
        }
    }
}
