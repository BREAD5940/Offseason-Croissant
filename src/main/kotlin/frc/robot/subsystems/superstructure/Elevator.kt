package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kElevatorRange
import frc.robot.Ports
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts
import frc.robot.Ports.SuperStructurePorts.ElevatorPorts.MASTER_INVERTED
import kotlin.math.PI
import kotlin.math.withSign
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports

/**
 * The (singleton) [ConcurrentFalconJoint] elevator of Croissant.
 */
object Elevator : ConcurrentFalconJoint<Meter, FalconSRX<Meter>>() {

    // kv and ka calculation
    private const val G: Double = 14.67 // output over input
    private const val R = 0.114 // ohms?
    private const val radius = 1.5 * kInchToMeter / 2.0 // radius, meters
    private const val m = 35.0 // kilogram
    const val motorCount = 4.0
    private const val internalKv = 164.3 // volts per meter per sec
    private const val internalKt = 0.0212 // volts per meter per sec squared
    private const val metersKa = (R * radius * m) / (G * internalKt)
    private const val metersKv = (G * G * internalKt * metersKa) / (R * radius * radius * m * internalKv)

//    private val lowGearTransmissionHAB = DCMotorTransmission(
//            1 / (metersKv * 1 / (2.0 / radius)), // 603.2 /* rad per sec free */ / R / 12.0, // 42:1 gearing
//            radius * radius * m / (2.0 * metersKa / (2.0 / radius)),
//            0.0 // totally a guess
//    )

    /**
     * If the elevator should be in low gear right now.
     * On change, the elevator will shift the shifter and set either position closed loop or motion magic gains.
     */
    var wantsLowGear by Delegates.observable(false) {
        _, _, wantsLow ->
        shifter.state = if (wantsLow) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
        if (wantsLow) setLowSpeedPositionGains() else setMotionMagicMode()
    }

    private fun setLowSpeedPositionGains() {
        motor.useMotionProfileForPosition = false
        motor.setClosedLoopGains(0.15, 0.0)
    }

    private val shifter = FalconDoubleSolenoid(2, 3, Ports.kPCMID).apply {
        state = FalconSolenoid.State.Reverse
    }

    override val motor = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master: FalconSRX<Meter> = FalconSRX(ElevatorPorts.TALON_PORTS[0],
                ElevatorPorts.LENGTH_MODEL)

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(ElevatorPorts.TALON_PORTS[1], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[2], DefaultNativeUnitModel),
                FalconSRX(ElevatorPorts.TALON_PORTS[3], DefaultNativeUnitModel))

        init {
            master.outputInverted = MASTER_INVERTED
            master.voltageCompSaturation = 12.volt
            master.talonSRX.configSelectedFeedbackSensor(ElevatorPorts.SENSOR)
            master.talonSRX.setSensorPhase(ElevatorPorts.MASTER_SENSOR_PHASE)

            followers.forEachIndexed {
                index, followerMotor ->
                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(ElevatorPorts.FOLLOWER_INVERSION[index])
                master.voltageCompSaturation = 12.volt
            }

            // Config master settings and stuff
            master.talonSRX.configForwardSoftLimitThreshold(58500)
            master.talonSRX.configReverseSoftLimitThreshold(1300)
            master.talonSRX.configForwardSoftLimitEnable(true)
            master.talonSRX.configReverseSoftLimitEnable(false)
            master.talonSRX.configPeakOutputForward(1.0)
            master.talonSRX.configPeakOutputReverse(-1.0)
            master.talonSRX.selectProfileSlot(0, 0)
            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)

//            master.talonSRX.enableVoltageCompensation(true)
            master.voltageCompSaturation = 12.volt

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() =
                setHighSpeedMotionMagicGains()

        override var softLimitForward: SIUnit<Meter>
            get() = master.softLimitForward
            set(value) {
                master.softLimitForward = value
            }

        override var softLimitReverse: SIUnit<Meter>
            get() = master.softLimitReverse
            set(value) {
                master.softLimitReverse = value
            }

        /**
         * Configure the master talon for motion magic control
         */
        fun setHighSpeedMotionMagicGains() {
            // TODO also wrap the solenoid boi for the shifter?

            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((5500.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(8000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)

            master.setClosedLoopGains(
                    0.45 * 1.2, 4.0, ff = 0.3
            )
        }
    }

    /**
     * Configure the motor for position closed loop and
     * disable motion profile
     */
    fun setPositionMode() = motor.run {
        setClosedLoopGains(0.17, 0.0, 0.0)
        useMotionProfileForPosition = false
    }

    fun setClimbMode() = motor.run {
        setClosedLoopGains(0.375, 0.0, 0.0)
        useMotionProfileForPosition = false
    }

    fun setClimbVelocityMode() = motor.run {
        setClosedLoopGains(0.9, 0.0, 0.0)
        useMotionProfileForPosition = false
    }

    const val reduction = 14.66
    fun setClimbProfile(position: SIUnit<Meter>, velocity: SIUnit<Velocity<Meter>>) {
        // meters per second div meters per rotation is rotations per second
        val rotPerSec = velocity.value / (PI * 1.5.inches.inMeters())
        val radPerSec = rotPerSec * PI * 2

        val torque = 35.0 /* kg */ * 9.8 /* g */ * 0.75.inches.inMeters()

        val stallTorque = reduction * 0.71 * 4
        val freeYeet = 1961 /* rad per sec */ / reduction
        var voltage = torque / stallTorque + radPerSec / freeYeet
        voltage = voltage.withSign(-1)

        println("elevator voltage is $voltage")

//        wantedState = WantedState.Position(position, voltage.volt, over)
    }

    /**
     * Configure the motor for motion profile closed loop
     * and enable the use of motion profile
     */
    fun setMotionMagicMode() = motor.run {
        setClosedLoopGains()
        useMotionProfileForPosition = true
    }

    // The elevator's limit switch
    private val innerStageMinLimitSwitch = DigitalInput(0)
    val limitSwitchTriggered: Boolean get() = !innerStageMinLimitSwitch.get()

    init { motor.encoder.position }

    // Set the elevator height to a sane-ish number by default
    override fun lateInit() {
        motor.encoder.resetPosition(35.0.inches)
        wantsLowGear = false
    }

    @Suppress("UNCHECKED_CAST")
    override fun customizeWantedState(wantedState: WantedState): WantedState =
            when (wantedState) {
                /** add the [wantedState] and [elevatorOffset] to get an offset total and then bound to the [kElevatorRange] */
                is WantedState.Position<*> -> { ((wantedState as WantedState.Position<Meter>)).coerceIn(kElevatorRange) }
                else -> wantedState
            }

    // The current state of the elevator
    override val currentState get() =
        motor.currentState

    /** Calculate the arbitrary feed forward given a [currentState] */
    override fun calculateFeedForward(currentState: MultiMotorTransmission.State<Meter>) =
            if (currentState.position > 33.0.inches) 1.2.volts else (-0.72).volts // volts
}
