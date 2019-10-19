@file:Suppress("RemoveRedundantQualifierName")

package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Controls
import frc.robot.auto.routines.withExit
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.ManualDriveCommand
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.hid.getY
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import java.awt.Color
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.withSign

object ClimbSubsystem : FalconSubsystem() {

    val stiltMotor: FalconMAX<Meter> = FalconMAX(9, CANSparkMaxLowLevel.MotorType.kBrushless,
            // the encoder is attached behind a 1:9 versaplanetary and a 1:2 pulley thing
            NativeUnitLengthModel(
                    1.nativeUnits * 7.0 * 3.0 * 2.0,
                    1.5.inch / 2
            )
    ).apply {

        setPIDGains(1.0 * 7.0 / 3.0, 0.0)
        encoder.canEncoder.positionConversionFactor = -1.0
        encoder.resetPosition(kZero)
        canSparkMax.setSmartCurrentLimit(40, 40) // TODO check
        brakeMode = false
//        canSparkMax.burnFlash()
    }
    fun configureVelocityPIDMode() {
        stiltMotor.setPIDGains(0.007, 0.0)
    }

    val reduction = 42.0 / 1.0
    fun setClimbProfile(position: SIUnit<Meter>, velocity: SIUnit<Velocity<Meter>>, offset: SIUnit<Meter>) {
        // meters per second div meters per rotation is rotations per second
        val rotPerSec = velocity.value / (PI * 1.5.inch.meter)
        val radPerSec = rotPerSec * PI * 2

        val torque = 35.0 /* kg */ * 9.8 /* g */ * 0.75.inch.meter

        val stallTorque = reduction * 2.6
        val freeYeet = 594.4 /* rad per sec */ / reduction
        var voltage = torque / stallTorque + radPerSec / freeYeet
        voltage = voltage.withSign(-1)
        println("stilt voltage $voltage")

        stiltMotor.setPosition(position + offset, voltage.volt)
    }

    private val kZero = 25.inch
    fun zero() = stiltMotor.encoder.resetPosition(kZero)

    val prepMove = sequential {
        +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
        +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 93.degree) // TODO check preset
        val move = SuperstructurePlanner.everythingMoveTo(25.inch - 10.inch /* hab 2 */, (-5).degree, 93.degree) // TODO check preset
        +parallel {
            +move
            +(RunCommand(Runnable { intakeWheels.setDutyCycle(0.3) }, ClimbSubsystem).withExit { !move.isScheduled }.whenFinished { intakeWheels.setNeutral() })
        }
    }

    val hab3prepMove = sequential {
        +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
        +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 93.degree) // TODO check preset
        val move = SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 93.degree) // TODO check preset
        +parallel {
            +move
            +(RunCommand(Runnable { intakeWheels.setDutyCycle(0.3) }, ClimbSubsystem).withExit { !move.isScheduled }.whenFinished { intakeWheels.setNeutral() })
        }
    }

    override fun periodic() {
        SmartDashboard.putNumber("Stilt pos", ClimbSubsystem.stiltMotor.encoder.position.inch)
        SmartDashboard.putNumber("Stilt amps", ClimbSubsystem.stiltMotor.drawnCurrent.amp)
//        stiltMotor.setDutyCycle(0.0, 1.volt) // negative voltage is pushing us down
    }

    val hab2ClimbCommand = object : FalconCommand(ClimbSubsystem,
            Elevator, Proximal, Wrist, Superstructure) {

        val targetHeight = 13.inch
//        val yeetForwardSource by lazy { { Controls.operatorJoy.getRawAxis(1) } }
//        val endCommand by lazy { { Controls.operatorJoy.getRawButton(12) } }
//        val yeetForwardSource by lazy { { Controls.driverControllerLowLevel.getY(GenericHID.Hand.kLeft) } }
//        val yeetForwardSource by lazy { ManualDriveCommand.speedSource }

        val yeetForwardSource by lazy {
            { val toRet = Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kRight) - Controls
                    .driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft)
//                    println("speed $toRet")
                val compensated = toRet * -1.0
                val deadbanded = ((compensated.absoluteValue - ManualDriveCommand.kDeadband / 1.8) / (1.0 - ManualDriveCommand.kDeadband / 1.8)) * compensated.sign

                // throttle curve should be between [-1. 1] or so.
                ManualDriveCommand.kisscalc(deadbanded, 0.77, 0.0, 0.00116)
            }
        }

        val endCommand by lazy { { Controls.driverControllerLowLevel.getRawButton(1) } }
        val elevatorSource by lazy { { Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft) } }

        var startTime = 0.0
//        val yeetUpVelocity = (-6).inch // meters per second
        val hab2Offset = 13.inch
        var stiltsInPosition = false
        var elevatorInPosition = false
        var voltageArray: ArrayList<Double> = arrayListOf()

        override fun initialize() {
            stiltMotor.controller.setOutputRange(-1.0, 1.0)
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 0.3)
            Proximal.wantedState = WantedState.Position((-15).degree)
            Elevator.wantsLowGear = true
//            Elevator.setClimbVelocityMode()
            Proximal.setClimbPositionMode()
            Wrist.wantedState = WantedState.Position(88.degree)
            Controls.isClimbing = true
            startTime = Timer.getFPGATimestamp()
            LEDs.wantedState = LEDs.State.Blink(0.15.second, Color(130, 24, 30))
        }
        override fun execute() {
            if (Elevator.currentState.position < 18.inch + hab2Offset && Elevator.currentState.position > 13.inch + hab2Offset) Proximal.wantedState = WantedState.Position((-37).degree)
            else if (Elevator.currentState.position < 12.5.inch + hab2Offset) {
                Proximal.wantedState = WantedState.Position((-45).degree)
                Wrist.wantedState = WantedState.Position(86.degree)
            }
//            }
            if (!elevatorInPosition) elevatorInPosition = Elevator.currentState.position < 13.inch + hab2Offset
            if (!stiltsInPosition) stiltsInPosition = stiltMotor.encoder.position < 13.inch - 4.5.inch + hab2Offset

            println("elevator current ${Elevator.motor.drawnCurrent}, stilt current ${stiltMotor.drawnCurrent}")

//            if(elevatorInPosition || Elevator.currentState.position < 12.5.inch + hab2Offset) Elevator.wantedState = WantedState.Position(12.inch + hab2Offset) else {
// //                Elevator.setClimbProfile(targetState, yeetUpVelocity.velocity)
// //                Elevator.wantedState = WantedState.Velocity(yeetUpVelocity.velocity)
// //                voltageArray.add(Elevator.motor.voltageOutput.value)
// //                Elevator.wantedState = WantedState.Position(12.inch) // just keep yeeting
//                Elevator.wantedState = WantedState.Voltage((-0.2 - elevatorSource()).volt * 12)
//            }
            println("average elevator voltage ${voltageArray.average()}")
//            if(stiltsInPosition || stiltMotor.encoder.position < 12.inch - 4.inch) stiltMotor.setPosition(12.inch - 4.5.inch) else {
//                setClimbProfile(targetState, yeetUpVelocity.velocity, (-4.5).inch)
//            }
            stiltMotor.setPosition(12.inch - 4.5.inch + hab2Offset - 1.5.inch)
            Elevator.wantedState = WantedState.Position(12.inch - 0.5.inch)

            var s3nd = yeetForwardSource() * -1.0
            if (s3nd < -0.1) s3nd = -0.2

            val wantedIntake = if (Timer.getFPGATimestamp() < startTime + 2.0) 1.0 else if (s3nd > 0.0) s3nd + 0.35 else s3nd

            intakeWheels.setDutyCycle(wantedIntake)
            DriveSubsystem.lowGear = true
            DriveSubsystem.tankDrive(s3nd / 5.0, s3nd / 5.0)

            println("hab wheel amps ${intakeWheels.drawnCurrent.amp}, stilt amps ${stiltMotor.drawnCurrent.amp}")
        }

        override fun isFinished() = endCommand()

        override fun end(interrupted: Boolean) {
            Elevator.wantsLowGear = false
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            Proximal.setMotionMagicMode()
            intakeWheels.setNeutral()
            stiltMotor.controller.setOutputRange(-0.2, 0.2)
            stiltMotor.setPosition(25.inch)
            Controls.isClimbing = false

            if (stiltsInPosition && elevatorInPosition) {
                Elevator.wantedState = WantedState.Position(24.inch)
                Proximal.wantedState = WantedState.Position((-20).degree)
                LEDs.wantedState = LEDs.State.Solid(Color.GREEN)
                GlobalScope.launch {
                    delay(1500)
                    LEDs.wantedState = LEDs.State.Off
                    delay(250)
                    LEDs.wantedState = LEDs.State.Default
                }
            }

//            Superstructure.kMatchStartToStowed.schedule()
        }
    }

    val hab3ClimbCommand = object : FalconCommand(ClimbSubsystem,
            Elevator, Proximal, Wrist, Superstructure) {

        val targetHeight = 13.inch
        //        val yeetForwardSource by lazy { { Controls.operatorJoy.getRawAxis(1) } }
//        val endCommand by lazy { { Controls.operatorJoy.getRawButton(12) } }
//        val yeetForwardSource by lazy { { Controls.driverControllerLowLevel.getY(GenericHID.Hand.kLeft) } }
        val endCommand by lazy { { Controls.driverControllerLowLevel.getRawButton(1) } }
        val elevatorSource by lazy { { Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft) } }

        val yeetForwardSource by lazy {
            { val toRet = Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kRight) - Controls
                    .driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft)
//                    println("speed $toRet")
                val compensated = toRet * -1.0
                val deadbanded = ((compensated.absoluteValue - ManualDriveCommand.kDeadband / 1.8) / (1.0 - ManualDriveCommand.kDeadband / 1.8)) * compensated.sign

                // throttle curve should be between [-1. 1] or so.
                ManualDriveCommand.kisscalc(deadbanded, 0.77, 0.0, 0.00116)
            }
        }

        var startTime = 0.0
        //        val yeetUpVelocity = (-6).inch // meters per second
//        val hab2Offset = 13.inch
        var stiltsInPosition = false
        var elevatorInPosition = false
        var voltageArray: ArrayList<Double> = arrayListOf()

        override fun initialize() {
            stiltMotor.controller.setOutputRange(-1.0, 1.0)
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 0.25)
            Proximal.wantedState = WantedState.Position((-15).degree)
            Elevator.wantsLowGear = true
//            Elevator.setClimbVelocityMode()
            Proximal.setClimbPositionMode()
            Wrist.wantedState = WantedState.Position(88.degree)
            Controls.isClimbing = true
            startTime = Timer.getFPGATimestamp()
            LEDs.wantedState = LEDs.State.Blink(0.15.second, Color(130, 24, 30))
        }
        override fun execute() {
            if (Elevator.currentState.position < 18.inch && Elevator.currentState.position > 13.inch) Proximal.wantedState = WantedState.Position((-37).degree)
            else if (Elevator.currentState.position < 12.5.inch) {
                Proximal.wantedState = WantedState.Position((-47).degree)
                Wrist.wantedState = WantedState.Position(86.degree)
            }
//            }
            if (!elevatorInPosition) elevatorInPosition = Elevator.currentState.position < 13.inch
            if (!stiltsInPosition) stiltsInPosition = stiltMotor.encoder.position < 13.inch - 4.5.inch

            println("elevator current ${Elevator.motor.drawnCurrent}, stilt current ${stiltMotor.drawnCurrent}")
            println("average elevator voltage ${voltageArray.average()}")
            stiltMotor.setPosition(12.inch - 4.5.inch - 1.inch)
            Elevator.wantedState = WantedState.Position(12.inch)

            var s3nd = yeetForwardSource() * -1.0
            if (s3nd < -0.1) s3nd = -0.2

            val wantedIntake = if (Timer.getFPGATimestamp() < startTime + 3.5) 1.0 else if (s3nd > 0.0) s3nd + 0.35 else s3nd

            intakeWheels.setDutyCycle(wantedIntake)
            DriveSubsystem.lowGear = true
            DriveSubsystem.tankDrive(s3nd / 5.0, s3nd / 5.0)

            println("hab wheel amps ${intakeWheels.drawnCurrent.amp}, stilt amps ${stiltMotor.drawnCurrent.amp}")
        }

        override fun isFinished() = endCommand()

        override fun end(interrupted: Boolean) {
            Elevator.wantsLowGear = false
            Elevator.motor.master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            Proximal.setMotionMagicMode()
            intakeWheels.setNeutral()
            stiltMotor.controller.setOutputRange(-0.2, 0.2)
            stiltMotor.setPosition(25.inch)
            Controls.isClimbing = false

            if (stiltsInPosition && elevatorInPosition) {
                Elevator.wantedState = WantedState.Position(24.inch)
                Proximal.wantedState = WantedState.Position((-20).degree)
                LEDs.wantedState = LEDs.State.Solid(Color.GREEN)
                GlobalScope.launch {
                    delay(1500)
                    LEDs.wantedState = LEDs.State.Off
                    delay(250)
                    LEDs.wantedState = LEDs.State.Default
                }
            }

//            Superstructure.kMatchStartToStowed.schedule()
        }
    }

    override fun lateInit() {
        SmartDashboard.putData("test move", sequential {
            +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(35.inch, (-5).degree, 80.degree) // TODO check preset
            +SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 95.degree) // TODO check preset
        })
        SmartDashboard.putData("hab 3", hab3prepMove)

//        SmartDashboard.putData("closed loop climb", fullS3ndClimbCommand)
//
//        SmartDashboard.putData("straight out", Superstructure.kHatchMid)
//        SmartDashboard.putData("SketchyTest", SketchyTest())
    }

    val intakeWheels = FalconSRX(45, DefaultNativeUnitModel).apply {
        outputInverted = true
        configCurrentLimit(true, FalconSRX.CurrentLimitConfig(20.amp, 0.5.second, 15.amp))
    }

    private val currentStateMutex = Object()
    val currentState = MultiMotorTransmission.State(0.meter)
        get() = synchronized(currentStateMutex) { field }

//    var wantedState: WantedState = WantedState.Nothing
//        get() = synchronized(wantedStateMutex) { field }
//        set(newValue) = synchronized(wantedStateMutex) { field = newValue }

//    /**
//     * Determine if the joint is within the [tolerance] of the current wantedState.
//     * If the wantedState isn't [WantedState.Position<*>], return false.
//     */
//    fun isWithTolerance(tolerance: SIUnit<Meter>): Boolean {
//        val state = wantedState as? WantedState.Position<*> ?: return false // smart cast state, return false if it's not Position
//
//        return abs(state.targetPosition.value - currentState.position.value) < tolerance.value
//    }
}

fun <K : SIKey> FalconMAX<K>.setPIDGains(p: Double, d: Double, ff: Double = 0.0) {
    controller.p = p
    controller.d = d
    controller.ff = ff
    controller.i = 0.0
}