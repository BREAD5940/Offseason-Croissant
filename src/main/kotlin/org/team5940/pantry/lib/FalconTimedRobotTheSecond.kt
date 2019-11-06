package org.team5940.pantry.lib

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.subsystems.EmergencyHandleable

abstract class FalconTimedRobotTheSecond {

    enum class Mode {
        NONE,
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    }

    var currentMode = Mode.NONE
        private set

    private val emergencyReadySystems = arrayListOf<EmergencyHandleable>()
    var emergencyActive = false
        protected set

    protected val wrappedValue = WpiTimedRobot()

    protected inner class WpiTimedRobot : TimedRobot() {

        private val kLanguage_Kotlin = 6

        init {
            HAL.report(FRCNetComm.tResourceType.kResourceType_Language, kLanguage_Kotlin)
        }

        override fun robotInit() {
            currentMode = FalconTimedRobotTheSecond.Mode.NONE
            this@FalconTimedRobotTheSecond.robotInit()
            FalconSubsystemHandler.lateInit()
            LiveWindow.disableAllTelemetry()
        }

        override fun autonomousInit() {
            currentMode = FalconTimedRobotTheSecond.Mode.AUTONOMOUS
            this@FalconTimedRobotTheSecond.autonomousInit()
            FalconSubsystemHandler.autoReset()
        }

        override fun teleopInit() {
            currentMode = FalconTimedRobotTheSecond.Mode.TELEOP
            this@FalconTimedRobotTheSecond.teleopInit()
            FalconSubsystemHandler.teleopReset()
        }

        override fun disabledInit() {
            currentMode = FalconTimedRobotTheSecond.Mode.DISABLED
            this@FalconTimedRobotTheSecond.disabledInit()
            FalconSubsystemHandler.setNeutral()
        }

        override fun testInit() {
            currentMode = FalconTimedRobotTheSecond.Mode.TEST
            this@FalconTimedRobotTheSecond.testInit()
        }

        override fun robotPeriodic() {
            this@FalconTimedRobotTheSecond.robotPeriodic()
            try { CommandScheduler.getInstance().run() } catch (e: Exception) { e.printStackTrace() }
        }

        override fun autonomousPeriodic() {
            this@FalconTimedRobotTheSecond.autonomousPeriodic()
        }

        override fun teleopPeriodic() {
            this@FalconTimedRobotTheSecond.teleopPeriodic()
        }

        override fun disabledPeriodic() {
            this@FalconTimedRobotTheSecond.disabledPeriodic()
        }
    }

    protected open fun robotInit() {}
    protected open fun autonomousInit() {}
    protected open fun teleopInit() {}
    protected open fun disabledInit() {}
    private fun testInit() {}

    protected open fun robotPeriodic() {}
    protected open fun autonomousPeriodic() {}
    protected open fun teleopPeriodic() {}
    protected open fun disabledPeriodic() {}

    open operator fun FalconSubsystem.unaryPlus() {
        FalconSubsystemHandler.add(this)
        if (this is EmergencyHandleable) {
            emergencyReadySystems.add(this)
        }
    }

    fun activateEmergency() {
        emergencyReadySystems.forEach { it.activateEmergency() }
        emergencyActive = true
    }

    fun recoverFromEmergency() {
        emergencyReadySystems.forEach { it.recoverFromEmergency() }
        emergencyActive = false
    }

    fun start() {
        RobotBase.startRobot { wrappedValue }
    }
}

internal object FalconSubsystemHandler {
    private val registeredSubsystems = arrayListOf<FalconSubsystem>()

    fun add(subsystem: FalconSubsystem) {
        registeredSubsystems.add(subsystem)
    }

    fun lateInit() = registeredSubsystems.forEach { it.lateInit() }
    fun autoReset() = registeredSubsystems.forEach { it.autoReset() }
    fun teleopReset() = registeredSubsystems.forEach { it.teleopReset() }
    fun setNeutral() = registeredSubsystems.forEach { it.setNeutral() }
}
