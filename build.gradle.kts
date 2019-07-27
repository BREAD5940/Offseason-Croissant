import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.frc.FRCJavaArtifact
import edu.wpi.first.gradlerio.frc.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    id("edu.wpi.first.GradleRIO") version "2019.4.1"
    id("org.jetbrains.kotlin.jvm") version "1.3.11"
    id("idea")
    id("org.jlleitschuh.gradle.ktlint") version "8.0.0"
}

val roborioTargetName = "roborio"

val kMainRobotClass = "frc.robot.RobotKt"

deploy {
    targets {
        // Add the RoboRIO as a target
        target(roborioTargetName, RoboRIO::class.java, closureOf<RoboRIO> {
            team = 5940
        })
    }
    artifacts {
        // Send the JAR to the RoboRIO
        artifact("frcJava", FRCJavaArtifact::class.java, closureOf<FRCJavaArtifact> {
            targets.add(roborioTargetName)
            debug = frc.getDebugOrDefault(false)
        })
    }
}

repositories {
    jcenter()
    maven { setUrl("http://dl.bintray.com/kyonifer/maven") }
    maven { setUrl("https://jitpack.io") }
    maven { setUrl("http://dev.imjac.in/maven") }
    mavenLocal()
}

dependencies {
    // Kotlin Standard Library and Coroutines
    compile(kotlin("stdlib"))
    compile("org.jetbrains.kotlinx", "kotlinx-coroutines-core", "1.1.0")

    // FalconLibrary
//    compile("com.github.mcm001:falconlibrary:c5019e366c")
//    compile("com.github.mcm001:FalconLibrary:0.0.1")
    compile("com.github.5190GreenHopeRobotics:FalconLibrary:8c33535a46")

    compile("com.fazecast:jSerialComm:2.4.1") // jserialcomm for jevois
    compile("com.github.salomonbrys.kotson", "kotson", "2.5.0") // gson

    implementation("org.mockito:mockito-core:2.23.+")

    // WPILib and Vendors
    wpi.deps.wpilib().forEach { compile(it) }
    wpi.deps.vendor.java().forEach { compile(it) }
    wpi.deps.vendor.jni(NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.vendor.jni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

    // compile("openrio.powerup", "MatchData", "2018.01.07")

    // Gson
    compile("com.github.salomonbrys.kotson", "kotson", "2.5.0")

//    compile("com.github.mcm001:pantrycommon:2d570ea774")

    compile("com.github.Oblarg:Oblog:2.8.1")

    // XChart for Simulations and Tests
    compile("org.knowm.xchart", "xchart", "3.2.2")

    // Unit Testing
    testCompile("junit", "junit", "4.12")
}

tasks.jar {
    doFirst {
        from(configurations.compile.get().map {
            if (it.isDirectory) it else zipTree(it)
        })
        manifest(GradleRIOPlugin.javaManifest(kMainRobotClass))
    }
}

tasks {
    withType<Wrapper>().configureEach {
        gradleVersion = "5.0"
    }
    withType<KotlinCompile>().configureEach {
        kotlinOptions {
            jvmTarget = "1.8"
            freeCompilerArgs += "-Xjvm-default=compatibility"
        }
    }
}