import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.frc.FRCJavaArtifact
import edu.wpi.first.gradlerio.frc.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import org.gradle.api.tasks.compile.JavaCompile

plugins {
    id("edu.wpi.first.GradleRIO") version "2019.4" +
            ".1"
    id("org.jetbrains.kotlin.jvm") version "1.3.50"
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
            jvmArgs = listOf(
                    "-Xmx20M",
                    "-XX:+UseG1GC" // ,
//                    "-Dcom.sun.management.jmxremote=true",
//                    "-Dcom.sun.management.jmxremote.port=1099",
//                    "-Dcom.sun.management.jmxremote.local.only=false",
//                    "-Dcom.sun.management.jmxremote.ssl=false",
//                    "-Dcom.sun.management.jmxremote.authenticate=false",
//                    "-Djava.rmi.server.hostname=10.59.40.2"
            )
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
    compile("org.jetbrains.kotlinx", "kotlinx-coroutines-core", "1.3.0-RC")

    // FalconLibrary
//    compile("com.github.5190GreenHopeRobotics:FalconLibrary:c1ba9286f9")

    compile("com.github.mcm001:falconlibrary:bccc56aa04")

    compile("com.fazecast:jSerialComm:2.4.1") // jserialcomm for jevois
    compile("com.github.salomonbrys.kotson", "kotson", "2.5.0") // gson

//    implementation("org.mockito:mockito-core:2.23.+")

    compile("org.ejml:ejml-simple:0.38")

    // WPILib and Vendors
    wpi.deps.wpilib().forEach { compile(it) }
    wpi.deps.vendor.java().forEach { compile(it) }
    wpi.deps.vendor.jni(NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.vendor.jni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

    compile("com.github.Oblarg:Oblog:2.8.1")

    // XChart for Simulations and Tests
    testCompile("org.knowm.xchart", "xchart", "3.2.2")

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
        doFirst {
            exec {
                commandLine("python3.7", "src/main/python/LTVUnicycleLRQ.py")
            }
        }
    }
    withType<JavaCompile>().configureEach {
        doFirst {
            println("Hullo")
        }
    }
}