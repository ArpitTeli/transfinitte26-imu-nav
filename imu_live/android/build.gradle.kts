// Plugins come from the buildscript classpath (works with an offline Gradle cache that
// has the Android Gradle plugin but not its plugin-marker artifacts).
buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath("com.android.tools.build:gradle:8.5.2")
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.9.24")
    }
}
