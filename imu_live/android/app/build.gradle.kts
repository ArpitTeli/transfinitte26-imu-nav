plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.imunav.stream"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.imunav.stream"
        minSdk = 26
        targetSdk = 35
        versionCode = 4
        versionName = "1.3"
    }
    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
}
