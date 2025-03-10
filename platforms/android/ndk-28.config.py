ABIs = [
    ABI("3", "arm64-v8a",   None, cmake_vars=dict(ANDROID_GRADLE_PLUGIN_VERSION='8.8.0', GRADLE_VERSION='8.10.2')),
    ABI("5", "x86_64",      None, cmake_vars=dict(ANDROID_GRADLE_PLUGIN_VERSION='8.8.0', GRADLE_VERSION='8.10.2')),
    # ABI("4", "x86",         None, cmake_vars=dict(ANDROID_GRADLE_PLUGIN_VERSION='8.8.0', GRADLE_VERSION='8.10')),
]
