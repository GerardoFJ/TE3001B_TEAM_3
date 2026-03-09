Import("env")

env.Append(
    LIBPATH=[
        env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV/micro_ros_arduino/src/esp32")
    ],
    LIBS=["microros"]
)
