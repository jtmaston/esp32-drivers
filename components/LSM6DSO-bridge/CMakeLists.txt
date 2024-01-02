idf_component_register(SRCS
        LSM6DSO.c
        driver/lsm6dso_reg.c
                    INCLUDE_DIRS
        include
        driver/

        REQUIRES i2cDriverCommon
)
