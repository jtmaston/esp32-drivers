idf_component_register(SRCS
        i2cdrivertests.c
                    INCLUDE_DIRS
        .
                    REQUIRES i2cDriverCommon AT24 LM75 MPL3115 PCA9555 esp_event LSM6DSO-bridge
)
