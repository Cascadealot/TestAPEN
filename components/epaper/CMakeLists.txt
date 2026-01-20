# e-Paper display component - only built for UI node
# Direct SSD1680 driver with corrected partial refresh LUT

if(CONFIG_TESTAP2_NODE_UI)
    idf_component_register(
        SRCS
            "epaper.c"
        INCLUDE_DIRS
            "include"
        REQUIRES
            driver
        PRIV_REQUIRES
            esp_log
    )
else()
    # Register empty component for non-UI nodes
    idf_component_register(
        INCLUDE_DIRS
            "include"
        REQUIRES
            driver
    )
endif()
