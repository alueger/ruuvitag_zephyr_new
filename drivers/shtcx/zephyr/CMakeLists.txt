if(CONFIG_SHTCX)
    zephyr_include_directories(.)
    zephyr_library()

    zephyr_library_sources(shtcx.c)
endif()