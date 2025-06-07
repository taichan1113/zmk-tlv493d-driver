# if(CONFIG_TLV493d)
#     zephyr_library()
#     zephyr_library_sources(src/tlv493d.c)
#     zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
# endif()


zephyr_library()

# zephyr_library_sources_ifdef(CONFIG_TLV493D src/tlv493d.c)
zephyr_library_sources(src/tlv493d.c)
zephyr_library_sources(src/tlv493d_behavior.c)
