# configura a placa com wifi
set(PICO_BOARD pico_w)

add_executable(main
    main.c server_common.c
    )

target_link_libraries(main
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    )

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )
pico_btstack_make_gatt_header(main PRIVATE "${CMAKE_CURRENT_LIST_DIR}/temp_sensor.gatt")