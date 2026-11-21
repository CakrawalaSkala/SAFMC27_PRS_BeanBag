idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES lora button easy_gpio esp_driver_gpio esp_driver_ledc driver freertos log esp_timer
)