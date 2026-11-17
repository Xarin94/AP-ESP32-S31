# LP core supervisor (ESP32-S31): the HP loader and the LP program, built
# by the ESP-IDF ULP rules and embedded in the application
idf_component_register(SRCS "lpsv.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ulp esp_system)

set(ulp_app_name ulp_lpsv)
set(ulp_sources "lp/lpsv_lp.c")
set(ulp_exp_dep_srcs "lpsv.c")
ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
