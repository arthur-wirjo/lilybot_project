idf_component_register(SRCS "usb_test.c"
                    INCLUDE_DIRS "."
		    REQUIRES nvs_flash esp_tinyusb
)
