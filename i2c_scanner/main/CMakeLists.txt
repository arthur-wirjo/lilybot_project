idf_component_register(SRCS "i2c_scanner.c"
                    INCLUDE_DIRS "."
	    	    REQUIRES driver
	    		     esp_timer)
