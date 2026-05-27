idf_component_register(SRCS "micro-ROS.c" "personal_motor_control.c"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component 
		    	     driver 
			     esp_timer
		     	     mpu9250)
