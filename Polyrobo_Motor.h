/*************************************************************************
 * Library to control motors
 *
 * By Nathan Burnham nburn42@gmail.com
 *
 * Assumes all motors have pwm pins
 ************************************************************************/
#ifndef _POLYROBO_MOTOR_H_
#define _POLYROBO_MOTOR_H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
  #define WIRE_WRITE Wire.send
#endif


class Polyrobo_Motor {
    public:
        Polyrobo_Motor(uint8_t motor_count, const uint8_t forward_pin_list [], const uint8_t backward_pin_list []);

        uint8_t motor_count;

        void setLimits(uint8_t motor_number, uint16_t raw_max, uint16_t raw_min);
        void setFeedback(uint8_t motor_number, uint8_t feedback_pin, bool feedback_orientation);

        void setSpeed(uint8_t motor_number, bool forward, bool reverse, uint8_t speed);
        void setPositionGlobal(uint8_t motor_number, uint8_t position, uint8_t speed);
        void setPositionRelative(uint8_t motor_number, bool direction, uint8_t change, uint8_t speed);
        
        void execute();

        uint16_t *current_position_list;

        bool *_need_update;
        uint16_t *_min_list;
        uint16_t *_max_list;
        const uint8_t *_forward_pin_list;
        const uint8_t *_backward_pin_list;
        bool *_feedback_used_list;
        uint8_t *_feedback_pin_list;
        bool *_feedback_orientation_list;
        bool *_mode_list;
        uint8_t *_speed_list;
        bool *_forward_list;
        bool *_backward_list;
        uint16_t *_target_position_list;
    private:

        uint16_t _map(uint8_t motor_number, uint8_t position);
        uint8_t _unmap(uint8_t motor_number, uint16_t rawPosition);
        void _close_enough(uint8_t motor_number);


};

#endif /* POLYROBO_MOTOR_H_ */
