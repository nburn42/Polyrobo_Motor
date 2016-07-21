/***********************************************************************
 *Polyrobo motor control library
 *
 *written by Nathan Burnham nburn42@gmail.com
 *
 *
 * ********************************************************************/
#include "Polyrobo_Motor.h"

#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
  #define WIRE_WRITE Wire.send
#endif


Polyrobo_Motor::Polyrobo_Motor(uint8_t motor_count, uint8_t forward_pin_list [], uint8_t backward_pin_list []) {
    _need_update = new bool [motor_count];
    _max_list = new uint16_t [motor_count];
    _min_list = new uint16_t [motor_count];
    _forward_pin_list = forward_pin_list;
    _backward_pin_list = backward_pin_list;
    _feedback_used_list  = new bool [motor_count];
    _feedback_pin_list = new uint8_t [motor_count];
    _feedback_orientation_list = new bool [motor_count];
    _mode_list = new bool [motor_count];
    _speed_list = new uint8_t [motor_count];
    _forward_list = new bool [motor_count];
    _backward_list = new bool [motor_count];
    _target_position_list  = new uint8_t [motor_count];


    for(int8_t i = 0; i < motor_count; i++) {
        pinMode(_forward_pin_list[i], OUTPUT);
        pinMode(_backward_pin_list[i], OUTPUT);

        _need_update[i] = true;
        _max_list[i] = 255;
        _min_list[i] = 0;
        _feedback_used_list[i] = false;
        _feedback_pin_list[i] = 0;
        _feedback_orientation_list[i] = true;
        _mode_list[i] = false; // false = speed, true = position
        _speed_list[i] = HIGH;
        _forward_list[i] = false;
        _backward_list[i] = false;
        _target_position_list[i] = 127;
    }
}

// TODOS
// add asserts to all fuctions to check that motor_index is within range

void Polyrobo_Motor::setLimits(uint8_t motor_number, uint16_t raw_max, uint16_t raw_min) {
    _max_list[motor_number] = raw_max;
    _min_list[motor_number] = raw_min; 
}


void Polyrobo_Motor::setFeedback(uint8_t motor_number, uint8_t feedback_pin, bool feedback_orientation) {
    _feedback_used_list[motor_number] = true;
    _feedback_pin_list[motor_number] = feedback_pin;
    _feedback_orientation_list[motor_number] = feedback_orientation;
}
void Polyrobo_Motor::setSpeed(uint8_t motor_number, bool forward, bool backward, uint8_t speed=HIGH) {
    bool update = false;
    if(_speed_list[motor_number] != speed) update = true;
    if(_forward_list[motor_number] != forward) update = true;
    if(_backward_list[motor_number] != backward) update = true;
    if(_mode_list[motor_number] != false) update = true;

    if(update) {
        _need_update[motor_number] = true;
        _speed_list[motor_number] = speed;
        _forward_list[motor_number] = forward;
        _backward_list[motor_number] = backward;
        _mode_list[motor_number] = false;
    }
}

void Polyrobo_Motor::setPositionGlobal(uint8_t motor_number, uint8_t position, uint8_t speed=HIGH) {
    bool update = false;
    uint8_t rawPosition = _map(motor_number, position);
    if(_target_position_list[motor_number] != rawPosition) update = true;
    if(_speed_list[motor_number] != speed) update = true;
    if(_mode_list[motor_number] != false) update = true;

    if(update) {
        _need_update[motor_number] = true;
        _target_position_list[motor_number] = rawPosition;
        _speed_list[motor_number] = speed;
        _mode_list[motor_number] = false;
    }
}
void Polyrobo_Motor::setPositionRelative(uint8_t motor_number, bool direction, uint8_t change, uint8_t speed=HIGH) {
    bool update = false;
    int16_t signedChange; 
    if(direction) {
        signedChange = change;
    } else {
        signedChange = change * -1;
    }
    int16_t position = _unmap(motor_number, _target_position_list[motor_number]) + signedChange;

    if(position > 255) position = 255;
    if(position < 0) position = 0;

    uint16_t rawPosition = _map(motor_number, position);

    if(_target_position_list[motor_number] != rawPosition) update = true;
    if(_speed_list[motor_number] != speed) update = true;
    if(_mode_list[motor_number] != false) update = true;

    if(update) {
        _need_update[motor_number] = true;
        _target_position_list[motor_number] = rawPosition;
        _speed_list[motor_number] = speed;
        _mode_list[motor_number] = false;
    }
}
void Polyrobo_Motor::execute() {
    for(uint8_t i = 0; i < motor_count; i++) {
        if(_feedback_used_list[i] && _mode_list[i]) {
            _current_position_list[i] = analogRead(_feedback_pin_list[i]);
           _close_enough(i);
        }
    }

    for(uint8_t i = 0; i < motor_count; i++) {
        if(_need_update[i]) {
            if(_forward_list[i]) {
                analogWrite(_forward_pin_list[i], _speed_list[i]);
            } else {
                analogWrite(_forward_pin_list[i], LOW);
            }
            if(_backward_list[i]) {
                analogWrite(_backward_pin_list[i], _speed_list[i]);
            } else {
                analogWrite(_backward_pin_list[i], LOW);
            }
        }
    }
}

uint16_t Polyrobo_Motor::_map(uint8_t motor_number, uint8_t position) {
    uint16_t top, bottom;
    if(_feedback_orientation_list[motor_number]) {
        top = _max_list[motor_number];
        bottom = _min_list[motor_number];
    } else {
        bottom = _max_list[motor_number];
        top = _min_list[motor_number];
    }
    return map(position, 255, 0, top, bottom);
}

uint8_t Polyrobo_Motor::_unmap(uint8_t motor_number, uint16_t rawPosition) {
    uint16_t top, bottom;
    if(_feedback_orientation_list[motor_number]) {
        top = _max_list[motor_number];
        bottom = _min_list[motor_number];
    } else {
        bottom = _max_list[motor_number];
        top = _min_list[motor_number];
    }
    return map(rawPosition, top, bottom, 255, 0);
}

void Polyrobo_Motor::_close_enough(uint8_t motor_number) {
    uint16_t gap = 10;
    uint16_t current = _current_position_list[motor_number];
    uint16_t target = _target_position_list[motor_number];
    bool forward = _forward_list[motor_number];
    bool backward = _backward_list[motor_number];
    if(current < target - gap) {
        if(forward && !backward) {
            forward = true;
            backward = false;  
            _need_update[motor_number] = true;
        }
    } else if (current > target + gap) {
        if (!forward && backward) {
            forward = false;
            backward = true;
            _need_update[motor_number] = true;
        }
    } else {
        if(!forward && !backward) { 
            forward = false;
            backward = false;
            _need_update[motor_number] = true;
        }
    }
}
