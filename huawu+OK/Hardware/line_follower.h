#ifndef __LINE_FOLLOWER_H_
#define __LINE_FOLLOWER_H_

void line_follower_init(void);
void line_following_control(void);
void set_target_speed(float speed);
void stop_car(void);
uint8_t is_line_lost(void);
void handle_line_lost(void);


#endif