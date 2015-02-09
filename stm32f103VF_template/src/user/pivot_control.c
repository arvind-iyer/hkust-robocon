#include "pivot_control.h"

void pivot_turn_left(void)
{
}

void pivot_turn_right(void)
{
}

void pivot_init(void)
{
		register_special_char_function('u', pivot_turn_left);
		register_special_char_function('o', pivot_turn_right);
}
