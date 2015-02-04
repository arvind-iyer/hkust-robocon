#include "racket_control.h"

static bool command_received = false;

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
}

void racket_received_command(void)
{
	command_received = true;
}

bool did_receive_command(void)
{
	return command_received;
}
