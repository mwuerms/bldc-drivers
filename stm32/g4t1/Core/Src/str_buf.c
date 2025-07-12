/**
 * Martin Egli
 * 2023-05-03
 * mmlib: str_buf string buffer functions
 */

#include <stddef.h>
#include "str_buf.h"


#include <stdio.h>

uint16_t str_buf_clear(char *str, uint16_t str_size) {
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    str[0] = STRING_TERMINATION;
    return 0;
}

uint16_t str_buf_find_end_pos(char *str, uint16_t str_size) {
    uint16_t pos;
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }

    for(pos = 0; pos < str_size; pos++) {
        if(str[pos] == STRING_TERMINATION) {
            // found end
            return pos;
        }
    }
    // error, did not find end
    return str_size;
}

// private function, skip the checks at the begining, increment pos after appending
static uint16_t str_buf_append_char_inc_pos(char *str, uint16_t str_size, uint16_t pos, char c) {
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if(pos == (str_size - 2)) {
        // this is the end position in str
        str[pos] = STRING_TERMINATION;
        return str_size;
    }
    *str++ = c;
    *str = STRING_TERMINATION;
    return pos+1;
}

uint16_t str_buf_append_char(char *str, uint16_t str_size, char c) {
    uint16_t pos;
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    pos = str_buf_find_end_pos(str, str_size);
    return str_buf_append_char_inc_pos(str, str_size, pos, c);
}

uint16_t str_buf_append_string(char *str, uint16_t str_size, char *str_append) {
    uint16_t pos;
    char c;
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    // check if str_append is valid
    if(str_append == NULL) {
        // error, invalid
        return str_size;
    }

    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    // copy str_append to str, go from pos
    while(pos < str_size) {
        c = *str_append++;
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, c);
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
        if(c == STRING_TERMINATION) {
            // found end in str_append
            break;
        }
    }
    return pos;
}

uint16_t str_buf_copy_pos_size(char *str, uint16_t str_size, char *str_copy, uint16_t copy_to_pos, uint16_t copy_size) {
    uint16_t pos;
    char c;
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    // check if str_append is valid
    if(str_copy == NULL) {
        // error, invalid
        return str_size;
    }
    if(copy_to_pos + copy_size >= str_size) {
        // error, invalid
        return str_size;
    }

    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if(copy_to_pos > pos) {
        // it will be outside of pos, but still valid
        // no error, but no effect, so do not copy, do not continue
        return pos;
    }

    // all checks OK, start copy routine
    for(;copy_size; copy_size--) {
        c = *str_copy++;
        str[copy_to_pos++] = c;
    }
    if(c != STRING_TERMINATION) {
        // attention, set new termination
        // check for str_size
        if(copy_to_pos >= str_size) {
            copy_to_pos = str_size -1;
        }
        str[copy_to_pos] = STRING_TERMINATION;
    }
    return copy_to_pos;
}

uint16_t str_buf_delete_from_position(char *str, uint16_t str_size, uint16_t del_pos, uint16_t nb_chars) {
    uint16_t pos, copy_pos;
    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    // check if nb_chars is valid, must be < str_size
    if(nb_chars >= str_size) {
        // error, invalid
        return str_size;
    }

    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if((del_pos + nb_chars) >= pos) {
        // outside pos, so nothing to do
        // skip any copying/deleting
        return pos;
    }
    //str_buf_copy_pos_size(char *str, uint16_t str_size, char *str_copy, uint16_t copy_to_pos, uint16_t copy_size) {
    copy_pos = del_pos + nb_chars;
    return str_buf_copy_pos_size(str, str_size, &str[copy_pos], del_pos, (pos-copy_pos));
}

uint16_t str_buf_delete_from_head(char *str, uint16_t str_size, uint16_t nb_chars) {
    return str_buf_delete_from_position(str, str_size, 0, nb_chars);
}

uint16_t str_buf_delete_from_tail(char *str, uint16_t str_size, uint16_t nb_chars) {
    uint16_t tail_pos;
    tail_pos = str_buf_find_end_pos(str, str_size);
    if(tail_pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }

    if(nb_chars >= tail_pos) {
        tail_pos = 0;
    }
    else {
        tail_pos -= nb_chars;
    }
    // nothing to copy, just set termination
    str[tail_pos] = STRING_TERMINATION;
    return tail_pos;
}

uint16_t str_buf_append_uint8(char *str, uint16_t str_size, uint8_t num) {
    return str_buf_append_uint32_nb_digits(str, str_size, (uint32_t)num, 3);
}

uint16_t str_buf_append_int8(char *str, uint16_t str_size, int8_t num) {
    return str_buf_append_int32_nb_digits(str, str_size, (int32_t)num, 3);
}

uint16_t str_buf_append_uint16(char *str, uint16_t str_size, uint16_t num) {
    return str_buf_append_uint32_nb_digits(str, str_size, (uint32_t)num, 5);
}

uint16_t str_buf_append_int16(char *str, uint16_t str_size, int16_t num) {
	return str_buf_append_int32_nb_digits(str, str_size, (int32_t)num, 5);
}

uint16_t str_buf_append_uint32(char *str, uint16_t str_size, uint32_t num) {
    return str_buf_append_uint32_nb_digits(str, str_size, (uint32_t)num, 10);
}

uint16_t str_buf_append_int32(char *str, uint16_t str_size, int32_t num) {
    return str_buf_append_int32_nb_digits(str, str_size, (int32_t)num, 10);
}

uint16_t str_buf_append_uint32_nb_digits(char *str, uint16_t str_size, uint32_t num, uint16_t nb_digits) {
    uint16_t pos;
    uint16_t digit_pos, use_char;
	int32_t digits[] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
    uint16_t total_nb_digits = (sizeof(digits)/sizeof(digits[0]));
	uint8_t digit_value;

    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if(num == 0) {
        // only append '0', and done here
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0');
        return pos;
    }
    if(nb_digits == 0)  {
        // ignore, do nothing
        return pos;
    }
    if(nb_digits > total_nb_digits) {
        // max nb of digits
        nb_digits = total_nb_digits;
    }

	use_char = 0;
	for(digit_pos = (total_nb_digits - nb_digits); digit_pos < total_nb_digits; digit_pos++) {
		digit_value = (uint8_t)(num/digits[digit_pos]);
		if((digit_value != 0) || (use_char != 0)) {
			use_char = 1;
			pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0'+digit_value);
			if(pos == str_size) {
                // pos at the end: done, no more space available
                return str_size;
            }
			num -= digit_value * digits[digit_pos];
		}
	}
    return pos;
}

uint16_t str_buf_append_int32_nb_digits(char *str, uint16_t str_size, int32_t num, uint16_t nb_digits) {
    uint16_t pos;
	int32_t digits[] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
    uint16_t total_nb_digits = (sizeof(digits)/sizeof(digits[0]));

    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if(num == 0) {
        // only append '0', and done here
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0');
        return pos;
    }
    if(nb_digits == 0)  {
        // ignore, do nothing
        return pos;
    }
    if(nb_digits > total_nb_digits) {
        // max nb of digits
        nb_digits = total_nb_digits;
    }
    if(num < 0) {
        num *= -1;
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '-');
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    }
    return str_buf_append_uint32_nb_digits(str, str_size, (uint32_t)num, nb_digits);
}

uint16_t str_buf_append_float(char *str, uint16_t str_size, float num, uint16_t post_presision) {
    uint32_t x;
    uint16_t pos, n;
    uint8_t digit_value;

    pos = str_buf_find_end_pos(str, str_size);
    if(num == 0.0f) {
        // special case
        return str_buf_append_string(&str[pos], str_size, "0.0");
    }
    if(num < 0.0f) {
        num *= -1;
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '-');
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    }

    // print number before '.'
    x = (uint32_t)num;
    pos = str_buf_append_uint32(str, str_size, x);
    if(pos == str_size) {
        // pos at the end: done, no more space available
        return str_size;
    }
    if(post_presision == 0) {
		// do not add any decimal places
		return pos;
	}
	if(post_presision >= 9) {
		// max is 9
		post_presision = 9;
	}
    // print number after '.'
    num -= (float)x;
    pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '.');
    if(pos == str_size) {
        // pos at the end: done, no more space available
        return str_size;
    }
    for(n = 0; n < post_presision; n++) {
        num *= 10.0f;
        digit_value = (uint8_t)num;
        num -= (float)digit_value;
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0'+digit_value);
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    }
    return pos;
}

uint16_t str_buf_append_0x_hex_upper_case_nb_digits(char *str, uint16_t str_size, uint32_t num, uint16_t nb_digits) {
    uint16_t pos;
    uint16_t digit_pos;
    uint16_t total_nb_digits = sizeof(uint32_t)*2;
    uint8_t digit_value;

    // check if str is valid
    if(str == NULL) {
        // error, invalid
        return str_size;
    }
    // check if str_size is valid
    if(str_size == 0) {
        // error, invalid
        return str_size;
    }
    pos = str_buf_find_end_pos(str, str_size);
    if(pos == str_size) {
        // error, already at the end, no more space available
        // did not do anything
        return str_size;
    }
    if(num == 0) {
        // only append '0', and done here
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0');
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, 'x');
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0');
        return pos;
    }
    if(nb_digits == 0)  {
        // ignore, do nothing
        return pos;
    }
    if(nb_digits > total_nb_digits) {
        // max nb of digits
        nb_digits = total_nb_digits;
    }
    pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0');
    if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, 'x');
    if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    for(digit_pos = (total_nb_digits - nb_digits); digit_pos < total_nb_digits; digit_pos++) {
        digit_value = 0;
		// append '0' at the beginning as anyway
        pos = str_buf_append_char_inc_pos(&str[pos], str_size, pos, '0'+digit_value);
        if(pos == str_size) {
            // pos at the end: done, no more space available
            return str_size;
        }
    }
    return pos;
}
