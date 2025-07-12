/**
 * Martin Egli
 * 2023-05-03
 * mmlib: str_buf string buffer functions
 */

#ifndef _MM_str_buf_H_
#define _MM_str_buf_H_

#include <stdint.h>

#define STRING_TERMINATION '\0'

/**
 * clear the string, set str_buf_TERMINATION to head (position = 0) ot the string
 * @param   str         string to find the end in
 * @param   str_size    size of the string
 * @return  pos = 0: valid position
 */
uint16_t str_buf_clear(char *str, uint16_t str_size);

/**
 * find the end position of a given string, '\0'-terminated
 * @param   str         string to find the end in
 * @param   str_size    size of the string
 * @return  pos = 0 ... x: valid position, found the end before str_size, 
 *          pos = str_size: could not find the end, pos invalid
 */
uint16_t str_buf_find_end_pos(char *str, uint16_t str_size);

/** 
 * append a single char c to the string str
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   c           char to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_char(char *str, uint16_t str_size, char c);

/** 
 * append the string str_append to the string str
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   str_append  string to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_string(char *str, uint16_t str_size, char *str_append);

/** 
 * copy a string to de destination string, give position where to copy and copy string size
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   str_copy  string to copy
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_copy_pos_size(char *str, uint16_t str_size, char *str_copy, uint16_t copy_to_pos, uint16_t copy_size);

/**
 * delete a given number of chars from a given position of the string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   del_pos     delete from position, must be < str_size
 * @param   nb_chars    number of chars to delete
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_delete_from_position(char *str, uint16_t str_size, uint16_t del_pos, uint16_t nb_chars);

/**
 * delete a given number of chars from the head (position = 0) of the string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   nb_chars    number of chars to delete
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_delete_from_head(char *str, uint16_t str_size, uint16_t nb_chars);

/**
 * delete a given number of chars from the tail (end) of the string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   nb_chars    number of chars to delete
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_delete_from_tail(char *str, uint16_t str_size, uint16_t nb_chars);

/**
 * append a number in uint8_t as chars to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_uint8(char *str, uint16_t str_size, uint8_t num);

/**
 * append a number in int8_t as chars with sign (-='-', +=' ') to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_int8(char *str, uint16_t str_size, int8_t num);

/**
 * append a number in uint16_t as chars to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_uint16(char *str, uint16_t str_size, uint16_t num);

/**
 * append a number in int16_t as chars with sign (-='-', +=' ') to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_int16(char *str, uint16_t str_size, int16_t num);

/**
 * append a number in uint32_t as chars to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_uint32(char *str, uint16_t str_size, uint32_t num);

/**
 * append a number in int32_t as chars with sign (-='-', +=' ') to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_int32(char *str, uint16_t str_size, int32_t num);

/**
 * append a number in uint32_t as chars to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @param   nb_digits   number of digits to use
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_uint32_nb_digits(char *str, uint16_t str_size, uint32_t num, uint16_t nb_digits);

/**
 * append a number in int32_t as chars with sign (-='-', +=' ') to the destination string
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @param   nb_digits   number of digits to use
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_int32_nb_digits(char *str, uint16_t str_size, int32_t num, uint16_t nb_digits);

/**
 * append a number in float as chars with sign (-='-', +=' ') to the destination string
 * post point presision = number of chars after '.'
 * 
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @param   nb_digits   number of digits to use
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 */
uint16_t str_buf_append_float(char *str, uint16_t str_size, float num, uint16_t post_presision);

/**
 * append a number in hex as chars to the destination string in upper case, prepend with "0x"
 * @param   str         destination string to append to, this string will be changed
 * @param   str_size    size of the destination string
 * @param   num         number to append
 * @param   nb_digits   number of digits to use
 * @return  pos, end position in string
 *          = 0 ... x: valid position
 *          = str_size: appended as much as possible, but reached the end of destination string
 * 
 */
uint16_t str_buf_append_0x_hex_upper_case_nb_digits(char *str, uint16_t str_size, uint32_t num, uint16_t nb_digits);

#endif // _MM_str_buf_H_
