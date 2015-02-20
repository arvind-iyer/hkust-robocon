#include "my_string.h"

/**
  * @brief Return a pointer to the first occurence of str2 in str1, or NULL pointer of str2 is not part of str1.
  * @param str1: C string to be scanned.
  * @param str2: C string containing the sequence of characters to match.
  * @param case_sensitive: Case sensitive for the character comparison
  * @example  mystrstr("Hello World!", "world", true) returns NULL,
              mystrstr("Hello World!", "world", false) returns the address of the char 'W' of str1.
  */
char* my_strstr(char* str1, const char* str2, bool case_sensitive) 
{
  if (case_sensitive) {
    return strstr(str1, str2);
  } else {
    const unsigned int str1_len = strlen(str1);
    const unsigned int str2_len = strlen(str2);
    char str1_clone[str1_len+1];
    char str2_clone[str2_len+1];
    
    strcpy(str1_clone, str1);
    strcpy(str2_clone, str2);
    
    // Down case str1
    for (unsigned int i = 0; i < str1_len; ++i) {
      if (str1_clone[i] >= 'A' && str1_clone[i] <= 'Z') {
        str1_clone[i] += 'a' - 'A';
      }
    }
    // Down case str2
    for (unsigned int i = 0; i < str2_len; ++i) {
      if (str2_clone[i] >= 'A' && str2_clone[i] <= 'Z') {
        str2_clone[i] += 'a' - 'A';
      }
    }
    
    char* strstr_clone = strstr(str1_clone, str2_clone);
    
    
    if (strstr_clone != NULL) {
      unsigned int size = strstr_clone - str1_clone;
      return (str1 + size);
      
    } else {
      return NULL;
    }
  }
  
}

