// textfile.h: interface for reading and writing text files
// www.lighthouse3d.com
//
// You may use these functions freely.
// they are provided as is, and no warranties, either implicit,
// or explicit are given
//////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_WARNINGS

char *textFileRead(const char *fn);
int textFileWrite(char *fn, char *s);
