#ifndef __KIT_H__
#define __KIT_H__
void kit_setup();
void kit_loop();
void kit_startPlaying(char *s_uid);
int kit_isPlaying();

void kit_startRecording(char *s_uid);
int kit_isRecording();

void kit_stopAll();

extern const char* myname;
#endif // __KIT_H__