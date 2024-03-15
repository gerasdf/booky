#ifndef __KIT_H__
#define __KIT_H__
void kit_setup();
void kit_loop();
void kit_startPlaying(char *s_uid);
void kit_stopPlaying();
int kit_isPlaying();

void kit_startRecording(char *s_uid);
void kit_stopRecording();
int kit_isRecording();

extern const char* myname;
#endif // __KIT_H__