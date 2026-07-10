# Direction of arrival from the reSpeaker XVF3800 mic array.
std_msgs/Header header

# Azimuth of the detected sound source, 0..359 degrees (0 = front of array).
uint16 angle

# True when the array's voice-activity detector reports speech this sample.
bool speech_detected
