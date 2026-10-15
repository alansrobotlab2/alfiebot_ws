# ASRResult.msg
string asrresult
# Wall-clock time (time.time()) the speaker stopped talking, for latency logs.
float64 speech_end 0.0
# Seconds of audio transcribed, and how long transcription took.
float32 duration 0.0
float32 asr_seconds 0.0
# End-of-turn model score at commit (-1 when no turn model ran), and why the
# turn was committed: "complete", "hold_released", "hold_timeout", "hold_max",
# "no_turn_model".
float32 turn_prob -1.0
string reason ''
