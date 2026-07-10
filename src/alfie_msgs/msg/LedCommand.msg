# LED ring control for the reSpeaker XVF3800 4-Mic Array (12 RGB LEDs).

# Effect modes (map directly to the XVF3800 LED_EFFECT parameter).
uint8 EFFECT_OFF = 0
uint8 EFFECT_BREATH = 1
uint8 EFFECT_RAINBOW = 2
uint8 EFFECT_SOLID = 3
uint8 EFFECT_DOA = 4
uint8 EFFECT_RING = 5

uint8 effect

# Brightness, 0..255 (used by breath/rainbow modes).
uint8 brightness

# Effect speed, 0..255 (used by breath/rainbow modes).
uint8 speed

# Packed color 0x00RRGGBB (used by solid/breath modes).
uint32 color
