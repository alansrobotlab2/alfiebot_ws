# One open-vocabulary detection from NanoOWL (OWL-ViT).
string label       # the text query that matched, e.g. "a person"
uint16 class_id    # index of the label within the prompt's query list
float32 score      # sigmoid confidence, 0.0 .. 1.0

# Axis-aligned bounding box in normalized image coordinates (0.0 .. 1.0),
# corner format with the origin at the top-left of the image.
float32 x0
float32 y0
float32 x1
float32 y1
