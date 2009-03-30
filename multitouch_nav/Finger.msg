# A message for finger touch events on a multi-touch table

# the identifier for this finger
int8 id

# position of the finger on the table
int32 x
int32 y

# event types
int8 FINGER_DOWN=1
int8 FINGER_DRAG=2
int8 FINGER_UP=3

# the type for this event
int8 eventType