# Expression message for OLAF's emotional displays
# Used by personality coordinator to command all expression modules

uint8 emotion_type    # 0=neutral, 1=happy, 2=curious, 3=thinking, 4=confused, 5=sad, 6=excited
uint8 intensity       # 1-5 (1=subtle, 5=extreme)
uint16 duration_ms    # Expression duration in milliseconds
