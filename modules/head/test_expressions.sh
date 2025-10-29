#!/bin/bash
# Test script for OLAF Head Module Eye Expressions
# Tests all expression shapes and colors

echo "=========================================="
echo "OLAF Head Module - Expression Test Script"
echo "Testing new shapes and colors"
echo "=========================================="
echo ""

# Check if module is detected
echo "1. Detecting module at 0x08..."
if i2cdetect -y 1 | grep -q "08"; then
    echo "✓ Module detected"
else
    echo "✗ Module NOT detected! Check wiring."
    exit 1
fi
echo ""

echo "2. Testing all expressions at intensity 3..."
echo "   Watch for different shapes and colors!"
echo ""

# NEUTRAL - Cyan rounded rectangles
echo "Expression: NEUTRAL (Cyan rounded rectangles)"
i2cset -y 1 0x08 0x10 0x00  # NEUTRAL
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# HAPPY - Green upward arcs (smiling)
echo "Expression: HAPPY (Green arcs - smiling eyes)"
i2cset -y 1 0x08 0x10 0x01  # HAPPY
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# CURIOUS - Cyan circles (asymmetric sizes)
echo "Expression: CURIOUS (Cyan circles - one larger)"
i2cset -y 1 0x08 0x10 0x02  # CURIOUS
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# THINKING - Cyan rounded rectangles (looking up-right)
echo "Expression: THINKING (Cyan rounded rects - looking up)"
i2cset -y 1 0x08 0x10 0x03  # THINKING
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# CONFUSED - Orange horizontal lines (squinting)
echo "Expression: CONFUSED (Orange lines - squinting)"
i2cset -y 1 0x08 0x10 0x04  # CONFUSED
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# SAD - Blue downward arcs (frowning)
echo "Expression: SAD (Blue arcs - frowning eyes)"
i2cset -y 1 0x08 0x10 0x05  # SAD
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

# EXCITED - White large circles
echo "Expression: EXCITED (White circles - wide eyed)"
i2cset -y 1 0x08 0x10 0x06  # EXCITED
i2cset -y 1 0x08 0x11 0x03  # Intensity 3
sleep 4

echo ""
echo "3. Testing intensity levels with HAPPY expression..."
echo "   Watch size changes!"
echo ""

i2cset -y 1 0x08 0x10 0x01  # HAPPY

for intensity in 1 2 3 4 5; do
    echo "   Intensity: $intensity"
    i2cset -y 1 0x08 0x11 $intensity
    sleep 3
done

echo ""
echo "4. Testing intensity levels with EXCITED expression..."
echo "   Watch size and blink speed changes!"
echo ""

i2cset -y 1 0x08 0x10 0x06  # EXCITED

for intensity in 1 2 3 4 5; do
    echo "   Intensity: $intensity"
    i2cset -y 1 0x08 0x11 $intensity
    sleep 3
done

echo ""
echo "5. Returning to NEUTRAL..."
i2cset -y 1 0x08 0x10 0x00  # NEUTRAL
i2cset -y 1 0x08 0x11 0x02  # Intensity 2
echo ""

echo "=========================================="
echo "Test complete!"
echo ""
echo "Expression Summary:"
echo "  0=NEUTRAL   (cyan rounded rects)"
echo "  1=HAPPY     (green arcs ⌣)"
echo "  2=CURIOUS   (cyan circles ○)"
echo "  3=THINKING  (cyan rounded rects)"
echo "  4=CONFUSED  (orange lines ―)"
echo "  5=SAD       (blue arcs ⌢)"
echo "  6=EXCITED   (white circles ◉)"
echo "=========================================="
