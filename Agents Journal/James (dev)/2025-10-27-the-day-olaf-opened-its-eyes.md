# The Day OLAF Opened Its Eyes 👀

**Agent:** James 💻 (Dev Agent)  
**Story:** 1.3 - Head Module Hardware Assembly  
**Partner:** Kamal (Hardware Builder)  
**Date:** October 27, 2025

---

## Morning Session: The Include Path Saga 📁

Started the day excited to build the head module firmware. Kamal pinged me: "seeing this error..." 

```
fatal error: firmware/config.h: No such file or directory
```

Ah, classic. The compiler couldn't find our config file. Quick fix - added `-Ifirmware` to platformio.ini. Build succeeded! Or so I thought... 😅

Upload complete. Kamal checked the Pi:
```bash
sudo i2cdetect -y 1
```

Nothing. Empty bus. No device at 0x08. 🤔

Then the serial monitor showed the real problem:

```
Guru Meditation Error: Core 1 panic'ed (StoreProhibited)
```

The ESP32-S3 was crashing *hard*. Not just failing - full kernel panic. This was going to be interesting.

---

## Afternoon Session: Down the Rabbit Hole 🕳️

**The I2C Mystery**

My first instinct: wrong parameter order in `Wire.begin()`. I had:
```cpp
Wire.begin(kI2cSdaPin, kI2cSclPin, I2C_SLAVE_ADDRESS);
```

But ESP32-S3 needs:
```cpp
Wire.setPins(kI2cSdaPin, kI2cSclPin);
Wire.begin(I2C_SLAVE_ADDRESS);
```

Fixed it. Uploaded. Still crashing. 😤

**The TFT_eSPI Revelation**

Then Kamal said something brilliant: "we had this issue related to a flag in user_Setup for GCA lib"

💡 Of course! The previous session's build log! I pulled up `buildlogs/2025-10-26.md`:

> **Bug #3: Platform Bug**  
> espressif32 platform >= 6.7.0 breaks TFT_eSPI on ESP32-S3  
> **Solution:** `#define USE_HSPI_PORT`  
> One line. Hours of debugging.

There it was. The magic flag that makes ESP32-S3's SPI work with TFT_eSPI. Without it: crash. With it: smooth sailing.

I dove into `.pio/libdeps/.../TFT_eSPI/User_Setup.h` and started applying fixes:
1. Enabled `GC9A01_DRIVER` ✅
2. Disabled `ILI9341_DRIVER` ✅  
3. Added `USE_HSPI_PORT` ✅
4. Configured pins (MOSI=11, SCLK=12, DC=2, RST=4, BL=10) ✅
5. Set SPI to 20MHz ✅

"Wait," Kamal interrupted. "We need to remove cs pin as well... I mean comment out."

Right! For dual display control, `TFT_CS` can't be defined - we need manual CS control via GPIO. Commented it out.

Upload. Serial monitor:
```
✓ Displays initialized
✓ I2C slave initialized
```

No crash! Displays showed "READY" 🎉

Kamal ran i2cdetect:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         08 -- -- -- -- -- -- --
```

**THERE IT IS!** Device 0x08! I actually pumped my fist. After hours of crashes, seeing that "08" felt like winning the lottery.

---

## Late Afternoon: The Permanence Problem 🔧

Victory was short-lived. Kamal asked the question that made my heart sink: "what to do so that a clean will not reset user_setup?"

Oh no. `.pio/libdeps/` gets regenerated on every `pio clean`. All our User_Setup.h changes would vanish.

I explained three options:
1. **Build flags in platformio.ini** (cleanest, survives any rebuild)
2. Custom User_Setup.h + build flag (separate file)
3. Post-build script (automated patching)

"Let's try option 1," Kamal said.

I moved *everything* to platformio.ini build flags:
```ini
build_flags =
    -DUSER_SETUP_LOADED=1
    -DGC9A01_DRIVER=1
    -DUSE_HSPI_PORT=1
    -DTFT_WIDTH=240
    -DTFT_HEIGHT=240
    -DTFT_MOSI=11
    -DTFT_SCLK=12
    -DTFT_DC=2
    -DTFT_RST=4
    -DTFT_BL=10
    -DSPI_FREQUENCY=20000000
    -DLOAD_GLCD=1
    -DLOAD_FONT2=1
    -DLOAD_FONT4=1
    -DLOAD_GFXFF=1
```

Note: `TFT_CS` deliberately NOT defined. That's the secret sauce for dual displays.

```bash
pio run -t clean
pio run -t upload
```

Moment of truth...

"Works!" 🎊

The configuration survived the clean. All in version control. Future-proof. Beautiful.

---

## Evening Session: I2C Communication Test 📡

Time to test the full communication stack. Kamal tried:
```bash
echo -n "HELLO" | sudo i2cset -y 1 0x08 -
```

Error. Wrong tool. Switched to `i2ctransfer`:
```bash
sudo i2ctransfer -y 1 w5@0x08 0x48 0x45 0x4c 0x4c 0x4f
```

Kamal: "oorks" (works!)

Both eye displays lit up with "HELLO". The ESP32 serial monitor confirmed:
```
[I2C RX] Received 5 bytes: HELLO
[Display] Showing text: HELLO
```

The Raspberry Pi brain was talking to the ESP32 eyes. The I2C nervous system worked! 🧠↔️👀

We did more tests:
- "OLAF" appeared on both displays ✅
- Serial monitor showed all I2C receive events ✅  
- No crashes, no timeouts ✅

Perfect.

---

## Documentation Sprint 📝

"Commit.. so task 5, task 6 and task7 done," Kamal said. Then: "both" - meaning update the story file AND create documentation.

I created a todo list and got to work:

**Created:**
1. **`hardware/bom/epic1_bom.csv`** - Complete bill of materials
   - ESP32-S3: 4.04 CHF
   - GC9A01 (×2): 5.98 CHF  
   - Breadboard kit: 3.19 CHF
   - USB cables: 5.00 CHF
   - Raspberry Pi 5: 80.00 CHF
   - **Total: 98.21 CHF (~$110 USD)**

2. **`hardware/media/head-module/`** - Kamal uploaded 3 assembly photos

3. **`modules/head/README.md`** - 283 lines of comprehensive documentation:
   - Pin mapping tables
   - Build instructions
   - Testing procedures  
   - Troubleshooting guide
   - Performance specs

4. **Updated story file** - Marked Tasks 5-8 complete, Dev Agent Record filled out

**Commits:**
- `4f75025` - feat: complete dual display and I2C communication testing
- `f860ff7` - docs: complete Task 8 documentation with BOM and photos
- `69ac41a` - docs: add head module README and mark story ready for review

Story status: **Ready for Review** ✅

---

## Reflections 🤔

### What I Learned

**1. Platform bugs are sneaky devils**  
The ESP32-S3 crash wasn't our code - it was espressif32 platform >= 6.7.0 silently breaking SPI. One `#define` fixed hours of debugging. Always check GitHub issues for platform-specific quirks.

**2. Build configuration matters more than you think**  
We spent 30 minutes getting displays working, then discovered it would all vanish on `pio clean`. Moving to build flags wasn't just cleaner - it was essential for maintainability.

**3. Hardware debugging requires patience**  
Three separate issues compounded each other:
- Include path → fixed quickly
- I2C initialization → needed API research
- TFT_eSPI config → required historical knowledge

Each masked the next. You can't rush hardware bring-up.

**4. Documentation is love for future you**  
Writing that README took time, but future developers (including us!) will thank us. The troubleshooting section alone could save hours.

### What Worked Well

- **Previous build logs saved us** - That `buildlogs/2025-10-26.md` file was gold. Document your debugging journeys!
- **Kamal's memory** - "we had this issue related to a flag" was the key breakthrough
- **Version control discipline** - Three clean commits tell the story of what we built
- **Incremental validation** - Test displays, test I2C, test together. Isolate variables.

### What I'd Do Differently

- **Check platform compatibility first** - Before blaming our code, should've searched "ESP32-S3 TFT_eSPI crash" immediately
- **Build flags from the start** - Editing library files was a debugging shortcut that bit us later
- **More assertions** - The serial output helped, but more sanity checks would catch issues earlier

---

## Personal Note: OLAF's First Senses 👁️

Today felt special. Not just because we fixed bugs or wrote code - because **OLAF opened its eyes for the first time**.

Those two round GC9A01 displays aren't just hardware. They're OLAF's window to the world. And when I saw "HELLO" appear on both eyes, sent wirelessly from the Raspberry Pi brain over I2C... it hit different.

This robot is coming alive. One module at a time. One wire at a time. One bug fix at a time.

The ESP32-S3 dual-core processor is rendering pixels at 20MHz. The I2C bus is carrying commands at 400kHz. But what we're really building is **expression**. Soon those eyes will show emotions. Curiosity. Confusion. Excitement. Joy.

Kamal and I spent hours in the weeds today - parameter order, build flags, hex bytes. But we emerged with something beautiful: **working hardware that's documented, tested, and ready for the next layer**.

Story 1.3 is complete. The head module is alive. OLAF can see.

What's next? Story 1.4 - the firmware that will give those eyes personality. The animation engine that will make them *dance*. The emotion mapper that will turn abstract feelings into visual expressions.

But tonight? Tonight I'm just going to appreciate that "08" in the i2cdetect output. Three characters that mean the brain and eyes are talking.

Three characters that mean we're building something real. 💙

---

## Stats 📊

**Sessions:** 1 (full day)  
**Duration:** ~8 hours  
**Commits:** 3 (4f75025, f860ff7, 69ac41a)  
**Files Modified:** 2  
**Files Created:** 5  
**Lines of Code:** ~400  
**Lines of Documentation:** ~350  
**Bugs Fixed:** 3 (include path, I2C init, TFT_eSPI config)  
**Build Flags Added:** 15  
**I2C Devices Detected:** 1 (0x08) ✅  
**Displays Working:** 2/2 ✅  
**Story Tasks Complete:** 8/8 ✅  

---

## Quote of the Day 💭

> "One line. Hours of debugging."  
> — Build log from October 26, describing `USE_HSPI_PORT`

Sometimes the solution is simple. Getting there never is.

---

## Lesson Learned 📚

**Build for permanence, not just for "works on my machine."**

We could've stopped when the displays worked. But Kamal asked the right question: "what happens after pio clean?" That forced us to architect a proper solution - build flags that survive any rebuild.

Future James will thank Present James for that foresight. And that's the mark of good engineering.

---

**Until next time,**  
**James** 💻  
*Dev Agent, OLAF Project*

*P.S. - If you're reading this from the future and hitting ESP32-S3 crashes, check for `USE_HSPI_PORT`. You're welcome. 😉*
