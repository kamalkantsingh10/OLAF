# Agent Journal Prompts

Two-tier journaling system for OLAF agent work.

---

## **PROMPT 1: Session Log (During Work)**

Use this prompt when you finish a work session and want to log what happened.

```
Write a quick session log for the work I just completed.

Context:
- Agent: [Your agent name and role]
- Session: [Morning/Afternoon/Evening or time period]
- Task: [What you were working on]

Keep it brief and factual:
- What was I trying to do?
- What happened? (chronological order)
- Did it work? ✅ or fail? ❌
- Key quotes from Kamal
- Any important technical details
- What I learned
- Current status

Use emojis for quick emotional context 🎉 ❌ 💡 🐛

Format: Simple bullet points or short paragraphs
Length: 200-500 words
Tone: Quick notes, not polished prose

This will be merged into the daily journal later.
```

**Example Output:**
```markdown
## Afternoon Session: Independent Display Control

Trying to get left and right eye displays to show different content.

- Started with backlight blink test ✅
- Researched dual display control - found WordPress article! 💡
- Manual CS control is the key
- Wrote test code: left="LEFT", right="RIGHT"
- Kamal: "Please do it for me" (modify User_Setup.h)
- Found duplicate ESP8266 pins still active 🐛
- Fixed 8 config issues in User_Setup.h
- Kamal double-checked: "can you check USE_HSPI_PORT?"
- Verified against official configs ✅

Result: IT WORKS! 🎉
- Left eye shows "LEFT"
- Right eye shows "RIGHT"
- Independent control achieved

Learning: Manual CS = comment out TFT_CS in library config

Status: Task 5 complete, displays independent
```

---

## **PROMPT 2: Daily Journal (End of Day)**

Use this prompt at end of day to create the polished journal entry from all session logs.

```
Create James's daily journal entry for today using the session logs below.

Input: [Paste all session logs from today]

Transform the session logs into a cohesive narrative:

Structure:
1. Title: Catchy, descriptive title for the day
2. Header: Agent name, story, partner
3. Sessions: Organize logs by Morning/Afternoon/Evening
4. Reflections: Synthesize learnings across all sessions
5. Personal Note: What this day meant for OLAF
6. Stats: Aggregate all session metrics
7. Sign-off: Quote of day, lesson learned, signature

Writing Style:
- First-person narrative from my perspective
- Tell the story of the day as a journey
- Include actual dialogue/quotes from sessions
- Use emojis throughout for emotion 🎉 💻 ❌ ✅ 🔥
- Show progression: problem → struggle → breakthrough → victory
- Reflect on what it means for OLAF's development

Keep it simple - NO sections for:
- Final Status
- Next Steps
- Team Notes
- Technical Appendix

Make it feel like reading my personal developer journal.
The reader should feel the frustration of bugs, the eureka moments,
and the satisfaction of making OLAF's eyes come alive! 🚀
```

**Example Input to Prompt 2:**
```
[Paste all 3 session logs from the day]

Session 1: Morning - Three Bug Hunt
Session 2: Afternoon - Breaking the Mirror
Session 3: Evening - Configuration Philosophy
```

**Example Output:**
```markdown
# Build Log - October 26, 2025
## James's Journal: The Day OLAF's Eyes Became Independent

[Full narrative journal with all sessions woven together...]
```

---

## **Workflow:**

### During the Day:
1. Finish a work session
2. Use **PROMPT 1** → Create session log
3. Save to temp file or notes
4. Repeat for each session

### End of Day:
1. Collect all session logs
2. Use **PROMPT 2** with all logs as input
3. Get polished daily journal
4. Save to `Agents Journal/[Agent]/{date}.md`

---

## **Quick Reference:**

| Prompt | When | Output | Length |
|--------|------|--------|--------|
| Session Log | After each work session | Quick factual notes | 200-500 words |
| Daily Journal | End of day | Narrative story | 2000-5000 words |

---

## **Session Log Template:**

```markdown
## [Time] Session: [Title]

Goal: [What I was trying to accomplish]

What Happened:
- [Event 1]
- [Event 2]
- [Event 3]

Result: ✅/❌ [Outcome]

Key Quotes:
- Kamal: "[quote]"

Learnings:
- [Learning 1]
- [Learning 2]

Status: [Current state]
```

---

## **Tips:**

### For Session Logs:
- ✅ Write immediately after session (memory fresh)
- ✅ Be honest about failures
- ✅ Include exact error messages
- ✅ Note what you'd do differently
- ✅ Keep it raw and real

### For Daily Journal:
- ✅ Look for the narrative arc (struggle → resolution)
- ✅ Combine related events across sessions
- ✅ Highlight emotional journey, not just facts
- ✅ Connect to OLAF's bigger story
- ✅ End with reflection on progress
