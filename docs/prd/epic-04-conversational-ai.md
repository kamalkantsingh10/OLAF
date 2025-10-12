# Epic 4: AI Agent Framework

**Epic Goal:** Build an agent framework that brings AI agents to physical life, enabling tool use, function calling, and embodied intelligence. Integrate hybrid local + cloud AI: Hailo-accelerated Whisper for speech recognition + cloud AI agents (Claude/GPT-4) for reasoning, with personality-driven responses and multi-step planning.

**Dependencies:**
- Epic 1 (Foundation + Minimal Personality) must be complete
- Epic 3 (Complete Personality Expression System) must be complete

**Estimated Effort:** 50-60 hours (2-3 weeks)

**Note:** Agent framework implementation (Pydantic AI, LangGraph, or custom) will be finalized during technical architecture phase.

---

## User Stories

### Story 4.1: Agent Framework Foundation & Cloud AI Integration

**As a** developer building an embodied AI agent,
**I want** an agent framework integrated with cloud AI APIs,
**so that** AI agents can reason, plan, and control Olaf through tool use.

#### Acceptance Criteria:
1. Agent framework architecture designed (framework TBD: Pydantic AI, LangGraph, or custom)
2. Claude API client module created in orchestrator (Python)
3. API key loaded from secure config file: `~/.olaf/secrets/claude_api_key`
4. Client supports Claude 3.5 Sonnet model with **tool use / function calling**
5. Request/response handling with proper error management:
   - Network timeout: 30 seconds, auto-retry 3 times
   - Rate limiting: respect 429 responses, exponential backoff
   - API errors: log and gracefully degrade (fallback responses)
6. Token usage tracking and logging (input/output tokens per request)
7. Cost estimation calculator: tracks daily/monthly API spend
8. ROS2 service created: `/olaf/agent/execute` (input: text, output: agent actions)
9. Tool/function definition structure designed (will be populated in later stories)
10. Unit tests verify API integration with mock responses
11. Environment variable override: `OLAF_CLAUDE_API_KEY` (for testing)

**Technical Notes:**
- Use `anthropic` Python SDK with tools/function calling support
- Implement request queue to prevent concurrent API calls
- Store API key with 600 permissions for security
- Log all requests/responses for debugging (sanitize sensitive data)
- Design for extensibility: support multiple agent frameworks later

---

### Story 4.2: Conversation Context Management

**As a** user having extended conversations with Olaf,
**I want** Olaf to remember previous exchanges within a session,
**so that** conversations feel coherent and contextual.

#### Acceptance Criteria:
1. Conversation context manager maintains sliding window of last 10 exchanges
2. Context stored as list of message pairs: `[(user_input, assistant_response), ...]`
3. Context passed to Claude API as message history
4. Context cleared on explicit command: "forget our conversation" or `/olaf/ai/clear_context` service
5. Context auto-expires after 30 minutes of inactivity
6. Context persisted to disk: `~/.olaf/context/session_YYYYMMDD_HHMMSS.json`
7. Context reload on startup if last session < 30 minutes ago
8. Token limit management: truncate oldest messages if context > 8000 tokens
9. Privacy mode: context not saved to disk when enabled
10. Unit tests verify context retention and truncation logic

**Technical Notes:**
- Use Claude's message format: `{"role": "user", "content": "..."}`
- Implement LRU cache for context retrieval
- Context file encrypted at rest (optional, configurable)

---

### Story 4.3: System Prompt & Personality Configuration

**As a** user interacting with Olaf,
**I want** Claude to respond with Olaf's personality and capabilities in mind,
**so that** conversations feel like talking to Olaf, not a generic chatbot.

#### Acceptance Criteria:
1. System prompt template created: `config/system_prompt.yaml`
2. System prompt includes:
   - Olaf's identity: "You are Olaf, a friendly companion robot..."
   - Capabilities: "You can move, express emotions, display information..."
   - Limitations: "You cannot access the internet, you have limited mobility..."
   - Personality traits: "You are curious, helpful, slightly playful..."
   - Response guidelines: "Keep responses concise (2-3 sentences), use simple language..."
3. System prompt supports variable interpolation: `{current_emotion}`, `{battery_level}`, `{location}`
4. Prompt includes current state context:
   - Current expression state (e.g., "You are currently feeling HAPPY at intensity 3")
   - Battery level (e.g., "Your battery is at 45%")
   - Recent actions (e.g., "You just moved to the kitchen")
5. Prompt templates versioned and stored in git
6. Admin can reload prompt without restart: `/olaf/ai/reload_prompt` service
7. A/B testing support: multiple prompt variants selectable via config
8. Prompt effectiveness logged: user satisfaction ratings (optional)

**Technical Notes:**
- Use Jinja2 for template rendering
- System prompt prepended to every Claude API request
- Max system prompt length: 1000 tokens
- Include ethical guidelines (privacy, safety, appropriate responses)

---

### Story 4.4: Local Whisper STT Integration (Hailo-Accelerated)

**As a** user interacting with Olaf hands-free,
**I want** fast, accurate speech recognition running locally on Olaf,
**so that** conversations feel natural and responsive without cloud dependency.

#### Acceptance Criteria:
1. **Hailo AI Kit setup:** Hailo-8L accelerator configured for Whisper inference
2. **Whisper model deployment:** Whisper tiny or base model loaded on Hailo NPU
3. Voice input module created using Hailo-accelerated Whisper for STT
4. Microphone input captured via Raspberry Pi USB mic or I2S mic array
5. Wake word detection: "Hey Olaf" or "Olaf" triggers listening mode (Porcupine or similar)
6. Listening mode indicated by expression change: CURIOUS at intensity 3
7. **STT performance:** Transcribes user speech to text with < 1 second latency (Hailo-accelerated)
8. Transcribed text published to `/olaf/agent/user_input` topic
9. Voice input errors handled gracefully:
   - No speech detected: timeout after 10 seconds, revert to idle
   - Unintelligible speech: trigger CONFUSED expression, prompt retry
   - Ambient noise filtering: reduce false wake word triggers
10. Voice input disabled in Quiet Mode (10 PM - 7 AM)
11. Manual push-to-talk mode: button press activates listening (no wake word)
12. Voice input tested in noisy environments (TV, music, multiple speakers)
13. **Offline capability:** STT works without internet connection

**Technical Notes:**
- Use Hailo Dataflow Compiler to optimize Whisper for Hailo-8L NPU
- Whisper tiny (39M params) or base (74M params) for speed/accuracy balance
- Wake word detection via Porcupine (Picovoice) or alternatives
- Microphone: USB (16kHz, mono) or I2S mic array for better quality
- Implement VAD (Voice Activity Detection) to trigger STT only when speech detected
- Benchmark latency: target <1s from speech end to transcription

---

### Story 4.5: Response-to-Expression Mapping

**As a** user interacting with Olaf via voice,
**I want** Claude responses to trigger appropriate beep/expression combinations,
**so that** I understand Olaf's emotional reaction even without text or speech output.

#### Acceptance Criteria:
1. Response-to-expression mapper analyzes Claude response for:
   - Sentiment (positive, negative, neutral) → maps to emotion type
   - Confidence level → maps to expression intensity (1-5)
   - Response type (answer, question, acknowledgment) → affects expression duration
2. Expression mapping rules:
   - Positive sentiment → HAPPY (intensity based on sentiment score)
   - Negative sentiment → SAD (intensity based on sentiment score)
   - Uncertain/questioning → CURIOUS (intensity 3)
   - Surprised/unexpected info → SURPRISED (intensity 4)
   - Acknowledgment/confirmation → CONTENT (intensity 2)
   - Processing complex info → THINKING (intensity 3)
3. Expression sequence for responses:
   - Response received → trigger mapped expression + beep
   - Hold expression for 3-5 seconds
   - Fade back to CONTENT (idle state)
4. Long responses (> 50 words) trigger extended expression:
   - Initial beep/expression
   - Periodic subtle beeps every 3 seconds to indicate "still responding"
   - Final beep/expression when complete
5. Response logged to `/olaf/ai/response` topic (for debugging/analytics only, not displayed)
6. Tested with 20 response scenarios covering all emotion mappings

**Technical Notes:**
- Use VADER sentiment analysis for response sentiment
- Response stored in memory for context but never displayed to user
- Expression duration scales with response length
- User understands Olaf through expressions alone (personality-first design)

---

### Story 4.6: Conversation Flow Orchestration

**As a** developer integrating all components,
**I want** a conversation orchestrator that coordinates voice input, AI processing, and expression-only responses,
**so that** the full conversation loop works seamlessly.

#### Acceptance Criteria:
1. Conversation orchestrator node created (main conversation loop)
2. Orchestration flow:
   - Wake word detected → trigger CURIOUS expression + beep
   - Capture voice input (STT) → trigger THINKING expression + beep
   - Send text to Claude API → maintain THINKING expression
   - Receive response → analyze sentiment, trigger appropriate expression + beep sequence
   - Hold response expression for 3-5 seconds
   - Return to idle → CONTENT expression
3. Orchestrator subscribes to:
   - `/olaf/conversation/user_input` (from STT)
   - `/olaf/expression/current_state` (for coordination)
4. Orchestrator publishes to:
   - `/olaf/ai/response` (Claude response text, internal only for context/analytics)
   - `/olaf/conversation/state` (idle, listening, thinking, responding)
5. State machine implemented for conversation states
6. Conversation loop tested end-to-end with 20 sample dialogues
7. Error recovery: any component failure returns to idle state
8. Performance metrics logged: total latency from user speech to response expression

**Technical Notes:**
- Use ROS2 state machine library or implement custom
- Target total latency: < 5 seconds (STT 2s + API 3s + expression instant)
- Implement timeout fallbacks for each stage
- Response expression duration: 3-5 seconds based on response complexity
- No visual/audio output of response text - expressions communicate meaning

---

### Story 4.7: Function Calling for Robot Actions

**As a** user asking Olaf to perform physical actions,
**I want** Claude to trigger robot functions (move, look, project),
**so that** Olaf responds with actions, not just words.

#### Acceptance Criteria:
1. Claude function calling implemented with 5 core functions:
   - `move_to(location: str)` → triggers navigation (Epic 5 dependency)
   - `look_at(direction: str)` → pans neck to direction (left, right, up, down)
   - `express_emotion(emotion: str, intensity: int)` → triggers expression
   - `display_info(content: str)` → triggers projection (Epic 6 dependency)
   - `remember(key: str, value: str)` → stores user preference
2. System prompt updated with function definitions
3. Claude responses parsed for function calls (JSON format)
4. Function executor module calls appropriate ROS2 services
5. Function results fed back to Claude for confirmation response
6. Multi-step actions supported: "go to kitchen and show me the weather"
7. Function call logging: track which actions are triggered most
8. Safety checks: validate parameters before execution (e.g., valid locations)
9. Function call tested with 15 command variations
10. Fallback: if function fails, Claude generates apologetic response

**Technical Notes:**
- Use Claude's native function calling (tools parameter)
- Function definitions stored in `config/functions.yaml`
- Implement action queue for sequential function execution
- Dry-run mode for testing without actual robot movement

---

### Story 4.8: Conversational Memory & User Preferences

**As a** user interacting with Olaf over multiple sessions,
**I want** Olaf to remember my preferences and past conversations,
**so that** interactions feel personalized and continuous.

#### Acceptance Criteria:
1. Long-term memory storage: `~/.olaf/memory/user_profile.json`
2. Memory includes:
   - User name (if provided)
   - Preferences: favorite topics, quiet hours, preferred volume
   - Past conversation summaries (key points from last 10 sessions)
   - Learned facts: "user has a cat named Whiskers"
3. Memory updated via function call: `remember(key, value)`
4. Memory recalled automatically in system prompt: "You know the user's name is Alice..."
5. Memory management commands:
   - "What do you remember about me?" → lists stored memories
   - "Forget that I like jazz" → removes specific memory
   - "Forget everything about me" → clears all user data
6. Privacy mode: memory storage disabled when active
7. Memory encryption at rest (optional, configurable)
8. Memory size limit: 5000 tokens, auto-summarize if exceeded
9. Memory tested with multi-session conversation scenarios

**Technical Notes:**
- Use JSON format with versioning for schema updates
- Implement memory summarization via Claude API (weekly)
- GDPR compliance: easy data export/deletion

---

### Story 4.9: Action-Triggered Response Enhancement

**As a** user asking Olaf to perform actions,
**I want** Claude to trigger physical responses (movement, projection) in addition to expressions,
**so that** Olaf communicates through actions, not just emotions.

#### Acceptance Criteria:
1. Response parser identifies action triggers:
   - "show me..." → triggers display_info() function (Epic 6 dependency)
   - "go to..." → triggers move_to() function (Epic 5 dependency)
   - "look at..." → triggers look_at() function (neck movement)
2. Action sequence coordination:
   - Action triggered → appropriate expression plays (e.g., CURIOUS for "looking")
   - Action executes → expression maintained during action
   - Action completes → confirmation expression (HAPPY if success, SAD if failed)
3. Multi-action sequences supported:
   - "Go to the kitchen and show me the time" → navigate + project
   - Actions executed sequentially with appropriate expressions
4. Action feedback via expressions only (no text/voice confirmation)
5. Fallback: if action unavailable, trigger SAD expression + return to idle
6. Tested with 10 action scenarios (movement, projection, look direction)

**Technical Notes:**
- Depends on Epic 5 (navigation) and Epic 6 (projection) for full functionality
- Action queue implements FIFO execution
- Expression provides user feedback on action status

---

### Story 4.10: Conversation Analytics & Improvement

**As a** developer improving Olaf's conversational abilities,
**I want** detailed analytics on conversation patterns and failures,
**so that** I can identify and fix common issues.

#### Acceptance Criteria:
1. Conversation analytics logger tracks:
   - Total conversations per day/week/month
   - Average conversation length (number of exchanges)
   - Most common user intents (extracted from transcripts)
   - API latency breakdown (STT, Claude, expression response)
   - Error rates by component (STT failures, API errors, expression issues)
   - Function call frequency and success rate
   - User satisfaction signals (e.g., "thank you" vs. "never mind")
2. Analytics dashboard accessible via web UI: `http://localhost:8080/conversation-analytics`
3. Dashboard visualizations:
   - Conversation volume over time (line chart)
   - Intent distribution (pie chart)
   - Latency heatmap (by time of day)
   - Error log (filterable table)
4. Export analytics to CSV for external analysis
5. Privacy-preserving: transcript storage optional, anonymization available
6. Analytics used to identify improvement areas (e.g., high STT error rate → retune wake word)
7. Monthly summary report auto-generated

**Technical Notes:**
- Use Flask + Chart.js for dashboard
- Store analytics in SQLite database
- Implement data retention policy (90 days default)

---

### Story 4.11: Documentation & Content Creation

**As a** community member interested in Olaf's AI capabilities,
**I want** comprehensive documentation and demo videos,
**so that** I can understand and extend the conversational AI system.

#### Acceptance Criteria:
1. Wiki pages created:
   - Conversational AI architecture diagram
   - Claude API setup guide (API key, config)
   - System prompt customization guide
   - Function calling developer guide
   - Voice input/output troubleshooting
   - Privacy and data retention policies
2. README.md updated with Epic 4 completion status
3. Code comments added to all AI-related modules
4. YouTube video recorded (10-15 minutes):
   - Voice conversation demo (wake word → expression/beep response only)
   - Function calling showcase (movement, expressions triggered by speech)
   - Context retention demo (multi-turn conversation via expressions)
   - Memory system demo (remembering preferences)
   - Behind-the-scenes: API flow, expression mapping, latency breakdown
5. LinkedIn post with conversation highlights and video link
6. GitHub discussion thread: "What should Olaf learn to do next?"
7. All Epic 4 code merged to main branch with passing CI/CD

**Technical Notes:**
- Record video in quiet environment for clear audio
- Use subtitles to show STT transcription accuracy
- Include B-roll of conversation flow diagram
- Demonstrate both successful and error-recovery scenarios

---

## Definition of Done

- [ ] All 11 user stories completed with passing acceptance criteria
- [ ] Claude API integration functional with error handling and retry logic
- [ ] Voice input (STT) with wake word detection operational
- [ ] Response-to-expression mapping triggers appropriate beep/emotion combinations
- [ ] Conversation orchestrator coordinates full loop (voice → AI → expression/beep response only)
- [ ] Function calling enables robot actions via natural language
- [ ] Long-term memory stores user preferences across sessions
- [ ] Action-triggered responses combine expressions with physical actions (movement, projection)
- [ ] Conversation analytics dashboard tracks performance and errors
- [ ] End-to-end latency < 5 seconds (wake word → expression response)
- [ ] 20 conversation scenarios tested successfully (expression-only responses)
- [ ] Documentation complete with setup guides and troubleshooting
- [ ] YouTube video published showcasing conversational AI (expression-based communication)
- [ ] All code reviewed, commented, and merged to main branch

---

## Success Metrics

- **Conversation Latency:** < 3 seconds total (Whisper STT <1s + Cloud agent <2s + expression instant)
- **STT Accuracy:** > 95% wake word detection, > 90% transcription accuracy (Hailo-accelerated Whisper)
- **Agent Reliability:** > 99% uptime, < 1% error rate
- **Tool Use Success:** > 95% valid tool calls executed correctly
- **Expression Mapping:** > 90% appropriate emotion triggered for agent responses
- **Context Retention:** 10+ exchanges maintained without loss
- **Multi-step Planning:** Agent successfully executes 3-step action sequences
- **Offline STT:** Whisper operates without internet, latency <1s
- **User Engagement:** Average 5+ exchanges per conversation (expression-based feedback)
- **Community Engagement:** ≥ 200 YouTube views, ≥ 50 LinkedIn reactions within first week (agent framework demo)

---

## Technical Dependencies

- Epic 1: ROS2 foundation, orchestrator
- Epic 3: Expression system (beep/emotion-driven responses)
- **Hailo AI Kit:** Raspberry Pi 5 AI Hat with Hailo-8L NPU (13 TOPS)
- **Python libraries:** `anthropic`, `openai-whisper`, `porcupine`, `jinja2`, `flask`, `vaderSentiment`
- **Agent framework:** TBD (Pydantic AI, LangGraph, or custom implementation)
- **Hailo tools:** Hailo Dataflow Compiler, Hailo Python API
- **Hardware:**
  - Raspberry Pi 5 8GB + Hailo AI Kit
  - USB microphone or I2S mic array (connected to Raspberry Pi)
  - Passive piezo buzzer (for beeps)
- **Cloud AI:** Claude API key (requires Anthropic account)
- **Whisper model:** Tiny (39M) or Base (74M) optimized for Hailo-8L

---

## Privacy & Security Notes

- **API Key Security:** Stored with 600 permissions, never logged
- **Conversation Privacy:** Transcripts stored locally, optional encryption
- **Data Retention:** 90-day default, configurable
- **GDPR Compliance:** User can export/delete all data on request
- **Privacy Mode:** Disables context saving and long-term memory
- **Anonymization:** Analytics can strip personally identifiable information
