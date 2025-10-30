# AI Agents Layer

This directory will contain the AI agentic development layer for OLAF.

**Status:** Architecture in design phase

## Planned Capabilities

- **Conversational AI:** Claude/GPT integration for natural dialogue
- **Personality Generation:** Emotion mapping and expression coordination
- **Speech Processing:**
  - Speech-to-text (Whisper on Hailo accelerator)
  - Text-to-speech synthesis
- **Context Management:** Conversation memory and state tracking
- **Decision Making:** High-level behavioral planning

## Integration Approach

**Separation of Concerns:**
- **This layer (`agents/`):** AI reasoning, natural language, decision-making
- **ROS2 layer (`ros2/`):** Hardware coordination, motion control, SLAM
- **Firmware layer (`firmware/`):** Real-time hardware control, sensors

**Communication:**
- Agents use `rclpy` to publish/subscribe to ROS2 topics
- Pure Python services/modules (not ROS2 packages)
- Enables standalone AI development and testing

## Design Philosophy

1. **Decouple Intelligence from Coordination**
   - AI agents focus on "what to do"
   - ROS2 orchestrator focuses on "how to do it"

2. **Enable Standalone Development**
   - Test AI behavior without hardware
   - Iterate quickly on conversation logic
   - Mock ROS2 interfaces for unit testing

3. **Flexible Architecture**
   - Not locked into ROS2 as sole integration method
   - Can evolve independently as AI capabilities advance
   - Easier to swap LLM providers or add new AI services

## Future Structure (Proposed)

```
agents/
├── personality/           # Emotion mapping, personality traits
│   ├── emotion_mapper.py
│   └── personality_config.yaml
├── conversation/          # Conversational AI integration
│   ├── ai_agent.py       # Main AI agent service
│   ├── claude_client.py  # Anthropic Claude API
│   └── gpt_client.py     # OpenAI GPT fallback
├── stt/                   # Speech-to-text
│   └── whisper_hailo.py  # Hailo-accelerated Whisper
├── tts/                   # Text-to-speech
│   └── tts_engine.py
├── context/               # Conversation memory
│   └── memory_manager.py
├── ros_bridge.py          # ROS2 topic interface
└── requirements.txt       # Python dependencies
```

## Integration Example

```python
# agents/conversation/ai_agent.py (conceptual)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIAgent:
    def __init__(self):
        self.node = Node('ai_agent')

        # Subscribe to user input (from STT)
        self.node.create_subscription(
            String, '/olaf/user_input', self.on_user_input, 10
        )

        # Publish emotion commands
        self.emotion_pub = self.node.create_publisher(
            String, '/olaf/ai/emotion_command', 10
        )

    def on_user_input(self, msg):
        # Process with Claude/GPT
        response, emotion = self.generate_response(msg.data)

        # Publish emotion to orchestrator
        self.emotion_pub.publish(String(data=f"{emotion},3"))
```

---

**To be designed and implemented in future iterations.**

**Last Updated:** 2025-10-30
**Architecture Review:** Pending
