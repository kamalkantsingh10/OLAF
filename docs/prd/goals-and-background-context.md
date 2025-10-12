# Goals and Background Context

## Goals

- **Bring AI agents to physical life:** Create an embodied AI platform where AI agents (Claude, GPT-4) gain physical form with personality expression
- Deliver a functional V1 modular personal assistant robot with engaging personality expression (OLED eyes, articulated ears/neck, R2D2-style beeping)
- **Enable hybrid AI architecture:** Local AI acceleration (Hailo AI Kit + Whisper STT) for fast sensing + cloud agents for reasoning
- **Implement agent framework:** Tool use, function calling, multi-step planning in physical space (framework TBD during architecture phase)
- Implement autonomous SLAM navigation for mobility around indoor apartment environments
- Provide practical information display via floor projection system
- Prove modular MECE architecture enabling independent weekend-sprint development of each module
- Create complete build-in-public documentation (3D files, BOM, wiring diagrams, setup guides) for community replication
- Establish credibility in embodied AI space through consistent weekly content (LinkedIn posts, monthly YouTube videos)
- Enable daily meaningful interactions (2-3+ per day) making Olaf a genuine companion, not a desk toy

## Background Context

AI agents are rapidly evolving with impressive reasoning capabilities, but they remain trapped in text interfaces and voice-only speakers. Meanwhile, existing robotics projects force impossible choices: commercial robots are expensive closed-source black boxes, educational platforms lack modern AI agent integration, functional ROS2 robots are purely utilitarian, and simple hobbyist kits sacrifice all advanced capabilities. **No open-source framework exists for bringing AI agents to physical life** with personality, modular architecture, local AI acceleration, and maker-accessible budget (~$400-$1000).

Olaf bridges this gap as an **embodied AI agent platform**. Built around a three-layer architecture (Module Layer with independent ESP32-powered hardware, Orchestration Layer on Raspberry Pi 5 8GB + Hailo AI Kit coordinating behaviors, Intelligence Layer via hybrid local + cloud AI), Olaf gives AI agents physical form. Local Whisper STT (Hailo-accelerated) provides fast speech recognition, cloud AI agents (Claude/GPT-4) handle reasoning and tool use, while personality orchestration coordinates multiple expression channels (eyes, ears, neck, beeps) to create emergent character. The MECE principle ensures each module owns its domain exclusively, enabling weekend development sprints, independent testing, and progressive capability enhancement.

```
┌─────────────────────────────────────────────────────────────┐
│              INTELLIGENCE LAYER (Hybrid AI)                 │
│  Local: Whisper STT (Hailo) | Cloud: Agent (Claude/GPT-4)  │
│  • Speech Recognition (local, fast)                         │
│  • Agent Reasoning & Tool Use (cloud)                       │
│  • Multi-step Planning & Context                            │
└──────────────────────┬──────────────────────────────────────┘
                       │ HTTPS/REST + Local Inference
┌──────────────────────▼──────────────────────────────────────┐
│             ORCHESTRATION LAYER                             │
│    (Raspberry Pi 5 8GB + Hailo AI Kit - Python/ROS2)       │
│  • Agent Orchestration      • SLAM Navigation               │
│  • Personality Coordination • Sensor Fusion                 │
│  • Local AI Inference       • State Management              │
│  • Module Discovery         • Tool Execution                │
└──┬────────┬────────┬────────┬────────┬──────────────────────┘
   │        │        │        │        │
   │ ROS2   │ ROS2   │ ROS2   │ ROS2   │ ROS2 Topics
   │ Topic  │ Topic  │ Topic  │ Topic  │ Topic
   │        │        │        │        │
┌──▼───┐ ┌─▼────┐ ┌─▼────┐ ┌─▼──────┐ ┌▼─────┐
│ HEAD │ │ EARS │ │ NECK │ │PROJECT-│ │ BASE │
│      │ │      │ │      │ │  OR    │ │      │
│ESP32 │ │ESP32 │ │ESP32 │ │ ESP32  │ │ESP32 │
└──┬───┘ └──┬───┘ └──┬───┘ └───┬────┘ └──┬───┘
   │        │        │         │         │
┌──▼────────▼────────▼─────────▼─────────▼────┐
│          MODULE LAYER (Hardware)             │
│  • OLED Eyes      • 2-DOF Ears (2x)          │
│  • RGBD Camera    • 3-DOF Neck               │
│  • Microphone     • Floor Projector          │
│  • mmWave Sensor  • Hoverboard Motors        │
│  • Speaker/Beeps  • LED Indicators           │
└──────────────────────────────────────────────┘
```

This project validates that **AI agents can be brought to physical life** with personality, local AI acceleration for responsiveness, and embodied intelligence—built transparently so the maker community can replicate, extend, and contribute.

## Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-10-10 | v1.0 | Initial PRD draft | John (PM Agent) |

---
