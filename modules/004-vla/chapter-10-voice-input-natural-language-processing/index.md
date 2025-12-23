# Chapter 10: Voice Input and Natural Language Processing

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the fundamentals of OpenAI Whisper for voice processing
- Set up voice input pipelines for humanoid robots
- Implement speech-to-text conversion with accuracy optimization
- Apply natural language understanding techniques
- Validate voice commands and handle errors appropriately

## 10.1 Introduction to OpenAI Whisper for Voice Processing

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system that converts spoken language into written text. For humanoid robotics, Whisper enables natural voice interaction, allowing users to communicate with robots using everyday language.

### Key Features of Whisper:
- **Multilingual Support**: Capable of recognizing and transcribing multiple languages
- **Robustness**: Handles various accents, background noise, and audio quality conditions
- **Accuracy**: High transcription accuracy even in challenging conditions
- **Real-time Processing**: Can operate in real-time for interactive applications

### Whisper Model Variants:
- **Tiny**: Fastest but least accurate, suitable for real-time applications
- **Base**: Good balance of speed and accuracy
- **Small**: Better accuracy with moderate processing time
- **Medium**: High accuracy for most applications
- **Large**: Highest accuracy but requires more computational resources

## 10.2 Setting Up Voice Input Pipelines

To implement voice input for humanoid robots, you need to establish a complete audio processing pipeline:

### Audio Capture Configuration:
- **Microphone Selection**: Choose appropriate microphones for the robot's form factor
- **Audio Quality**: Configure sampling rate (typically 16kHz) and bit depth
- **Noise Reduction**: Implement preprocessing to reduce ambient noise
- **Audio Buffering**: Set up circular buffers for continuous audio capture

### Example Audio Capture Setup:
```python
import pyaudio
import wave
import numpy as np

# Audio configuration
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024

# Initialize audio interface
audio = pyaudio.PyAudio()

# Create audio stream
stream = audio.open(
    format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=True,
    frames_per_buffer=CHUNK
)

print("Audio input initialized for voice processing")
```

## 10.3 Speech-to-Text Conversion and Accuracy Optimization

Whisper's speech-to-text conversion involves several optimization techniques to maximize accuracy:

### Model Selection Strategy:
- Use larger models for applications requiring high accuracy
- Use smaller models for real-time applications with strict latency requirements
- Consider computational constraints of the humanoid robot platform

### Accuracy Enhancement Techniques:
- **Prompt Engineering**: Provide context to guide transcription
- **Language Specification**: Specify the language for better accuracy
- **Temperature Control**: Adjust generation parameters for consistency
- **Batch Processing**: Process longer audio segments for better context

### Example Whisper Integration:
```python
import whisper

# Load Whisper model
model = whisper.load_model("medium")

# Transcribe audio file
result = model.transcribe("audio_input.wav")

# Access transcription
transcription = result["text"]
confidence = result["text"]  # Note: Whisper doesn't provide confidence scores directly

print(f"Transcribed text: {transcription}")
```

## 10.4 Natural Language Understanding Techniques

After converting speech to text, natural language understanding (NLU) extracts meaning and intent:

### Intent Recognition:
- **Command Classification**: Identify robot commands from transcribed text
- **Entity Extraction**: Extract specific objects, locations, or parameters
- **Contextual Understanding**: Interpret commands based on previous interactions

### Example NLU Implementation:
```python
import spacy
from transformers import pipeline

# Load NLP model
nlp = spacy.load("en_core_web_sm")

# Set up intent classification
classifier = pipeline("text-classification",
                     model="microsoft/DialoGPT-medium")

def extract_intent(text):
    doc = nlp(text)

    # Extract entities
    entities = [(ent.text, ent.label_) for ent in doc.ents]

    # Determine intent based on keywords and context
    intent = classify_command(text)

    return {
        "intent": intent,
        "entities": entities,
        "original_text": text
    }

def classify_command(text):
    # Simple keyword-based classification
    if any(word in text.lower() for word in ["move", "go", "walk", "navigate"]):
        return "navigation"
    elif any(word in text.lower() for word in ["pick", "grasp", "take", "grab"]):
        return "manipulation"
    elif any(word in text.lower() for word in ["stop", "halt", "pause"]):
        return "stop"
    else:
        return "unknown"
```

## 10.5 Voice Command Validation and Error Handling

Robust voice processing systems include validation and error handling:

### Validation Strategies:
- **Grammar Checking**: Verify commands follow expected patterns
- **Context Validation**: Ensure commands make sense in current context
- **Safety Validation**: Check for potentially unsafe commands
- **Ambiguity Resolution**: Handle unclear or ambiguous commands

### Error Handling Techniques:
- **Confidence Thresholding**: Only process transcriptions above confidence threshold
- **Re-prompting**: Request clarification for unclear commands
- **Fallback Mechanisms**: Provide alternative input methods
- **Error Logging**: Track common errors for system improvement

## 10.6 Integration with ROS 2

Voice processing systems integrate with ROS 2 for command execution:

### ROS 2 Message Types:
- **Audio Data**: Custom message types for audio streams
- **Transcription Results**: Standard string messages with confidence scores
- **Command Messages**: Structured messages for robot commands

### Example ROS 2 Node:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class VoiceProcessorNode(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Publishers and subscribers
        self.transcription_pub = self.create_publisher(String, 'transcription', 10)
        self.command_pub = self.create_publisher(String, 'robot_command', 10)
        self.audio_sub = self.create_subscription(AudioData, 'audio_input', self.audio_callback, 10)

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("medium")

    def audio_callback(self, msg):
        # Process audio data and transcribe
        transcription = self.transcribe_audio(msg.data)

        # Publish transcription
        trans_msg = String()
        trans_msg.data = transcription
        self.transcription_pub.publish(trans_msg)

        # Process command
        self.process_command(transcription)

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceProcessorNode()
    rclpy.spin(voice_node)
    voice_node.destroy_node()
    rclpy.shutdown()
```

## Exercises

1. Set up a basic Whisper transcription pipeline with audio input
2. Implement intent recognition for common robot commands
3. Add confidence thresholding to filter low-quality transcriptions
4. Create a ROS 2 node that processes audio input and publishes transcriptions

## Summary

This chapter introduced OpenAI Whisper for voice processing in humanoid robotics. You learned how to set up audio input pipelines, optimize speech-to-text conversion, apply natural language understanding techniques, and integrate with ROS 2. These capabilities form the foundation for natural language interaction with humanoid robots.