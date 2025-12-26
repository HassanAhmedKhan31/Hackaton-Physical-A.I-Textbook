# The Robot's Ears (Speech Recognition)

The first step in interaction is hearing. We need to convert sound waves (analog air pressure) into text (digital strings). For this, we use **OpenAI Whisper**, a state-of-the-art Automatic Speech Recognition (ASR) system.

## 1. Why Whisper?
Whisper is robust. It works in noisy environments, understands accents, and handles technical jargon better than older systems.

*   **Cloud API:** Highly accurate, requires internet, costs money per minute.
*   **Local (Whisper.cpp):** Free, private, runs on the robot's GPU, but might be slower or less accurate on low-end hardware.

For this course, we will use the Cloud API for simplicity, but the code structure is identical for local inference.

## 2. Practical: Recording Audio
First, we need a Python script to capture audio from the robot's microphone.

```python
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav

def record_audio(duration=5, fs=44100):
    print("Recording...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()  # Wait until recording is finished
    print("Finished.")
    wav.write('command.wav', fs, recording)
```

## 3. sending to Whisper
Once we have `command.wav`, we send it to the API.

```python
import openai

def transcribe(filename):
    audio_file = open(filename, "rb")
    transcript = openai.Audio.transcribe("whisper-1", audio_file)
    return transcript["text"]
```

## 4. Multilingual Support (The Translation Bonus)
One of the most powerful features of Whisper is its ability to understand **99 languages** and translate them to English automatically.

### The Urdu Feature
Imagine you are deploying this robot in a region where English is not the primary language.
*   **User:** "Robot, agay jao" (Urdu: آگے جاؤ)
*   **Whisper API:** You can set the `translate` flag. Whisper hears Urdu but outputs the English text: "Robot, go forward."
*   **Robot Brain:** The robot receives "Go forward," which matches its existing logic. No code changes are needed in the navigation stack!

```python
transcript = openai.Audio.translate("whisper-1", audio_file) # .translate instead of .transcribe
print(transcript["text"]) # Outputs: "Go forward"
```

This effectively gives your robot a "Universal Translator," allowing it to serve a global audience instantly.

## 5. ROS 2 Integration
We wrap this logic in a ROS 2 Lifecycle Node.
1.  **State:** The node waits in `IDLE`.
2.  **Trigger:** It detects a "Wake Word" (like "Hey Robot") or a button press.
3.  **Action:** It records, transcribes, and publishes the string to the `/human_command` topic.

This decoupling allows us to swap out the microphone hardware or the ASR provider without breaking the rest of the robot's brain.