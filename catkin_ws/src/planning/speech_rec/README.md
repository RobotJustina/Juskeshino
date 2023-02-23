# Speech Recognition
This repo is based on [Whisper](https://github.com/openai/whisper) by OpenAI and on [whisper-mic](https://github.com/mallorbc/whisper_mic).  Also, it's based on [Vosk](https://alphacephei.com/vosk/) by Alpha-cephei an on [ros-vosk](https://github.com/alphacep/ros-vosk).

## Setup

1. Run ```pip install -r requirements.txt```


## Speech Recognition Server

### Structure

**Topics**

```
/speech_recognition/final_result
```

of type `std_msgs/String`


**Services**

```
/speech_recognition/vosk_service
```

or 

```
/speech_recognition/whisper_service
```

of type `ros_whisper_vosk/GetSpeech` 

where GetSpeech.srv:

```
---
string  data
```

And

```
/speech_recognition/fake_vosk_service
```

or 

```
/speech_recognition/fake_whisper_service
```

of type `ros_whisper_vosk/FakeGetSpeech` 

where GetSpeech.srv:

```
string text
---
string  data
```

### Initialization

To start a service (only one at a time), in a different terminal run:

**Whisper**

```
rosrun ros_whisper_vosk whisper_service.py --english
```

**Vosk**

```
rosrun ros_whisper_vosk vosk_service.py
```

### Usage

In one terminal, subsribe to the topic:

```
rostopic echo /speech_recognition/final_result
```

**Command line Vosk example**

And in a different terminal, call the service:

```
rosservice call /speech_recognition/vosk_service "{}"
```

or

```
rosservice call /speech_recognition/fake_vosk_service "text: 'fake sentence'"
```

**Command line Whisper example**

And in a different terminal, call the service:

```
rosservice call /speech_recognition/whisper_service "{}"
```

or

```
rosservice call /speech_recognition/fake_whisper_service "text: 'fake sentence'"
```
