# Natural Language Processing

This repo uses [spaCy](https://spacy.io/) by OpenAI and on [Stanza](https://stanfordnlp.github.io/stanza/) by Stanford NLP Group for syntactic structure of sentences.

## Setup

1. Run ```pip install -r requirements.txt```


## Conceptual Dependencies Server

### Structure

**Topics**

```
/conceptual_dependencies/final_result
```

of type `conceptual_deps/StringArray`

where StringArray.msg:

```
string[]  data
```

**Services**

```
/conceptual_dependencies/condep_service
```

of type `conceptual_deps/GetConDep` 

where GetConDep.srv:

```
---
StringArray  cds
```

and 

```
/conceptual_dependencies/text_condep_service
```

of type `conceptual_deps/GetTextConDep` 

where GetTextSpeech.srv:

```
string text
---
StringArray  cds
```

### Initialization

To start a service (only one at a time), run:

**Whisper**

```
roslaunch conceptual_deps condep_whisper.launch
```

**Vosk**

```
roslaunch conceptual_deps condep_vosk.launch
```

### Usage

**1. Command line example**

In one terminal, subsribe to the topic:

```
rostopic echo /conceptual_dependencies/final_result
```

And in a different terminal, call the service:

```
rosservice call /conceptual_dependencies/condep_service "{}"
```

or

```
rosservice call /conceptual_dependencies/text_condep_service "text: 'fake sentence'"
```

**2.Client Example**

In one terminal start the client:

```
rosrun conceptual_deps condep_client.py
```

And in a different terminal, call it:

```
rostopic pub /conceptual_dependencies/condep_text_client std_msgs/String "data: 'fake sentence'"
```
Read the code in the file `condep_client.py` for details.
