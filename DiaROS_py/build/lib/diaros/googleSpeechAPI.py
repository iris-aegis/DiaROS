#export GOOGLE_APPLICATION_CREDENTIALS="credential.json"
#or
#set GOOGLE_APPLICATION_CREDENTIALS=C:\Users\81802\voice-dialogue\credential.json

from google.cloud import speech_v1p1beta1 as speech

class GoogleSpeechAPI:
    def __init__(self, sample_rate):
        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=sample_rate,
            language_code='ja-JP',
            max_alternatives=1)

        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            interim_results=True)

    def recognize(self, audio_generator):
        requests = (speech.StreamingRecognizeRequest(
                    audio_content=content)for content in audio_generator)
        return self.client.streaming_recognize(self.streaming_config, requests)