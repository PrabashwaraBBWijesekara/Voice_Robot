#!/usr/bin/env python3

import pyaudio
import time
import wave
from google.cloud import speech
import io
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

# Audio recording settings
chunk = 1024
format = pyaudio.paInt16
channels = 1
rate = 44100
Output_Filename = "/home/prabashwara/Recorded.wav"

class SpeechRecognizerNode(Node):

    def __init__(self):
        super().__init__('speech_recognizer')
        self.publisher = self.create_publisher(String, '/recognizer/output', 10)
        self.get_logger().info("Speech Recognizer Node has started.")
    
    def speech_to_text(self, audio_file):
        try:
            client = speech.SpeechClient()

            # Read the audio file
            with io.open(audio_file, "rb") as audio:
                content = audio.read()

            # Configure recognition
            config = speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=rate,
                language_code="en-US",
            )

            # Create RecognitionAudio instance
            audio_content = speech.RecognitionAudio(content=content)

            # Call the API
            response = client.recognize(config=config, audio=audio_content)

            # Publish the transcribed text to the /recognizer/output topic
            for result in response.results:
                transcript = result.alternatives[0].transcript
                self.get_logger().info(f"Transcript: {transcript}")

                # Create and publish a message
                msg = String()
                msg.data = transcript
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in speech-to-text: {e}")

    def start_recording(self):
        p = pyaudio.PyAudio()

        stream = p.open(format=format,
                        channels=channels,
                        rate=rate,
                        input=True,
                        frames_per_buffer=chunk)

        frames = []
        recording_event = threading.Event()

        # Listener thread to stop recording
        def stop_recording_listener():
            input()  # Wait for Enter key to stop
            self.get_logger().info("Stopping recording")
            recording_event.set()

        self.get_logger().info("Press Enter to start recording")
        input()  # Wait for Enter key to start
        self.get_logger().info("Recording... Press Enter to stop.")

        # Start the listener thread
        listener_thread = threading.Thread(target=stop_recording_listener)
        listener_thread.start()

        # Record audio until the event is set
        while not recording_event.is_set():
            try:
                data = stream.read(chunk, exception_on_overflow=False)
                frames.append(data)
            except Exception as e:
                self.get_logger().error(f"Error during audio recording: {e}")
                break

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the recorded audio
        try:
            with wave.open(Output_Filename, 'wb') as wf:
                wf.setnchannels(channels)
                wf.setsampwidth(p.get_sample_size(format))
                wf.setframerate(rate)
                wf.writeframes(b''.join(frames))

            self.get_logger().info(f"Audio recorded and saved to {Output_Filename}")
            self.speech_to_text(Output_Filename)
        except Exception as e:
            self.get_logger().error(f"Error saving audio: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizerNode()

    try:
        while rclpy.ok():
            node.start_recording()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
