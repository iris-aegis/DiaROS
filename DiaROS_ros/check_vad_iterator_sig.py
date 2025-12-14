import torch
import inspect
import sys

try:
    silero_vad_model, utils = torch.hub.load(
        repo_or_dir='snakers4/silero-vad',
        model='silero_vad',
        force_reload=False,
        onnx=False
    )
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    
    print("VADIterator __init__ arguments:")
    print(inspect.signature(VADIterator.__init__))
    
except Exception as e:
    print(f"Error: {e}")
