# デモ用モデル読み込み

import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import torch.optim as optim
import torchaudio
import torchaudio.functional as Fa
import torch.nn.functional as F
import torchaudio.transforms as T
import os
import glob
# from IPython.display import Audio, display
# import pandas as pd
# from tqdm.notebook import tqdm
import matplotlib.pyplot as plt
from torch.nn.utils.rnn import pad_sequence
from torch.nn.utils.rnn import pack_padded_sequence
from torch.nn.utils.rnn import pad_packed_sequence
import torchvision.transforms as transforms
import torchvision.models as models
# import parselmouth
import numpy as np
import csv
import librosa
# import librosa.display
import re
import json
import time
# from einops import rearrange, repeat
# from einops.layers.torch import Rearrange
import math
# import h5py
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor

# デモで使うモデル
# wav2vecと自作modelの2段構成
class ModelTurntaking:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(self.device)

        # wav2vecの準備
        MODEL_ID = "jonatasgrosman/wav2vec2-large-xlsr-53-japanese"
        self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(MODEL_ID)
        self.wav2vec = Wav2Vec2ForSequenceClassification.from_pretrained(MODEL_ID, output_hidden_states=True).to(self.device)

        self.model = self.model_load()

        n = self.count_parameters(self.model)
        print("Number of parameters: %s" % n)

    def count_parameters(self, model):
        return sum(p.numel() for p in model.parameters() if p.requires_grad)

    def get_likely_index(self, tensor):
        # バッチの各要素について、最も確率の高いラベルインデックスを得る
        return tensor.argmax(dim=-1)

    def model_load(self):
        print(torch.__version__)
        print(torchaudio.__version__)
        # 日本語訳注：cudaと出力されればOKです

        model = self.wav2vec
        model.to(self.device)

        model_path = 'model.pth'
        model.load_state_dict(torch.load(model_path))
        print(torch.load(model_path).keys())

        return model

    def judge_turntaking(self, sound_array):
        inputs = self.feature_extractor(sound_array, sampling_rate=16000, return_tensors="pt")
        inputs = inputs.to(self.device)
        with torch.no_grad():
            output = self.model(**inputs)
            output = output.logits

        
        pred = self.get_likely_index(output)
        softmax_func = nn.Softmax(dim=1)
        if softmax_func(output)[0][0] > 0.75:
            pred = 0
        else:
            pred = 1

        return pred, softmax_func(output)