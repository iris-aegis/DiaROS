# Speech Output Interface (Iso)
# DMからSpeechOutputへの音声再生リクエスト

string audio_type           # "backchannel" | "response"
string stage                # "first" | "second"
string wav_path             # 音声ファイルパス
float64 duration_sec        # 音声長（秒）
int32 request_id            # リクエストID
int64 timestamp_ns          # タイムスタンプ（ナノ秒）
string session_id           # セッションID
