import os
import random
import shutil

def randomize_and_save_files(source_folder, output_folder, file_prefix, num_files):
    # フォルダが存在しない場合は作成
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # ファイルリストの取得
    file_list = [f for f in os.listdir(source_folder) if f.endswith('.wav')]

    # ランダムに入れ替え
    random.shuffle(file_list)

    # ファイルをコピーして保存
    for i in range(num_files):
        source_file = os.path.join(source_folder, file_list[i])
        output_file = os.path.join(output_folder, f"{file_prefix}{i + 1}.wav")
        shutil.copyfile(source_file, output_file)
        print(f"Copying {source_file} to {output_file}")

if __name__ == "__main__":
    source_folder = "static_response_source"
    output_folder = "static_response_random"
    file_prefix = "static_response_random_"
    num_files = 60

    randomize_and_save_files(source_folder, output_folder, file_prefix, num_files)