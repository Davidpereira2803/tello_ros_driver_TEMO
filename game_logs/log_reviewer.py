import csv
from datetime import datetime

log_file = "/home/david/Projects/TEMO_ros_ws/src/tello_ros_driver_TEMO/game_logs/ps4_game_log_2025-06-24_1750778521000.csv"

game_start = None
game_end = None
score = 0
shots = 0
reloads = 0
hits = 0

with open(log_file, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        event = row['Event'].strip()
        timestamp = row['Timestamp'].strip()
        details = row['Details'].strip()

        if event == "Game_Start_Time" and timestamp:
            game_start = int(timestamp)
        elif event == "Game_End_Time" and timestamp:
            game_end = int(timestamp)
        elif event == "Shot_Fired":
            shots += 1
        elif event == "Target_Hit":
            hits += 1
        elif event == "Reload":
            reloads += 1

duration_ms = game_end - game_start if game_start and game_end else 0
duration_sec = duration_ms / 1000
minutes = int(duration_sec // 60)
seconds = int(duration_sec % 60)
accuracy = (hits / shots) * 100 if shots > 0 else 0
score = hits

start_time_str = datetime.fromtimestamp(game_start / 1000).strftime("%Y-%m-%d %H:%M:%S") if game_start else "Unknown"
end_time_str = datetime.fromtimestamp(game_end / 1000).strftime("%Y-%m-%d %H:%M:%S") if game_end else "Unknown"

print("Game Summary")
print(f"Start Time   : {start_time_str}")
print(f"End Time     : {end_time_str}")
print(f"Duration     : {minutes} min {seconds} sec")
print(f"Total Shots  : {shots}")
print(f"Total Reloads: {reloads}")
print(f"Hits         : {hits}")
print(f"Score        : {score}")
print(f"Accuracy     : {accuracy:.2f}%")
