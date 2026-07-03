import os

file_path = r"C:\Users\DELL\.gemini\antigravity\brain\2997c274-70b8-4262-b5d7-451899d0a010\uploaded_media_1782978565273.img"
if os.path.exists(file_path):
    with open(file_path, "rb") as f:
        header = f.read(32)
        print("Header:", header)
        print("Header (hex):", header.hex())
else:
    print("File not found")
