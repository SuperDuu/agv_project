try:
    import speech_recognition as sr
    r = sr.Recognizer()
    with sr.AudioFile(r"c:\Users\DELL\agv_project\arm\scratch\audio.wav") as source:
        audio = r.record(source)
    try:
        text = r.recognize_google(audio, language="vi-VN")
        print("Transcript:", text)
    except Exception as e:
        print("Google speech error:", e)
except ImportError:
    print("speech_recognition not installed")
