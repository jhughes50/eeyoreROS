import os


if os.path.exists("/dev/boson_ser") and os.path.exists("/dev/boson_video"):    
    print("--device /dev/boson_video -v /dev/boson_video:/dev/boson_video --device /dev/boson_ser -v /dev/boson_ser:/dev/boson_ser")

