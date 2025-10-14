# ESP32_AudioStreaming

Purpose: - listen, stream and capture high quality enviromental audio over internet, sounds of nature, temperature, humidity and pressure of certain place.

How to use : - point your browser to http://<DHCP_allocated_IP of ESP32 or http://<DHCP_allocated_IP/wav using VLC media player

BOM:  - ESP32 Wroom - Developement board
      - ICS 43434 MEMS I2S digital microphone
      - BMP/BME280 temperature sensor

ToDO: - test Opus adaptive variable bitrate codec
      - switch between WAV/Opus streaming
      - design STL printable enclosure
      - store and graph temp/hum/pressure evolution by day/week/month/year
      - control loads/perform actions via 1-4 relay switches
      - add UTC time to interface
      - (add propagation conditions to interface)
      and more.. 
