##  靈巧手採集壓力手套
JQ Industries Fabric E-Skin Python Serial SDK


https://jq-industries.com/

https://glove.jq-industries.io/

Docs.   
https://drive.google.com/drive/folders/1Ftpt9zI-pt2uKs1Tzs6vsckd_4OFVVIm?usp=sharing

### USAGE    
pip install pyserial    

List USB port  e.g. Linux  /dev/ttyUSB0, Windows COM8 COM9    

```bash=
python jq_eskin_serial.py --list    
sudo chmod 666 /dev/ttyUSB0    
python jq_eskin_serial.py --port /dev/ttyUSB0 --raw    
```

Test  Left hand
```bash=
python jq_LeftHand.py
```


