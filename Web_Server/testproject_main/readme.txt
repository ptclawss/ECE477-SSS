Admin credentials:
ID:team8
PW:purdueteam8
email: nguye683@purdue.edu

pip install django
pip install qrcode[pil]

When making changes to models and data bases:
python manage.py makemigrations
python manage.py migrate

When Running Server, Compile:
python manage.py runserver

When Running Admin Privileges:
python manage.py createsuper

If running based on Default IP:ss
http://127.0.0.1:8000/seat_app/seat_status/
http://127.0.0.1:8000/admin/


How to run same network but different devices:
ipconfig (windows) ifconfig (linux) to get machine server ip (ip of the machine that runs server)
change the allowed host: add this ip address into the allowed host in settings.py
python manage.py runserver 0.0.0.0:8000
Then access from another device:http://[machine running server ipaddress]:8000/seat_app/seat_status/

When you python manage.py runserver, the ip-address that is shown is the router IP address

Overall to address the webpage not updating issue:
what you want to see is something like: [27/Nov/2023 01:34:45] "GET /static/seat_app/output.json HTTP/1.1" 304 0
If you don't see multiple messages like that:
1. Clear Cache or Ctrl + Shift + R for force clear Cache
2. Make sure VScode is on auto save

