#!/usr/bin/expect

spawn sudo chmod 777 /dev/ttyTHS0
 expect {                 
 "password for nvidia:" { send "nvidia\r";}
 }  

interact




