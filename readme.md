This file structure uses the select.h file to decide which file to run
For testing purposes, comment all macros there and only enable one for that sensor
Make sure to always match pin number
Bmt is giving trouble, might be bme? or vice versa

https://randomnerdtutorials.com/solved-could-not-find-a-valid-bme280-sensor/

Make sure you only have one macro enabled at a time.
Each macro represents a file name!

To implement firebase, i'll recommend creating a seperate module and importing that in main and replace with the commaent under json

