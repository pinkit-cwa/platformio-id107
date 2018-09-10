# platformio-id107
nrf51822 ID107 smartwatch arduino heartrate BPM

----------------------------------------------
this is work in progress, that piggy backs on previous work by many people, mainly : @goran-mahovlic, @rogerclarkmelbourne, @curtpw, @Gordon, @micooke and in this case @arglurgl

I used code from https://github.com/dingari/ota-dfu-python.git in order to upload firmware with a bluetooth 4.0 dongle on a raspberry pi code is included in the repository


to compile : platformio run
to upload : ./genhex  (uses stlink and has to be executed twice for some reason ....)





some backgroundinfo can be found here : 
* https://github.com/najnesnaj/smartband 
* https://github.com/najnesnaj/ota-dfu-smartband





------------------------------------------------------------
there's a wiki on howto configure platformio and other stuff
------------------------------------------------------------


the original firmware uses a legacy FOTA (firmware over the air) procedure


by executing the ./uploadblue script

the watch will be put in DFU mode, and you program will be uploaded over the air!

might be neccessary to execute the script twice ....


gen_dat generates application.dat




