# zmk_config_bbq10_pro
This page contains zmk firmware config for bbq10 keyboard pro

<p align="center">
<img width="800" alt="image" src="https://github.com/user-attachments/assets/e1e959c9-8ee6-4ef4-9ea9-cb80f5779503" />
</p>

### Use this repo to generate your own ZMK keymap for the bbq10 Pro keyboard.  
## Get started  
```Option1```   
**0. Register a github account if you don't have one.**  
**1. Fork this repo.**  

<p align="center">
<img width="600" alt="image" src="https://github.com/user-attachments/assets/e576fd0b-678e-4323-a580-c01299bf4f5f" />
</p>

**2. Open up `config/zitaotech_q10.keymap` and edit the keymap to your liking.**  
**3. After editing the keymap, choose commit changes.**  
**and then check the ```Github Actions``` section.**  

<p align="center">
<img width="600" alt="image" src="https://github.com/user-attachments/assets/4da91dd7-5766-4a3d-8edc-27eb10ade472" />
</p>

**Your new firmware file should be available for download.**  
**5. Unzip the firmware.zip file. You should see files: `zitaotech_q10-zmk.uf2` and  `settings_reset-zitaotech_q10-zmk.uf2`**  
**6. Flash the keyboard with your new firmware:`zitaotech_q10-zmk.uf2.`**  

```Option2```  
**0. Register a github account if you don't have one.**  
**1. Fork this repo.**  
**2. Access the [keymap editor web](https://nickcoutsos.github.io/keymap-editor/)**  
**3. Login with your Github Account on the web**  
**4. Choose the right repository and you can edit the keymap more intuitivly**  

<p align="center">
<img width="800"  alt="image" src="https://github.com/user-attachments/assets/f0cb0fee-396a-4f53-b64d-b29fb8883069" />
</p>

**5. After editting the keymap there will be another github action compiling and you will have the firmware.zip file**  
**6. Flash the keyboard with your new firmware.**  

**More Info about the web app please access this [github page](https://github.com/nickcoutsos/keymap-editor)**  

## Build your own driver
You can change the source code under the ```/config/boards/arm/zitaotech_q10/custom_driver```

For example if you want to change the scroll direction when capslock is activated:
Go to `a320_scroll_steps()` in [a320.c](config/boards/arm/zitaotech_q10/custom_driver/a320.c) and change ```scroll_remainder_y -= dy``` to ```scroll_remainder_y += dy``` for vertical scrolling, or ```scroll_remainder_x += dx``` to ```scroll_remainder_x -= dx``` for horizontal scrolling.
Each swipe scrolls in one direction only: after `SCROLL_AXIS_LOCK_COUNTS` sensor counts it locks to the axis it moved more along, until the finger stops or lifts.

The scroll speed is set by `CONFIG_A320_TRACKPAD_SCROLL_INTERVAL` in `config/zitaotech_q10.conf`: the trackpad movement (sensor counts) per wheel step, the bigger this value, the slower the trackpad scrolls.
